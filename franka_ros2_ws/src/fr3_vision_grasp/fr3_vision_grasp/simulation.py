"""MuJoCo state owner. Rendering and stepping are called from one thread."""
import numpy as np
import mujoco
from contextlib import nullcontext
from .core import ARM, HOME, intrinsics


class Simulation:
    def __init__(self, scene, width=640, height=480):
        self.model = mujoco.MjModel.from_xml_path(str(scene))
        self.data = mujoco.MjData(self.model)
        self.joints = [self.model.joint(n).id for n in ARM]
        self.qadr = self.model.jnt_qposadr[self.joints]
        self.dadr = self.model.jnt_dofadr[self.joints]
        self.actuators = [self.model.actuator(n).id for n in ARM]
        self.finger_joints = [self.model.joint(f'finger_joint{i}').id for i in (1, 2)]
        self.fqadr = self.model.jnt_qposadr[self.finger_joints]
        self.fdadr = self.model.jnt_dofadr[self.finger_joints]
        self.gripper = self.model.actuator('gripper').id
        self.limits = self.model.jnt_range[self.joints].copy()
        self.camera = self.model.camera('top_camera').id
        self.width, self.height = width, height
        self.model.vis.global_.offwidth = max(width, self.model.vis.global_.offwidth)
        self.model.vis.global_.offheight = max(height, self.model.vis.global_.offheight)
        self.model.vis.quality.offsamples = 0
        self.model.vis.quality.shadowsize = 1024
        self.k = intrinsics(width, height, self.model.cam_fovy[self.camera])
        self.renderer = None
        self.reset()

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self.qadr] = HOME
        self.data.qpos[self.fqadr] = 0.04
        self.data.ctrl[self.actuators] = HOME
        self.data.ctrl[self.gripper] = 0.04
        mujoco.mj_forward(self.model, self.data)

    def step(self):
        # Compensate modeled bias, but preserve contact and gripper physics.
        self.data.qfrc_applied[self.dadr] = self.data.qfrc_bias[self.dadr]
        mujoco.mj_step(self.model, self.data)

    def render(self, lock=None):
        if self.renderer is None:
            self.renderer = mujoco.Renderer(self.model, self.height, self.width)
        self.renderer.disable_depth_rendering()
        # Copy the scene under the physics lock, then render outside it. OpenGL
        # belongs to this caller thread; a slow GPU must not stall joint control.
        with lock if lock is not None else nullcontext():
            self.renderer.update_scene(self.data, camera=self.camera)
            self.frame_time = float(self.data.time)
        # Shadows/reflections are cosmetic and expensive on software OpenGL.
        self.renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
        self.renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = False
        rgb = self.renderer.render().copy()
        self.renderer.enable_depth_rendering()
        depth = self.renderer.render().copy()  # Renderer returns linear, metric depth.
        self.renderer.disable_depth_rendering()
        far = self.model.vis.map.zfar * self.model.stat.extent
        depth[(depth >= far * 0.999) | ~np.isfinite(depth) | (depth <= 0)] = np.nan
        return rgb, depth.astype(np.float32)

    def camera_pose(self):
        rotation = self.data.cam_xmat[self.camera].reshape(3, 3) @ np.diag([1., -1., -1.])
        q = np.empty(4)
        mujoco.mju_mat2Quat(q, rotation.ravel())
        return self.data.cam_xpos[self.camera].copy(), q

    def contacts(self, object_name):
        body = self.model.body(object_name).id
        fingers = {self.model.body(n).id: n for n in ('left_finger', 'right_finger')}
        sides = set()
        for contact in self.data.contact:
            a, b = self.model.geom_bodyid[[contact.geom1, contact.geom2]]
            if a == body and b in fingers:
                sides.add(fingers[b])
            if b == body and a in fingers:
                sides.add(fingers[a])
        return sorted(sides)

    def close(self):
        if self.renderer is not None:
            self.renderer.close()
