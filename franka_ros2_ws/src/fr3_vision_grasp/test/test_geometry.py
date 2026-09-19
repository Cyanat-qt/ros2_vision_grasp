"""Model agreement and malformed-command regression checks, without a ROS graph."""
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest
import xacro
from ament_index_python.packages import get_package_share_directory
from fr3_vision_grasp.core import ARM, HOME, intrinsics, unproject, validate_trajectory, sample_trajectory
from fr3_vision_grasp.simulation import Simulation


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle)*skew + (1-np.cos(angle))*(skew @ skew)


def urdf_tcp(root, positions):
    frames = {'base': np.eye(4)}
    pending = list(root.findall('joint'))
    while pending:
        progressed = False
        for joint in pending[:]:
            parent = joint.find('parent').get('link')
            if parent not in frames:
                continue
            t = np.eye(4)
            origin = joint.find('origin')
            if origin is not None:
                t[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
                r, p, y = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')
                t[:3, :3] = rotation([0, 0, 1], y) @ rotation([0, 1, 0], p) @ rotation([1, 0, 0], r)
            if joint.get('type') in ('revolute', 'continuous'):
                axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                t[:3, :3] = t[:3, :3] @ rotation(axis, positions.get(joint.get('name'), 0.))
            frames[joint.find('child').get('link')] = frames[parent] @ t
            pending.remove(joint)
            progressed = True
        assert progressed, 'Disconnected URDF tree'
    return frames['fr3_hand_tcp']


def test_mujoco_urdf_tcp_agreement():
    share = Path(get_package_share_directory('fr3_vision_grasp'))
    description = Path(get_package_share_directory('franka_description'))
    root = ET.fromstring(xacro.process_file(str(description / 'robots/fr3/fr3.urdf.xacro'),
        mappings={'hand': 'true', 'tcp_xyz': '0 0 0.1034', 'with_sc': 'false'}).toxml())
    sim = Simulation(share / 'scene/scene_grasp.xml')
    rng = np.random.default_rng(42)
    for q in [HOME, *rng.uniform(sim.limits[:, 0], sim.limits[:, 1], (15, 7))]:
        sim.data.qpos[sim.qadr] = q
        mujoco.mj_forward(sim.model, sim.data)
        expected = urdf_tcp(root, dict(zip(ARM, q)))
        actual = sim.data.site('gripper')
        np.testing.assert_allclose(actual.xpos, expected[:3, 3], atol=2e-5)
        np.testing.assert_allclose(actual.xmat.reshape(3, 3), expected[:3, :3], atol=2e-5)


def point(q=None, t=1.):
    return SimpleNamespace(positions=list(HOME if q is None else q), velocities=[], accelerations=[],
        effort=[], time_from_start=SimpleNamespace(sec=int(t), nanosec=round((t-int(t))*1e9)))


def test_trajectory_validation_and_reordering():
    limits = np.column_stack((np.full(7, -4), np.full(7, 4)))
    order = validate_trajectory(ARM[::-1], [point(HOME[::-1])], limits)
    np.testing.assert_allclose(np.array(HOME[::-1])[order], HOME)
    bad = [([], [point()]), (ARM, []), (ARM, [point(t=1), point(t=1)]),
           (ARM, [point([np.nan]*7)]), (ARM, [point([10]*7)]),
           (ARM[:-1]+[ARM[0]], [point()])]
    for names, points in bad:
        with pytest.raises(ValueError):
            validate_trajectory(names, points, limits)


def test_metric_depth_and_trajectory_endpoints():
    k = intrinsics(640, 480, 50)
    np.testing.assert_allclose(unproject(k[0, 2], k[1, 2], 1.2, k), [0, 0, 1.2])
    assert unproject(420, 340, 1.2, k)[0] > 0
    for z in [np.nan, np.inf, 0, -1]:
        with pytest.raises(ValueError):
            unproject(1, 1, z, k)
    a, b = np.zeros(7), np.ones(7)
    np.testing.assert_allclose(sample_trajectory([0, 2], [a, b], [a, a], 1), b/2)
    np.testing.assert_allclose(sample_trajectory([0, 2], [a, b], [a, a], 3), b)
