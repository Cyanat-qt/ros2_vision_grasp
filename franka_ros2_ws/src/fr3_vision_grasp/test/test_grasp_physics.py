"""Physical regression: a closed, stationary grasp must not creep out of the pads."""
from pathlib import Path

import mujoco
import numpy as np
from ament_index_python.packages import get_package_share_directory

from fr3_vision_grasp.simulation import Simulation


def test_contact_grasp_holds_for_30_seconds_then_releases():
    sim = Simulation(Path(get_package_share_directory('fr3_vision_grasp')) / 'scene/scene_grasp.xml')
    # Known downward TCP configurations at (0.45, 0, 0.065) and (0.45, 0, 0.165).
    # Only initialize the robot; the cube is always advanced by contact physics.
    grasp = np.array([0.00351190, 0.27323129, -0.00302859, -2.55371610,
                      0.00264070, 2.82694517, 0.78348273])
    lifted = np.array([0.00324424, -0.00578750, -0.00324805, -2.56054149,
                       -0.00003394, 2.55475409, 0.78542264])
    sim.data.qpos[sim.qadr] = grasp
    sim.data.ctrl[sim.actuators] = grasp
    mujoco.mj_forward(sim.model, sim.data)
    dt = sim.model.opt.timestep

    def advance(duration):
        for _ in range(round(duration / dt)):
            sim.step()

    advance(0.5)
    for t in np.arange(0, 1.5, dt):
        sim.data.ctrl[sim.gripper] = 0.04 * max(0, 1 - t / 1.2)
        sim.step()
    assert len(sim.contacts('red_object')) == 2
    for t in np.arange(0, 4, dt):
        fraction = min(1, t / 3.5)
        fraction = fraction**2 * (3 - 2 * fraction)
        sim.data.ctrl[sim.actuators] = grasp + fraction * (lifted - grasp)
        sim.step()
    advance(1)
    initial = sim.data.body('red_object').xpos.copy()
    assert initial[2] > 0.15
    for _ in range(300):
        advance(0.1)
        assert len(sim.contacts('red_object')) == 2, 'Stationary closed gripper dropped the cube'
        assert np.linalg.norm(sim.data.body('red_object').xpos - initial) < 0.002, 'Grasp is creeping'

    # Opening must release the object under gravity, without any simulated attachment.
    sim.data.ctrl[sim.gripper] = 0.04
    advance(2)
    assert not sim.contacts('red_object')
    assert abs(sim.data.body('red_object').xpos[2] - 0.065) < 0.001
