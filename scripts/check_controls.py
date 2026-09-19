#!/usr/bin/env python3
"""Control protocol regression check. Run on an idle demo before picking."""
import json
import time
import numpy as np
import rclpy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from std_srvs.srv import Trigger
from validate_demo import Evaluator
from fr3_vision_grasp.core import ARM


def main():
    rclpy.init()
    node = Evaluator()
    report = {}
    def await_result(future):
        node.until(future.done, 30)
        return future.result()
    arm = ActionClient(node, FollowJointTrajectory, '/fr3_arm_controller/follow_joint_trajectory')
    hand = ActionClient(node, GripperCommand, '/franka_gripper/gripper_action')
    try:
        assert arm.wait_for_server(timeout_sec=20) and hand.wait_for_server(timeout_sec=20)
        node.until(lambda: 'joints' in node.latest)
        q = np.array(node.latest['joints'].position[:7])
        bad = FollowJointTrajectory.Goal()
        bad.trajectory.joint_names = ARM[:-1]
        bad.trajectory.points = [JointTrajectoryPoint(positions=q.tolist(), time_from_start=Duration(sec=1))]
        assert not await_result(arm.send_goal_async(bad)).accepted
        report['malformed_arm_goal_rejected'] = True
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        target = q.copy()
        target[0] += 0.1
        goal.trajectory.points = [JointTrajectoryPoint(positions=target.tolist(), time_from_start=Duration(sec=6))]
        active = await_result(arm.send_goal_async(goal))
        assert active.accepted
        result_future = active.get_result_async()
        concurrent = await_result(arm.send_goal_async(goal))
        assert not concurrent.accepted
        reset = node.create_client(Trigger, '/simulation/reset')
        assert reset.wait_for_service(timeout_sec=5)
        assert not await_result(reset.call_async(Trigger.Request())).success
        report['concurrent_goal_and_busy_reset_rejected'] = True
        started = time.monotonic()
        node.until(lambda: time.monotonic()-started > 1.)
        assert np.max(np.abs(np.array(node.latest['joints'].position[:7])-q)) > 0.002
        assert await_result(active.cancel_goal_async()).goals_canceling
        assert await_result(result_future).status == GoalStatus.STATUS_CANCELED
        started = time.monotonic()
        node.until(lambda: time.monotonic()-started > 0.5)
        stopped = np.array(node.latest['joints'].position[:7])
        started = time.monotonic()
        node.until(lambda: time.monotonic()-started > 1.)
        assert np.max(np.abs(np.array(node.latest['joints'].position[:7])-stopped)) < 0.005
        report['cancel_holds_actual_position'] = True
        invalid = GripperCommand.Goal()
        invalid.command.position = 0.08
        assert not await_result(hand.send_goal_async(invalid)).accepted
        for position in [0., 0.04]:
            goal = GripperCommand.Goal()
            goal.command.position, goal.command.max_effort = position, 20.
            handle = await_result(hand.send_goal_async(goal))
            assert handle.accepted
            result = await_result(handle.get_result_async())
            assert result.status == GoalStatus.STATUS_SUCCEEDED and result.result.reached_goal
            assert abs(result.result.position-position) < 0.002
        report['gripper_open_close_and_width_validation'] = True
        node.service('/simulation/reset')
        report['idle_reset'] = True
        print(json.dumps(report, indent=2))
    finally:
        arm.destroy()
        hand.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
