#!/usr/bin/env python3

import json
import os

import actionlib
import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def load_trajectory(json_path, joint_names, time_scale):
    with open(json_path, "r", encoding="utf-8") as handle:
        data = json.load(handle)

    if "trajectory" not in data:
        raise ValueError("Missing 'trajectory' array in {}".format(json_path))

    trajectory = JointTrajectory()
    trajectory.joint_names = joint_names

    for entry in data["trajectory"]:
        point = JointTrajectoryPoint()
        point.positions = entry.get("positions", [])
        point.velocities = entry.get("velocities", [])
        if "accelerations" in entry:
            point.accelerations = entry["accelerations"]
        point.time_from_start = rospy.Duration(
            float(entry.get("time_from_start", 0.0)) * time_scale
        )
        trajectory.points.append(point)

    return trajectory


def wait_for_action(name, timeout):
    client = actionlib.SimpleActionClient(name, FollowJointTrajectoryAction)
    if not client.wait_for_server(rospy.Duration(timeout)):
        rospy.logerr("Timed out waiting for action server: %s", name)
        return None
    return client


def validate_file(path_label, path_value):
    if not path_value or not os.path.isfile(path_value):
        rospy.logerr("%s path is invalid: %s", path_label, path_value)
        return False
    return True


def main():
    rospy.init_node("bimanual_trajectory_player")

    left_json = rospy.get_param("~left_json", "")
    right_json = rospy.get_param("~right_json", "")
    left_action = rospy.get_param(
        "~left_action", "/left_arm_controller/follow_joint_trajectory"
    )
    right_action = rospy.get_param(
        "~right_action", "/right_arm_controller/follow_joint_trajectory"
    )
    left_joint_names = rospy.get_param("~left_joint_names", [])
    right_joint_names = rospy.get_param("~right_joint_names", [])
    start_delay = float(rospy.get_param("~start_delay", 1.0))
    time_scale = float(rospy.get_param("~time_scale", 1.0))
    action_timeout = float(rospy.get_param("~action_timeout", 10.0))
    wait_for_result = bool(rospy.get_param("~wait_for_result", True))

    if time_scale <= 0.0:
        rospy.logerr("~time_scale must be > 0.0")
        return

    if not left_joint_names or not right_joint_names:
        rospy.logerr("~left_joint_names and ~right_joint_names must be set")
        return

    if not validate_file("Left JSON", left_json):
        return
    if not validate_file("Right JSON", right_json):
        return

    left_client = wait_for_action(left_action, action_timeout)
    right_client = wait_for_action(right_action, action_timeout)
    if left_client is None or right_client is None:
        return

    try:
        left_traj = load_trajectory(left_json, left_joint_names, time_scale)
        right_traj = load_trajectory(right_json, right_joint_names, time_scale)
    except (ValueError, json.JSONDecodeError) as exc:
        rospy.logerr("Failed to load trajectories: %s", exc)
        return

    start_time = rospy.Time.now() + rospy.Duration(start_delay)
    left_traj.header.stamp = start_time
    right_traj.header.stamp = start_time

    left_goal = FollowJointTrajectoryGoal(trajectory=left_traj)
    right_goal = FollowJointTrajectoryGoal(trajectory=right_traj)

    left_client.send_goal(left_goal)
    right_client.send_goal(right_goal)

    if wait_for_result:
        left_client.wait_for_result()
        right_client.wait_for_result()


if __name__ == "__main__":
    main()
