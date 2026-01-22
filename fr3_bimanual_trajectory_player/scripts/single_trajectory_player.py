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


def main():
    rospy.init_node("single_trajectory_player")

    trajectory_json = rospy.get_param("~trajectory_json", "")
    action_name = rospy.get_param(
        "~action", "/arm_controller/follow_joint_trajectory"
    )
    joint_names = rospy.get_param("~joint_names", [])
    start_delay = float(rospy.get_param("~start_delay", 1.0))
    time_scale = float(rospy.get_param("~time_scale", 1.0))
    action_timeout = float(rospy.get_param("~action_timeout", 10.0))
    wait_for_result = bool(rospy.get_param("~wait_for_result", True))

    if time_scale <= 0.0:
        rospy.logerr("~time_scale must be > 0.0")
        return

    if not joint_names:
        rospy.logerr("~joint_names must be set")
        return

    if not trajectory_json or not os.path.isfile(trajectory_json):
        rospy.logerr("Trajectory JSON path is invalid: %s", trajectory_json)
        return

    client = actionlib.SimpleActionClient(action_name, FollowJointTrajectoryAction)
    if not client.wait_for_server(rospy.Duration(action_timeout)):
        rospy.logerr("Timed out waiting for action server: %s", action_name)
        return

    try:
        trajectory = load_trajectory(trajectory_json, joint_names, time_scale)
    except (ValueError, json.JSONDecodeError) as exc:
        rospy.logerr("Failed to load trajectory: %s", exc)
        return

    trajectory.header.stamp = rospy.Time.now() + rospy.Duration(start_delay)
    goal = FollowJointTrajectoryGoal(trajectory=trajectory)
    client.send_goal(goal)

    if wait_for_result:
        client.wait_for_result()


if __name__ == "__main__":
    main()
