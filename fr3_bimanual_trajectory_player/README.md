# FR3 Bimanual Trajectory Player

This package plays the JSON joint trajectories in `trajectory/left_arm_lift 2.json`
and `trajectory/right_arm_lift 2.json` on a bimanual Franka FR3 setup using the
`FollowJointTrajectory` action interface.

## Dependencies
- ROS Noetic
- `franka_ros` with two FR3 arms configured in `ros_control`

If you do not already have Franka support installed, clone and build
`franka_ros`, then configure your dual-arm controllers.

## Usage
1. Update the joint names and controller action topics in
   `fr3_bimanual_trajectory_player/config/bimanual_player.yaml` or via launch args.
2. Build your catkin workspace.
3. Run:

```bash
roslaunch fr3_bimanual_trajectory_player play_bimanual_trajectories.launch
```

## Parameters
- `~left_json`, `~right_json`: JSON trajectory paths.
- `~left_action`, `~right_action`: `FollowJointTrajectory` action names.
- `~left_joint_names`, `~right_joint_names`: 7 joint names per arm.
- `~start_delay`: seconds to delay start for sync (default 1.0).
- `~time_scale`: multiply `time_from_start` (default 1.0).
- `~action_timeout`: seconds to wait for action servers.
- `~wait_for_result`: wait for goals to finish if true.
