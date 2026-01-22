# FR3 Trajectory Recorder

Records joint trajectories during kinesthetic teaching and saves them as JSON
and YAML (JointTrajectory-like) files.

This package does not enable gravity compensation by itself. Start your
gravity-compensation controller(s) in `franka_ros` before recording.

## Dependencies
- ROS Noetic
- `franka_ros` with `franka_control`
- `python3-yaml` (for YAML output)

## Single-arm recording
```bash
roslaunch fr3_trajectory_recorder single_arm_recorder.launch
```

You'll be prompted to type `yes` in the terminal to start recording.

Output files:
- `<output_dir>/<output_basename>.json`
- `<output_dir>/<output_basename>.yaml`

## Bimanual recording
```bash
roslaunch fr3_trajectory_recorder bimanual_recorder.launch
```

Output files:
- `<output_dir>/left_<output_basename>.json`
- `<output_dir>/right_<output_basename>.json`
- `<output_dir>/left_<output_basename>.yaml`
- `<output_dir>/right_<output_basename>.yaml`

## Useful parameters
- `~joint_states_topic`: defaults to `/joint_states`
- `~record_rate`: Hz, defaults to 100
- `~duration`: seconds, 0 means wait for Enter to stop
- `~use_joint_state_velocities`: use JointState velocities if true
- `~output_dir`, `~output_basename`

## Controller switching (optional)
If you want the recorder to switch controllers, set these parameters:
- `~controller_manager_ns`, `~start_controllers`, `~stop_controllers`
- For bimanual: `~left_controller_manager_ns`, `~right_controller_manager_ns`,
  `~left_start_controllers`, `~right_start_controllers`, etc.
