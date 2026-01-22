#!/usr/bin/env python3

import json
import os
import threading

import rospy
import yaml
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from sensor_msgs.msg import JointState


def ensure_dir(path):
    os.makedirs(path, exist_ok=True)


def write_json(path, points):
    data = {"trajectory": []}
    for point in points:
        data["trajectory"].append(
            {
                "time_from_start": point["time_from_start"],
                "positions": point["positions"],
                "velocities": point["velocities"],
            }
        )
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(data, handle, indent=2)


def write_yaml(path, joint_names, points):
    yaml_data = {
        "joint_names": joint_names,
        "points": points,
    }
    with open(path, "w", encoding="utf-8") as handle:
        yaml.safe_dump(yaml_data, handle, sort_keys=False)


def switch_controllers(ns, start, stop, strictness, timeout):
    if not start and not stop:
        return True

    service_name = "/controller_manager/switch_controller"
    if ns:
        service_name = "/" + ns.strip("/") + service_name

    try:
        rospy.wait_for_service(service_name, timeout=timeout)
    except rospy.ROSException:
        rospy.logerr("Switch controller service not available: %s", service_name)
        return False

    try:
        proxy = rospy.ServiceProxy(service_name, SwitchController)
        req = SwitchControllerRequest()
        req.start_controllers = start
        req.stop_controllers = stop
        req.strictness = int(strictness)
        req.start_asap = True
        req.timeout = rospy.Duration(timeout)
        resp = proxy(req)
        if not resp.ok:
            rospy.logerr("Switch controller call failed for %s", service_name)
        return resp.ok
    except rospy.ServiceException as exc:
        rospy.logerr("Switch controller call error: %s", exc)
        return False


class BimanualRecorder:
    def __init__(
        self,
        left_joint_names,
        right_joint_names,
        topic,
        rate_hz,
        use_joint_velocities,
    ):
        self.left_joint_names = left_joint_names
        self.right_joint_names = right_joint_names
        self.rate_hz = rate_hz
        self.use_joint_velocities = use_joint_velocities
        self.lock = threading.Lock()
        self.latest_msg = None
        self.left_indices = None
        self.right_indices = None
        self.left_max_index = None
        self.right_max_index = None
        self.left_points = []
        self.right_points = []
        self.prev_left_positions = None
        self.prev_right_positions = None
        self.prev_time = None
        self.start_time = None
        self.stop_event = threading.Event()
        self.sub = rospy.Subscriber(topic, JointState, self._callback, queue_size=10)

    def _callback(self, msg):
        with self.lock:
            self.latest_msg = msg

    def _build_indices(self, msg, joint_names):
        name_to_index = {name: idx for idx, name in enumerate(msg.name)}
        try:
            return [name_to_index[name] for name in joint_names]
        except KeyError as exc:
            rospy.logerr("Missing joint in JointState: %s", exc)
            return None

    def start(self):
        self.start_time = rospy.Time.now()
        self.stop_event.clear()
        thread = threading.Thread(target=self._record_loop)
        thread.daemon = True
        thread.start()
        return thread

    def stop(self):
        self.stop_event.set()

    def _record_loop(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown() and not self.stop_event.is_set():
            with self.lock:
                msg = self.latest_msg

            if msg is None:
                rate.sleep()
                continue

            if self.left_indices is None:
                self.left_indices = self._build_indices(msg, self.left_joint_names)
                self.right_indices = self._build_indices(msg, self.right_joint_names)
                if self.left_indices is None or self.right_indices is None:
                    self.stop_event.set()
                    return
                self.left_max_index = max(self.left_indices) if self.left_indices else None
                self.right_max_index = (
                    max(self.right_indices) if self.right_indices else None
                )

            now = rospy.Time.now()
            left_positions = [msg.position[idx] for idx in self.left_indices]
            right_positions = [msg.position[idx] for idx in self.right_indices]

            left_velocities = None
            right_velocities = None
            if (
                self.use_joint_velocities
                and msg.velocity
                and self.left_max_index is not None
                and self.right_max_index is not None
                and len(msg.velocity) > max(self.left_max_index, self.right_max_index)
            ):
                left_velocities = [msg.velocity[idx] for idx in self.left_indices]
                right_velocities = [msg.velocity[idx] for idx in self.right_indices]
            else:
                left_velocities = [0.0] * len(left_positions)
                right_velocities = [0.0] * len(right_positions)
                if (
                    self.prev_left_positions is not None
                    and self.prev_right_positions is not None
                    and self.prev_time is not None
                ):
                    dt = (now - self.prev_time).to_sec()
                    if dt > 0.0:
                        left_velocities = [
                            (pos - prev) / dt
                            for pos, prev in zip(
                                left_positions, self.prev_left_positions
                            )
                        ]
                        right_velocities = [
                            (pos - prev) / dt
                            for pos, prev in zip(
                                right_positions, self.prev_right_positions
                            )
                        ]

            time_from_start = (now - self.start_time).to_sec()
            self.left_points.append(
                {
                    "time_from_start": time_from_start,
                    "positions": left_positions,
                    "velocities": left_velocities,
                }
            )
            self.right_points.append(
                {
                    "time_from_start": time_from_start,
                    "positions": right_positions,
                    "velocities": right_velocities,
                }
            )

            self.prev_left_positions = left_positions
            self.prev_right_positions = right_positions
            self.prev_time = now
            rate.sleep()


def main():
    rospy.init_node("bimanual_trajectory_recorder")

    left_joint_names = rospy.get_param(
        "~left_joint_names",
        [
            "panda1_joint1",
            "panda1_joint2",
            "panda1_joint3",
            "panda1_joint4",
            "panda1_joint5",
            "panda1_joint6",
            "panda1_joint7",
        ],
    )
    right_joint_names = rospy.get_param(
        "~right_joint_names",
        [
            "panda2_joint1",
            "panda2_joint2",
            "panda2_joint3",
            "panda2_joint4",
            "panda2_joint5",
            "panda2_joint6",
            "panda2_joint7",
        ],
    )
    joint_states_topic = rospy.get_param("~joint_states_topic", "/joint_states")
    output_dir = os.path.expanduser(
        rospy.get_param("~output_dir", "~/.ros/trajectories")
    )
    output_basename = rospy.get_param("~output_basename", "bimanual")
    record_rate = float(rospy.get_param("~record_rate", 100.0))
    use_joint_velocities = bool(rospy.get_param("~use_joint_state_velocities", True))
    duration = float(rospy.get_param("~duration", 0.0))

    left_controller_manager_ns = rospy.get_param("~left_controller_manager_ns", "panda1")
    right_controller_manager_ns = rospy.get_param(
        "~right_controller_manager_ns", "panda2"
    )
    left_start_controllers = rospy.get_param("~left_start_controllers", [])
    left_stop_controllers = rospy.get_param("~left_stop_controllers", [])
    right_start_controllers = rospy.get_param("~right_start_controllers", [])
    right_stop_controllers = rospy.get_param("~right_stop_controllers", [])
    strictness = int(rospy.get_param("~strictness", 2))
    switch_timeout = float(rospy.get_param("~switch_timeout", 5.0))

    ensure_dir(output_dir)

    if not left_joint_names or not right_joint_names:
        rospy.logerr("~left_joint_names and ~right_joint_names must be set")
        return

    if record_rate <= 0.0:
        rospy.logerr("~record_rate must be > 0")
        return

    if left_start_controllers or left_stop_controllers:
        if not switch_controllers(
            left_controller_manager_ns,
            left_start_controllers,
            left_stop_controllers,
            strictness,
            switch_timeout,
        ):
            return

    if right_start_controllers or right_stop_controllers:
        if not switch_controllers(
            right_controller_manager_ns,
            right_start_controllers,
            right_stop_controllers,
            strictness,
            switch_timeout,
        ):
            return

    answer = input("Type 'yes' to start recording: ").strip().lower()
    if answer != "yes":
        rospy.loginfo("Recording canceled.")
        return

    recorder = BimanualRecorder(
        left_joint_names,
        right_joint_names,
        joint_states_topic,
        record_rate,
        use_joint_velocities,
    )
    thread = recorder.start()

    if duration > 0.0:
        rospy.sleep(duration)
    else:
        input("Recording... press Enter to stop.")

    recorder.stop()
    thread.join()

    left_json = os.path.join(output_dir, "left_{}.json".format(output_basename))
    right_json = os.path.join(output_dir, "right_{}.json".format(output_basename))
    left_yaml = os.path.join(output_dir, "left_{}.yaml".format(output_basename))
    right_yaml = os.path.join(output_dir, "right_{}.yaml".format(output_basename))

    write_json(left_json, recorder.left_points)
    write_json(right_json, recorder.right_points)
    write_yaml(left_yaml, left_joint_names, recorder.left_points)
    write_yaml(right_yaml, right_joint_names, recorder.right_points)

    rospy.loginfo("Saved left JSON trajectory to %s", left_json)
    rospy.loginfo("Saved right JSON trajectory to %s", right_json)
    rospy.loginfo("Saved left YAML trajectory to %s", left_yaml)
    rospy.loginfo("Saved right YAML trajectory to %s", right_yaml)


if __name__ == "__main__":
    main()
