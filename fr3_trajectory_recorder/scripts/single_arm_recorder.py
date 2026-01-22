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


def write_json(path, joint_names, points):
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


class SingleArmRecorder:
    def __init__(self, joint_names, topic, rate_hz, use_joint_velocities):
        self.joint_names = joint_names
        self.rate_hz = rate_hz
        self.use_joint_velocities = use_joint_velocities
        self.lock = threading.Lock()
        self.latest_msg = None
        self.indices = None
        self.max_index = None
        self.points = []
        self.prev_positions = None
        self.prev_time = None
        self.start_time = None
        self.stop_event = threading.Event()
        self.sub = rospy.Subscriber(topic, JointState, self._callback, queue_size=10)

    def _callback(self, msg):
        with self.lock:
            self.latest_msg = msg

    def _build_indices(self, msg):
        name_to_index = {name: idx for idx, name in enumerate(msg.name)}
        try:
            indices = [name_to_index[name] for name in self.joint_names]
            self.max_index = max(indices) if indices else None
            return indices
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

            if self.indices is None:
                self.indices = self._build_indices(msg)
                if self.indices is None:
                    self.stop_event.set()
                    return

            now = rospy.Time.now()
            positions = [msg.position[idx] for idx in self.indices]

            velocities = None
            if (
                self.use_joint_velocities
                and msg.velocity
                and self.max_index is not None
                and len(msg.velocity) > self.max_index
            ):
                velocities = [msg.velocity[idx] for idx in self.indices]
            else:
                velocities = [0.0] * len(positions)
                if self.prev_positions is not None and self.prev_time is not None:
                    dt = (now - self.prev_time).to_sec()
                    if dt > 0.0:
                        velocities = [
                            (pos - prev) / dt
                            for pos, prev in zip(positions, self.prev_positions)
                        ]

            point = {
                "time_from_start": (now - self.start_time).to_sec(),
                "positions": positions,
                "velocities": velocities,
            }
            self.points.append(point)

            self.prev_positions = positions
            self.prev_time = now
            rate.sleep()


def main():
    rospy.init_node("single_arm_trajectory_recorder")

    joint_names = rospy.get_param(
        "~joint_names",
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
    joint_states_topic = rospy.get_param("~joint_states_topic", "/joint_states")
    output_dir = os.path.expanduser(
        rospy.get_param("~output_dir", "~/.ros/trajectories")
    )
    output_basename = rospy.get_param("~output_basename", "single_arm")
    record_rate = float(rospy.get_param("~record_rate", 100.0))
    use_joint_velocities = bool(rospy.get_param("~use_joint_state_velocities", True))
    duration = float(rospy.get_param("~duration", 0.0))

    controller_manager_ns = rospy.get_param("~controller_manager_ns", "")
    start_controllers = rospy.get_param("~start_controllers", [])
    stop_controllers = rospy.get_param("~stop_controllers", [])
    strictness = int(rospy.get_param("~strictness", 2))
    switch_timeout = float(rospy.get_param("~switch_timeout", 5.0))

    ensure_dir(output_dir)

    if not joint_names:
        rospy.logerr("~joint_names must be set")
        return

    if record_rate <= 0.0:
        rospy.logerr("~record_rate must be > 0")
        return

    if start_controllers or stop_controllers:
        if not switch_controllers(
            controller_manager_ns,
            start_controllers,
            stop_controllers,
            strictness,
            switch_timeout,
        ):
            return

    answer = input("Type 'yes' to start recording: ").strip().lower()
    if answer != "yes":
        rospy.loginfo("Recording canceled.")
        return

    recorder = SingleArmRecorder(
        joint_names, joint_states_topic, record_rate, use_joint_velocities
    )
    thread = recorder.start()

    if duration > 0.0:
        rospy.sleep(duration)
    else:
        input("Recording... press Enter to stop.")

    recorder.stop()
    thread.join()

    json_path = os.path.join(output_dir, "{}.json".format(output_basename))
    yaml_path = os.path.join(output_dir, "{}.yaml".format(output_basename))
    write_json(json_path, joint_names, recorder.points)
    write_yaml(yaml_path, joint_names, recorder.points)

    rospy.loginfo("Saved JSON trajectory to %s", json_path)
    rospy.loginfo("Saved YAML trajectory to %s", yaml_path)


if __name__ == "__main__":
    main()
