#!/usr/bin/env python3
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Bool, String
from action_msgs.msg import GoalStatus

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class IiwaGesture(Node):
    def __init__(self):
        super().__init__("iiwa_gesture")

        # existing
        self.declare_parameter("status_topic", "/final_project_core/status")
        self.declare_parameter("trigger_topic", "/final_project_core/interaction_start")
        self.declare_parameter("done_topic", "/final_project_core/iiwa_observe_done")

        self.declare_parameter("action_name", "/iiwa/iiwa_arm_trajectory_controller/follow_joint_trajectory")
        self.declare_parameter("joint_names", ["joint_a1","joint_a2","joint_a3","joint_a4","joint_a5","joint_a6","joint_a7"])
        self.declare_parameter("observe_positions", [0.4, -0.78, 0.0, 1.7, 0.0, 0.8, 0.0])
        self.declare_parameter("observe_time_sec", 2.0)

        # press interface
        self.declare_parameter("press_topic", "/final_project_core/press_button_request")
        self.declare_parameter("press_done_topic", "/final_project_core/iiwa_press_done")

        # ✅ YOUR PRESS JOINTS HERE
        self.declare_parameter("press_positions", [-0.5, -1.35, 0.0, 0.7, 0.0, -0.95, 0.0])
        self.declare_parameter("press_time_sec", 2.0)

        # return to rest after press (default: back to observe pose)
        self.declare_parameter("rest_positions", [0.4, -0.78, 0.0, 1.7, 0.0, 0.8, 0.0])
        self.declare_parameter("rest_time_sec", 2.0)

        # read params
        self.status_topic = self.get_parameter("status_topic").value
        self.trigger_topic = self.get_parameter("trigger_topic").value
        self.done_topic = self.get_parameter("done_topic").value

        self.action_name = self.get_parameter("action_name").value
        self.joint_names: List[str] = list(self.get_parameter("joint_names").value)

        self.observe_positions: List[float] = [float(x) for x in self.get_parameter("observe_positions").value]
        self.observe_time_sec = float(self.get_parameter("observe_time_sec").value)

        self.press_topic = self.get_parameter("press_topic").value
        self.press_done_topic = self.get_parameter("press_done_topic").value
        self.press_positions: List[float] = [float(x) for x in self.get_parameter("press_positions").value]
        self.press_time_sec = float(self.get_parameter("press_time_sec").value)

        self.rest_positions: List[float] = [float(x) for x in self.get_parameter("rest_positions").value]
        self.rest_time_sec = float(self.get_parameter("rest_time_sec").value)

        # pub/sub
        self.pub_status = self.create_publisher(String, self.status_topic, 10)

        self.pub_done = self.create_publisher(Bool, self.done_topic, 10)
        self.sub_trigger = self.create_subscription(Bool, self.trigger_topic, self._on_trigger, 10)

        self.pub_press_done = self.create_publisher(Bool, self.press_done_topic, 10)
        self.sub_press = self.create_subscription(Bool, self.press_topic, self._on_press, 10)

        self.client = ActionClient(self, FollowJointTrajectory, self.action_name)

        self.busy = False
        self.current_task = "none"

        self._status(
            f"Ready. Observe trigger: {self.trigger_topic} | Press trigger: {self.press_topic} | Action: {self.action_name}"
        )

    def _status(self, msg: str):
        self.get_logger().info(msg)
        self.pub_status.publish(String(data=msg))

    def _publish_done(self, ok: bool):
        if self.current_task == "observe":
            self.pub_done.publish(Bool(data=bool(ok)))
        elif self.current_task == "press":
            self.pub_press_done.publish(Bool(data=bool(ok)))

    def _on_trigger(self, msg: Bool):
        if not msg.data or self.busy:
            return
        # observe: single waypoint
        self._send_trajectory(
            points=[(self.observe_positions, self.observe_time_sec)],
            task="observe",
        )

    def _on_press(self, msg: Bool):
        if not msg.data or self.busy:
            return

        # press -> HOLD 1s -> return to rest
        t_press = float(self.press_time_sec)          # arrivo sul pulsante
        t_hold  = t_press + 1.5                       # fermo 1 secondo (stessa posa)
        t_rest  = t_hold + float(self.rest_time_sec)  # ritorno a riposo

        self._send_trajectory(
            points=[
                (self.press_positions, t_press),
                (self.press_positions, t_hold),   # ✅ HOLD 1s
                (self.rest_positions,  t_rest),
            ],
            task="press",
        )

    def _send_trajectory(self, points: List[Tuple[List[float], float]], task: str):
        self.busy = True
        self.current_task = task

        # reset done flags
        self._publish_done(False)

        if len(self.joint_names) != 7:
            self._status("Config error: joint_names must have length 7.")
            self.busy = False
            self.current_task = "none"
            return

        for pos, _t in points:
            if len(pos) != 7:
                self._status("Config error: each positions array must have length 7.")
                self.busy = False
                self.current_task = "none"
                return

        if not self.client.wait_for_server(timeout_sec=5.0):
            self._status(f"Trajectory action server not available: {self.action_name}")
            self.busy = False
            self.current_task = "none"
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.joint_names)

        traj_points: List[JointTrajectoryPoint] = []
        for pos, t in points:
            pt = JointTrajectoryPoint()
            pt.positions = list(pos)

            sec = int(t)
            pt.time_from_start.sec = sec
            pt.time_from_start.nanosec = int((t - sec) * 1e9)

            traj_points.append(pt)

        goal.trajectory.points = traj_points

        if task == "press":
            self._status("Press requested: moving to press pose, then returning to rest pose...")
        else:
            self._status("Observe requested: moving to observe pose...")

        fut = self.client.send_goal_async(goal)
        fut.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, fut):
        gh = fut.result()
        if not gh or not gh.accepted:
            self._status(f"{self.current_task} goal rejected.")
            self._publish_done(False)
            self.busy = False
            self.current_task = "none"
            return

        self._status(f"{self.current_task} goal accepted. Waiting result...")
        res_fut = gh.get_result_async()
        res_fut.add_done_callback(self._on_result)

    def _on_result(self, fut):
        res = fut.result()
        if res and res.status == GoalStatus.STATUS_SUCCEEDED:
            self._status(f"✅ iiwa finished {self.current_task} trajectory.")
            self._publish_done(True)
        else:
            st = res.status if res else "None"
            self._status(f"⛔ iiwa {self.current_task} finished NOT succeeded (status={st}).")
            self._publish_done(False)

        self.busy = False
        self.current_task = "none"


def main():
    rclpy.init()
    node = IiwaGesture()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
