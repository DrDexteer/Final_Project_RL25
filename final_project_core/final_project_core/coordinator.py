#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped, Pose
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity
from nav2_msgs.srv import ClearEntireCostmap


def quat_from_yaw(yaw: float):
    """Quaternion for roll=pitch=0, yaw=yaw."""
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class Coordinator(Node):
    """
    Flow:
      1) wait /final_project_core/interaction_request = True
      2) send Nav2 goal to Fra2mo
      3) on SUCCEEDED -> dwell
      4) publish /final_project_core/interaction_start = True (trigger iiwa observe)
      5) optionally wait /final_project_core/iiwa_observe_done = True
      6) wait for fresh /aruco_single/pose => SUCCESS
      7) set ready_to_press=True (publish /final_project_core/ready_to_press = True)
      8) ONLY when user publishes manual press request, trigger iiwa press:
         - user publishes /final_project_core/manual_press_request = True
         - coordinator forwards to /final_project_core/press_button_request (iiwa_gesture)
         - optionally waits /final_project_core/iiwa_press_done
      9) when press done -> open door via /world/<world>/set_pose

     10) AFTER door opened, user can trigger enter-room Nav2 goal:
         - user publishes /final_project_core/enter_room_request = True
         - coordinator sends a second Nav2 goal (enter.* params)
    """

    def __init__(self):
        super().__init__("coordinator")

        # -------------------------
        # Parameters
        # -------------------------
        self.declare_parameter("status_topic", "/final_project_core/status")
        self.declare_parameter("aruco_pose_topic", "/aruco_single/pose")
        self.declare_parameter("nav2_action_name", "/navigate_to_pose")

        self.declare_parameter("interaction.map_frame", "map")
        self.declare_parameter("interaction.x", 0.0)
        self.declare_parameter("interaction.y", 0.0)
        self.declare_parameter("interaction.yaw", 0.0)

        self.declare_parameter("interaction.dwell_sec", 1.0)
        self.declare_parameter("interaction.aruco_timeout_sec", 1.0)
        self.declare_parameter("interaction.max_wait_sec", 20.0)
        self.declare_parameter("interaction.cooldown_sec", 2.0)

        self.declare_parameter("interaction.require_request", True)
        self.declare_parameter("interaction.request_topic", "/final_project_core/interaction_request")

        self.declare_parameter("interaction.trigger_topic", "/final_project_core/interaction_start")

        self.declare_parameter("interaction.wait_iiwa_done", True)
        self.declare_parameter("interaction.iiwa_done_topic", "/final_project_core/iiwa_observe_done")
        self.declare_parameter("interaction.iiwa_done_timeout_sec", 8.0)

        # forwarded to iiwa_gesture
        self.declare_parameter("interaction.press_topic", "/final_project_core/press_button_request")
        self.declare_parameter("interaction.wait_iiwa_press_done", True)
        self.declare_parameter("interaction.iiwa_press_done_topic", "/final_project_core/iiwa_press_done")
        self.declare_parameter("interaction.iiwa_press_done_timeout_sec", 30.0)

        # manual press gating (you publish here)
        self.declare_parameter("interaction.manual_press_topic", "/final_project_core/manual_press_request")
        self.declare_parameter("interaction.ready_topic", "/final_project_core/ready_to_press")
        self.declare_parameter("interaction.ready_valid_sec", 30.0)

        # door (open on press done)
        self.declare_parameter("door.enable", True)
        self.declare_parameter("door.world", "final_project_world")
        self.declare_parameter("door.entity_name", "door_closed")
        self.declare_parameter("door.x", 3.55)
        self.declare_parameter("door.y", 0.03)
        self.declare_parameter("door.yaw", 1.5708)
        self.declare_parameter("door.z_open", 1.6)

        # enter room (manual trigger after door opened)
        self.declare_parameter("enter.enable", True)
        self.declare_parameter("enter.require_door_opened", True)
        self.declare_parameter("enter.request_topic", "/final_project_core/enter_room_request")

        self.declare_parameter("enter.map_frame", "map")
        self.declare_parameter("enter.x", 8.20)
        self.declare_parameter("enter.y", 0.03)
        self.declare_parameter("enter.yaw", 1.5708)

        self.declare_parameter("enter.final_x", 6.23)
        self.declare_parameter("enter.final_y", -2.66)
        self.declare_parameter("enter.final_yaw", 1.5708)
        self.declare_parameter("enter.use_final", True)

        self.declare_parameter("enter.max_retries", 3)
        self.declare_parameter("enter.retry_delay_sec", 1.0)
        self.declare_parameter("enter.clear_costmaps_on_retry", True)
        self.declare_parameter("enter.clear_local_srv", "/local_costmap/clear_entirely_local_costmap")
        self.declare_parameter("enter.clear_global_srv", "/global_costmap/clear_entirely_global_costmap")



        # -------------------------
        # Read params
        # -------------------------
        self.status_topic = self.get_parameter("status_topic").value
        self.aruco_pose_topic = self.get_parameter("aruco_pose_topic").value
        self.nav2_action_name = self.get_parameter("nav2_action_name").value

        self.map_frame = self.get_parameter("interaction.map_frame").value
        self.goal_x = float(self.get_parameter("interaction.x").value)
        self.goal_y = float(self.get_parameter("interaction.y").value)
        self.goal_yaw = float(self.get_parameter("interaction.yaw").value)

        self.dwell_sec = float(self.get_parameter("interaction.dwell_sec").value)
        self.aruco_timeout_sec = float(self.get_parameter("interaction.aruco_timeout_sec").value)
        self.max_wait_sec = float(self.get_parameter("interaction.max_wait_sec").value)
        self.cooldown_sec = float(self.get_parameter("interaction.cooldown_sec").value)

        self.require_request = bool(self.get_parameter("interaction.require_request").value)
        self.request_topic = self.get_parameter("interaction.request_topic").value
        self.trigger_topic = self.get_parameter("interaction.trigger_topic").value

        self.wait_iiwa_done = bool(self.get_parameter("interaction.wait_iiwa_done").value)
        self.iiwa_done_topic = self.get_parameter("interaction.iiwa_done_topic").value
        self.iiwa_done_timeout_sec = float(self.get_parameter("interaction.iiwa_done_timeout_sec").value)

        self.press_topic = self.get_parameter("interaction.press_topic").value
        self.wait_iiwa_press_done = bool(self.get_parameter("interaction.wait_iiwa_press_done").value)
        self.iiwa_press_done_topic = self.get_parameter("interaction.iiwa_press_done_topic").value
        self.iiwa_press_done_timeout_sec = float(self.get_parameter("interaction.iiwa_press_done_timeout_sec").value)

        self.manual_press_topic = self.get_parameter("interaction.manual_press_topic").value
        self.ready_topic = self.get_parameter("interaction.ready_topic").value
        self.ready_valid_sec = float(self.get_parameter("interaction.ready_valid_sec").value)

        self.door_enable = bool(self.get_parameter("door.enable").value)
        self.door_world = str(self.get_parameter("door.world").value)
        self.door_entity_name = str(self.get_parameter("door.entity_name").value)
        self.door_x = float(self.get_parameter("door.x").value)
        self.door_y = float(self.get_parameter("door.y").value)
        self.door_yaw = float(self.get_parameter("door.yaw").value)
        self.door_z_open = float(self.get_parameter("door.z_open").value)

        self.enter_enable = bool(self.get_parameter("enter.enable").value)
        self.enter_require_door_opened = bool(self.get_parameter("enter.require_door_opened").value)
        self.enter_request_topic = str(self.get_parameter("enter.request_topic").value)

        self.enter_map_frame = str(self.get_parameter("enter.map_frame").value)
        self.enter_x = float(self.get_parameter("enter.x").value)
        self.enter_y = float(self.get_parameter("enter.y").value)
        self.enter_yaw = float(self.get_parameter("enter.yaw").value)

        self.enter_final_x = float(self.get_parameter("enter.final_x").value)
        self.enter_final_y = float(self.get_parameter("enter.final_y").value)
        self.enter_final_yaw = float(self.get_parameter("enter.final_yaw").value)
        self.enter_use_final = bool(self.get_parameter("enter.use_final").value)


        self.enter_max_retries = int(self.get_parameter("enter.max_retries").value)
        self.enter_retry_delay_sec = float(self.get_parameter("enter.retry_delay_sec").value)
        self.enter_clear_costmaps_on_retry = bool(self.get_parameter("enter.clear_costmaps_on_retry").value)
        self.enter_clear_local_srv = str(self.get_parameter("enter.clear_local_srv").value)
        self.enter_clear_global_srv = str(self.get_parameter("enter.clear_global_srv").value)

        self.clear_local_cli = self.create_client(ClearEntireCostmap, self.enter_clear_local_srv)
        self.clear_global_cli = self.create_client(ClearEntireCostmap, self.enter_clear_global_srv)

        self.enter_retry_count = 0


        # -------------------------
        # Pub/Sub (publishers first to avoid callback before pub exists)
        # -------------------------
        self.pub_status = self.create_publisher(String, self.status_topic, 10)
        self.pub_trigger = self.create_publisher(Bool, self.trigger_topic, 10)
        self.pub_press = self.create_publisher(Bool, self.press_topic, 10)
        self.pub_ready = self.create_publisher(Bool, self.ready_topic, 10)

        self.sub_aruco = self.create_subscription(PoseStamped, self.aruco_pose_topic, self._on_aruco, 10)

        self.sub_req = None
        if self.require_request:
            self.sub_req = self.create_subscription(Bool, self.request_topic, self._on_request, 10)

        self.sub_iiwa_done = None
        if self.wait_iiwa_done:
            self.sub_iiwa_done = self.create_subscription(Bool, self.iiwa_done_topic, self._on_iiwa_done, 10)

        self.sub_press_done = None
        if self.wait_iiwa_press_done:
            self.sub_press_done = self.create_subscription(Bool, self.iiwa_press_done_topic, self._on_press_done, 10)

        self.sub_manual_press = self.create_subscription(Bool, self.manual_press_topic, self._on_manual_press, 10)

        # ENTER ROOM trigger
        self.sub_enter = self.create_subscription(Bool, self.enter_request_topic, self._on_enter_request, 10)

        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, self.nav2_action_name)

        # Door service client
        self.door_opened = False
        self.set_pose_srv_name = f"/world/{self.door_world}/set_pose"
        self.set_pose_cli = self.create_client(SetEntityPose, self.set_pose_srv_name)

        # -------------------------
        # State
        # -------------------------
        self.state = "IDLE"
        self.last_cycle_end = -1e9

        self.dwell_start = 0.0
        self.trigger_time = 0.0

        self.last_aruco_time = -1e9
        self.iiwa_done_time = -1e9

        self.aruco_success_time = -1e9

        self.press_trigger_time = -1e9
        self.press_done_time = -1e9
        self.press_pulse_end = -1e9  # internal 200ms pulse

        self.ready_to_press = False
        self.ready_time = -1e9

        self.timer = self.create_timer(0.1, self._tick)

        self.enter_queue = []

        self._enter_retry_timer = None
        self.enter_current_wp = None  # (x,y,yaw) corrente per retry per-waypoint



        # init outputs
        self.pub_ready.publish(Bool(data=False))
        self.pub_trigger.publish(Bool(data=False))
        self.pub_press.publish(Bool(data=False))

        if self.require_request:
            self._status(f"Ready. Publish Bool True on {self.request_topic} to start.")
        else:
            self._start_cycle()

    # -------------------------
    # Helpers
    # -------------------------
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _status(self, msg: str):
        self.get_logger().info(msg)
        self.pub_status.publish(String(data=msg))

    def _set_ready(self, val: bool):
        self.ready_to_press = bool(val)
        if val:
            self.ready_time = self._now()
        self.pub_ready.publish(Bool(data=bool(val)))

    def _press_pulse(self):
        now = self._now()
        self.pub_press.publish(Bool(data=True))
        self.press_pulse_end = now + 0.2

    def _open_door(self):
        if not self.door_enable:
            return
        if self.door_opened:
            return

        if not self.set_pose_cli.wait_for_service(timeout_sec=1.0):
            self._status(f"Door open failed: service not available: {self.set_pose_srv_name}")
            return

        req = SetEntityPose.Request()
        req.entity = Entity(name=self.door_entity_name, type=Entity.MODEL)
        req.pose = Pose()
        req.pose.position.x = self.door_x
        req.pose.position.y = self.door_y
        req.pose.position.z = self.door_z_open

        q = quat_from_yaw(self.door_yaw)
        req.pose.orientation.x = float(q[0])
        req.pose.orientation.y = float(q[1])
        req.pose.orientation.z = float(q[2])
        req.pose.orientation.w = float(q[3])

        fut = self.set_pose_cli.call_async(req)

        def _done_cb(f):
            try:
                resp = f.result()
                if resp and resp.success:
                    self.door_opened = True
                    self._status("✅ Door opened (set_pose success).")
                else:
                    self._status("⛔ Door open failed (set_pose returned false).")
            except Exception as e:
                self._status(f"⛔ Door open exception: {e}")

        fut.add_done_callback(_done_cb)

    def _clear_costmaps(self):
        if not self.enter_clear_costmaps_on_retry:
            return
        req = ClearEntireCostmap.Request()

        if self.clear_local_cli.wait_for_service(timeout_sec=0.2):
            self.clear_local_cli.call_async(req)
        else:
            self._status(f"⚠️ clear_local service not available: {self.enter_clear_local_srv}")

        if self.clear_global_cli.wait_for_service(timeout_sec=0.2):
            self.clear_global_cli.call_async(req)
        else:
            self._status(f"⚠️ clear_global service not available: {self.enter_clear_global_srv}")

    def _schedule_enter_retry(self):
        # one-shot timer (rclpy timer è periodico, quindi lo cancelliamo dentro callback)
        if self._enter_retry_timer is not None:
            try:
                self._enter_retry_timer.cancel()
            except Exception:
                pass
            self._enter_retry_timer = None

        def _cb():
            if self._enter_retry_timer is not None:
                self._enter_retry_timer.cancel()
                self._enter_retry_timer = None
            # riprova lo stesso waypoint (enter_queue[0] non è stato poppato)
            self._send_next_enter_goal()

        self._enter_retry_timer = self.create_timer(self.enter_retry_delay_sec, _cb)


    # -------------------------
    # ENTER ROOM Nav2
    # -------------------------
    def _send_next_enter_goal(self):
        if not self.enter_queue:
            self._status("✅ Enter plan completed.")
            self.state = "IDLE"
            self.last_cycle_end = self._now()
            self.enter_current_wp = None
            self.enter_retry_count = 0
            return

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self._status(f"Nav2 action server NOT available: {self.nav2_action_name}")
            self.enter_queue = []
            self.state = "IDLE"
            self.last_cycle_end = self._now()
            self.enter_current_wp = None
            self.enter_retry_count = 0
            return

        wp = self.enter_queue[0]  # peek
        if wp != self.enter_current_wp:
            # nuovo waypoint -> reset retry
            self.enter_current_wp = wp
            self.enter_retry_count = 0

        x, y, yaw = wp
        self._status(f"➡️ Enter waypoint -> x={x:.3f}, y={y:.3f}, yaw={yaw:.3f} (frame={self.enter_map_frame})")

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.enter_map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)

        q = quat_from_yaw(float(yaw))
        goal.pose.pose.orientation.x = float(q[0])
        goal.pose.pose.orientation.y = float(q[1])
        goal.pose.pose.orientation.z = float(q[2])
        goal.pose.pose.orientation.w = float(q[3])

        self.state = "WAIT_ENTER_NAV"
        fut = self.nav_client.send_goal_async(goal)
        fut.add_done_callback(self._on_enter_nav_goal_response)


    def _on_enter_nav_goal_response(self, fut):
        gh = fut.result()
        if not gh or not gh.accepted:
            self._status("⛔ Enter waypoint rejected. Stopping plan.")
            self.enter_queue = []
            self.state = "IDLE"
            self.last_cycle_end = self._now()
            return

        res_fut = gh.get_result_async()
        res_fut.add_done_callback(self._on_enter_nav_result)


    def _on_enter_nav_result(self, fut):
        res = fut.result()
        if not res or res.status != GoalStatus.STATUS_SUCCEEDED:
            st = res.status if res else "None"
            self._status(f"⚠️ Enter waypoint NOT succeeded (status={st}).")

            if self.enter_retry_count < self.enter_max_retries:
                self.enter_retry_count += 1
                self._status(f"🔁 Retry {self.enter_retry_count}/{self.enter_max_retries}: clearing costmaps and retrying in {self.enter_retry_delay_sec:.1f}s...")
                self._clear_costmaps()
                self.state = "IDLE"  # torna “libero” per rimandare goal
                self._schedule_enter_retry()
                return

            self._status("⛔ Retries exhausted. Stopping enter plan.")
            self.enter_queue = []
            self.state = "IDLE"
            self.last_cycle_end = self._now()
            self.enter_current_wp = None
            self.enter_retry_count = 0
            return

        # SUCCESS: ora pop e vai al prossimo
        if self.enter_queue:
            self.enter_queue.pop(0)

        self._status("✅ Enter waypoint reached.")
        # passa al prossimo waypoint: qui forziamo reset del “current”
        self.enter_current_wp = None
        self.enter_retry_count = 0
        self._send_next_enter_goal()



    # -------------------------
    # Callbacks
    # -------------------------
    def _on_request(self, msg: Bool):
        if msg.data and self.state == "IDLE":
            self._start_cycle()

    def _on_aruco(self, _: PoseStamped):
        self.last_aruco_time = self._now()

    def _on_iiwa_done(self, msg: Bool):
        if msg.data:
            self.iiwa_done_time = self._now()

    def _on_press_done(self, msg: Bool):
        if msg.data:
            self.press_done_time = self._now()

    def _on_manual_press(self, msg: Bool):
        if not msg.data:
            return

        now = self._now()

        if (not self.ready_to_press) or ((now - self.ready_time) > self.ready_valid_sec):
            self._status("⛔ Manual press ignored: NOT ready (no recent ArUco success).")
            return

        if self.state != "IDLE":
            self._status(f"⛔ Manual press ignored: coordinator busy (state={self.state}).")
            return

        self._status("➡️ Manual press accepted: triggering iiwa press.")
        self._set_ready(False)  # consume readiness

        self.press_trigger_time = now
        self.press_done_time = -1e9

        self._press_pulse()

        if self.wait_iiwa_press_done:
            self.state = "WAIT_PRESS_DONE"
        else:
            self.state = "IDLE"
            self.last_cycle_end = now

    def _on_enter_request(self, msg: Bool):
        if not msg.data:
            return

        if not self.enter_enable:
            self._status("⛔ Enter room ignored: enter.enable is False.")
            return

        if self.state != "IDLE":
            self._status(f"⛔ Enter room ignored: coordinator busy (state={self.state}).")
            return

        if self.enter_require_door_opened and not self.door_opened:
            self._status("⛔ Enter room ignored: door not opened yet.")
            return

        # build 2-step plan: entry -> final
        self.enter_queue = [(self.enter_x, self.enter_y, self.enter_yaw)]
        if self.enter_use_final:
            self.enter_queue.append((self.enter_final_x, self.enter_final_y, self.enter_final_yaw))

        self._status(f"➡️ Enter plan accepted: {len(self.enter_queue)} waypoint(s).")
        self._send_next_enter_goal()


    # -------------------------
    # Main logic
    # -------------------------
    def _start_cycle(self):
        now = self._now()
        if (now - self.last_cycle_end) < self.cooldown_sec:
            self._status("Cooldown active, ignoring start.")
            return

        self._set_ready(False)

        self._status(
            f"Nav2 goal -> x={self.goal_x:.3f}, y={self.goal_y:.3f}, yaw={self.goal_yaw:.3f} rad (frame={self.map_frame})"
        )

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self._status(f"Nav2 action server NOT available: {self.nav2_action_name}")
            self.state = "IDLE"
            self.last_cycle_end = now
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        goal.pose.pose.position.x = float(self.goal_x)
        goal.pose.pose.position.y = float(self.goal_y)

        q = quat_from_yaw(float(self.goal_yaw))
        goal.pose.pose.orientation.x = float(q[0])
        goal.pose.pose.orientation.y = float(q[1])
        goal.pose.pose.orientation.z = float(q[2])
        goal.pose.pose.orientation.w = float(q[3])

        self.state = "WAIT_NAV"
        fut = self.nav_client.send_goal_async(goal)
        fut.add_done_callback(self._on_nav_goal_response)

    def _on_nav_goal_response(self, fut):
        gh = fut.result()
        if not gh or not gh.accepted:
            self._status("Nav2 goal rejected.")
            self.state = "IDLE"
            self.last_cycle_end = self._now()
            return

        self._status("Nav2 goal accepted. Waiting result...")
        res_fut = gh.get_result_async()
        res_fut.add_done_callback(self._on_nav_result)

    def _on_nav_result(self, fut):
        res = fut.result()
        if not res or res.status != GoalStatus.STATUS_SUCCEEDED:
            st = res.status if res else "None"
            self._status(f"Nav2 finished NOT succeeded (status={st}).")
            self.state = "IDLE"
            self.last_cycle_end = self._now()
            return

        self._status("Fra2mo reached interaction pose. Dwell...")
        self.state = "DWELL"
        self.dwell_start = self._now()

    def _tick(self):
        now = self._now()

        # auto end press pulse
        if self.press_pulse_end > 0 and now >= self.press_pulse_end:
            self.pub_press.publish(Bool(data=False))
            self.press_pulse_end = -1e9

        if self.state == "DWELL":
            if (now - self.dwell_start) >= self.dwell_sec:
                self._status("Triggering iiwa to observe pose...")
                self.pub_trigger.publish(Bool(data=True))
                self.trigger_time = now
                self.iiwa_done_time = -1e9
                self.state = "WAIT_IIWA_DONE" if self.wait_iiwa_done else "WAIT_ARUCO"

        elif self.state == "WAIT_IIWA_DONE":
            if self.iiwa_done_time >= self.trigger_time:
                self._status("iiwa_observe_done received. Waiting ArUco...")
                self.state = "WAIT_ARUCO"
            elif (now - self.trigger_time) > self.iiwa_done_timeout_sec:
                self._status("Timeout waiting iiwa_observe_done. Proceeding to ArUco check anyway...")
                self.state = "WAIT_ARUCO"

        elif self.state == "WAIT_ARUCO":
            if self.last_aruco_time >= self.trigger_time and (now - self.last_aruco_time) <= self.aruco_timeout_sec:
                self._status("✅ ArUco detected. Fra2mo in position confirmed.")
                self.pub_trigger.publish(Bool(data=False))
                self.aruco_success_time = now

                if self.wait_iiwa_done and self.iiwa_done_time < self.trigger_time:
                    self._status("ArUco OK, waiting iiwa_observe_done before enabling manual press...")
                    self.state = "WAIT_READY_AFTER_ARUCO"
                    return

                self._status(
                    f"✅ Ready to press. Publish Bool True on {self.manual_press_topic} (valid {self.ready_valid_sec:.0f}s)."
                )
                self._set_ready(True)
                self.state = "IDLE"
                self.last_cycle_end = now
                return

            if (now - self.trigger_time) > self.max_wait_sec:
                self._status("⛔ ArUco not detected in time. Check visibility / pose / lighting / marker size.")
                self.pub_trigger.publish(Bool(data=False))
                self.state = "IDLE"
                self.last_cycle_end = now

        elif self.state == "WAIT_READY_AFTER_ARUCO":
            if self.iiwa_done_time >= self.trigger_time:
                self._status(
                    f"✅ Ready to press. Publish Bool True on {self.manual_press_topic} (valid {self.ready_valid_sec:.0f}s)."
                )
                self._set_ready(True)
                self.state = "IDLE"
                self.last_cycle_end = now
                return

            if (now - self.aruco_success_time) > self.iiwa_done_timeout_sec:
                self._status("⚠️ Still no iiwa_observe_done, enabling press anyway.")
                self._set_ready(True)
                self.state = "IDLE"
                self.last_cycle_end = now
                return

        elif self.state == "WAIT_PRESS_DONE":
            if self.press_done_time >= self.press_trigger_time:
                self._status("✅ iiwa_press_done received. Opening door...")
                self._open_door()
                self.state = "IDLE"
                self.last_cycle_end = now
                return

            if (now - self.press_trigger_time) > self.iiwa_press_done_timeout_sec:
                self._status("⛔ Timeout waiting iiwa_press_done. Ending anyway.")
                self.state = "IDLE"
                self.last_cycle_end = now
                return

        # WAIT_ENTER_NAV is handled by the Nav2 callbacks (no polling needed)


def main():
    rclpy.init()
    node = Coordinator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
