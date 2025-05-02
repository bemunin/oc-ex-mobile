import math

import rclpy
from geometry_msgs.msg import Twist
from oc_interfaces.srv import ImageToText
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState, Joy
from std_msgs.msg import String

from .joypad import LogitechF710


class JoyControlNode(Node):
    def __init__(self):
        super().__init__("joy_control")
        # Ros communication protocol
        self._pub_vel = self.create_publisher(Twist, "cmd_vel", 10)
        self._sub_joy = self.create_subscription(Joy, "joy", self._joy_callback, 10)
        self._pub_camera = self.create_publisher(String, "isaac_active_camera", 10)
        self._pub_lift_bar_js = self.create_publisher(JointState, "lift_bar_js", 10)
        self._sub_camera = self.create_subscription(
            Image, "/front_stereo_camera/left/image_raw", self._camera_callback, 10
        )
        self.vlm_cli = self.create_client(ImageToText, "image_to_text_service")

        # Variables
        self._joypad = LogitechF710()
        self._max_linear_speed_mps = 2.2  # 2.2 m/s (according to the robot's spec)
        self._max_angular_speed_rps = 1.0  # 1.0 rad/s
        self._linear_update_step = 0.25  # 0.25 m/s
        self._angular_update_step = 0.1  # 0.25 rad/s
        self._cctvs = [
            "/World/CCTV/cctv_1",
            "/World/CCTV/cctv_2",
        ]
        self._robot_cams = [
            "/World/iw_hub_ROS/camera_mount/transporter_camera_third_person",
            "/World/iw_hub_ROS/chassis/front_hawk/left/camera_left",
            "/World/iw_hub_ROS/chassis/front_hawk/right/camera_right",
        ]

        # State
        self._prev_joy_state = None
        self._system_state = {
            "active_view": "",  # robot | cctv
            "cctv_cam_idx": 0,
            "robot": {
                "lift_bar": False,
                "linear_speed": 0.5,
                "angular_speed": 0.5,
                "robot_cam_idx": 0,
                "left_cam_image": None,
            },
        }

    def _joy_callback(self, joy_msg: Joy):
        joypad = self._joypad
        joypad.joy_state = self._joypad.map(joy_msg)
        sys_state = self._system_state
        robot_state = self._system_state["robot"]

        self._update_robot_speed(robot_state, joypad)
        self._update_active_cam(sys_state, joypad)
        self._action_control()

    def _camera_callback(self, msg: Image):
        self._system_state["robot"]["left_cam_image"] = msg

    def _update_robot_speed(self, robot_state, joypad):
        # Adjust Linear
        if joypad.is_press("button_left"):
            current_speed_mps = robot_state["linear_speed"]
            inc_speed = current_speed_mps + self._linear_update_step
            robot_state["linear_speed"] = min(inc_speed, self._max_linear_speed_mps)
            self.get_logger().info(f"Increase speed: {robot_state['linear_speed']}")
        elif joypad.is_press("button_left_back"):
            current_speed_mps = robot_state["linear_speed"]
            dec_speed = current_speed_mps - self._linear_update_step
            robot_state["linear_speed"] = max(dec_speed, 0.0)
            self.get_logger().info(f"Decrease speed: {robot_state['linear_speed']}")

        # Adjust Angular
        if joypad.is_press("button_right"):
            current_speed_rps = robot_state["angular_speed"]
            inc_speed = current_speed_rps + self._angular_update_step
            robot_state["angular_speed"] = min(inc_speed, self._max_angular_speed_rps)
            self.get_logger().info(
                f"Increase angular speed: {robot_state['angular_speed']}"
            )
        elif joypad.is_press("button_right_back"):
            current_speed_rps = robot_state["angular_speed"]
            dec_speed = current_speed_rps - self._angular_update_step
            robot_state["angular_speed"] = max(dec_speed, 0.0)
            self.get_logger().info(
                f"Decrease angular speed: {robot_state['angular_speed']}"
            )

    def _update_active_cam(self, sys_state, joypad):
        msg = String()

        if joypad.is_press("button_x"):
            sys_state["active_view"] = "cctv"
            curr_idx = sys_state["cctv_cam_idx"]
            next_idx = (curr_idx + 1) % len(self._cctvs)
            sys_state["cctv_cam_idx"] = next_idx
            msg.data = self._cctvs[next_idx]
            self._pub_camera.publish(msg)
        elif joypad.is_press("button_b"):
            sys_state["active_view"] = "robot"
            robot_state = sys_state["robot"]
            curr_idx = robot_state["robot_cam_idx"]
            next_idx = (curr_idx + 1) % len(self._robot_cams)
            robot_state["robot_cam_idx"] = next_idx
            msg.data = self._robot_cams[next_idx]
            self._pub_camera.publish(msg)

    def _action_control(self):
        joypad = self._joypad
        robot_state = self._system_state["robot"]

        # movement control
        twist_msg = Twist()
        if abs(joypad.value("analog_left_y")):
            direction = joypad.value("analog_left_y")
            twist_msg.linear.x = direction * robot_state["linear_speed"]
        if abs(joypad.value("analog_right_x")):
            direction = math.copysign(1, joypad.value("analog_right_x"))
            twist_msg.angular.z = direction * robot_state["angular_speed"]

        self._pub_vel.publish(twist_msg)

        # Lift bar control
        if joypad.is_press("button_y"):
            robot_state["lift_bar"] = not robot_state["lift_bar"]
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ["lift_joint"]
            msg.position = [0.04] if robot_state["lift_bar"] else [0.0]
            self._pub_lift_bar_js.publish(msg)

        # Send image to VLM service
        if joypad.is_press("button_a"):
            image_msg = robot_state["left_cam_image"]
            if image_msg is None:
                return

            if not self.vlm_cli.wait_for_service(timeout_sec=0.1):
                self.get_logger().error(
                    "Unable to connect to VLM service, check your VLM service node!"
                )
                return

            request = ImageToText.Request()
            request.image = image_msg

            # Call the service asynchronously
            future = self.vlm_cli.call_async(request)

            # Add a callback to handle the result when the future is complete
            future.add_done_callback(self._vlm_response)

    def _vlm_response(self, future):
        try:
            response = future.result()
            if response is not None:
                checkers = [
                    "yellow caution sign",
                    "yellow caution cone",
                    "yellow traffic cone",
                    "caution sign",
                    "caution cone",
                    "yellow sign",
                    "yellow cone",
                ]
                text = response.text
                self.get_logger().info(f"VLM: {text}")
                if any(checker in text for checker in checkers):
                    self.get_logger().warning(
                        "\033[93mVLM: Caution sign detected!\033[0m"
                    )

            else:
                self.get_logger().error("VLM service returned no result.")
        except Exception as e:
            self.get_logger().error(f"Failed to call service: {e}")


def main(args=None):
    rclpy.init(args=args)
    joy_control = JoyControlNode()
    rclpy.spin(joy_control)
    joy_control.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
