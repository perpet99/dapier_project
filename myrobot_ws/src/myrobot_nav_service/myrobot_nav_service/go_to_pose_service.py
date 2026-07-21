#!/usr/bin/env python3
import math
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from myrobot_interfaces.srv import GoToPose


def yaw_to_quaternion(yaw: float):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class GoToPoseService(Node):
    def __init__(self):
        super().__init__("go_to_pose_service")

        self.declare_parameter("frame_id", "map")
        self.declare_parameter("action_name", "navigate_to_pose")
        self.declare_parameter("action_server_timeout_sec", 5.0)
        self.declare_parameter("navigation_timeout_sec", 120.0)

        cb_group = ReentrantCallbackGroup()
        action_name = self.get_parameter("action_name").value

        self._action_client = ActionClient(
            self, NavigateToPose, action_name, callback_group=cb_group
        )
        self._srv = self.create_service(
            GoToPose, "go_to_pose", self._on_request, callback_group=cb_group
        )
        self.get_logger().info(f"go_to_pose service ready (action: {action_name})")

    def _make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        frame_id = self.get_parameter("frame_id").value
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quaternion(float(yaw))
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def _on_request(self, request: GoToPose.Request, response: GoToPose.Response):
        server_timeout = self.get_parameter("action_server_timeout_sec").value
        if not self._action_client.wait_for_server(timeout_sec=server_timeout):
            response.success = False
            response.message = "navigate_to_pose action server not available"
            return response

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._make_pose(request.x, request.y, request.yaw)

        self.get_logger().info(
            f"Navigating to x={request.x}, y={request.y}, yaw={request.yaw}"
        )

        done_event = threading.Event()
        result_holder = {"success": False, "message": "Unknown error"}

        def goal_response_callback(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                result_holder["success"] = False
                result_holder["message"] = "Goal rejected by navigate_to_pose"
                done_event.set()
                return

            result_future = goal_handle.get_result_async()

            def result_callback(res_future):
                status = res_future.result().status
                if status == GoalStatus.STATUS_SUCCEEDED:
                    result_holder["success"] = True
                    result_holder["message"] = "Reached target pose"
                elif status == GoalStatus.STATUS_CANCELED:
                    result_holder["success"] = False
                    result_holder["message"] = "Navigation canceled"
                else:
                    result_holder["success"] = False
                    result_holder["message"] = f"Navigation failed with status {status}"
                done_event.set()

            result_future.add_done_callback(result_callback)

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(goal_response_callback)

        nav_timeout = self.get_parameter("navigation_timeout_sec").value
        if not done_event.wait(timeout=nav_timeout):
            response.success = False
            response.message = f"Timed out after {nav_timeout}s waiting for navigation result"
            return response

        response.success = result_holder["success"]
        response.message = result_holder["message"]
        return response


def main():
    rclpy.init()
    node = GoToPoseService()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
