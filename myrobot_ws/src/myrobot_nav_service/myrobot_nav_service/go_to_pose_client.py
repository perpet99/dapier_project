#!/usr/bin/env python3
import argparse
import sys

import rclpy
from rclpy.node import Node

from myrobot_interfaces.srv import GoToPose


def main():
    parser = argparse.ArgumentParser(description="Call the go_to_pose service")
    parser.add_argument("x", type=float)
    parser.add_argument("y", type=float)
    parser.add_argument("yaw", type=float, nargs="?", default=0.0)
    parser.add_argument("--service-name", default="go_to_pose")
    parser.add_argument("--timeout-sec", type=float, default=130.0)
    args = parser.parse_args()

    rclpy.init()
    node = Node("go_to_pose_client")
    client = node.create_client(GoToPose, args.service_name)

    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error(f"Service '{args.service_name}' not available")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    request = GoToPose.Request()
    request.x = args.x
    request.y = args.y
    request.yaw = args.yaw

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout_sec)

    if future.result() is None:
        node.get_logger().error("Service call timed out or failed")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    result = future.result()
    print(f"success={result.success} message={result.message}")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if result.success else 1)


if __name__ == "__main__":
    main()
