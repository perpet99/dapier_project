#!/usr/bin/env python3
import argparse
import math
import time
from typing import Dict, List

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def yaw_to_quaternion(yaw: float):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def make_pose(frame_id: str, x: float, y: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp.sec = 0
    pose.header.stamp.nanosec = 0
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0

    qx, qy, qz, qw = yaw_to_quaternion(yaw)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


def load_goals(path: str) -> Dict:
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError("Goal file must be a YAML object")
    if "goals" not in data or not isinstance(data["goals"], list) or not data["goals"]:
        raise ValueError("Goal file must include non-empty 'goals' list")
    return data


def main():
    parser = argparse.ArgumentParser(description="Run sequential Nav2 goals from YAML")
    parser.add_argument("--goals", required=True, help="Path to goal YAML")
    parser.add_argument("--use-sim-time", action="store_true", help="Use /clock from simulator")
    args = parser.parse_args()

    data = load_goals(args.goals)
    frame_id = data.get("frame_id", "map")
    initial_pose = data.get("initial_pose")
    goals: List[Dict] = data["goals"]

    rclpy.init()
    navigator = BasicNavigator()

    if args.use_sim_time:
        navigator._node.set_parameters([
            rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
        ])

    if initial_pose:
        init = make_pose(
            frame_id,
            initial_pose.get("x", 0.0),
            initial_pose.get("y", 0.0),
            initial_pose.get("yaw", 0.0),
        )
        navigator.setInitialPose(init)
        time.sleep(1.0)

    navigator.waitUntilNav2Active()

    for idx, goal in enumerate(goals, start=1):
        target = make_pose(
            frame_id,
            goal["x"],
            goal["y"],
            goal.get("yaw", 0.0),
        )

        print(f"[Goal {idx}/{len(goals)}] x={goal['x']}, y={goal['y']}, yaw={goal.get('yaw', 0.0)}")
        navigator.goToPose(target)

        while not navigator.isTaskComplete():
            navigator.getFeedback()
            time.sleep(0.2)

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"[Goal {idx}] SUCCEEDED")
        elif result == TaskResult.CANCELED:
            print(f"[Goal {idx}] CANCELED")
            break
        else:
            print(f"[Goal {idx}] FAILED")
            break

    navigator.lifecycleShutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
