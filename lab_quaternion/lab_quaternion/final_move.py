#!/usr/bin/env python3

import os
import rclpy
import time
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Pose, Twist
from .gen3lite_pymoveit2 import Gen3LiteArm
from .go_to_position import MoveToPosition
from .control_gripper import GripperController


class TurtleBot3Controller(Node):
    """Controls linear and angular movement with speed and acceleration adjustments."""

    MAX_SPEED = 0.24
    ACCELERATION_STEP = 0.02

    def __init__(self):
        super().__init__('turtlebot3_controller')

        self.declare_parameter('linear_vel', 0.2)
        self.declare_parameter('angular_vel', 0.0)
        self.declare_parameter('duration', 5.0)

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        time.sleep(1)

    def move(self):
        """Executes motion with gradual acceleration."""

        target_linear_velocity = self.get_parameter('linear_vel').value
        angular_velocity = self.get_parameter('angular_vel').value
        duration = self.get_parameter('duration').value

        if target_linear_velocity > self.MAX_SPEED:
            self.get_logger().warn(f"Speed {target_linear_velocity} m/s exceeds maximum. Limiting to {self.MAX_SPEED} m/s.")
            target_linear_velocity = self.MAX_SPEED

        if duration <= 0:
            self.get_logger().warn("Duration must be positive. Using 1 second.")
            duration = 1.0

        self.get_logger().info(f"Executing movement: Linear {target_linear_velocity} m/s, Angular {angular_velocity} rad/s, Duration {duration} s")

        msg = Twist()
        current_speed = 0.0
        start_time = time.time()

        while time.time() - start_time < duration:
            if current_speed < target_linear_velocity:
                current_speed += self.ACCELERATION_STEP
                current_speed = min(current_speed, target_linear_velocity)
            elif current_speed > target_linear_velocity:
                current_speed -= self.ACCELERATION_STEP
                current_speed = max(current_speed, target_linear_velocity)

            msg.linear.x = current_speed
            msg.angular.z = angular_velocity
            self.publisher_.publish(msg)

            time.sleep(0.1)

        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info("Robot has stopped.")


class PickAndPlaceSequence:
    """Manages the entire pick-and-place process using MoveToPosition and GripperController."""

    def __init__(self):
        self.arm = Gen3LiteArm()

    def move_to_pose(self, x, y, z, qx, qy, qz, qw, description=""):
        mover = MoveToPosition()

        mover.set_parameters([
            Parameter("x", Parameter.Type.DOUBLE, x),
            Parameter("y", Parameter.Type.DOUBLE, y),
            Parameter("z", Parameter.Type.DOUBLE, z),
            Parameter("qx", Parameter.Type.DOUBLE, qx),
            Parameter("qy", Parameter.Type.DOUBLE, qy),
            Parameter("qz", Parameter.Type.DOUBLE, qz),
            Parameter("qw", Parameter.Type.DOUBLE, qw),
        ])

        mover.execute_movement()
        mover.destroy_node()
        print(f"Position '{description}' reached.")

    def control_gripper(self, command, position=0.5):
        gripper_node = GripperController()

        gripper_node.set_parameters([
            Parameter("command", Parameter.Type.STRING, command),
            Parameter("position", Parameter.Type.DOUBLE, position),
        ])

        gripper_node.control_gripper()
        gripper_node.destroy_node()

    def execute_sequence(self):
        positions = [
            ("close",),
            (0.390, 0.071, 0.120, 1.000, 0.002, 0.011, -0.012, "Pick Position"),
            ("open",),
            (0.390, 0.071, 0.39, 1.000, 0.002, 0.011, -0.012, "Lift Position"),
            (-0.314, 0.047, 0.163, -0.542, 0.832, 0.071, -0.095, "Pre-Put Position 1"),
            (-0.314, 0.041, -0.036, -0.548, 0.835, -0.006, -0.044, "Pre-Put Position 2"),
            (-0.267, -0.007, -0.232, -0.552, 0.829, -0.084, 0.007, "Put Position 1"),
            ("close",),
        ]

        self.arm.go_vertical()

        for action in positions:
            if isinstance(action, tuple):
                if len(action) == 1:
                    self.control_gripper(action[0])
                else:
                    self.move_to_pose(*action)


def execute_movement_sequence(movement_list):
    for move in movement_list:
        controller = TurtleBot3Controller()
        controller.set_parameters([
            Parameter("linear_vel", Parameter.Type.DOUBLE, move["linear_vel"]),
            Parameter("angular_vel", Parameter.Type.DOUBLE, move["angular_vel"]),
            Parameter("duration", Parameter.Type.DOUBLE, move["duration"]),
        ])
        controller.move()
        controller.destroy_node()
        time.sleep(1)
    time.sleep(3)


def main():
    rclpy.init()

    pick_and_place = PickAndPlaceSequence()
    pick_and_place.execute_sequence()

    rclpy.shutdown()
    print("Task execution completed.")


if __name__ == "__main__":
    main()
