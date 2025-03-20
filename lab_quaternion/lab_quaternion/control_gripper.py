import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import time
from .gen3lite_pymoveit2 import Gen3LiteGripper

class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')
        self.gripper = Gen3LiteGripper()

        self.declare_parameter("command", "open")
        self.declare_parameter("position", 0.5)

    def control_gripper(self):
        """Executes gripper actions based on provided parameters."""
        command = self.get_parameter("command").value
        position = self.get_parameter("position").value

        if command == "open":
            self.get_logger().info("Opening gripper.")
            self.gripper.open()
        elif command == "close":
            self.get_logger().info("Closing gripper.")
            self.gripper.close()
        elif command == "move":
            if 0.0 <= position <= 0.85:
                self.get_logger().info(f"Moving gripper to position: {position}")
                self.gripper.move_to_position(position)
            else:
                self.get_logger().error(f"Invalid position value: {position}. Must be between 0.0 and 0.85.")
        else:
            self.get_logger().error(f"Unrecognized command: {command}. Available commands are 'open', 'close', or 'move'.")

        self.get_logger().info("Gripper action completed.")


def main(args=None):
    rclpy.init(args=args)
    gripper_controller = GripperController()
    gripper_controller.control_gripper()
    gripper_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
