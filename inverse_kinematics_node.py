import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
from math import acos, atan2, sqrt, pi


class InverseKinematicsNode(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_node')
        self.L1 = 2.0
        self.L2 = 1.5
        self.current_position = Point()

        self.subscription = self.create_subscription(
            Point,
            '/end_effector_position',
            self.end_effector_callback,
            10
        )

        self.publisher = self.create_publisher(Float64MultiArray, '/joint_angles_goal', 10)

        # Timer to check input every 1s
        self.create_timer(1.0, self.get_user_input)

    def end_effector_callback(self, msg):
        self.current_position = msg

    def get_user_input(self):
        try:
            direction = input("Enter direction (x or y): ").strip().lower()
            if direction not in ['x', 'y']:
                self.get_logger().warning("Invalid direction. Use 'x' or 'y'.")
                return

            distance = float(input("Enter distance (<= 0.5m): ").strip())
            if abs(distance) > 0.5:
                self.get_logger().warning("Distance must be ≤ 0.5 meters.")
                return

            # Compute new target position
            target = Point()
            target.x = self.current_position.x + (distance if direction == 'x' else 0)
            target.y = self.current_position.y + (distance if direction == 'y' else 0)

            if self.is_reachable(target.x, target.y):
                theta1, theta2 = self.inverse_kinematics(target.x, target.y)
                msg = Float64MultiArray()
                msg.data = [theta1, theta2]
                self.publisher.publish(msg)
                self.get_logger().info(f"Published joint angles: θ1={theta1:.2f}, θ2={theta2:.2f}")
            else:
                self.get_logger().warning("Target position is unreachable. Skipping.")

        except Exception as e:
            self.get_logger().error(f"Error reading input: {e}")

    def is_reachable(self, x, y):
        dist = sqrt(x**2 + y**2)
        return dist <= (self.L1 + self.L2) and dist >= abs(self.L1 - self.L2)

    def inverse_kinematics(self, x, y):
        # Law of cosines
        cos_theta2 = (x**2 + y**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        theta2 = acos(cos_theta2)

        # Compute theta1
        k1 = self.L1 + self.L2 * cos_theta2
        k2 = self.L2 * sqrt(1 - cos_theta2**2)
        theta1 = atan2(y, x) - atan2(k2, k1)

        return theta1, theta2


def main():
    rclpy.init()
    node = InverseKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()