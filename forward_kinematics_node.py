import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from math import cos, sin, pi

class ForwardKinematicsNode(Node):
    def __init__(self):
        super().__init__('forward_kinematics_node')
        self.L1 = 2.0
        self.L2 = 1.5
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.publisher = self.create_publisher(Point, '/end_effector_position', 10)

    def joint_state_callback(self, msg):
        try:
            theta1 = msg.position[0] + pi / 2
            theta2 = msg.position[1]
            x = self.L1 * cos(theta1) + self.L2 * cos(theta1 + theta2)
            y = self.L1 * sin(theta1) + self.L2 * sin(theta1 + theta2)
            point = Point()
            point.x = float(x)
            point.y = float(y)
            point.z = 0.0
            self.publisher.publish(point)
            self.get_logger().info(f'End-effector position: x={x:.2f}, y={y:.2f}')
        except IndexError:
            self.get_logger().error('JointState message missing position data.')

def main():
    rclpy.init()
    node = ForwardKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()