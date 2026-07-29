import rclpy
from rclpy.node import Node

from .gravity_model import calculate_gravity_torques


class GravityCompensationNode(Node):

    def __init__(self):
        super().__init__("gravity_compensation_node")

        self.timer = self.create_timer(
            1.0,
            self.timer_callback,
        )

        self.get_logger().info("Gravity compensation node started")

    def timer_callback(self):
        q1 = 0.0
        q2 = 0.0

        tau1, tau2 = calculate_gravity_torques(q1, q2)

        self.get_logger().info(
            f"q1={q1:.3f}, q2={q2:.3f}, "
            f"tau1={tau1:.3f} N*m, tau2={tau2:.3f} N*m"
        )


def main(args=None):
    rclpy.init(args=args)

    node = GravityCompensationNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
