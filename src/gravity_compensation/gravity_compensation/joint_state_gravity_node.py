import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from .gravity_model import calculate_gravity_torques


class JointStateGravityNode(Node):

    def __init__(self):
        super().__init__("joint_state_gravity_node")
        self.declare_parameter("m1", 1.0)
        self.declare_parameter("m2", 1.0)
        self.declare_parameter("l1", 1.0)
        self.declare_parameter("lc1", 0.5)
        self.declare_parameter("lc2", 0.5)
        self.declare_parameter("g", 9.81)
        self.subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
            )       
        self.torque_publisher = self.create_publisher(
        Float64MultiArray,
        "/gravity_compensation/torques",
        10,
        )
       

        self.get_logger().info(
            "Waiting for /joint_states"
        )

    def joint_state_callback(self, msg):
        if "joint1" not in msg.name or "joint2" not in msg.name:
            self.get_logger().warning(
                "Message does not contain joint1 and joint2"
            )
            return

        joint1_index = msg.name.index("joint1")
        joint2_index = msg.name.index("joint2")

        if (
            joint1_index >= len(msg.position)
            or joint2_index >= len(msg.position)
        ):
            self.get_logger().warning(
                "Joint position data is incomplete"
            )
            return

        q1 = msg.position[joint1_index]
        q2 = msg.position[joint2_index]

        m1 = self.get_parameter("m1").value
        m2 = self.get_parameter("m2").value
        l1 = self.get_parameter("l1").value
        lc1 = self.get_parameter("lc1").value
        lc2 = self.get_parameter("lc2").value
        g = self.get_parameter("g").value

        tau1, tau2 = calculate_gravity_torques(
            q1,
            q2,
            m1=m1,
            m2=m2,
            l1=l1,
            lc1=lc1,
            lc2=lc2,
            g=g,
        )
        torque_message = Float64MultiArray()
        torque_message.data = [tau1, tau2]
        self.torque_publisher.publish(torque_message)
        self.get_logger().info(
            f"q1={q1:.3f}, q2={q2:.3f}, "
            f"tau1={tau1:.3f} N*m, tau2={tau2:.3f} N*m"
        )


def main(args=None):
    rclpy.init(args=args)

    node = JointStateGravityNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
