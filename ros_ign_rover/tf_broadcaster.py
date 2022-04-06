from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

class FramePublisher(Node):

    def __init__(self):
        super().__init__('tf_broadcaster')

        # Declare and acquire `rover_name` parameter
        self.declare_parameter('rover_name', 'rover')
        self.rover_name = self.get_parameter(
            'rover_name').get_parameter_value().string_value

        # Initialize the transform broadcaster
        self.br = TransformBroadcaster(self)

        # Subscribe to a rover/pose topic and call handle_rover_pose
        # callback function on each message
        self.subscription = self.create_subscription(
            PoseStamped,
            f'/{self.rover_name}/pose',
            self.handle_rover_pose,
            1)
        self.subscription

    def handle_rover_pose(self, msg):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation.x = msg.pose.orientation.x
        t.transform.rotation.y = msg.pose.orientation.y
        t.transform.rotation.z = msg.pose.orientation.z
        t.transform.rotation.w = msg.pose.orientation.w

        # Send the transformation
        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
