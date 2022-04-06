import sys

from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

import tf_transformations


class StaticFramePublisher(Node):
   """
   Publishes an identity transform from the `rover::base_link_gpu_lidar`
   frame to the `lidar_link` frame.

   The transform is only published once at startup.
   """

   def __init__(self):
      super().__init__('static_tf_broadcaster')

      self._tf_publisher = StaticTransformBroadcaster(self)

      # Publish static transforms once at startup
      self.make_transforms()

   def make_transforms(self):
      static_transformStamped = TransformStamped()
      static_transformStamped.header.stamp = self.get_clock().now().to_msg()
      static_transformStamped.header.frame_id = 'lidar_link'
      static_transformStamped.child_frame_id = 'rover/base_link/gpu_lidar'
      static_transformStamped.transform.translation.x = 0.0
      static_transformStamped.transform.translation.y = 0.0
      static_transformStamped.transform.translation.z = 0.0
      quat = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)
      static_transformStamped.transform.rotation.x = quat[0]
      static_transformStamped.transform.rotation.y = quat[1]
      static_transformStamped.transform.rotation.z = quat[2]
      static_transformStamped.transform.rotation.w = quat[3]

      self._tf_publisher.sendTransform(static_transformStamped)


def main():
   logger = rclpy.logging.get_logger('logger')

   # pass parameters and initialize node
   rclpy.init()
   node = StaticFramePublisher()
   try:
      rclpy.spin(node)
   except KeyboardInterrupt:
      pass

   rclpy.shutdown()
