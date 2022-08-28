from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan

import rclpy
from rclpy.node import Node


class LaserScanTransformFLUToFRD(Node):
    '''
    Transform LaserScan data from BODY_FLU to BODY_FRD

    Assumptions:
      - Scan data is horizontal
      - Scan data is symmetric about the x-axis
      - Equal magnitude limits: |angle_max| = |angle_min| 

    The transform is implemented by reversing the order of the
    ranges and intensities arrays
    '''

    def __init__(self):
        super().__init__('laser_scan_transform_flu_to_frd')

        # Declare and acquire `rover_name` parameter
        self.declare_parameter('rover_name', 'rover')
        self.rover_name = self.get_parameter(
            'rover_name').get_parameter_value().string_value

        # Subscribe to a /sensors/laser_scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            f'/sensors/laser_scan',
            self.handle_laser_scan,
            1)
        self.subscription

        self.publisher = self.create_publisher(
            LaserScan, 'sensors/laser_scan_frd', 10)


    def handle_laser_scan(self, msg):
        '''
        Flip the laser scan data from ROS FLU to aerospace FRD convention
        '''
        scan = LaserScan()

        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = msg.header.frame_id

        scan.angle_min = msg.angle_min
        scan.angle_max = msg.angle_max
        scan.angle_increment = msg.angle_increment
        scan.time_increment = msg.time_increment
        scan.scan_time = msg.scan_time
        scan.range_min = msg.range_min
        scan.range_max = msg.range_max
        scan.ranges = msg.ranges
        scan.intensities = msg.intensities

        # TODO: assumes range angle min/max is symmetric about the x-axis
        scan.ranges.reverse()
        scan.intensities.reverse()

        # Publish the transformed scan
        self.publisher.publish(scan)

def main():
    rclpy.init()
    node = LaserScanTransformFLUToFRD()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
