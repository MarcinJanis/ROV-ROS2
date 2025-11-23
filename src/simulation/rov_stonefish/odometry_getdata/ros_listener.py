import sys 

import rclpy
from rclpy.node import Node

from stonefish_ros2.msg import DVL, ThrusterState
from sensor_msgs.msg import Range, Image, CompressedImage, NavSatFix, Imu, LaserScan
from nav_msgs.msg import Odometry

import message_filters
from message_filters import ApproximateTimeSynchronizer, Subscriber

import numpy as np
import cv2

# Params
SLOP_TIME = 0.9 # time tolerance to put measurements in one package
QUEUE_SIZE = 10 # size of que to hold msgs of every kind

class StonefishSubscriber(Node):
    """
    Class of listener ROS2 Node to capture data.
    """

    def __init__(self):
        super().__init__('stonefish_listener')
        
        # create subscriptions
        # self.sub_GPS = Subscriber(self, NavSatFix, '/bluerov2/gps')   
        self.sub_odometry = Subscriber(self, Odometry, '/bluerov2/odometry')
        # self.sub_FLS_image = Subscriber(self, Image, '/bluerov2/fls/image')
        self.sub_multibeam = Subscriber(self, LaserScan, '/bluerov2/multibeam')


        self.init = False
        self.t0_odometry_f = 0

        # Aproximate Time Synchronizer - used to synchronize msgs into package
        self.ats = ApproximateTimeSynchronizer(
                [self.sub_odometry,  self.sub_multibeam], #, self.sub_GPS, self.sub_FLS_image], # Subs list
                QUEUE_SIZE,                   # Queue size
                SLOP_TIME,                    # Time tolerance
                allow_headerless=False        # Discard headless msgs
            )
        
        self.ats.registerCallback(self.sync_callback)
        self.get_logger().info('Subscriptions created:\n- GPS\n- Odometry\n- FLS')


    def sync_callback(self, odometry_msg: Odometry, multibeam_msg: LaserScan): #: Odometry, gps_msg: NavSatFix, fls_compress_msg: CompressedImage):
        
        # First call
        if not self.init:
            self.t0_odometry_f = odometry_msg.header.stamp.sec + odometry_msg.header.stamp.nanosec * 1e-9
            self.init = True
        # Odometry 
        t_odometry_f = odometry_msg.header.stamp.sec + odometry_msg.header.stamp.nanosec * 1e-9 - self.t0_odometry_f
        pos_odometry = np.array([  odometry_msg.pose.pose.position.x,
                                        odometry_msg.pose.pose.position.y,
                                        odometry_msg.pose.pose.position.z,
                                        odometry_msg.pose.pose.orientation.x,
                                        odometry_msg.pose.pose.orientation.y,
                                        odometry_msg.pose.pose.orientation.z,
                                        odometry_msg.pose.pose.orientation.w
                                    ], dtype=np.float64)
        
        vel_odometry = np.array([   odometry_msg.twist.twist.linear.x,
                                    odometry_msg.twist.twist.linear.y,
                                    odometry_msg.twist.twist.linear.z,
                                    odometry_msg.twist.twist.angular.x,
                                    odometry_msg.twist.twist.angular.y,
                                    odometry_msg.twist.twist.angular.z
                                ], dtype=np.float64)
        
        self.get_logger().info(f'\n--- time: {t_odometry_f:.4f} ---\n x: {pos_odometry[0]:.4f}, y: {pos_odometry[1]:.4f}, z: {pos_odometry[2]:.4f}\n Rx: {pos_odometry[3]:.4f}, Ry: {pos_odometry[4]:.4f}, Rz: {pos_odometry[5]:.4f}')
        # output_string = (
        #     f"\r[T: {t_odometry:.4f}s] "
        #     f"POS (x,y,z): {pos_odometry[0]:.4f}, {pos_odometry[1]:.4f}, {pos_odometry[2]:.4f} | "
        #     f"QUAT (x,y,z,w): {pos_odometry[3]:.4f}, {pos_odometry[4]:.4f}, {pos_odometry[5]:.4f}, {pos_odometry[6]:.4f}"
        # )
        # sys.stdout.write(output_string.ljust(150))
        # sys.stdout.flush() 


def main(args=None):
    rclpy.init(args=args)
    stonefish_subscriber = StonefishSubscriber()
    try:
        rclpy.spin(stonefish_subscriber)
    except KeyboardInterrupt:
        sys.stdout.write('\n')
        pass  

    stonefish_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()