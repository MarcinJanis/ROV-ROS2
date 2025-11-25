import sys 

import rclpy
from rclpy.node import Node

from stonefish_ros2.msg import DVL, ThrusterState
from sensor_msgs.msg import Range, Image, CompressedImage, NavSatFix, Imu, LaserScan
from nav_msgs.msg import Odometry
from ds4_driver_msgs.msg import Status

import message_filters
from message_filters import ApproximateTimeSynchronizer, Subscriber

import numpy as np
import cv2
from cv_bridge import CvBridge


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
        self.sub_FLS_image = Subscriber(self, Image, '/bluerov2/fls/image')
        # self.sub_multibeam = Subscriber(self, LaserScan, '/bluerov2/multibeam')

        self.bridge = CvBridge()

        self.init = False
        self.t0_odometry_f = 0

        # self.VISU_FLAG = True

        # Aproximate Time Synchronizer - used to synchronize msgs into package
        self.ats = ApproximateTimeSynchronizer(
                [self.sub_odometry,  self.sub_FLS_image], #, self.sub_GPS, self.sub_FLS_image], # Subs list
                QUEUE_SIZE,                   # Queue size
                SLOP_TIME,                    # Time tolerance
                allow_headerless=False        # Discard headless msgs
            )
        
        self.ats.registerCallback(self.sync_callback)

        # Data containers
        self.POSITION = None # np.array, dtype float64 shape (6,)
        self.VELOCITY = None # np.array, dtype float64 shape (6,)
        self.FLS = None # np.aray (image)
        self.TIME_STAMP = 0.0

        self.get_logger().info('Subscriptions created:\n- GPS\n- Odometry\n- FLS')


    def sync_callback(self, odometry_msg: Odometry, FLS_msg: Image): #: Odometry, gps_msg: NavSatFix, fls_compress_msg: CompressedImage):
        
        # First call
        if not self.init:
            self.t0_odometry_f = odometry_msg.header.stamp.sec + odometry_msg.header.stamp.nanosec * 1e-9
            self.init = True

        # --- Odometry --- 
        self.TIME_STAMP = odometry_msg.header.stamp.sec + odometry_msg.header.stamp.nanosec * 1e-9 - self.t0_odometry_f
        self.POSITION = np.array([  odometry_msg.pose.pose.position.x,
                                        odometry_msg.pose.pose.position.y,
                                        odometry_msg.pose.pose.position.z,
                                        odometry_msg.pose.pose.orientation.x,
                                        odometry_msg.pose.pose.orientation.y,
                                        odometry_msg.pose.pose.orientation.z,
                                        odometry_msg.pose.pose.orientation.w
                                    ], dtype=np.float64)
        
        self.VELOCITY = np.array([   odometry_msg.twist.twist.linear.x,
                                    odometry_msg.twist.twist.linear.y,
                                    odometry_msg.twist.twist.linear.z,
                                    odometry_msg.twist.twist.angular.x,
                                    odometry_msg.twist.twist.angular.y,
                                    odometry_msg.twist.twist.angular.z
                                ], dtype=np.float64)
        

        # --- FLS ----
        try:
            fls_img_cv = self.bridge.imgmsg_to_cv2(FLS_msg, desired_encoding='bgr8')
            fls_img_cv = cv2.normalize(fls_img_cv, None,  alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            self.FLS = fls_img_cv
        except Exception as e:
            self.get_logger().error(f'[Error] Image conversion error:\n{e}')


# --- Control Node ---


class StonefishPublisher(Node):
    def __init__(self):
        super().__init__('stonefish_publisher')
        publisher_buff = 5 
        self.publisher = self.create_publisher(Status, 'status', publisher_buff)

    def send_cmd(self, cmd_shift: list, cmd_rotate: list): 
        msg = Status()
        
        shift_x, shift_y, shift_z = cmd_shift
        rotate_x, rotate_y, rotate_z = cmd_rotate

        # Axis mapping
        msg.axis_left_x = float(shift_y) # left/right
        msg.axis_left_y = float(shift_x) # forward/backward
        
        msg.axis_right_x = float(rotate_z) # yaw (z-axis rot.)
        msg.axis_right_y = float(shift_z) # up/down

        # Pitch
        if rotate_y > 0:
            msg.axis_r2 = float(rotate_y)
            msg.axis_l2 = 0.0
        elif rotate_y < 0:
            msg.axis_r2 = 0.0
            msg.axis_l2 = float(-rotate_y)
        else: 
            msg.axis_r2 = 0.0
            msg.axis_l2 = 0.0

        # Roll 
        if rotate_x > 0:
            msg.button_dpad_right = int(rotate_x)
            msg.button_dpad_left = 0
        elif rotate_x < 0: 
            msg.button_dpad_right = 0
            msg.button_dpad_left = int(-rotate_x)
        else: 
            msg.button_dpad_right = 0
            msg.button_dpad_left = 0

        self.publisher.publish(msg)
        # print(f'[Event] Message sent: Linear={cmd_shift}, Angular={cmd_rotate}')