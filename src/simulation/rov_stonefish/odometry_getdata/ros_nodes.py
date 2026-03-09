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

from geometry_msgs.msg import WrenchStamped

from threading import Event

# Params
SLOP_TIME = 0.2 # time tolerance to put measurements in one package
QUEUE_SIZE = 10 # size of que to hold msgs of every kind

class StonefishSubscriber(Node):
    """
    Class of listener ROS2 Node to capture data.
    """

    def __init__(self):
        super().__init__('stonefish_listener')
        
        self.new_data_event = Event()
        # create subscriptions
        self.sub_odometry = Subscriber(self, Odometry, '/bluerov2/odometry')
        # self.sub_fls_image = Subscriber(self, Image, '/bluerov2/fls/image')
        self.sub_fls_image = Subscriber(self, Image, '/bluerov2/fls/display')
        self.sub_imu = Subscriber(self, Imu, '/bluerov2/imu')
        self.sub_dvl = Subscriber(self, DVL, '/bluerov2/dvl_sim')
        self.sub_altitude = Subscriber(self, Range, '/bluerov2/altitude')
        self.bridge = CvBridge()

        self.init = False
        self.t0_odometry_f = 0

        # Aproximate Time Synchronizer - used to synchronize msgs into package
        self.ats = ApproximateTimeSynchronizer(
            [self.sub_odometry, self.sub_fls_image, self.sub_dvl, self.sub_altitude, self.sub_imu], 
            QUEUE_SIZE,                   
            SLOP_TIME,                    
            allow_headerless=False        
        )
        
        self.ats.registerCallback(self.sync_callback)

        # Data containers
        self.POSITION = None # np.array, dtype float64 shape (7,)
        self.VELOCITY = None # np.array, dtype float64 shape (6,)
        self.FLS = None # np.aray (image)
        self.DVL_VEL = None # np.array, dtype float64 shape (3,)
        self.DVL_ALTITUDE = None # float64, shape (1, )
        self.IMU = None # np.array, dtype float64, shape (10, )
        self.TIME_STAMP = 0.0

        self.get_logger().info('Subscriptions created:\n- -> Odometry\n- -> FLS\n- -> DVL')


    def sync_callback(self, odometry_msg: Odometry, fls_msg: Image, dvl_msg: DVL, altitude_msg: Range, imu_msg : Imu): 
        

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
        
        self.VELOCITY = np.array([  odometry_msg.twist.twist.linear.x,
                                    odometry_msg.twist.twist.linear.y,
                                    odometry_msg.twist.twist.linear.z,
                                    odometry_msg.twist.twist.angular.x,
                                    odometry_msg.twist.twist.angular.y,
                                    odometry_msg.twist.twist.angular.z
                                ], dtype=np.float64)
        # --- DVL --- 
        self.DVL_VEL = np.array([
            dvl_msg.velocity.x,
            dvl_msg.velocity.y,
            dvl_msg.velocity.z
        ], dtype = np.float64)

        self.DVL_ALTITUDE = altitude_msg.range
        # --- IMU ---
        self.IMU = np.array([
            # 1. Orientation (Kwaternion)
            imu_msg.orientation.x,
            imu_msg.orientation.y,
            imu_msg.orientation.z,
            imu_msg.orientation.w,
            
            # 2. Angular Velocity (Gyroscope)
            imu_msg.angular_velocity.x,
            imu_msg.angular_velocity.y,
            imu_msg.angular_velocity.z,
            
            # 3. Linear Acceleration (Accelerometr)
            imu_msg.linear_acceleration.x,
            imu_msg.linear_acceleration.y,
            imu_msg.linear_acceleration.z
        ], dtype=np.float64)


        # --- FLS ----
        try:
            fls_img_cv = self.bridge.imgmsg_to_cv2(fls_msg, desired_encoding='passthrough')
            fls_img_cv = np.nan_to_num(fls_img_cv, nan=0.0)
            if fls_img_cv.dtype == np.float32 or fls_img_cv.dtype == np.float64:
                self.FLS = (np.clip(fls_img_cv, 0.0, 1.0) * 255).astype(np.uint8)
            else:
                self.FLS = cv2.convertScaleAbs(fls_img_cv, alpha=(255.0/65535.0) if fls_img_cv.dtype == np.uint16 else 1.0)
 
        except Exception as e:
            self.get_logger().error(f'[Error] Image conversion error:\n{e}')

        self.new_data_event.set()
# --- Control Node ---

class StonefishPublisher(Node):
    def __init__(self):
        super().__init__('stonefish_publisher')
        self.publisher = self.create_publisher(WrenchStamped, '/joy_wrench_stmp', 10)
    
    def send_cmd(self, cmd_shift: list, cmd_rotate: list): 

        msg = WrenchStamped()
        
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link' 

        shift_x, shift_y, shift_z = cmd_shift
        rotate_x, rotate_y, rotate_z = cmd_rotate

        # Direct mapping forces (Force [N])
        msg.wrench.force.x = float(shift_x)  # Forward/backward
        msg.wrench.force.y = float(shift_y)  # Left/right
        msg.wrench.force.z = float(shift_z)  # Up/down

        # Direct mapping torque (Torque [Nm])
        msg.wrench.torque.x = float(rotate_x) # Roll
        msg.wrench.torque.y = float(rotate_y) # Pitch
        msg.wrench.torque.z = float(rotate_z) # Yaw

        # self.get_logger().info(f'[Event] Command set: Force={cmd_shift}, Torque={cmd_rotate}')
        self.publisher.publish(msg)