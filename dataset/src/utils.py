import numpy as np
import cv2

data_headers = {
    'step_idx':0,
    't':1,
    'pos_x':2,
    'pos_y':3,
    'pos_z':4,
    'orient_x':5,
    'orient_y':6,
    'orient_z':7,
    'orient_w':8,
    'dvl_vel_x':9,
    'dvl_vel_y':10,
    'dvl_vel_z':11,
    'dvl_vel_x':12,
    'alt':13,
    'imu_orient_x':14,
    'imu_orient_y':15,
    'imu_orient_z':16,
    'imu_gyro_x':17,
    'imu_gyro_y':18,
    'imu_gyro_z':19,
    'imu_accel_x':20,
    'imu_accel_y':21,
    'imu_accel_z':22,
}
    
def polar2cartesian(img, r_min = 2.0, r_max = 30.0, theta_min = -65*np.pi/180, theta_max = 65*np.pi/180, out_shape = None, bg = 0):
    # r - ranges
    # t - theta - beam angle
    nr, nt = img.shape 

    if out_shape is None:
        out_shape = (nr, 2*nr)

    output = np.ones(out_shape, dtype=np.uint8)

    # center of out img
    cy = out_shape[0]
    cx = out_shape[1]//2

    # create mesh grid for output size
    X, Y = np.meshgrid(np.arange(out_shape[1]), np.arange(out_shape[0]))

    # shift coords to set (0, 0) in robot position 
    X = (X - cx).astype(np.float32)
    Y = (cy - Y).astype(np.float32)

    # convert to real world vals
    scale = r_max / cy
    X = X * scale
    Y = Y * scale

    # create remapping -> r = sqrt(x^2 + y^2), theta = atan2(x, y)
    R = np.sqrt(X**2 + Y**2)
    T = np.arctan2(X, Y)

    # convert to input image coords sys 
    dR = (r_max - r_min) / nr 
    dT = (theta_max - theta_min) / nt

    R_map = (R - r_min)/dR
    Theta_map = (T - theta_min)/dT

    # output = cv2.remap(img, R_map.astype(np.float32), Theta_map.astype(np.float32), interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=255)
    # Theta_map odpowiada za kolumny (wiązki), R_map za wiersze (odległość)
    output = cv2.remap(img, Theta_map.astype(np.float32), R_map.astype(np.float32), 
                    interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=bg)
    return output