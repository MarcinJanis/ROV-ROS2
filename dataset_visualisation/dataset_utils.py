import os
import numpy as np
import cv2
from scipy.ndimage import gaussian_filter

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
    



def addSpeckleNoise(img, sigma = 0.5, m_min = 10, m_max = 200, beam_width = 5.0):
    '''
    Adds speckle noise to sonar image
    Sources: 
    https://www.researchgate.net/publication/4252645_Speckle_Simulation_Based_on_B-Mode_Echographic_Image_Acquisition_Model
    '''

    h, w = img.shape

    # intensity to amplitude I ~ A^2
    A = np.sqrt(img)

    m = np.random.randint(m_min, m_max + 1, size=(h, w)) # number os scatters for each pixel

    scale_map = sigma * np.sqrt(m)

    u = np.random.normal(loc=0.0, scale=scale_map , size=(h, w)) # real component of noise - amplitude
    v = np.random.normal(loc=0.0, scale=scale_map , size=(h, w)) # imaginary comonent of noise - phase

    # Gaussian filter with wide kernel 
    u_cor = gaussian_filter(u, sigma=(0, beam_width))
    v_cor = gaussian_filter(v, sigma=(0, beam_width))

    normalization_factor = np.sqrt(2 * np.pi * beam_width) # to keep energy for noise, despite gaussian blure

    u = u_cor * normalization_factor
    v = v_cor * normalization_factor

    # Add amplitude values
    A = A + u 

    img_noised = A**2 + v**2 # sum and module of coherent component (real objects echo) and incoherent component (echo from scatters)

    img_noised = img_noised.astype(np.uint8)

    return img_noised

import numpy as np

def energyLoss(img, alpha=0.008):
    rows = np.arange(img.shape[0])
    decay = np.exp(-alpha * rows) 
    decay = np.ones(img.shape)* 255 * decay[:, np.newaxis]
    img_after_loss = img + decay
    return np.clip(img_after_loss, 0, 255).astype(np.uint8)

def addBandReflects(img, omega1 = 0.02, omega2 = 0.12, gain = 0.02):
    cols = np.arange(img.shape[1])
    bands = (np.sin(cols * omega1) + np.sin(cols * omega2)) * gain * 255
    img_with_bands = bands + img
    return np.clip(img_with_bands, 0, 255).astype(np.uint8)

def Polar2Cartesian(img, r_min = 2.0, r_max = 30.0, theta_min = -65*np.pi/180, theta_max = 65*np.pi/180, out_shape = None):
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
                    interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=255)
    return output