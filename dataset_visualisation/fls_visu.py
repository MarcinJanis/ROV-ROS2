import numpy as np 
import cv2
import os
import matplotlib.pyplot as plt

os.environ['DISPLAY'] = ':23'



def Polar2Cartesian(img, r_min = 2.0, r_max = 30.0, theta_min = -65*np.pi/180, theta_max = 65*np.pi/180, out_shape = None):
    # r - range - distance
    # t - theta - angle
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

    output = cv2.remap(img, R_map.astype(np.float32), Theta_map.astype(np.float32), interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=255)
    
    return output
   

# -------------------------- #

fls_dir = './dataset/seq_1/fls'
imgs = sorted(
    os.listdir(fls_dir),
    key=lambda x: int(os.path.splitext(x)[0])
)


for img in imgs:
    img_pth = os.path.join(fls_dir, img)
    I = cv2.imread(img_pth)
    I = cv2.cvtColor(I, cv2.COLOR_BGR2GRAY)
    I_cart = Polar2Cartesian(I)

    I_cart = cv2.resize(I_cart, None, fx=0.5, fy=0.5)
    cv2.imshow('Cartesian', I_cart)

    if cv2.waitKey(500) & 0xFF == 27:  
        break

cv2.destroyAllWindows()
