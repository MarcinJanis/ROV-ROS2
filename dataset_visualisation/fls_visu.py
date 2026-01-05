import numpy as np 
import cv2
import os
import matplotlib.pyplot as plt


from dataset_utils import addSpeckleNoise, Polar2Cartesian, energyLoss, addBandReflects

os.environ['DISPLAY'] = ':23'


fls_dir = './dataset/seq_3/fls'
imgs = sorted(
    os.listdir(fls_dir),
    key=lambda x: int(os.path.splitext(x)[0])
)


for img in imgs:
    img_pth = os.path.join(fls_dir, img)
    I = cv2.imread(img_pth)
    I = cv2.cvtColor(I, cv2.COLOR_BGR2GRAY)
    # add noise
    I = addBandReflects(I, omega1 = 0.03, omega2 = 0.07, gain = 0.02)
    I = energyLoss(I, alpha = 0.02)
    I = addSpeckleNoise(I, m_min = 30, m_max = 100, sigma = 0.25, beam_width=2.0)
    I = I.astype(np.uint8)
    # convert to cartesian
    I_cart = Polar2Cartesian(I, r_min = 0.5, r_max = 30.0, theta_min = -65*np.pi/180, theta_max = 65*np.pi/180, out_shape = None)
    I_cart = cv2.resize(I_cart, None, fx=0.5, fy=0.5)
    cv2.imshow('Cartesian', I_cart)

    if cv2.waitKey(200) & 0xFF == 27:  
        break

cv2.destroyAllWindows()
