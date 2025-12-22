import numpy as np 
import cv2
import os
import matplotlib.pyplot as plt


from dataset_utils import addSpeckleNoise, Polar2Cartesian, energyLoss

os.environ['DISPLAY'] = ':23'


fls_dir = './dataset/seq_1/fls'
imgs = sorted(
    os.listdir(fls_dir),
    key=lambda x: int(os.path.splitext(x)[0])
)


for img in imgs:
    img_pth = os.path.join(fls_dir, img)
    I = cv2.imread(img_pth)
    I = cv2.cvtColor(I, cv2.COLOR_BGR2GRAY)
    # I = energyLoss(I)
    I = addSpeckleNoise(I, m_min = 30, m_max = 100, sigma = 0.25, beam_width=2.0)
    I_cart = Polar2Cartesian(I)

    I_cart = cv2.resize(I_cart, None, fx=0.5, fy=0.5)
    cv2.imshow('Cartesian', I_cart)

    if cv2.waitKey(500) & 0xFF == 27:  
        break

cv2.destroyAllWindows()
