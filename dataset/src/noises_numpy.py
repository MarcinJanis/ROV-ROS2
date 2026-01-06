import numpy as np
from scipy.ndimage import gaussian_filter

def addSpeckleNoise(img, sigma = 0.5, m_min = 10, m_max = 200, beam_width = 5.0):
    '''
    Adds speckle noise to sonar image
    Based on: https://www.researchgate.net/publication/4252645_Speckle_Simulation_Based_on_B-Mode_Echographic_Image_Acquisition_Model
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

    # img_noised = img_noised.astype(np.uint8)

    return img_noised

import numpy as np

def energyLoss(img, alpha=0.008):
    rows = np.arange(img.shape[0])
    decay = np.exp(-alpha * rows) 
    decay = np.ones(img.shape)* 255 * decay[:, np.newaxis]
    img_after_loss = img + decay
    return np.clip(img_after_loss, 0, 255)

def addBandReflects(img, omega1 = 0.02, omega2 = 0.12, gain = 0.02):
    cols = np.arange(img.shape[1])
    bands = (np.sin(cols * omega1) + np.sin(cols * omega2)) * gain * 255
    img_with_bands = bands + img
    return np.clip(img_with_bands, 0, 255)