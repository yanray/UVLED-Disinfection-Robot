import numpy as np
import math
from math import pi

def dh_matrix(a, alpha, d, theta):
    # Return a Denavit-Hartenberg matrix with the provided parameters
    cosTH = np.cos(theta)
    sinTH = np.sin(theta)
    cosAL = np.cos(alpha)
    sinAL = np.sin(alpha)

    row1 = [cosTH,  -(sinTH * cosAL),   sinTH * sinAL,       a * cosTH]
    row2 = [sinTH,  cosTH * cosAL,      -(cosTH * sinAL),    a * sinTH]
    row3 = [0,      sinAL,              cosAL,               d        ]
    row4 = [0,      0,                  0,                   1        ]

    a_4D = np.matrix([row1, row2, row3, row4])
    
    return a_4D

## a, alpha, d, theta

## input 5 theta value
## output DH matrix

def fk_srv(theta):
    a_4D1 = dh_matrix(0.015, -(pi / 2), 0.07, theta[0] + 0)
    a_4D2 = dh_matrix(0.097, 0, 0, theta[1] - (pi / 2))
    a_4D3 = dh_matrix(0.097, 0, 0, theta[2] + 0)
    a_4D4 = dh_matrix(0, pi / 2, 0, theta[3] + (pi / 2))
    a_4D5 = dh_matrix(0, 0, 0.16, theta[4] + 0)

    I = np.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    H00 = I
    H01 = I * a_4D1
    H02 = H01 * a_4D2
    H03 = H02 * a_4D3
    H04 = H03 * a_4D4
    H05 = H04 * a_4D5
    
    return H05

    
