import numpy as np
import math
from math import pi
import FK

## input 5 angle value
## output Jacobian

def vk_srv(angles):
        FK_now = FK.fk_srv(angles)
        delta_theta = 0.00000001
        
        jac = np.zeros((15,))
        
        for i in range(0, 5, 1):
                delta_angles = [angles[0], angles[1], angles[2], angles[3], angles[4]]
                delta_angles[i] += delta_theta

                FK_new_delta0 = FK.fk_srv(delta_angles)

                end_effector_now = [FK_now[0, 3], FK_now[1, 3], FK_now[2, 3]]
                end_effector_new_delta0 = [FK_new_delta0[0, 3], FK_new_delta0[1, 3], FK_new_delta0[2, 3]]

                ## compute data
                jac[0 + i] = (end_effector_new_delta0[0] - end_effector_now[0]) / delta_theta
                jac[5 + i] = (end_effector_new_delta0[1] - end_effector_now[1]) / delta_theta
                jac[10 + i] = (end_effector_new_delta0[2] - end_effector_now[2]) / delta_theta
                
                delta_angles[i] -= delta_theta

        return jac.reshape(3,5)

    
