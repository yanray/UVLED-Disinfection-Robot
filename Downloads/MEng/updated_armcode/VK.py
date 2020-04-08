import numpy as np
import math
from math import pi
import FK

## input 5 angle value
## output Jacobian

def vk_srv(angles):
        FK_now = FK.fk_srv(angles)
        #FK_now is a D-H matrix returned by F-K routine, it is nothing but the end goal position 
        delta_theta = 0.00000001
        
        jac = np.zeros((15,))
        
        for i in range(0, 5, 1):
                delta_angles = [angles[0], angles[1], angles[2], angles[3], angles[4]]
                delta_angles[i] += delta_theta #To add small theta to make new angles 

                FK_new_delta0 = FK.fk_srv(delta_angles)
                #FK_now_delta0 is a D-H matrix returned by F-K routine with the new angles, it is nothing but the end goal position 

                end_effector_now = [FK_now[0, 3], FK_now[1, 3], FK_now[2, 3]]
                #end_effector_now stores the translation matrix i.e. last column of the first three rows
                #Translation Matrix contains the positions 
                end_effector_new_delta0 = [FK_new_delta0[0, 3], FK_new_delta0[1, 3], FK_new_delta0[2, 3]]

                ## compute data
                #Jacobian Matrix returns the Jacobian computed (f(x+delta(x) - f(x)) / (delta(x) )
                #From Servo 1 to Servo 5, we calculate Jacobian for all three dimesnions i.e. x,y,z , hence the need of 15 values 
                jac[0 + i] = (end_effector_new_delta0[0] - end_effector_now[0]) / delta_theta
                jac[5 + i] = (end_effector_new_delta0[1] - end_effector_now[1]) / delta_theta
                jac[10 + i] = (end_effector_new_delta0[2] - end_effector_now[2]) / delta_theta
                
                delta_angles[i] -= delta_theta #Rretaining to the original matrix of angles 

        return jac.reshape(3,5)
    #Converts vector into a 3*5 matrix with each row for x,y,z and each column representing servo 

    
