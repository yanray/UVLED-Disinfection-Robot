import numpy as np
from math import radians
from random import uniform
from math import sqrt, atan2, pi, radians, sin, cos, atan
import FK, VK

limits = [radians(90), radians(90), radians(135), radians(135), radians(0)]

def gen_config():
        # create and return a valid arm config
        return [ uniform(-ll, ll) for ll in limits ]


def ik_srv(goal_position):
        interative_flag = True
        interative_times = 0
        inital_flag = False
        inital_time = 1
        counter = 0
        last_error = 0
        last2_error = 0

        current_angles = [0, 0, 0, 0, 0]
        ## initial joint angles
        vector_q = current_angles
        e = FK.fk_srv(vector_q)
        beta = 11.7

        while(interative_flag):
                if(inital_flag):
                        inital_flag = False
                        vector_q = gen_config()
                        vector_q[3] = (vector_q[1] + vector_q[2])
                        inital_time += 1
                        #print("Give a new joint anlge !!!!")

                ## Step1 J(q)
                vector_q[3] = -(vector_q[1] + vector_q[2])
                #vector_q[0] = - pi / 2 
                J_q = VK.vk_srv(vector_q)

                ## Step2 delta_e
                #g = goal_position

                delta_e = beta * np.array([goal_position[0] - e[0,3], goal_position[1] - e[1,3], goal_position[2] - e[2,3]])

                ## Step3 compute change 
                J_q_matrix = J_q
                J_q_inverse = J_q_matrix.T
                #print("jac:", J_q_inverse)
                delta_q = np.dot(J_q_inverse, delta_e)
                #print("delta_q:", delta_q)

                ## Step4 apply new vector q
                vector_q[3] = -(vector_q[1] + vector_q[2])
                #vector_q[0] = -pi / 2
                vector_q = vector_q + delta_q
                print("vector_q:", vector_q)
                for i in range(0, 5, 1):
                        limit_check = abs(vector_q[i]) - abs(limits[i])
                        if (abs(vector_q[i]) > abs(limits[i]) or vector_q[0] > 0):
                                #print("beyound angle limits")
                                vector_q = gen_config()
                                vector_q[3] = (vector_q[1] + vector_q[2])

                ## Step5 compute error
                e = FK.fk_srv(vector_q)
                
                np_e = np.array([e[0, 3], e[1, 3], e[2, 3]])
                np_g = np.array([goal_position[0], goal_position[1], goal_position[2]])
                error = np.linalg.norm(np_e - np_g)
                interative_times += 1
                print("error:", error)
                if(abs(last_error - error) < 0.00001 and error > 0.5):
                        inital_flag = True
                        interative_times = 0
                counter += 1
                #print("loop time:", counter)
                if(interative_times > 2000):
                        inital_flag = True
                        interative_times = 0
                if(error < 0.01):
                        interative_flag = False
                if(error < 0.5):
                        beta = 3.7;
                else:
                        beta = 11.7
                last2_error = last_error
                last_error = error

        print("error:", error)
        print("goal:", np_g)
        print("mine pose:", np_e)
        
        return vector_q


