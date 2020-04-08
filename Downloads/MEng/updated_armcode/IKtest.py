import numpy as np
from math import radians
from random import uniform
from math import sqrt, atan2, pi, radians, sin, cos, atan
import FK, VK

limits = [radians(270), radians(270), radians(315), radians(315), radians(0)] #last angle 0 as last wali motor sabse niche wali h which \
#will not move

def gen_config():
        # create and return a valid arm config
        return [ uniform(-ll, ll) for ll in limits ]
    #uniform returns random float value b/w -ll to ll
    #gen_config returns random value  b/w -ll to ll, it returns entire list  



def ik_srv(goal_position):
    #We recieve the goal_postion value from main code to control the arm 
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
        #e stores the end goal position returned from fk_srv , e is a D-H matrix of the goal postion 
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
                #J_q has the jacobian matrix for the corresponding vector_q which contains the angles 

                ## Step2 delta_e
                #g = goal_position

                delta_e = beta * np.array([goal_position[0] - e[0,3], goal_position[1] - e[1,3], goal_position[2] - e[2,3]])
                #We calculate the difference b/w goal_postion from the arm_control code and the goal position recieved from FK
                #and from vector_q computed here. Why is it multiplied by beta? 
        

                ## Step3 compute change 
                J_q_matrix = J_q
                J_q_inverse = J_q_matrix.T
                #print("jac:", J_q_inverse)
                delta_q = np.dot(J_q_inverse, delta_e)
                #Makes a new vector which is a multiplication of Jacobian Inverse Matrix and delta e which is formed by the difference
                #of goal position from FK and e calculated using vector_q here 
                #print("delta_q:", delta_q)

                ## Step4 apply new vector q
                vector_q[3] = -(vector_q[1] + vector_q[2])
                #vector_q[0] = -pi / 2
                vector_q = vector_q + delta_q
                #print("vector_q:", vector_q) #2020 comment
                #The next few steps are used to see if we go beyond the limits, if yes then we set to the initial configuration
                #by calling gen_config again and resetting it 
                for i in range(0, 5, 1):
                        limit_check = abs(vector_q[i]) - abs(limits[i])
                        if (abs(vector_q[i]) > abs(limits[i]) or vector_q[0] > 0):
                                #print("beyound angle limits")
                                vector_q = gen_config()
                                vector_q[3] = (vector_q[1] + vector_q[2])

                ## Step5 compute error
                ##Get the new goal position using the new vector_q which we computed which is after limit check 
                e = FK.fk_srv(vector_q)
                
                np_e = np.array([e[0, 3], e[1, 3], e[2, 3]]) #Make and array for the translation matrix again 
                np_g = np.array([goal_position[0], goal_position[1], goal_position[2]]) # Make and array from goal_position obtained
                #Understand from here! #2020 comment 
                #from main code 
                error = np.linalg.norm(np_e - np_g)
                interative_times += 1
                #print("error:", error) #2020 comment
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

        #print("error:", error) #2020 comment
        #print("goal:", np_g) #2020 comment
        #print("mine pose:", np_e) #2020 comment
         
        return vector_q


