import numpy as np
import modern_robotics as mr
import math
from variables import *

def time_calc():

    Ttotal = 13     #Total time for the motion

    robot = values()    #function values from the code variables
    
    Tse_init = robot.Tse_init   #initial configuration of the gripper wrt state
    Tsc_init = robot.Tsc_init   #initial configuration of cube wrt state
    Tsc_final = robot.Tsc_final #final configuration of cube wrt state

    a = math.pi/6   #parametrization
    Tce_standoff = np.array([[-np.sin(a), 0, np.cos(a), 0], [0, 1, 0, 0], [-np.cos(a), 0, -np.sin(a), 0.025], [0, 0, 0 , 1]])  #Height above ground level different
    Tce_grasp = np.array([[-np.sin(a), 0, np.cos(a), 0], [0, 1, 0, 0], [-np.cos(a), 0, -np.sin(a), 0], [0, 0, 0 , 1]])  #configuration of cube wrt gripper when it grasps the cube

    T_standoff_init = np.matmul(Tsc_init, Tce_standoff) #initial configuration of the hovering gripper
    T_grasp = np.matmul(Tsc_init, Tce_grasp)    #initial configuration of the grasping gripper wrt state
    T_standoff_final = np.matmul(Tsc_final, Tce_standoff)   #final configuration of hovering gripper wrt state
    T_release = np.matmul(Tsc_final, Tce_grasp) #final configuration of grasping gripper wrt state

    Tconfig = np.array([Tse_init, T_standoff_init, T_grasp, T_grasp, T_standoff_init, T_standoff_final, T_release, T_release, T_standoff_final])    #execution of trajectory

    Tse_init_q = np.array([Tse_init[0][-1], Tse_init[1][-1], Tse_init[2][-1]])  #initial location and rotation of gripper wrt state
    T_standoff_init_q = np.array([T_standoff_init[0][-1], T_standoff_init[1][-1], T_standoff_init[2][-1]])  #initial location and rotation of hovering gripper wrt state
    T_standoff_final_q = np.array([T_standoff_final[0][-1], T_standoff_final[1][-1], T_standoff_final[2][-1]])  #final location and rotation of hovering gripper wrt state
    T_grasp_q = np.array([T_grasp[0][-1], T_grasp[1][-1], T_grasp[2][-1]])  #initial location and rotation of grasping gripper wrt state
    T_release_q = np.array([T_release[0][-1], T_release[1][-1], T_release[2][-1]])  #final location and rotation of grasping gripper wrt state

    d1 = np.linalg.norm(Tse_init_q - T_standoff_init_q) #distance from the initial gripper and hovering gripper configuration
    d2 = np.linalg.norm(T_standoff_init_q - T_grasp_q) #distance from the hovering gripper and grasping gripper configuration
    d5 = np.linalg.norm(T_standoff_init_q - T_standoff_final_q) #distance from the initial hovering gripper and final hovering gripper configuration
    d6 = np.linalg.norm(T_standoff_final_q - T_release_q) #distance from the hovering gripper and final grasping gripper configuration

    dtotal = d1 + d2*2 + d5 + d6*2 #total distance measured. Since d2 and d6 are written twice in the configuration.

    tgrip_open = 0.63   #time required to open gripper
    tgrip_close = 0.63  #time required to close gripper

    #Timing of each segment calculated based on the fraction of distance from the total distance
    t1 = d1 * (Ttotal - tgrip_open - tgrip_close) / dtotal
    t2 = d2 * (Ttotal - tgrip_open - tgrip_close) / dtotal
    t4 = t2
    t5 = d5 * (Ttotal - tgrip_open - tgrip_close) / dtotal
    t6 = d6 * (Ttotal - tgrip_open - tgrip_close) / dtotal
    t8 = t6

    dt = 0.01

    #Duration array
    durations = np.array([t1, t2, tgrip_close, t4, t5, t6, tgrip_open, t8])

    for i in range(len(durations)):
        durations[i] = round(durations[i] * 100)/100

    return Tconfig, durations
