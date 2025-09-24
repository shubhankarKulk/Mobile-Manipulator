import numpy as np
import modern_robotics as mr
import math
import csv
from variables import *
from timing_calc import *

def TrajectoryGenerator(T_init, T_final, Tf, dt, grasp_state, trajectories):

    N = round(Tf/dt + 1)
    traj = mr.ScrewTrajectory(T_init, T_final, Tf, N, 3)
    #Calculates the 12 vector trajectory
    
    for i in range(len(traj)):
        output = "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %d\n" % (traj[i][0][0], traj[i][0][1], traj[i][0][2], \
                                                                                                                     traj[i][1][0], traj[i][1][1], traj[i][1][2], traj[i][2][0], \
                                                                                                                     traj[i][2][1], traj[i][2][2], traj[i][0][-1], traj[i][1][-1], \
                                                                                                                     traj[i][2][-1], grasp_state)
        trajectories.write(output)


def PathPlan():

    #Values from the code variables.py
    robot = values()

    #opening and closing sequence of grasping
    grasp_close = [2, 3, 4, 5]
    grasp_open = [0, 1, 6, 7]

    #Sequence and duration of trajectory    
    Tconfig, durations = time_calc()

    #Open CSV file for trajectory generation
    trajectories = open("../results/trajectories.csv", "w")

    #Set the grasp to open if it is in the opening sequence, else set it to close
    for i in range(len(durations)):
        if i in grasp_close :
            TrajectoryGenerator(Tconfig[i], Tconfig[i+1], durations[i], robot.dt, 1, trajectories)
        if i in grasp_open:
            TrajectoryGenerator(Tconfig[i], Tconfig[i+1], durations[i], robot.dt, 0, trajectories)

    trajectories.close()

