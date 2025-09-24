import numpy as np
import modern_robotics as mr
import math
import csv
import time
from variables import *
from nextState import *
from trajectory_generation import *
from feedback_control import *
import matplotlib.pyplot as plt

def EndEffectorConfig(config, Moe, Tbo, Blist):
    """Calculates the End Effector Configuration Tse

    :param config: Current configuration of the robot
    :param Moe: Initial configuration of gripper in steady state 
    :param Tbo: Initial configuration of body frame wrt arm frame
    :param Blist: Body Jacobians of the Arm
    :return Tse: Returns the end effector configuration

    """
    phi, x, y = config[0:3]
    arm = config[3:8]
    Tsb = np.array([[np.cos(phi), -np.sin(phi), 0, x], [np.sin(phi), np.cos(phi), 0, y], [0, 0, 1, 0.0963], [0, 0, 0 , 1]]) #fixed S to chassis frame b

    Toe = mr.FKinBody(Moe, Blist, arm)
    Tso = np.matmul(Tsb, Tbo)
    Tse = np.matmul(Tso, Toe)

    return Tse

def TransformFormer(traj):
    """Returns the SE(3) configuration for given flattened array

    :param traj: Current Trajectory Value 
    :return T: Calculated 4x4 SE(3) Value of flattened array

    """
    R = traj[0:9].reshape(3,3)
    q = traj[9:12]

    T = np.zeros((4,4))
    T[:3,:3] = R
    T[:3,3] = q
    T[3,:] = [0,0,0,1]

    return T

def ErrorPlot(Kp, Ki):
    """Draws the Error Plot for the robot

    :param Kp: Value of P for PI controller
    :param Ki: Value of I for PI controller
    
    """
    error_csv = np.loadtxt('../results/error.csv', delimiter = ',')

    plt.plot(error_csv)
    plt.title = ("Error Plot with Kp = {Kp} and Ki = {Ki}")
    plt.savefig('../results/' + f'Kp = {Kp}, Ki = {Ki}.png')

def main():
    
    robot = values()
    
    PathPlan()  #calculating the trajectory of the robot
    
    print("Trajectory Generation Complete")
    traj = np.loadtxt('../results/trajectories.csv', delimiter = ',')

    f = open('../results/final_robot.csv', 'w')
    writer = csv.writer(f)
    writer.writerow(robot.initial_config)

    f1 = open('../results/error.csv', 'w')
    writer1 = csv.writer(f1)
    
    for i in range(len(traj)-1):
        X = robot.X #Current actual end-effector configuration
        Xd = TransformFormer(traj[i])   #Current end-effector reference configuration
        Xd_next = TransformFormer(traj[i+1])    #End-effector reference configuration at the next timestep in the reference trajectory
    
        speeds, error_integral = FeedbackControl(X, Xd, Xd_next, robot.Kp, robot.Ki, robot.dt, robot.current_config, robot.error_integral, writer1) #Calculating the speeds and error_integrals
        robot.speeds = speeds   #Setting new speeds   
        robot.error_integral = error_integral   #Setting new error_integral
        grip = traj[i,12]   #Getting the information of gripper status from trajectory

        robot.current_config = np.append((NextState(robot.current_config, robot.speeds, robot.dt, robot.max_speed)), int(grip)) #Updating the configuration of the robot
        writer.writerow(robot.current_config)

        robot.X = EndEffectorConfig(robot.current_config, robot.Moe, robot.Tbo, robot.Blist) #Upfating the value of actual end-effector configuration

    print("Final CSV file Generated")
    f.close()
    f1.close()
    ErrorPlot(robot.Kp_gain, robot.Ki_gain)
    print("Plot Complete")

if __name__ == "__main__":
    main()
