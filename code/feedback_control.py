import numpy as np
import modern_robotics as mr
import csv
import math
from variables import *
from nextState import *

def Jacobian(Toe, Hp, Blist, thetalist):
    """Calculates the Jacobian of the Robot

    :param Toe: A 4x4 Transformation Matrix of base wrt gripper
    :param Hp: A 4x4 Pseudoinverse Matrix of the Homogeneous Matrix
    :param Blist: A list of Body Jacobians for the Robot arm
    :param thetalist: A list of angles of the robot arm
    :return Je: A 6xN Jacobian Matrix consisting of Base Jacobian and Arm Jacobian

    """
    Teo = np.linalg.inv(Toe)
    AdTeb = mr.Adjoint(Teo)

    m = Hp.shape[1]
    F6 = np.zeros((6, m))
    F6[2:5,:] = Hp
    
    Jbase = np.matmul(AdTeb, F6)
    Jarm = mr.JacobianBody(Blist, thetalist)

    Je = np.concatenate((Jarm, Jbase), axis = 1)

    return Je

def TestLimits(config):
    """Tests the Limits of the Joints of the Arm

    :param config: A 1x12 configuration vector
    :return constrainJoints: Returns the constrains (if any) present on joints 3 and 4

    """
    theta1, theta2, theta3, theta4, theta5 = config[3:8]
    constrainJoints = []

    if theta3 < -2 or theta3 > 2:
        constrainJoints.append(3)

    if theta4 < -2 or theta4 > 2:
        constrainJoints.append(4)

    return constrainJoints

def pinv_tol(matrix, tol):
    """Calculates pseudo inverse considering the tolerance

    :param matrix: A NxM Matrix
    :param tol: Tolerance
    :return matrix_inv: pseudo inverse of the matrix with tolerances

    """
    matrix[np.where(np.abs(matrix) < tol)] = 0
    matrix_inv = np.linalg.pinv(matrix)
    return matrix_inv

def FeedbackControl(X, Xd, Xdnext, Kp, Ki, dt, current_config, error_integral, writer):
    """Calculates the Kinematic Task Space Feedforward plus Feedback control

    :param X: A 4x4 Transformation Matrix which is the actual end effector configuration 
    :param Xd: A 4x4 Transformation Matrix which is the current end effector reference configuration
    :param Xdnext: A 4x4 Transformation Matrix which is the next end effector configuration after time step
    :param Kp: P parameter for the PI Controller
    :param Ki: I parameter for the PI Controller
    :param dt: Time step
    :param current_config: A 12 vector configuration giving the current configuration of the robot
    :param error_integral: The error Twist after a time step
    :param writer: Function for CSV writing
    :return speeds, error_integral: speeds and error_integrals after a time step

    """
    robot = values()
    F = robot.Hp

    Moe = robot.Moe     #Initial configuration of gripper wrt arm frame
    Blist = robot.Blist     #List of Body Jacobians
    thetalist = current_config[3:8].T   #List of angles of the Arm

    Toe = mr.FKinBody(Moe, Blist, thetalist)    #Forward Kinematics of the Arm
    Je = Jacobian(Toe, F, Blist, thetalist)         #Jacobian of the Arm

    Xinv = np.linalg.inv(X)
    Xdinv = np.linalg.inv(Xd)

    Xerr = mr.se3ToVec(mr.MatrixLog6(np.matmul(Xinv, Xd)))  #Error in twist
    writer.writerow(Xerr)

    error_integral = error_integral + Xerr * dt     #error after a time step

    Vd = mr.se3ToVec((1/dt) * mr.MatrixLog6(np.matmul(Xdinv,Xdnext)))   #Feedforward reference twist
    Adj_Vd = np.matmul(mr.Adjoint(np.matmul(Xinv, Xd)), Vd)     #Adjoint of feedforward twist

    V = Adj_Vd + np.matmul(Kp, Xerr) + np.matmul(Ki, error_integral)    #The current twist

    tolerance = 0.002
    speeds = np.matmul(pinv_tol(Je, tolerance), V)  #calculating the speeds of the joints

##    print("\nError twist:", np.around(Xerr,3), "\n\nVd:", Vd, "\n\nAdj_Vd:", Adj_Vd, "\n\nV:", V, "\n\nspeeds:\n", np.around(speeds,3))

    #This calls the next state function and calculates the next configuration
    next_config = NextState(current_config, speeds, dt, 15)

    #Here, the joint limits are tested.
    constrain = TestLimits(next_config)
    if constrain != []:
        for joint in constrain:
            Je[:,joint-1] = 0
        speeds = np.matmul(pinv_tol(Je, tolerance), V)

    return speeds, error_integral



