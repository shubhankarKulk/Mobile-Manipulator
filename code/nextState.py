import numpy as np
import modern_robotics as mr
import math
from variables import *
import csv

def AngleUpdate(angle, speed, dt):
     """Updates the angles of the wheels and the arm

    :param angle: A 1x9 vector
    :param speed: A 1x9 vector
    :param dt: Time step
    :return curr_angles: predicted angles corresponding to the inputs

    """
     curr_angles = []
     for i in range(len(angle)):
          curr_angles.append(angle[i] + speed[i] * dt)

     return curr_angles

def NextStateWheels(T, angles):
    """Finds the change in position of the chassis

    :param T: A 1x3 vector
    :param angles: A 1x9 vector
    :return qnew: predicted location qnew corresponding to the inputs

    """
    robot = values()
    F = robot.Hp

    Vb = np.matmul(F, angles.T)
    omegabz = Vb[0]
    v_bx = Vb[1]
    v_by = Vb[2]

    if omegabz < 0.00001:
        deltaqb = Vb
    else:
        deltaxb = (v_bx*np.sin(omegabz) + v_by*(np.cos(omegabz) - 1)) / omegabz
        deltayb = (v_by*np.sin(omegabz) + v_bx*(1 - np.cos(omegabz))) / omegabz
        deltaqb = np.array([omegabz, deltaxb, deltayb]).T

    phi = T[0]
    deltaq = np.matmul(np.array([[1,0,0],[0,np.cos(phi),-np.sin(phi)],[0,np.sin(phi),np.cos(phi)]]), deltaqb)

    qnew = T + deltaq

    return qnew
    

def NextState(current_state, speeds, dt, max_speed):
    """Predicting the next state of the robot

    :param current_state: A 1x12 vector
    :param speeds: A 1x9 vector
    :param dt: Time delay
    :paran max_speed: Maximum Speed of the wheels
    :return new_state: predicted 12 vector for chassis corresponding to the inputs

    """
    new_state = np.ones(12)

    phi,x,y = current_state[0:3]
    angles = current_state[3:12]

    for i in range(len(speeds)):
        if speeds[i] < -max_speed:
            speeds[i] = -max_speed
        elif speeds[i] > max_speed:
            speeds[i] = max_speed

    new_state[0:3] = NextStateWheels(current_state[0:3], dt*speeds[5:9])
    new_state[3:12] = AngleUpdate(angles,speeds,dt)
    
    return new_state

