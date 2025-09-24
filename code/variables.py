import numpy as np
import modern_robotics as mr
import math

def TsbVal(phi, x, y):
    """Calculates the Tsb Matrix based on the given angle and location

    :param phi: Angle of chassis turned in z axis
    :param x: X coordinate of chassis
    :param y: Y coordinate of chassis
    :return Tsb: Calculated 4x4 Transformation Tsb

    """
    Tsb = np.array([[np.cos(phi),-np.sin(phi),0,x],[np.sin(phi),np.cos(phi),0,y],[0,0,1,0.0963],[0,0,0,1]])
    return Tsb

class values():
    def __init__(self):
        
        self.initial_config = np.array([np.pi/6,-0.1,0.1,0,-0.2,0.2,-1.6,0,0,0,0,0,0]) # (phi,x,y,J1,J2,J3,J4,J5,W1,W2,W3,W4,gripper)
        self.current_config = self.initial_config   #Initial configuration of the bot and will be updated as there is a change in configuration
        self.speeds = np.zeros(13)      #speeds of the joints

        self.Tbo = np.array([[1, 0, 0, 0.1662], [0, 1, 0, 0], [0, 0, 1, 0.0026], [0, 0, 0 , 1]]) #chassis frame to base frame of arm
        self.Moe = np.array([[1, 0, 0, 0.033], [0, 1, 0, 0], [0, 0, 1, 0.6546], [0, 0, 0 , 1]])  #base frame of the arm wrt frame of the gripper 

        #List of Body Jacobians
        self.B1 = np.array([0, 0, 1, 0, 0.033, 0])  
        self.B2 = np.array([0, -1, 0, -0.5076, 0, 0])
        self.B3 = np.array([0, -1, 0, -0.3526, 0, 0])
        self.B4 = np.array([0, -1, 0, -0.2176, 0, 0])
        self.B5 = np.array([0, 0, 1, 0, 0, 0])

        self.Blist = np.array([self.B1, self.B2, self.B3, self.B4, self.B5]).T
        self.thetalist = self.initial_config[3:8].T #List of angles of the arm

        self.X = np.array([[0.17,0,0.985,0.387],[0,1,0,0],[-0.985,0,0.17,0.57],[0,0,0,1]])  #Actual end Effector configuration
        self.Xd = np.array([[0,0,1,0.5],[0,1,0,0],[-1,0,0,0.5],[0,0,0,1]])  #Current end-effector reference configuration
        self.Xd_next = np.array([[0,0,1,0.6],[0,1,0,0],[-1,0,0,0.3],[0,0,0,1]]) #End-effector reference configuration at the next timestep in the reference trajectory

        self.Kp_gain = 5    #Gain Value for P controller
        self.Ki_gain = 0 #Gain Value for I controller
        self.Kp = self.Kp_gain * np.identity(6) #Calculation of Kp matrix
        self.Ki = self.Ki_gain * np.identity(6)   #Calculation of Ki matrix

        self.error_integral = np.zeros(6)           #Error Integral Calculation

        self.dt = 0.01                                         #Time step
        self.max_speed = 15                            #Maximum speed of the wheels

        self.l = 0.47                                           #Length of the chassis
        self.w = 0.15                                         #Width of the Chassis
        self.r = 0.0475                                      #Radius of the wheels
        self.Hp = (self.r/4)*np.array([[-1/(self.l+self.w), 1/(self.l+self.w), 1/(self.l+self.w), -1/(self.l+self.w)], [1, 1, 1, 1], [-1, 1, -1, 1]])   #Pseudo inverse Homogeneous Matrix

        self.Tse_init = np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.5], [0, 0, 0 , 1]])      #initial configuration of gripper wrt state frame
        self.Tsc_init = np.array([[1, 0, 0, 1], [0, 1, 0, 0], [0, 0, 1, 0.025], [0, 0, 0, 1]])      #initial configuration of cube wrt state frame
        self.Tsc_final = np.array([[0, 1, 0, 0], [-1, 0, 0, -1], [0, 0, 1, 0.025], [0, 0, 0, 1]])   #final configuration of cube wrt state frame

        # Cube positions for new task 
        self.Tsc_init_2 = np.array([[1,0,0,1],[0,1,0,1],[0,0,1,0.025],[0,0,0,1]])
        self.Tsc_final_2 = np.array([[0,1,0,1],[-1,0,0,-1],[0,0,1,0.025],[0,0,0,1]])

print("Initial Variable Calculation Complete")




