import csv
import numpy as np
from math import sqrt, sinh, cosh
from numpy import max
from numpy.linalg import inv

from dataclasses import dataclass

@dataclass
class PS_Parameters:
    v_StB_Wpn_d_x: float
    v_StB_Wpn_d_y: float
    stept: float
    stept_n: float
    W_Wpc_d: float
    L_Wpc_d: float
    heading_change: float
    waypointPSP: any
    psp_foot_w: any
    switch_PSP: any
    v_StB_Wpn_d: any
    v_StB_Wpc_d: any


class PS_Planner:
    def __init__(self):
        self.PS_parameters_curr = PS_Parameters(v_StB_Wpn_d_x=0.0, v_StB_Wpn_d_y=0.0, stept=0.0, stept_n=0.0, 
                                                 W_Wpc_d=0.0, L_Wpc_d=0.0, heading_change=0.0, 
                                                 waypointPSP=None, psp_foot_w=None, switch_PSP=None, 
                                                 v_StB_Wpn_d=None, v_StB_Wpc_d=None)


    def read_PSP(self, filepath):
        # Initialize lists to store data
                # Initialize lists to store data
        dl_switch_list = []
        ds_switch_list = []
        step_length_list = []
        step_width_list = []
        step_time_list = []
        dheading_list = []
        t1_list = []
        t2_list = []
        opt_vn_list = []
        foot_list_x = []
        foot_list_y = []
        waypoint_list_x = []
        waypoint_list_y = []
        s_switch_list = []
        l_switch_list = []
        foot_list_z = []
        dh_list = []

        # Read each row and extract data
        with open(filepath, "r") as f:
            for line in f:
                values = line.split()
                dl_switch_list.append(float(values[0]))
                ds_switch_list.append(float(values[1]))
                step_length_list.append(float(values[2]))
                step_width_list.append(float(values[3]))
                step_time_list.append(float(values[4]))
                dheading_list.append(float(values[5]))
                t1_list.append(float(values[6]))
                t2_list.append(float(values[7]))
                opt_vn_list.append(float(values[8]))
                foot_list_x.append(float(values[9]))
                foot_list_y.append(float(values[10]))
                waypoint_list_x.append(float(values[11]))
                waypoint_list_y.append(float(values[12]))
                s_switch_list.append(float(values[13]))
                l_switch_list.append(float(values[14]))
                # foot_list_z.append(float(values[15]))
                # dh_list.append(float(values[16]))

        # Convert lists to NumPy arrays
        dl_switch_list = np.array(dl_switch_list).reshape(-1,1)
        ds_switch_list = np.array(ds_switch_list).reshape(-1,1)
        step_length_list = np.array(step_length_list).reshape(-1,1)
        step_width_list = np.array(step_width_list).reshape(-1,1)
        step_time_list = np.array(step_time_list).reshape(-1,1)
        dheading_list = np.array(dheading_list).reshape(-1,1)
        t1_list = np.array(t1_list).reshape(-1,1)
        t2_list = np.array(t2_list).reshape(-1,1)
        opt_vn_list = np.array(opt_vn_list).reshape(-1,1)
        foot_list_x = np.array(foot_list_x).reshape(-1,1)
        foot_list_y = np.array(foot_list_y).reshape(-1,1)
        waypoint_list_x = np.array(waypoint_list_x).reshape(-1,1)
        waypoint_list_y = np.array(waypoint_list_y).reshape(-1,1)
        s_switch_list = np.array(s_switch_list).reshape(-1,1)
        l_switch_list = np.array(l_switch_list).reshape(-1,1)
        # foot_list_z = np.array(foot_list_z).reshape(-1,1)
        # dh_list = np.array(dh_list).reshape(-1,1)
        
        self.target_vel_PSP = np.hstack([dl_switch_list, ds_switch_list])
        self.step_l = step_length_list
        self.step_w = step_width_list
        self.step_t = step_time_list
        self.delta_theta = dheading_list
        self.psp_t1 = t1_list
        self.psp_t2 = t2_list
        self.v_apex = opt_vn_list
        self.W_foot =  np.hstack([foot_list_x, foot_list_y])
        self.waypoint_PSP = np.hstack([waypoint_list_x, waypoint_list_y])
        self.switch_PSP_list = np.hstack([s_switch_list, l_switch_list])
        
        self.num_steps = len(dl_switch_list)
        
    
    def getPSP_step(self, step_index):
        
        if step_index + 1 >= self.num_steps:  # stop when there are no more steps in psp plan
            self.PS_parameters_curr.v_StB_Wpn_d_x = np.max([self.PS_parameters_curr.v_StB_Wpn_d_x - 0.1, 0.0])
            self.PS_parameters_curr.v_StB_Wpn_d_y = 0
            self.PS_parameters_curr.stept = 0.1
            self.PS_parameters_curr.stept_n = 0.1
            self.PS_parameters_curr.W_Wpc_d = 0.25
            self.PS_parameters_curr.L_Wpc_d = 0.0
            self.PS_parameters_curr.heading_change = 0.0
            self.PS_parameters_curr.v_StB_Wpn_d_x = 0.0
            self.PS_parameters_curr.waypointPSP = self.waypoint_PSP[-1]  # [num_steps-1]
            self.PS_parameters_curr.psp_foot_w = self.W_foot[-1]  # [num_steps-1]
            print("PSP plan ended")
            
        else:  # next PSP step
            self.PS_parameters_curr.v_StB_Wpn_d_x = self.target_vel_PSP[step_index + 1][1]
            self.PS_parameters_curr.v_StB_Wpn_d_y = self.target_vel_PSP[step_index + 1][0]
            # //=====uncomment for PSP Input :
            if self.PS_parameters_curr.v_StB_Wpn_d_x == 0:  # when the target is stopping use heuristics for step time and step width
                self.PS_parameters_curr.stept = 0.4
                self.PS_parameters_curr.stept_n = 0.4
                self.PS_parameters_curr.W_Wpc_d = 0.4
                self.PS_parameters_curr.L_Wpc_d = 0.0
                self.PS_parameters_curr.heading_change = self.delta_theta[step_index + 1]
                self.PS_parameters_curr.v_StB_Wpn_d_y = 0.0
                print("target stop & stopping")
                
            elif self.PS_parameters_curr.v_StB_Wpn_d_x != 0 and self.target_vel_PSP[step_index][1] == 0:  # currently stopping and targeting forward velocity, target step time is only t_1
                self.PS_parameters_curr.stept = self.PS_parameters_curr.stept_n
                self.PS_parameters_curr.stept_n = self.psp_t1[step_index + 1]
                self.PS_parameters_curr.W_Wpc_d = 0.4
                self.PS_parameters_curr.heading_change = self.delta_theta[step_index + 1]
                self.PS_parameters_curr.v_StB_Wpn_d_y = 0 #0.5 * 0.4 * sqrt(9.81 / 1.01) * sinh(sqrt(9.81 / 1.01) * 0.4) / (1 + cosh(sqrt(9.81 / 1.01) * 0.4))
                print("stopping & target walking")
                
            else:  # for every other walking step
                self.PS_parameters_curr.stept = self.PS_parameters_curr.stept_n
                self.PS_parameters_curr.stept_n = self.psp_t2[step_index] + self.psp_t1[step_index + 1]
                self.PS_parameters_curr.L_Wpc_d = self.step_l[step_index]
                self.PS_parameters_curr.W_Wpc_d = self.step_w[step_index]
                self.PS_parameters_curr.heading_change = self.delta_theta[step_index + 1] # At the start of the turning step in PSP, stance leg is already matches target
                print("walking")
            # for logging
            self.PS_parameters_curr.waypointPSP = self.waypoint_PSP[step_index]
            self.PS_parameters_curr.psp_foot_w = self.W_foot[step_index]
            if step_index == 0:
                self.PS_parameters_curr.switch_PSP = self.switch_PSP_list[step_index]
                self.PS_parameters_curr.waypointPSP = self.waypoint_PSP[step_index]
                self.PS_parameters_curr.psp_foot_w = self.W_foot[step_index]
            else:
                self.PS_parameters_curr.switch_PSP = self.switch_PSP_list[step_index - 1]
                self.PS_parameters_curr.waypointPSP = self.waypoint_PSP[step_index - 1]
                self.PS_parameters_curr.psp_foot_w = self.W_foot[step_index - 1]
        

        
        #   //Transforming velocity to the current waypoint frame
        # Matrix3d R_Wp_cn = AngleAxisd(PS_parameters_curr.heading_change, Vector3d::UnitZ()).toRotationMatrix();
        # PS_parameters_curr.v_StB_Wpn_d = Vector3d{PS_parameters_curr.v_StB_Wpn_d_x, PS_parameters_curr.v_StB_Wpn_d_y, 0};
        # PS_parameters_curr.v_StB_Wpc_d = R_Wp_cn * PS_parameters_curr.v_StB_Wpn_d;  
                
        # Transforming velocity to the current waypoint frame
        print("Remember Kasidit! heading_change is assumed to be in radian right now.")
        COS = np.cos((np.atleast_1d(self.PS_parameters_curr.heading_change)[0])) 
        SIN = np.sin((np.atleast_1d(self.PS_parameters_curr.heading_change)[0]))
        
        # Rotation matrix
        R_Wp_cn = np.array([[COS, -SIN, 0], 
                            [SIN,  COS, 0],
                            [0  ,    0, 1]])
        
        self.PS_parameters_curr.v_StB_Wpn_d = np.vstack([self.PS_parameters_curr.v_StB_Wpn_d_x, self.PS_parameters_curr.v_StB_Wpn_d_y, 0])
        self.PS_parameters_curr.v_StB_Wpc_d = R_Wp_cn @ self.PS_parameters_curr.v_StB_Wpn_d
  
    def getControlHyperparameters(self):
            # Accessing required 
            # parameters from self.PS_parameters_curr
            step_time_crnt = 0.4 # np.clip(np.atleast_1d(self.PS_parameters_curr.stept)[0],0.0,0.9)
            step_time_next = 0.4 #np.clip(np.atleast_1d(self.PS_parameters_curr.stept_n)[0],0.0,0.9)
            v_StB_d_x = self.PS_parameters_curr.v_StB_Wpc_d[0][0]+0.5
            v_StB_d_y = self.PS_parameters_curr.v_StB_Wpc_d[1][0]*0
            v_StB_d_z = self.PS_parameters_curr.v_StB_Wpc_d[2][0]
            stepl_d = np.atleast_1d(self.PS_parameters_curr.L_Wpc_d)[0]
            # if np.atleast_1d(self.PS_parameters_curr.W_Wpc_d)[0] > 0:
            #     stepw_d = np.atleast_1d(self.PS_parameters_curr.W_Wpc_d)[0]+0.4 ## maybe change tp 0.35
            # else:
            #     stepw_d = np.atleast_1d(self.PS_parameters_curr.W_Wpc_d)[0]-0.4 
            
            stepw_d = 0.4# np.atleast_1d(self.PS_parameters_curr.W_Wpc_d)[0]
            heading_change = np.atleast_1d(self.PS_parameters_curr.heading_change)[0]

            
            print('time_curr: ',step_time_crnt)
            print('time_next: ',step_time_next)
            print('vx: ',v_StB_d_x)
            print('vy: ',v_StB_d_y)
            # print(v_StB_d_z)
            print('dtheta: ', heading_change)
            print('stepl: ',stepl_d)
            print('stepw: ',stepw_d)


            # Return as a single NumPy array
            return np.array([step_time_crnt, step_time_next, v_StB_d_x, v_StB_d_y, v_StB_d_z, stepl_d, stepw_d, heading_change])
        