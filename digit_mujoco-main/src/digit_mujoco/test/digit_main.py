import os
import datetime
import sys

from cfg.test.vel_track_test.vel_track_test_env_config import DigitTestEnvConfig
from cfg.test.vel_track_test.vel_track_test_config import DigitTestConfig
from garage.experiment.deterministic import set_seed
from envs.digit.test.vel_track_test.vel_track_test_env_flat import DigitTestEnvFlat
import numpy as np
import time

from cfg.test.vel_track_test.PS_Planner import PS_Planner
import rospy
import std_msgs

def main():
    
    #Intialize ROS node
    rospy.init_node('digit_main', anonymous=True)
        
    while not rospy.is_shutdown():
        
        #---------------------------
        # Flags
        #
        PSP_flag = False
        UserKeyInput_flag = not PSP_flag
        
        #---------------------------
        
        if PSP_flag:
            if UserKeyInput_flag:
                raise("Don't do both PSP_flag & UserKeyInput_flag")
        
        if UserKeyInput_flag:
            if PSP_flag:
                raise("Don't do both PSP_flag & UserKeyInput_flag")
    
        # directories
        home_path = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)) , os.pardir))
        log_time = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        log_dir = os.path.join(home_path, 'logs/record_vel_track/digit-state-' + log_time + ".csv")
        PSP_dir = os.path.join(home_path, 'log_psp_belief/psp-hyperparam-demo.txt')


        # config
        cfg_env = DigitTestEnvConfig()
        cfg_test = DigitTestConfig()
        cfg_env.vis_record.snapshot_gap = cfg_test.snapshot.snapshot_gap

        set_seed(cfg_env.seed)
        cfg_env.commands.samples.UserKeyInput_flag = UserKeyInput_flag
        
        #---------------------------------
        
        #PS Planner
        ps_planner = PS_Planner()
        ps_planner.read_PSP(filepath=PSP_dir)
        cfg_env.commands.samples.PSP_flag = PSP_flag #override keyinput if True
        cfg_env.commands.samples.ps_planner = ps_planner
        cfg_env.commands.resampling_time = 0.35
        
        #Example Demo of PSP 
        if PSP_flag:
            cfg_env.terrain.terrain_type = 'custom'
        
        #---------------------------------
        
        #Max simulation times
        if UserKeyInput_flag or PSP_flag: #For user key input
            
            cfg_env.env.max_time = 1000000

        else:
            
            # rollout
            # maximum time is set such that all the commands should be simulated
            cfg_env.env.max_time = max(cfg_env.env.max_time, cfg_env.commands.resampling_time* cfg_env.commands.samples.x_vel.shape[0])
            
        max_num_steps = int(cfg_env.env.max_time / cfg_env.control.control_dt)
        
        #---------------------------------
        
        #Instantiate Environment after setting up config
        env = DigitTestEnvFlat(cfg_env, log_dir)

        #---------------------------------

        #Main Loop
        eps_length = 0
        prev_obs, eps_info = env.reset()

        while eps_length < max_num_steps:
            st_time = time.time()
            a, agent_info = np.zeros(12,), {}
            es = env.step(a)  
    
            eps_length += 1
                
            if not es.terminal:
                prev_obs = es.observation

            # reset on terminal or time-out
            if es.last:
                prev_obs, eps_info = env.reset()
                print("Ouch! End Sim.")
                break
                
            end_time = time.time()
            
            # if (end_time - st_time) > env.cfg.control.control_dt:
            #     print("the simulation looks slower than it actually is")
            if (end_time - st_time) < env.cfg.control.control_dt:
                time.sleep(env.cfg.control.control_dt - (end_time - st_time))
                
        rospy.signal_shutdown("Finished execution")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        # Call rospy.signal_shutdown() to stop the node when the code is finished
        rospy.signal_shutdown("Finished execution")

