import numpy as np

from utils.reward_functions import *
from envs.digit import DigitEnvFlat

class DigitTestEnvFlat(DigitEnvFlat):
    def __init__(self, cfg, log_dir=""):
        super().__init__(cfg, log_dir)
        self._reset_counter = 0
        self._command_sample_counter = 0

    def reset(self):
        ret_val = super().reset()
        self._reset_counter += 1
        return ret_val
    
    def _sample_commands(self):
        """
        sample command for test env.
        This should follow the fixed velocity schedule.
        make sure to call mbc.set_usr_command or mbc.reset after this. so that mbc's usr command is sync with env's.
        """
        
        #Set user command by given key inputs or specified sequences of velocities
        usr_command = np.zeros(3, dtype=np.float32)
        
        if self.cfg.commands.samples.UserKeyInput_flag:
            usr_command[0] = self.cfg.commands.samples.x_vel
            usr_command[1] = self.cfg.commands.samples.y_vel
            usr_command[2] = self.cfg.commands.samples.ang_vel
        else:
            usr_command[0] = self.cfg.commands.samples.x_vel[self._command_sample_counter % self.cfg.commands.samples.x_vel.shape[0]]
            usr_command[1] = self.cfg.commands.samples.y_vel[self._command_sample_counter % self.cfg.commands.samples.y_vel.shape[0]]
            usr_command[2] = self.cfg.commands.samples.ang_vel[self._command_sample_counter % self.cfg.commands.samples.ang_vel.shape[0]]
        
        if self.cfg.commands.samples.PSP_flag:
            
            print('step:  ', self.cfg.commands.samples.step_index)
            
            self.usr_command = usr_command
            self._command_sample_counter += 1
            self.cfg.commands.samples.ps_planner.getPSP_step(self.cfg.commands.samples.step_index)
            self.PS_parameters_curr = self.cfg.commands.samples.ps_planner.getControlHyperparameters()
            self.cfg.commands.samples.step_index += 1
            
            print("PS parameters :\n", self.PS_parameters_curr)
            print("---------------------------------------------------")
            
        else:
            
            print("User's commanded velocity: ")
            print("x_vel: ", usr_command[0])
            print("y_vel: ", usr_command[1])
            print("ang_vel: ", usr_command[2])
            print("---------------------------------------------------")
            
            if abs(usr_command[0]) < self.cfg.commands.ranges.cut_off:
                usr_command[0] = 0.
            if abs(usr_command[1]) < self.cfg.commands.ranges.cut_off:
                usr_command[1] = 0.
            if abs(usr_command[2]) < self.cfg.commands.ranges.cut_off:
                usr_command[2] = 0.

            self.usr_command = usr_command
            self._command_sample_counter += 1
            self.PS_parameters_curr = np.zeros(8)
            
            
            