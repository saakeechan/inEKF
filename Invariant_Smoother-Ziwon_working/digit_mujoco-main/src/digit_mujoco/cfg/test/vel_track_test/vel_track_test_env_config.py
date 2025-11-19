# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2021 ETH Zurich, Nikita Rudin

import numpy as np
from cfg.base.base_config import BaseConfig,ConfigObj
from cfg.digit_env_config import DigitEnvConfig
import rospy
import std_msgs
from cfg.test.vel_track_test.PS_Planner import PS_Planner


class DigitTestEnvConfig(DigitEnvConfig):
    class env(DigitEnvConfig.env):
        max_time = 10.0  # every this time, the env is reset ans command is updated
    class commands(DigitEnvConfig.commands):
        resampling_time = 5. # commands are updated every reset
        class ranges(DigitEnvConfig.commands.ranges):
            cut_off = 0.0
        class samples(ConfigObj): # this can be adjusted before passed to environment
            def __init__(self) -> None:
                super().__init__()
                self.sub = rospy.Subscriber('/keyinput', std_msgs.msg.Int32, self.state_callback)  
                self.changeCmd = False
                self.PSP_flag = False
                self.UserKeyInput_flag = False
                
                self.x_vel = np.atleast_1d(0)
                self.y_vel = np.atleast_1d(0)
                self.ang_vel = np.atleast_1d(0)
                
                self.ps_planner = None
                self.step_index = 0
                
                # self.x_vel = np.array([0.1, 0, 0.1, 0, 0.1])
                # self.y_vel = np.array([0.,0.2, 0, 0.1, 0]) # 0.5
                # self.ang_vel = np.array([0.,0, 0, 0, 0]) 
                
            
            def state_callback(self, msg):
                print("received key input :" , msg.data)
                
                if not self.PSP_flag:
                    self.changeCmd = True
                    
                    #ranges
                    x_vel_min =   DigitEnvConfig.commands.ranges.x_vel_range[0]
                    x_vel_max =   DigitEnvConfig.commands.ranges.x_vel_range[1]
                    y_vel_min =   DigitEnvConfig.commands.ranges.y_vel_range[0]
                    y_vel_max =   DigitEnvConfig.commands.ranges.y_vel_range[1]
                    ang_vel_min = DigitEnvConfig.commands.ranges.ang_vel_range[0]
                    ang_vel_max = DigitEnvConfig.commands.ranges.ang_vel_range[1]
                    
                    if msg.data == 65362: #Forward
                        self.x_vel = np.atleast_1d(np.min( [x_vel_max , self.x_vel[0] + 0.1]))
                        
                    elif msg.data == 65364: #Backward
                        self.x_vel = np.atleast_1d(np.max( [x_vel_min , self.x_vel[0] - 0.1]))
                        
                    elif msg.data == 65361: #Left
                        self.ang_vel = np.atleast_1d(np.min( [ang_vel_max, self.ang_vel[0] + 0.1]))
                        # self.y_vel = np.atleast_1d(np.max( y_vel_min, self.y_vel - 0.1))
                        
                    elif msg.data == 65363: #Right   
                        self.ang_vel = np.atleast_1d(np.max( [ang_vel_min, self.ang_vel[0] - 0.1]))
                        # self.y_vel = np.atleast_1d(np.min( y_vel_max, self.y_vel + 0.1))
                        
                else:
                    
                    print("PS Planner is active. Key inputs are disabled")
                    
                
            
            

    class control(DigitEnvConfig.control):
        mbc_control = True # if true, mbc action is used in def step()

    class terrain(DigitEnvConfig.terrain): # this can be adjusted before passed to environment
        terrain_type = "flat" # flat, test_rough, rough_random???
        terrain_path = "terrain_info"        
        terrain_level_schedule = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9])
        which_terrain = 0
    
    class vis_record(DigitEnvConfig.vis_record): #TODO: change this
        visualize = True # should set to false when training
        record = False # should visualize true
        record_fps = 15
    
    class domain_randomization(DigitEnvConfig.domain_randomization):
        is_true = False
    
    class obs_noise(DigitEnvConfig.obs_noise):
        is_true = False