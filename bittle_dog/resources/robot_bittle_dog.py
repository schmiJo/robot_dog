from numpy.random.mtrand import f
import pybullet as p
import os
import math
import numpy as np
import time


class RobotDog:
    
    def __init__(self, client) -> None:
        
        f_name = os.path.join(os.path.dirname(__file__), 'robot_bittle_dog/urdf/bittle.urdf') 
        self.robot_dog = p.loadURDF(fileName=f_name, basePosition=[0,0, 0], physicsClientId=client)
        self.client = client
        # rotor Joint indices
        self.joint_indices = [0,#"left back shoulder joint"
                              1,#'"left back_knee_joint"
                              2,#"left front shoulder joint"
                              3,#"left front knee joint",
                              4,#"right_ back shoulder joint"
                              5,#"right_back_knee_joint"
                              6,#"right_front_ shoulder_joint"
                              7,#"right_front_knee_joint"
                              ]
        
        #The minimum value that joint can achive (angles given in radians)
        self.joint_min = []
        
        #The maximum value that joint can achive (angles given in radians)
        self.joint_max = []
        
        self.num_joints = p.getNumJoints(self.robot_dog)
        
        print("This ROBOT HAS JOINTS ==================================")
        print(self.num_joints)
        
        for i in self.joint_indices:
            joint_info = p.getJointInfo(self.robot_dog , i)
            self.joint_min.append(joint_info[8])
            self.joint_max.append(joint_info[9])
            
        
        self.base_pos_and_orient = p.getBasePositionAndOrientation(self.robot_dog)
          
        
        # joint speed
        self.joint_speed = 0
        
        
        # Drag constants (Todo: Domain Randomize here)
        self.c_rolling = 0.2
        self.c_drag = 0.01
        
    def get_ids(self):
        return self.robot_dog, self.client
    
    
    def get_robot_dog_id(self):
        return self.robot_dog
    
    
    def apply_action(self, action: np.array):
        # expects action to be of shape (1,8) and the values between -1 and 1
        
        # might delete this for performance reasons
        action[action < -1] = -1
        action[action > 1] = 1
         
        p.setJointMotorControlArray(self.humanoid, self.joint_indices, controlMode=p.VELOCITY_CONTROL, forces=action*50)
    
    def reset(self):
        # TODO: Implement
        pass
    
    def get_observation(self):
        # TODO: Implement
        pass
        
    
        