import gym
import numpy as np
import pybullet as p

import time

import math

from bittle_dog.resources.plane import Plane
from bittle_dog.resources.robot_bittle_dog import RobotDog
 
 
class BittleDogEnv(gym.Env):
    metadata = {'render.modes': ['human']}  
    
    def __init__(self) -> None:
        super().__init__()
        
        self.action_size = 12
        self.obs_size = 141
        
        
        self.action_space = gym.spaces.box.Box(
            low = np.array([-1] * self.action_size),
            high = np.array([1] * self.action_size)
        )
        
        self.observation_space = gym.spaces.box.Box(
            low = np.array([-1]* self.obs_size),
            high = np.array([1] * self.obs_size)    
        )
         
        self.client = p.connect(p.GUI)
        #self.client = p.connect(p.DIRECT)
        # Reduce length of episodes for RL algorithms
        p.setTimeStep(1/30, self.client)
        self.prev_distance_to_origin = 0
        
        self.done = False
        
        self.reward_factor = 20
        
        self.humanoid = None
        self.plane = None 
        p.resetSimulation(self.client)
        p.setGravity(0,0,-9.8)
        
        #load the Dog
        self.plane = Plane(self.client)
        self.robot_dog = RobotDog(self.client)
        
         
    def step(self, action):
        
        #Feed the action to the humanoid ang get observation of the state
        self.robot_dog.apply_action(action)
        p.stepSimulation()
        robot_dog_obs = np.hstack(self.robot_dog.get_observation())
        
        # Compute rewards as in distance to origin
        
        x_robo_dog = robot_dog_obs[0];
        y_robo_dog = robot_dog_obs[1];
    
         
        dist_to_origin = -y_robo_dog
        reward = max(dist_to_origin - self.prev_distance_to_origin, 0) * self.reward_factor;
        
        self.prev_distance_to_origin = dist_to_origin
        
        
        #Done by running off boundaries
        if (x_robo_dog >= 500 or x_robo_dog <= -500 or
                y_robo_dog >= 500 or y_robo_dog <= -500):
            self.done = True
            reward += 50
            
        contactPoints =   p.getContactPoints(self.robot_dog.get_robot_dog_id(), self.plane.get_plane_id() );
         
        for contact in contactPoints:
            if contact[1] == 1 and contact[2] == 0: 
                 if(contact[3] == 1):
                     # The Robot Flipped on his back -> terminate and punish
                     reward = -10
                     self.done = True

        
        obs = np.array(robot_dog_obs, dtype=np.float32)
        
        return obs, reward, self.done, dict()
    
    def reset(self):
        
        self.robot_dog.reset()
        
        self.done = False
        
        obs = self.robot_dog.get_observation()
        
        return np.hstack(obs)

    
    def render(self):
        pass
     
     