import gym 
import bittle_dog 
env = gym.make("Bittle_Dog-v0") 

observation = env.reset()
for _ in range(100000):
  #env.render()
  action = env.action_space.sample() # your agent here (this takes random actions)
  observation, reward, done, info = env.step(action) 
  if done:
    observation = env.reset()
env.close()