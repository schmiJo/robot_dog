from gym.envs.registration import register
register(
    id='Bittle_Dog-v0', 
    entry_point='bittle_dog.envs:BittleDogEnv'
)