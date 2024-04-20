import time
from environment import Environment

env = Environment()

for i in range(10):
    state, reward, done = env.step([2., 6., 2., 12.])
    print(state)
    time.sleep(1)
