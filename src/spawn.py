import time
import subprocess

NUM_PROC = 4

for i in range(NUM_PROC):
    command = ["roslaunch", "bipedal_robot", "spawn.launch", f"name:=robot{i}",
               f"pose:=-x 0 -y {i*10} -z 2.5"]
    result = subprocess.Popen(command, stdout=subprocess.PIPE)
    time.sleep(2)
