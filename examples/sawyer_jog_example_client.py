from RobotRaconteur.Client import *
import time
import numpy as np

c = RRN.ConnectService('rr+tcp://localhost:58653?service=robot')

robot_info = c.robot_info
print(robot_info)

c.enable()

print(c.robot_state.PeekInValue()[0].command_mode)

robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", c)

halt_mode = robot_const["RobotCommandMode"]["halt"]
jog_mode = robot_const["RobotCommandMode"]["jog"]

c.command_mode = halt_mode
time.sleep(0.1)
c.command_mode = jog_mode

joint_diff = np.array([-5, 0, 0, 0, 0, 5, 0]) * np.deg2rad(1)
joint_vel = np.ones((7,))


for i in range(50):
    t = time.time()
    c.jog_freespace(np.array([1, 0.5, 1, 0.5, 0.5, 2, 2]) * np.sin(t) * np.deg2rad(5), joint_vel, False)
    print(hex(c.robot_state.PeekInValue()[0].robot_state_flags))
    time.sleep(0.1)
