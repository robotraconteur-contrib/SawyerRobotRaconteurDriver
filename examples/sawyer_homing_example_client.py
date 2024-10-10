from RobotRaconteur.Client import *
import time
import numpy as np

c = RRN.ConnectService('rr+tcp://localhost:58653?service=robot')

robot_info = c.robot_info
print(robot_info)

c.disable()

time.sleep(2)

c.enable()

print(hex(c.robot_state.PeekInValue()[0].robot_state_flags))

robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", c)

halt_mode = robot_const["RobotCommandMode"]["halt"]
home_mode = robot_const["RobotCommandMode"]["homing"]


try:
    c.command_mode = halt_mode
except: pass

print(hex(c.robot_state.PeekInValue()[0].robot_state_flags))
time.sleep(0.1)
c.command_mode = home_mode
print(hex(c.robot_state.PeekInValue()[0].robot_state_flags))

homing = c.home()
try:
    while True:
        print(homing.Next())
except RR.StopIterationException: pass
