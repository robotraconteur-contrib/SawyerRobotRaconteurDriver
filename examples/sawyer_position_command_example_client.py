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
position_mode = robot_const["RobotCommandMode"]["position_command"]

RobotJointCommand = RRN.GetStructureType("com.robotraconteur.robotics.robot.RobotJointCommand",c)

c.command_mode = halt_mode
time.sleep(0.1)
c.command_mode = position_mode

cmd_w = c.position_command.Connect()
state_w = c.robot_state.Connect()

state_w.WaitInValueValid()

command_seqno = 1

t_start = time.time()

while (True):
    t = time.time()

    robot_state = state_w.InValue

    command_seqno += 1
    joint_cmd1 = RobotJointCommand()
    joint_cmd1.seqno = command_seqno
    joint_cmd1.state_seqno = robot_state.seqno
    joint_cmd1.command = np.array([1,0,0,0,1,0,0])*np.sin(t-t_start)*np.deg2rad(5)

    cmd_w.OutValue = joint_cmd1

    print(hex(c.robot_state.PeekInValue()[0].robot_state_flags))
    time.sleep(0.01)