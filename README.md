<p align="center"><img src="https://raw.githubusercontent.com/robotraconteur/robotraconteur/refs/heads/master/docs/figures/logo-header.svg"></p>

# Sawyer Robot Raconteur Driver

## Introduction

Robot Raconteur standard robot driver for the Rethink Robotics Sawyer Robot

This driver communicates with the Saywer robot using ROS 1 topics and provides a standard Robot Raconteur service with type `com.robotraconteur.robotics.robot.Robot`.

This driver uses the `ros_csharp_interop` library (https://github.com/johnwason/ros_csharp_interop).

It is highly recommended that the docker image be used. See below for instructions.

Example driver clients are in the `examples/` directory.

The [Robot Raconteur Training Simulator](https://github.com/robotraconteur-contrib/robotraconteur_training_sim) contains a simulated Sawyer robot in the multi-robot scene.

## Connection Info

The default connection information is as follows. These details may be changed using `--robotraconteur-*` command
line options when starting the service. Also see the
[Robot Raconteur Service Browser](https://github.com/robotraconteur/RobotRaconteur_ServiceBrowser) to detect
services on the network.

- URL: `rr+tcp://localhost:58653?service=robot`
- Device Name: `sawyer_robot`
- Node Name: `sawyer_robot`
- Service Name: `robot`
- Root Object Type:
  - `com.robotraconteur.robotics.robot.Robot`

## ROS Settings

The Sawyer robot acts as a ROS master on the network. Before starting the driver, the following environmental variables
must be configured:

* `ROS_MASTER_URI` - The ROS master URI. This is typically something like `http://192.168.1.5:11311`. Replace
`192.168.1.5` with the actual IP address of the Sawyer robot.
* `ROS_IP` - The IP address of the computer running the driver. This must be the ethernet interface on the same network with the Sawyer robot. Firewalls must be disabled so that the incoming connections from the robot can be accepted. A typical IP address is `192.168.1.6`.

## Command Line Arguments

The following command line arguments are available:

* `--robot-info-file=` - The robot info file. Info files are available in the `config/` directory. See [robot info file documentation](https://github.com/robotraconteur/robotraconteur_standard_robdef/blob/master/docs/info_files/robot.md)
* `--robot-name=` - Overrides the robot device name. Defaults to `sawyer_robot`.
* `--electric-gripper` - The Rethink electric gripper is attached.
* `--vacuum-gripper` - The Rethink vacuum gripper is attached.
* `--gripper-info-file=` - The gripper info file. Info files are available in the `config/` directory. See [tool info file documentation](https://github.com/robotraconteur/robotraconteur_standard_robdef/blob/master/docs/info_files/tool.md)
* `--gripper-name=` - Override the gripper device name. Defaults to `sawyer_electric_gripper` or `sawyer_vacuum_gripper`.

## Running With Docker 

Docker is the recommended way to run the driver. Building and running can be complicated due to the dependencies on ROS and C\#.

```
sudo docker run --rm --net=host --privileged  -v /var/run/robotraconteur:/var/run/robotraconteur -v /var/lib/robotraconteur:/var/lib/robotraconteur -e ROS_MASTER_URI=http://192.168.1.5:11311 -e ROS_IP=192.168.1.6 wasontech/sawyer-robotraconteur-driver /opt/sawyer_robotraconteur_driver/bin/SawyerRobotRaconteurDriver --robot-info-file=/config/sawyer_robot_default_config.yml
```

Replace `ROS_MASTER_URI` and `ROS_IP` with the settings from your network. Use the available command line options to configure the gripper and robot info files.  It may be necessary to mount a docker "volume" to access configuration yml files that are not included in the docker image. See the docker documentation for instructions on mounting a local directory as a volume so it can be accessed inside the docker.

## Building

Building the driver is complicated and not recommended. Refer to the `Dockerfile` for the required commands.

## Acknowledgment

This work was supported in part by the Advanced Robotics for Manufacturing ("ARM") Institute under Agreement Number W911NF-17-3-0004 sponsored by the Office of the Secretary of Defense. The views and conclusions contained in this document are those of the authors and should not be interpreted as representing the official policies, either expressed or implied, of either ARM or the Office of the Secretary of Defense of the U.S. Government. The U.S. Government is authorized to reproduce and distribute reprints for Government purposes, notwithstanding any copyright notation herein.

This work was supported in part by the New York State Empire State Development Division of Science, Technology and Innovation (NYSTAR) under contract C160142.

![](https://github.com/robotraconteur/robotraconteur/blob/master/docs/figures/arm_logo.jpg?raw=true)
![](https://github.com/robotraconteur/robotraconteur/blob/master/docs/figures/nys_logo.jpg?raw=true)