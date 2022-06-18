
# running the example controller in a HITL simulation
There are two controllers -- an altitude PD controller, and a position PD controller. The controller source code is in `src/rm_stuff/src/`

We will need 3 terminal windows to run the controller.
In 2 terminal windows, run:
- `roscore`
- `roslaunch dji_sdk_demo client_with_manifold.launch`
    - This launch file runs an interactive script that lets you request/release API control of the drone

In the demo client, press `b` + Enter to get API control of the drone, then press `d` + Enter to make the drone take off.
Once this is done, we can use the 3rd terminal window to run the actual controller (one of the below):
- Position controller
    - `roslaunch rm_stuff lateral_pid.launch enable_phasespace:=false`
- Altitude controller
    - `roslaunch rm_stuff altitude_pid.launch enable_phasespace:=false`

The setpoints are specified in the launch file. 


# running the example controller in the drone lab

## Steps


First, ensure that you have turned on the Phasespace, calibrated it, and obtained the JSON marker file.

Additionally, lower the drone safety net before flying the drone. Ensure that you are connected to
the same network as the drone (possibly `eduroam`) and know the IP address of both the drone and the base station/laptop.
The command `hostname -I` can be used to find the IP address of the current machine. It may be necessariy to plug peripherals (monitor, mouse, keyboard) 
into the drone in order to find its IP.

The recommended way to SSH into the drone is to use the [VSCode Remote - SSH](https://code.visualstudio.com/docs/remote/ssh) extension.
Instructions on how to use this extension to SSH into another machine (i.e the drone) can be found on the [VSCode website](https://code.visualstudio.com/docs/remote/ssh#_connect-to-a-remote-host).
This is the recommended way to SSH into the drone, as it may ocassionally be necessary to twiddle with launch file arguments or otherwise edit files,
and it would be significantly more inconvenient to do so through the terminal-based CLI client.


On both the base station and the drone, it is important to set the `ROS_IP` environment variable.
On the drone, this is set in `~/.bashrc`, meaning that it will be avaliable by default in every terminal.
However, the IP address is hard coded inside `~/.bashrc`, which means that it needs to be routinely checked in case the 
IP address of the drone changes.
On the base station, it can be set using `source set_ros_ip.sh`, where `set_ros_ip.sh` is a bash script in the root of
the catkin workspace. This script checks what the current IP is before setting `ROS_IP`. 

On the drone, it is equally important to set the `ROS_MASTER_URI` environment variable. This is also set (hardcoded) in `~/.bashrc` on the drone,
so ensure that the IP address there is up-to-date as well.

First, *on the base station*, run the following commands in separate terminals (in VSCode, the shortcut to open a new terminal is ``Ctrl + Shift + ` ``):
- `roscore`
- `rosrun phasespace rigid_tracker cs-phasespace.cs.umn.edu [path to tracker filename]`
    - Unless the arrangement of the markers on the drone has been changed, the path to the tracker filename from the root of this catkin workspace is `src/phasespace/tracker_03_20_2022.json`
    - This `rigid_tracker` node will publish phasespace data to the `/phasespace_markers` topic (locations of individual marker LEDs), `/phasespace_rigids` topic (position + orientation of rigid bodies tracked by the phasespace), and `/tf` (frames of reference for bodies tracked by the phasespace -- frame names come from the name of the tracker in the JSON file)
- `rosbag record /test_failsafe/desired_vs_actual /test_failsafe/error /test_failsafe/pose /test_failsafe/yaw /test_failsafe/velocity`
    - This will record information about how the controller is performing to a rosbag file. It can be later inspected using `rqt_bag`. This is a 
    useful way to check to see if the drone is properly reaching the setpoint, or to check if your gains are tuned properly.

At this point, the marker LEDs on the drone should turn red. Additionally, we should be able to `rostopic echo /phasespace_rigids` on both the drone and the base station to read 
nonzero position values for the drone. If you are not able to read values from the drone, then there is a networking issue (likely with the values of the `ROS_IP` variable on the drone and/or the base station).

At this point, we are ready to start flying the drone, and almost ready to start using our PD controller. Ensure that the drone joysticks are turned on and set to `F` mode.

Next, on the drone, run the following command
- `roslaunch dji_sdk_demo client_with_manifold.launch`
This will start the ROS node that interfaces with the drone hardware, as well as a TUI to interact with the drone.
From this TUI, press `b` + Enter to request control. When you are ready to start flying the drone, press `d` + Enter to take off.

Before starting the controller, inspect `src/rm_stuff/launch/lateral_pid.launch` to ensure that the PD gains and controller setpoint are 
correct. To start the controller on the drone, run the following command in a separate terminal:
- `roslaunch rm_stuff lateral_pid.launch enable_phasespace:=true`

The drone should now go to the specified location. 

The controller is also able to follow a sequence of waypoints. In order to send a path to the controller, 
1. alter `path.yaml` to have the waypoints that you want
2. call the service
    - `rosservice call /drone/followPath "$(cat path.yaml)"`
    - The file `path.yaml` is a message of type `nav_msgs/Path`

## issues you may run into:

- The Phasespace LEDs appear to be sensitive to electromagnetic interference. Sometimes, streaming images from the 
  Intel Realsense can be too much for the LEDs, causing them to turn off. When the LEDs are off, the Phasespace will 
  report that the drone is at `(0, 0, 0)`, potentially causing the controller to behave incorrectly.
