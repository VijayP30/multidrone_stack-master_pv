# Airsim

AirSim is a drone simulator built on top of Unreal Engine. Although it has a custom C++/Python API for interacting with 
the simulated drone, there is a ROS wrapper around this API. 

The AirSim documentation can be found [here](https://microsoft.github.io/AirSim/).

## running the AirSim simulation

First, open the Unreal Editor and click "Play" to start the simulation. It is very important that the simulation is
started before you try to start the AirSim ROS node, otherwise the ROS node will hang for an uncomfortably long time.

Once the simulation is running, the ROS nodes can be started with 


```sh
roslaunch airsim_drone_control everything.launch
```


This will cause the drone to fly around the apriltag 


## adjusting the camera's settings

The settings for the camera are located in `~/Documents/AirSim/settings.json`. Using this file, you can control
a number of settings, such as the camera FoV, orientation, and resolution. 

To have settings changes take effect
1. Stop the ROS nodes
2. Stop the simulation (by clicking "stop" in the Unreal Editor)
3. Start the simulation

It is not necessary to close and reopen the Unreal Editor

## adjusting the path the drone flies on

Unlike the controller for the DJI drone, the path for the AirSim drone is hard-coded into the file `src/airsim_drone_control/src/move_drone_along_trajectory.cc`.

## making the apriltag wave

Edit the blueprint for `boxflap`. Edit the construction script. Alter the amplitude and bias as appropriate. These values are upper bounds -- as the drone moves away from
the apriltag, the amplitude, frequency, and bias will all go to 0.
