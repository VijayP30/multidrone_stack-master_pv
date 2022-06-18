# multidrone_stack

This is a ROS workspace containing packages that are useful for flying the 
DJI Matrice 100 drones

included packages:
- `Onboard-SDK-ROS`
    - package interfaces with DJI drone directly
- `phasespace`
    - package contains utilities useful for reading data from the Phasespace system
    - the Phasespace is the motion capture system in the Drone Lab used for localization
- `rm_stuff`
    - contains PID controllers to control the position of the drone
- `airsim_ros_pkgs` 
    - contains the ROS node that interfaces with AirSim
    - package contents taken from [AirSim GitHub repository](https://github.com/microsoft/AirSim)
- `aprilslam`
    - for detecting Apriltags
    - bundles `apriltag_mit` as a dependency

## building for the first time

To build `aprilslam`, we must first install GTSAM:

```sh
sudo add-apt-repository -y ppa:borglab/gtsam-release-4.0
sudo apt-get update
sudo apt-get install -y libgtsam-dev libgtsam-unstable-dev
```

Second, we must build AirSim:

```sh
./vendor/AirSim/setup.sh
./vendor/AirSim/build.sh
```

Now, we are ready to build our ROS packages

```sh
catkin build
```

The use of `catkin build` from `python-catkin-tools` is preferred over `catkin_make`.

Additionally, before running the AirSim simulator, you should copy `./airsim_example_settings.json` to `~/Documents/AirSim/settings.json`

## More details
- [using the PID controller on a DJI drone/in a DJI simulator](src/rm_stuff/README.md)
- [using `aprilslam`](src/aprilslam/README_abridged.md)
- [using AirSim ROS packages](src/airsim_drone_control/README.md)
- [using Phasespace](src/phasespace/phasespace_docs.pdf)

## potential issues you may run into

* If the computer running the DJI HITL simulation goes to sleep, it may cause commands to no 
longer register in the simulation. Closing and reopening the simulator should fix this issue.

* The `phasespace` package is unlikely to compile with `catkin_make` if you have freshly cloned
the respository. If you absolutely cannot use `catkin build`, to resolve this, run the command `catkin_make --pkg phasespace`
a couple of times until the `phasespace` package compiles on its own. Then, 
you should be able to compile the other packages using `catkin_make`.


## git repos we have vendored
- [aprilslam](https://github.com/ProjectArtemis/aprilslam)
- [AirSim](https://github.com/microsoft/AirSim)
- (an old version of) DJI [Onboard-SDK-ROS](https://github.com/dji-sdk/Onboard-SDK-ROS)
