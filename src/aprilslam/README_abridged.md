# Using Aprilslam

Aprilslam is a package for detecting and mapping/conducting SLAM using apriltags. Since we are using the Phasespace system, 
don't use any of the SLAM functionality, although we still need to install GTSAM in order to compile the package


## turning on the camera on the drone

There is a Realsense camera attached to the drone. The package to publish images from this camera is in another repository, `multidrone_nri_stack`,
which should already be cloned on the drone's Jetson. 

To publish images from the camera to ROS, run

```sh
source ~/virtualenv/multi_drone/bin/activate
source ~/multidrone_nri_stack/devel/setup.bash

python3 ~/multidrone_nri_stack/src/multi_camera/scripts/onboard_scripts/drone_station.py /camera/info:=/camera/camera_info /camera/color:=/camera/image_rect
```

then, to detect apriltags in the image, run

```sh
roslaunch aprilslam slam.launch
```

finally, to publish the transform between the camera frame and the drone marker frame, run
```sh
rosrun tf static_transform_publisher  0.170829 -0.0605029 -0.0924762  0.669839 -0.23261 0.643091 0.289209 tracker1 camera_frame 100
```
assuming that the drone frame is called `tracker1` according to `tf`. These values should be correct as long as the marker LEDs and camera on the drone
have not been moved since May 2022. Additionally, future calibrations should likely have similar values to this (if your newly calibrated values are extremely 
different, it is possible that something went wrong while calibrating).


If you are having issues localizing both the drone and the apriltag using Phasespace, you can also use `rosrun tf static_transform_publisher` to publish a static transform 
between the `phasespace` frame and the `gt_tag` frame (the ground truth location of the tag)

## Measuring the transform between the drone and the camera

First, 
1. Ensure the camera is publishing images to ROS
2. Ensure that aprilslam is detecting the location of the apriltags
3. Ensure that `tf` knows about the `gt_tag` frame (i.e the ground truth location of the tag)

Then, run the following ROS node

```sh
rosrun rm_stuff calibration
```

Normal hand-eye calibration is a complicated process that involves analyzing motions between consecutive frames.
This ROS node works off the fact that we can simply compute the transform from the camera to the drone knowing:
1. the transform from the drone's location to the phasespace system's origin
2. the transform from the phasespace origin to the apriltag
3. the transform from the apriltag to the camera (as measured by aprilslam)

While running this node, move the drone around the apriltag. This node prints the runnning average of all of the transforms. 
You will probably get a better result if you move the drone around the tag (slowly, so as to not blur the camera frame) so that you can 
get more data samples for the averaging.

Once you are satisfied, `Ctrl +  C` to exit the node and remember the last average that it printed out. You can use `rosrun tf static_transform_publisher <x> <y> <z> <qx> <qy> <qz> <qw> tracker1 camera_frame 100` to publish this transform to tf.


## Calibrating the camera

There are plenty of calibration checkerboards in the lab. Pick a calibration checkerboard and
- Count the number of internal corners on the checkerboard 
- Measure the size (in mm) of one of the squares of the checkerboard

Then, you can use the following command to run the camera calibrator

```sh
rosrun camera_calibration cameracalibrator.py --size <width>x<height> --square <square size in meters> --no-service-check image:=/camera/image_rect camera:=/camera 
```

Move the checkerboard around in the camera's field of view. Once you have collected a sufficiently wide dataset, the "calibrate" button in the GUI will enable. Once clicked,
it will compute the distortion parameters and camera intrinsic matrix.

It is important that the checkerboard is very flat (otherwise, the calibrator will think that there is a lot of lens distortion in the image).

Currently, the calibration results are hard-coded into `~/multidrone_nri_stack/src/multi_camera/scripts/onboard_scripts/camera_pub.py`, but in the future, they may be 
stored in a better way.
