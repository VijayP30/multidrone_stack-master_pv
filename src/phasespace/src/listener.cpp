#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "phasespace/Markers.h"
#include "phasespace/Cameras.h"

using namespace std;

void error_cb(const std_msgs::String::ConstPtr &msg)
{
  ROS_INFO_STREAM("message=" << msg->data.c_str());
}

void camera_cb(const phasespace::Cameras &msg)
{
  for(size_t i = 0; i < msg.cameras.size(); i++)
    ROS_INFO_STREAM("id=" << msg.cameras[i].id
                    << " pos[" << msg.cameras[i].x << " " << msg.cameras[i].y << " " << msg.cameras[i].z
                    << "] rot[" << msg.cameras[i].qw << " " << msg.cameras[i].qx << " " << msg.cameras[i].qy << " " << msg.cameras[i].qz
                    << "] cond=" << msg.cameras[i].cond);
}

void marker_cb(const phasespace::Markers &msg)
{
  // ROS_INFO_STREAM("number of markers" << msg.markers.size());
  int count = 0;
  double sum_x = 0;
  double sum_y = 0;
  double sum_z = 0;
  for(size_t i = 0; i < msg.markers.size(); i++) {
    if (msg.markers[i].x != 0) {
      sum_x += msg.markers[i].x;
      sum_y += msg.markers[i].y;
      sum_z += msg.markers[i].z;
      count += 1;
    }
    // ROS_INFO_STREAM("t=" << msg.markers[i].time
    //                 << " id=" << msg.markers[i].id
    //                 << " [" << msg.markers[i].x << " " << msg.markers[i].y << " " << msg.markers[i].z
    //                 << "] cond=" << msg.markers[i].cond);
  }

  ROS_INFO_STREAM("avg x" << sum_x / count << ", avg y" << sum_y / count << ", avg z" << sum_z /count);

  ROS_INFO_STREAM("---");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle nh;
  ros::Subscriber markerSub = nh.subscribe("phasespace_markers", 10, marker_cb);
  // ros::Subscriber cameraSub = nh.subscribe("phasespace_cameras", 1000, camera_cb);
  // ros::Subscriber errorSub = nh.subscribe("phasespace_errors", 1000, error_cb);
  ros::spin();  
  return 0;
}
