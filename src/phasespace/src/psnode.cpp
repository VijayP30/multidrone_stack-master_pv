#include <iostream>

#include "ros/ros.h"
#include "owl.hpp"
#include "std_msgs/String.h"
#include <phasespace/Camera.h>
#include <phasespace/Cameras.h>
#include <phasespace/Marker.h>
#include <phasespace/Markers.h>

using namespace std;

int main(int argc, char** argv)
{
  // get the owl server address through the command line
  // 'rosrun phasespace phasespace_node 192.168.1.123'
  string address = argc > 1 ? argv[1] : "localhost";

  // create the context
  OWL::Context owl;
  OWL::Markers markers;
  OWL::Cameras cameras;

  // initialize ROS
  ros::init(argc, argv, "phasespace_node");
  ros::NodeHandle nh;
  ros::Publisher errorsPub = nh.advertise<std_msgs::String>("phasespace_errors", 1000, true);
  ros::Publisher camerasPub = nh.advertise<phasespace::Cameras>("phasespace_cameras", 1000, true);
  ros::Publisher markersPub = nh.advertise<phasespace::Markers>("phasespace_markers", 1000);

  ROS_INFO("hello!");

  // simple example
  if(owl.open(address) <= 0 || owl.initialize("timebase=1,1000000") <= 0) {
    ROS_ERROR("Could not connect to the address %s", address.c_str());
    return 0;
  }
  ROS_INFO("successfully connected to %s!", address.c_str());

  // start streaming
  owl.streaming(1);

  // main loop
  while(ros::ok() && owl.isOpen() && owl.property<int>("initialized"))
    {
      const OWL::Event *event = owl.nextEvent(1000);
      if(!event) continue;

      if(event->type_id() == OWL::Type::ERROR)
        {
          cerr << event->name() << ": " << event->str() << endl;
          std_msgs::String str;
          str.data = event->str();
          errorsPub.publish(str);
        }
      else if(event->type_id() == OWL::Type::CAMERA)
        {
          if(event->name() == string("cameras") && event->get(cameras) > 0)
            {
              phasespace::Cameras out;
              for(OWL::Cameras::iterator c = cameras.begin(); c != cameras.end(); c++)
                {
                  phasespace::Camera cam;
                  //
                  cam.id = c->id;
                  cam.flags = c->flags;
                  cam.x = c->pose[0];
                  cam.y = c->pose[1];
                  cam.z = c->pose[2];
                  cam.qw = c->pose[3];
                  cam.qx = c->pose[4];
                  cam.qy = c->pose[5];
                  cam.qz = c->pose[6];
                  cam.cond = c->cond;
                  //
                  out.cameras.push_back(cam);
                }
              camerasPub.publish(out);
            }
        }
      else if(event->type_id() == OWL::Type::FRAME)
        {
          if(event->find("markers", markers) > 0)
            {
              phasespace::Markers out;
              for(OWL::Markers::iterator m = markers.begin(); m != markers.end(); m++)
                {
                  phasespace::Marker mout;
                  mout.id = m->id;
                  mout.time = m->time;
                  mout.flags = m->flags;
                  mout.cond = m->cond;
                  mout.x = m->x;
                  mout.y = m->y;
                  mout.z = m->z;
                  out.markers.push_back(mout);
                }

              markersPub.publish(out);
            }
        }
    } // while

  owl.done();
  owl.close();
  
  return 0;
}
