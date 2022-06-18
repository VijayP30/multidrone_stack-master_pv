#include <ros/ros.h>

#include <phasespace/Markers.h>
#include <phasespace/Marker.h>
#include <phasespace/Rigids.h>
#include <phasespace/Rigid.h>
#include <sstream>
#include <rm_stuff/common.h>

#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/Header.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>

class PhasespaceToRVizPipe {
private:
    std_msgs::ColorRGBA markerColor;
    std_msgs::ColorRGBA rigidsColor;

    ros::Subscriber phasespaceMarkersSubscriber;
    ros::Publisher rvizMarkerArrayPublisher;

    ros::Subscriber phasespaceRigidsSubscriber;
    ros::Publisher rvizRigidPublisher;

    tf2_ros::TransformBroadcaster droneTfBroadcaster;


    void processMarkers(phasespace::Markers markers) {
        ROS_INFO_THROTTLE(5, "still receiving markers from phasespace");
        static int markerSeq = 0;
        visualization_msgs::MarkerArray rvizMarkers;
        for (auto& psMarker : markers.markers) {
            std_msgs::Header rvizMarkerHeader;
            rvizMarkerHeader.frame_id = "phasespace";
            rvizMarkerHeader.seq = markerSeq;
            rvizMarkerHeader.stamp = ros::Time::now(); //(0, psMarker.time);

            visualization_msgs::Marker rvizMarker;
            rvizMarker.header = rvizMarkerHeader;

            rvizMarker.ns = "phasespace_markers";
            rvizMarker.id = psMarker.id;
            rvizMarker.type = visualization_msgs::Marker::CUBE;
            rvizMarker.action = visualization_msgs::Marker::ADD;
            rvizMarker.pose.position.x = psMarker.x / 1000.0;
            rvizMarker.pose.position.y = psMarker.y / 1000.0;
            rvizMarker.pose.position.z = psMarker.z / 1000.0;
            rvizMarker.scale.x = rvizMarker.scale.y = rvizMarker.scale.z = 0.1;
            rvizMarker.color = markerColor;
            rvizMarker.lifetime = ros::Duration(1.0);

            rvizMarkers.markers.push_back(rvizMarker);
        }

        rvizMarkerArrayPublisher.publish(rvizMarkers);
        markerSeq++;
    }

    void processRigids(const phasespace::RigidsConstPtr &rigids) {
        ROS_INFO_THROTTLE(5, "still receiving rigids from phasespace");
        static int rigidsSeq = 0;

        visualization_msgs::MarkerArray rvizRigids;
        for(auto& psRigid : rigids->rigids) {
            std_msgs::Header rvizMarkerHeader;
            rvizMarkerHeader.frame_id = "phasespace";
            rvizMarkerHeader.seq = rigidsSeq;
            rvizMarkerHeader.stamp = ros::Time::now(); //(0, psMarker.time);

            visualization_msgs::Marker rvizRigid;
            rvizRigid.header = rvizMarkerHeader;

            rvizRigid.ns = "phasespace_markers";
            rvizRigid.id = psRigid.id;
            rvizRigid.type = visualization_msgs::Marker::CUBE;
            rvizRigid.action = visualization_msgs::Marker::ADD;
            rvizRigid.pose.position.x = psRigid.x / 1000.0;
            rvizRigid.pose.position.y = psRigid.y / 1000.0;
            rvizRigid.pose.position.z = psRigid.z / 1000.0;
            rvizRigid.pose.orientation.w = psRigid.qw;
            rvizRigid.pose.orientation.x = psRigid.qx;
            rvizRigid.pose.orientation.y = psRigid.qy;
            rvizRigid.pose.orientation.z = psRigid.qz;
            rvizRigid.scale.x = rvizRigid.scale.z = 1.0;
            rvizRigid.scale.y = 0.2;
            rvizRigid.color = rigidsColor;
            rvizRigid.lifetime = ros::Duration(1.0);

            rvizRigids.markers.push_back(rvizRigid);

            geometry_msgs::TransformStamped transform;
            transform.transform.rotation.w = psRigid.qw;
            transform.transform.rotation.x = psRigid.qx;
            transform.transform.rotation.y = psRigid.qy;
            transform.transform.rotation.z = psRigid.qz;

            transform.transform.translation.x = psRigid.x / 1000.0;
            transform.transform.translation.y = psRigid.y / 1000.0;
            transform.transform.translation.z = psRigid.z / 1000.0;

            transform.header = rvizMarkerHeader;
            std::stringstream child_frame_id;
            child_frame_id << "phasespace/rigid_" << psRigid.id;
            transform.child_frame_id = child_frame_id.str();
            droneTfBroadcaster.sendTransform(transform);
        }


        rigidsSeq++;
        rvizRigidPublisher.publish(rvizRigids);
    }
public:
    PhasespaceToRVizPipe(ros::NodeHandle nh) {
        markerColor.r = 1;
        markerColor.g = 1;
        markerColor.b = 1;
        markerColor.a = 0.5;

        rigidsColor.r = 1;
        rigidsColor.g = 0;
        rigidsColor.b = 1;
        rigidsColor.a = 0.2;

        phasespaceMarkersSubscriber = nh.subscribe("/phasespace_markers", 1, &PhasespaceToRVizPipe::processMarkers, this);
        phasespaceRigidsSubscriber = nh.subscribe("/phasespace_rigids", 1, &PhasespaceToRVizPipe::processRigids, this);
        
        rvizMarkerArrayPublisher = nh.advertise<visualization_msgs::MarkerArray>("/phasespace_markers/viz", 1, false);
        rvizRigidPublisher = nh.advertise<visualization_msgs::MarkerArray>("/phasespace_rigids/viz", 1, false);
    }
};

class PublishLocalizerOutput {
private:
    std::unique_ptr<LocalizationSensor> localizer;
    ros::Timer timer;
    ros::Publisher pointPublisher;

    tf2_ros::TransformBroadcaster droneTfBroadcaster;

    void pushNewFrame(const ros::TimerEvent &te) {
        ROS_INFO_THROTTLE(5, "still receiving rigids from localizer");
        static int pushNewFrameSeq = 0;
        auto pose = localizer->getPose();

        std_msgs::Header header;
        header.frame_id = "world";
        header.seq = pushNewFrameSeq++;
        header.stamp = ros::Time::now(); //(0, psMarker.time);

        geometry_msgs::TransformStamped transform;
        transform.header = header;

        transform.transform.rotation.w = pose.rotation.w();
        transform.transform.rotation.x = pose.rotation.x();
        transform.transform.rotation.y = pose.rotation.y();
        transform.transform.rotation.z = pose.rotation.z();

        transform.transform.translation.x = pose.translation.x();
        transform.transform.translation.y = pose.translation.y();
        transform.transform.translation.z = pose.translation.z();

        transform.child_frame_id = "world/drone_localizer";
        droneTfBroadcaster.sendTransform(transform);

        Eigen::Vector3d inverseRotatedPoint = pose.rotation.inverse() * Eigen::Vector3d({0, 1, 0});
        geometry_msgs::PointStamped point;
        point.header = header;
        point.header.frame_id = "world/drone_localizer";
        point.point.x = inverseRotatedPoint.x();
        point.point.y = inverseRotatedPoint.y();
        point.point.z = inverseRotatedPoint.z();

        pointPublisher.publish(point);

    }

public:
    PublishLocalizerOutput(ros::NodeHandle nh, std::string localizationSource) {
        localizer = LocalizationSensor::getLocalizationSensor(nh, localizationSource);
        timer = nh.createTimer(ros::Duration(1 / 60.0), &PublishLocalizerOutput::pushNewFrame, this);
        pointPublisher = nh.advertise<geometry_msgs::PointStamped>("/test_failsafe/velocity_vector_point", 1, false);
    }
};

int main(int argc, char* argv[]) {
    if (argc < 2) {
        ROS_ERROR("usage: rosrun rm_stuff phasespace_to_rviz <localization source>");
        return 1;
    }

    ros::init(argc, argv, "phasespace_markers_to_rviz");
    ros::NodeHandle nh;


    PhasespaceToRVizPipe rvizPublisher(nh);
    PublishLocalizerOutput localizerPublisher(nh, std::string(argv[1]));
    ros::spin();

    return 0;
}