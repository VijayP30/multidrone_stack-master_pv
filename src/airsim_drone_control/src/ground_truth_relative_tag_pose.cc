#include <ros/ros.h>
#include <Eigen/Dense>

#include <aprilslam/Apriltags.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Float64.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>  
#include <tf2_ros/transform_listener.h>

static Eigen::Vector3d vecFromOdometryPose(const nav_msgs::Odometry &pose) {
    return Eigen::Vector3d({pose.pose.pose.position.x, pose.pose.pose.position.y, pose.pose.pose.position.z });
}

static Eigen::Quaterniond quatFromOdometryPose(const nav_msgs::Odometry &pose) {
    return Eigen::Quaterniond(
        pose.pose.pose.orientation.w,
        pose.pose.pose.orientation.x,
        pose.pose.pose.orientation.y,
        pose.pose.pose.orientation.z
    );
}

class Processor {
private:
    tf2_ros::TransformBroadcaster tfBroadcaster;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    ros::Subscriber droneOdometrySubscriber;
    ros::Publisher groundTruthRelativeTagPosePublisher;
    ros::Subscriber apriltagsSubscriber;
    ros::Publisher errorTransformPublisher;
    ros::Publisher droneZPublisher;

    ros::Timer errorCalculationTimer;

    void receiveOdometryMsg(nav_msgs::Odometry odometry) {
        auto dronePos = vecFromOdometryPose(odometry);
        auto droneQuat = quatFromOdometryPose(odometry);

        Eigen::Vector3d tagPosWorldFrame({ 6.0, 0, -1.2 }); // north-east-down frame, similar to drone
        auto tagPosDroneFrame = droneQuat.inverse() * (tagPosWorldFrame - dronePos);

        geometry_msgs::Pose tagPose;
        tagPose.orientation.w = 1;
        tagPose.orientation.x = tagPose.orientation.y = tagPose.orientation.z = 0;
        tagPose.position.x = tagPose.position.y = tagPose.position.z = 0; 

        std_msgs::ColorRGBA color;
        color.r = 255;
        color.g = 255;
        color.b = 255;
        color.a = 255;

        visualization_msgs::Marker marker;
        marker.header.frame_id = "gt_tag";
        marker.header.stamp = odometry.header.stamp;
        marker.ns = "36h11 tag";
        marker.id = 7;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = marker.scale.y = 0.6;
        marker.scale.z = marker.scale.x / 10;
        marker.color = color;
        marker.pose = tagPose;

        groundTruthRelativeTagPosePublisher.publish(marker);

        std_msgs::Float64 height;
        height.data = -odometry.pose.pose.position.z;
        droneZPublisher.publish(height);
    }


    void receiveAprilTagMsg(aprilslam::Apriltags apriltags) {
        if (apriltags.apriltags.size() != 1) {
            ROS_WARN_STREAM("warning: observed " << apriltags.apriltags.size() << " apriltags (expected to see exactly 1)");
            if (apriltags.apriltags.size() == 0) {
                return;
            }
        }

        // at least 1 apriltag
        auto& apriltag = apriltags.apriltags.at(0);

        geometry_msgs::Pose tagPose = apriltag.pose;

        std_msgs::ColorRGBA color;
        color.r = 255;
        color.g = 0;
        color.b = 255;
        color.a = 255;

        visualization_msgs::Marker marker;
        marker.header.frame_id = "front_center_custom_optical";
        marker.header.stamp = apriltags.header.stamp;
        marker.ns = "36h11 tag (observed)";
        marker.id = 70;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = marker.scale.y = 0.6;
        marker.scale.z = marker.scale.x / 10;
        marker.color = color;
        marker.pose = tagPose;

        groundTruthRelativeTagPosePublisher.publish(marker);

        geometry_msgs::TransformStamped transformStamped;
  
        transformStamped.header.stamp = apriltags.header.stamp;
        transformStamped.header.frame_id = "front_center_custom_optical";
        transformStamped.child_frame_id = std::string("36h11_tag");
        transformStamped.transform.translation.x = tagPose.position.x;
        transformStamped.transform.translation.y = tagPose.position.y;
        transformStamped.transform.translation.z = tagPose.position.z;

        Eigen::Quaterniond tagOrientation = Eigen::Quaterniond( tagPose.orientation.w, tagPose.orientation.x, tagPose.orientation.y, tagPose.orientation.z );//.inverse();
        transformStamped.transform.rotation.w = tagOrientation.w();
        transformStamped.transform.rotation.x = tagOrientation.x();
        transformStamped.transform.rotation.y = tagOrientation.y();
        transformStamped.transform.rotation.z = tagOrientation.z();

        tfBroadcaster.sendTransform(transformStamped);
    }

    void calculateError(const ros::TimerEvent& timerEvent) {
        try {
            auto transform = tfBuffer.lookupTransform("36h11_tag", "gt_tag", ros::Time(0));
            ROS_INFO_STREAM_THROTTLE(3, "z error: " << transform.transform.translation.z);
            errorTransformPublisher.publish(transform);
        } catch (const tf2::LookupException& e) {
            // that's ok, we'll try again later
        } catch (const tf2::ExtrapolationException& e) {
            // that's ok, we'll try again later
        }
    }

public:
    Processor(ros::NodeHandle nh) : tfListener(tfBuffer) {
        droneOdometrySubscriber = nh.subscribe<nav_msgs::Odometry>("/airsim_node/drone_1/odom_local_ned", 1, &Processor::receiveOdometryMsg, this);
        groundTruthRelativeTagPosePublisher = nh.advertise<visualization_msgs::Marker>("/ground_truth_relative_drone_pos", 1);
        apriltagsSubscriber = nh.subscribe<aprilslam::Apriltags>("/camera/apriltags", 1, &Processor::receiveAprilTagMsg, this);
        errorCalculationTimer = nh.createTimer(ros::Duration(0.1), &Processor::calculateError, this);
        errorTransformPublisher = nh.advertise<geometry_msgs::TransformStamped>("/april_tag_localization_error", 1);
        droneZPublisher = nh.advertise<std_msgs::Float64>("/drone_z", 1);
    }
};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "ground_truth_relative_tag_pose");
    ros::NodeHandle nh;

    Processor aprilTagMsgProcessor(nh);
    ros::spin();

    return 0;
}