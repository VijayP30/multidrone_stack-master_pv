#include <ros/ros.h>
#include <vector>
#include <Eigen/Dense>

#include "airsim_drone_control/pd_controller.h"
#include "airsim_ros_pkgs/Takeoff.h"
#include "airsim_ros_pkgs/VelCmd.h"
#include "nav_msgs/Odometry.h"

static const double offset = 6;
static const double length = 3;
static const std::vector<Eigen::Vector3d> pointsToVisit = {
    Eigen::Vector3d{length/2 + offset, 0, -5},
    Eigen::Vector3d{-length/2 + offset, 0, -5}};

static Eigen::Vector3d vecFromPosePos(const nav_msgs::Odometry &odometry)
{
    return Eigen::Vector3d{odometry.pose.pose.position.x, odometry.pose.pose.position.y, odometry.pose.pose.position.z};
}

static Eigen::Quaterniond quatFromPoseQuat(const nav_msgs::Odometry &odometry)
{
    return Eigen::Quaterniond(
        odometry.pose.pose.orientation.w,
        odometry.pose.pose.orientation.x,
        odometry.pose.pose.orientation.y,
        odometry.pose.pose.orientation.z
    );
}

class TrajectoryFollower
{
private:
    // technically, this should also include velocity + yaw + yaw rate info
    const std::vector<Eigen::Vector3d> path;
    int currentNodeOnPath = 0;

    nav_msgs::Odometry lastOdometryMsg;
    ros::Time lastRun;

    ros::Subscriber odometrySubscriber;
    ros::Publisher velocityPublisher;
    ros::Timer controlLawTimer;

    pd::PDController xPidController;
    pd::PDController yPidController;
    pd::PDController zPidController;
    pd::PDController yawPidController;

    void setLastOdometryMsg(nav_msgs::Odometry msg)
    {
        ROS_INFO_THROTTLE(10, "still setting odometry");
        lastOdometryMsg = msg;
    }

    void recomputeOutput(const ros::TimerEvent& event)
    {
        ROS_INFO_THROTTLE(10, "still computing output . . .");

        const auto currentLocation = vecFromPosePos(lastOdometryMsg);
        const auto currentOrientation = quatFromPoseQuat(lastOdometryMsg);

        const auto rotatedVector = (currentOrientation * Eigen::Vector3d(1, 0, 0)).block<2, 1>(0, 0);
        const auto yaw = atan2(rotatedVector.y(), rotatedVector.x());
        

        if ((path.at(currentNodeOnPath) - currentLocation).norm() < 0.05) // within 5cm of point, move to next one
        {
            currentNodeOnPath = (currentNodeOnPath + 1) % path.size();
        }
        const auto &desiredLocation = path.at(currentNodeOnPath);

        const auto now = ros::Time::now();
        const double dt = (now - lastRun).toSec();

        xPidController.setSetpoint(desiredLocation.x());
        yPidController.setSetpoint(desiredLocation.y());
        zPidController.setSetpoint(desiredLocation.z());
        yawPidController.setSetpoint(0);

        {
            airsim_ros_pkgs::VelCmd velCmd;
            velCmd.twist.linear.x = xPidController.update(currentLocation.x(), dt);
            velCmd.twist.linear.y = yPidController.update(currentLocation.y(), dt);
            velCmd.twist.linear.z = zPidController.update(currentLocation.z(), dt);
            velCmd.twist.angular.z = yawPidController.update(yaw, dt);
            velocityPublisher.publish(velCmd);
        }

        lastRun = now;
    }

public:
    TrajectoryFollower(ros::NodeHandle nh,
                       std::vector<Eigen::Vector3d> trajectory,
                       pd::PDGains xyGains,
                       pd::PDGains zGains,
                       pd::PDGains yawGains
                       ) : path(trajectory),
                                              xPidController(xyGains, trajectory.at(0).x()),
                                              yPidController(xyGains, trajectory.at(0).y()),
                                              zPidController(zGains, trajectory.at(0).z()),
                                              yawPidController(yawGains, 0)
    {
        odometrySubscriber = nh.subscribe<nav_msgs::Odometry>("/airsim_node/drone_1/odom_local_ned", 1, &TrajectoryFollower::setLastOdometryMsg, this);
        velocityPublisher = nh.advertise<airsim_ros_pkgs::VelCmd>("/airsim_node/drone_1/vel_cmd_world_frame", 0);
        controlLawTimer = nh.createTimer(ros::Duration(0.1), &TrajectoryFollower::recomputeOutput, this);

        // wait for a message to come through
        ROS_INFO("constructing Trajectory follower -- waiting for first odometry message");
        ros::topic::waitForMessage<nav_msgs::Odometry>(odometrySubscriber.getTopic());
        ROS_INFO("got first odometry message!");
        lastRun = ros::Time::now();
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "move_drone_along_trajectory");

    ros::NodeHandle nh;
    // ros::ServiceClient takeOffServiceClient = nh.serviceClient<airsim_ros_pkgs::Takeoff>("/airsim_node/drone_1/takeoff");
    // airsim_ros_pkgs::Takeoff takeoffCall;
    // takeoffCall.request.waitOnLastTask = true;
    // takeOffServiceClient.call(takeoffCall);

    TrajectoryFollower trajFollower(
        nh,
        pointsToVisit,
        pd::PDGains(0.5, 0),
        pd::PDGains(0.5, 0),
        pd::PDGains(0.5, 0)
    );

    ros::spin();

    return 0;
}