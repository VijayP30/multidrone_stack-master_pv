#include <ros/ros.h>
#include "rm_stuff/common.h"
#include "rm_stuff/pid_controller.h"
#include <dji_sdk/dji_drone.h>

#include <deque>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <rm_stuff/YawInfo.h>
#include <rm_stuff/LimitedVector3.h>
#include <rm_stuff/FollowPath.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>

static rm_stuff::LimitedVector3 makeMsg(Eigen::Vector3d desired,
                                        Eigen::Vector3d limited,
                                        double limit)
{
    rm_stuff::LimitedVector3 ret;

    ret.desired.x = desired.x();
    ret.desired.y = desired.y();
    ret.desired.z = desired.z();

    ret.actual.x = limited.x();
    ret.actual.y = limited.y();
    ret.actual.z = limited.z();

    ret.desiredMagnitude = desired.norm();
    ret.actualMagnitude = limited.norm();
    ret.limit = limit;

    return ret;
}

static double yawFromQuat(Eigen::Quaterniond quat) {
    tf2::Quaternion quat2(quat.x(), quat.y(), quat.z(), quat.w());
    return tf2::getYaw(quat2);
}

static double yawFromQuat(geometry_msgs::Quaternion quat) {
    return tf2::getYaw(quat);
}

class Metrics
{
private:
    ros::Publisher velPublisher;
    ros::Publisher accPublisher;
    ros::Publisher trackingErrorPublisher;
    ros::Publisher setpointPublisher;
    ros::Publisher posePublisher;
    ros::Publisher yawPublisher;
    ros::Publisher desiredVsActualPublisher;

public:
    Metrics(ros::NodeHandle nh)
    {
        velPublisher = nh.advertise<rm_stuff::LimitedVector3>("test_failsafe/velocity", 1, false);
        accPublisher = nh.advertise<rm_stuff::LimitedVector3>("test_failsafe/acceleration", 1, false);
        yawPublisher = nh.advertise<rm_stuff::YawInfo>("test_failsafe/yaw", 1, false);

        trackingErrorPublisher = nh.advertise<geometry_msgs::Vector3>("test_failsafe/error", 1, false);
        setpointPublisher = nh.advertise<geometry_msgs::Vector3>("test_failsafe/setpoint", 1, false);
        posePublisher = nh.advertise<geometry_msgs::Pose>("test_failsafe/pose", 1, false);
        desiredVsActualPublisher = nh.advertise<rm_stuff::LimitedVector3>("test_failsafe/desired_vs_actual", 1, false);
    }

    void logYaw(double desiredYaw, double actualYaw, double desiredYawVelocity, double actualYawVelocity) {
        rm_stuff::YawInfo msg;
        msg.actualYaw = actualYaw;
        msg.desiredYaw = desiredYaw;
        msg.yawError = desiredYaw - actualYaw;
        msg.yawCommand = desiredYawVelocity;
        msg.yawCommandLimited = actualYawVelocity;

        yawPublisher.publish(msg);
    }

    void logVelVectors(Eigen::Vector3d desired, Eigen::Vector3d limited, double limit)
    {
        velPublisher.publish(makeMsg(desired, limited, limit));
    }

    void logAccVectors(Eigen::Vector3d desired, Eigen::Vector3d limited, double limit)
    {
        accPublisher.publish(makeMsg(desired, limited, limit));
    }

    void logTrackingError(Eigen::Vector3d desiredPosition, Eigen::Vector3d actualPosition)
    {
        auto error = desiredPosition - actualPosition;
        geometry_msgs::Vector3 errorMsg;
        errorMsg.x = error.x();
        errorMsg.y = error.y();
        errorMsg.z = error.z();

        trackingErrorPublisher.publish(errorMsg);

        rm_stuff::LimitedVector3 desiredVsActual;
        desiredVsActual.desired.x = desiredPosition.x();
        desiredVsActual.desired.y = desiredPosition.y();
        desiredVsActual.desired.z = desiredPosition.z();

        desiredVsActual.actual.x = actualPosition.x();
        desiredVsActual.actual.y = actualPosition.y();
        desiredVsActual.actual.z = actualPosition.z();
        desiredVsActualPublisher.publish(desiredVsActual);
    }

    void logPose(Pose pose)
    {
        posePublisher.publish(pose.toMsg());
    }
};


struct Setpoint {
    Eigen::Vector3d location;
    double yaw;

    Setpoint(double x, double y, double z, double yaw) : location((Eigen::Vector3d() << x, y, z).finished()), yaw(yaw) {}
    Setpoint(Eigen::Vector3d loc, double y) : location(loc), yaw(y) {}

};

std::ostream& operator<<(std::ostream& os, const Setpoint& setpoint) {
    return os << "Setpoint [" << setpoint.location.x() << ", " << setpoint.location.y() << ", " << setpoint.location.z() << "]" << " yaw " << setpoint.yaw;
}

class PDControllerNode
{
private:
    bool enabled = true;

    std::unique_ptr<LocalizationSensor> localizationSensor;

    std::deque<Setpoint> setpoints; // path that we should follow

    DJIDrone drone;

    pid::PIDController pidX;
    pid::PIDController pidY;
    pid::PIDController pidZ;
    pid::PIDController pidYaw;

    Eigen::Vector3d lastVelocityCommand;

    Metrics log;

    ros::Timer timer;
    ros::ServiceServer pathServiceServer;

    Eigen::Vector3d calculateNewDesiredVelocity(double dt)
    {
        auto pose = localizationSensor->getPose();
        Eigen::Vector3d output{
            pidX.update(pose.translation.x(), dt),
            pidY.update(pose.translation.y(), dt),
            pidZ.update(pose.translation.z(), dt)};
        // transform output velocity vector from world frame to drone body frame
        return pose.rotation.inverse() * output;
    }

    void updateDroneVelocityCommand(const ros::TimerEvent &timer)
    {
        log.logPose(localizationSensor->getPose());
        log.logTrackingError(Eigen::Vector3d { pidX.getSetpoint(), pidY.getSetpoint(), pidZ.getSetpoint() }, localizationSensor->getPose().translation);
        static const double maxAcceleration = 1.0;
        static const double maxSpeedZ = .3;
        static const double maxSpeedXY = .3;

        double dt = (timer.current_real - timer.last_real).toSec();

        Eigen::Vector3d nonLimitedVel{0.0, 0.0, 0.0}; // default to stopping the drone
        if (enabled)
        {
            nonLimitedVel = calculateNewDesiredVelocity(dt);
        }

        // Limit acceleration
        const auto desiredAcc = ((nonLimitedVel - lastVelocityCommand) / dt);
        if (desiredAcc.norm() > maxAcceleration)
        {
            ROS_WARN_STREAM("limiting acceleration! tried to accelerate at " << desiredAcc << " m/s^2, limited to " << maxAcceleration);
            const auto limitedVelocityDelta = (maxAcceleration * desiredAcc.normalized()) * dt;
            nonLimitedVel = lastVelocityCommand + limitedVelocityDelta;
            // isControllerBeingLimited = true;
        }
        const auto limitedAcc = ((nonLimitedVel - lastVelocityCommand) / dt);
        log.logAccVectors(desiredAcc, limitedAcc, maxAcceleration);

        // Limit velocity
        Eigen::Vector3d limitedVel = nonLimitedVel;
        if (std::abs(limitedVel.z()) > maxSpeedZ)
        {
            ROS_WARN_STREAM("limiting velocity (z)!");
            limitedVel.z() = std::copysign(maxSpeedZ, limitedVel.z());
        }
        if (std::abs(limitedVel.x()) > maxSpeedXY) {
            ROS_WARN_STREAM("limiting velocity (x)!");
            limitedVel.x() = std::copysign(maxSpeedXY, limitedVel.x());
        }
        if (std::abs(limitedVel.y()) > maxSpeedXY) {
            ROS_WARN_STREAM("limiting velocity (y)!");
            limitedVel.y() = std::copysign(maxSpeedXY, limitedVel.y());
        }

        log.logVelVectors(nonLimitedVel, limitedVel, maxSpeedZ);

        double currentYaw = yawFromQuat(localizationSensor->getPose().rotation);
        double yawCmd = -pidYaw.update(
            currentYaw, dt
        );
        log.logYaw(pidYaw.getSetpoint(), currentYaw, yawCmd, yawCmd);

        lastVelocityCommand = limitedVel;
        drone.velocity_control(
            0,
            limitedVel(0),
            -limitedVel(1),
            limitedVel(2),
            yawCmd);

        if (currentSetpointAchieved() && setpoints.size() > 0) {
            ROS_INFO("proceeding to next setpoint !! ");
            proceedToNextSetpoint();
        }
    }

    bool currentSetpointAchieved() {
        double currentYaw = yawFromQuat(localizationSensor->getPose().rotation);
        double yawError = std::abs(pidYaw.getSetpoint() - currentYaw);

        double locationError = (localizationSensor->getPose().translation - (Eigen::Vector3d() << pidX.getSetpoint(), pidY.getSetpoint(), pidZ.getSetpoint()).finished()).norm();

        return yawError < 0.1 && locationError < 0.10; // must be within 0.1 radians + 10 cm of target pose
    }

    void proceedToNextSetpoint() {
        auto setpoint = setpoints.at(0); 
        ROS_INFO_STREAM("going to setpoint " << setpoint);
        setpoints.pop_front();

        pidX.setSetpoint(setpoint.location.x());
        pidY.setSetpoint(setpoint.location.y());
        pidZ.setSetpoint(setpoint.location.z());
        pidYaw.setSetpoint(setpoint.yaw);
    }

    bool handlePathRequest(rm_stuff::FollowPath::Request &req, rm_stuff::FollowPath::Response &response) {
        setpoints.clear();
        ROS_INFO_STREAM("received new setpoints");
        for (auto& desiredPose : req.path.poses) {
            setpoints.emplace_back(
                desiredPose.pose.position.x,
                desiredPose.pose.position.y,
                desiredPose.pose.position.z,
                yawFromQuat(desiredPose.pose.orientation)
            );
            ROS_INFO_STREAM(setpoints.back());
        }

        return true;
    }

public:
    PDControllerNode(ros::NodeHandle nh,
                     std::unique_ptr<LocalizationSensor> lS,
                     Eigen::Vector3d desiredLocation,
                     double desiredYaw,
                     pid::PIDGains xyGains,
                     pid::PIDGains zGains,
                     pid::PIDGains yawGains)
        : localizationSensor(std::move(lS)),
          drone(nh),
          pidX(xyGains, desiredLocation.x()),
          pidY(xyGains, desiredLocation.y()),
          pidZ(zGains, desiredLocation.z()),
          pidYaw(yawGains, desiredYaw, &pid::angularErrorCalculator),
          log(nh)
    {
        // ROS_INFO_STREAM("current setpoint: " << currentSetpoint.location << ", yaw: " << currentSetpoint.yaw);
        ROS_INFO_STREAM("setpoints x " << pidX.getSetpoint() << " y " << pidY.getSetpoint() << " z " << pidZ.getSetpoint() << " yaw " << pidYaw.getSetpoint());
        pathServiceServer = nh.advertiseService("/drone/followPath", &PDControllerNode::handlePathRequest, this);
        timer = nh.createTimer(ros::Duration(1 / 60.0), &PDControllerNode::updateDroneVelocityCommand, this);
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "drone_lateral_pd_controller");
    ros::NodeHandle nh;

    if (argc < 2)
    {
        ROS_INFO("usage: rosrun rm_stuff drone_altitude_controller <localization source>");
        return 1;
    }
    std::string sourceName(argv[1]);
    std::unique_ptr<LocalizationSensor> localizationSource = LocalizationSensor::getLocalizationSensor(nh, sourceName);

    DJIDrone drone(nh);

    pid::PIDGains xyGains(0.3, 0.0, 0.0);
    pid::PIDGains zGains(0.3, 0.0, 0.0);
    pid::PIDGains yawGains(0.3, 0.0, 0.0);
    Eigen::Vector3d desiredLocation { 0.0, 2.0, 0.0 };
    double desiredYaw = 0;

    nh.getParam("/drone_position_controller/xy/kP", xyGains.kP);
    nh.getParam("/drone_position_controller/xy/kD", xyGains.kD);

    nh.getParam("/drone_position_controller/z/kP", zGains.kP);
    nh.getParam("/drone_position_controller/z/kD", zGains.kD);

    nh.getParam("/drone_position_controller/yaw/kP", yawGains.kP);
    nh.getParam("/drone_position_controller/yaw/kD", yawGains.kD);

    nh.getParam("/drone_position_controller/desiredX", desiredLocation.x());
    nh.getParam("/drone_position_controller/desiredY", desiredLocation.y());
    nh.getParam("/drone_position_controller/desiredZ", desiredLocation.z());
    nh.getParam("/drone_position_controller/desiredYaw", desiredYaw);

    ROS_INFO_STREAM("XY gains: " << xyGains);
    ROS_INFO_STREAM("Z gains: " << zGains);
    ROS_INFO_STREAM("desiredLocation: " << desiredLocation);
    ROS_INFO("waiting for first localization reading ...");
    localizationSource->blockUntilFirstReading();

    ROS_INFO("starting controller . .. ");
    PDControllerNode node(nh, std::move(localizationSource), desiredLocation, desiredYaw, xyGains, zGains, yawGains);
    ros::spin(); 

    return 0;
}