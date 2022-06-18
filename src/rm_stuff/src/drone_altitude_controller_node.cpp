
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include "rm_stuff/pid_controller.h"
#include "rm_stuff/common.h"
#include <dji_sdk/dji_drone.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>

using pid::PIDController;
using pid::PIDGains;

class Metrics
{
private:
    ros::Publisher velPublisher;
    ros::Publisher accPublisher;

    ros::Publisher dronePosePublisher;
    ros::Publisher setpointPublisher;
    ros::Publisher errorPublisher;

    static std_msgs::Float64 toMsg(double val)
    {
        std_msgs::Float64 ret;
        ret.data = val;
        return ret;
    }

public:
    Metrics(ros::NodeHandle nh)
    {

        velPublisher = nh.advertise<std_msgs::Float64MultiArray>("test_failsafe/velocity", 1, false);
        accPublisher = nh.advertise<std_msgs::Float64MultiArray>("test_failsafe/acceleration", 1, false);

        dronePosePublisher = nh.advertise<geometry_msgs::Pose>("test_failsafe/drone_pose", 1, true);
        errorPublisher = nh.advertise<std_msgs::Float64>("test_failsafe/error", 1, true);
        setpointPublisher = nh.advertise<std_msgs::Float64>("test_failsafe/setpoint", 1, true);
    }

    void logVel(double maxVel, double desiredVel, double limitedVel, double actualVel)
    {
        std_msgs::Float64MultiArray msg;

        msg.data.push_back(maxVel);
        msg.data.push_back(desiredVel);
        msg.data.push_back(limitedVel);
        msg.data.push_back(actualVel);

        velPublisher.publish(msg);
    }

    void logAcc(double maxAcc, double desiredAcc, double limitedAcc, double actualAcc)
    {
        std_msgs::Float64MultiArray msg;

        msg.data.push_back(maxAcc);
        msg.data.push_back(desiredAcc);
        msg.data.push_back(limitedAcc);
        msg.data.push_back(actualAcc);

        accPublisher.publish(msg);
    }
    void logPose(Pose pose) { dronePosePublisher.publish(pose.toMsg()); }
    void logError(double error) { errorPublisher.publish(toMsg(error)); }
    void logSetpoint(double setPoint) { setpointPublisher.publish(toMsg(setPoint)); }
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "custom_controller_rm");
    ros::NodeHandle nh;
    if (argc < 2)
    {
        ROS_INFO("usage: rosrun rm_stuff drone_altitude_controller <localization source>");
        return 1;
    }

    DJIDrone drone(nh);
    Metrics logger(nh);

    // figure out if we should be using phasespace or dji simulator for localization
    std::string localizationSource(argv[1]);
    std::unique_ptr<LocalizationSensor> actualPositionGetter = LocalizationSensor::getLocalizationSensor(nh, localizationSource);

    // drone.request_sdk_permission_control();
    drone.takeoff();

    double kP = 0.3, kI = 0.0, kD = 50.0, setpoint = 2.0;

    nh.getParam("/drone_altitude_controller/kP", kP);
    nh.getParam("/drone_altitude_controller/kI", kI);
    nh.getParam("/drone_altitude_controller/kD", kD);
    nh.getParam("/drone_altitude_controller/height", setpoint);

    ROS_INFO_STREAM("kP: " << kP << ", kI: " << kI << ", kD: " << kD << ", setpoint: " << setpoint);

    ROS_INFO("waiting for first localization reading ...");
    actualPositionGetter->blockUntilFirstReading();

    PIDController pidController(PIDGains(kP, kI, kD, 0, 0), setpoint);
    // PIDController pidController(PIDGains(500000000.0, 0, 0, 0, 0), 10);
    ros::Time lastTime = ros::Time::now();
    // last time we were able to send a command
    ros::Time lastUnsaturatedTime = ros::Time::now();
    bool wasSaturatedForTooLong = false;

    // todo
    // ✅ turn off rotation too close/far checks
    // ✅ if saturates maxspeed/max acceleration for 3 seconds, shut off
    // 4 graphs
    //   - velocity over time
    //   - acceleration over time
    //   - xyz
    //   - theta
    // roslaunch with parameters

    double maxAcceleration = 1.0; // meters / second^2
    double maxSpeed = 0.6;        // meters / second
    Eigen::Vector3d lastCommand;

    ROS_INFO("starting main loop!");

    bool dataWasPreviouslyFresh = true;

    ros::Rate loopTimer(60);

    while (ros::ok())
    {
        const auto now = ros::Time::now();
        const double dt = (now - lastTime).toSec();

        const bool newlySaturatedForTooLong = (now - lastUnsaturatedTime).toSec() > 1.0;
        wasSaturatedForTooLong = wasSaturatedForTooLong || newlySaturatedForTooLong;

        Eigen::Vector3d bodyFrameVelocityVector({0.0, 0.0, 0.0});
        // If data is stale, we want to stop the drone -> have 0 velocity
        // Or, stop drone if controller has been trying to make drone go too fast
        // Else, use PID controller
        // if (true)
        // {
        const auto pose = actualPositionGetter->getPose();
        const double newOutput = pidController.update(pose.translation(2), dt);
        const Eigen::Vector3d desiredWorldFrameVelocityVector = {0.0, 0.0, newOutput};
        // pose.rotation maps vectors from body frame to world frame, so inverse will do opposite
        bodyFrameVelocityVector = pose.rotation.inverse() * desiredWorldFrameVelocityVector;
        dataWasPreviouslyFresh = true;
        // }
        // else if (dataWasPreviouslyFresh)
        // {
        //     ROS_WARN("data is stale!");
        //     dataWasPreviouslyFresh = false;
        // }
        // else if (newlySaturatedForTooLong)
        // {
        //     ROS_WARN("saturated for too long");
        // }

        const auto accelerationVector = ((bodyFrameVelocityVector - lastCommand) / dt);

        const double desiredVel = bodyFrameVelocityVector.norm();
        const double desiredAcc = accelerationVector.norm();

        bool isControllerBeingLimited = false;

        // Limit acceleration
        if (desiredAcc > maxAcceleration)
        {
            ROS_WARN_STREAM("limiting acceleration! tried to accelerate at " << desiredAcc << " m/s^2, limited to " << maxAcceleration);
            const auto limitedVelocityDelta = (maxAcceleration * accelerationVector.normalized()) * dt;
            bodyFrameVelocityVector = lastCommand + limitedVelocityDelta;
            isControllerBeingLimited = true;
        }

        const double limitedAcc = ((bodyFrameVelocityVector - lastCommand) / dt).norm();
        // Limit velocity
        if (bodyFrameVelocityVector.norm() > maxSpeed)
        {
            ROS_WARN_STREAM("limiting velocity! tried to go at " << bodyFrameVelocityVector.norm() << " m/s, limited to " << maxSpeed);
            bodyFrameVelocityVector = maxSpeed * bodyFrameVelocityVector.normalized();
            isControllerBeingLimited = true;
        }

        const double limitedVel = bodyFrameVelocityVector.norm();

        logger.logSetpoint(pidController.getSetpoint());
        logger.logPose(actualPositionGetter->getPose());
        logger.logAcc(maxAcceleration, desiredAcc, limitedAcc, actualPositionGetter->getAcc().norm());
        logger.logVel(maxSpeed, desiredVel, limitedVel, actualPositionGetter->getVel().norm());

        if (!isControllerBeingLimited)
        {
            lastUnsaturatedTime = now;
        }

        drone.velocity_control(
            0,
            bodyFrameVelocityVector(0),
            bodyFrameVelocityVector(1),
            bodyFrameVelocityVector(2),
            0.0);

        lastCommand = bodyFrameVelocityVector;

        ros::spinOnce();
        loopTimer.sleep();
    }

    drone.landing();
    ros::spinOnce();

    return 0;
}