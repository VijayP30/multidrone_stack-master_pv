#ifndef RM_STUFF_COMMON_H__
#define RM_STUFF_COMMON_H__

#include <Eigen/Dense>
#include <ros/ros.h>
#include <phasespace/Rigid.h>
#include <phasespace/Rigids.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <memory>
#include <roscpp/Empty.h>

/// Pose should be in meters with z axis up
struct Pose
{
    ros::Time time;
    Eigen::Vector3d translation;
    Eigen::Quaterniond rotation;

    Pose(ros::Time time, Eigen::Vector3d t, Eigen::Quaterniond r)
        : time(time), translation(t), rotation(r)
    {
    }

    geometry_msgs::Pose toMsg() const
    {
        geometry_msgs::Pose ret;

        ret.position.x = translation(0);
        ret.position.y = translation(1);
        ret.position.z = translation(2);

        ret.orientation.w = rotation.w();
        ret.orientation.x = rotation.x();
        ret.orientation.y = rotation.y();
        ret.orientation.z = rotation.z();

        return ret;
    }
};

bool arePosesTooSimilar(double dt, Pose oldReading, Pose newReading);
bool arePosesTooDisparate(double dt, Pose oldReading, Pose newReading);

/// Abstract class that knows where the drone is
/// The idea is that the constructor will subscribe to a topic, where the
/// callback is a class method which calls ReceiveUpdatedData
class LocalizationSensor
{
private:
    Pose prevPose;
    Eigen::Vector3d measuredVel;
    Eigen::Vector3d measuredAcc;
    bool dataIsStale = false;

protected:
    /// Update our pose, checking that it does not violate our data read failure conditions
    /// Specifically, that
    ///     1. delta between poses should not be too high -- drone should not
    ///        move that fast
    ///     2. poses should not be too close -- repeated data means phasespace
    ///        has not updated for some reason
    void RecieveUpdatedData(Pose newPose);

public:
    LocalizationSensor() : prevPose(ros::Time::now(), {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0})
    {
    }

    virtual void blockUntilFirstReading() = 0;

    Pose getPose()
    {
        return prevPose;
    }

    Eigen::Vector3d getVel()
    {
        return measuredVel;
    }

    Eigen::Vector3d getAcc()
    {
        return measuredAcc;
    }

    bool isDataStale();
    
    static std::unique_ptr<LocalizationSensor> getLocalizationSensor(ros::NodeHandle nh, std::string& localizationSource) ;
};


/// LocalizationSensor subclass that reads data from the Phasespace
class PhasespaceLocalizer : public LocalizationSensor
{
private:
    ros::Subscriber subscriberHandle;

    void update(phasespace::Rigids rigids);

public:
    PhasespaceLocalizer(ros::NodeHandle nh)
    {
        subscriberHandle = nh.subscribe<phasespace::Rigids>("phasespace_rigids", 10, &PhasespaceLocalizer::update, this);
    }

    void blockUntilFirstReading() override
    {
        ros::topic::waitForMessage<phasespace::Rigids>("phasespace_rigids");
    }
};


/// LocalizationSensor that reads from the DJI simulation odometry
class SimulatorLocalizer : public LocalizationSensor
{
private:
    enum State
    {
        NORMAL,
        STOP_READING,
        REPEAT_READING
    };

    ros::Subscriber subscriberHandle;
    std::vector<ros::ServiceServer> servers;
    SimulatorLocalizer::State state = NORMAL;

    void update(nav_msgs::Odometry odometry);

    bool setNormalState(roscpp::EmptyRequest &req, roscpp::EmptyResponse &res)
    {
        state = NORMAL;
        return true;
    }
    bool setStopReadingState(roscpp::EmptyRequest &req, roscpp::EmptyResponse &res)
    {
        state = STOP_READING;
        return true;
    }
    bool setRepeatReadingState(roscpp::EmptyRequest &req, roscpp::EmptyResponse &res)
    {
        state = REPEAT_READING;
        return true;
    }

public:
    SimulatorLocalizer(ros::NodeHandle nh)
    {
        subscriberHandle = nh.subscribe<nav_msgs::Odometry>("dji_sdk/odometry", 10, &SimulatorLocalizer::update, this);
        servers.push_back(nh.advertiseService("test_failsafe/set_normal_state", &SimulatorLocalizer::setNormalState, this));
        servers.push_back(nh.advertiseService("test_failsafe/set_stop_reading_state", &SimulatorLocalizer::setStopReadingState, this));
        servers.push_back(nh.advertiseService("test_failsafe/set_repeat_reading_state", &SimulatorLocalizer::setStopReadingState, this));
    }

    void blockUntilFirstReading() override
    {
        ros::topic::waitForMessage<nav_msgs::Odometry>("dji_sdk/odometry");
    }
};


#endif // RM_STUFF_COMMON_H__
