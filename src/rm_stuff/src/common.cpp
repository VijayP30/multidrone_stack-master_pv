#include "rm_stuff/common.h"

/// expects Poses in meters, dt in seconds
bool arePosesTooSimilar(double dt, Pose oldReading, Pose newReading)
{
    static const double EPSILON = 1e-6; // Meters, 1e-6 meter = 0.001 millimeter = 1 micrometer
                                        // Natural noise in phasespace readings should exceed it

    const double posDelta = (newReading.translation - oldReading.translation).norm();
    const double quatDelta = newReading.rotation.angularDistance(oldReading.rotation);
    if (posDelta < EPSILON)
    {
        ROS_WARN("position too close");
        return true;
    }
    // else if (quatDelta < EPSILON) {
    //     ROS_WARN("rotation too close");
    //     return true;
    // }
    else
    {
        return false;
    }
}

// expects Poses in meters, dt in seconds
bool arePosesTooDisparate(double dt, Pose oldReading, Pose newReading)
{
    static const double MAX_EXPECTED_SPEED = 1000.0;  // meters / second
    static const double MAX_ANGULAR_VELOCITY = 0.5; // radians / second

    const double delta = (newReading.translation - oldReading.translation).norm();
    const double avgSpeed = delta / dt;
    const double avgAngVel = newReading.rotation.angularDistance(oldReading.rotation) / dt;
    if (delta >= 1e-3 && avgSpeed > MAX_EXPECTED_SPEED)
    {
        ROS_WARN_STREAM("perceived speed of " << avgSpeed << " is too high (max: " << MAX_EXPECTED_SPEED << ", oldPos: " << oldReading.translation << ", newPos: " << newReading.translation << "dt: " << dt << ")");
        return true;
    }
    // else if (avgAngVel > MAX_ANGULAR_VELOCITY) {
    //     ROS_WARN("perceived angular velocity too high");
    //     return true;
    // }
    else
    {
        return false;
    }
}

void LocalizationSensor::RecieveUpdatedData(Pose newPose)
{
    const double dtSecs = (newPose.time - prevPose.time).toSec();
    if ((!arePosesTooSimilar(dtSecs, prevPose, newPose) && !arePosesTooDisparate(dtSecs, prevPose, newPose)))
    {
        const Eigen::Vector3d delta = newPose.translation - prevPose.translation;

        Eigen::Vector3d currentMeasuredVel;
        currentMeasuredVel = delta / dtSecs;

        const Eigen::Vector3d velDelta = currentMeasuredVel - measuredVel;
        measuredAcc = velDelta / dtSecs;

        measuredVel = currentMeasuredVel;
        prevPose = newPose;
    }
}

bool LocalizationSensor::isDataStale()
{
    const ros::Time now = ros::Time::now();
    const double dtSecs = (now - prevPose.time).toSec();
    // dataIsStale |= dtSecs > 5.0;
    return dataIsStale;
}

std::unique_ptr<LocalizationSensor> LocalizationSensor::getLocalizationSensor(ros::NodeHandle nh, std::string& localizationSource) {
    std::unique_ptr<LocalizationSensor> actualPositionGetter;
    {
        if (localizationSource == "phasespace") // if using phasespace
        {
            ROS_INFO("Using localization data from phasespace");
            actualPositionGetter = std::unique_ptr<LocalizationSensor>(new PhasespaceLocalizer(nh));
        }
        else if (localizationSource == "dji_sdk")
        {
            ROS_INFO("Using localization data from DJI SDK");
            actualPositionGetter = std::unique_ptr<LocalizationSensor>(new SimulatorLocalizer(nh));
        }
        else
        {
            ROS_ERROR_STREAM("invalid localization source: '" << localizationSource << "'");
            return nullptr;
        }
    }
    return actualPositionGetter;
}

void PhasespaceLocalizer::update(phasespace::Rigids rigids)
{
    // Rotation that rotates vectors from phasespace axes (with Y up) to world axes (with Z up)
    static const Eigen::Quaterniond ROTATE_Z_UP(std::cos(M_PI_4), std::sin(M_PI_4), 0, 0);
    static const Eigen::Quaterniond ROTATE_Y_UP(std::cos(M_PI_4), -std::sin(M_PI_4), 0, 0);

    auto &selectedRigid = rigids.rigids[1];

    // pos comes from phasespace in millimeters
    Eigen::Vector3d pos({selectedRigid.x,
                            selectedRigid.y,
                            selectedRigid.z});

    // change to meters
    pos /= 1000.0;

    Eigen::Quaterniond rot = Eigen::Quaterniond({selectedRigid.qw,
                                                    selectedRigid.qx,
                                                    selectedRigid.qy,
                                                    selectedRigid.qz});

    // Change orientation to have z axis up
    this->RecieveUpdatedData(Pose(
        ros::Time(0, selectedRigid.time), // time comes in from phasespace node as unix nanos
        ROTATE_Z_UP * pos,
        ROTATE_Z_UP * rot * ROTATE_Y_UP));
}

void SimulatorLocalizer::update(nav_msgs::Odometry odometry)
{
    static const Eigen::Matrix3d NEGATE_Y = (Eigen::Matrix3d() <<
        1, 0, 0,
        0, -1, 0,
        0, 0, 1 
    ).finished();

    switch (state)
    {
    case NORMAL:
    {
        auto pose = odometry.pose.pose;
        // The DJI frame is apparently left handed with x-north, y-east, z-up
        // We need to convert it to right-handed x-north y-west z-up
        Eigen::Quaterniond quatDji {pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z};
        Eigen::Matrix3d quatMtxDji(quatDji);
        Eigen::Matrix3d quatMtxRightHanded = NEGATE_Y * quatMtxDji * NEGATE_Y;
        Eigen::Quaterniond quatRightHanded(quatMtxRightHanded);

        this->RecieveUpdatedData(Pose(
            odometry.header.stamp,
            {pose.position.x,
             -pose.position.y,
             pose.position.z},
            quatRightHanded));
        return;
    }
    case STOP_READING:
    {
        return;
    }
    case REPEAT_READING:
    {
        this->RecieveUpdatedData(getPose());
        return;
    }
    }
}
