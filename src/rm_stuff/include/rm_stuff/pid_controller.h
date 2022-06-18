#ifndef PID_CONTROLLER_H__
#define PID_CONTROLLER_H__

#include<iostream>
#include<functional>
#include<cmath>
#define _USE_MATH_DEFINES

namespace pid {

inline double defaultErrorCalculator(double setpoint, double measurement) {
    return setpoint - measurement;
}

/**
 * Calculate the error between a setpoint angle and a measured angle, assuming that
 * we want things to be in the range [-pi, pi]
 */
inline double angularErrorCalculator(double setpointRadians, double measurementRadians) {
    double error = setpointRadians - measurementRadians;
    return std::fmod(error + M_PI, 2 * M_PI) - M_PI;
}

struct PIDGains
{
    double kP;
    double kI;
    double kD;
    double integratorMin;
    double integratorMax;

    PIDGains(
        double kP,
        double kI,
        double kD,
        double integratorMin = 0.0,
        double integratorMax = 0.0)
        : kP(kP),
          kI(kI),
          kD(kD),
          integratorMin(integratorMin),
          integratorMax(integratorMax) {}
    friend std::ostream& operator<<(std::ostream& os, const PIDGains& gains);
};

class PIDController
{
private:
    const PIDGains pidGains_;
    double lastError_;
    double errorIntegral_;

    double setpoint_;

    std::function<double(double, double)> errorCalculator_;

public:
    PIDController(PIDGains pidGains, double setpoint, std::function<double(double, double)> errorCalculator = &defaultErrorCalculator)
        : pidGains_(pidGains),
          setpoint_(setpoint),
          lastError_(0),
          errorIntegral_(0),
          errorCalculator_(errorCalculator) {}

    double update(double newActual, double dt);
    void setSetpoint(double setpoint) { setpoint_ = setpoint; }
    double getSetpoint() const { return setpoint_; };
};

};

#endif // PID_CONTROLLER_H__