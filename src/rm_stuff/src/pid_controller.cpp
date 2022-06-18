#include "rm_stuff/pid_controller.h"

static double clamp(double value, double low, double high)
{
    if (value < low)
    {
        return low;
    }
    else if (value > high)
    {
        return high;
    }
    else
    {
        return value;
    }
}

double pid::PIDController::update(double newActual, double dt)
{
    const double error = errorCalculator_(setpoint_, newActual);
    const double errorDelta = (error - lastError_) / dt;
    errorIntegral_ = errorIntegral_ + (error * dt); 
    // clamp(errorIntegral_ + (error * dt),
    //                        pidGains_.integratorMin,
    //                        pidGains_.integratorMax);

    const double output = (pidGains_.kP * error) +
                          (pidGains_.kI * errorIntegral_) +
                          (pidGains_.kD * errorDelta);

    lastError_ = error;
    return output;
}

std::ostream& pid::operator<<(std::ostream& os, const pid::PIDGains& gains)
{
    os << "PIDGains{ kP = " << gains.kP << ", kI = " << gains.kI << ", kD = " << gains.kD << " }";
    return os;
}
