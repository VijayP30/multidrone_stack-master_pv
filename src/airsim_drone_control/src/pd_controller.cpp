#include "airsim_drone_control/pd_controller.h"

double pd::PDController::update(double newActual, double dt)
{
    const double error = setpoint_ - newActual;
    const double errorDelta = (error - lastError_) / dt;

    const double output = (pidGains_.kP * error) +
                          (pidGains_.kD * errorDelta);

    lastError_ = error;
    return output;
}
