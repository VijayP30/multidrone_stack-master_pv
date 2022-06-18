#ifndef PID_CONTROLLER_H__
#define PID_CONTROLLER_H__

namespace pd {

struct PDGains
{
    double kP;
    double kD;

    PDGains(
        double kP,
        double kD)
        : kP(kP),
          kD(kD) {}
};

class PDController
{
private:
    const PDGains pidGains_;
    double lastError_;

    double setpoint_;

public:
    PDController(PDGains pidGains, double setpoint)
        : pidGains_(pidGains),
          setpoint_(setpoint),
          lastError_(0) {}

    double update(double newActual, double dt);
    void setSetpoint(double setpoint) { setpoint_ = setpoint; }
    double getSetpoint() const { return setpoint_; };
};

};

#endif // PID_CONTROLLER_H__