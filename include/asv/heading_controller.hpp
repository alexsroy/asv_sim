#pragma once

namespace asv {

// Wrap an angle to [-pi, pi].
double wrap_pi(double angle);

struct PidParams {
    double kp = 1.0;
    double ki = 0.0;
    double kd = 2.0;
    double delta_max = 0.6;  // rudder limit [rad], about 34 degrees
};

class HeadingController {
public:
    explicit HeadingController(PidParams params = {});

    // Returns the rudder command [rad]. r is the measured yaw rate [rad/s].
    double update(double psi_ref, double psi, double r, double dt);

    void reset();

private:
    PidParams p_;
    double integral_ = 0.0;
};

}  // namespace asv