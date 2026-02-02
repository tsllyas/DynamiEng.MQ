#pragma once

class Pid {
private:
    double kp, ki, kd;
    double integral = 0;
    double prev_meas = 0;
    double i_min, i_max;
    double out_min, out_max;

public:
    Pid(double kp, double ki, double kd, double i_min = -20, double i_max = 20,
        double out_min = -20, double out_max = 20)
        :kp(kp), ki(ki), kd(kd),
        i_min(i_min), i_max(i_max),
        out_min(out_min), out_max(out_max) {
    }
    double update(double target, double meas, double dt) {
        auto clamp = [](double v, double lo, double hi) {
            if (v < lo) return lo;
            if (v > hi) return hi;
            return v;
            };
        if (dt <= 0) return 0;

        double err = target - meas;

        integral += err * dt;
        integral = clamp(integral, i_min, i_max);  

        double derivative = -(meas - prev_meas) / dt;
        prev_meas = meas;

        double out = kp * err + ki * integral + kd * derivative;
        return clamp(out, out_min, out_max);
    }
};