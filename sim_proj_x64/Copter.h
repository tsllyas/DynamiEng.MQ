#pragma once
#include <memory>
#include "Export.h"

struct COPTER_API Vector3
{
    double x;
    double y;
    double z;
};

struct COPTER_API AttitudeEstimator
{
    double roll;
    double pitch;
    double yaw;
};
//=================== Pid =======================================
class COPTER_API Pid {
public:
    Pid(double kp, double ki, double kd, double i_min = -20,
        double i_max = 20, double out_min = -20, double out_max = 20);
    ~Pid() noexcept;

    double update(double target, double meas) noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};
//=================== End of Pid =======================================
//Users can also customize PID access. The fixed step size of the model is 0.01 seconds

//=================== Copter =======================================
class COPTER_API Copter
{
public:
    Copter();
    ~Copter() noexcept;

    void step(double motor0, double motor1, double motor2, double motor3) noexcept;

    // Internal estimator
    AttitudeEstimator getAttitudeEstimator() const noexcept;

    // Get real sensor values
    Vector3 getGyro() const noexcept;
    Vector3 getAccel() const noexcept;
    Vector3 getMagnetometer() const noexcept;

    // Get and print state
    Vector3 getPosition() const noexcept;
    Vector3 getVelocity() const noexcept;
    double getStepSize() const noexcept;
    void printState() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};

//=================== End of Copter =======================================

// Note: Nose points toward positive Y axis, right side toward positive X axis, upward vertical direction is positive Z axis
//
// World coordinate system is identical to body coordinate system
//
// Motors are: front-left 0, front-right 1, rear-left 2, rear-right 3
