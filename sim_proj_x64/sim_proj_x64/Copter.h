#pragma once

#ifdef COPTER_EXPORTS
#define COPTER_API __declspec(dllexport)
#else
#define COPTER_API __declspec(dllimport)
#endif

//=================== Copter =======================================

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

// Copter class
class COPTER_API Copter
{
public:
    Copter();
    ~Copter();

    void step(double motor0, double motor1, double motor2, double motor3, double dt);

    // Internal estimator
    AttitudeEstimator getAttitudeEstimator() const;

    // Get real sensor values
    Vector3 getGyro() const;
    Vector3 getAccel() const;
    Vector3 getMagnetometer() const;

    // Get and print state
    Vector3 getPosition() const;
    Vector3 getVelocity() const;
    void printState() const;

private:
    struct Impl;
    Impl* impl;
    Copter(const Copter&) = delete;
    Copter& operator=(const Copter&) = delete;
};

//=================== End of Copter =======================================

// Note: Nose points toward positive Y axis, right side toward positive X axis, upward vertical direction is positive Z axis
//
// World coordinate system is identical to body coordinate system
//
// Motors are: front-left 0, front-right 1, rear-left 2, rear-right 3
