#include "Copter.h"
#include "algorithm"
#include <iostream>
#include <thread>

// This program simulates a quadcopter flying to a target position (x=200, y=200, z=50)
// using PID controllers for position and attitude.

Copter copter;

// ------------------- PID -------------------
Pid zPid(0.5, 0.000, 1.5);
Pid xPosPid(0.4, 0.000, 2, -0.4, 0.4, -0.4, 0.4);
Pid yPosPid(0.4, 0.000, 2, -0.4, 0.4, -0.4, 0.4);
Pid rollPid(4.0, 0.0, 2.0, -5, 5, -5, 5);
Pid pitchPid(4.0, 0.0, 2.0, -5, 5, -5, 5);
Pid yawPid(1, 0.0, 1.0, -0.5, 0.5, -0.5, 0.5);

// ------------------- Main program -------------------
int main() {
    double mass = 1.0;
    double g = 9.81;
	double dt = copter.getStepSize();

    double target_z = 50;
    double target_x = 200;
    double target_y = 200;
    double target_yaw = 0;

    for (int i = 0; i < 6000; i++) // Trial version limited to 5000 steps
    {

        /* ================= Controller ================= */

        Vector3 pos = copter.getPosition();
        double thrust = zPid.update(target_z, pos.z) + mass * g;

        double target_pitch = yPosPid.update(target_y, pos.y);
        double target_roll = xPosPid.update(target_x, pos.x);

        AttitudeEstimator attitude = copter.getAttitudeEstimator();

        double tau_x = 0;
        double tau_y = 0;
        double tau_z = 0;

        tau_x = pitchPid.update(target_pitch, attitude.pitch);
        tau_y = rollPid.update(target_roll, attitude.roll);
        tau_z = yawPid.update(target_yaw, attitude.yaw);

        thrust = std::max(0.0, thrust);

        /* ================= Motor mixing ================= */
        double mt0 = thrust / 4 - tau_x / 4 + tau_y / 4 + tau_z / 4;
        double mt1 = thrust / 4 - tau_x / 4 - tau_y / 4 - tau_z / 4;
        double mt2 = thrust / 4 + tau_x / 4 + tau_y / 4 - tau_z / 4;
        double mt3 = thrust / 4 + tau_x / 4 - tau_y / 4 + tau_z / 4;


        /* ================= Dynamics ================= */
        copter.step(mt0, mt1, mt2, mt3);

        /* ================= Print ================= */
        if (i % 10 == 0) {
            std::cout << "Time: " << i * dt << " s, ";
            copter.printState();
            //std::this_thread::sleep_for(std::chrono::milliseconds(100)); // can be used to control printing speed
        }
    }

    return 0;
}
