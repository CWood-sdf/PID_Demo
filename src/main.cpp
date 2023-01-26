/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       woodc                                                     */
/*    Created:      1/25/2023, 9:27:29 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include <iostream>
#include <cmath>
using std::cout;
using std::endl;
using std::flush;

using namespace vex;

// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain Brain;
// Inertial sensor
inertial Inertial = inertial(PORT1);
motor FL = motor(PORT2, ratio18_1, false);
motor FR = motor(PORT3, ratio18_1, true);
motor BL = motor(PORT4, ratio18_1, false);
motor BR = motor(PORT5, ratio18_1, true);

// Motor groups to make things easier
motor_group Left = motor_group(FL, BL);
motor_group Right = motor_group(FR, BR);

/**
 * @brief This function makes sure that the angle is between -180 and 180
 *
 * @param a The angle to limit
 * @return double
 */
double posNeg180(double a)
{
    while (a > 180)
    {
        a -= 360;
    }
    while (a <= -180)
    {
        a += 360;
    }
    return a;
}

/*******
 *
 * THE PID CONSTANTS
 *
 * These values need to be tuned to use the PID
 *
 */
double kP = 0;
double kI = 0;
double kD = 0;

void turnPid(double target, double maxError)
{
    target = posNeg180(target);
    double integral = 0;
    double lastError = 0;
    double error = 0;

    int timeInterval = 10; // ms
    int timeInRange = 0;

    while (timeInRange < 300)
    {
        // Get the current angle
        double currentAngle = posNeg180(Inertial.rotation());

        // Get the error
        error = target - currentAngle;

        // Get the integral
        integral += error;

        // Get the derivative
        double derivative = error - lastError;

        // Get the power
        double power = kP * error + kI * integral + kD * derivative;

        // Spin the motors
        Left.spin(forward, power, voltageUnits::volt);
        Right.spin(forward, -power, voltageUnits::volt);

        // Update the last error
        lastError = error;

        // Check if we are in range
        if (std::abs(error) < maxError)
        {
            timeInRange += timeInterval;
        }
        else
        {
            timeInRange = 0;
        }

        // Wait
        wait(timeInterval, msec);
    }
    cout << "PID Done" << endl;
    cout << "Current Angle: " << Inertial.rotation() << ", Target Angle:" << target << endl;
}

int main()
{
    // Calibrate inertial sensor
    cout << "Calibrating Inertial..." << flush;
    Inertial.calibrate();
    // wait for calibration to complete
    while (Inertial.isCalibrating())
    {
        wait(100, msec);
        cout << "." << flush;
    }

    task::sleep(1000);
    turnPid(90, 3);

    // turnPid(180, 3);

    // turnPid(270, 3);

    // turnPid(360, 3);

    // turnPid(180, 3);

    // turnPid(90, 3);

    // turnPid(270, 3);

    while (1)
    {

        // Allow other tasks to run
        this_thread::sleep_for(10);
    }
}
