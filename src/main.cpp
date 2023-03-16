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

controller Controller = controller();

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
double turnKP = 0;
double turnKI = 0;
double turnKD = 0;

void turnPid(double target, double maxError = 1)
{
    target = posNeg180(target);
    double integral = 0;
    double lastError = 0;
    double error = 0;

    int timeInterval = 10; // ms
    int timeInRange = 0;

    int timeTaken = 0;
    while (timeInRange < 100)
    {
        if(timeTaken > 5000){
            cout << "5 second turn loop timeout" << endl;
        }
        // Get the current angle
        double currentAngle = posNeg180(Inertial.rotation());

        // Get the error
        error = target - currentAngle;

        // Get the integral
        integral += error;

        // Get the derivative
        double derivative = error - lastError;

        // Get the power
        double power = turnKP * error + turnKI * integral + turnKD * derivative;

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
        timeTaken += timeInterval;
    }
    cout << "PID Done" << endl;
    cout << "Current Angle: " << Inertial.rotation() << ", Target Angle:" << target << endl;
}
int update(){
    while(1){
        if(Controller.ButtonA.pressing()){
            while(Controller.ButtonA.pressing()){
                wait(10, msec);
            }
            return 1;
        }
        if(Controller.ButtonB.pressing()){
            while(Controller.ButtonB.pressing()){
                wait(10, msec);
            }
            return 0;
        }
        if(Controller.ButtonX.pressing()){
            while(Controller.ButtonX.pressing()){
                wait(10, msec);
            }
            return -1;
        }
        wait(10, msec);
    }
}
void updateValues(){
    cout << "turnKP" << endl;   
    turnKP += pInc * update();
    cout << "turnKD" << endl;
    turnKD += dInc * update();
    cout << "double turnKP = " << turnKP << ";\ndouble turnKD = " << turnKD << ";" << endl;

}
void tunePid(){
    double pInc = 0.1;
    double dInc = 0.05;
    while(1){
        turnPid(90, 1);
        updateValues();
        turnPid(0, 1);
        updateValues();
    }
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
