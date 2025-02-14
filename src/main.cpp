/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       jackdo1012                                                */
/*    Created:      12/16/2024, 7:58:48 AM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "iostream"
#include "vex.h"

using namespace vex;

competition Competition;

Drive chassis = Drive(9, 11, 12, vex::motor_group(leftBackMotor, leftFrontMotor),
                      vex::motor_group(rightBackMotor, rightFrontMotor));

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions */
/*                                                                           */
/*  You may want to perform some actions before the competition starts. */
/*  Do them in the following function.  You must return from this function
 */
/*  or the autonomous and usercontrol tasks will not be started.  This */
/*  function is only called once after the V5 has been powered on and */
/*  not every time that the robot is disabled. */
/*---------------------------------------------------------------------------*/

void pre_auton(void)
{
    // All activities that occur before the competition starts
    // Example: clearing encoders, setting servo positions, ...
    Inertial.calibrate(3);
    while (Inertial.isCalibrating())
    {
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Inertial Calibrating");
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Inertial Calibrating");
        wait(50, msec);
    }
    Brain.Screen.clearScreen();
    Controller.Screen.clearScreen();
    armSensor.setPosition(0, vex::rotationUnits::rev);
    leftMotors.setStopping(vex::brake);
    rightMotors.setStopping(vex::brake);
    intakeMotor.setVelocity(100, percent);
    conveyorMotor.setVelocity(90, percent);
    leftArmMotor.setVelocity(70, percent);
    rightArmMotor.setVelocity(70, percent);
}

enum MotorStatus
{
    forward,
    reverse,
    stop
};

// off, loading, neutral
double armPoses[] = {0, 60, 279, 380};

MotorStatus intakeStatus = MotorStatus::stop;
int armPos = 0;
double armTarget = 0;
int numberOfPos = 4;
double armPrevErr = 0;
bool clampStatus = false;
bool doinkerStatus = false;
bool powerProcessing = false;
bool slowIntake = false;

void printOnController(int row, int col, std::string data)
{
    Controller.Screen.clearLine(row);
    Controller.Screen.setCursor(row, col);
    Controller.Screen.print(data.c_str());
}

void runIntake()
{
    if (intakeStatus == MotorStatus::forward)
    {
        intakeMotor.stop();
        conveyorMotor.stop();
        intakeStatus = MotorStatus::stop;
    }
    else
    {
        intakeMotor.spin(vex::forward);
        conveyorMotor.spin(vex::forward);
        intakeStatus = MotorStatus::forward;
    }
}

void reverseIntake()
{
    if (intakeStatus == MotorStatus::reverse)
    {
        intakeMotor.stop();
        conveyorMotor.stop();
        intakeStatus = MotorStatus::stop;
    }
    else
    {
        intakeMotor.spin(vex::reverse);
        conveyorMotor.spin(vex::reverse);
        intakeStatus = MotorStatus::reverse;
    }
}

void toggleClamp()
{
    clampStatus = !clampStatus;
    clampCylinder.set(clampStatus);
    std::string printData = "Clamp: " + std::string(clampStatus ? "on" : "off");
    printOnController(1, 1, printData);
}

void toggleDoinker()
{
    doinkerStatus = !doinkerStatus;
    doinkerCylinder.set(doinkerStatus);
}

void armControl()
{
    double kp = .5;
    double kd = .2;
    double err = armTarget - (leftArmMotor.position(vex::rotationUnits::rev) * 360);
    double output = kp * err + kd * (err - armPrevErr);
    armPrevErr = err;
    output = clamp(-50, output, 50);
    leftArmMotor.spin(vex::forward, output, percent);
    rightArmMotor.spin(vex::forward, output, percent);
}

int armMovementTask()
{
    leftArmMotor.setPosition(0, vex::rotationUnits::rev);
    while (true)
    {
        armControl();
        vex::task::sleep(10);
    }
    return 0;
}

void armOut()
{
    if (armPos < numberOfPos - 1)
    {

        armPos += 1;
        armTarget = armPoses[armPos];
    }
}

void armIn()
{
    if (armPos > 0)
    {
        armPos--;
        armTarget = armPoses[armPos];
    }
}

void slowIntakeControl()
{
    conveyorMotor.setVelocity(5, percent);
}

void fastIntakeControl()
{
    conveyorMotor.setVelocity(90, percent);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void)
{
    // ..........................................................................
    // Insert autonomous user code here.
    // ..........................................................................
    chassis.setInitPos(0, 0, 90);
    chassis.changeDrivePid(2.7, 0, 10);
    chassis.changeTurnPid(1.5, .01, 6);
    runIntake();
    wait(500, msec);
    runIntake();
    chassis.driveToPoint(0, 5, 90, 2000, 200);
    chassis.changeTurnPid(1.5, .01, 8);
    chassis.turnToHeading(197, 2000);
    chassis.changeTurnPid(1.5, 0, 6);
    chassis.changeDrivePid(2.7, 0, 30);
    chassis.driveToPoint(-20 * cos(toRadian(197)), 5 - 20 * sin(toRadian(197)), 197, 2000, 10);
    clampCylinder.set(true);
    chassis.turnToHeading(90, 2000, 10);
    runIntake();
    chassis.changeDrivePid(1.5, 0, 10, 3);
    chassis.changeTurnPid(.5, 0, 8, 3);

    chassis.driveToPoint(23, 33, 60, 4000, 50);

    chassis.changeDrivePid(2, 0, 10, 1.5);
    chassis.changeTurnPid(1.5, 0, 6, 1);
    chassis.turnToHeading(8, 4000, 10);
    chassis.driveToPoint(47, 34, 8, 4000);
    chassis.turnToHeading(275, 4000);
    chassis.driveToPoint(47, 2, 275, 4000);

    wait(500, msec);
    chassis.turnToHeading(50, 3000);
    chassis.driveToPoint(56, 17, 50, 4000);
    chassis.turnToHeading(100, 2000);
    chassis.driveToPoint(58, 5, 100, 4000);
    clampCylinder.set(false);
    chassis.turnToHeading(117, 2000);
    chassis.driveToPoint(55, 11, 117, 4000);
    chassis.turnToHeading(180, 2000);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol()
{
    std::string printData = "Clamp: " + std::string(clampStatus ? "on" : "off");
    printOnController(1, 1, printData);

    Controller.ButtonA.pressed(toggleClamp);
    Controller.ButtonDown.pressed(toggleDoinker);
    Controller.ButtonR1.pressed(armOut);
    Controller.ButtonR2.pressed(armIn);
    Controller.ButtonL1.pressed(runIntake);
    Controller.ButtonL2.pressed(reverseIntake);
    Controller.ButtonB.pressed(slowIntakeControl);
    Controller.ButtonB.released(fastIntakeControl);
    Controller.Screen.clearScreen();
    // User control code here, inside the loop
    while (1)
    {
        double leftSpeed = Controller.Axis3.position() + Controller.Axis1.position();
        double rightSpeed = Controller.Axis3.position() - Controller.Axis1.position();

        if (fabs(leftSpeed) < 5)
        {
            leftSpeed = 0;
        }
        if (fabs(rightSpeed) < 5)
        {
            rightSpeed = 0;
        }
        leftMotors.spin(vex::forward, leftSpeed, percent);
        rightMotors.spin(vex::forward, rightSpeed, percent);
        wait(20, msec); // Sleep the task for a short amount of time
                        // to prevent wasted resources.
    }
}

int odomTest()
{
    while (true)
    {
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("X: %f", chassis.odom.xPos);
        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("Y: %f", chassis.odom.yPos);
        Controller.Screen.setCursor(3, 1);
        Controller.Screen.print("Orientation: %f", chassis.odom.orientation);
        vex::task::sleep(20);
    }
    return 0;
}

//
// Main will set up the competition functions and callbacks.
//
int main()
{
    // Set up callbacks for autonomous and driver control periods.
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);

    // Run the pre-autonomous function.
    pre_auton();

    vex::task armTask(armMovementTask);

    // Prevent main from exiting with an infinite loop.
    while (true)
    {
        wait(100, msec);
    }
}
