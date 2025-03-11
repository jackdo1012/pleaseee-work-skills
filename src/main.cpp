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
    leftMotors.setStopping(vex::brake);
    rightMotors.setStopping(vex::brake);
    intakeMotor.setVelocity(100, percent);
    conveyorMotor.setVelocity(100, percent);
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
// double armPoses[] = {0, 50, 279, 380};
double armPoses[] = {0, 52, 279};
// double armPoses[] = {0, 52, 150, 279, 380};

int numberOfPos = 3;
MotorStatus intakeSystemStatus = MotorStatus::stop;
int armPos = 0;
double armTarget = 0;
double armPrevErr = 0;
bool clampStatus = false;
bool doinkerStatus = false;
bool powerProcessing = false;
bool slowIntake = false;
int pos = 1; // 1: red left, 2: red right, 3: blue left, 4: blue right
bool colorSorting = true;

void printOnController(int row, int col, std::string data)
{
    Controller.Screen.clearLine(row);
    Controller.Screen.setCursor(row, col);
    Controller.Screen.print(data.c_str());
}

void toggleIntakeOnly()
{
    intakeMotor.spin(vex::forward);
}

void runIntake()
{
    if (intakeSystemStatus == MotorStatus::forward)
    {
        intakeMotor.stop();
        conveyorMotor.stop();
        intakeSystemStatus = MotorStatus::stop;
    }
    else
    {
        intakeMotor.spin(vex::forward);
        conveyorMotor.spin(vex::forward);
        intakeSystemStatus = MotorStatus::forward;
    }
}

void reverseIntake()
{
    if (intakeSystemStatus == MotorStatus::reverse)
    {
        intakeMotor.stop();
        conveyorMotor.stop();
        intakeSystemStatus = MotorStatus::stop;
    }
    else
    {
        intakeMotor.spin(vex::reverse);
        conveyorMotor.spin(vex::reverse);
        intakeSystemStatus = MotorStatus::reverse;
    }
}
void toggleClamp()
{
    clampStatus = !clampStatus;
    clampCylinder.set(clampStatus);
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
    conveyorMotor.setVelocity(100, percent);
}

void sideSelection()
{
    Brain.Screen.setFillColor(red);
    Brain.Screen.drawRectangle(0, 0, 240, 240);
    Brain.Screen.setCursor(3, 8);
    Brain.Screen.print("red left");

    Brain.Screen.setFillColor(red);
    Brain.Screen.drawRectangle(240, 0, 240, 120);
    Brain.Screen.setCursor(3, 32);
    Brain.Screen.print("red right");

    Brain.Screen.setFillColor(blue);
    Brain.Screen.drawRectangle(0, 120, 240, 120);
    Brain.Screen.setCursor(10, 8);
    Brain.Screen.print("blue left");

    Brain.Screen.setFillColor(blue);
    Brain.Screen.drawRectangle(240, 120, 240, 120);
    Brain.Screen.setCursor(10, 32);
    Brain.Screen.print("blue right");

    while (!Brain.Screen.pressing())
    {
        wait(5, msec);
    }

    if (Brain.Screen.xPosition() < 240)
    {
        if (Brain.Screen.yPosition() < 120)
        {
            pos = 1;
            Brain.Screen.setFillColor(red);
            Brain.Screen.drawRectangle(0, 0, 480, 240);
            armTarget = armPoses[1];
        }
        else
        {
            pos = 3;
            Brain.Screen.setFillColor(blue);
            Brain.Screen.drawRectangle(0, 0, 480, 240);
        }
    }
    else
    {
        if (Brain.Screen.yPosition() < 120)
        {
            pos = 2;
            Brain.Screen.setFillColor(red);
            Brain.Screen.drawRectangle(0, 0, 480, 240);
        }
        else
        {
            pos = 4;
            Brain.Screen.setFillColor(blue);
            Brain.Screen.drawRectangle(0, 0, 480, 240);
            armTarget = armPoses[1];
        }
    }
}
void colorSortingFunc()
{
    if (colorSensor.color() < 1000000 && pos - 2 > 0)
    {
        return;
    }
    else if (colorSensor.color() > 1000000 && pos - 2 <= 0)
    {
        return;
    }
    if (colorSorting)
    {
        wait(50, msec);
        conveyorMotor.stop(vex::brake);
        wait(1, seconds);
        conveyorMotor.spin(vex::forward);
        intakeMotor.spin(vex::forward);
        intakeSystemStatus = MotorStatus::forward;
    }
}

void toggleColorSorting()
{
    colorSorting = !colorSorting;
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
    Controller.ButtonA.pressed(toggleClamp);
    Controller.ButtonDown.pressed(toggleDoinker);
    Controller.ButtonR1.pressed(armOut);
    Controller.ButtonR2.pressed(armIn);
    Controller.ButtonL1.pressed(runIntake);
    Controller.ButtonL2.pressed(reverseIntake);
    Controller.ButtonB.pressed(slowIntakeControl);
    Controller.ButtonB.released(fastIntakeControl);
    Controller.ButtonX.pressed(toggleIntakeOnly);

    Controller.Screen.clearScreen();
    // User control code here, inside the loop
    while (1)
    {
        double lateral = Controller.Axis1.position();

        double leftSpeed = Controller.Axis3.position() + lateral;
        double rightSpeed = Controller.Axis3.position() - lateral;

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

int redirect()
{
    while (true)
    {
        if (conveyorMotor.torque() >= 0.35 && intakeSystemStatus == MotorStatus::forward)
        {
            wait(700, msec);
            if (conveyorMotor.torque() >= 0.35)
            {
                conveyorMotor.spin(vex::reverse);
                wait(300, msec);
                conveyorMotor.spin(vex::forward);
            }
        }
        wait(50, msec);
    }
    return 0;
}

void colorSortFunc()
{
    if (colorSensor.hue() < 80) // red
    {
    }
    else if (colorSensor.hue() > 80) // blue
    {
        return;
    }
    wait(50, msec);
    conveyorMotor.stop();
    wait(500, msec);
    conveyorMotor.spin(vex::forward);
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
    vex::task redirectTask(redirect);

    // colorSensor.setLightPower(100, percent);
    // colorSensor.objectDetected(colorSortFunc);

    wait(500, msec);

    chassis.setInitPos(0, 0, 90);
    conveyorMotor.spin(vex::forward);
    wait(700, msec);
    conveyorMotor.spin(vex::reverse);
    intakeMotor.spin(vex::forward);

    chassis.driveToPoint(0, 10, 5000, 50);
    chassis.turnToHeading(350, 2000);
    conveyorMotor.stop();

    chassis.driveToPoint(-19.5, 12, 2000, 10);
    clampCylinder = true;
    wait(100, msec);
    chassis.turnToHeading(94, 2000);

    chassis.driveToPoint(-24, 31, 2000, 3);
    conveyorMotor.spin(vex::forward);
    chassis.turnToHeading(140, 2000, 3);
    chassis.driveToPoint(-47.5, 85, 7000, 3);
    armTarget = 52;

    conveyorMotor.spin(vex::forward);

    chassis.changeDrivePid(6, 0, 50, 1);
    chassis.changeHeadingPid(2, 0, 15, 1);
    chassis.driveToPoint(-38, 62, 5000, 400); // drive back to do wall stake
    chassis.defaultDrivePid();
    chassis.defaultHeadingPid();

    chassis.turnToHeading(180, 3000, 300);

    conveyorMotor.spin(vex::reverse);
    armTarget = 150;
    wait(200, msec);
    conveyorMotor.spin(vex::forward);

    chassis.changeDrivePid(2.5, 0, 35);
    chassis.driveToPoint(-62, 61, 2000); // drive into the wall stake
    chassis.defaultDrivePid();

    chassis.drive(12, 12);
    armTarget = 279;
    wait(300, msec);
    chassis.stop();
    chassis.driveToPoint(-49.5, 61, 2000);
    armTarget = 0;
    chassis.turnToHeading(270, 2000);

    redirectTask.suspend();

    chassis.changeDrivePid(2, 0, 80);
    chassis.driveToPoint(-46.5, 3, 8000, 300);
    chassis.driveToPoint(-46.5, 3, 8000, 300);
    chassis.defaultDrivePid();
    wait(200, msec);
    chassis.turnToHeading(130, 3000);

    redirectTask.resume();

    chassis.driveToPoint(-58, 19, 3000);
    chassis.turnToHeading(84, 2000);
    chassis.driveToPoint(-60, 6, 2000);
    clampCylinder = false;
    conveyorMotor.spin(vex::reverse);
    wait(200, msec);
    conveyorMotor.spin(vex::forward);

    chassis.changeDrivePid(3.3, 0, 55);

    chassis.driveToPoint(-44, 103, 8000);
    conveyorMotor.stop();
    chassis.defaultDrivePid();
    chassis.turnToHeading(220, 3000);
    conveyorMotor.spinFor(vex::forward, 400, msec);

    chassis.driveToPoint(-22, 118, 2000, 5);
    clampCylinder = true;

    chassis.turnToHeading(172, 2000);
    doinkerCylinder = true;
    intakeMotor.stop();
    chassis.changeDrivePid(10, 0.2, 10);
    chassis.changeTurnPid(4, 0.3, 10);
    chassis.driveToPoint(-48, 123, 4000);
    chassis.turnToHeading(280, 5000, 3);
    clampCylinder = false;
    chassis.changeTurnPid(2, 0, 10);
    chassis.turnToHeading(325, 5000);
    chassis.defaultDrivePid();
    chassis.defaultHeadingPid();
    chassis.defaultTurnPid();
    doinkerCylinder = false;
    chassis.driveToPoint(-51, 121, 2000, 3);
    intakeMotor.spin(vex::forward);

    chassis.driveToPoint(-36, 109, 2000, 300);
    chassis.turnToHeading(180, 2000, 300);

    chassis.driveToPoint(1, 109, 4000, 10);
    clampCylinder = true;

    chassis.turnToHeading(225, 2000, 300);
    conveyorMotor.spin(vex::forward);
    chassis.driveToPoint(-20, 83, 4000, 600);
    chassis.turnToHeading(317, 2000);
    intakeMotor.stop();
    conveyorMotor.stop();
    // chassis.driveToPoint(22, 42, 3000, 3);
    // intakeMotor.spin(vex::forward);
    // conveyorMotor.spin(vex::forward);
    // chassis.driveToPoint(45, 17, 3000);

    odomTest();

    // Prevent main from exiting with an infinite loop.
    while (true)
    {
        wait(100, msec);
    }
}
