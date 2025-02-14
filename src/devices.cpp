#include "vex.h"

using namespace vex;

vex::brain Brain = vex::brain();
vex::controller Controller = vex::controller();
// motors and piston
vex::motor intakeMotor = vex::motor(PORT3, ratio6_1, true);
vex::motor conveyorMotor = vex::motor(PORT18, ratio6_1, true);
vex::motor ptoMotor = vex::motor(PORT18, ratio6_1, true);
vex::motor leftArmMotor = vex::motor(PORT20, false);
vex::motor rightArmMotor = vex::motor(PORT6, true);
vex::digital_out clampCylinder = vex::digital_out(Brain.ThreeWirePort.A);
vex::digital_out doinkerCylinder = vex::digital_out(Brain.ThreeWirePort.H);
// sensors
vex::rotation verticalOdo = vex::rotation(PORT13);
vex::rotation horizontalOdo = vex::rotation(PORT12);
vex::rotation armSensor = vex::rotation(PORT8);
vex::inertial Inertial = vex::inertial(PORT5);
// drivetrain
vex::motor leftBackMotor = vex::motor(PORT4, ratio6_1, true);
vex::motor leftFrontMotor = vex::motor(PORT2, ratio6_1, true);
vex::motor rightBackMotor = vex::motor(PORT16, ratio6_1, false);
vex::motor rightFrontMotor = vex::motor(PORT17, ratio6_1, false);
vex::motor_group leftMotors = vex::motor_group(leftBackMotor, leftFrontMotor);
vex::motor_group rightMotors = vex::motor_group(rightBackMotor, rightFrontMotor);