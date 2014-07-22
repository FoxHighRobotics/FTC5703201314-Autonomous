#pragma config(Hubs,  S3, HTServo,  none,     none,     none)
#pragma config(Hubs,  S4, HTMotor,  none,     none,     none)
#pragma config(Sensor, S1,     sensorIR,       sensorHiTechnicIRSeeker1200)
#pragma config(Sensor, S2,     sensorUltrasonic, sensorSONAR)
#pragma config(Sensor, S3,     ,               sensorI2CMuxController)
#pragma config(Sensor, S4,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S4_C1_1,     motorRight,    tmotorTetrix, PIDControl, encoder)
#pragma config(Motor,  mtr_S4_C1_2,     motorLeft,     tmotorTetrix, PIDControl, reversed, encoder)
#pragma config(Servo,  srvo_S3_C1_1,    servoArmPitch,        tServoStandard)
#pragma config(Servo,  srvo_S3_C1_2,    servoArmSegment,      tServoContinuousRotation)
#pragma config(Servo,  srvo_S3_C1_3,    servoGripper,         tServoContinuousRotation)
#pragma config(Servo,  srvo_S3_C1_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S3_C1_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S3_C1_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

// The Doppler Effect 2013
// Autonomous B

#include "JoystickDriver.c"

#include "AutonomousB.h"
#include "AutonomousCommon.c"

void autonomousB() {

}

task main() {
	// Initialize robot
	initialize();

	// Wait for FCS to begin autonomous period
	waitForStart();

	// Autonomous routine
	autonomousB();
}
