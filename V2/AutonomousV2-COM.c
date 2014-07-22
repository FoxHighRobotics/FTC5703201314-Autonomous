
// The Doppler Effect 2013
// Autonomous V2-COM Source

/* -------------------------------------------------- */

// V2-COM purpose:
// - This file contains code common to all autonomous programs
// - This file is not standalone; each autonomous program must include this source file

// V2-COM setup:
// - N/A

// Used for common autonomous functions
#include "AutonomousV2-COM.h"

short motorDriveLeft;
short motorDriveRight;

short sensorIR;
short sensorUltrasonic;
short sensorLight;

void initialize() {
	// Motor speed regulation
	nMotorPIDSpeedCtrl[motorDriveLeft]  = mtrSpeedReg;
	nMotorPIDSpeedCtrl[motorDriveRight] = mtrSpeedReg;
	
	// Reset encoder values
	nMotorEncoder[motorDriveLeft]  = 0;
	nMotorEncoder[motorDriveRight] = 0;
	
	// Increase precision with PWM off-duty braking
	bFloatDuringInactiveMotorPWM = false;
}

void configureMotors(short paramMotorDriveLeft, short paramMotorDriveRight) {
	motorDriveLeft  = paramMotorDriveLeft;
	motorDriveRight = paramMotorDriveRight;
}

void configureSensors(short paramSensorIR, short paramSensorUltrasonic, short paramSensorLight) {
	sensorIR         = paramSensorIR;
	sensorUltrasonic = paramSensorUltrasonic;
	sensorLight      = paramSensorLight;
}

void driveWithEncoderGoal(int powerLeft, int powerRight, int countsLeft, int countsRight) {
	// Reset encoder values
	nMotorEncoder[motorDriveLeft]  = 0;
	nMotorEncoder[motorDriveRight] = 0;
	
	// Set encoder targets
	nMotorEncoderTarget[motorDriveLeft]  = countsLeft;
	nMotorEncoderTarget[motorDriveRight] = countsRight;
	
	// Set motor powers
	motor[motorDriveLeft]  = powerLeft;
	motor[motorDriveRight] = powerRight;
	
	// Number of motors still running
	int runningMotors = 2;
	
	// While at least one motor is still running
	while (runningMotors > 0) {
		if (nMotorRunState[motorDriveRight] == runStateIdle) {
			motor[motorDriveRight] = 0;
			runningMotors--;
		}
		if (nMotorRunState[motorDriveLeft] == runStateIdle) {
			motor[motorDriveLeft] = 0;
			runningMotors--;
		}
		wait1Msec(50);
	}
}

void driveWithIRGoal(int powerLeft, int powerRight, int target, int targetThreshold) {
	// Reset encoder values
	nMotorEncoder[motorDriveLeft]  = 0;
	nMotorEncoder[motorDriveRight] = 0;
	
	// Set motor powers
	motor[motorDriveLeft]  = powerLeft;
	motor[motorDriveRight] = powerRight;
	
	
}
