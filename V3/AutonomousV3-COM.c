
// The Doppler Effect 2013-2014
// Autonomous V3-COM Source
// Designed for FTC Block Party 2013-2014

/* -------------------------------------------------- */

// V3-COM purpose:
// - This file contains code common to each autonomous program
// - This file is not standalone, rather it is designed to be included by each autonomous program

// V3-COM setup:
// - N/A

// Used for common autonomous functions
#include "AutonomousV3-COM.h"

// Local references to sensors
short refSensorIR;
short refSensorSonar;

// Local references to motors
short refMotorDriveLeft;
short refMotorDriveRight;
short refMotorArmPitch;
short refMotorElevator;

// Local references to servos
short refServoArmGripper;

// Set to true to pause the current task
// Used for when the collision avoidance system takes over
bool pauseTask;

// Localizes references to robot sensor interfaces
void configureSensors(short paramSensorIR, short paramSensorSonar) {
	refSensorIR    = paramSensorIR;
	refSensorSonar = paramSensorSonar;
}

// Localizes references to robot motor interfaces
void configureMotors(short paramMotorDriveLeft, short paramMotorDriveRight, short paramMotorArmPitch, short paramMotorElevator) {
	refMotorDriveLeft  = paramMotorDriveLeft;
	refMotorDriveRight = paramMotorDriveRight;
	refMotorArmPitch   = paramMotorArmPitch;
	refMotorElevator   = paramMotorElevator;
}

// Localizes references to robot servo interfaces
void configureServos(short paramServoArmGripper) {
	refServoArmGripper = paramServoArmGripper;
}

// Common initialization function
void initialize() {
	// Motor speed regulation
	nMotorPIDSpeedCtrl[refMotorDriveLeft]  = mtrSpeedReg;
	nMotorPIDSpeedCtrl[refMotorDriveRight] = mtrSpeedReg;

	// Reset encoder values
	nMotorEncoder[refMotorDriveLeft]  = 0;
	nMotorEncoder[refMotorDriveRight] = 0;

	// Increase precision with PWM off-duty braking
	bFloatDuringInactiveMotorPWM = false;

	// Clamp block
	closeGripper();
}

// Opens the gripper
void openGripper() {
	servo[refServoArmGripper] = 0;
}

// Closes the gripper
void closeGripper() {
	servo[refServoArmGripper] = 255;
}

// Drives until a specific encoder goal is achieved
void driveWithEncoderGoal(int powerLeft, int powerRight, int countsLeft, int countsRight) {
	// Reset encoders
	nMotorEncoder[refMotorDriveLeft]  = 0;
	nMotorEncoder[refMotorDriveRight] = 0;

	// Apply power
	motor[refMotorDriveLeft]  = powerLeft;
	motor[refMotorDriveRight] = powerRight;

	// While the goal has not been achieved, wait
	while (pauseTask || abs(nMotorEncoder[refMotorDriveLeft]) < abs(countsLeft) || abs(nMotorEncoder[refMotorDriveRight]) < abs(countsRight)) {
		wait1Msec(LOOP_DELAY);
	}

	// Stop motors
	motor[refMotorDriveLeft]  = 0;
	motor[refMotorDriveRight] = 0;
}

// Drives until a specific ranged encoder goal is achieved
void driveWithRangedEncoderGoal(int powerLeft, int powerRight, int countsLeftLow, int countsRightLow, int countsLeftHigh, int countsRightHigh) {
	// Reset encoders
	nMotorEncoder[refMotorDriveLeft]  = 0;
	nMotorEncoder[refMotorDriveRight] = 0;

	// Apply power
	motor[refMotorDriveLeft]  = powerLeft;
	motor[refMotorDriveRight] = powerRight;

	// While the goal has not been achieved, wait
	while (pauseTask || nMotorEncoder[refMotorDriveLeft] < countsLeftLow || nMotorEncoder[refMotorDriveLeft] > countsLeftHigh || nMotorEncoder[refMotorDriveRight] < countsRightLow || nMotorEncoder[refMotorDriveRight] > countsRightHigh) {
		wait1Msec(LOOP_DELAY);
	}

	// Stop motors
	motor[refMotorDriveLeft]  = 0;
	motor[refMotorDriveRight] = 0;
}

// Drives until a specific thresholded goal is achieved on the given sensor
void driveWithSensorGoal(int powerLeft, int powerRight, short sensor, int value, int threshold) {
	// Reset encoders
	nMotorEncoder[refMotorDriveLeft]  = 0;
	nMotorEncoder[refMotorDriveRight] = 0;

	// Apply power
	motor[refMotorDriveLeft]  = powerLeft;
	motor[refMotorDriveRight] = powerRight;

	// While the goal has not been achieved, wait
	while (pauseTask || abs(SensorValue[sensor] - value) > threshold) {
		wait1Msec(LOOP_DELAY);
	}

	// Stop motors
	motor[refMotorDriveLeft]  = 0;
	motor[refMotorDriveRight] = 0;
}

// Drives until a specific ranged goal is achieved on the given sensor
void driveWithRangedSensorGoal(int powerLeft, int powerRight, short sensor, int rangeLow, int rangeHigh) {
	// Reset encoders
	nMotorEncoder[refMotorDriveLeft]  = 0;
	nMotorEncoder[refMotorDriveRight] = 0;

	// Apply power
	motor[refMotorDriveLeft]  = powerLeft;
	motor[refMotorDriveRight] = powerRight;

	// While the goal has not been achieved, wait
	while (pauseTask || SensorValue[sensor] < rangeLow || SensorValue[sensor] > rangeHigh) {
		wait1Msec(LOOP_DELAY);
	}

	// Stop motors
	motor[refMotorDriveLeft]  = 0;
	motor[refMotorDriveRight] = 0;
}

// Display delay prompt and return result
int promptDelay() {
	bool menu = true;
	int delay = 0;
	int button = -1;

	while (menu) {
		switch (nNxtButtonPressed) {
			case -1:
				if (button != -1) {
					button = -1;
				}
				break;
			case 0: // Gray
				if (button != 0) {
					button = 0;
				}
				break;
			case 1: // Right
				if (button != 1) {
					button = 1;
					delay++;
				}
				break;
			case 2: // Left
				if (button != 2) {
					button = 2;
					delay--;
				}
				break;
			case 3: // Orange
				if (button != 3) {
					button = 3;
					menu = false;
				}
				break;
		}

		eraseDisplay();
		nxtDisplayCenteredBigTextLine(4, "%i seconds", delay);
	}

	return delay;
}

// A background task that attempts to avoid forward collisions
task collisionAvoidance() {
	while (ENABLE_COLLISION_AVOIDANCE) {

		eraseDisplay();
		nxtDisplayTextLine(1, "L: %i", nMotorEncoder[refMotorDriveLeft]);
		nxtDisplayTextLine(2, "R: %i", nMotorEncoder[refMotorDriveRight]);
		// Enter collision avoidance mode if things get too close for comfort
		if (SensorValue[refSensorSonar] < COLLISION_AVOIDANCE_THRESHOLD) {
			// Assume drive control
			pauseTask = true;

			// Record current drive powers
			int originalLeft  = motor[refMotorDriveLeft];
			int originalRight = motor[refMotorDriveRight];

			// Move to a safe distance from the object
			while (SensorValue[refSensorSonar] < COLLISION_AVOIDANCE_THRESHOLD*2) {
				motor[refMotorDriveLeft]  = (COLLISION_AVOIDANCE_THRESHOLD - SensorValue[refSensorSonar])*-10;
				motor[refMotorDriveRight] = (COLLISION_AVOIDANCE_THRESHOLD - SensorValue[refSensorSonar])*-10;
				wait1Msec(LOOP_DELAY);
			}

			// Release drive control
			pauseTask = false;

			// Restore original drive powers
			motor[refMotorDriveLeft]  = originalLeft;
			motor[refMotorDriveRight] = originalRight;
		}
		wait1Msec(LOOP_DELAY);
		EndTimeSlice();
	}
}
