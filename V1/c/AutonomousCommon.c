
// The Doppler Effect 2013
// Autonomous A&B Common Code

#include "AutonomousCommon.h"

short nMotorRight;
short nMotorLeft;
short nServoGripper;
short nServoArmPitch;
short nServoArmSegment;
short nSensorIR;
short nSensorUltrasonic;

// Robot initialization
void initialize() {
	// Motor speed regulation
	nMotorPIDSpeedCtrl[nMotorRight] = mtrSpeedReg;
	nMotorPIDSpeedCtrl[nMotorLeft]  = mtrSpeedReg;

	// Reset encoders
	resetEncoderValues();

	// Increase precision with off-duty PWM braking
	bFloatDuringInactiveMotorPWM = false;

	// Default to closed gripper
	setGripperState(false);

	// Wait for everything to catch up
	wait1Msec(2000);
}

// Reset encoder values for both motors
void resetEncoderValues() {
	nMotorEncoder[nMotorRight] = 0;
	nMotorEncoder[nMotorLeft]  = 0;
}

// Drive until the given per-motor encoder values are achieved
void driveToEncoderValues(int powerLeft, int powerRight, int countsLeft, int countsRight) {
	resetEncoderValues();

	nMotorEncoderTarget[nMotorRight] = countsRight;
	nMotorEncoderTarget[nMotorLeft]  = countsLeft;

	motor[nMotorRight] = powerRight;
	motor[nMotorLeft]  = powerLeft;

	int runningMotors = 2;

	while (true) {
		if (nMotorRunState[nMotorRight] == runStateIdle) {
			motor[nMotorRight] = 0;
			runningMotors--;
		}
		if (nMotorRunState[nMotorLeft] == runStateIdle) {
			motor[nMotorLeft] = 0;
			runningMotors--;
		}
		if (runningMotors == 0) {
			break;
		}
	}
}

// Drive until the IR sensor reports a value within the given range, inclusive
void driveToIR(int powerLeft, int powerRight, int irRangeBottom, int irRangeTop) {
	resetEncoderValues();

	motor[nMotorRight] = powerRight;
	motor[nMotorLeft]  = powerLeft;

	while (!(SensorValue[nSensorIR] >= irRangeBottom && SensorValue[nSensorIR] <= irRangeTop)) {
		wait1Msec(10);
	}

	motor[nMotorRight] = 0;
	motor[nMotorLeft]  = 0;
}

// Drive until the ultrasonic sensor reports a value within the given range, inclusive
void driveToUltrasonic(int powerLeft, int powerRight, int distanceRangeBottom, int distanceRangeTop) {
	resetEncoderValues();

	motor[nMotorRight] = powerRight;
	motor[nMotorLeft]  = powerLeft;

	while (!(SensorValue[nSensorUltrasonic] >= distanceRangeBottom && SensorValue[nSensorUltrasonic] <= distanceRangeTop)) {
		wait1Msec(10);
	}

	motor[nMotorRight] = 0;
	motor[nMotorLeft]  = 0;
}

// Sets the state of the gripper
void setGripperState(bool state) {
	if (state) {
		servo[nServoGripper] = 0;
	} else {
		servo[nServoGripper] = 0;
	}
}
