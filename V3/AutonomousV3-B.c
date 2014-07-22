#pragma config(Hubs,  S3, HTMotor,  HTMotor,  none,     none)
#pragma config(Hubs,  S4, HTServo,  none,     none,     none)
#pragma config(Sensor, S1,     sensorIR,       sensorHiTechnicIRSeeker1200)
#pragma config(Sensor, S2,     sensorSonar,    sensorSONAR)
#pragma config(Sensor, S3,     ,               sensorI2CMuxController)
#pragma config(Sensor, S4,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S3_C1_1,     motorDriveRight, tmotorTetrix, PIDControl, reversed)
#pragma config(Motor,  mtr_S3_C1_2,     motorDriveLeft, tmotorTetrix, PIDControl)
#pragma config(Motor,  mtr_S3_C2_1,     motorElevator, tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S3_C2_2,     motorArmPitch, tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S4_C1_1,    servoArmGripper,      tServoContinuousRotation)
#pragma config(Servo,  srvo_S4_C1_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S4_C1_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S4_C1_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S4_C1_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S4_C1_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

// The Doppler Effect 2013-2014
// Autonomous V3-B Source
// Designed for FTC Block Party 2013-2014

/* -------------------------------------------------- */

// V3-B purpose:
// - Drive up to IR-identified basket (with baskets on right)
// - Deposit autonomous block
// - Park fully on ramp

// V3-B setup:
// - Pivot IRSeekerV2 sensor 45 degrees CW

#include "AutonomousV3-B.h"
#include "AutonomousV3-COM.c"

#include "JoystickDriver.c"

task main() {
	// Prompt for delay
	int delay = promptDelay();

	// Localize external robot references
	configureSensors(sensorIR, sensorSonar);
	configureMotors(motorDriveLeft, motorDriveRight, motorArmPitch, motorElevator);
	configureServos(servoArmGripper);

	// Initialize
	initialize();

	// Wait for the FCS start signal
	waitForStart();

	// Wait the designated time
	wait1Msec(delay*1000);

	// Initial arm upwards movement
	motor[refMotorArmPitch] = 75;
	wait1Msec(1000);
	motor[refMotorArmPitch] = 0;

	// Start collision avoidance subsystem
	StartTask(collisionAvoidance);

	// Run autonomous B
	autonomousB();
}

// Actual routine code
void autonomousB() {
	// Drive until parallel with IR beacon
	driveWithSensorGoal(100, 100, sensorIR, 7, 0);

	// Record distance travelled
	int distLeft  = nMotorEncoder[motorDriveLeft];
	int distRight = nMotorEncoder[motorDriveRight];

	// Move forward a bit more
	driveWithEncoderGoal(50, 50, ENCODER_RATIO_DRIVE_LEFT*0.2, ENCODER_RATIO_DRIVE_RIGHT*0.2, ENCODER_RATIO_DRIVE_LEFT/100, ENCODER_RATIO_DRIVE_RIGHT/100);
	wait1Msec(200);

	wait1Msec(5000);

	// Add to the distance
	//distLeft  += nMotorEncoder[motorDriveLeft];
	//distRight += nMotorEncoder[motorDriveRight];

	// Rotate CW to face basket
	driveWithSensorGoal(-50, 50, sensorIR, 4, 0);
	wait1Msec(200);

	// Record distance from basket
	int distance = SensorValue[sensorSonar];

	// Drive up to basket
	driveWithSensorGoal(50, 50, sensorSonar, 25, 3); // 22 for 360
	wait1Msec(200);

	// Open gripper
	openGripper();
	wait1Msec(500);

	// Drive away from basket
	driveWithSensorGoal(-50, -50, sensorSonar, distance, 5);
	wait1Msec(200);

	// Rotate CW to face start position
	driveWithEncoderGoal(-50, 50, -ENCODER_RATIO_DRIVE_LEFT*0.13340*2.5, ENCODER_RATIO_DRIVE_RIGHT*0.13340*2.5, ENCODER_RATIO_DRIVE_LEFT/100, ENCODER_RATIO_DRIVE_RIGHT/100);
	//driveWithSensorGoal(-50, 50, sensorIR, 1, 0);
	wait1Msec(200);

	wait1Msec(5000);

	// Drive to the approximate start position
	driveWithEncoderGoal(100, 100, distRight, distLeft, ENCODER_RATIO_DRIVE_LEFT/100, ENCODER_RATIO_DRIVE_RIGHT/100);
	wait1Msec(200);

	while (true) {}

	// Rotate CCW
	driveWithEncoderGoal(50, -50, ENCODER_RATIO_DRIVE_LEFT*0.13340*2, -ENCODER_RATIO_DRIVE_RIGHT*0.13340*2, ENCODER_RATIO_DRIVE_LEFT/100, ENCODER_RATIO_DRIVE_RIGHT/100);
	wait1Msec(200);

	// Drive forward
	driveWithEncoderGoal(100, 100, ENCODER_RATIO_DRIVE_LEFT, ENCODER_RATIO_DRIVE_RIGHT, ENCODER_RATIO_DRIVE_LEFT/100, ENCODER_RATIO_DRIVE_RIGHT/100);
	wait1Msec(200);

	// Rotate CCW
	driveWithEncoderGoal(50, -50, ENCODER_RATIO_DRIVE_LEFT*0.13340*2, -ENCODER_RATIO_DRIVE_RIGHT*0.13340*2, ENCODER_RATIO_DRIVE_LEFT/100, ENCODER_RATIO_DRIVE_RIGHT/100);
	wait1Msec(200);

	// Drive onto ramp
	driveWithEncoderGoal(100, 100, ENCODER_RATIO_DRIVE_LEFT, ENCODER_RATIO_DRIVE_RIGHT, ENCODER_RATIO_DRIVE_LEFT/100, ENCODER_RATIO_DRIVE_RIGHT/100);
}
