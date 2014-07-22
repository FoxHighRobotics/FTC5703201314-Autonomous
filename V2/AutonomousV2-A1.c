#pragma config(Hubs, S4, HTMotor, HTMotor, HTServo, none)
#pragma config(Sensor, S1, sensorIR, sensorHiTechnicIRSeeker1200)
#pragma config(Sensor, S2, sensorUltrasonic, sensorSONAR)
#pragma config(Sensor, S3, sensorLight, sensorLightActive)
#pragma config(Motor, mtr_S4_C1_1, motorRight, tmotorTetrix, PIDControl, encoder)
#pragma config(Motor, mtr_S4_C1_2, motorLeft, tmotorTetrix, PIDControl, reversed, encoder)
#pragma config(Motor, mtr_S4_C2_1, motorArm, tmotorTetrix)
#pragma config(Servo, srvo_S4_C3_1, servoGripper, tServoContinuousRotation)

// The Doppler Effect 2013
// Autonomous V2-A1 Source

/* -------------------------------------------------- */

// V2-A1 purpose:
// - Drive up to IR-identified basket (with baskets on the left)
// - Deposit autonomous block
// - Leave with potentially enough time remaining for ally

// V2-A1 setup:
// - Pivot IRSeekerV2 sensor 45 degrees CCW
// - Face direction parallel to baskets

// Used for FCS functions
#include "JoystickDriver.c"

// Used for autonomous-specific functions
#include "AutonomousV2-A1.h"
#include "AutonomousV2-COM.c"

task main() {
	// Configuration functions in AutonomousV2-COM to allow interface from another file
	configureMotors(motorRight, motorLeft);
	configureSensors(sensorIR, sensorUltrasonic, sensorLight);
	
	// Common initialization
	initialize();
	
	// Wait for FCS signal
	waitForStart();
	
	// Begin autonomous routine
	routineA1();
}

void routineA1() {
	
}
