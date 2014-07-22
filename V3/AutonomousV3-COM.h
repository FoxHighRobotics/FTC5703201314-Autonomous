
// The Doppler Effect 2013-2014
// Autonomous V3-COM Header
// Designed for FTC Block Party 2013-2014

/* -------------------------------------------------- */

// Duration of delay within while loops (in milliseconds)
#define LOOP_DELAY 20

// Ratio of encoder counts to meters travelled
// Latest calibration: January 10, 2014
#define ENCODER_RATIO_DRIVE_LEFT 7400
#define ENCODER_RATIO_DRIVE_RIGHT 7200

// Durations of movements for gripper servo to open and close (in milliseconds)
#define GRIPPER_OPEN_DURATION 2000

// Whether or not to enable the forward collision avoidance subsystem
#define ENABLE_COLLISION_AVOIDANCE true

// The threshold value for activating the collision avoidance subsystem
#define COLLISION_AVOIDANCE_THRESHOLD 15

// Function prototypes
void configureSensors(short paramSensorIR, short paramSensorSonar);
void configureMotors(short paramMotorDriveLeft, short paramMotorDriveRight, short paramMotorArmPitch, short paramMotorElevator);
void configureServos(short paramServoArmGripper);
void initialize();
void openGripper();
void closeGripper();
void driveWithEncoderGoal(int powerLeft, int powerRight, int countsLeft, int countsRight, int thresholdLeft, int thresholdRight);
void driveWithRangedEncoderGoal(int powerLeft, int powerRight, int countsLeftLow, int countsRightLow, int countsLeftHigh, int countsRightHigh);
void driveWithSensorGoal(int powerLeft, int powerRight, short sensor, int value, int threshold);
void driveWithRangedSensorGoal(int powerLeft, int powerRight, short sensor, int rangeLow, int rangeHigh);
int promptDelay();
task collisionAvoidance();
