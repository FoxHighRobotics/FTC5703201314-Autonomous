
// The Doppler Effect 2013
// Autonomous A&B Common Code Header

void initialize();
void resetEncoderValues();
void driveToEncoderValues(int powerLeft, int powerRight, int countsLeft, int countsRight);
void driveToIR(int powerLeft, int powerRight, int irRangeBottom, int irRangeTop);
void driveToUltrasonic(int powerLeft, int powerRight, int distanceRangeBottom, int distanceRangeTop);
void setGripperState(bool state);
