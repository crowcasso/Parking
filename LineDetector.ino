//*********************************************************
// Utility funtions for using the IMU Gyroscope sensor
//
// K. Altmann
//*********************************************************

// REMEMBER TO CALL linesensors.calibrate(); in the setup function
// before using any of these

uint16_t lsValues[5];
uint16_t maxLight = 0;
uint16_t minLight = 65535;
boolean calibratedOverLine = false;
uint16_t blackThresh = 3000;
uint16_t whiteThresh = 2000;

LineSensors lineSensors;

//*********************************************************
// Calibrate the line sensor

void calibrateLineSensors() {
  lineSensors.calibrate();
}

//*********************************************************
// Read the line sensor

uint16_t lineReading() {
  lineSensors.read(lsValues);
  return lsValues[2];
}

//*********************************************************
// Is the robot on black?

boolean onBlack() {
  uint16_t current = lineReading();
  if(current >= blackThresh)
    return true;
  return false;
}

//*********************************************************
// Is the robot on white?

boolean onWhite() {
  uint16_t current = lineReading();
  if(current <= whiteThresh)
    return true;
  return false;
}

//*********************************************************
// Drive forward and record the highest and lowest values
// This way, the robot can work in different light conditions
// not just the lights you may have once recorded the values in

void calibrate10cm() {
  // do this forward and slowly
  int16_t speed = 50;
  uint16_t current;
  
  // how many ticks for 10 cm?
  int16_t desiredTicks = cm_to_ticks(10);
  if(desiredTicks > 1000) {
    desiredTicks -= DRIVE_REDUCTION;
  }

  // make sure the encoder starts at 0
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  beginGyroCorrection(speed);

  // wait until we reach our desired number of ticks
  while (abs(encoders.getCountsLeft()) < desiredTicks) {
    gyroCorrection(speed, 1);
    current = lineReading();
    if (current < minLight) {
      minLight = current;
    }
    if (current > maxLight) {
      maxLight = current;
    }
  }
  // turn off the motors
  motors.setSpeeds(0, 0);
  uint16_t difference = (maxLight - minLight);
  blackThresh = minLight + 0.7*difference;
  whiteThresh = minLight + 0.3*difference;
  calibratedOverLine = true;
}
