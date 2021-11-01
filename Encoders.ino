//*********************************************************
// Utility funtions for using motors and encoders
// 
// K. Altmann
//*********************************************************
 
#include <Pololu3piPlus32U4.h>
using namespace Pololu3piPlus32U4;

Motors motors;
Encoders encoders;

#define sign(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))
#define SPEED_OFFSET 3
#define DRIVE_REDUCTION 80

// conversion from cm to ticks
const float ticks_per_rotation = 358.3;
const float wheel_diameter_cm = 3.2;
const float cm_per_rotation = PI * wheel_diameter_cm;
const float ticks_per_cm = ticks_per_rotation / cm_per_rotation;

//*********************************************************
// convert cm to encoder ticks (3pi+ 30:1 motors)

int16_t cm_to_ticks(float cm) {
 return (int16_t) (ticks_per_cm * abs(cm));
}

//*********************************************************
// convert encoder ticks to cm (3pi+ 30:1 motors)

float ticks_to_cm(int16_t ticks) {
 return (ticks / ticks_per_cm);
}

//*********************************************************
// drive for a distance using encoders

void drive_distance(int16_t speed, float distance_cm) {

  // deal with forward/backward
  int16_t direction = sign(speed) * sign(distance_cm);
  speed = direction * abs(speed);

  // how many ticks for the number of cm?
  int16_t desiredTicks = cm_to_ticks(distance_cm);
  if(desiredTicks > 1000) {
    desiredTicks -= DRIVE_REDUCTION;
  }

  // make sure the encoder starts at 0
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  beginGyroCorrection(speed);

  // wait until we reach our desired number of ticks
  while (abs(encoders.getCountsLeft()) < desiredTicks) {
    gyroCorrection(speed, direction);
  }

  // turn off the motors
  stopRobot();
}

//*********************************************************
// drive to a black line
// using Gyro correction currently, but
// could easily swap to using EncoderCorrection

void drive_to_line(int16_t speed) {
  int16_t direction = sign(speed);
  
  beginGyroCorrection(speed);           // Setup for the correction

  // drive until we reach the black line
  while (!onBlack()) {
    gyroCorrection(speed, direction);   // This is where the correction happens
  }

  // turn off the motors
  stopRobot();
}

//*********************************************************
// call before you correct straightness using encoders

int16_t prevLeftCount = 0;
int16_t prevRightCount = 0;

void beginEncoderCorrection(int16_t speed) {
  // reset the previous encoder values
  prevLeftCount = 0;
  prevRightCount = 0;

  // turn on the motors
  motors.setSpeeds(speed, speed);
}

//*********************************************************
// correct straightness using encoders

void encoderCorrection(int16_t speed, int16_t direction) {

    // current encoder values
    int16_t leftCount = encoders.getCountsLeft();
    int16_t rightCount = encoders.getCountsRight();

    // how many ticks since last loop?
    int16_t leftDiff = abs(leftCount - prevLeftCount);
    int16_t rightDiff = abs(rightCount - prevRightCount);

    int16_t leftSpeed = speed;
    int16_t rightSpeed = speed;
    
    // if one side has ticked more, slow it down and speed up the other
    if (leftDiff > rightDiff) {
      leftSpeed = leftSpeed - (SPEED_OFFSET * direction);
      rightSpeed = rightSpeed + (SPEED_OFFSET * direction);
    } else if (leftDiff < rightDiff) {
      leftSpeed = leftSpeed + (SPEED_OFFSET * direction);
      rightSpeed = rightSpeed - (SPEED_OFFSET * direction);
    }

    // adjust the speeds
    motors.setSpeeds(leftSpeed, rightSpeed);

    // remember the counts for the next loop
    prevLeftCount = leftCount;
    prevRightCount = rightCount;
}

//*********************************************************
// drive based on time
// obsolete

void drive_time(int16_t speed, unsigned long millis) {
  motors.setSpeeds(speed, speed);
  delay(millis);
  stopRobot();
}

//*********************************************************
// gets encoder value; a utility function so other files can read
// encoders without referencing them directly

int16_t getEncoderValue() {
  return(encoders.getCountsLeft());
}

//*********************************************************
// stops robot; a utility function so other files can stop
// the robot without referencing motors directly

void stopRobot() {
  motors.setSpeeds(0, 0);
}
