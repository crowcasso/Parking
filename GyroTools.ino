//*********************************************************
// Utility funtions for using the IMU Gyroscope sensor
//
// K. Altmann
//*********************************************************

// REMEMBER TO CALL turnSensorSetup(); in the setup function
// before using any of these

#include <Pololu3piPlus32U4IMU.h>
IMU imu;

//*********************************************************
// call before you correct straightness using gyro

void beginGyroCorrection(int16_t speed) {
   turnSensorReset();

  // turn on the motors
  motors.setSpeeds(speed, speed);
}

//*********************************************************
// correct straightness using gyro

#define MINSPEED 50

void gyroCorrection(int16_t speed, int16_t direction) {
    int32_t useOffset = SPEED_OFFSET;

   
    // current angle values
    turnSensorUpdate();
    int32_t heading = getTurnSensorAngle();
    
    int16_t leftSpeed = speed;
    int16_t rightSpeed = speed;

    if (heading != 0) {  
      if (abs(speed) > (MINSPEED + 10)) { 
            // can't do big corrections if speed is small
        useOffset = constrain(map(abs(heading), 1, 90, 1, 10),1,10);     
      }
      
      // headed away from straight ahead, so change the speeds
      if (heading < 0) {
        leftSpeed = leftSpeed - (useOffset * direction);
        rightSpeed = rightSpeed + (useOffset * direction);
      } else if (heading > 0) {
        leftSpeed = leftSpeed + (useOffset * direction);
        rightSpeed = rightSpeed - (useOffset * direction);
      }
    }
    // adjust the speeds
    motors.setSpeeds(leftSpeed, rightSpeed);
}

//*********************************************************
// pivot turn an angle using gyro

void turnGyro(int16_t angle) {
  int16_t direction = sign(angle);  //+ is clockwise; left motor forward
  angle = abs(angle);
  int16_t useSpeed = MINSPEED;

  // we need to turn slightly under the desired amount because the stop is not
  // immediate. These offsets were found experimentally to give a really good
  // 90 degree turn
  int16_t offset;
  if (direction == 1) {
    offset = -4;
  }
  else {
    offset = -5;
  }
  
  turnSensorReset();
  turnSensorUpdate();
  int32_t heading = getTurnSensorAngle();

  // turn until we reach the desired angle
  while (abs(heading) < (angle+offset)) {
    useSpeed = map(angle - abs(heading),1,180,MINSPEED,100);      // ramp the speed to turn faster 
                                                                  // when further from the target
    motors.setSpeeds(useSpeed*direction, -1*useSpeed*direction);
    turnSensorUpdate();
    heading = getTurnSensorAngle();
  }

  // turn off the motors
  stopRobot();
}
