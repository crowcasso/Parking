//*********************************************************
// Solution for Parking Lab
//
// K. Altmann
//*********************************************************

#include <Pololu3piPlus32U4.h>
#include <PololuBuzzer.h>
using namespace Pololu3piPlus32U4;

//**************************************
// State definitions for parking

#define NEAR_WALL 0           // Wall is close to robot
#define FAR_WALL  1           // Wall is far from robot

PololuBuzzer buzzer;          // Help with debugging
int16_t toFrontWall;          // Distance found immediately when you turn
int16_t deepThresh;           // Distances greater than this will be
                              //    deep enough for parking (FAR_WALL)
int16_t shallowThresh;        // Distances smaller than this will be
                              //    considered too shallow to park (NEAR_WALL)
int16_t state = NEAR_WALL;    // The first measurement is guaranteed to not
                              //    be a parking spot
float halfARobot = 4.5;       // The length of half a robot in cm

//***************************************
// lookingForParking assumes you have turned
// and need to drive straight until EITHER
// a spot is found, or a black line is found
// It will then either stop, or park and stop

void lookingForParking() {
  int16_t direction = 1;
  int16_t speed = 75;
  boolean keepDriving = true;
  boolean foundSpot = false;
  int16_t currentDistance;
  int16_t frontEncoderVal;
  int16_t backEncoderVal;

  beginGyroCorrection(speed);     // I want to drive straight

  // drive until we find a spot or a line
  while (keepDriving) {

    if (onBlack()) {                    // If robot is on the black line
      keepDriving = false;              // We should stop driving
      foundSpot = false;                // and there was no spot found
    }
    else {
      currentDistance = measure_distance();   // Measue the distance to the wall

      // The State Machine
      if (state == NEAR_WALL and currentDistance >= deepThresh) { // If you were shallow, but see something deep...
        state = FAR_WALL;
        buzzer.playFrequency(440, 200, 15);      // Let me know you saw the transition
        frontEncoderVal = getEncoderValue();     // Remember where the transition began
      }

      if (state == FAR_WALL and currentDistance <= shallowThresh) { // If you were deep, but see something shallow...
        state = NEAR_WALL;
        buzzer.playFrequency(440, 200, 15);      // Let me know you saw the transition
        backEncoderVal = getEncoderValue();      // Remember where the transition was

        // Convert the difference in tick values to cm
        // If the width of the opening was more than 10 cm...
        if (ticks_to_cm(backEncoderVal - frontEncoderVal) > 10) {
          keepDriving = false;        // We should stop driving
          foundSpot = true;           // and the spot was good
        }
      }

      gyroCorrection(speed, direction);   // Here's where the corrections to drive straight happen
    }
  }

  stopRobot(); // turn off the motors

  if (foundSpot) { // If we found a place, let's park
    delay(200);

    // drive backwards to the center of the spot
    drive_distance(-75, ticks_to_cm(getEncoderValue() - (backEncoderVal + frontEncoderVal) / 2) - halfARobot);

    delay(500);
    currentDistance = measure_distance();     // measure to the wall (in mm) to know how far to drive in
    delay(200);
    turnGyro(90);                             // turn to face the opening
    delay(500);
    drive_distance(50, currentDistance / 10 - 2); // drive in the measured distance (converted to cm)
    //    minus 2 cm to not hit the back wall
  }
}

//************************************************************************

void setup() {
  delay(2000);              // pause 2s for things to settle
  turnSensorSetup();        // initialize the turn sensor
  turnSensorReset();        // makes the current orientation 0
  calibrateLineSensors();   // calibrates the line sensors

  buzzer.playFrequency(440, 200, 15);   // Tell us it is ready to go
  delay(2000);              // pause 2s
}

//************************************************************************

void loop() {
  calibrate10cm();          // This procedure will drive 10 cm and record the highest and lowest light sensor values
  drive_to_line(75);        // Find the line driving at speed 75

  delay(500);
  turnGyro(-90);            // Turn left
  delay(500);

  toFrontWall = measure_distance();        // Measure Distance
  deepThresh = toFrontWall + 100;          // Set thresholds for what will be the near and far walls
  shallowThresh = toFrontWall + 30;

  lookingForParking();                     // Do the main work of looking
  delay(10000);                            // Wait 10 seconds to be picked up and powered off
}
