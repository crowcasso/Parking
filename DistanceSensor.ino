//*********************************************************
// Utility funtions for using the distance sensor
//
// K. Altmann
//*********************************************************

const uint8_t DISTANCEPIN = 1;

const uint8_t NODETECTION = -1;
const uint8_t NOPULSE = -2;

//************************************************************************
// reading the Pololu Distance Sensor with Pulse Width Output, 130cm Max

int16_t measure_distance() {

  int16_t t = pulseIn(DISTANCEPIN, HIGH);
  if (t == 0)
  {
    // pulseIn() did not detect the start of a pulse within 1 second.
    return (NOPULSE);
  }
  else if (t > 1850)
  {
    // No detection.
    return (NODETECTION);
  }
  else
  {
    // Valid pulse width reading. Convert pulse width in microseconds to distance in millimeters.
    int16_t d = (t - 1000) * 2;
    
    // Limit minimum distance to 0.
    if (d < 0) {
      d = 0;
    }
    return (d);
  }
}

//************************************************************************
