

void  PositionServos(float Az, float El, float hmHdg) {
  
  int llAz = 0;     // Set the limits of the servos here
  int ulAz = 180;    // If the Az limts are exceeded on the Az servo,it goes sulking in the corner
  int llEl = 0;
  int ulEl = 180;
  int MaxPWM = 2400;
  int MinPWM = 650;
/*
  Serial.print("Az = " );
  Serial.print(Az);
  Serial.print("\t El = ");
  Serial.print(El);
  Serial.print("\t hmHdg = " );
  Serial.println(hmHdg);
*/
  int pntAz = pointAz(Az, hmHdg);  // Remap Azimuth into a 180 degree window with home-heading centre, negative behind us
  
  if (pntAz<0) {        // Pointing direction is negative, so it needs to point behind us,
    pntAz = 0-pntAz;    // so flip the frame of reference over and mirror it (neg to pos)
    El = 180-El;        // and make the el servo reach over and behind
  }

  // If the craft moves out of this restricted field of reference, servos should wait at the most recent in-field position
  // This code is not neccessary with two 180 degree servos covering an entire hemisphere

  if ((pntAz>=llAz) && (pntAz<=ulAz) && (El>=llEl) && (El<=ulEl)) {   // Our restricted FOV
    LastGoodpntAz = pntAz;
    LastGoodEl = El;
  }
  // Note my 180 degree servos have a PWM range of 650 through 2400 microseconds - centre 1525
  azPWM = map(LastGoodpntAz, ulAz, llAz, MinPWM, MaxPWM);  // Map the az / el to the servo PWMs
  elPWM = map(LastGoodEl, ulEl, llEl, MinPWM, MaxPWM);   // Servos happen to be mounted such that action is reversed
  
  azServo.writeMicroseconds(azPWM);
  elServo.writeMicroseconds(elPWM);
  
 /*
  Serial.print("pntAz = " );
  Serial.print(pntAz);
  Serial.print(" El = ");
  Serial.print(El);
  Serial.print(" LastGoodpntAz = " );
  Serial.print(LastGoodpntAz);
  Serial.print(" LastGoodEl = ");
  Serial.print(LastGoodEl);
  Serial.print(" hmHdg = " );
  Serial.print(hmHdg);
  Serial.print(" azPWM = " );
  Serial.print(azPWM);
  Serial.print(" elPWM = ");
  Serial.println(elPWM);
*/
}
 //*************************************************** 
  int pointAz(int Azin, int rhmHdg)  {
  // Remap the craft's absolute Az from the home position to the 180 degree azimuth aperture facing us, and behind us
  // 0 degrees on the left and 180 on the right, same but mirrored and negative behind us 180 on the right 0 on the left.
  // Home (antenna) heading is at the centre (90 deg) of the 180 degree forward azimuth aperture
  // pointAz is the heading to the craft from the home position, and thus where to point the antenna, 
  //  RELATIVE to our frame of reference.
  
  int diff;
  int pntAz= Azin;

  /*
  Serial.print("pointAz = ");
  Serial.print(pointAz);
  Serial.print(" rhmHdg = ");
  Serial.print(rhmHdg);
  */

  pntAz = pntAz - rhmHdg + 90;    // Make heading relative to 90 degrees in front of us
  if (pntAz<0) pntAz = 90-pntAz;  // Correct the 90 deg boundary if necessary

  if ((pntAz>180) && (pntAz<=360)) pntAz=180-pntAz;  // Mirror the hemisphere behind us, and make it negative
  if (pntAz>360) pntAz=pntAz-360;   // All the way round to positive territory again

  return pntAz;
}
//***************************************************


