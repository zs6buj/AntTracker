

void  PositionServos(float Az, float El, float hmHdg) {
 
  int azPWM = 0;
  int elPWM = 0;

  int RemAz = RemappedAz(Az, hmHdg);  // Remap Azimuth into a 180 degree window with home-heading centre
  /*
      Serial.print("hmHdg = ");
      Serial.print(hmHdg);
      Serial.print("  AzIn = ");
      Serial.print(Az);
      Serial.print("    ");
      Serial.print("AzOut = ");
      Serial.println(RemAz);
    */
  azPWM = map(RemAz, 180, 0, 1000, 2000);  // Map the az / el to the servo PWMs
  elPWM = map(El, 90, 0, 1000, 2000);   // Servos happen to be mounted such that action is reversed
  
  azServo.writeMicroseconds(azPWM);
  elServo.writeMicroseconds(elPWM);
/*
  Serial.print("Az = " );
  Serial.print(Az,0);
  Serial.print("\t El = ");
  Serial.print(El);
  Serial.print("\t hmHdg = " );
  Serial.print(hmHdg);
  Serial.print("\t azPWM = " );
  Serial.print(azPWM);
  Serial.print("\t elPWM = ");
  Serial.println(elPWM);
  */
}
 //*************************************************** 
  int RemappedAz(int rAz, int rhmHdg)  {
  // Remap our Az and home heading to the 180 degree azimuth aperture facing us
  // Home heading is at the centre (90 deg) of the 180 degree azimuth aperture
  int diff;
  // Shift the degrees up to remove the 360 degree boundary, the differnce remains the same
  rAz +=360;
  rhmHdg+=360;

  if (rAz > rhmHdg)
    diff = rAz - rhmHdg;
  else
    diff = rhmHdg - rAz;  
  if (diff >180) exit;   // Ignore results outside our window
  
  rAz = rAz - rhmHdg + 90; 
  
  if (rAz<0) rAz = 90-rAz;  // Correct the 90 deg boundary if necessary
  return rAz;
  }
//***************************************************


