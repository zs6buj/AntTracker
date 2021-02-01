
void  PositionServos(float Az, float El, float hmHdg) {

  // Note my 180 degree servos have a PWM range of 700 through 2300 microseconds 
  // Your's may differ 
  // Uncomment TestServos() in setup() code to observe how well your servos reach their limits

#ifdef Az_Servo_360           // Set the limits of the servos here
  int16_t llAz = 0;           // Az lower limit in degrees
  int16_t ulAz = 360;         // Az upper limit in degrees
  int16_t llEl = 0;           // El lower limit in degrees
  int16_t ulEl = 90;          // El upper limit in degrees
  int16_t MinAzPWM = 650;     // Self explanatory
  int16_t MaxAzPWM = 2400;
  int16_t MinElPWM = 1125;
  int16_t MaxElPWM = 1875;
#else
  int16_t llAz = 0;     
  int16_t ulAz = 180;    
  int16_t llEl = 0;
  int16_t ulEl = 180;
  int16_t MaxAzPWM = 2400;
  int16_t MinAzPWM = 600;
  int16_t MaxElPWM = 2400;
  int16_t MinElPWM = 600;
#endif
  #if defined Debug_All || defined Debug_Servos
  Debug.print("Az = " );
  Debug.print(Az);
  Debug.print("\t El = ");
  Debug.print(El);
  Debug.print("\t hmHdg = " );
  Debug.println(hmHdg);
#endif
  int16_t pntAz = pointAz(Az, hmHdg);  // Remap Azimuth into a 180 degree window with home-heading centre, negative behind us

  #ifndef Az_Servo_360            // Note if NOT. Do this for 180, not for 360 azimuth servo
  if (pntAz<0) {        // Pointing direction is negative, so it needs to point behind us,
    pntAz = 0 - pntAz;   // so flip the frame of reference over and mirror it (neg to pos)
    El = 180 - El;        // and make the el servo reach over and behind
  }
  #endif 
   
  // If the craft moves out of this restricted field of reference, servos should wait at the most recent in-field position
  // This code is not neccessary with two 180 degree servos covering an entire hemisphere

  if ((pntAz>=llAz) && (pntAz<=ulAz) && (El>=llEl) && (El<=ulEl)) {   // Our restricted FOV
    LastGoodpntAz = pntAz;
    LastGoodEl = El;
  }

  azPWM = map(LastGoodpntAz, ulAz, llAz, MinAzPWM, MaxAzPWM);  // Map the az / el to the servo PWMs
  elPWM = map(LastGoodEl, ulEl, llEl, MinElPWM, MaxElPWM);     // Servos happen to be mounted such that action is reversed
  
  azServo.writeMicroseconds(azPWM);
  elServo.writeMicroseconds(elPWM);
  
  #if defined Debug_All || defined Debug_Servos
  Debug.print("pntAz = " );
  Debug.print(pntAz);
  Debug.print(" El = ");
  Debug.print(El);
  Debug.print(" LastGoodpntAz = " );
  Debug.print(LastGoodpntAz);
  Debug.print(" LastGoodEl = ");
  Debug.print(LastGoodEl);
  Debug.print(" hmHdg = " );
  Debug.print(hmHdg);
  Debug.print(" azPWM = " );
  Debug.print(azPWM);
  Debug.print(" elPWM = ");
  Debug.println(elPWM);
  #endif
  
}
 //*************************************************** 
  int16_t pointAz(int16_t Azin, int16_t rhmHdg)  {
  // Remap the craft's absolute Az from the home position to the 180 degree azimuth aperture facing us, and behind us
  // 0 degrees on the left and 180 on the right, same but mirrored and negative behind us 180 on the right 0 on the left.
  // Home (antenna) heading is at the centre (90 deg) of the 180 degree forward azimuth aperture
  // pointAz is the heading to the craft from the home position, and thus where to point the antenna, 
  //  RELATIVE to our frame of reference.
  
  int16_t pntAz= Azin;

  #if defined Debug_All || defined Debug_Servos
//  Debug.print("pointAz = ");
//  Debug.print(pntAz);
//  Debug.print(" rhmHdg = ");
//  Debug.println(rhmHdg);
  #endif

  pntAz = pntAz - rhmHdg + 90;    // Make heading relative to 90 degrees in front of us

  #ifdef Az_Servo_360     // conditional patch by mric3412
    if (pntAz < 0)  pntAz = 90 - pntAz;
  #endif
  
  #ifndef Az_Servo_360            // Note if not. Do this for 180, not for 360 azimuth servo
    if (pntAz < 0)                // Patch by mric3412 for 360 servos
    {
      if (rhmHdg <= 90) pntAz = 90 - pntAz;
      if (rhmHdg > 90 and rhmHdg <= 180) pntAz = - 180 - pntAz;
      if (rhmHdg > 180 and rhmHdg <= 270) pntAz = pntAz - 180;
      if (rhmHdg > 270) pntAz = 360 + pntAz;
    }
    if ((pntAz>180) && (pntAz<=360)) pntAz=180-pntAz;  // Mirror the hemisphere behind us, and make it negative
    if ((pntAz<-180) && (pntAz>=-360)) pntAz=-360-pntAz;
  #endif  
    
  if (pntAz>360) pntAz=pntAz-360;   // All the way round to positive territory again
  if (pntAz<-360) pntAz=pntAz+360;  // patch by mric3412

  
  return pntAz;
}
//***************************************************
