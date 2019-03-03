#if (Heading_Source == 3)  // Tracker_Compass   //  else don't compile this module
#include <Wire.h>
#include <QMC5883L.h>

QMC5883L compass;

#ifndef Debug
  #define Debug               Serial         // USB 
#endif

// Get the mag declination of your location here http://www.magnetic-declination.com/ and insert in # defined in next line
#define DECLINATION  -18.9 // In degrees
 

//******************************************************************************
uint8_t Initialise_Compass() {

  // There is no check for missing magnetometer in this library. Possibly scan for known address
  
  Wire.begin();
  Serial.begin(9600);
  compass.init();

 return 1;
}

//******************************************************************************
float GetMagHeading() {
  int x,y,z;
  compass.read(&x,&y,&z);
  
  float fHeading = RadToDeg(atan2(y, x));  //All in degrees now
  
  fHeading += DECLINATION;  // Add magnetic declination

  fHeading = Normalise_360(fHeading);
  
  #if defined Debug_All || defined Debug_Compass
    /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
 //   Debug.print("X: "); Debug.print(x); Debug.print("  ");
 //   Debug.print("Y: "); Debug.print(y); Debug.print("  ");
 //   Debug.print("Z: "); Debug.print(z); Debug.println("  ");
    Debug.print("Heading = "); Debug.println(fHeading,0); 
  #endif   

  return fHeading;
}

//******************************************************************************

int16_t Normalise_360(int16_t arg) {
  if (arg < 0) arg += 360;
  if (arg > 359) arg -= 360;
  return arg;
}
//***************************************************
float RadToDeg (float _Rad) {
  return _Rad * 180 / PI;  
}
//***************************************************
#endif
