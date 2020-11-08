
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

// Get the mag declination of your location here http://www.magnetic-declination.com/ and insert in # defined in next line
#define DECLINATION  -18.9 // In degrees
 
 Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

//******************************************************************************
uint8_t Initialise_Compass() {

  if(!mag.begin()) {
    Debug.println("No HMC5883 compass found!");
    return 0;
  }

  #if defined Debug_All || defined Debug_Compass
    sensor_t sensor;
    mag.getSensor(&sensor);
    Debug.println("----- Compass Found -----");
    Debug.print  ("Sensor:       "); Debug.println(sensor.name);
    Debug.print  ("Driver Ver:   "); Debug.println(sensor.version);
    Debug.print  ("Unique ID:    "); Debug.println(sensor.sensor_id);
    Debug.print  ("Max Value:    "); Debug.print(sensor.max_value); Debug.println(" uT");
    Debug.print  ("Min Value:    "); Debug.print(sensor.min_value); Debug.println(" uT");
    Debug.print  ("Resolution:   "); Debug.print(sensor.resolution); Debug.println(" uT");  
    Debug.println("--------------------------");
    Debug.println("");
 #endif

 return 1;
}

//******************************************************************************
float GetMagHeading() {
  
  // Read Magnetometer
  sensors_event_t event; 
  mag.getEvent(&event);
  
  float fHeading = RadToDeg(atan2(event.magnetic.y, event.magnetic.x));  //All in degrees now
  
  fHeading += DECLINATION;  // Add magnetic declination

  fHeading = Normalise_360(fHeading);
  
  #if defined Debug_All || defined Debug_Compass
    /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
 //   Debug.print("X: "); Debug.print(event.magnetic.x); Debug.print("  ");
 //   Debug.print("Y: "); Debug.print(event.magnetic.y); Debug.print("  ");
 //   Debug.print("Z: "); Debug.print(event.magnetic.z); Debug.print("  ");Debug.println("uT");
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
