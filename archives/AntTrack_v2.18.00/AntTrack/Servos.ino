
#if (defined STM32F103C) || (defined MAPLE_MINI)     // NOTE: ONLY STM32 library uses PWM writeMicroseconds command
  #ifdef Az_Servo_360           // Set the limits of the servos here
    int16_t llAz = 0;           // Az lower limit in degrees
    int16_t ulAz = 360;         // Az upper limit in degrees
    int16_t llEl = 0;           // El lower limit in degrees
    int16_t ulEl = 90;          // El upper limit in degrees
  #else
    int16_t llAz = 0;     
    int16_t ulAz = 180;    
    int16_t llEl = 0;
    int16_t ulEl = 180;
  #endif
#endif

void pointServos(uint16_t worldAz, uint16_t ourEl, uint16_t boxHdg) {

  // Default pwm values for SG90 servos: 500 and 2400, adjust if necessary
  uint16_t minUs = 615;   // my adjusted values
  uint16_t maxUs = 2257;  

  if (worldAz == 360) worldAz = 0;   // our servo azimuth working range is 0 deg through 359 deg

  #if defined Debug_All || defined Debug_Servos
    Log.print("\nworldAz = " );
    Log.print(worldAz);
    Log.print("\t ourEl = ");
    Log.print(ourEl);
    Log.print("\t boxHdg = " );
    Log.println(boxHdg);
  #endif 
  
  const int16_t antOffset = 90;                     // antenna ofsset from box, zero degrees == left, 90 deg straight ahead
  
  // Remap tracker box azimuth to world compass azimuth
  int16_t ourAz = worldAz - boxHdg - antOffset;    // here we compensate for offset of the box to worldview, and the 
  ourAz = wrap360(ourAz);

  #if not defined Az_Servo_360                // If 180 deg az servo
    if ( (ourAz > 180) && (ourAz <= 360) ) {  // Pointing direction is behind us,
      //Log.printf("before ourAz:%d ourEl:%d \n", ourAz, ourEl);
      ourAz -= 180;                           // so flip the frame of reference over and mirror it 
      ourEl = 180 - ourEl;                    // and make the el servo reach over and behind
      //snprintf(snprintf_buf, snp_max, "after ourAz:%d ourEl:%d \n", ourAz, ourEl);            
      //Log.print(snprintf_buf);  
    }
  #endif 

  moveServos(ourAz, ourEl);
  
  #if defined Debug_All || defined Debug_Servos
    Log.print("ourAz = " );
    Log.print(ourAz);
    Log.print(" ourEl = ");
    Log.print(ourEl);
    Log.print(" boxHdg = " );
    Log.print(boxHdg);
    Log.print(" azPWM = " );
    Log.print(azPWM);
    Log.print(" elPWM = ");
    Log.println(elPWM);
  #endif
}

//=====================================================
void moveServos(uint16_t az, uint16_t el) {
  
  // Reverse servo action if necessary
  #if defined Az_Servo_360   
    #if defined ReverseAzimuth    
      az = 360 - az;
    #endif 
    #if defined ReverseElevation    
      el = 360 - el;
    #endif      
  #else
    #if defined ReverseAzimuth    
      az = 180 - az;
    #endif 
    #if defined ReverseElevation    
      el = 180 - el;
    #endif     
  #endif
  
  #if (defined TEENSY3X) || (defined ESP32) || (defined ESP8266) // Degrees     NOTE: myservo.attach(pin, 1000, 2000);
    azServo.write(az);
    elServo.write(el);
  #elif (defined STM32) || (defined MAPLE)                       // Microseconds
    azPWM = map(az, minAz, maxAz, minAzPWM, maxAzPWM);        // Map the az/el degrees to the servo PWMs 
    elPWM = map(el, maxEl, minEl, maxElPWM, minElPWM);        
    azServo.writeMicroseconds(azPWM);
    elServo.writeMicroseconds(elPWM);
  #endif
  

}
//=====================================================
void TestServos() {
  moveServos(0, 0);  // az, el
  delay(4000); 
  moveServos(90, 0);
  delay(4000); 
  moveServos(90, 90);
  delay(4000);
  moveServos(90, 0);
  delay(4000);      
  moveServos(180, 0);
  delay(4000);       
  moveServos(90, 0);
  delay(4000);
  moveServos(90, 180);
  delay(4000);
  moveServos(90, 0);
}
