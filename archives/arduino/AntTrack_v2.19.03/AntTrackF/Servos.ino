
void pointServos(int16_t worldAz, int16_t ourEl, int16_t boxHdg) {

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
      //Log.printf("after ourAz:%d ourEl:%d \n", ourAz, ourEl);            
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

  //NOTE: myservo.attach(pin, 1000, 2000)
  azServo.write(az);
  elServo.write(el);
    

  

}
//=====================================================
void TestServos() {
  moveServos(0, 0);  // az, el
  delay(1500); 
  moveServos(90, 0);
  delay(1500); 
  moveServos(90, 90);
  delay(1500);
  moveServos(90, 0);
  delay(1500);      
  moveServos(180, 0);
  delay(1500);       
  moveServos(90, 0);
  delay(1500);
  moveServos(90, 180);
  delay(1500);
  moveServos(90, 0);
}
