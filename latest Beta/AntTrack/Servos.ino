
void  PositionServos(uint16_t worldAz, uint16_t ourEl, uint16_t boxHdg) {

  if (worldAz == 360) worldAz = 0;   // our servo azimuth working range is 0 deg through 359 deg

  #if defined Debug_All || defined Debug_Servos
    Log.print("\nworldAz = " );
    Log.print(worldAz);
    Log.print("\t ourEl = ");
    Log.print(ourEl);
    Log.print("\t boxHdg = " );
    Log.println(boxHdg);
  #endif 
  const int16_t antOffset = 90;                     // degrees, antenna offset to box
  // Remap tracker box azimuth to world compass azimuth
  int16_t ourAz = worldAz - boxHdg - antOffset;    // here we compensate for offset of the box to worldview, and the 
  ourAz = wrap360(ourAz);

  #if not defined Az_Servo_360                 // If 180 deg az servo
    if ( (ourAz > 180) && (ourAz <= 360) ) {  // Pointing direction is behind us,
       //Log.printf("before ourAz:%d ourEl:%d \n", ourAz, ourEl);
      ourAz -= 180;                    // so flip the frame of reference over and mirror it 
      ourEl = 180 - ourEl;                        // and make the el servo reach over and behind
      //Log.printf("after ourAz:%d ourEl:%d \n", ourAz, ourEl);
    }
  #endif 


  #if (defined TEENSY3X)      // NOTE: myservo.attach(pin, 1000, 2000);
    azServo.write(ourAz);
    elServo.write(ourEl);
  #else
    azPWM = map(ourAz, minAz, maxAz, maxAzPWM, minAzPWM);     // Map the az / el to the servo PWMs
    elPWM = map(ourEl, minEl, maxEl, maxElPWM, minElPWM);     // my servos happen to be mounted such that action is reversed  
    azServo.writeMicroseconds(azPWM);
    elServo.writeMicroseconds(elPWM);
  #endif
  
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
void TestServos() {

  PositionServos(90, 0, 90); 
  for (int i=0; i<=360; i+=30) {  // 360 == 0 again
    delay(60);
    PositionServos(i, 30, 90);   
    }
  for (int i=0; i<=90; i+=2) {
    delay(60);
    PositionServos(90, i, 90);   
    }
   PositionServos(90, 0, 90);   
   
}