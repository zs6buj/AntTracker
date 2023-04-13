
#if (Heading_Source  == 3) || (Heading_Source  == 4) // Tracker_Compass or (GPS + Compass)

  // If Compass_Declination not defined in config, fallback to a default value of Johannesburg, South Africa
  #if !(defined Compass_Declination)
    #define Compass_Declination  -18.9 // In degrees   http://www.magnetic-declination.com/ 
  #endif

  #if defined HMC5883L  
    #include <Adafruit_Sensor.h>
    #include <Adafruit_HMC5883_U.h>
    Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
   #elif  defined QMC5883L  
    // no libs - we do it the long way 
   #endif


  //=========================================================================
  bool initialiseCompass() {
  #if ( (defined ESP32) || (defined ESP8266) )
    Wire.begin(SDA, SCL);
  #else
    Wire.begin();   // default pins are defined in Wire.h 
  #endif
  
  #if defined HMC5883L  
    if(!mag.begin()) {
      log.println("No HMC5883 compass found!");
      return false;
    }
    if (magTimeout()) {       // this test requires my patched Adafruit_HMC5883_U.cpp
      log.println("No HMC5883 compass found! No box orientation!");
      LogScreenPrintln("No compass!");     
      return false;      
    }
    #if defined Debug_All || defined Debug_boxCompass
        sensor_t sensor;
        mag.getSensor(&sensor);
        log.println("----- Compass Found -----");
        log.print  ("Sensor:       "); log.println(sensor.name);
        log.print  ("Driver Ver:   "); log.println(sensor.version);
        log.print  ("Unique ID:    "); log.println(sensor.sensor_id);
        log.print  ("Max Value:    "); log.print(sensor.max_value); log.println(" uT");
        log.print  ("Min Value:    "); log.print(sensor.min_value); log.println(" uT");
        log.print  ("Resolution:   "); log.print(sensor.resolution); log.println(" uT");  
        log.println("--------------------------");
        log.println("");
    #endif  
    return true;  
    
  #elif  defined QMC5883L   
    I2C_write_AddrDev_AddrReg_Byte(0x0d,0x0b,1);
    I2C_write_AddrDev_AddrReg_Byte(0x0d,0x09,B00000001);
    
    if (!QMC5883L_Ready()) {
      log.print("QMC5883L not ready! Retrying");
    }
    uint8_t retry = 20;
    while ((!QMC5883L_Ready()) && (retry) ) {   
      log.printf(" %d", retry);
      retry--;
      delay(100);
    }
    if (QMC5883L_Ready()) {
      log.println("QMC5883L compass ready");
      LogScreenPrintln("Compass ready");   
      return true;
    } else {
      log.println("\nFailed. No box orientation!"); 
      LogScreenPrintln("No box compass!");        
      return false;
    }
  #endif

  }

  //====================================================
  float getTrackerboxHeading() {
    float fHeading = 0.0;
    float val = 0.0;
    
    #if defined HMC5883L
      sensors_event_t event; 
      mag.getEvent(&event);
      val = 180/M_PI * atan2(event.magnetic.y, event.magnetic.x);  // Degrees   
      
    #elif defined QMC5883L 
      int16_t x,y,z;           // Raw compass output values
      int16_t offx = 0;        //calibration offsets (future)
      int16_t offy = 0;  
      if (getQMC5883L(&x, &y, &z) ) {
        val = 180/M_PI * atan2((float)(x-offx),(float)(y-offy));  // Degrees    
    #endif
        
        val += Compass_Declination;  // Add magnetic declination
        fHeading = (float)wrap360((uint16_t)val);
        #if defined BOX_COMPASS_ALIGN        
          //log.printf("heading before align:%3.0fdeg   ", fHeading);
          fHeading = applyCompassAlignment(fHeading, BOX_COMPASS_ALIGN);
         //log.printf("heading after align:%3.0fdeg\n", fHeading);         
        #endif
     
        #if defined Debug_All || defined Debug_boxCompass
          // Display the results (magnetic vector values are in micro-Tesla (uT)) */
          // log.print("x: "); log.print(x); log.print("  ");
          // log.print("y: "); log.print(y); log.print("  ");
          // log.print("z: "); log.print(z); log.print("  ");log.println("uT");
          log.print("Heading = "); log.println(fHeading,0); 
        #endif 
          
        return fHeading;   

    #if defined QMC5883L     
      } else {
        log.println("Compass not ready ");
        return 0.0;
      }
    #endif    
  }

  #if  defined QMC5883L
  //====================================================
  
  bool QMC5883L_Ready() {
     byte stat,ovfl,skipped;
     I2C_write_AddrDev_AddrReg_Byte(0x0d,0x0b,1);
     Wire.beginTransmission(0x0d); // Read from status reg
     Wire.write(0x06);
     int num = Wire.requestFrom((byte)0x0d, (byte)1);      // address, quantity
     stat = Wire.read(); // DOR Data out Ready (SKIPPED).   
     Wire.endTransmission();
     if (stat == 0xff) return false;
     ovfl    = stat & 0x02;
     skipped = stat & 0x04;
     bool rdy = (stat & 0x01) == 1;
     //log.printf("num:%d  stat:%d  rdy:%d \n", num, stat, rdy);
     return rdy; 
  }

  //====================================================

  bool getQMC5883L(int16_t *x, int16_t *y, int16_t *z) {
     if ( !getQMC5883LRaw(x, y, z) ) return false;
     int tmp = *y;
     *y = -*x;     // x is down.
     *x = tmp;     // y is to the right.
     return true;
  }
  //====================================================
  
  bool getQMC5883LRaw(int16_t *x, int16_t *y, int16_t *z) {  // If data is not ready x,y,z are not changed.

   if ( !QMC5883L_Ready() ) return false;

   Wire.beginTransmission(0x0d);
   Wire.write(0x00);     // read from address zero = x,y,z registers.
   int16_t err = Wire.endTransmission();

   if (!err) {
      Wire.requestFrom((byte)0x0d, (byte)6); //Blocking?
      while(Wire.available()<6); //Wait if above blocking then this not needed.
      *x  = (int16_t)(Wire.read() | Wire.read() << 8);
      *y  = (int16_t)(Wire.read() | Wire.read() << 8);
      *z  = (int16_t)(Wire.read() | Wire.read() << 8);
   }
   return true;
  }

  //====================================================

  void I2C_write_AddrDev_AddrReg_Byte(byte i2cAddr, byte regaddr, byte d ) {
     Wire.beginTransmission(i2cAddr);
     Wire.write(regaddr);
     Wire.write(d);
     Wire.endTransmission();
  }
  
#endif

  //====================================================
  #if  defined HMC5883L
    bool magTimeout() {
      sensors_event_t event;  
      return (!(mag.getEvent(&event)));
    }
  #endif

#endif  // end of whole Compass module  
