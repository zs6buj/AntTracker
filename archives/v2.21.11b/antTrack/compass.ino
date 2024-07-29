
#if (HEADINGSOURCE  == 3) || (HEADINGSOURCE  == 4) // Tracker_Compass or (GPS + Compass)

  // If Compass_Declination not defined in config, fallback to a default value of Johannesburg, South Africa
  #if !(defined Compass_Declination)
    #define Compass_Declination  -18.9 // In degrees   http://www.magnetic-declination.com/ 
  #endif

  #if defined HMC5883L  
    #include <Adafruit_Sensor.h>
    #include <Adafruit_HMC5883_U.h>
    Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
   #elif  defined QMC5883L  
     #include <QMC5883LCompass.h>
     QMC5883LCompass compass;
   #endif

  //=========================================================================
  bool initialiseCompass() 
  {
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
      #if defined DEBUG_ALL || defined DEBUG_BOXCOMPASS
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
    #elif defined QMC5883L  
     initialiseCompass
      compass.init();
      return true;
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
      compass.read();
      val = (float)compass.getAzimuth();  
    #endif
        val += Compass_Declination;  // Add magnetic declination
        fHeading = (float)wrap360((uint16_t)val);
        #if defined BOX_COMPASS_ALIGN        
          //log.printf("heading before align:%3.0fdeg   ", fHeading);
          fHeading = applyCompassAlignment(fHeading, BOX_COMPASS_ALIGN);  // orientation of the compass relative to the box, not the world
         //log.printf("heading after align:%3.0fdeg\n", fHeading);         
        #endif
     
        #if defined DEBUG_ALL || defined DEBUG_BOXCOMPASS
          // Display the results (magnetic vector values are in micro-Tesla (uT)) */
          // log.print("x: "); log.print(x); log.print("  ");
          // log.print("y: "); log.print(y); log.print("  ");
          // log.print("z: "); log.print(z); log.print("  ");log.println("uT");
          log.print("Heading = "); log.println(fHeading,0); 
        #endif 
        return fHeading;      
  }
  //====================================================
  #if  defined HMC5883L
    bool magTimeout() {
      sensors_event_t event;  
      return (!(mag.getEvent(&event)));
    }
  #endif
#endif  // end of whole Compass module  
