#include <Arduino.h>

#if (HEADINGSOURCE == 4)   //   Trackerbox GPS  

  TinyGPSPlus tboxGPS;
  
  void  getTrackerboxLocation()  {
    uint8_t sats = 0;
    uint16_t yyyy = 0;
    float hdop = 0;
    static bool got_data = false;
    
    while (boxgpsSerial.available() > 0) 
    {
      if (tboxGPS.encode(boxgpsSerial.read())) {
    
        if (tboxGPS.location.isValid())  {
          hom.lat = tboxGPS.location.lat();
          hom.lon = tboxGPS.location.lng();
          hom.alt = tboxGPS.altitude.meters();
          new_boxGPS_data = true;  
          if (tboxGPS.satellites.isValid()) sats = tboxGPS.satellites.value();
          //boxgpsGood = (sats >= 7);     
          boxgpsGood = (hdop < 2.0);  
          boxgpsGood_millis = millis();     // Time of last boxGPS packet
          got_data = true;
        }
        
        if (tboxGPS.hdop.isValid()) {
          hdop = tboxGPS.hdop.value() * 0.01;
          got_data = true;       
        }

        if ( (tboxGPS.date.isValid()) && (tboxGPS.time.isValid()) ) {
          yyyy = tboxGPS.date.year();
          dt.yyyy = yyyy - 1900;  // epoch year
          dt.mm = tboxGPS.date.month();
          dt.dd = tboxGPS.date.day();        
          dt.h = tboxGPS.time.hour();
          dt.m = tboxGPS.time.minute();
          dt.s = tboxGPS.time.second();
          
          epochSync = getEpoch(dt);
          if (epochSync != 0) {
            millisSync = millis();
            timeGood = true;
          }
          got_data = true;    
        }
        if (got_data) {
          got_data = false;

          #if defined DEBUG_ALL || defined DEBUG_BOXGPS
            log.print("Sats: "); log.print(sats);
            if ( (sats) && (new_boxGPS_data) ) {
              log.print("  boxgpsGood: "); log.print(boxgpsGood);        
              log.print("  hom.lat: "); log.print(hom.lat, 7);
              log.print("  ");
              log.print("hom.lon: "); log.print(hom.lon, 7);
              log.print("  hom.alt: "); log.print(hom.alt, 1);
              log.print("  hdop: "); log.print(hdop, 1); 
                  
              /*
              log.print("  yyyy:");  log.print(yyyy);
              log.print("  epoch yyyy:");  log.print(dt.yyyy);          
              log.print("  mm:");  log.print(dt.mm);        
              log.print("  dd:");  log.print(dt.dd);
              log.print("  h:");  log.print(dt.h);
              log.print("  m:");  log.print(dt.m);        
              log.print("  s:");  log.print(dt.s);
              */
              log.print("  UTC Time:");  log.println(TimeString(getEpoch(dt)));
            }
          #endif 
          return;     
        }
        
      }  // read() loop
    }  // available() loop      
  }
  
#endif  // end of TrackerBox GPS - HEADINGSOURCE == 4
