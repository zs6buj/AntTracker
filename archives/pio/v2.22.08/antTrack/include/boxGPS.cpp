#include <Arduino.h>

#if (HEADINGSOURCE == 4)   //   Trackerbox GPS  
  
  struct compdate {
   uint16_t yyyy;  
   uint8_t mm, dd, h, m, s;
  };

  struct compdate dt = {
    0,0,0,0,0,0};

  TinyGPSPlus tboxGPS;

  uint8_t DaysInMonth(uint8_t mth, uint8_t yr ) {
  uint8_t days;  
    if (mth == 4 || mth == 6 || mth == 9 || mth == 11)
      days = 30;
    else if (mth == 02)
    {
      bool leapyear = (yr % 4 == 0 && yr % 100 != 0) || (yr % 400 == 0);
      if (leapyear == 0)
          days = 28;
      else 
          days = 29;
    }
    else 
    days = 31;
    return days;   
  }
  
  uint32_t getEpoch(struct compdate &dt) {
  uint32_t  epoch;
    epoch = dt.yyyy * (365 * 86400L);  // 24 * 60 * 60 = 86400
    for (int i= 1 ; i<=dt.mm ; i++) {
      epoch += (DaysInMonth(dt.mm, dt.yyyy) * 86400L);
    }    
    epoch += (dt.dd * 86400L);
    epoch += (dt.h * 3600);
    epoch += (dt.m * 60);
    epoch += dt.s;
    return epoch;
  }  
  
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
