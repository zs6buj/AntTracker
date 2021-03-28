  // 9600 NMEA
#if ( (Telemetry_In == 0) || (Heading_source == 4) )  //  Serial in or have Trackerbox GPS  
  
  #include <TinyGPS++.h>

  #define inSerial             Serial1  

#endif

#if (Telemetry_In == 0) //  Serial in

  TinyGPSPlus inGPS;

  struct compdate {
   uint16_t yyyy;  
   uint8_t mm, dd, h, m, s;
  };

  struct compdate dt = {
    0,0,0,0,0,0};

  // **********************************************************
  void Setup_inGPS() {
    #if defined Debug_All || defined Debug_inGPS
       Log.println("Setup inGPS");
    #endif  
  
    inSerial.end();

    inSerial.begin(inBaud, SERIAL_8N1, in_rxPin, in_txPin);  // likely 9600 for NMEA
  }
  // **********************************************************
  void GPS_Receive() {
  uint8_t sats = 0;
  uint16_t yyyy = 0;
  float hdop = 0;
  bool got_data = false;
  CheckForTimeouts();

  while (inSerial.available() > 0) {
    if (inGPS.encode(inSerial.read())) {
  
      if (inGPS.location.isValid())  {
        hbGood = true;
        cur.lat = inGPS.location.lat();
        cur.lon = inGPS.location.lng();
        cur.alt = inGPS.altitude.meters();

        if (inGPS.satellites.isValid()) sats = inGPS.satellites.value();
        gpsGood = (sats >= 8);       // or maybe hdop < 2.0
        new_GPS_data = true;
        gpsGood_millis = millis();     // Time of last good GPS packet
        got_data = true;
      }
       
      if (inGPS.hdop.isValid()) {
        hdop = inGPS.hdop.value() * 0.01;
        got_data = true;
      }
      
      if (inGPS.date.isValid())  {
        yyyy = inGPS.date.year();
        dt.yyyy = yyyy - 1900;  // epoch year
        dt.mm = inGPS.date.month();
        dt.dd = inGPS.date.day();  
        got_data = true;  
      } 
      if (inGPS.time.isValid())  {   
        dt.h = inGPS.time.hour();
        dt.m = inGPS.time.minute();
        dt.s = inGPS.time.second();
        epochSync = getEpoch(dt);
        timeGood = true;
        got_data = true;  
        got_data = true;  
      }  
      if (got_data) return;

       
    }  // read() loop
  }  // available() loop
      
  if (headingSource==1 && (gpsGood) && (!homeInitialised) && (!homSaved)) AutoStoreHome();  // Only need this when headingSource is flight GPS 
         
  }

  //====================================================
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

    //====================================================
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
 
#endif // end of inGPS

//====================================================
//====================================================
#if (Heading_Source == 4)

TinyGPSPlus tboxGPS;

void  getTrackerboxLocation()  {
  uint8_t sats = 0;
  uint16_t yyyy = 0;
  float hdop = 0;
  static bool got_data = false;
  
  while (gpsSerial.available() > 0) {
    if (tboxGPS.encode(gpsSerial.read())) {
  
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

        #if defined Debug_All || defined Debug_boxGPS
          Log.print("Sats: "); Log.print(sats);
          if ( (sats) && (new_boxGPS_data) ) {
            Log.print("  boxgpsGood: "); Log.print(boxgpsGood);        
            Log.print("  hom.lat: "); Log.print(hom.lat, 7);
            Log.print("  ");
            Log.print("hom.lon: "); Log.print(hom.lon, 7);
            Log.print("  hom.alt: "); Log.print(hom.alt, 1);
            Log.print("  hdop: "); Log.print(hdop, 1); 
                 
            /*
            Log.print("  yyyy:");  Log.print(yyyy);
            Log.print("  epoch yyyy:");  Log.print(dt.yyyy);          
            Log.print("  mm:");  Log.print(dt.mm);        
            Log.print("  dd:");  Log.print(dt.dd);
            Log.print("  h:");  Log.print(dt.h);
            Log.print("  m:");  Log.print(dt.m);        
            Log.print("  s:");  Log.print(dt.s);
            */
            Log.print("  UTC Time:");  Log.println(TimeString(getEpoch(dt)));
          }
        #endif 
        return;     
      }
       
    }  // read() loop
  }  // available() loop      
}
#endif
        
