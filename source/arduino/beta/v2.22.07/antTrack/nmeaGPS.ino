#if (PROTOCOL == 8) || (PROTOCOL == 0)  
  
  // 9600 NMEA GPS
  
  TinyGPSPlus inGPS;          // Instantiate object

  struct compdate {
   uint16_t yyyy;  
   uint8_t mm, dd, h, m, s;
  };

  struct compdate dt = {
    0,0,0,0,0,0};

//====================================================
  void NMEA_GPS_Receive() 
  {    
    uint8_t sats = 0;
    uint16_t yyyy = 0;
    float hdop = 0;
    bool got_data = false;

    while (getNextPacket() > 0) 
    {
      if (inGPS.encode(nextByte() ) ) 
      {
        if (inGPS.location.isValid())  
        {
          cur.lat = inGPS.location.lat();
          cur.lon = inGPS.location.lng();
          cur.alt = inGPS.altitude.meters();
          if (finalHomeStored) {
            cur.alt_ag = cur.alt - hom.alt;
          } else {
            cur.alt_ag = 0;
          }   
          if (inGPS.satellites.isValid()) sats = inGPS.satellites.value();
          gpsGood = (sats >= 8);       // or maybe hdop < 2.0
          new_GPS_data = true;
          gpsGood_millis = millis();     // Time of last good GPS packet     
          got_data = true;
        }
        
        if (inGPS.hdop.isValid()) 
        {
          hdop = inGPS.hdop.value() * 0.01;
          got_data = true;
        }
        
        if (inGPS.date.isValid())  
        {
          yyyy = inGPS.date.year();
          dt.yyyy = yyyy - 1900;  // epoch year
          dt.mm = inGPS.date.month();
          dt.dd = inGPS.date.day();  
          got_data = true;  
        } 
        if (inGPS.time.isValid())  
        {   
          dt.h = inGPS.time.hour();
          dt.m = inGPS.time.minute();
          dt.s = inGPS.time.second();
          epochSync = getEpoch(dt);
          timeGood = true;
          got_data = true;  
        }  
        if (got_data) return;

        
      }  // read() loop
    }  // available() loop        
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
  
#endif  // end of whole NMEA module
