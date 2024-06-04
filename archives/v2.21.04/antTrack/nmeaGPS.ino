  // 9600 NMEA GPS
  
  #include <TinyGPS++.h>

  #define inSerial             Serial1  

  TinyGPSPlus inGPS;          // Instantiate object

  struct compdate {
   uint16_t yyyy;  
   uint8_t mm, dd, h, m, s;
  };

  struct compdate dt = {
    0,0,0,0,0,0};

  #if (MEDIUM_IN == 1) || (MEDIUM_IN == 3)//  NMEA UART or BT Serial in
  // **********************************************************
  void setup_nmeaGPS() 
  {      // this is NOT the Trackebox GPS. See lower down.
    #if defined DEBUG_All || defined DEBUG_inGPS
       log.println("Setup nmeaGPS");
    #endif  
  
    inSerial.end();
    
    #if ( (defined ESP32) || (defined ESP8266) )
      inSerial.begin(inBaud, SERIAL_8N1, in_rxPin, in_txPin);  // likely 9600 for NMEA
    #else
      inSerial.begin(inBaud);  // Serial1 default pins - rx1 tx2 on STM32F1xx
    #endif

  }
  // **********************************************************
  void NMEA_GPS_Receive() 
  {
  #if (PROTOCOL == 8)  || (PROTOCOL == 0)     
    uint8_t sats = 0;
    uint16_t yyyy = 0;
    float hdop = 0;
    bool got_data = false;
    CheckStatusAndTimeouts();

    while (inSerial.available() > 0) 
    {
      if (inGPS.encode(inSerial.read())) 
      {
        if (inGPS.location.isValid())  
        {
          hbGood = true;
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
          hbGood_millis= millis();       // good GPS data is equivalent to a mavlink heartbeat      
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
  #endif          
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

  #endif  // end of MEDIUM_IN == 1 or #  - UART or BT

