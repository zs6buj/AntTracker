#include <Arduino.h>

#if (PROTOCOL == 8) || (PROTOCOL == 0)  
  // 9600 NMEA GPS
  
  TinyGPSPlus inGPS;          // Instantiate object

//====================================================
  void nmeaGPS_Receive() 
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
          if ((finalHomeStored) || (headingSource == 4))  
          {
            cur.alt_ag = cur.alt - hom.alt;
          } else 
          {
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
  
#endif  // end of whole NMEA module
