#include <Arduino.h>

#if (PROTOCOL == 9) || (PROTOCOL == 0)  
  void crsfReceive() 
  {
  #if (MEDIUM_IN  == 1) || (MEDIUM_IN  == 3)     // UART or BT select

    if (crsf.readCrsfFrame(crsf.frame_lth))      // exposes discovered frame_lth if needed
    {
      uint8_t len = crsf.frame_lth;
  #endif

  #if (MEDIUM_IN  == 2) // UDP selected
    uint16_t len = readUDP();
    if (len) 
    {
  #endif
  #if (MEDIUM_IN  == 5) // ESPNOW selected
    if (espnow_received)  // flag
    {
      espnow_received = false;
      int16_t len = espnow_len;
  #endif
      telem_millis = millis();
      telemGood = true;
      #if defined DEBUG_CRSF_BUF
        printBytes(&*crsf.crsf_buf, len);
      #endif
      uint8_t crsf_id = crsf.decodeTelemetry(&*crsf.crsf_buf, len);
      //log.printf("crsf_id:%2X\n", crsf_id);
      if (crsf_id == GPS_ID)   // 0x02
      {
        // don't use gps heading, use attitude yaw below
        cur.lon = crsf.gpsF_lon;
        cur.lat = crsf.gpsF_lat;    
        cur.alt = crsf.gps_altitude;
        gpsfixGood = (crsf.gps_sats >=5);  // with 4 sats, altitude value can be bad

        new_GPS_data = hdgGood = altGood = lonGood = latGood = gpsfixGood;
    
        if ((finalHomeStored) || (headingsource == 4))  
        {
          cur.alt_ag = cur.alt - hom.alt;
        } else 
        {
          cur.alt_ag = 0;
        }           
        hud_num_sats = crsf.gps_sats;         // these for the display
        hud_grd_spd = crsf.gpsF_groundspeed;
       // hud_rssi
        //motArmed
       // pt_home_dist
       // hud_climb 
  #if defined DEBUG_CRSF_GPS          
        log.print("GPS id:");
        crsf.printByte(crsf_id, ' ');
        log.printf("lat:%2.7f  lon:%2.7f", crsf.gpsF_lat, crsf.gpsF_lon);
        log.printf("  ground_spd:%.1fkm/hr", crsf.gpsF_groundspeed);
        log.printf("  hdg:%.2fdeg", crsf.gpsF_heading);
        log.printf("  alt:%dm", crsf.gps_altitude);
        log.printf("  sats:%d", crsf.gps_sats); 
        log.printf("  gpsfixGood:%d", gpsfixGood); 
        log.printf("  hdgGood:%d\n", hdgGood); 
  #endif          
      }
      if (crsf_id == BATTERY_ID) 
      { 
        hud_bat1_volts = crsf.batF_voltage;           
        hud_bat1_amps = crsf.batF_current;
        hud_bat1_mAh = crsf.batF_fuel_drawn * 1000;
  #if defined DEBUG_CRSF_BAT        
        log.print("BATTERY id:");
        crsf.printByte(crsf_id, ' ');
        log.printf("volts:%2.1f", crsf.batF_voltage);
        log.printf("  amps:%3.1f", crsf.batF_current);
        log.printf("  Ah_drawn:%3.1f", crsf.batF_fuel_drawn);
       log.printf("  remaining:%3u%%\n", crsf.bat_remaining);
  #endif 
      }
     
      if (crsf_id == ATTITUDE_ID)
      {
        cur.hdg = crsf.attiF_yaw;
  #if defined DEBUG_CRSF_ATTI
        log.print("ATTITUDE id:");
        crsf.printByte(crsf_id, ' '); 
        log.printf("pitch:%3.1fdeg", crsf.attiF_pitch);
        log.printf("  roll:%3.1fdeg", crsf.attiF_roll);
        log.printf("  yaw:%3.1fdeg\n", crsf.attiF_yaw);  
  #endif          
      }    
  
      if (crsf_id == FLIGHT_MODE_ID)
      {
        motArmed = !(crsf.flightMode.compare("ARM"));  // Returns 0 if both the strings are the same
  #if defined DEBUG_CRSF_FLIGHT_MODE 
        log.print("FLIGHT_MODE id:");
        crsf.printByte(crsf_id, ' ');
        log.printf("lth:%u %s motArmed:%u\n", crsf.flight_mode_lth, crsf.flightMode.c_str(), motArmed);
  #endif
      gpsGood = (gpsfixGood & lonGood & latGood & altGood);    
      if (gpsGood) gpsGood_millis = millis();     // Time of last good GPS packet 
      }
    }      
    
    #if defined DEBUG_GOODFLAGS
    static bool prevGpsGood = 0;
    if (gpsGood != prevGpsGood)
    {
      log.printf("gpsGood:%u  gpsfixGood:%u  lonGood:%u  latGood:%u  altGood:%u  hdgGood:%u  boxhdgGood:%u \n", gpsGood, gpsfixGood, lonGood, latGood, altGood, hdgGood, boxhdgGood);  
      prevGpsGood = gpsGood;
    }      
    #endif
  }    
#endif  // end of crsfReceive  
        
