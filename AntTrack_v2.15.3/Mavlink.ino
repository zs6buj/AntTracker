

// ******************************************
// Mavlink Message Types

  mavlink_message_t msg;
  uint8_t             readbuf[300];  
  uint8_t             sendbuf[128];
  
// Mavlink Header
uint8_t    system_id;
uint8_t    component_id;
uint8_t    target_component;
uint8_t    mvType;

uint8_t    apo_sysid;
uint8_t    apo_compid;
uint8_t    apo_targsys;
uint8_t    apo_targcomp;

// Message #0  HEARTHBEAT to FC
uint8_t    apo_mission_type;              // Mav2
uint8_t    apo_type = 0;
uint8_t    apo_autopilot = 0;
uint8_t    apo_base_mode = 0;
uint32_t   apo_custom_mode = 0;
uint8_t    apo_system_status = 0;

// Message #0  HEARTHBEAT from FC
uint8_t    ap_type = 0;
uint8_t    ap_autopilot = 0;
uint8_t    ap_base_mode = 0;
uint32_t   ap_custom_mode = 0;
uint8_t    ap_system_status = 0;
uint8_t    ap_mavlink_version = 0;

// Message #2  SYSTEM_TIME
uint64_t   ap_time_unix_usec;
uint32_t   ap_time_boot_ms;

// Message #24  GPS_RAW_INT 
uint8_t    ap_fixtype = 0;            //   0= No GPS, 1 = No Fix, 2 = 2D Fix, 3 = 3D Fix
uint8_t    ap_sat_visible = 0;        // numbers of visible satelites
uint8_t    ap_gps_status = 0;         // (ap_sat_visible*10) + ap_fixtype; 
int32_t    ap_latitude = 0;           // 7 assumed decimal places
int32_t    ap_longitude = 0;          // 7 assumed decimal places
int32_t    ap_amsl24 = 0;             // 1000 = 1m
uint16_t   ap_eph;                    // GPS HDOP horizontal dilution of position (unitless)
uint16_t   ap_epv;                    // GPS VDOP vertical dilution of position (unitless)
uint16_t   ap_vel;                    //  GPS ground speed (m/s * 100)
uint16_t   ap_cog;                    // Course over ground in degrees * 100, 0.0..359.99 degrees. 

// Message GLOBAL_POSITION_INT ( #33 ) (Filtered)
int32_t ap_lat;            // Latitude, expressed as degrees * 1E7
int32_t ap_lon;            // Longitude, expressed as degrees * 1E7
int32_t ap_amsl33;         // Altitude above mean sea level (millimeters)
int32_t ap_alt_ag;         // Altitude above ground (millimeters)
int16_t ap_vx;             //  Ground X Speed (Latitude, positive north), expressed as m/s * 100
int16_t ap_vy;             //  Ground Y Speed (Longitude, positive east), expressed as m/s * 100
int16_t ap_vz;             // Ground Z Speed (Altitude, positive down), expressed as m/s * 100
uint16_t ap_hdg;           // Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees  


// Mavlink varaibles
uint32_t hb_millis = 0;
uint16_t  hb_count = 0;

//***************************************************
uint16_t len;
void Mavlink_Receive() { 

  mavlink_status_t status;
  gotRecord = false;
  
  #if (Telemetry_In == 0)              // Serial    
    while(inSerial.available()) {
      uint8_t c = inSerial.read();

      #ifdef Debug_Mav_Buffer
        Debug.println("Mavlink buffer : ");
      PrintMavBuffer(&msg);
      #endif
    
      if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {

        #if defined Debug_All || defined Debug_Input
          Debug.println("Serial record read:");
          PrintMavBuffer(&msg);  
        #endif
        gotRecord = true;
      }
   } 
  #endif 

  #if (Telemetry_In == 1) // Bluetooth

     bool msgReceived = Read_Bluetooth(&msg);

     if (msgReceived) {

        gotRecord = true;     

        #ifdef  Debug_FC_Down   
          Debug.println("BT record read:");
          PrintMavBuffer(&msg);
        #endif      
      }

  #endif   

  #if (Telemetry_In == 2)  //  WiFi
    
    #if (WiFi_Protocol == 1) // TCP 
    
      bool msgReceived = Read_TCP(&msg);

      if (msgReceived) {
        
        gotRecord = true;   

        #if defined Debug_All || defined Debug_Input 
          Debug.print("Received WiFi TCP message. msgReceived=" ); Debug.println(msgReceived);
          PrintMavBuffer(&msg);
        #endif      
      } // else no message received, drop thru - no block

    #endif
      
    #if (WiFi_Protocol == 2) // UDP
      bool msgReceived = Read_UDP(&msg);

      if (msgReceived) {
        
        gotRecord = true;   

        #if defined Debug_All || defined Debug_Input
          Debug.println(" UDP WiFi record read:");
          PrintMavBuffer(&msg);
        #endif      
      }
       
    #endif 
    
 #endif 

    if (gotRecord) {
      switch(msg.msgid) {
    
        case MAVLINK_MSG_ID_HEARTBEAT:    // #0   http://mavlink.org/messages/common
          ap_type = mavlink_msg_heartbeat_get_type(&msg);
          ap_autopilot = mavlink_msg_heartbeat_get_autopilot(&msg);
          ap_base_mode = mavlink_msg_heartbeat_get_base_mode(&msg);
          ap_custom_mode = mavlink_msg_heartbeat_get_custom_mode(&msg);
          ap_system_status = mavlink_msg_heartbeat_get_system_status(&msg);
          ap_mavlink_version = mavlink_msg_heartbeat_get_mavlink_version(&msg);
          hb_millis=millis(); 

          #if defined Debug_All || defined Debug_Mav_Heartbeat
            Debug.print("Mavlink in #0 Heartbeat: ");           
            Debug.print("ap_type="); Debug.print(ap_type);   
            Debug.print("  ap_autopilot="); Debug.print(ap_autopilot); 
            Debug.print("  ap_base_mode="); Debug.print(ap_base_mode); 
            Debug.print(" ap_custom_mode="); Debug.print(ap_custom_mode);   
            Debug.print("  ap_system_status="); Debug.print(ap_system_status); 
            Debug.print("  ap_mavlink_version="); Debug.println(ap_mavlink_version);
          #endif

          if(!hbGood) {
            ap_fixtype = 0;  
            hb_count++; 
            #ifdef Debug_Status
            Debug.print(" hb_count=");
            Debug.print(hb_count);
            Debug.println("");
            #endif
            if((hb_count >= 3) || (homeInitialised)) {  // If 3 heartbeats or 1 hb && previously connected, we are connected
              hbGood=true;                       
              #ifdef Debug_Status
              Debug.println("hbGood=true"); 
              DisplayPrintln("Heartbeat good");
              #endif
              hb_count=0;
              }
          }
          break;
        case MAVLINK_MSG_ID_SYSTEM_TIME:          // #02
          if (!hbGood) return;
          ap_time_unix_usec = mavlink_msg_system_time_get_time_unix_usec(&msg);
          ap_time_boot_ms = mavlink_msg_system_time_get_time_boot_ms(&msg);
          timeGood = true;
          
          LostPowerCheckAndRestore(ap_time_unix_usec/1E6);  // Within 5 minutes, then restore from EEPROM
          
          #if defined Debug_All || defined Debug_Time 
            Debug.print("Mavlink in #02 SYSTEM_TIME: ");  
            Debug.print("ap_time_unix sec="); Debug.print(ap_time_unix_usec/1E6, 0);
            Debug.print("  ap_time_boot_ms="); Debug.print(ap_time_boot_ms/1E3, 3);
            Debug.print("  UTC time="); Debug.println(TimeString(ap_time_unix_usec/1E6));
          #endif
          break;
        case MAVLINK_MSG_ID_GPS_RAW_INT:          // #24
          if (!hbGood) break;        
          ap_fixtype = mavlink_msg_gps_raw_int_get_fix_type(&msg);                   // 0 = No GPS, 1 =No Fix, 2 = 2D Fix, 3 = 3D Fix
          ap_sat_visible =  mavlink_msg_gps_raw_int_get_satellites_visible(&msg);    // number of visible satelites
          ap_gps_status = (ap_sat_visible*10) + ap_fixtype; 
          if(ap_fixtype > 2)  {
            ap_latitude = mavlink_msg_gps_raw_int_get_lat(&msg);
            ap_longitude = mavlink_msg_gps_raw_int_get_lon(&msg);
            ap_amsl24 = mavlink_msg_gps_raw_int_get_alt(&msg);             // 1m = 1000 
            ap_eph = mavlink_msg_gps_raw_int_get_eph(&msg);                // GPS HDOP 
            ap_epv = mavlink_msg_gps_raw_int_get_epv(&msg);                // GPS VDOP 
            ap_vel = mavlink_msg_gps_raw_int_get_vel(&msg);                // GPS ground speed (m/s * 100)
            ap_cog = mavlink_msg_gps_raw_int_get_cog(&msg);                // Course over ground (NOT heading) in degrees * 100
            millisGPS = millis();                                          // Time of last good GPS packet
          }
          #if defined Debug_All || defined Debug_Mav_GPS 
            Debug.print("Mavlink in #24 GPS_RAW_INT: ");  
            Debug.print("ap_fixtype="); Debug.print(ap_fixtype);
            if (ap_fixtype==1) Debug.print(" No GPS");
              else if (ap_fixtype==2) Debug.print(" No Lock");
              else if (ap_fixtype==3) Debug.print(" 3D Lock");
              else if (ap_fixtype==4) Debug.print(" 3D+ Lock");
              else Debug.print(" Unknown");

            Debug.print("  sats visible="); Debug.print(ap_sat_visible);
            Debug.print("  GPS status="); Debug.print(ap_gps_status);
            Debug.print("  latitude="); Debug.print((float)(ap_latitude)/1E7, 7);
            Debug.print("  longitude="); Debug.print((float)(ap_longitude)/1E7, 7);
            Debug.print("  gps alt amsl"); Debug.print((float)(ap_amsl24)/1E3, 0);
            Debug.print("  eph (hdop)="); Debug.print(ap_eph);               // HDOP
            Debug.print("  epv (vdop)="); Debug.print(ap_epv);
            Debug.print("  vel="); Debug.print((float)ap_vel / 100, 1);         // GPS ground speed (m/s)
            Debug.print("  cog="); Debug.println((float)ap_cog / 100, 1);       // Course over ground in degrees
          #endif     
          break;
 
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:     // #33
          if ((!hbGood) || (ap_fixtype < 3)) break;  
          // We have a 3D Lock - change to 4 if you want 3D plus
          
          ap_lat = mavlink_msg_global_position_int_get_lat(&msg);             // Latitude, expressed as degrees * 1E7
          ap_lon = mavlink_msg_global_position_int_get_lon(&msg);             // Pitch angle (rad, -pi..+pi)
          ap_amsl33 = mavlink_msg_global_position_int_get_alt(&msg);          // x Supposedly altitude above mean sea level (millimeters)
          ap_alt_ag = mavlink_msg_global_position_int_get_relative_alt(&msg); // Altitude above ground (millimeters)
          ap_vx = mavlink_msg_global_position_int_get_vx(&msg);               //  Ground X Speed (Latitude, positive north), expressed as m/s * 100
          ap_vy = mavlink_msg_global_position_int_get_vy(&msg);               //  Ground Y Speed (Longitude, positive east), expressed as m/s * 100
          ap_vz = mavlink_msg_global_position_int_get_vz(&msg);               // Ground Z Speed (Altitude, positive down), expressed as m/s * 100
          ap_hdg = mavlink_msg_global_position_int_get_hdg(&msg);             // Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees          ap_ap_amsl = mavlink_msg_attitude_get_yaw(&msg);                // Yaw angle (rad, -pi..+pi)
          millisGPS = millis();                                               // Time of last good GPS packet
          #ifdef QLRS
            ap_hdg = ap_hdg * 100;  //  Compensate for QLRS heading already divided by 100
          #endif
          if (!gpsGood) {
            gpsGood = true;
            #ifdef Debug_Status
              Debug.println("gpsGood=true");  
            #endif
          }
          new_GPS_data = true;

          cur.lat =  (float)ap_lat / 1E7;
          cur.lon = (float)ap_lon / 1E7;
          cur.alt = ap_amsl24 / 1E3;
          cur.hdg = ap_hdg / 100;
          
          if (Heading_Source==1 && (gpsGood) && (!homSaved)) AutoStoreHome();  // Only need this when Heading_Source is GPS 

          #if defined Debug_All || defined Debug_Mav_GPS
            Debug.print("Mavlink in #33 GPS Int: ");
            Debug.print(" ap_lat="); Debug.print((float)ap_lat / 1E7, 6);
            Debug.print(" ap_lon="); Debug.print((float)ap_lon / 1E7, 6);
            Debug.print(" ap_amsl="); Debug.print((float)ap_amsl33 / 1E3, 0);
            Debug.print(" ap_alt_ag="); Debug.print((float)ap_alt_ag / 1E3, 1);           
            Debug.print(" ap_vx="); Debug.print((float)ap_vx / 100, 1);
            Debug.print(" ap_vy="); Debug.print((float)ap_vy / 100, 1);
            Debug.print(" ap_vz="); Debug.print((float)ap_vz / 100, 1);
            Debug.print(" ap_hdg="); Debug.println((float)ap_hdg / 100, 1);
          #endif 
                            
          break;
      }

    if (hbGood && (millis() - hb_millis >= 8000)){
      hbGood = false;   // If no heartbeat for 8 seconds then link timed out 
      gpsGood = false;
      DisplayPrintln("Mavlink timed out!");  
      Debug.println("Mavlink timed out!"); 
    }
  }
} 
  
 //********************************************************************************
#if ((Telemetry_In == 2) && (WiFi_Protocol == 1))     //  WiFi TCP

bool Read_TCP(mavlink_message_t* msgptr)  {
   if ( (!wifiSuGood) || (!outbound_clientGood) ) return false; 

    bool msgRcvd = false;
    mavlink_status_t _status;

        // remember, only 1 [0] outbound client (FC side) is possible
        active_client_idx = 0;

        len = clients[active_client_idx]->available();     // is there some data to read
        uint16_t tcp_count = len;
        if(tcp_count > 0) {           // if so, read until no more

          while(tcp_count--)  {
            
            int result = clients[active_client_idx]->read();   //  TCP READ <<================
            
            if (result >= 0)  {  // -1 if no char read

                msgRcvd = mavlink_parse_char(MAVLINK_COMM_0, result, msgptr, &_status);  // PARSE <<===========
                if(msgRcvd) {
                    if(!hb_heard_from) {
                        if(msgptr->msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                            hb_heard_from     = true;
                            hb_system_id      = msgptr->sysid;
                            hb_comp_id        = msgptr->compid;
                            hb_seq_expected   = msgptr->seq + 1;
                            hb_last_heartbeat = millis();
                        }
                    } else {
                        if(msgptr->msgid == MAVLINK_MSG_ID_HEARTBEAT)
                          hb_last_heartbeat = millis();
                          checkLinkErrors(msgptr);
                    }
                    break;
                }
            }
          }
        }

    return msgRcvd;
}
#endif
//********************************************************************************
#if ((Telemetry_In == 2) && (WiFi_Protocol == 2))     //  WiFi UDP
bool Read_UDP(mavlink_message_t* msgptr)  {
    if (!wifiSuGood) return false;  
    bool msgRcvd = false;
    mavlink_status_t _status;
    
    len = UDP.parsePacket();
    int UDP_count = len;
    if(UDP_count > 0) {

        while(UDP_count--)  {

            int result = UDP.read();
            if (result >= 0)  {

                msgRcvd = mavlink_parse_char(MAVLINK_COMM_2, result, msgptr, &_status);
                if(msgRcvd) {
                  
                    UDP_remoteIP = UDP.remoteIP();  // remember which remote client sent this packet so we can target it
                    PrintRemoteIP();
                    if(!hb_heard_from) {
                        if(msgptr->msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                            hb_heard_from      = true;
                            hb_system_id       = msgptr->sysid;
                            hb_comp_id         = msgptr->compid;
                            hb_seq_expected   = msgptr->seq + 1;
                            hb_last_heartbeat = millis();
                        }
                    } else {
                        if(msgptr->msgid == MAVLINK_MSG_ID_HEARTBEAT)
                          hb_last_heartbeat = millis();
                          checkLinkErrors(msgptr);
                    }
                    
                    break;
                }
            }
        }
    }
    
    return msgRcvd;
}
#endif
//********************************************************************************
#if (Telemetry_In == 1)           // Bluetooth

bool Read_Bluetooth(mavlink_message_t* msgptr)  {
    
    bool msgRcvd = false;
    mavlink_status_t _status;
    
    len = SerialBT.available();
    uint16_t bt_count = len;
    if(bt_count > 0) {

        while(bt_count--)  {
            int result = SerialBT.read();
            if (result >= 0)  {

                msgRcvd = mavlink_parse_char(MAVLINK_COMM_2, result, msgptr, &_status);
                if(msgRcvd) {

                    if(!hb_heard_from) {
                        if(msgptr->msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                            hb_heard_from     = true;
                            hb_system_id      = msgptr->sysid;
                            hb_comp_id        = msgptr->compid;
                            hb_seq_expected   = msgptr->seq + 1;
                            hb_last_heartbeat = millis();
                        }
                    } else {
                        if(msgptr->msgid == MAVLINK_MSG_ID_HEARTBEAT)
                          hb_last_heartbeat = millis();
                          checkLinkErrors(msgptr);
                    }
                 
                    break;
                }
            }
        }
    }
    
    return msgRcvd;
}
#endif
//================================================================================================= 
void Write_To_FC(uint32_t msg_id) {
  
  #if (Telemetry_In == 0)              // Serial to FC
    len = mavlink_msg_to_send_buffer(sendbuf, &msg);
    inSerial.write(sendbuf,len);  
         
    #if defined  Debug_FC_Write
      if (msg_id) {    //  dont print heartbeat - too much info
        Debug.printf("Write to FC Serial: len=%d\n", len);
        PrintMavBuffer(&msg);
      }  
    #endif    
  #endif

  #if (Telemetry_In == 1)              // Bluetooth to FC   
        bool msgSent = Send_Bluetooth(&msg);      
        #ifdef  Debug_FC_Write
          Debug.print("Write to FC Bluetooth: msgSent="); Debug.println(msgSent);
          if (msgSent) PrintMavBuffer(&msg);
        #endif     
  #endif

  #if (Telemetry_In == 2)              // WiFi to FC  

      if (wifiSuGood) { 
        #if (WiFi_Protocol == 1)      // TCP/IP
           active_client_idx = 0;             // we only ever need 1
           bool msgSent = Send_TCP(&msg);  // to FC   
           #ifdef  Debug_FC_Write
             Debug.print("Write to FC WiFi TCP: msgSent="); Debug.println(msgSent);
             PrintMavBuffer(&msg);
           #endif    
         #endif   
         #if (WiFi_Protocol == 2)       // UDP 
           active_client_idx = 0;             // we only ever need 1 here   
           bool msgSent = Send_UDP(&msg);     // to FC    
           #ifdef  Debug_FC_Write
             Debug.print("Write to FC WiFi UDP: msgSent="); Debug.println(msgSent);
             if (msgSent) PrintMavBuffer(&msg);
           #endif           
         #endif                                                             
      }
   
  #endif       
}  
//================================================================================================= 
  #if (Telemetry_In == 1)     //  BT
  bool Send_Bluetooth(mavlink_message_t* msgptr) {

    bool msgSent = false;
     
    uint16_t len = mavlink_msg_to_send_buffer(sendbuf, msgptr);
  
    size_t sent = SerialBT.write(sendbuf,len);

    if (sent == len) {
      msgSent = true;
      link_status.packets_sent++;
    }

    return msgSent;
  }
  #endif
//================================================================================================= 
#if (Telemetry_In == 2) 
  #if (WiFi_Protocol == 1)     //  TCP
  bool Send_TCP(mavlink_message_t* msgptr) {
  if ( (!wifiSuGood) || (!outbound_clientGood) ) return false; 
    bool msgSent = false;
    uint16_t len = mavlink_msg_to_send_buffer(sendbuf, msgptr);
  
    size_t sent =  clients[active_client_idx]->write(sendbuf,len);  

    if (sent == len) {
      msgSent = true;
      link_status.packets_sent++;
    }

    return msgSent;
  }
  #endif
#endif  
//================================================================================================= 
#if (Telemetry_In == 2) 
  #if (WiFi_Protocol == 2)     //  UDP
  bool Send_UDP(mavlink_message_t* msgptr) {
    if (!wifiSuGood) return false;  
    bool msgSent = false;

    if (msgptr->msgid == MAVLINK_MSG_ID_HEARTBEAT) {
      UDP_remoteIP = localIP;
      UDP_remoteIP[3] = 255;       // always broadcast a heartbeat on the local LAN, either from the GCS or from the FC                 
   //   Debug.print("Broadcast heartbeat UDP_remoteIP="); Debug.println(UDP_remoteIP.toString());     
    } 

    UDP.beginPacket(UDP_remoteIP, UDP_remotePort);

    uint16_t len = mavlink_msg_to_send_buffer(sendbuf, msgptr);
  
    size_t sent = UDP.write(sendbuf,len);

    if (sent == len) {
      msgSent = true;
      link_status.packets_sent++;
      UDP.flush();
    }

    bool endOK = UDP.endPacket();
 //   if (!endOK) Debug.printf("msgSent=%d   endOK=%d\n", msgSent, endOK);
    return msgSent;
  }
  #endif
#endif  
//********************************************************************************
 #if (Telemetry_In == 1)  ||  (Telemetry_In == 2)         // Bluetooth or WiFi
void checkLinkErrors(mavlink_message_t* msgptr)   {

    //-- Don't bother if we have not heard from the link (and it's the proper sys/comp ids)
    if(!hb_heard_from || msgptr->sysid != hb_system_id || msgptr->compid != hb_comp_id) {
        return;
    }
    uint16_t seq_received = (uint16_t)msgptr->seq;
    uint16_t packet_lost_count = 0;
    //-- Account for overflow during packet loss
    if(seq_received < hb_seq_expected) {
        packet_lost_count = (seq_received + 255) - hb_seq_expected;
    } else {
        packet_lost_count = seq_received - hb_seq_expected;
    }
    hb_seq_expected = msgptr->seq + 1;
    link_status.packets_lost += packet_lost_count;
}
#endif

 //================================================================================================= 
void nbdelay(uint32_t delaymS) { // non-blocking delay
uint32_t start;
  start = millis();
  
  while (millis() - start < delaymS) {     
    yield();
  }
} 
//================================================================================================= 
void Send_FC_Heartbeat() {
  
  apo_sysid = Device_sysid;                    // From config.h MP is 255, QGC default is 0
  apo_compid = Device_compid;                  // 158 Generic autopilot peripheral component ID. MP is 190
  apo_targsys = 1;                             // FC
  apo_targcomp = 1;                            // FC                   

  apo_type = MAV_TYPE_GCS;                       // 6 Pretend to be a GCS
 // apo_type = MAV_TYPE_ANTENNA_TRACKER;         // 5 
  apo_autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA; // 3 AP Mega
  apo_base_mode = 0;
  apo_system_status = MAV_STATE_ACTIVE;         // 4
   
  mavlink_msg_heartbeat_pack(apo_sysid, apo_compid, &msg, apo_type, apo_autopilot, apo_base_mode, apo_system_status, 0); 
  Write_To_FC(0); 
  #if defined Debug_Our_FC_Heartbeat
     Debug.print("Our own heartbeat to FC: #0 Heartbeat: ");  
     Debug.print("apo_sysid="); Debug.print(apo_sysid);   
     Debug.print("  apo_compid="); Debug.print(apo_compid);  
     Debug.print("  apo_targsys="); Debug.print(apo_targsys);   
     Debug.print("  apo_targcomp="); Debug.print(apo_targcomp);                         
     Debug.print("  apo_type="); Debug.print(apo_type);   
     Debug.print("  apo_autopilot="); Debug.print(apo_autopilot); 
     Debug.print("  apo_base_mode="); Debug.print(apo_base_mode); 
     Debug.print("  apo_custom_mode="); Debug.print(apo_custom_mode);
     Debug.print("  apo_system_status="); Debug.print(apo_system_status);    
     Debug.println();
  #endif   
}  
