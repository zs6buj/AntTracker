
// Forward declarations
void printByte(byte b);


// ******************************************
// Mavlink Message Types

  mavlink_message_t msg, sendmsg;
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

// Message # 1  SYS_status 
uint16_t   ap_voltage_battery1 = 0;    // 1000 = 1V
int16_t    ap_current_battery1 = 0;    //  10 = 1A
uint8_t    ap1_battery_remaining = 0;   // % Battery energy remaining - probably deprecated?


// Message #2  SYSTEM_TIME
uint64_t   ap_time_unix_usec;
uint32_t   ap_time_boot_ms;

// Message #24  GPS_RAW_INT 
uint8_t    ap24_fixtype = 0;            //   0= No GPS, 1 = No Fix, 2 = 2D Fix, 3 = 3D Fix
uint8_t    ap24_sat_visible = 0;        // numbers of visible satelites
uint8_t    ap24_gps_status = 0;         // (ap24_sat_visible*10) + ap24_fixtype; 
int32_t    ap24_lat = 0;                // 7 assumed decimal places
int32_t    ap24_lon = 0;                // 7 assumed decimal places
int32_t    ap24_amsl = 0;               // 1000 = 1m
uint16_t   ap24_eph;                    // GPS HDOP horizontal dilution of position (unitless)
uint16_t   ap24_epv;                    // GPS VDOP vertical dilution of position (unitless)
uint16_t   ap24_vel;                    //  GPS ground speed (m/s * 100)
uint16_t   ap24_cog;                    // Course over ground in degrees * 100, 0.0..359.99 degrees. 

// Message ATTITUDE ( #30 )
float ap_roll;                          // Roll angle (rad, -pi..+pi)
float ap_pitch;                         // Pitch angle (rad, -pi..+pi)
float ap_yaw;                           // Yaw angle (rad, -pi..+pi)
float ap_rollspeed;                     // Roll angular speed (rad/s)
float ap_pitchspeed;                    // Pitch angular speed (rad/s)
float ap_yawspeed;                      // Yaw angular speed (rad/s)

// Message GLOBAL_POSITION_INT ( #33 ) (Filtered)
int32_t ap33_lat;                       // Latitude, expressed as degrees * 1E7
int32_t ap33_lon;                       // Longitude, expressed as degrees * 1E7
int32_t ap33_amsl;                      // Altitude above mean sea level (millimeters)
int32_t ap33_alt_ag;                    // Altitude above ground (millimeters)
int16_t ap33_vx;                        //  Ground X Speed (Latitude, positive north), expressed as m/s * 100
int16_t ap33_vy;                        //  Ground Y Speed (Longitude, positive east), expressed as m/s * 100
int16_t ap33_vz;                        // Ground Z Speed (Altitude, positive down), expressed as m/s * 100
uint16_t ap33_hdg;                      // Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees  

// Message #35 RC_CHANNELS_RAW
uint8_t ap_rssi;
bool    ap_rssi_ft = true; // first rssi connection
uint8_t ap_rssi35;

//uint16_t ap65_chan16_raw;        // Used for RSSI uS 1000=0%  2000=100%
uint8_t  ap65_rssi;              // Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown

// Message #74 Vpt_HUD  
float    ap74_air_spd;
float    ap74_grd_spd;
int16_t  ap74_hdg;
uint16_t ap74_throt;          // %
float    ap74_amsl;   
float    ap74_climb; 

// Message #109 RADIO_status (Sik radio firmware)
uint8_t   ap109_rssi;           // local signal strength
uint8_t   ap109_remrssi;        // remote signal strength
uint8_t   ap109_txbuf;          // how full the tx buffer is as a percentage
uint8_t   ap109_noise;          // background noise level
uint8_t   ap109_remnoise;       // remote background noise level
uint16_t  ap109_rxerrors;       // receive errors
uint16_t  ap109_fixed;          // count of error corrected packets

// Message  #147 BATTERY_status 
uint8_t      ap_battery_id;       
uint8_t      ap_battery_function;
uint8_t      ap_bat_type;  
int16_t      ap_bat_temperature;    // centi-degrees celsius
uint16_t     ap_voltages[10];       // cell voltages in millivolts 
int16_t      ap_current_battery;    // in 10*milliamperes (1 = 10 milliampere)
int32_t      ap_current_consumed;   // mAh
int32_t      ap_energy_consumed;    // HectoJoules (intergrated U*I*dt) (1 = 100 Joule)
int8_t       ap147_battery_remaining;  // (0%: 0, 100%: 100)
int32_t      ap_time_remaining;     // in seconds
uint8_t      ap_charge_state;     

// Mavlink varaibles
uint16_t  hb_count = 0;
bool      rssiGood = false;
bool      rssi35 = false;
bool      rssi65 = false;
bool      rssi109 = false;

//=====================Forward declarations =================

 void PrintRemoteIP();
 void checkLinkErrors(mavlink_message_t* msgptr);
 
//===========================================================
                    
uint16_t len;
void Mavlink_Receive() 
{ 
#if (PROTOCOL == 1) || (PROTOCOL == 2) || (PROTOCOL == 0)     
  //==================== Periodically Send Our Own Heartbeat to FC to trigger tardy telemetry
  if(millis()- millisFcHheartbeat > 2000) {  // MavToPass heartbeat to FC every 2 seconds
  millisFcHheartbeat=millis();       
  Send_FC_Heartbeat();                     // for serial must have tx pin connected to dedicated telem radio rx pin  
  }

  mavlink_status_t status;
  gotRecord = false;
  #if (MEDIUM_IN == 1)              // Serial    
    while(inSerial.available()) {
      uint8_t c = inSerial.read();
      //Printbyte(c, false, ' ');
      #ifdef DEBUG_Mav_Buffer
        log.println("Mavlink buffer : ");
      PrintMavBuffer(&msg);
      #endif
    
      if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {

        #if defined DEBUG_All || defined DEBUG_Input
          log.println("Serial record read:");
          PrintMavBuffer(&msg);  
        #endif
        gotRecord = true;
      }
   } 
  #endif 

  #if (MEDIUM_IN == 3) // Bluetooth
     bool msgReceived = Read_Bluetooth(&msg);
     if (msgReceived) {
        gotRecord = true;     
        #ifdef  DEBUG_FC_Down   
          log.println("BT record read:");
          PrintMavBuffer(&msg);
        #endif      
      }
  #endif   

  #if (MEDIUM_IN == 2)  //  WiFi
    #if (WIFI_PROTOCOL == 1) // TCP 
      bool msgReceived = Read_TCP(&msg);
      if (msgReceived) {
        gotRecord = true;  
        #if defined DEBUG_All || defined DEBUG_Input 
          log.print("Received WiFi TCP message. msgReceived=" ); log.println(msgReceived);
          PrintMavBuffer(&msg);
        #endif      
      } // else no message received, drop thru - no block
    #endif
    #if (WIFI_PROTOCOL == 2) // UDP
      bool msgReceived = Read_UDP(&msg);
      if (msgReceived) {     
        gotRecord = true;   
        #if defined DEBUG_All || defined DEBUG_Input
          log.println(" UDP WiFi record read:");
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
          hbGood_millis = millis();    

          motArmed = ap_base_mode >> 7;  // motors armed!
                    
          #if defined DEBUG_All || defined DEBUG_Mav_Heartbeat
            log.print("Mavlink in #0 Heartbeat: ");           
            log.print("ap_type="); log.print(ap_type);   
            log.print("  ap_autopilot="); log.print(ap_autopilot); 
            log.print("  ap_base_mode="); log.print(ap_base_mode); 
            log.print("  motArmed="); log.print(motArmed);            
            log.print("  ap_custom_mode="); log.print(ap_custom_mode);   
            log.print("  ap_system_status="); log.print(ap_system_status); 
            log.print("  ap_mavlink_version="); log.println(ap_mavlink_version);
          #endif

          if(!hbGood) {
            ap24_fixtype = 0;  
            hb_count++; 
            #ifdef DEBUG_Status
            log.print(" hb_count=");
            log.print(hb_count);
            log.println("");
            #endif

            if((hb_count >= 3) || (finalHomeStored)) {  // If 3 heartbeats or 1 hb && previously connected, we are connected
              hbGood=true; 
              mavGood = true;   
              hbGood_millis = millis();                   
              hb_count=0;
              }

          }
          break;

        case MAVLINK_MSG_ID_SYS_STATUS:   // #1
          if (!mavGood) break;

          ap_voltage_battery1= Get_Volt_Average1(mavlink_msg_sys_status_get_voltage_battery(&msg));        // 1000 = 1V  i.e mV
          ap_current_battery1= Get_Current_Average1(mavlink_msg_sys_status_get_current_battery(&msg));     //  100 = 1A, i.e dA

          pt_bat1_volts = ap_voltage_battery1 * 0.01F;         // mV -> dV
          pt_bat1_amps = ap_current_battery1 * 0.1F;           // cA -> dA

          hud_bat1_volts = pt_bat1_volts;
          hud_bat1_amps = pt_bat1_amps;
          
          #if defined Mav_DEBUG_All || defined Mav_DEBUG_SysStatus || defined DEBUG_Batteries
            log.print("Mavlink from FC #1 Sys_status: ");     
            log.print(" Sensor health=");
            log.print(" Bat volts=");
            log.print((float)ap_voltage_battery1/ 1000, 3);   // now V
            log.print("  Bat amps=");
            log.println((float)ap_current_battery1/ 100, 3);   // now A

          #endif

         break;

        case MAVLINK_MSG_ID_SYSTEM_TIME:          // #02
          if (!hbGood) return;
          ap_time_unix_usec = mavlink_msg_system_time_get_time_unix_usec(&msg);
          ap_time_boot_ms = mavlink_msg_system_time_get_time_boot_ms(&msg);

          epochSync = ap_time_unix_usec/1E6;
          if (epochSync != 0) {
            timeGood = true;
            millisSync = millis();   
          }
          
          #if defined DEBUG_All || defined DEBUG_Time 
            log.print("Mavlink in #02 SYSTEM_TIME: ");  
            log.print("ap_time_unix sec="); log.print(ap_time_unix_usec/1E6, 0);
            log.print("  ap_time_boot_ms="); log.print(ap_time_boot_ms/1E3, 3);
            log.print("  UTC time="); log.println(TimeString(ap_time_unix_usec/1E6));
          #endif
          break;
        case MAVLINK_MSG_ID_GPS_RAW_INT:          // #24
          if (!hbGood) break;        
          ap24_fixtype = mavlink_msg_gps_raw_int_get_fix_type(&msg);                   // 0 = No GPS, 1 =No Fix, 2 = 2D Fix, 3 = 3D Fix
          ap24_sat_visible =  mavlink_msg_gps_raw_int_get_satellites_visible(&msg);    // number of visible satelites
          hud_num_sats = ap24_sat_visible;
          ap24_gps_status = (ap24_sat_visible*10) + ap24_fixtype; 
          if(ap24_fixtype > 2)  {
            ap24_lat = mavlink_msg_gps_raw_int_get_lat(&msg);
            ap24_lon = mavlink_msg_gps_raw_int_get_lon(&msg);
            ap24_amsl = mavlink_msg_gps_raw_int_get_alt(&msg);               // 1m = 1000 
            ap24_eph = mavlink_msg_gps_raw_int_get_eph(&msg);                // GPS HDOP 
            ap24_epv = mavlink_msg_gps_raw_int_get_epv(&msg);                // GPS VDOP 
            ap24_vel = mavlink_msg_gps_raw_int_get_vel(&msg);                // GPS ground speed (m/s * 100)
            ap24_cog = mavlink_msg_gps_raw_int_get_cog(&msg);                // Course over ground (NOT heading) in degrees * 100
            gpsGood_millis = millis();                                       // Time of last good GPS packet
          }

           cur.lat =  (float)ap24_lat / 1E7;
           cur.lon = (float)ap24_lon / 1E7;
           //cur.alt = ap24_amsl / 1E3; // use ap33_alt_ag

          if (ap24_sat_visible > 15) {                // @rotorman 2021/01/18
            if (ap24_sat_visible == 255)
              pt_numsats = 0; // for special case 255 == unknown satellite count
            else
             pt_numsats = 15; // limit to 15 due to only 4 bits available
          }  else 
              pt_numsats = ap24_sat_visible;
              
          hud_num_sats = pt_numsats;                
          gpsfixGood = (ap24_fixtype>=4);
          
          #if defined DEBUG_All || defined DEBUG_Mav_GPS 
            log.print("Mavlink in #24 GPS_RAW_INT: ");  
            log.print("ap24_fixtype="); log.print(ap24_fixtype);
            if (ap24_fixtype==1) log.print(" No GPS");
              else if (ap24_fixtype==2) log.print(" No Lock");
              else if (ap24_fixtype==3) log.print(" 3D Lock");
              else if (ap24_fixtype==4) log.print(" 3D+ Lock");
              else if (ap24_fixtype>4) log.print(" GPS fix > 4");             
              else log.print(" Unknown");

            log.print("  sats visible="); log.print(ap24_sat_visible);
            log.print("  GPS status="); log.print(ap24_gps_status);
            log.print("  latitude="); log.print((float)(ap24_lat)/1E7, 7);
            log.print("  longitude="); log.print((float)(ap24_lon)/1E7, 7);
            log.print("  gps alt amsl="); log.print((float)(ap24_amsl)/1E3, 0);
            log.print("  eph (hdop)="); log.print(ap24_eph);               // HDOP
            log.print("  epv (vdop)="); log.print(ap24_epv);
            log.print("  vel="); log.print((float)ap24_vel / 100, 1);         // GPS ground speed (m/s)
            log.print("  cog="); log.println((float)ap24_cog / 100, 1);       // Course over ground in degrees
          #endif    
           
          break;
        case MAVLINK_MSG_ID_ATTITUDE:                // #30
          if (!hbGood) break;   

          ap_roll = mavlink_msg_attitude_get_roll(&msg);              // Roll angle (rad, -pi..+pi)
          ap_pitch = mavlink_msg_attitude_get_pitch(&msg);            // Pitch angle (rad, -pi..+pi)
          ap_yaw = mavlink_msg_attitude_get_yaw(&msg);                // Yaw angle (rad, -pi..+pi)
          ap_rollspeed = mavlink_msg_attitude_get_rollspeed(&msg);    // Roll angular speed (rad/s)
          ap_pitchspeed = mavlink_msg_attitude_get_pitchspeed(&msg);  // Pitch angular speed (rad/s)
          ap_yawspeed = mavlink_msg_attitude_get_yawspeed(&msg);      // Yaw angular speed (rad/s)           

          ap_roll = RadToDeg(ap_roll);   // Now degrees
          ap_pitch = RadToDeg(ap_pitch);
          ap_yaw = RadToDeg(ap_yaw);
          pt_froll = ap_roll; 
          pt_fpitch = ap_pitch; 
          hud_pitch = pt_fpitch;
          hud_roll = pt_froll;         
  
          #if defined Mav_DEBUG_All || defined Mav_DEBUG_Attitude   
            log.print("Mavlink from FC #30 Attitude: ");      
            log.print(" ap_roll degs=");
            log.print(ap_roll, 1);
            log.print(" ap_pitch degs=");   
            log.print(ap_pitch, 1);
            log.print(" ap_yaw degs=");         
            log.println(ap_yaw, 1);
          #endif  
                     
          #if (defined frBuiltin)
            FrPort.PushMessage(0x5006, 0 );  // 0x5006 Attitude   
          #endif        

          break;   
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:     // #33
          if ((!hbGood) || (ap24_fixtype < 3)) break;  
          // We have a 3D Lock - change to 4 if you want 3D plus
          
          ap33_lat = mavlink_msg_global_position_int_get_lat(&msg);             // Latitude, expressed as degrees * 1E7
          ap33_lon = mavlink_msg_global_position_int_get_lon(&msg);             // Pitch angle (rad, -pi..+pi)
          ap33_amsl = mavlink_msg_global_position_int_get_alt(&msg);            // x Supposedly altitude above mean sea level (millimeters)
          ap33_alt_ag = mavlink_msg_global_position_int_get_relative_alt(&msg); // Altitude above ground (millimeters)
          ap33_vx = mavlink_msg_global_position_int_get_vx(&msg);               //  Ground X Speed (Latitude, positive north), expressed as m/s * 100
          ap33_vy = mavlink_msg_global_position_int_get_vy(&msg);               //  Ground Y Speed (Longitude, positive east), expressed as m/s * 100
          ap33_vz = mavlink_msg_global_position_int_get_vz(&msg);               // Ground Z Speed (Altitude, positive down), expressed as m/s * 100
          ap33_hdg = mavlink_msg_global_position_int_get_hdg(&msg);             // Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees          ap_ap_amsl = mavlink_msg_attitude_get_yaw(&msg);                // Yaw angle (rad, -pi..+pi)
          gpsGood_millis = millis();                                            // Time of last good GPS packet
          #ifdef QLRS
            ap33_hdg = ap33_hdg * 100;  //  Compensate for QLRS heading already divided by 100
          #endif
          if (!gpsGood) {
            gpsGood = true;
            gpsGood_millis = millis();
          }
          new_GPS_data = true;

          cur.lat =  (float)ap33_lat / 1E7;
          cur.lon = (float)ap33_lon / 1E7;
          cur.alt = ap33_alt_ag / 1E3;            
          if (finalHomeStored) {
            cur.alt_ag = cur.alt - hom.alt;
          } else {
            cur.alt_ag = 0;
          }        
          cur.hdg = ap33_hdg / 100;
          hdgGood=true;
          
          #if defined DEBUG_All || defined DEBUG_Mav_GPS
            log.print("Mavlink in #33 GPS Int: ");
            log.print(" ap33_lat="); log.print((float)ap33_lat / 1E7, 6);
            log.print(" ap33_lon="); log.print((float)ap33_lon / 1E7, 6);
            log.print(" ap33_amsl="); log.print((float)ap33_amsl / 1E3, 0);
            log.print(" ap33_alt_ag="); log.print((float)ap33_alt_ag / 1E3, 1);           
            log.print(" ap33_vx="); log.print((float)ap33_vx / 100, 1);
            log.print(" ap33_vy="); log.print((float)ap33_vy / 100, 1);
            log.print(" ap33_vz="); log.print((float)ap33_vz / 100, 1);
            log.print(" ap33_hdg="); log.println((float)ap33_hdg / 100, 1);
          #endif 
                            
          break;
          
        case MAVLINK_MSG_ID_RC_CHANNELS_RAW:         // #35
          if (!mavGood) break; 
          ap_rssi35 = mavlink_msg_rc_channels_raw_get_rssi(&msg);
          rssi35 = true;  
               
          if ((!rssi65) && (!rssi109)) { // If no #65 and no #109 received, then use #35
            rssiGood=true;
            ap_rssi = ap_rssi35 / 2.54;  // 254 -> 100%              
            pt_rssi = ap_rssi;  
          } 
          break;
        case MAVLINK_MSG_ID_RC_CHANNELS:             // #65
          if (!mavGood) break; 
            ap65_rssi = mavlink_msg_rc_channels_get_rssi(&msg);   // Receive RSSI 0: 0%, 254: 100%, 255: invalid/unknown       
            rssi65 = true;  
            if (!rssi109) { // If no #109 received, then use #65
              rssiGood=true; 
              ap_rssi = ap65_rssi / 2.54;  // 254 -> 100%    
              pt_rssi = ap_rssi;     
            }              
            break;
            
        case MAVLINK_MSG_ID_VFR_HUD:                 //  #74
          if (!mavGood) break;      
          ap74_air_spd = mavlink_msg_vfr_hud_get_airspeed(&msg);
          ap74_grd_spd = mavlink_msg_vfr_hud_get_groundspeed(&msg);      //  in m/s
          ap74_hdg = mavlink_msg_vfr_hud_get_heading(&msg);           //  in degrees
          ap74_throt = mavlink_msg_vfr_hud_get_throttle(&msg);           //  integer percent
          ap74_amsl = mavlink_msg_vfr_hud_get_alt(&msg);                 //  m
          ap74_climb = mavlink_msg_vfr_hud_get_climb(&msg);              //  m/s

          pt_yaw = ap74_hdg * 10; 
          hud_grd_spd = ap74_grd_spd;
          hud_climb = ap74_climb;
          
         #if defined Mav_DEBUG_All || defined Mav_DEBUG_Hud
            log.print("Mavlink from FC #74 VFR_HUD: ");
            log.print("Airspeed= "); log.print(ap74_air_spd, 2);                 // m/s    
            log.print("  Groundspeed= "); log.print(ap74_grd_spd, 2);            // m/s
            log.print("  Heading= ");  log.print(ap74_hdg);                      // deg
            log.print("  Throttle %= ");  log.print(ap74_throt);                 // %
            log.print("  Baro alt= "); log.print(ap74_amsl, 0);                  // m                  
            log.print("  Climb rate= "); log.println(ap74_climb);                // m/s
          #endif              
        case MAVLINK_MSG_ID_RADIO_STATUS:         // #109
          if (!mavGood) break;
            ap109_rssi = mavlink_msg_radio_status_get_rssi(&msg);            // air signal strength
            rssi109 = true;             
            // If we get #109 then it must be a SiK fw radio, so use this record for rssi
            rssiGood=true;  
            ap_rssi = ap109_rssi / 2.54;  // 254 -> 100%   
            pt_rssi = ap_rssi;  
            break;    
          
        case MAVLINK_MSG_ID_BATTERY_STATUS:      // #147   https://mavlink.io/en/messages/common.html
          if (!mavGood) break;       
          ap_battery_id = mavlink_msg_battery_status_get_id(&msg);  
          ap_current_battery = mavlink_msg_battery_status_get_current_battery(&msg);      // cA (10*milliamperes) (1 = 10 milliampere)
          ap_current_consumed = mavlink_msg_battery_status_get_current_consumed(&msg);    // mAh
          ap147_battery_remaining = mavlink_msg_battery_status_get_battery_remaining(&msg);  // (0%: 0, 100%: 100)  

          if (ap_battery_id == 0) {  // Battery 1
            pt_bat1_mAh = ap_current_consumed;                       
          } else if (ap_battery_id == 1) {  // Battery 2
              pt_bat2_mAh = ap_current_consumed;                              
          } 
          
          hud_bat1_mAh = pt_bat1_mAh;
           
          #if defined Mav_DEBUG_All || defined DEBUG_Batteries
            log.print("Mavlink from FC #147 Battery Status: ");
            log.print(" bat id= "); log.print(ap_battery_id); 
            log.print(" bat current mA= "); log.print(ap_current_battery*10); // now shows mA
            log.print(" ap_current_consumed mAh= ");  log.print(ap_current_consumed);   
            if (ap_battery_id == 0) {
              log.print(" my di/dt mAh= ");  
              log.println(Total_mAh1(), 0);  
            }
            else {
              log.print(" my di/dt mAh= ");  
              log.println(Total_mAh2(), 0);   
            }    
        //  log.print(" bat % remaining= ");  log.println(ap_time_remaining);       
          #endif                        
          
          break;          
      }
      
  }
#endif  
} 
  
 //********************************************************************************
#if ((MEDIUM_IN == 2) && (WIFI_PROTOCOL == 1))     //  WiFi TCP

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
#if ((MEDIUM_IN == 2) && (WIFI_PROTOCOL == 2))     //  WiFi UDP
bool Read_UDP(mavlink_message_t* msgptr)  {
    if (!wifiSuGood) return false;  
    bool msgRcvd = false;
    mavlink_status_t _status;
    len = mav_udp_object.parsePacket();
    int UDP_count = len;
    if(UDP_count > 0) {
        while(UDP_count--)  {
            int result = mav_udp_object.read();
            if (result >= 0)  {
                msgRcvd = mavlink_parse_char(MAVLINK_COMM_2, result, msgptr, &_status);
                if(msgRcvd) {   
                    UDP_remoteIP = mav_udp_object.remoteIP();  // remember which remote client sent this packet so we can target it
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
#if (MEDIUM_IN == 3)           // Mavlink Bluetooth

bool Read_Bluetooth(mavlink_message_t* msgptr)  
{  
    bool msgRcvd = false;
    mavlink_status_t _status;

    len = inSerial.available();
    uint16_t bt_count = len;
    if(bt_count > 0) 
    {
        while(bt_count--)  
        {
            int result = inSerial.read();
            if (result >= 0)  {

                msgRcvd = mavlink_parse_char(MAVLINK_COMM_2, result, msgptr, &_status);
                if(msgRcvd) 
                {
                    if(!hb_heard_from) 
                    {
                        if(msgptr->msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                            hb_heard_from     = true;
                            hb_system_id      = msgptr->sysid;
                            hb_comp_id        = msgptr->compid;
                            hb_seq_expected   = msgptr->seq + 1;
                            hb_last_heartbeat = millis();
                        }
                    } else 
                    {
                      if(msgptr->msgid == MAVLINK_MSG_ID_HEARTBEAT)
                      {
                      hb_last_heartbeat = millis();
                      }
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
void Send_To_FC(uint32_t msg_id) {
  
  #if (MEDIUM_IN == 1)              // Serial to FC
    len = mavlink_msg_to_send_buffer(sendbuf, &sendmsg);
    inSerial.write(sendbuf,len);  
         
    #if defined  DEBUG_FC_Write
      if (msg_id) {    //  dont print heartbeat - too much info
        log.printf("Write to FC Serial: len=%d\n", len);             
        PrintMavBuffer(&sendmsg);
      }  
    #endif    
  #endif

  #if (MEDIUM_IN == 3)              // Bluetooth to FC   
        bool msgSent = Send_Bluetooth(&sendmsg);      
        #ifdef  DEBUG_FC_Write
          log.print("Write to FC Bluetooth: msgSent="); log.println(msgSent);
          if (msgSent) PrintMavBuffer(&sendmsg);
        #endif     
  #endif

  #if (MEDIUM_IN == 2)              // WiFi to FC  

      if (wifiSuGood) { 
        #if (WIFI_PROTOCOL == 1)      // TCP/IP
           active_client_idx = 0;             // we only ever need 1
           bool msgSent = Send_TCP(&sendmsg);  // to FC   
           #ifdef  DEBUG_FC_Write
             log.print("Write to FC WiFi TCP: msgSent="); log.println(msgSent);
             PrintMavBuffer(&sendmsg);
           #endif    
         #endif   
         #if (WIFI_PROTOCOL == 2)       // UDP 
           active_client_idx = 0;             // we only ever need 1 here   
           bool msgSent = Send_UDP(&sendmsg);     // to FC    
           #ifdef  DEBUG_FC_Write
             log.print("Write to FC WiFi UDP: msgSent="); log.println(msgSent);
             if (msgSent) PrintMavBuffer(&sendmsg);
           #endif           
         #endif                                                             
      }
   
  #endif       
}  
//================================================================================================= 
  #if (MEDIUM_IN == 3)     //  BT
  bool Send_Bluetooth(mavlink_message_t* msgptr) {
    bool msgSent = false;
    uint16_t len = mavlink_msg_to_send_buffer(sendbuf, msgptr);
    size_t sent = inSerial.write(sendbuf,len);
    if (sent == len) {
      msgSent = true;
      link_status.packets_sent++;
    }
    return msgSent;
  }
  #endif
//================================================================================================= 
#if (MEDIUM_IN == 2) 
  #if (WIFI_PROTOCOL == 1)     //  TCP
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
#if (MEDIUM_IN == 2) 
  #if (WIFI_PROTOCOL == 2)     //  UDP
  bool Send_UDP(mavlink_message_t* msgptr) {
    if (!wifiSuGood) return false;  
    bool msgSent = false;

    if (msgptr->msgid == MAVLINK_MSG_ID_HEARTBEAT) {
      UDP_remoteIP = localIP;
      UDP_remoteIP[3] = 255;       // always broadcast a heartbeat on the local LAN, either from the GCS or from the FC                 
   //   log.print("Broadcast heartbeat UDP_remoteIP="); log.println(UDP_remoteIP.toString());     
    } 

    mav_udp_object.beginPacket(UDP_remoteIP, UDP_remotePort);

    uint16_t len = mavlink_msg_to_send_buffer(sendbuf, msgptr);
  
    size_t sent = mav_udp_object.write(sendbuf,len);

    if (sent == len) {
      msgSent = true;
      link_status.packets_sent++;
      mav_udp_object.flush();
    }

    bool endOK = mav_udp_object.endPacket();
 //   if (!endOK) {
 //       log.printf("msgSent=%d   endOK=%d\n", msgSent, endOK);
 // }
    return msgSent;
  }
  #endif
#endif  
//********************************************************************************
#if (MEDIUM_IN == 3)  ||  (MEDIUM_IN == 2) || (MEDIUM_IN == 3)        // Bluetooth or WiFi
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
  
  apo_sysid = DEVICE_SYSID;                    // From config.h MP is 255, QGC default is 0
  apo_compid = DEVICE_COMPID;                  // 158 Generic autopilot peripheral component ID. MP is 190
  apo_targsys = 1;                             // FC
  apo_targcomp = 1;                            // FC                   

  apo_type = MAV_TYPE_GCS;                       // 6 Pretend to be a GCS
 // apo_type = MAV_TYPE_ANTENNA_TRACKER;         // 5 
  apo_autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA; // 3 AP Mega
  apo_base_mode = 0;
  apo_system_status = MAV_STATE_ACTIVE;         // 4
   
  mavlink_msg_heartbeat_pack(apo_sysid, apo_compid, &sendmsg, apo_type, apo_autopilot, apo_base_mode, apo_system_status, 0); 
  Send_To_FC(0); 
  #if defined DEBUG_Our_FC_Heartbeat
     log.print("Our own heartbeat to FC: #0 Heartbeat: ");  
     log.print("apo_sysid="); log.print(apo_sysid);   
     log.print("  apo_compid="); log.print(apo_compid);  
     log.print("  apo_targsys="); log.print(apo_targsys);   
     log.print("  apo_targcomp="); log.print(apo_targcomp);                         
     log.print("  apo_type="); log.print(apo_type);   
     log.print("  apo_autopilot="); log.print(apo_autopilot); 
     log.print("  apo_base_mode="); log.print(apo_base_mode); 
     log.print("  apo_custom_mode="); log.print(apo_custom_mode);
     log.print("  apo_system_status="); log.print(apo_system_status);    
     log.println();
  #endif   
} 
//================================================================================================= 
#ifdef Data_Streams_Enabled    
void RequestDataStreams() {    //  REQUEST_DATA_STREAM ( #66 ) DEPRECATED. USE SRx, SET_MESSAGE_INTERVAL INSTEAD

  apo_sysid = 255;              // Reply to APM FC
  apo_compid = 190;             // Reply to APM FC
  apo_targsys = 1;              // FC
  apo_targcomp = 1;             // FC

  const int maxStreams = 7;
  const uint8_t mavStreams[] = {
  MAV_DATA_STREAM_RAW_SENSORS,
  MAV_DATA_STREAM_EXTENDED_STATUS,
  MAV_DATA_STREAM_RC_CHANNELS,
  MAV_DATA_STREAM_POSITION,
  MAV_DATA_STREAM_EXTRA1, 
  MAV_DATA_STREAM_EXTRA2,
  MAV_DATA_STREAM_EXTRA3
  };

  const uint16_t mavRates[] = { 0x04, 0x0a, 0x04, 0x0a, 0x04, 0x04, 0x04};
 // req_message_rate The requested interval between two messages of this type

  for (int i=0; i < maxStreams; i++) {
    mavlink_msg_request_data_stream_pack(apo_sysid, apo_compid, &sendmsg,
        apo_targsys, apo_targcomp, mavStreams[i], mavRates[i], 1);    // start_stop 1 to start sending, 0 to stop sending   
                          
  Send_To_FC(66);
    }
 // log.println("Mavlink to FC #66 Request Data Streams:");
}
#endif
 
