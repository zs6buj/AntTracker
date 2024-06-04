//=================================================================================================  
//================================================================================================= 
//
//                                       U T I L I T I E S
//
//================================================================================================= 
//=================================================================================================   

String TimeString (unsigned long epoch){
 uint8_t hh = (epoch  % 86400L) / 3600;   // remove the days (86400 secs per day) and div the remainer to get hrs
 uint8_t mm = (epoch  % 3600) / 60;       // calculate the minutes (3600 ms per minute)
 uint8_t ss = (epoch % 60);               // calculate the seconds

  String S = "";
  if (hh<10) S += "0";
  S += String(hh);
  S +=":";
  if (mm<10) S += "0";
  S += String(mm);
  S +=":";
  if (ss<10) S += "0";
  S += String(ss);
  return S;
}
//=================================================================================================  
void PrintMavBuffer(const void *object) {
  
    const unsigned char * const bytes = static_cast<const unsigned char *>(object);
  int j;

uint8_t   tl;

uint8_t mavNum;

//Mavlink 1 and 2
uint8_t mav_magic;               // protocol magic marker
uint8_t mav_len;                 // Length of payload

//uint8_t mav_incompat_flags;    // MAV2 flags that must be understood
//uint8_t mav_compat_flags;      // MAV2 flags that can be ignored if not understood

uint8_t mav_seq;                // Sequence of packet
//uint8_t mav_sysid;            // ID of message sender system/aircraft
//uint8_t mav_compid;           // ID of the message sender component
uint8_t mav_msgid;            
/*
uint8_t mav_msgid_b1;           ///< first 8 bits of the ID of the message 0:7; 
uint8_t mav_msgid_b2;           ///< middle 8 bits of the ID of the message 8:15;  
uint8_t mav_msgid_b3;           ///< last 8 bits of the ID of the message 16:23;
uint8_t mav_payload[280];      ///< A maximum of 255 payload bytes
uint16_t mav_checksum;          ///< X.25 CRC_Out
*/

  
  if ((bytes[0] == 0xFE) || (bytes[0] == 0xFD)) {
    j = -2;   // relative position moved forward 2 places
  } else {
    j = 0;
  }
   
  mav_magic = bytes[j+2];
  if (mav_magic == 0xFE) {  // Magic / start signal
    mavNum = 1;
  } else {
    mavNum = 2;
  }
/* Mav1:   8 bytes header + payload
 * magic
 * length
 * sequence
 * sysid
 * compid
 * msgid
 */
  
  if (mavNum == 1) {
    log.print("mav1: /");

    if (j == 0) {
      Printbyte(bytes[0], false, ' '); // CRC_Out1
      Printbyte(bytes[1], false, ' '); // CRC_Out2

      log.print("/");
      }
    mav_magic = bytes[j+2];   
    mav_len = bytes[j+3];
 //   mav_incompat_flags = bytes[j+4];;
 //   mav_compat_flags = bytes[j+5];;
    mav_seq = bytes[j+6];
 //   mav_sysid = bytes[j+7];
 //   mav_compid = bytes[j+8];
    mav_msgid = bytes[j+9];

    //log.print(TimeString(millis()/1000)); log.print(": ");
  
    log.print("seq="); log.print(mav_seq); log.print("\t"); 
    log.print("len="); log.print(mav_len); log.print("\t"); 
    log.print("/");
    for (int i = (j+2); i < (j+10); i++) {  // Print the header
       Printbyte(bytes[i], false, ' '); 
    }
    
    log.print("  ");
    log.print("#");
    log.print(mav_msgid);
    if (mav_msgid < 100) log.print(" ");
    if (mav_msgid < 10)  log.print(" ");
    log.print("\t");
    
    tl = (mav_len+10);                // Total length: 8 bytes header + Payload + 2 bytes CRC_Out
 //   for (int i = (j+10); i < (j+tl); i++) {  
    for (int i = (j+10); i <= (tl); i++) {    
      Printbyte(bytes[i], false, ' '); 
    }
    if (j == -2) {
      log.print("//");
      Printbyte(bytes[mav_len + 8], false, ' '); 
      Printbyte(bytes[mav_len + 9], false, ' ');       
      }
    log.println("//");  
  } else {

/* Mav2:   10 bytes
 * magic
 * length
 * incompat_flags
 * mav_compat_flags 
 * sequence
 * sysid
 * compid
 * msgid[11] << 16) | [10] << 8) | [9]
 */
    
    log.print("mav2:  /");
    if (j == 0) {
      Printbyte(bytes[0], false, ' '); // CRC_Out1 
      Printbyte(bytes[1], false, ' '); // CRC_Out2            
      log.print("/");
    }
    mav_magic = bytes[2]; 
    mav_len = bytes[3];
//    mav_incompat_flags = bytes[4]; 
  //  mav_compat_flags = bytes[5];
    mav_seq = bytes[6];
//    mav_sysid = bytes[7];
   // mav_compid = bytes[8]; 
    mav_msgid = (bytes[11] << 16) | (bytes[10] << 8) | bytes[9]; 

    //log.print(TimeString(millis()/1000)); log.print(": ");

    log.print("seq="); log.print(mav_seq); log.print("\t"); 
    log.print("len="); log.print(mav_len); log.print("\t"); 
    log.print("/");
    for (int i = (j+2); i < (j+12); i++) {  // Print the header
     Printbyte(bytes[i], false, ' '); 
    }

    log.print("  ");
    log.print("#");
    log.print(mav_msgid);
    if (mav_msgid < 100) log.print(" ");
    if (mav_msgid < 10)  log.print(" ");
    log.print("\t");

 //   tl = (mav_len+27);                // Total length: 10 bytes header + Payload + 2B CRC_Out +15 bytes signature
    tl = (mav_len+22);                  // This works, the above does not!
    for (int i = (j+12); i < (tl+j); i++) {   
       if (i == (mav_len + 12)) {
        log.print("/");
      }
      if (i == (mav_len + 12 + 2+j)) {
        log.print("/");
      }
      Printbyte(bytes[i], false, ' ');       
    }
    log.println();
  }

   log.print("Raw: ");
   for (int i = 0; i < 40; i++) {  //  unformatted
      Printbyte(bytes[i], false, ' ');  
    }
   log.println();
  
}
//======================================= 
void Printbyte(byte b, bool LF, char delimiter) {
  
  if ( (b == 0x7E) && (LF) ) {//             || (b == 0x10)  || (b == 0x32)) {
    log.println();
  } 
  if (b<=0xf) log.print("0");
  log.print(b,HEX);
  log.write(delimiter);
}

//==================================
void printByte(byte b) {
  if (b<=0xf) log.print("0");
    log.print(b,HEX);
    log.print(" ");
}

//======================================= 
void printBuffer(int lth){
  for ( int i = 0; i < lth; i++ ) {
    Printbyte(inBuf[i], false, ' ');
  }
  log.println();
}
//======================================= 
void PrintFrsBuffer(byte *buf, uint8_t len){
  log.print("len:"); log.print(len); log.print("  ");
  for ( int i = 0; i < len; i++ ) {
    Printbyte(buf[i], false, ' ');
  }
  log.println();
}
//=======================================
void scanI2C()
// 0x03C for SSD1306_Display, 0x1E for HMC5883L, 0x0D for QMC5883
{
  uint8_t found = 0;
  log.print("Scanning I2C bus: ");
  for(int addr = 1; addr < 127; addr++ ) 
  {
    Wire.beginTransmission(addr);
      uint8_t err = Wire.endTransmission();
    if (err == 0) 
    {
      log.print("I2C device found at 0x");
      printByte(addr);
      if (addr == 0x0C)
      {
        log.println("likely SSD1306_Display");
      } else
      if (addr == 0x0D)
      {
        log.println("likely QMC5883L compass");
      } else
      if (addr == 0x0E)
      {
        log.println("likely HMC5883L compass");
      }
      found++;
    }
    else if (err==4) 
    {
      log.print("Unknown error at addr 0x");
      printByte(addr);
    }    
  }
  if (found == 0) 
  {
    log.println("No I2C devices found");
  }
}
//=======================================
bool PacketGood() 
{
  // Allow 1 degree of lat and lon away from home, i.e. 60 nautical miles radius at the equator
  // Allow 1km up and 300m down from home altitude
  if (finalHomeStored==0) {  //  You can't use the home co-ordinates for a reasonability test if you don't have them yet
    return true;
    }
  if (cur.lat<(hom.lat-1.0) || cur.lat>(hom.lat+1.0)) {  // Also works for negative lat
    #if defined SHOW_BAD_PACKETS
      log.print(" Bad lat! cur.lat=");
      log.print(cur.lat,7);  
      log.print(" hom.lat=");log.print(hom.lat,7);
      log.println("  Packet ignored");  
    #endif
    return false; 
    }
  if (cur.lon<(hom.lon-1.0) || cur.lon>(hom.lon+1.0)) { // Also works for negative lon
    #if defined SHOW_BAD_PACKETS   
      log.print(" Bad lon! cur.lon=");
      log.print(cur.lon,7);  
      log.print(" hom.lon=");log.print(hom.lon,7);
      log.println("  Packet ignored");
    #endif  
    return false;  
    }
  if (cur.alt<(hom.alt-300) || cur.alt>(hom.alt+2000)) {
    #if defined SHOW_BAD_PACKETS    
      log.print(" Bad alt! cur.alt=");
      log.print(cur.alt,0);  
      log.print(" hom.alt=");log.print(hom.alt,0);
      log.println("  Packet ignored");
    #endif    
    return false;  
    }
  if ((cur.alt-hom.alt)<-300 || (cur.alt-hom.alt)>2000) {
      log.print(" Bad alt! cur.alt=");
      log.print(cur.alt,0);  
      log.print(" hom.alt=");log.print(hom.alt,0);
      log.println("  Packet ignored");    
    return false;  
    }
  if (headingsource == 2) { //  Heading source from flight controller
    if (cur.hdg<0 || cur.hdg>360) {
      #if defined SHOW_BAD_PACKETS
        log.print(" Bad hdg! cur.hdg=");
        log.print(cur.hdg,0);  
        log.print(" hom.hdg=");log.print(hom.hdg,0);
        log.println("  Packet ignored");    
      #endif
      return false;  
     }
 }
  
return true;
}
//=================================================================================================  

  void CheckStatusAndTimeouts() 
  {
    if ((millis() - btGood_millis) > ((timeout_secs-1) * 1000) ) 
    {
      btGood = false;        // If no BT traffic 
    }        
    if ((millis() - hbGood_millis) > ((2*timeout_secs) * 1000)) 
    {
       mavGood = false; 
       hbGood = false;         
     }    
    if ((millis() - frGood_millis) > (timeout_secs * 1000)) 
    {
       frGood = false;
       pwmGood = false;  
    }
    if ((millis() - pwmGood_millis) > (timeout_secs * 1000) ) 
    {
       pwmGood = false;
    }
    if ((millis() - gpsGood_millis) > (timeout_secs * 1000) ) 
    {
      gpsGood = false;        // If no meaGPS packet  
      new_GPS_data = true;   
    }   
    #if (headingsource == 4)
      if ((millis() - boxgpsGood_millis) > (timeout_secs * 1000) ) {
        boxgpsGood = false;        // If no box GPS packet  
      }  
    #endif
    ReportOnlineStatus();
    ServiceTheStatusLed();  
  }   
    //===================================================================     
    
  void ReportOnlineStatus() 
  {
      if (btGood != btPrev) 
      {  
        btPrev = btGood;
        if (btGood) {
           log.println("Good BT data stream");
           LogScreenPrintln("Good BT stream");         
         } else {
          log.println("BT data stream timeout *");
          LogScreenPrintln("BT stream timeout");         
        }
      }  
      if (motArmed != motPrev)
      {  // report on change of status
         motPrev = motArmed;
         if (motArmed) {
           log.println("Motors Armed!");
           LogScreenPrintln("Motors Armed!");         
         } else {
          log.println("Motors Disarmed");
          LogScreenPrintln("Motors Disarmed");         
         }
      }      
      if (frGood != frPrev) 
      {  // report on change of status
         frPrev = frGood;
         if (frGood) {
           log.println("FrSky read good!");
           LogScreenPrintln("FrSky read ok");         
         } else {
          log.println("FrSky read timeout!");
          LogScreenPrintln("FrSky timeout");         
         }
      }
      if (pwmGood != pwmPrev) 
      {  
         pwmPrev = pwmGood;
         if (pwmGood) 
         {
           log.println("RC PWM good");
           LogScreenPrintln("RC PWM good");         
         } else {
          log.println("RC PWM timeout");
          LogScreenPrintln("RC PWM timeout");         
         }
      }        
       if (gpsGood != gpsPrev) {  
         gpsPrev = gpsGood;
         if (gpsGood) {
           log.println("Good flight computer GPS lock");
           LogScreenPrintln("FC GPS good");         
         } else {
          log.println("Flight Computer GPS timeout *********");
          LogScreenPrintln("FC GPS timeout");         
         }
       }    
       #if (headingsource == 4)
         if (boxgpsGood != boxgpsPrev) {  
           boxgpsPrev = boxgpsGood;
           if (boxgpsGood) {
             log.println("Good box GPS lock");
             LogScreenPrintln("Box GPS good");         
           } else {
            log.println("No box GPS lock!");
            LogScreenPrintln("No box GPS lock!");         
           }
         }     
       #endif           
  } 

//=================================================================================================  
uint32_t epochNow() {
return (epochSync + (millis() - millisSync) / 1E3);
}
//=================================================================================================  
void LostPowerCheckAndRestore(uint32_t epoch_now) { // only ever called if active time supporting protocol 
  if ((!timeGood) || (epoch_now == 0)) return;
  
  if (lostPowerCheckDone) return;

  #if defined DEBUG_All || defined DEBUG_Time || defined DEBUG_Home
    log.print("Checking for RestoreHomeFromFlash conditions:"); 
    log.print("  epochHome="); log.print(TimeString(epochHome())); 
    log.print("  epochNow="); log.println(TimeString(epochNow()));
  #endif 

  #if (HEADINGSOURCE != 4)  // If NOT (Tracker GPS + Compass). Home could move constantly.
    uint16_t decay_secs = epoch_now -  epochHome();  
    if (decay_secs <= home_decay_secs) {  //  restore home if restart within decay seconds
      RestoreHomeFromFlash();     
      log.printf("Home data restored from NVM, decay %d secs is within limit of %d secs\n", decay_secs, home_decay_secs);     
      //log.printf("Home data restored from NVM, decay %d secs is within limit of %d secs\n", decay_secs, home_decay_secs);     
      LogScreenPrintln("Home data restored");
      LogScreenPrintln("from Flash. Go Fly!");  
      finalHomeStored=1;           
    }
  #endif
  
  lostPowerCheckDone = true;
}
//=================================================================================================  
void SaveHomeToFlash() {

  EEPROMWritelong(0, epochNow());   // epochHome
  EEPROMWritelong(1, hom.lon*1E6);  // float to long
  EEPROMWritelong(2, hom.lat*1E6);
  EEPROMWritelong(3, hom.alt*10);
  EEPROMWritelong(4, hom.hdg*10);  
  
#if defined DEBUG_All || defined DEBUG_EEPROM || defined DEBUG_Time || defined DEBUG_Home
  log.print("  firstHomeStored="); log.print(firstHomeStored);
  log.print("  home.lon="); log.print(hom.lon, 6);
  log.print("  home.lat="); log.print(hom.lat, 6);
  log.print("  home.alt="); log.print(hom.alt, 1);
  log.print("  home.hdg="); log.println(hom.hdg, 1);

#endif   
}

//=================================================================================================  
void StoreEpochPeriodic() {
  uint32_t epochPeriodic = epochNow();
  EEPROMWritelong(5, epochPeriodic); // Seconds

  if (finalHomeStored) {
    EEPROMWritelong(0, epochPeriodic); // UPDATE epochHome
    #if defined DEBUG_All || defined DEBUG_EEPROM || defined DEBUG_Time || defined DEBUG_Home
      log.print("epochHome stored="); log.println(TimeString(epochPeriodic));
    #endif  
  }
  
  #if defined DEBUG_All || defined DEBUG_EEPROM || defined DEBUG_Time || defined DEBUG_Home
  log.print("epochPeriodic stored="); log.println(TimeString(epochPeriodic));
  #endif  
}

//=================================================================================================  
uint32_t epochHome() {

uint32_t epHome = EEPROMReadlong(0);

 #if defined DEBUG_All || defined DEBUG_EEPROM
   log.print("epochHome="); log.println(TimeString(epHome));

 #endif
 return epHome;    
}
//=================================================================================================  
void RestoreHomeFromFlash() {

  hom.lon = EEPROMReadlong(1) / 1E6; //long back to float
  hom.lat = EEPROMReadlong(2) / 1E6;
  hom.alt = EEPROMReadlong(3) / 10;
  hom.hdg = EEPROMReadlong(4) / 10;
  
  #if defined DEBUG_All || defined DEBUG_EEPROM || defined DEBUG_Time || defined DEBUG_Home
    log.print("  home.lon="); log.print(hom.lon, 6);
    log.print("  home.lat="); log.print(hom.lat, 6);
    log.print("  home.alt="); log.print(hom.alt, 0);
    log.print("  home.hdg="); log.println(hom.hdg, 0);
  #endif  
}
//=================================================================================================  
  
#if (MEDIUM_IN == 2) ||  (MEDIUM_IN == 2)   //  Mav WiFi or FrSky UDP
 
  void SetupWiFi() { 

    bool apMode = false;  // used when STA fails to connect
     //===============================  S T A T I O N   =============================
     
    #if (WiFi_Mode == 2) || (WiFi_Mode == 3)  // STA or SPA>AP
      uint8_t retry = 0;
      log.print("Trying to connect to ");  
      log.print(STAssid); 
      LogScreenPrintln("WiFi trying ..");

      WiFi.disconnect(true);   // To circumvent "wifi: Set status to INIT" error bug
      delay(500);
      WiFi.mode(WIFI_STA);
      delay(500);
      
      WiFi.begin(STAssid, STApw);
      while (WiFi.status() != WL_CONNECTED){
        retry++;
        if (retry > 20) {
          log.println();
          log.println("Failed to connect in STA mode");
          LogScreenPrintln("Failed in STA Mode");
          wifiSuDone = true;
          
          #if (WiFi_Mode ==  3)       
            apMode = true;            // Rather go establish an AP instead
            log.println("Starting AP instead.");
            LogScreenPrintln("Starting AP instead");  
            //new from Target0815:
            log.println("WiFi-Reset ...");
            WiFi.mode(WIFI_MODE_NULL);    
            delay(500);             
          #endif  
          
          break;
        }
        delay(500);
        Serial.print(".");
      }
      
      if (WiFi.status() == WL_CONNECTED) {
        localIP = WiFi.localIP();
        log.println();
        log.println("WiFi connected!");
        log.print("Local IP address: ");
        log.print(localIP);

        #if (MEDIUM_IN == 2) && (WIFI_PROTOCOL == 1)   // Mav TCP
            log.print("  TCP port: ");
            log.println(TCP_localPort);    //  UDP port is printed lower down
        #else 
            log.println();
        #endif 
         
        wifi_rssi = WiFi.RSSI();
        log.print("WiFi RSSI:");
        log.print(wifi_rssi);
        log.println(" dBm");

        LogScreenPrintln("Connected! My IP =");
        LogScreenPrintln(localIP.toString());
        
        #if (WIFI_PROTOCOL == 1)        // TCP                                                
          #if (MEDIUM_IN  == 2)     // We are a client and need to connect to a server
             outbound_clientGood = NewOutboundTCPClient();
          #endif
        #endif
        #if (MEDIUM_IN == 2)                // Mavlink 
          #if (WIFI_PROTOCOL == 2)         // Mav UDP 
            mav_udp_object.begin(UDP_localPort);
            log.printf("Mav UDP instance started, listening on IP %s, UDP local port %d\n", localIP.toString().c_str(), UDP_localPort);                 
            LogScreenPrint("UDP port = ");  LogScreenPrintln(String(UDP_localPort));
          #endif
        #endif

        #if (MEDIUM_IN == 2)               // FrSky
          frs_udp_object.begin(UDP_localPort+1);
          log.printf("Frs UDP instance started, listening on IP %s, UDP port %d\n", localIP.toString().c_str(), UDP_localPort+1);                  
          LogScreenPrint("UDP port = ");  LogScreenPrintln(String(UDP_localPort+1));       
        #endif
        
        wifiSuDone = true;
        wifiSuGood = true;
      } 
    #endif
     //===============================  Access Point   =============================  

    #if (WiFi_Mode == 1)  // AP
      apMode = true;
    #endif

    if (apMode)   {
      WiFi.mode(WIFI_AP);
      WiFi.softAP(APssid, APpw, APchannel);
      localIP = WiFi.softAPIP();
      log.print("AP IP address: ");
      log.print (localIP); 
      log.print("  SSID: ");
      log.println(String(APssid));
      LogScreenPrintln("WiFi AP SSID =");
      LogScreenPrintln(String(APssid));

        #if (WIFI_PROTOCOL == 1)              // TCP                                                
          #if (MEDIUM_IN  == 2)            // We are a client and need to connect to a server
             outbound_clientGood = NewOutboundTCPClient();
          #endif
        #endif
        #if (MEDIUM_IN == 2)                // Mavlink 
          #if (WIFI_PROTOCOL == 2)             // Mav UDP 
            mav_udp_object.begin(UDP_localPort);
            log.printf("Mav UDP instance started, listening on IP %s, UDP port %d\n", localIP.toString().c_str(), UDP_localPort);              
            LogScreenPrint("UDP port = ");  LogScreenPrintln(String(UDP_localPort));
          #endif
        #endif

        #if (MEDIUM_IN == 2)                // FrSky
          frs_udp_object.begin(UDP_localPort);
          log.printf("Frs UDP instance started, listening on IP %s, UDP port %d\n", localIP.toString().c_str(), UDP_localPort);         
          LogScreenPrint("UDP port = ");  LogScreenPrintln(String(UDP_localPort));       
        #endif
      
      wifiSuGood = true;
 
    }           

      
    #ifndef Start_WiFi  // if not button override
      delay(2000);      // debounce button press
    #endif  

    wifiSuDone = true;
 }   

  //=================================================================================================  
  #if (WIFI_PROTOCOL == 1)    //  WiFi TCP    
  
  bool NewOutboundTCPClient() {
  static uint8_t retry = 3;

    WiFiClient newClient;        
    while (!newClient.connect(TCP_remoteIP, TCP_remotePort)) {
      log.print("Local outgoing tcp client connect failed, retrying ");  log.println(retry);
      retry--;
      if (retry == 0) {
         log.println("Tcp client connect aborted!");
         return false;
      }
      nbdelay(4000);
    }
    active_client_idx = 0;     // use the first client object for  our single outgoing session   
    clients[0] = new WiFiClient(newClient); 
    log.print("Local tcp client connected to remote server IP:"); log.print(TCP_remoteIP);
    log.print(" remote Port:"); log.println(TCP_remotePort);
    nbdelay(1000);
    LogScreenPrintln("Remote server IP =");
    log.printf("%d.%d.%d.%d", TCP_remoteIP[0], TCP_remoteIP[1], TCP_remoteIP[2], TCP_remoteIP[3]);               
    
 //   LogScreenPrintln(TCP_remoteIP.toString()); 
    return true;
  }
  #endif
  //=================================================================================================  
   #if (MEDIUM_IN == 2) &&  (WIFI_PROTOCOL == 2) //  WiFi && UDP - Print the remote UDP IP the first time we get it  
   void PrintRemoteIP() {
    if (FtRemIP)  {
      FtRemIP = false;
      log.print("Client connected: Remote UDP IP: "); log.print(UDP_remoteIP);
      log.print("  Remote  UDP port: "); log.println(UDP_remotePort);
      LogScreenPrintln("Client connected");
      LogScreenPrintln("Remote UDP IP =");
      LogScreenPrintln(UDP_remoteIP.toString());
      LogScreenPrintln("Remote UDP port =");
      LogScreenPrintln(String(UDP_remotePort));
     }
  }
  #endif
  
 #endif  

  //=================================================================================================  

  uint32_t Get_Volt_Average1(uint16_t mV)  {
    if (bat1.avg_mV < 1) bat1.avg_mV = mV;  // Initialise first time
    bat1.avg_mV = (bat1.avg_mV * 0.6666) + (mV * 0.3333);  // moving average
    return bat1.avg_mV;
  }
  //=================================================================================================  
uint32_t Get_Current_Average1(uint16_t cA)  {   // in 100*milliamperes (1 = 100 milliampere)
 
  if (bat1.avg_cA < 1){
    bat1.avg_cA = cA;  // Initialise first time
  }
  bat1.avg_cA = (bat1.avg_cA * 0.6666F) + (cA * 0.333F);  // moving average
  return bat1.avg_cA;
  }
  //=================================================================================================   

  uint32_t Abs(int32_t num) {
    if (num<0) 
      return (num ^ 0xffffffff) + 1;
    else
      return num;  
  } 
//=================================================================================================  
float RadToDeg(float _Rad) {
  return _Rad * 180 / PI;  
} 
//=================================================================================================  
//Add two bearing in degrees and correct for 360 boundary
int16_t Add360(int16_t arg1, int16_t arg2) {  
  int16_t ret = arg1 + arg2;
  if (ret < 0) ret += 360;
  if (ret > 359) ret -= 360;
  return ret; 
}  
//================================================================================================= 
#if (defined ESP32)   && ( (MEDIUM_IN == 2) || (MEDIUM_IN == 2)) && (defined DEBUG_WiFi)
void WiFiEventHandler(WiFiEvent_t event)  {
    log.printf("[WiFi-event] event: %d ", event);        

    switch (event) {
        case SYSTEM_EVENT_WIFI_READY: 
            log.println("WiFi interface ready");
            break;
        case SYSTEM_EVENT_SCAN_DONE:
            log.println("Completed scan for access points");
            break;
        case SYSTEM_EVENT_STA_START:
            log.println("WiFi client started");
            break;
        case SYSTEM_EVENT_STA_STOP:
            log.println("WiFi client stopped");
            break;
        case SYSTEM_EVENT_STA_CONNECTED:
            log.println("Connected to access point");
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            log.println("Disconnected from WiFi access point");
            break;
        case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:
            log.println("Authentication mode of access point has changed");
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            log.print("Obtained IP address: ");
            log.println(WiFi.localIP());
            break;
        case SYSTEM_EVENT_STA_LOST_IP:
            log.println("Lost IP address and IP address is reset to 0");
            break;
        case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:
            log.println("WiFi Protected Setup (WPS): succeeded in enrollee mode");
            break;
        case SYSTEM_EVENT_STA_WPS_ER_FAILED:
            log.println("WiFi Protected Setup (WPS): failed in enrollee mode");
            break;
        case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:
            log.println("WiFi Protected Setup (WPS): timeout in enrollee mode");
            break;
        case SYSTEM_EVENT_STA_WPS_ER_PIN:
            log.println("WiFi Protected Setup (WPS): pin code in enrollee mode");
            break;
        case SYSTEM_EVENT_AP_START:
            log.println("WiFi access point started");
            break;
        case SYSTEM_EVENT_AP_STOP:
            log.println("WiFi access point  stopped");
            break;
        case SYSTEM_EVENT_AP_STACONNECTED:
            log.println("Client connected");
            break;
        case SYSTEM_EVENT_AP_STADISCONNECTED:
            log.println("Client disconnected");
            break;
        case SYSTEM_EVENT_AP_STAIPASSIGNED:
            log.println("Assigned IP address to client");
            break;
        case SYSTEM_EVENT_AP_PROBEREQRECVED:
            log.println("Received probe request");
            break;
        case SYSTEM_EVENT_GOT_IP6:
            log.println("IPv6 is preferred");
            break;
        case SYSTEM_EVENT_ETH_START:
            log.println("Ethernet started");
            break;
        case SYSTEM_EVENT_ETH_STOP:
            log.println("Ethernet stopped");
            break;
        case SYSTEM_EVENT_ETH_CONNECTED:
            log.println("Ethernet connected");
            break;
        case SYSTEM_EVENT_ETH_DISCONNECTED:
            log.println("Ethernet disconnected");
            break;
        case SYSTEM_EVENT_ETH_GOT_IP:
            log.println("Obtained IP address");
            break;
        default: break;
    }
}
#endif
/* AVAILABLE EVENTS:
0  SYSTEM_EVENT_WIFI_READY               < ESP32 WiFi ready
1  SYSTEM_EVENT_SCAN_DONE                < ESP32 finish scanning AP
2  SYSTEM_EVENT_STA_START                < ESP32 station start
3  SYSTEM_EVENT_STA_STOP                 < ESP32 station stop
4  SYSTEM_EVENT_STA_CONNECTED            < ESP32 station connected to AP
5  SYSTEM_EVENT_STA_DISCONNECTED         < ESP32 station disconnected from AP
6  SYSTEM_EVENT_STA_AUTHMODE_CHANGE      < the auth mode of AP connected by ESP32 station changed
7  SYSTEM_EVENT_STA_GOT_IP               < ESP32 station got IP from connected AP
8  SYSTEM_EVENT_STA_LOST_IP              < ESP32 station lost IP and the IP is reset to 0
9  SYSTEM_EVENT_STA_WPS_ER_SUCCESS       < ESP32 station wps succeeds in enrollee mode
10 SYSTEM_EVENT_STA_WPS_ER_FAILED        < ESP32 station wps fails in enrollee mode
11 SYSTEM_EVENT_STA_WPS_ER_TIMEOUT       < ESP32 station wps timeout in enrollee mode
12 SYSTEM_EVENT_STA_WPS_ER_PIN           < ESP32 station wps pin code in enrollee mode
13 SYSTEM_EVENT_AP_START                 < ESP32 soft-AP start
14 SYSTEM_EVENT_AP_STOP                  < ESP32 soft-AP stop
15 SYSTEM_EVENT_AP_STACONNECTED          < a station connected to ESP32 soft-AP
16 SYSTEM_EVENT_AP_STADISCONNECTED       < a station disconnected from ESP32 soft-AP
17 SYSTEM_EVENT_AP_STAIPASSIGNED         < ESP32 soft-AP assign an IP to a connected station
18 SYSTEM_EVENT_AP_PROBEREQRECVED        < Receive probe request packet in soft-AP interface
19 SYSTEM_EVENT_GOT_IP6                  < ESP32 station or ap or ethernet interface v6IP addr is preferred
20 SYSTEM_EVENT_ETH_START                < ESP32 ethernet start
21 SYSTEM_EVENT_ETH_STOP                 < ESP32 ethernet stop
22 SYSTEM_EVENT_ETH_CONNECTED            < ESP32 ethernet phy link up
23 SYSTEM_EVENT_ETH_DISCONNECTED         < ESP32 ethernet phy link down
24 SYSTEM_EVENT_ETH_GOT_IP               < ESP32 ethernet got IP from connected AP
25 SYSTEM_EVENT_MAX
*/

//=================================================================================================   
//                           D I S P L A Y   S U P P O R T   -   ESP Only - for now
//================================================================================================= 

  #if defined displaySupport  
    void HandleDisplayButtons() {

      if (millis() - last_log_millis > 15000) { // after 15 seconds default to flight info screen
        last_log_millis = millis();             // and enable toggle button again
        show_log = false;
      }
        
      if (show_log) {
        if (infoNewPress) {     
          PaintLogScreen(row, show_last_row);  // one time     
          infoNewPress = false; 
          last_log_millis = millis();
        }
      } else {            // else show flight info
        DisplayFlightInfo();             
      }
      
      //log.printf("busy=%d  new=%d log=%d  bounce=%d  info=%d\n", infoPressBusy, infoNewPress, show_log, info_debounce_millis, info_millis); 
      
     #if ((defined ESP32) || (defined ESP8266))   // Teensy does not have touch pins          
      if ( (Tup != -1) && (Tdn != -1) ) {         // if ESP touch pin-pair enumerated
        if (upButton) {
          Scroll_Display(up);
        }
        if (dnButton) {     
          Scroll_Display(down);
        }     
      } else
      #endif
      
      if ( (Pup != -1) && (Pdn != -1) ) {   // if digital pin-pair enumerated
        upButton = !digitalRead(Pup);       // low == pressed
        if (upButton) {                 
          Scroll_Display(up);
        }
        dnButton = !digitalRead(Pdn);
        if (dnButton) {
          Scroll_Display(down);
        }        
      }        
    }

    //===================================
    #if ((defined ESP32) || (defined ESP8266)) 
    void IRAM_ATTR gotButtonUp(){
      upButton = true;
    }

    void IRAM_ATTR gotButtonDn(){
      dnButton = true;  
    }
    
    void IRAM_ATTR gotButtonInfo(){
      infoButton = true;
    }
    #endif 
    //===================================
    void Scroll_Display(scroll_t up_dn) {
      
      if (millis() - scroll_millis < 300) return;
      show_log = true;    
      scroll_millis = millis(); 
      
      if (up_dn == up) {  // towards last line painted, so lines move up
         scroll_row--;
         scroll_row = constrain(scroll_row, scr_h_ch, row);
         upButton = false; 
         PaintLogScreen(scroll_row, show_last_row);   // paint down to scroll_row
      }
      if (up_dn == down) {  // towards first line painted, so lines move down
          scroll_row++; 
          scroll_row = constrain(scroll_row, scr_h_ch, row);       
          dnButton = false; 
          PaintLogScreen(scroll_row, show_last_row);  // paint down to scroll_row      
      }   
    }
    //===================================================================  
    #if defined displaySupport       
    void SetScreenSizeOrient(uint8_t txtsz, uint8_t scr_orient) {

      #if (defined SSD1306_Display) || (defined SSD1331_Display)   // rotation arguement depends on display type
        if (scr_orient == 0) {          // portrait
          display.setRotation(1);              
          scr_h_px = SCR_H_PX;
          scr_w_px = SCR_W_PX;
        } else                       
        if (scr_orient == 1) {          // landscape
          display.setRotation(0);             
          scr_h_px = SCR_W_PX;
          scr_w_px = SCR_H_PX;
        }
      #else                             // ST7789 (T-Display) and ILI9341_Display
        if (scr_orient == 0) {          // portrait
          display.setRotation(0);       // or 4            
          scr_h_px = SCR_H_PX;
          scr_w_px = SCR_W_PX;
        } else
        if (scr_orient == 1) {          // landscape
          display.setRotation(3);       // or 1         
          scr_h_px = SCR_W_PX;
          scr_w_px = SCR_H_PX;
        }
      #endif
        
      display.setTextSize(txtsz);   
      
      if (txtsz == 1) {
        char_w_px = 6;    
      } else 
      if(txtsz == 2) {       
        char_w_px = 12;   
      } else       
      if  (txtsz == 3) { 
        char_w_px = 18;   
      } else         
      if  (txtsz == 4) {
        char_w_px = 24;    
      } else           
      if  (txtsz == 5)  {
        char_w_px = 30;    
      }     
   
      char_h_px = (uint8_t)(char_w_px * 1.4);   // vertical spacing

      scr_w_ch = scr_w_px / char_w_px;
      scr_h_ch = scr_h_px / char_h_px;       
    }
    #endif
    //===================================
    void PaintLogScreen(uint8_t new_row, last_row_t last_row_action) { 
      if (display_mode != logg) { 
          SetupLogDisplayStyle();
          display_mode = logg; 
      }  
     
        #if (defined ST7789_Display) || (defined SSD1331_Display) ||  (defined ILI9341_Display)   
        //  hardware SPI pins defined in config.h 
          display.fillScreen(SCR_BACKGROUND);                 
        #elif (defined SSD1306_Display) 
          display.clearDisplay();
        #endif  
        display.setCursor(0,0);  
        int8_t first_row;
        int8_t last_row;
        if (row < scr_h_ch) {
          first_row = (last_row_action==omit_last_row) ? 1 : 0; 
          last_row = (last_row_action==omit_last_row) ? new_row : new_row ;           
        } else {
          first_row = (last_row_action==omit_last_row) ? (new_row - scr_h_ch +1) : (new_row - scr_h_ch); 
          last_row = (last_row_action==omit_last_row) ? new_row : (new_row );            
        }     
        for (int i = first_row ; i < last_row; i++) { // drop first line, display rest of old lines & leave space for new line          
          display.println(ScreenRow[i].x);
        }
   
        #if (defined SSD1306_Display)
          display.display();
        #endif 
    }
    
  #endif  // end of defined displaySupport
    
    //===================================
    void LogScreenPrintln(String S) {
    #if defined displaySupport   
      if (display_mode != logg) {
          SetupLogDisplayStyle();
          display_mode = logg; 
          PaintLogScreen(row, omit_last_row);
      } else {   
        if (row >= scr_h_ch) {                      // if the new line exceeds the page lth, re-display existing lines
          PaintLogScreen(row, omit_last_row);
        }
      }
      uint16_t lth = strlen(S.c_str());           // store the new line a char at a time
      if (lth > scr_w_ch) {    
        log.printf("Display width of %d exceeded for |%s|\n", scr_w_ch, S.c_str()); // scr_w_ch = max_col-1            
        lth = scr_w_ch-1;  // prevent array overflow
      }

      for (int i=0 ; i < lth ; i++ ) {
        ScreenRow[row].x[col] = S[i];
        col++;
      } 

      for (col=col ; col  < scr_w_ch; col++) {    //  padd out the new line to eol
        ScreenRow[row].x[col] = '\0';
      } 

      display.println(ScreenRow[row].x);        // display the new line, which is always the last line
      #if (defined SSD1306_Display)
        display.display();
      #endif  

      col = 0;
      row++;
      if (row > max_row-1) {
        log.println("Display rows exceeded!");
        row = max_row-1;  // prevent array overflow
      }
      last_log_millis = millis();             // and enable toggle button again
      show_log = true;  
          
    #endif       
    } // ready for next line

    //===================================
   
    void LogScreenPrint(String S) {
    #if defined displaySupport   
    
      if (display_mode != logg) {
          SetupLogDisplayStyle();
          display_mode = logg; 
          PaintLogScreen(row, omit_last_row);
      } else {   
        if (row >= scr_h_ch) {                      // if the new line exceeds the page lth, re-display existing lines
          PaintLogScreen(row, omit_last_row);
        }
      }
      display.print(S);                         // the new line
      #if (defined SSD1306_Display)
        display.display();
      #endif 
       
      uint8_t lth = strlen(S.c_str());          // store the line a char at a time
      if (lth > scr_w_ch) {
        log.printf("Display width of %d exceeded for |%s|\n", scr_w_ch, S.c_str()); // scr_w_ch = max_col-1             
        lth = scr_w_ch-1;  // prevent array overflow
      }  

      for (int i=0 ; i < lth ; i++ ) {
        ScreenRow[row].x[col] = S[i];
        col++;
      } 
      for (col=col ; col < scr_w_ch; col++) {  //  padd out to eol
        ScreenRow[row].x[col] = '\0';
      }
      
      if (col > scr_w_ch-1) {   // only if columns exceeded, increment row
        col = 0;
        row++;
      }
      last_log_millis = millis();             // and enable toggle button again
      show_log = true;
    #endif    
    } // ready for next line
    
    //===================================
    void LogScreenPrintChar(char ch) {
    #if defined displaySupport   

      if (display_mode != logg) {
          SetupLogDisplayStyle();
          display_mode = logg; 
          PaintLogScreen(row, omit_last_row);
      } else {   
        if (row >= scr_h_ch) {                      // if the new line exceeds the page lth, re-display existing lines
          PaintLogScreen(row, omit_last_row);
        }
      }
      // display.setCursor( ((col+1) * char_w_px), ((row+1) * char_h_px) );
      display.print(ch);                         // the new char
      #if (defined SSD1306_Display)
        display.display();
      #endif 

      ScreenRow[row].x[col] = ch;
      col++;
          
      if (col > scr_w_ch-1) {   // if columns exceeded, increment row
        col = 0;
        row++;
      }
      last_log_millis = millis();             // and enable toggle button again
      show_log = true;
    #endif    
    } // ready for next char
    
    //===================================
    
    #if defined displaySupport  
    
    void DisplayFlightInfo() {
      uint16_t xx, yy; 
      if (display_mode != flight_info) {
          SetupInfoDisplayStyle();
          display_mode = flight_info; 
      }
      

      #if  (defined ILI9341_Display)

        if (millis() - info_millis > 200) {    // refresh rate
          info_millis = millis();  

          // artificial horizon
          draw_horizon(hud_roll, hud_pitch, scr_w_px, scr_h_px);
          
          SetScreenSizeOrient(2, SCR_ORIENT);   // text size, screen orientation, 26 ch wide x 15 ch deep
          
          // sats visible
          xx = 0;
          yy = 1 * char_h_px;;          
          display.setCursor(xx, yy);  
          snprintf(snprintf_buf, snp_max, "Sats:%d", hud_num_sats); 
          display.fillRect(xx +(5*char_w_px), yy, 2 * char_w_px, char_h_px, ILI9341_BLUE); // clear the previous line               
          display.println(snprintf_buf);  

          // heading (yaw)
          xx = 9 * char_w_px;
          yy = 1 * char_h_px;;          
          display.setCursor(xx, yy);  
          snprintf(snprintf_buf, snp_max, "Hdg:%.0f%", cur.hdg);
          display.fillRect(xx+(4*char_w_px), yy, 4 * char_w_px, char_h_px, ILI9341_BLUE); // clear the previous line                                
          display.println(snprintf_buf);

          // Radio RSSI
          xx = 17 * char_w_px;
          yy = 1 * char_h_px; ;          
          display.setCursor(xx, yy);  
          snprintf(snprintf_buf, snp_max, "RSSI:%d%%", hud_rssi); 
          display.fillRect(xx+(5*char_w_px), yy, 4 * char_w_px, char_h_px, ILI9341_BLUE); // clear the previous line               
          display.println(snprintf_buf);

          // Motors Armed
          xx = 0;
          yy = 3 * char_h_px;
          display.setCursor(xx, yy);         
          display.println("Arm:");
          if (motArmed) {
            display.fillRect(xx+(4*char_w_px), yy, char_w_px, char_h_px, ILI9341_GREEN);                        
          } else {
            display.fillRect(xx+(4*char_w_px), yy, char_w_px, char_h_px, ILI9341_RED);             
          }
          // Ground Speed
          xx = 7 * char_w_px;
          yy = 3 * char_h_px;                 
          display.setCursor(xx, yy);            
          snprintf(snprintf_buf, snp_max, "Spd %2.1f", hud_grd_spd); 
          display.fillRect(xx+(4*char_w_px), yy, (5 * char_w_px), char_h_px, ILI9341_BLUE);    // blank speed 
          display.println(snprintf_buf);  

          //  Climb m/s         
          xx = 16 * char_w_px;
          yy = 3 * char_h_px;                 
          display.setCursor(xx, yy);            
          snprintf(snprintf_buf, snp_max, "Climb %2.1f", hud_climb); 
          display.fillRect(xx+(6*char_w_px), yy, (5 * char_w_px), char_h_px, ILI9341_BLUE);    // blank climb 
          display.println(snprintf_buf);   
           
          // distance from home
          xx = 0;
          yy = 10 * char_h_px;        
          display.setCursor(xx, yy); 
          snprintf(snprintf_buf, snp_max, "Dist:%dm", pt_home_dist);    // m 
          display.fillRect(xx+(5*char_w_px), yy, (5*char_w_px), char_h_px, ILI9341_BLUE); // clear the previous line   
          display.println(snprintf_buf); 

          // arrow from home to uav, or uav to home
          xx = 14 * char_w_px;
          yy = 10 * char_h_px;   
          draw_hud_arrow(xx, yy, pt_home_angle, scr_w_px, scr_h_px);
   
          // altitude above home
          xx = 17 * char_w_px;
          yy = 10 * char_h_px;        
          display.setCursor(xx, yy); 
          snprintf(snprintf_buf, snp_max, "Alt:%3.0f", cur.alt_ag);    // m 
          display.fillRect(xx+(4*char_w_px), yy, (4*char_w_px), char_h_px, ILI9341_BLUE); // clear the previous line   
          display.println(snprintf_buf); 

          // voltage
          xx = 0;
          yy = 16 * char_w_px;        
          display.setCursor(xx, yy); 
          snprintf(snprintf_buf, snp_max, "V:%.1fV", hud_bat1_volts * 0.1F);     
          display.fillRect(xx+(2*char_w_px), yy, (6*char_w_px), char_h_px, ILI9341_BLUE); // clear the previous line   
          display.println(snprintf_buf); 
          
          // current
          xx = 9 * char_w_px;
          yy = 16 * char_w_px;        
          display.setCursor(xx, yy); 
          snprintf(snprintf_buf, snp_max, "A:%.0f", hud_bat1_amps * 0.1F);     
          display.fillRect(xx+(2*char_w_px), yy, (6*char_w_px), char_h_px, ILI9341_BLUE); // clear the previous line   
          display.println(snprintf_buf); 
          
          // Ah consumed
          xx = 18 * char_w_px;
          yy = 16 * char_w_px;        
          display.setCursor(xx, yy); 
          snprintf(snprintf_buf, snp_max, "Ah:%.1f", hud_bat1_mAh * 0.001F);     
          display.fillRect(xx+(3*char_w_px), yy, (5*char_w_px), char_h_px, ILI9341_BLUE); // clear the previous line   
          display.println(snprintf_buf);           
          
          // latitude and logitude
          SetScreenSizeOrient(1, SCR_ORIENT);   // text size, screen orientation             
          xx = 0;
          yy = 28 * char_h_px;                  // chars now small (6px wide, 6px high, screen 30ch h x 53 ch w)
          display.setCursor(xx,yy);       
          snprintf(snprintf_buf, snp_max, "Lat:%.7f", cur.lat);
          display.fillRect(xx, yy, (15*char_w_px), char_h_px, ILI9341_BLUE); // clear the previous line        
          display.println(snprintf_buf);  
          xx = 35 * char_w_px;   
          yy = 28 * char_h_px;  
          display.setCursor(xx, yy);    
          snprintf(snprintf_buf, snp_max, "Lon:%.7f", cur.lon);
          display.fillRect(xx, yy, 21 * char_w_px, char_h_px, ILI9341_BLUE); // clear the previous line            
          display.println(snprintf_buf);  
          SetScreenSizeOrient(2, SCR_ORIENT);   // text size, screen orientation, 26 ch wide x 15 ch deep
          display_mode = flight_info;
        }
        
      #else  // other display types
      
        if (millis() - info_millis > 2000) {    // refresh rate
          info_millis = millis();  

           // Number of Sats and RSSI
          xx = 0;
          yy = 0 * char_h_px;      
          display.setCursor(xx, yy);            
          snprintf(snprintf_buf, snp_max, "Sats %d RSSI %ld%%", hud_num_sats, hud_rssi); 
          display.fillRect(xx+(5*char_w_px), yy, (3 * char_w_px), char_h_px, SCR_BACKGROUND);  
          display.fillRect(xx+(12*char_w_px), yy, (4 * char_w_px), char_h_px, SCR_BACKGROUND);   // blank rssi  
          display.println(snprintf_buf);         
          #if (defined SSD1306_Display)
            display.display();
          #endif   
                 
          // Altitude 
          xx = 0;
          #if (defined SSD1306_Display) 
            yy = 1.2 * char_h_px;  
          #elif (defined SSD1331_Display)
            yy = 1.1 * char_h_px;            
          #elif (defined ST7789_Display)   
            yy = 1.8 * char_h_px;  
          #endif  
          display.setCursor(xx, yy);            
          snprintf(snprintf_buf, snp_max, "Alt %.0f", (cur.alt_ag));
          display.fillRect(xx+(4*char_w_px), yy, (4 * char_w_px), char_h_px, SCR_BACKGROUND);  // blank Alt
          display.println(snprintf_buf);  
          
          // Heading
          xx = 9 * char_w_px;
          #if (defined SSD1306_Display) 
            yy = 1.2 * char_h_px;  
          #elif (defined SSD1331_Display)
            yy = 1.1 * char_h_px;            
          #elif (defined ST7789_Display)   
            yy = 1.8 * char_h_px;  
          #endif  
          display.setCursor(xx, yy);            
          snprintf(snprintf_buf, snp_max, "Hdg %.0f", cur.hdg); 
          display.fillRect(xx+(4*char_w_px), yy, (5 * char_w_px), char_h_px, SCR_BACKGROUND); // blank Hdg
          display.println(snprintf_buf); 
                    
          // Ground Speed m/s
          xx = 0;
          #if (defined SSD1306_Display) 
            yy = 2.4 * char_h_px;  
          #elif (defined SSD1331_Display)
            yy = 2.2 * char_h_px;               
          #elif (defined ST7789_Display)   
            yy = 3.6 * char_h_px;  
          #endif               
          display.setCursor(xx, yy);            
          snprintf(snprintf_buf, snp_max, "Spd %2.1f", hud_grd_spd); 
          display.fillRect(xx+(4*char_w_px), yy, (4 * char_w_px), char_h_px, SCR_BACKGROUND);    // blank speed 
          display.println(snprintf_buf);  
                 
          // Climb - vert speed m/s
          xx = 9 * char_w_px;
          #if (defined SSD1306_Display) 
            yy = 2.4 * char_h_px;  
          #elif (defined SSD1331_Display)
            yy = 2.2 * char_h_px;               
          #elif (defined ST7789_Display)   
            yy = 3.6 * char_h_px;  
          #endif               
          display.setCursor(xx, yy);            
          snprintf(snprintf_buf, snp_max, "Clm %2.1f", hud_climb); 
          display.fillRect(xx+(4*char_w_px), yy, (4 * char_w_px), char_h_px, SCR_BACKGROUND);   // blank climb 
          display.println(snprintf_buf);   
                    
          // Volts, Amps and Ah 
          xx = 0;
          #if (defined SSD1306_Display) 
            yy = 3.6 * char_h_px;  
          #elif (defined SSD1331_Display)
            yy = 3.3 * char_h_px;                
          #elif (defined ST7789_Display)   
            yy = 5.4 * char_h_px;  
          #endif            
          display.setCursor(xx, yy);               
          snprintf(snprintf_buf, snp_max, "%2.1fV %2.0fA %2.1fAh", hud_bat1_volts * 0.1F, hud_bat1_amps * 0.1F, hud_bat1_mAh * 0.001F);     
          display.fillRect(xx, yy, scr_w_px, char_h_px, SCR_BACKGROUND); // clear the whole line  
          display.println(snprintf_buf); 

          // Latitude and Longitude
          xx = 0;
          
          #if (defined SSD1306_Display) 
            yy = 4.8 * char_h_px; 
            display.setCursor(xx,yy);       
            snprintf(snprintf_buf, snp_max, "Lat %3.7f", cur.lat);               
            display.fillRect(xx+(4*char_w_px), yy, 12 * char_w_px, char_h_px, SCR_BACKGROUND); // clear lat
            display.println(snprintf_buf);
            yy = 6.0 * char_h_px; 
            snprintf(snprintf_buf, snp_max, "Lon %3.7f", cur.lon);  
            display.fillRect(xx+(4*char_w_px), yy, 12 * char_w_px, char_h_px, SCR_BACKGROUND); // clear lon
            display.println(snprintf_buf);  
            display.display();  // NB dont forget me, SSD1306 only
            
          #elif (defined SSD1331_Display)       
            yy = 4.4 * char_h_px;
            display.setCursor(xx,yy);       
            snprintf(snprintf_buf, snp_max, "Lat %3.7f", cur.lat);               
            display.fillRect(xx+(4*char_w_px), yy, 12 * char_w_px, char_h_px, SCR_BACKGROUND); // clear lat
            display.println(snprintf_buf);
            yy = 5.5 * char_h_px; 
            snprintf(snprintf_buf, snp_max, "Lon %3.7f", cur.lon);  
            display.fillRect(xx+(4*char_w_px), yy, 12 * char_w_px, char_h_px, SCR_BACKGROUND); // clear lon
            display.println(snprintf_buf); 
                                                                  
          #elif (defined ST7789_Display)         
            yy = 7.2 * char_h_px;  
            display.setCursor(xx,yy);       
            snprintf(snprintf_buf, snp_max, "Lat %3.7f Lon %3.7f", cur.lat, cur.lon);        
            SetScreenSizeOrient(TEXT_SIZE -1, SCR_ORIENT);  // small text size       
            display.fillRect(xx+(4*char_w_px), yy, 12 * char_w_px, char_h_px, SCR_BACKGROUND); // clear the previous data 
            display.fillRect(xx+(16*char_w_px), yy, 12 * char_w_px, char_h_px, SCR_BACKGROUND); 
            display.println(snprintf_buf);  
            SetScreenSizeOrient(TEXT_SIZE, SCR_ORIENT);  // restore text size           
          #endif             
        }
      #endif    
    } 
    #endif    
    
    //===================================
    #if defined displaySupport  
    
    void SetupLogDisplayStyle() {

      SetScreenSizeOrient(TEXT_SIZE, SCR_ORIENT);
       
      #if (defined ST7789_Display)      // LILYGO® TTGO T-Display ESP32 1.14" ST7789 Colour LCD
        #if (SCR_ORIENT == 0)           // portrait
          display.setRotation(0);       // or 4 
          display.setTextFont(0);       // Original Adafruit font 0, try 0 thru 6 
        #elif (SCR_ORIENT == 1)         // landscape
          display.setRotation(3);       // or 1 
          display.setTextFont(1);        
        #endif   
         
        display.fillScreen(SCR_BACKGROUND);
        display.setTextColor(TFT_SKYBLUE);    
            
        //display.setTextColor(TFT_WHITE);
        //display.setTextColor(TFT_BLUE);  
        //display.setTextColor(TFT_GREEN, TFT_BLACK);
    
      #elif (defined SSD1306_Display)            // all  boards with SSD1306 OLED display
        display.clearDisplay(); 
        display.setTextColor(WHITE);     
             
      #elif (defined SSD1331_Display)            // T2 board with SSD1331 colour TFT display
        //  software SPI pins defined in config.h 
        display.fillScreen(SCR_BACKGROUND);
        display.setCursor(0,0);
        
      #elif (defined ILI9341_Display)           // ILI9341 2.8" COLOUR TFT SPI 240x320 V1.2  
        //  hardware SPI pins defined in config.h 
        display.fillScreen(ILI9341_BLUE);    
        display.setCursor(0,0);
       //#if (SCR_ORIENT == 0)              // portrait
       //   display.setRotation(2);          // portrait pins at the top rotation      
       // #elif (SCR_ORIENT == 1)            // landscape
       //   display.setRotation(3);          // landscape pins on the left    
        //#endif 
        #define SCR_BACKGROUND ILI9341_BLUE 
      #endif

    }
   #endif
   //===================================
   #if defined displaySupport  
    
    void SetupInfoDisplayStyle() {

      SetScreenSizeOrient(TEXT_SIZE, SCR_ORIENT);

      #if (defined ST7789_Display)      // LILYGO® TTGO T-Display ESP32 1.14" ST7789 Colour LCD
        #if (SCR_ORIENT == 0)           // portrait
          SetScreenSizeOrient(1, 0);    // change text size
          display.setTextSize(1);       // and font
          display.setTextFont(0);       // Original Adafruit font 0, try 0 thru 6 
        #elif (SCR_ORIENT == 1)         // landscape
          SetScreenSizeOrient(2, 3);    // change text size
          display.setTextFont(1);       // and font 
        #endif    

        display.fillScreen(SCR_BACKGROUND);
        display.setTextColor(TFT_SKYBLUE);    
            
        //display.setTextColor(TFT_WHITE);
        //display.setTextColor(TFT_BLUE);  
        //display.setTextColor(TFT_GREEN, TFT_BLACK);
    
      #elif (defined SSD1306_Display)            // all  boards with SSD1306 OLED display    
        display.clearDisplay(); 
        display.setTextColor(WHITE);  

      #elif (defined SSD1331_Display)            // T2 board with SSD1331 colour TFT display     
        //  SPI pins defined in config.h 
        display.fillScreen(BLACK);
        display.setTextColor(WHITE);  

      
      #elif (defined ILI9341_Display)            // ILI9341 2.8" COLOUR TFT SPI 240x320 V1.2          
        //  SPI pins defined in config.h 
        display.fillScreen(SCR_BACKGROUND);
        display.setRotation(3);          // landscape pins on the left   
        display.setCursor(0,0);
      #endif
     }
    #endif        
    //=================================================================== 
    #if (defined displaySupport) && (defined ILI9341_Display)
    uint32_t draw_horizon(float roll, float pitch, int16_t width, int16_t height) {
      int16_t x0, y0, x1, y1, xc, yc, ycp, lth, tick_lean;
      static int16_t px0, py0, px1, py1, pycp, ptick_lean; 
      uint8_t tick_height = 5;  
      float roll_slope = 0;      // [-180,180]
      float pitch_offset = 0;     //  [-90,90]
      const float AngleToRad = PI / 180;
      
      xc = width / 2;    // centre point / pivot / origin
      yc = height / 2;   // 

      display.drawLine(0 + (width * 0.2), yc, width - (width * 0.2), yc, ILI9341_WHITE);   // static reference horizon
      display.drawLine(xc, yc+10, xc, yc-10, ILI9341_WHITE);
      
      roll_slope = tan(roll * AngleToRad);             // roll slope 
      pitch_offset = (sin(pitch * AngleToRad)) * yc;   // pitch offset  
      tick_lean = roll_slope * tick_height;

      lth = (xc * 0.8) * cos(roll * AngleToRad);
      x0 = (xc - lth);
      x1 = (xc + lth);

      y0 = yc - ((xc - x0) * roll_slope);
      y1 = yc + ((x1 - xc) * roll_slope);   
  
      y0 += pitch_offset;
      y1 += pitch_offset;
      ycp = yc + pitch_offset;
     
      static bool ft = true;
      if (!ft) {
        display.drawLine(px0, py0, px1, py1, ILI9341_BLUE);   // Erase old horizon line 
        display.drawLine(xc-ptick_lean, pycp+tick_height, xc+ptick_lean, pycp-tick_height, ILI9341_BLUE); 
        display.drawLine(xc-ptick_lean+1, pycp+tick_height, xc+ptick_lean+1, pycp-tick_height, ILI9341_BLUE);
        display.drawLine(px0-ptick_lean+1, py0+tick_height, px0+ptick_lean+1, py0-tick_height, ILI9341_BLUE);
        display.drawLine(px1-ptick_lean+1, py1+tick_height, px1+ptick_lean+1, py1-tick_height, ILI9341_BLUE);        
        display.drawLine(px0+1, py0, px1+1, py1,ILI9341_BLUE);
      }  
      ft = false;
      display.drawLine(x0, y0, x1, y1, ILI9341_WHITE);      // Horizon line over the top
      display.drawLine(xc-tick_lean, ycp+tick_height, xc+tick_lean, ycp-tick_height, ILI9341_WHITE);
      display.drawLine(xc-tick_lean+1, ycp+tick_height, xc+tick_lean+1, ycp-tick_height, ILI9341_WHITE);
      display.drawLine(x0-tick_lean+1, y0+tick_height, x0+tick_lean+1, y0-tick_height, ILI9341_WHITE);
      display.drawLine(x1-tick_lean+1, y1+tick_height, x1+tick_lean+1, y1-tick_height, ILI9341_WHITE); 
      display.drawLine(x0+1, y0, x1+1, y1, ILI9341_WHITE);
      px0 = x0;
      py0 = y0;
      px1 = x1;
      py1 = y1;    
      pycp = ycp;
      ptick_lean = tick_lean;
    
      return micros();   
    }
    #endif
    //=================================================================== 
    #if  (defined ILI9341_Display)
    void draw_hud_arrow(int16_t x, int16_t y, int16_t home_angle, int16_t width, int16_t height) {

      int16_t x0, y0, x1, y1, x2, y2, x3, y3;
      static int16_t px0, py0, px1, py1, px2, py2, px3, py3;
      int16_t opp, adj, hyp, opp90, adj90, hyp90;
      const int16_t al = 40;  // arrow length  
      static bool ft = true;
      const float AngleToRad = PI / 180;
      int16_t arr_angle = 270 - home_angle; // change direction of rotation and correct angle
      
      if (HUD_ARROW_OFFSET == 999) {
        //hom.hdg = 360.0 - 90.0;  // fix hom.hdg to west for testing
        arr_angle = arr_angle - hom.hdg + 180;   // hdg is from uav to home, so home to uav +180          
      } else {
        arr_angle += HUD_ARROW_OFFSET;
      }
         
      arr_angle = (arr_angle < 0) ? arr_angle + 360 : arr_angle;
      arr_angle = (arr_angle > 360) ? arr_angle - 360 : arr_angle;   
       
      hyp = al / 2;        
      opp = hyp * sin(arr_angle * AngleToRad);
      adj = hyp * cos(arr_angle * AngleToRad);
      
      hyp90 = al / 5; 
      opp90 = hyp90 * sin((arr_angle + 90) * AngleToRad);
      adj90 = hyp90 * cos((arr_angle + 90) * AngleToRad); 
      
      x0 = x + adj; 
      y0 = y - opp; 
      
      x1 = x - adj;
      y1 = y + opp;
 
      x2 = x1 + adj90;
      y2 = y1 - opp90;

      x3 = x1 - adj90;
      y3 = y1 + opp90;
           
      if (!ft) {
       //display.drawTriangle(px0, py0, px2, py2, px3, py3, ILI9341_BLUE);   
       display.fillTriangle(px0, py0, px2, py2, px3, py3, ILI9341_BLUE);                 
      }
      ft = false;     

      //display.drawTriangle(x0, y0, x2, y2, x3, y3, ILI9341_RED); 
      display.fillTriangle(x0, y0, x2, y2, x3, y3, ILI9341_RED);         
             
      px0 = x0; py0 = y0; px1 = x1; py1 = y1; px2 = x2; py2 = y2; px3 = x3; py3 = y3;

    }
    #endif

    //===================================================================  
    bool homeButtonPushed() {
      bool hbp = false;
      if (SetHomePin != -1) {
        hbp = (!(digitalRead(SetHomePin)));            // Check if home button is pushed
      }
      #if (defined ESP32)
          //hbp = !hbp;     
      #endif  
      return hbp;
    }
    //===================================================================   

   uint16_t wrap360(int16_t ang) {
     if (ang < 0) ang += 360;
     if (ang > 359) ang -= 360;
     return ang;
   }
   
  //================================================================================================= 
  //                              C O M P A S S    A L I G N M E N T
  //=================================================================================================
  // params: 
  // float hdg -> angle value in degrees
  // uint8_t rotation -> rotation enum definition value, fe CW180_DEG
  float applyCompassAlignment(float hdg , uint8_t rotation) {
    float res = 0.0;
    
    switch (rotation) {
      default:
      case CW0_DEG:
      case ALIGN_DEFAULT:
      case CW180_DEG_FLIP:
          res = (360-hdg)+180;
          break;
      case CW90_DEG:
          res = hdg + 90;
          break;
      case CW180_DEG:
          res = hdg + 180;
          break;
      case CW270_DEG:
          res = hdg + 270;
          break;
      case CW0_DEG_FLIP:
          res = (360-hdg);
          break;
      case CW90_DEG_FLIP:
          res = (360-hdg) + 90;
          break;
      case CW270_DEG_FLIP:
          res = (360-hdg) + 270;
          break;
    }
    
    if (res > 360.0) {
      res = res - 360.0;
    }

    return  res;
  }
 
