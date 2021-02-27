//=================================================================================================  
//================================================================================================= 
//
//                                       U T I L I T I E S
//
//================================================================================================= 
//=================================================================================================
 
//=================================================================================================   
//                           D I S P L A Y   S U P P O R T   -   ESP Only - for now
//================================================================================================= 
    #if defined Display_Support  

    void IRAM_ATTR gotButtonUp(){
      upButton = true;
    }

    void IRAM_ATTR gotButtonDn(){
      dnButton = true;  
    }

    //===================================
    void Scroll_Display(scroll_t up_dn) {
      
      if (millis() - scroll_millis < 300) return;
      scroll_millis = millis(); 
      
      if (up_dn == up) {
         scroll_row--;
         scroll_row = constrain(scroll_row, screen_height, row);
         upButton = false; 
         PaintDisplay(scroll_row, show_last_row);   // paint down to scroll_row
      }
      if (up_dn == down) {
          scroll_row++; 
          scroll_row = constrain(scroll_row, screen_height, row);       
          dnButton = false; 
          PaintDisplay(scroll_row, show_last_row);  // paint down to scroll_row      
      }   
    }
    
    //===================================
    void PaintDisplay(uint8_t new_row, last_row_t last_row_action) {   

        #if (defined ST7789_Display)
          display.fillScreen(screenBackground);
        #elif (defined SSD1306_Display)
          display.clearDisplay();
        #endif  
        display.setCursor(0,0);  
        int8_t first_row = (last_row_action==omit_last_row) ? (new_row - screen_height +1) : (new_row - screen_height); 
        int8_t last_row = (last_row_action==omit_last_row) ? new_row : (new_row );        
        for (int i = first_row ; i < last_row; i++) { // drop first line, display rest of old lines & leave space for new line          
          display.println(ScreenRow[i].x);
        }
   
        #if (defined SSD1306_Display)
          display.display();
        #endif  
    }
    #endif    
    //===================================
    void DisplayPrintln(String S) {
    #if defined Display_Support        

      if (row >= screen_height) {                 // if the new line exceeds the page lth, re-display existing lines
        PaintDisplay(row, omit_last_row);
      }
      
      uint8_t lth = strlen(S.c_str());           // store the new line a char at a time
      if (lth > max_col-1) {
        Debug.printf("Display width of %d exceeded for |%s|\n", screen_width, S.c_str());  // screen_width = max_col-1
        lth = max_col-1;  // prevent array overflow
      }

      for (int i=0 ; i < lth ; i++ ) {
        ScreenRow[row].x[col] = S[i];
        col++;
      } 

      for (col=col ; col < max_col; col++) {    //  padd out the new line to eol
        ScreenRow[row].x[col] = '\0';
      } 

      display.println(ScreenRow[row].x);        // display the new line, which is always the last line
      #if (defined SSD1306_Display)
        display.display();
      #endif  

      col = 0;
      row++;
      if (row > max_row-1) {
        Debug.println("Display rows exceeded!");
        row = max_row-1;  // prevent array overflow
      }

    #endif       
    } // ready for next line

    //===================================
    void DisplayPrint(String S) {
      #if defined Display_Support  
     // scroll_row = row; 
      if (row >= screen_height) {              // if the new line exceeds the page lth, re-display existing lines
        PaintDisplay(row, omit_last_row);
      }
      display.print(S);                         // the new line
      #if (defined SSD1306_Display)
        display.display();
      #endif 
       
      uint8_t lth = strlen(S.c_str());          // store the line a char at a time
      if (lth > screen_width) {
        Debug.printf("Display width of %d exceeded for |%s|\n", screen_width, S.c_str());  // screen_width = max_col-1
        lth = max_col-1;  // prevent array overflow
      }  

      for (int i=0 ; i < lth ; i++ ) {
        ScreenRow[row].x[col] = S[i];
        col++;
      } 
      for (col=col ; col < max_col; col++) {  //  padd out to eol
        ScreenRow[row].x[col] = '\0';
      }
      
      if (col > max_col-1) {   // only if columns exceeded, increment row
        col = 0;
        row++;
       }
    #endif    
    } // ready for next line

 /*
    try these
    display.drawCentreString("Font size 4", 120, 30, 4); // Draw text centre at position 120, 30 using font 4
    display.drawString(" is pi", xpos, ypos, font); 

 */

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
    Debug.print("mav1: /");

    if (j == 0) {
      PrintByteLF(bytes[0], 0);   // CRC_Out1
      PrintByteLF(bytes[1], 0);   // CRC_Out2
      Debug.print("/");
      }
    mav_magic = bytes[j+2];   
    mav_len = bytes[j+3];
 //   mav_incompat_flags = bytes[j+4];;
 //   mav_compat_flags = bytes[j+5];;
    mav_seq = bytes[j+6];
 //   mav_sysid = bytes[j+7];
 //   mav_compid = bytes[j+8];
    mav_msgid = bytes[j+9];

    //Debug.print(TimeString(millis()/1000)); Debug.print(": ");
  
    Debug.print("seq="); Debug.print(mav_seq); Debug.print("\t"); 
    Debug.print("len="); Debug.print(mav_len); Debug.print("\t"); 
    Debug.print("/");
    for (int i = (j+2); i < (j+10); i++) {  // Print the header
      PrintByteLF(bytes[i], 0); 
    }
    
    Debug.print("  ");
    Debug.print("#");
    Debug.print(mav_msgid);
    if (mav_msgid < 100) Debug.print(" ");
    if (mav_msgid < 10)  Debug.print(" ");
    Debug.print("\t");
    
    tl = (mav_len+10);                // Total length: 8 bytes header + Payload + 2 bytes CRC_Out
 //   for (int i = (j+10); i < (j+tl); i++) {  
    for (int i = (j+10); i <= (tl); i++) {    
     PrintByteLF(bytes[i], 0);     
    }
    if (j == -2) {
      Debug.print("//");
      PrintByteLF(bytes[mav_len + 8], 0); 
      PrintByteLF(bytes[mav_len + 9], 0); 
      }
    Debug.println("//");  
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
    
    Debug.print("mav2:  /");
    if (j == 0) {
      PrintByteLF(bytes[0], 0);   // CRC_Out1
      PrintByteLF(bytes[1], 0);   // CRC_Out2 
      Debug.print("/");
    }
    mav_magic = bytes[2]; 
    mav_len = bytes[3];
//    mav_incompat_flags = bytes[4]; 
  //  mav_compat_flags = bytes[5];
    mav_seq = bytes[6];
//    mav_sysid = bytes[7];
   // mav_compid = bytes[8]; 
    mav_msgid = (bytes[11] << 16) | (bytes[10] << 8) | bytes[9]; 

    //Debug.print(TimeString(millis()/1000)); Debug.print(": ");

    Debug.print("seq="); Debug.print(mav_seq); Debug.print("\t"); 
    Debug.print("len="); Debug.print(mav_len); Debug.print("\t"); 
    Debug.print("/");
    for (int i = (j+2); i < (j+12); i++) {  // Print the header
     PrintByteLF(bytes[i], 0); 
    }

    Debug.print("  ");
    Debug.print("#");
    Debug.print(mav_msgid);
    if (mav_msgid < 100) Debug.print(" ");
    if (mav_msgid < 10)  Debug.print(" ");
    Debug.print("\t");

 //   tl = (mav_len+27);                // Total length: 10 bytes header + Payload + 2B CRC_Out +15 bytes signature
    tl = (mav_len+22);                  // This works, the above does not!
    for (int i = (j+12); i < (tl+j); i++) {   
       if (i == (mav_len + 12)) {
        Debug.print("/");
      }
      if (i == (mav_len + 12 + 2+j)) {
        Debug.print("/");
      }
      PrintByteLF(bytes[i], 0); 
    }
    Debug.println();
  }

   Debug.print("Raw: ");
   for (int i = 0; i < 40; i++) {  //  unformatted
      PrintByteLF(bytes[i], 0); 
    }
   Debug.println();
  
}
//=================================================================================================  
void PrintByteLF(byte b, bool LF) {  // line feed
  if ((b == 0x7E) && (LF)) {
    Debug.println();
  }
  if (b<=0xf) Debug.print("0");
  Debug.print(b,HEX);
}

void PrintByte(byte b) {
  if (b<=0xf) Debug.print("0");
    Debug.print(b,HEX);
    Debug.print(" ");
}

//=================================================================================================  
void DisplayTheBuffer (int lth){
  for ( int i = 0; i < lth; i++ ) {
    PrintByte(packetBuffer[i]);
  }
  Debug.println();
}

//=================================================================================================  
bool PacketGood() {
// Allow 1 degree of lat and lon away from home, i.e. 60 nautical miles radius at the equator
// Allow 1km up and 300m down from home altitude
if (homeInitialised==0) {  //  You can't use the home co-ordinates for a reasonability test if you don't have them yet
  return true;
  }
if (cur.lat<(hom.lat-1.0) || cur.lat>(hom.lat+1.0)) {  // Also works for negative lat
  Debug.print(" Bad lat! cur.lat=");
  Debug.print(cur.lat,7);  
  Debug.print(" hom.lat=");Debug.print(hom.lat,7);
  Debug.println("  Packet ignored");   
  return false; 
  }
if (cur.lon<(hom.lon-1.0) || cur.lon>(hom.lon+1.0)) { // Also works for negative lon
  Debug.print(" Bad lon! cur.lon=");
  Debug.print(cur.lon,7);  
  Debug.print(" hom.lon=");Debug.print(hom.lon,7);
  Debug.println("  Packet ignored");  
  return false;  
  }
if (cur.alt<(hom.alt-300) || cur.alt>(hom.alt+1000)) {
  Debug.print(" Bad alt! cur.alt=");
  Debug.print(cur.alt,0);  
  Debug.print(" hom.alt=");Debug.print(hom.alt,0);
  Debug.println("  Packet ignored");    
  return false;  
  }
if ((cur.alt-hom.alt)<-300 || (cur.alt-hom.alt)>1000) {
  Debug.print(" Bad alt! cur.alt=");
  Debug.print(cur.alt,0);  
  Debug.print(" hom.alt=");Debug.print(hom.alt,0);
  Debug.println("  Packet ignored");    
  return false;  
  }
if (Heading_Source == 2) { //  Heading source from flight controller
  if (cur.hdg<0 || cur.hdg>360) {
    Debug.print(" Bad hdg! cur.hdg=");
    Debug.print(cur.hdg,0);  
    Debug.print(" hom.hdg=");Debug.print(hom.hdg,0);
    Debug.println("  Packet ignored");    
    return false;  
   }
}
  
return true;
}
//=================================================================================================  
void CheckForTimeouts() {
  uint32_t cMillis = millis();
    if ((gpsGood==1) && (cMillis - millisGPS >= 5000)){
      gpsGood = 0;   // If no GPS packet for 5 seconds then GPS timeout 
      hbGood = 0;
      DisplayPrintln("No GPS packts/timeout");     
      #if defined Debug_All || defined Debug_FrSky || defined Debug_NMEA_GPS || defined Debug_LTM
        Debug.println("No GPS telemetry for 5 seconds"); 
      #endif  
    }
   ServiceTheStatusLed();
}

//=================================================================================================  
uint32_t epochNow() {
return (epochSync + (millis() - millisSync) / 1E3);
}
//=================================================================================================  
void LostPowerCheckAndRestore(uint32_t epochSyn) {  // this function only ever called by a time enabled protocol
  if ((!timeGood) || (epochSyn == 0)) return;
  
  epochSync = epochSyn;
  millisSync = millis();
 
  if (lostPowerCheckDone) return;

  #if defined Debug_All || defined Debug_Time || defined Debug_Home
    Debug.print("Checking for RestoreHomeFromFlash conditions:"); 
    Debug.print("  epochHome="); Debug.print(TimeString(epochHome())); 
    Debug.print("  epochNow="); Debug.println(TimeString(epochNow()));
  #endif 

  
  if ((epochNow() -  epochHome()) < 300) {  //  within 5 minutes
    RestoreHomeFromFlash();
    homeInitialised=1;           
  }
  
  lostPowerCheckDone = true;
}
//=================================================================================================  
void SaveHomeToFlash() {

  EEPROMWritelong(0, epochNow());   // epochHome
  EEPROMWritelong(1, hom.lon*1E6);  // float to long
  EEPROMWritelong(2, hom.lat*1E6);
  EEPROMWritelong(3, hom.alt*10);
  EEPROMWritelong(4, hom.hdg*10);  
  
#if defined Debug_All || defined Debug_EEPROM || defined Debug_Time || defined Debug_Home
  Debug.print("  homSaved="); Debug.print(homSaved);
  Debug.print("  home.lon="); Debug.print(hom.lon, 6);
  Debug.print("  home.lat="); Debug.print(hom.lat, 6);
  Debug.print("  home.alt="); Debug.print(hom.alt, 1);
  Debug.print("  home.hdg="); Debug.println(hom.hdg, 1);

#endif   
}

//=================================================================================================  
void StoreEpochPeriodic() {
  uint32_t epochPeriodic = epochNow();
  EEPROMWritelong(5, epochPeriodic); // Seconds

  if (homeInitialised) {
    EEPROMWritelong(0, epochPeriodic); // UPDATE epochHome
    #if defined Debug_All || defined Debug_EEPROM || defined Debug_Time || defined Debug_Home
      Debug.print("epochHome stored="); Debug.println(TimeString(epochPeriodic));
    #endif  
  }
  
  #if defined Debug_All || defined Debug_EEPROM || defined Debug_Time || defined Debug_Home
  Debug.print("epochPeriodic stored="); Debug.println(TimeString(epochPeriodic));
  #endif  
}

//=================================================================================================  
uint32_t epochHome() {

uint32_t epHome = EEPROMReadlong(0);

 #if defined Debug_All || defined Debug_EEPROM
   Debug.print("epochHome="); Debug.println(TimeString(epHome));

 #endif
 return epHome;    
}
//=================================================================================================  
void RestoreHomeFromFlash() {

  hom.lon = EEPROMReadlong(1) / 1E6; //long back to float
  hom.lat = EEPROMReadlong(2) / 1E6;
  hom.alt = EEPROMReadlong(3) / 10;
  hom.hdg = EEPROMReadlong(4) / 10;
  
  Debug.println("Home data restored from Flash"); 
  DisplayPrintln("Home data restored");
  DisplayPrintln("from Flash. Go Fly!");
  
  #if defined Debug_All || defined Debug_EEPROM || defined Debug_Time || defined Debug_Home
    Debug.print("  home.lon="); Debug.print(hom.lon, 6);
    Debug.print("  home.lat="); Debug.print(hom.lat, 6);
    Debug.print("  home.alt="); Debug.print(hom.alt, 0);
    Debug.print("  home.hdg="); Debug.println(hom.hdg, 0);
  #endif  
}
//=================================================================================================  
  
#if (Telemetry_In == 2)     //  WiFi / Mavlink
 
  void SetupWiFi() { 

    bool apMode = false;  // used when STA fails to connect
     //===============================  S T A T I O N   =============================
     
    #if (WiFi_Mode == 2)  // STA
      uint8_t retry = 0;
      Debug.print("Trying to connect to ");  
      Debug.print(STAssid); 
      DisplayPrintln("WiFi trying ..");

      WiFi.disconnect(true);   // To circumvent "wifi: Set status to INIT" error bug
      delay(500);
      WiFi.mode(WIFI_STA);
      delay(500);
      
      WiFi.begin(STAssid, STApw);
      while (WiFi.status() != WL_CONNECTED){
        retry++;
        if (retry > 10) {
          Debug.println();
          Debug.println("Failed to connect in STA mode");
          DisplayPrintln("Failed in STA Mode");
          wifiSuDone = true;
          
          #ifdef AutoAP  
            apMode = true;            // Rather go establish an AP instead
            Debug.println("Starting AP instead.");
            DisplayPrintln("Starting AP instead");  
            //new from Target0815:
            Debug.println("WiFi-Reset ...");
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
        Debug.println();
        Debug.println("WiFi connected!");
        Debug.print("Local IP address: ");
        Debug.print(localIP);

        #if   (WiFi_Protocol == 1)   // TCP
          Debug.print("  port: ");
          Debug.println(TCP_localPort);    //  UDP port is printed lower down
        #else 
          Debug.println();
        #endif 
         
        wifi_rssi = WiFi.RSSI();
        Debug.print("WiFi RSSI:");
        Debug.print(wifi_rssi);
        Debug.println(" dBm");

        DisplayPrintln("Connected! My IP =");
        DisplayPrintln(localIP.toString());
        
        #if (WiFi_Protocol == 1)   // TCP                                                
          #if (Telemetry_In  == 2) // We are a client and need to connect to a server
             outbound_clientGood = NewOutboundTCPClient();
          #endif
        #endif

        #if (WiFi_Protocol == 2)  // UDP
          UDP.begin(UDP_localPort);
          Debug.printf("UDP started, listening on IP %s, UDP port %d\n", localIP.toString().c_str(), UDP_localPort);
          DisplayPrint("UDP port = ");  DisplayPrintln(String(UDP_localPort));
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
      Debug.print("AP IP address: ");
      Debug.print (localIP); 
      Debug.print("  SSID: ");
      Debug.println(String(APssid));
      DisplayPrintln("WiFi AP SSID =");
      DisplayPrintln(String(APssid));

      #if (WiFi_Protocol == 2)      // UDP
        UDP.begin(UDP_localPort);
        Debug.printf("UDP started, listening on IP %s, UDP port %d\n", WiFi.softAPIP().toString().c_str(), UDP_localPort);
        DisplayPrint("UDP port = ");  DisplayPrintln(String(UDP_localPort));

      #endif 
      
      wifiSuGood = true;
 
    }           

      
    #ifndef Start_WiFi  // if not button override
      delay(2000);      // debounce button press
    #endif  

    wifiSuDone = true;
 }   

  //=================================================================================================  
  #if (WiFi_Protocol == 1)    //  WiFi TCP    
  
  bool NewOutboundTCPClient() {
  static uint8_t retry = 3;

    WiFiClient newClient;        
    while (!newClient.connect(TCP_remoteIP, TCP_remotePort)) {
      Debug.printf("Local outgoing tcp client connect failed, retrying %d\n", retry);
      retry--;
      if (retry == 0) {
         Debug.println("Tcp client connect aborted!");
         return false;
      }
      nbdelay(4000);
    }
    active_client_idx = 0;     // use the first client object for  our single outgoing session   
    clients[0] = new WiFiClient(newClient); 
    Debug.print("Local tcp client connected to remote server IP:"); Debug.print(TCP_remoteIP);
    Debug.print(" remote Port:"); Debug.println(TCP_remotePort);
    nbdelay(1000);
    DisplayPrintln("Remote server IP =");
    snprintf(snprintf_buf, snp_max, "%d.%d.%d.%d", TCP_remoteIP[0], TCP_remoteIP[1], TCP_remoteIP[2], TCP_remoteIP[3] );        
    DisplayPrintln(snprintf_buf); 

    
 //   DisplayPrintln(TCP_remoteIP.toString()); 
    return true;
  }
  #endif
  //=================================================================================================  
   #if (Telemetry_In == 2) &&  (WiFi_Protocol == 2) //  WiFi && UDP - Print the remote UDP IP the first time we get it  
   void PrintRemoteIP() {
    if (FtRemIP)  {
      FtRemIP = false;
      Debug.print("Client connected: Remote UDP IP: "); Debug.print(UDP_remoteIP);
      Debug.print("  Remote  UDP port: "); Debug.println(UDP_remotePort);
      DisplayPrintln("Client connected");
      DisplayPrintln("Remote UDP IP =");
      DisplayPrintln(UDP_remoteIP.toString());
      DisplayPrintln("Remote UDP port =");
      DisplayPrintln(String(UDP_remotePort));
     }
  }
  #endif
  
 #endif  

  //=================================================================================================  
  
