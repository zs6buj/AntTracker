#include <Arduino.h>

//=================================================================================================  
//================================================================================================= 
//
//                                       U T I L I T I E S
//
//================================================================================================= 
//=================================================================================================   

  //========================================================
  uint32_t TenToPwr(uint8_t pwr) 
  {
    uint32_t ttp = 1;
    for (int i = 1 ; i<=pwr ; i++) 
    {
      ttp*=10;
    }
    return ttp;
  }  
  //========================================================
  uint32_t createMask(uint8_t lo, uint8_t hi) 
  {
    uint32_t r = 0;
    for (unsigned i=lo; i<=hi; i++) r |= 1 << i;
      //  log.print(" Mask 0x=");
      //  log.println(r, HEX);      
    return r;
    }    
  uint32_t bit32Extract(uint32_t dword,uint8_t displ, uint8_t lth) 
  {
    uint32_t r = (dword & createMask(displ,(displ+lth-1))) >> displ;
    //  log.print(" Result=");
    // log.println(r);
    return r;
  }
  //========================================================

  uint32_t uint32Extract(uint8_t *buf, int posn){

    //  The number starts at byte "posn" of the received packet and is four bytes long.
    //  GPS payload fields are little-endian, i.e they need an end-to-end byte swap

    byte b1 = buf[posn+3];
    byte b2 = buf[posn+2];
    byte b3 = buf[posn+1];
    byte b4 = buf[posn]; 

    unsigned long highWord = b1 << 8 | b2;
    unsigned long lowWord  = b3 << 8 | b4;

      // Now combine the four bytes into an unsigned 32bit integer

    uint32_t myvar = highWord << 16 | lowWord;
    return myvar;
  }
  //========================================================
  int32_t int32Extract (uint8_t *buf, int posn){

    //  The number starts at byte "posn" of the received packet and is four bytes long.
    //  GPS payload fields are little-endian, i.e they need an end-to-end byte swap

    byte b1 = buf[posn+3];
    byte b2 = buf[posn+2];
    byte b3 = buf[posn+1];
    byte b4 = buf[posn]; 

    unsigned long highWord = b1 << 8 | b2;
    unsigned long lowWord  = b3 << 8 | b4;

    // Now combine the four bytes into an unsigned 32bit integer

    int32_t myvar = highWord << 16 | lowWord;
    return myvar;
  }
  //========================================================
  uint16_t uint16Extract(uint8_t *buf, int posn){

    //  The number starts at byte "posn" of the received packet and is two bytes long
    //  GPS payload fields are little-endian, i.e they need an end-to-end byte swap

      byte b1 = buf[posn+1];
      byte b2 = buf[posn];  

      // Now convert the 2 bytes into an unsigned 16bit integer

      uint16_t myvar = b1 << 8 | b2;
      return myvar;
  }
  //========================================================
  int16_t int16Extract(uint8_t *buf, int posn){

    //  The number starts at byte "posn" of the received packet and is two bytes long
    //  GPS payload fields are little-endian, i.e they need an end-to-end byte swap
    byte b1 = buf[posn+1];
    byte b2 = buf[posn];

    // Now convert the 2 bytes into a signed 16bit integer

      int16_t myvar = b1 << 8 | b2;
      return myvar;
  }
  //========================================================
  uint8_t uint8Extract(uint8_t *buf, int posn)
  {

    //  The number starts at byte "posn" of the received packet and is one byte long

    byte b1 = buf[posn];

    // Now convert the byte into an unsigned 8 bit integer

    uint8_t myvar = b1;
    return myvar;
  }
  //========================================================
  int8_t int8Extract(uint8_t *buf, int posn)
  {

    //  The number starts at byte "posn" of the received packet and is one byte long

    byte b1 = buf[posn];

    // Now convert the byte into an unsigned 8 bit integer

    int8_t myvar = b1;
    return myvar;
  }
    //==================================
  void printByte(uint8_t b) 
  {
    if (b<=0xf) log.print("0");
      log.print(b,HEX);
      log.print(" ");
  }
  //======================================= 
  void printBytes(uint8_t *buf, uint8_t lth)
  {
    for (auto i = 0; i < lth; i++ ) 
    {
      printByte(buf[i]);
    }
    log.println();
  }
  //======================================= 
  void PrintFrsBuffer(byte *buf, uint8_t len)
  {
    log.printf("len:%u  ", len); 
    for ( int i = 0; i < len; i++ ) {
      printByte(buf[i]);
    }
    log.println();
  }
  //======================================= 
  void Printbyte(byte b, bool LF, char delimiter) 
  {
    if ( (b == 0x7E) && (LF) ) {//             || (b == 0x10)  || (b == 0x32)) {
      log.println();
    } 
    if (b<=0xf) log.print("0");
    log.print(b,HEX);
    log.write(delimiter);
  }
//=================================================================================================  
uint32_t epochNow() {
return (epochSync + (millis() - millisSync) / 1E3);
}
//============================================================================
//                             EEPROM Routines 
//============================================================================

uint8_t EEPROMreadByte(uint16_t addr)
{
  #if defined ESP32
    uint8_t temp = EEPROM.readByte(addr);
  #else // stm32, teensy
    uint8_t temp = EEPROM.read(addr);
  #endif  
  return temp;
}

void EEPROMwriteByte(uint16_t addr, uint8_t byt)
{
  #if defined ESP32
    EEPROM.writeByte(addr, byt);
  #else // stm32, teensy
    EEPROM.write(addr, byt);
  #endif  
}

void EEPROMwriteULong(uint16_t addr, int32_t value) 
{ 
  #if (defined ESP32)
    EEPROM.writeULong(addr, epochNow());  
    EEPROM.commit();    
    #if defined DEBUG_EEPROM
      log.print("EEPROMwriteULong():"); 
      log.print("epochNow() 0x"); log.println(epochNow(), HEX);    
    #endif 
  #else // esp8266, stm32, teensy
    //one = most significant , four = least significant byte
    byte four = (value & 0xFF);
    byte three = ((value >> 8) & 0xFF);
    byte two = ((value >> 16) & 0xFF);
    byte one = ((value >> 24) & 0xFF);
    EEPROMwriteByte(addr, four);
    EEPROMwriteByte(addr + 1, three);
    EEPROMwriteByte(addr + 2, two);
    EEPROMwriteByte(addr + 3, one);
    #if defined DEBUG_EEPROM
      log.print("EEPROMwriteULong():"); 
      log.print("  composit="); log.print(value);     
      log.print("  one="); log.print(one, HEX);
      log.print("  two="); log.print(two, HEX);
      log.print("  three="); log.print(three, HEX);
      log.print("  four="); log.println(four, HEX);    
    #endif 
  #endif    
} 
//======================================================  
int32_t EEPROMreadULong(uint16_t addr) { 
  //One = Most significant , Four = Least significant byte
  uint32_t four = EEPROMreadByte(addr);
  uint32_t three = EEPROMreadByte(addr + 1);
  uint32_t two = EEPROMreadByte(addr + 2);
  uint32_t one = EEPROMreadByte(addr + 3);
  uint32_t composit = ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
  
  #if defined DEBUG_ALL || defined DEBUG_EEPROM
     log.print("EEPROMReadLong():"); 
     log.print("  one="); log.print(one, HEX);
     log.print("  two="); log.print(two, HEX);
     log.print("  three="); log.print(three, HEX);
     log.print("  four="); log.print(four, HEX);    
     log.print("  composit="); log.println(composit);      
  #endif     
  return composit;
}
//======================================================
void EEPROMwriteFloat(uint16_t idx, float value) 
{
  uint32_t temp = uint32_t(value);
  EEPROMwriteULong(idx, temp);
  #if defined DEBUG_EEPROM
    log.print("EEPROMwriteFloat():"); 
    log.printf("float:%5.7f uint32_t:%u\n", value, temp);   
  #endif      
}
//====================================================== 
 float EEPROMreadFloat(uint16_t idx) 
 { 
    uint32_t temp1 =  EEPROMreadULong(idx);
    float temp2 = (float)temp1;
    #if defined DEBUG_EEPROM
      log.print("EEPROMreadFloat():"); 
      log.printf("uint32_t:%u  float:%5.7f\n", temp1, temp2);   
    #endif  
    return temp2;
 }

//=================================================================================================  
uint32_t epochHome() 
{
  uint32_t epHome = 0;
  uint8_t offset = home_eeprom_offset + 20;  
  epHome = EEPROMreadULong(offset);
  offset += sizeof(unsigned long); //4
 #if defined DEBUG_ALL || defined DEBUG_EEPROM
   log.print("epochHome="); log.println(TimeString(epHome));
 #endif
 return epHome;    
}

//======================================= 
  /*
       ========= Support for ESPNOW (ELRS_Backpack) =======
  */
  #if (MEDIUM_IN  == 5)   
    bool eepromToMac(uint8_t _mac[6])
    {
      log.println("eepromToMac()");
      uint8_t offset = espnow_eeprom_offset;  // ==24, "home" uses 0 thru 19
      have_eeprom_mac = EEPROMreadByte(offset);
      if (have_eeprom_mac != 0xfd)
      {
        log.println("No soft_mac in eeprom");
        return false;
      }
      offset += sizeof(byte);
      for (auto i = 0; i < 6; i++)
      {
      _mac[i] = EEPROMreadByte(offset);
        //log.printf("%u:%u,", offset, _mac[i]);    
        offset += sizeof(byte);
      }
      //log.println();   
      return true;
    }

    void macToEeprom(const uint8_t addr[6])
    {
      log.println("macToEeprom()");
      uint8_t offset = espnow_eeprom_offset;  // ==24, "home" uses 0 thru 19
      have_eeprom_mac = 0xfd;
      EEPROMwriteByte(offset, have_eeprom_mac);
      //log.printf("eeprom[%u]:%2x\n", offset, have_eeprom_mac);
      offset += sizeof(have_eeprom_mac);
      for (auto i = 0; i < 6; i++)
      {
        EEPROMwriteByte(offset, addr[i]);
        //log.printf("%u:%u,", offset, addr[i]);    
        offset += sizeof(byte);
      }
      //log.println();  
      EEPROM.commit();
    }
    void setSoftMACAddress()
    {
        // MAC address can only be set with unicast, so first byte must be even, not odd
      soft_mac[0] = soft_mac[0] & ~0x01;
      WiFi.mode(WIFI_STA);
      WiFi.setTxPower(WIFI_POWER_19_5dBm);
      esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR);
      WiFi.begin("network-name", "pass-to-network", 1);
      WiFi.disconnect();
      log.println("Soft MAC address set"); 
      esp_wifi_set_mac(WIFI_IF_STA, soft_mac);
    }  // end ESP_NOW
  #endif
  #if (MEDIUM_IN  == 2)      // WiFi UDP select
  //========================================================
  void printRemoteIP()
  {
    if (showRemoteIP)
    {
      showRemoteIP = false;
      log.print("UDP client identified, remote IP: ");
      log.print(UDP_remoteIP);
      log.print(", remote port: ");
      log.println(UDP_REMOTEPORT);
    }
  }
  //========================================================
    uint16_t readUDP()
  {
    if (!wifiSuGood)
      return 0;
    if (!(wifiStaConnected))
      return 0;
    if (!(wifiApConnected))
      return 0;    
    #if (WIFI_MODE == 2)  // STA
      udp_read_port = UDP_REMOTEPORT; // used by printRemoteIP() only. read port set by ->begin(udp_read_port)).
      udp_send_port = UDP_LOCALPORT;
      //log.printf("readUDP() read port:%u    send port:%u\n", udp_read_port, udp_send_port);
    #endif
    #if (WIFI_MODE == 1)  // AP
        udp_read_port = UDP_LOCALPORT;
        udp_send_port = UDP_REMOTEPORT;
    #endif
    int16_t packetSize = udp_object.parsePacket();
    // esp sometimes reboots here: WiFiUDP.cpp line 213 char * buf = new char[1460];
    //log.printf("Read UDP port:%d  packetSize:%d\n", udp_read_port, packetSize);
    if (packetSize)
      {
        uint16_t len = 0;
        //#if (PROTOCOL == x) // for future
        //#endif
        #if (PROTOCOL == 9)  // this to prevent compile error when not protocol 9
          len = udp_object.read(&*crsf.crsf_buf, 64);
        #endif
        //lenlog.printf("len:%2u:", len);
        if (len >= 0)
        {
          wifi_recv_millis = millis();
          wifi_recv_good = true;
          //printBytes(buff);
          #if (not defined UDP_Broadcast)
            UDP_remoteIP = udp_object.remoteIP();
            bool in_table = false;
              for (int i = 1; i < max_clients; i++)
              {
                if (udpremoteip[i] == UDP_remoteIP)
                { // IP already in the table
                  //    log.printf("%s already in table\n", UDP_remoteIP.toString().c_str() );
                  in_table = true;
                  break;
                }
              }
              if (!in_table)
              { // if not in table, add it into empty slot, but not [0] reserved for otgoing (FC side)
                for (int i = 1; i < max_clients; i++)
                {
                  if ((udpremoteip[i][0] == 0) || (udpremoteip[i][3] == 255))
                  {                                // overwrite empty or broadcast ips
                    udpremoteip[i] = UDP_remoteIP; // remember unique IP of remote udp client so we can target it
                    log.printf("%s client inserted in UDP client table\n", UDP_remoteIP.toString().c_str());
                    showRemoteIP = true;
                    break;
                  }
                }
              }
          #endif
          printRemoteIP();
          return len;
        }  
      }
      return 0;
  }
  #endif  // end of UDP
  //===================================================
  uint16_t getNextPacket()
  { 
    uint16_t len = 0;
    #if (MEDIUM_IN == 1) ||  ((defined btBuiltin) &&  (MEDIUM_IN == 3))       //   UART or BT
      len=inSerial.available();     //   wait for more data
    #endif  
    #if (MEDIUM_IN == 2) && (defined wifiBuiltin)     // WiFi UDP  
      len = udp_object.parsePacket();                 // packet to in_buffer?
    #endif 
    #if (defined bleBuiltin) &&  (MEDIUM_IN == 4)      //   BLE4
      //len=inSerial.available();     //   wait for more data
    #endif  
    if (len > 0) 
    {
      telem_millis = millis();
      telemGood = true;
    }
    return len; 
  }
  //====================================================
  byte nextByte() 
  {
  byte b;
    iLth = getNextPacket();
    while (iLth==0) 
    {
      // checkStatusAndTimeouts();
      iLth = getNextPacket();
    }  
    // Data is available
    #if (MEDIUM_IN == 1) ||  ((defined btBuiltin) &&  (MEDIUM_IN == 3))       //   UART or BT
      b = inSerial.read();
    #endif  
    #if (MEDIUM_IN == 2) && (defined wifiBuiltin)                             // UDP  
      b = udp_object.read();
    #endif 
  // printByte(x, false, ' ');  
    return b;
  }
  //========================================================
  String TimeString (unsigned long epoch)
  {
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
  void PrintMavBuffer(const void *object) 
  {
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
        printByte(bytes[0]); // CRC_Out1
        printByte(bytes[1]); // CRC_Out2

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
        printByte(bytes[i]); 
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
        printByte(bytes[i]); 
      }
      if (j == -2) {
        log.print("//");
        printByte(bytes[mav_len + 8]); 
        printByte(bytes[mav_len + 9]);       
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
        printByte(bytes[0]); // CRC_Out1 
        printByte(bytes[1]); // CRC_Out2            
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
      printByte(bytes[i]); 
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
        printByte(bytes[i]);       
      }
      log.println();
    }
    log.print("Raw: ");
    for (int i = 0; i < 40; i++) {  //  unformatted
        printByte(bytes[i]);  
      }
    log.println();
  }


  //=======================================
  # if defined NEED_I2C
    void scanI2C()
    // 0x03C for SSD1306_DISPLAY, 0x1E for HMC5883L, 0x0D for QMC5883
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
            log.println("likely SSD1306_DISPLAY");
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
  #endif  
  //=======================================
  bool PacketGood() 
  {
    // Allow 1 degree of lat and lon away from home, i.e. 60 nautical miles radius at the equator
    // Allow 1km up and 300m down from home altitude
    if (finalHomeStored==0) {  //  You can't use the home co-ordinates for a reasonability test if you don't have them yet
      return true;
      }
    if (cur.lat<(hom.lat-1.0) || cur.lat>(hom.lat+1.0)) // Also works for negative lat
    {  
      #if defined SHOW_BAD_PACKETS
        log.print(" Bad lat! cur.lat=");
        log.print(cur.lat,7);  
        log.print(" hom.lat=");log.print(hom.lat,7);
        log.println("  Packet ignored");  
      #endif
      return false; 
      }
    if (cur.lon<(hom.lon-1.0) || cur.lon>(hom.lon+1.0)) // Also works for negative lon
    { // Also works for negative lon
      #if defined SHOW_BAD_PACKETS   
        log.print(" Bad lon! cur.lon=");
        log.print(cur.lon,7);  
        log.print(" hom.lon=");log.print(hom.lon,7);
        log.println("  Packet ignored");
      #endif  
      return false;  
      }
    if (cur.alt<(hom.alt - 300.0) || cur.alt>(hom.alt + 2000.0)) 
    {
      #if defined SHOW_BAD_PACKETS    
        log.print(" Bad alt! cur.alt=");
        log.print(cur.alt, 0);  
        log.print(" hom.alt=");log.print(hom.alt, 0);
        log.println("  Packet ignored");
      #endif    
      return false;  
      }
    if ((cur.alt-hom.alt) < -300.0 || (cur.alt-hom.alt) > 2000.0) 
    {
        log.print(" Bad alt! cur.alt=");
        log.print(cur.alt, 0);  
        log.print(" hom.alt=");log.print(hom.alt, 0);
        log.println("  Packet ignored");    
      return false;  
      }
    if (headingsource == 2) 
    { //  Heading source from flight controller
      if (cur.hdg<0 || cur.hdg>360) 
      {
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
    //===================================================================  
  #if defined DISPLAY_PRESENT       
  void setScreenSizeOrient(uint8_t txtsz, uint8_t scr_orient) 
  {

    #if (defined SSD1306_DISPLAY) || (defined SSD1331_DISPLAY)   // rotation arguement depends on display type
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
    #else                             // ST7789 (T-Display) and ILI9341_DISPLAY
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
  #if defined DISPLAY_PRESENT    
  void setupLogDisplayStyle() 
  {
    setScreenSizeOrient(TEXT_SIZE, SCR_ORIENT);
      
    #if (defined ST7789_DISPLAY)      // LILYGO® TTGO T-Display ESP32 1.14" ST7789 Colour LCD
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
  
    #elif (defined SSD1306_DISPLAY)            // all  boards with SSD1306 OLED display
      display.clearDisplay(); 
      display.setTextColor(WHITE);     
            
    #elif (defined SSD1331_DISPLAY)            // T2 board with SSD1331 colour TFT display
      //  software SPI pins defined in config.h 
      display.fillScreen(SCR_BACKGROUND);
      display.setCursor(0,0);
      
    #elif (defined ILI9341_DISPLAY)           // ILI9341 2.8" COLOUR TFT SPI 240x320 V1.2  
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
    #if defined DISPLAY_PRESENT
    void paintLogScreen(uint8_t new_row, last_row_t last_row_action) 
    { 
      if (display_mode != logg) { 
          setupLogDisplayStyle();
          display_mode = logg; 
      }  
        #if (defined ST7789_DISPLAY) || (defined SSD1331_DISPLAY) ||  (defined ILI9341_DISPLAY)   
        //  hardware SPI pins defined in config.h 
          display.fillScreen(SCR_BACKGROUND);                 
        #elif (defined SSD1306_DISPLAY) 
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
   
        #if (defined SSD1306_DISPLAY)
          display.display();
        #endif 
    }
  #endif  
//===================================
    void logScreenPrintln(String S) {
    #if defined DISPLAY_PRESENT   
     
      if (display_mode != logg) {
          setupLogDisplayStyle();
          display_mode = logg; 
          paintLogScreen(row, omit_last_row);
      } else {   
        if (row >= scr_h_ch) {                      // if the new line exceeds the page lth, re-display existing lines
          paintLogScreen(row, omit_last_row);
        }
      }
      uint16_t lth = strlen(S.c_str());           // store the new line a char at a time
      if (lth > scr_w_ch) {    
        log.printf("Display width of %d exceeded for |%s|\n", scr_w_ch, S.c_str());  // scr_w_ch = max_col-1
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
      #if (defined SSD1306_DISPLAY)
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
   
    void logScreenPrint(String S) {
    #if defined DISPLAY_PRESENT   
    
      if (display_mode != logg) {
          setupLogDisplayStyle();
          display_mode = logg; 
          paintLogScreen(row, omit_last_row);
      } else {   
        if (row >= scr_h_ch) {                      // if the new line exceeds the page lth, re-display existing lines
          paintLogScreen(row, omit_last_row);
        }
      }
      display.print(S);                         // the new line
      #if (defined SSD1306_DISPLAY)
        display.display();
      #endif 
       
      uint8_t lth = strlen(S.c_str());          // store the line a char at a time
      if (lth > scr_w_ch) {
        log.printf("Display width of %d exceeded for |%s|\n", scr_w_ch, S.c_str());  // scr_w_ch = max_col-1
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
void logScreenPrintChar(char ch) 
{
#if defined DISPLAY_PRESENT   

  if (display_mode != logg) {
      setupLogDisplayStyle();
      display_mode = logg; 
      paintLogScreen(row, omit_last_row);
  } else {   
    if (row >= scr_h_ch) {                      // if the new line exceeds the page lth, re-display existing lines
      paintLogScreen(row, omit_last_row);
    }
  }
  // display.setCursor( ((col+1) * char_w_px), ((row+1) * char_h_px) );
  display.print(ch);                         // the new char
  #if (defined SSD1306_DISPLAY)
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
//===========================================================================================
void driveLED(uint8_t gpio, uint8_t state)
{
  #if defined INVERT_LED
    state = (state == LOW) ? HIGH : LOW; // ternary
  #endif
  digitalWrite(gpio, state);
}  
//===========================================================================================
void blinkLED(uint16_t period)
{ 
  if (millis() - millisLED >= period) 
  {    // blink period
    //log.printf("%u ", period); 
    millisLED = millis();
    ledState = (ledState == LOW) ? HIGH : LOW; // ternary
  }
}  
//===========================================================================================
void serviceTheStatusLed() 
{
  #ifdef DEBUG_LEDS
    static bool telemPrev = false;
    static bool gpsPrev = false;   
    static bool boxgpsPrev = false;
    static bool finalHomePrev = false;
    bool linefeed = false;  
    if (telemGood != telemPrev)
    {
      log.printf("telemGood:%u  ", telemGood);
      telemPrev = telemGood;
      linefeed = true;
    } 
    if (gpsGood != gpsPrev)
    {
      log.printf("gpsGood:%u  ", gpsGood);
      gpsPrev = gpsGood;
      linefeed = true;  
    } 
    if (boxgpsGood != boxgpsPrev)
    {
      log.printf("boxgpsGood:%u  ", boxgpsGood);
      boxgpsPrev = boxgpsGood;
      linefeed = true;  
    } 
    if (finalHomeStored != finalHomePrev)
    {
      log.printf("finalHomeStored:%u  ", finalHomeStored);
      finalHomePrev = finalHomeStored;
      linefeed = true;  
    } 
    if(linefeed) log.println();
    linefeed = false;
 #endif
  if (gpsGood) 
  {
    if ( (finalHomeStored) || ( (boxgpsGood) && boxmagGood) )
      ledState = HIGH;    // when home stored
    else 
      blinkLED(400);      // when gpsGood
  } else 
  {
    if (telemGood) 
       blinkLED(1200);    // when telemGood
     else
       ledState = LOW;
  }
  if (StatusLed != -1)
  {
    driveLED(StatusLed, ledState);  
  }
  if (BuiltinLed != -1)
  {    
    driveLED(BuiltinLed, ledState);  
  }
}
  //===================================================================     
  void reportOnlineStatus() 
  {
      if (telemGood != telemPrev) 
      {  
        telemPrev = telemGood;
        if (telemGood) {
           log.println("Good telemetry stream");
           logScreenPrintln("Good telem stream");         
         } else {
          log.println("Telemetry timeout *");
          logScreenPrintln("Telem timeout *");         
        }
      } 
      if (btGood != btPrev) 
      {  
        btPrev = btGood;
        if (btGood) {
           log.println("Good BT data stream");
           logScreenPrintln("Good BT stream");         
         } else {
          log.println("BT data stream timeout *");
          logScreenPrintln("BT stream timeout");         
        }
      }  
      if (motArmed != motPrev)
      {  // report on change of status
         motPrev = motArmed;
         if (motArmed) {
           log.println("Motors Armed!");
           logScreenPrintln("Motors Armed!");         
         } else {
          log.println("Motors Disarmed");
          logScreenPrintln("Motors Disarmed");         
         }
      }      
      if (frGood != frPrev) 
      {  // report on change of status
         frPrev = frGood;
         if (frGood) {
           log.println("FrSky read good!");
           logScreenPrintln("FrSky read ok");         
         } else {
          log.println("FrSky read timeout!");
          logScreenPrintln("FrSky timeout");         
         }
      }
      if (pwmGood != pwmPrev) 
      {  
         pwmPrev = pwmGood;
         if (pwmGood) 
         {
           log.println("RC PWM good");
           logScreenPrintln("RC PWM good");         
         } else {
          log.println("RC PWM timeout");
          logScreenPrintln("RC PWM timeout");         
         }
      }        
       if (gpsGood != gpsPrev) {  
         gpsPrev = gpsGood;
         if (gpsGood) {
           log.println("Good flight computer GPS lock");
           logScreenPrintln("FC GPS good");         
         } else {
          log.println("Flight Computer GPS timeout *********");
          logScreenPrintln("FC GPS timeout");         
         }
       }    
       #if (headingsource == 4)
         if (boxgpsGood != boxgpsPrev) {  
           boxgpsPrev = boxgpsGood;
           if (boxgpsGood) {
             log.println("Good box GPS lock");
             logScreenPrintln("Box GPS good");         
           } else {
            log.println("No box GPS lock!");
            logScreenPrintln("No box GPS lock!");         
           }
         }     
       #endif           
  } 

  //=================================================================================================  
  void checkStatusAndTimeouts() 
  {
    if ((millis() - telem_millis) > ((timeout_secs) * 1000) ) 
    {
      telemGood = false;        // if no telemetry
    }       
    if ((millis() - btGood_millis) > ((timeout_secs) * 1000) ) 
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
    reportOnlineStatus();
    serviceTheStatusLed();  
  }   


//=================================================================================================  
void restoreHomeFromFlash() 
{
  uint8_t offset = home_eeprom_offset;  // 0
    uint32_t epoch_now = EEPROMreadULong(offset);
    offset += sizeof(unsigned long); //4
    hom.lon = EEPROMreadFloat(offset); 
    offset += sizeof(float); //4
    hom.lat = EEPROMreadFloat(offset);
    offset += sizeof(float); 
    hom.alt = EEPROMreadFloat(offset);
    offset += sizeof(float); 
    hom.hdg = EEPROMreadFloat(offset);
    offset += sizeof(float); 

  #if defined DEBUG_ALL || defined DEBUG_EEPROM || defined DEBUG_TIME || defined DEBUG_HOME
    log.print("  home.lon="); log.print(hom.lon, 6);
    log.print("  home.lat="); log.print(hom.lat, 6);
    log.print("  home.alt="); log.print(hom.alt, 0);
    log.print("  home.hdg="); log.println(hom.hdg, 0);
  #endif  
}
//=================================================================================================  
void lostPowerCheckAndRestore(uint32_t epoch_now) 
{ // only ever called if active time supporting protocol 
  if ((!timeGood) || (epoch_now == 0)) return;
  
  if (lostPowerCheckDone) return;

  #if defined DEBUG_ALL || defined DEBUG_TIME || defined DEBUG_HOME
    log.print("Checking for restoreHomeFromFlash conditions:"); 
    log.print("  epochHome="); log.print(TimeString(epochHome())); 
    log.print("  epochNow="); log.println(TimeString(epochNow()));
  #endif 

  #if (HEADINGSOURCE != 4)  // If NOT (Tracker GPS + Compass). Home could move constantly.
    uint16_t decay_secs = epoch_now -  epochHome();  
    if (decay_secs <= HOME_DECAY_SECS) {  //  restore home if restart within decay seconds
      restoreHomeFromFlash();     
      log.printf("Home data restored from NVM, decay %d secs is within limit of %d secs\n", decay_secs, HOME_DECAY_SECS);     
      //log.printf("Home data restored from NVM, decay %d secs is within limit of %d secs\n", decay_secs, HOME_DECAY_SECS);     
      logScreenPrintln("Home data restored");
      logScreenPrintln("from Flash. Go Fly!");  
      finalHomeStored=1;           
    }
  #endif
  
  lostPowerCheckDone = true;
}
//================================================================================================= 
void displayEEPROM() {
  log.print("EEPROM:");
  for (int i = 0; i < EEPROM_SIZE; i++) 
  {
    printByte(EEPROMreadByte(i)); 
  }
  log.println();
}
//=================================================================================================  
void saveHomeToFlash() 
{
  uint8_t offset = home_eeprom_offset;  // 0

    EEPROMwriteULong(offset, epochNow());// epochHome
    offset += sizeof(unsigned long); //4
    EEPROMwriteFloat(offset, hom.lon ); 
    offset += sizeof(float);
    EEPROMwriteFloat(offset, hom.lat);
    offset += sizeof(float);
    EEPROMwriteFloat(offset, hom.alt);
    offset += sizeof(float);
    EEPROMwriteFloat(offset, hom.hdg);  
    offset += sizeof(float);
    #if (defined ESP32) || (defined ESP8266)
      EEPROM.commit();
    #endif  

#if defined DEBUG_ALL || defined DEBUG_EEPROM || defined DEBUG_TIME || defined DEBUG_HOME
  log.print("  firstHomeStored:"); log.print(firstHomeStored);
  log.print("  home.lon:"); log.print(hom.lon, 6);
  log.print("  home.lat:"); log.print(hom.lat, 6);
  log.print("  home.alt:"); log.print(hom.alt, 1);
  log.print("  home.hdg:"); log.println(hom.hdg, 1);
#endif   
}
//=================================================================================================  
void storeEpochPeriodic() 
{
  uint8_t offset = home_eeprom_offset + 20;  //displaced by 20B
    uint32_t epochPeriodic = epochNow();
    EEPROMwriteULong(offset, epochPeriodic); // Seconds
    offset += sizeof(unsigned long);
    #if (defined ESP32) || (defined ESP8266)
      EEPROM.commit();
    #endif  
  if (finalHomeStored) 
  {
    offset = home_eeprom_offset;  //displaced by 0 bytes
    #if (defined ESP32) || (defined ESP8266)
      EEPROMwriteULong(offset, epochPeriodic); // UPDATE epochHome
      offset += sizeof(unsigned long);
      EEPROM.commit();
    #else // stm32, teensy
      EEPROMwriteULong(offset, epochPeriodic);  // Seconds
      offset += sizeof(unsigned long);
    #endif  
    #if defined DEBUG_ALL || defined DEBUG_EEPROM || defined DEBUG_TIME || defined DEBUG_HOME
      log.print("epochHome stored="); log.println(TimeString(epochPeriodic));
    #endif  
  }
  
  #if defined DEBUG_ALL || defined DEBUG_EEPROM || defined DEBUG_TIME || defined DEBUG_HOME
  log.print("epochPeriodic stored="); log.println(TimeString(epochPeriodic));
  #endif  
}

//=================================================================================================  
#if (MEDIUM_IN == 2)   //  WiFi
  void serviceWiFiRoutines()
  {
    // Report stations connected to/from our AP
    uint8_t AP_sta_count = WiFi.softAPgetStationNum();
    static uint8_t AP_prev_sta_count = 0;
    wifiApConnected = (AP_sta_count > 0);
    if (AP_sta_count > AP_prev_sta_count)
    {
      AP_prev_sta_count = AP_sta_count;
      log.printf("Remote STA %d connected to our AP\n", AP_sta_count);
    }
    else if (AP_sta_count < AP_prev_sta_count)
    { // a device has disconnected from the AP
      AP_prev_sta_count = AP_sta_count;
      log.println("A STA disconnected from our AP"); // back in listening mode
    }
  }
  void setupWiFi() 
  { 
    bool apMode = false;  // used when STA fails to connect
     //===============================  S T A T I O N   =============================
     
    #if (WIFI_MODE == 2) || (WIFI_MODE == 3)  // STA or SPA>AP
      uint8_t retry = 0;
      log.printf("Trying to connect to \"%s\" ", STAssid);  
     // log.print(STAssid); 
      logScreenPrintln("WiFi trying ..");

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
          logScreenPrintln("Failed in STA Mode");
          wifiSuDone = true;
          #if (WIFI_MODE ==  3)       
            apMode = true;            // Rather go establish an AP instead
            log.println("Starting AP instead.");
            logScreenPrintln("Starting AP instead");  
            #if defined ESP32
              //new from Target0815:
              log.println("WiFi-Reset ...");
              WiFi.mode(WIFI_MODE_NULL);    
              delay(500); 
            #endif      
            #if defined ESP8266
              log.println("WiFi-Reset ...");
              WiFi.mode(WIFI_OFF); 
              delay(500); 
            #endif                       
          #endif  
          
          break;
        }
        delay(500);
        log.print(".");
      }
      
      if (WiFi.status() == WL_CONNECTED) {
        wifiStaConnected = true;
        localIP = WiFi.localIP();
        log.println();
        log.println("WiFi connected!");
        log.print("Local IP address: ");
        log.print(localIP);

        #if (MEDIUM_IN == 2) && (WIFI_PROTOCOL == 1)   // Mav TCP
            log.print("  TCP port: ");
            log.println(TCP_LOCALPORT);    //  UDP port is printed lower down
        #else 
            log.println();
        #endif 
         
        wifi_rssi = WiFi.RSSI();
        log.print("WiFi RSSI:");
        log.print(wifi_rssi);
        log.println(" dBm");

        logScreenPrintln("Connected! My IP =");
        logScreenPrintln(localIP.toString());

        #if (WIFI_PROTOCOL == 1)          // TCP                                                
          // We are a client and need to connect to a server
          outbound_clientGood = NewOutboundTCPClient();
        #endif
        #if (WIFI_PROTOCOL == 2)          // UDP 
          udp_read_port = UDP_REMOTEPORT;
          udp_send_port = UDP_LOCALPORT; // so we swap read and send ports, local (read) becomes 14550
          //WiFiUDP UDP_STA_Object;
          //udp_object = new WiFiUDP(UDP_STA_Object);
          log.printf("Begin UDP using STA UDP object  read port:%d  send port:%d\n", udp_read_port, udp_send_port);
          udp_object.begin(udp_read_port); 
          log.printf("UDP instance started, listening on IP %s, UDP local port %d\n", localIP.toString().c_str(), udp_read_port);                 
          logScreenPrint("UDP port = ");  logScreenPrintln(String(udp_read_port));
        #endif
        wifiSuGood = true;
        wifiSuDone = true;
      } 
    #endif
     //===============================  Access Point   =============================  

    #if (WIFI_MODE == 1)  // AP
      apMode = true;
    #endif

    if (apMode)   
    {
      WiFi.mode(WIFI_AP);
      WiFi.softAP(APssid, APpw, APchannel);
      localIP = WiFi.softAPIP();
      log.print("AP IP address: ");
      log.print (localIP); 
      log.printf("  SSID: \"%s\"\n", String(APssid));
      logScreenPrintln("WiFi AP SSID =");
      logScreenPrintln(String(APssid));

      #if (MEDIUM_IN == 2)                  // WiFi
        #if (WIFI_PROTOCOL == 2)            // UDP 
          // regular AP
          udp_read_port = UDP_LOCALPORT;
          udp_send_port = UDP_REMOTEPORT;
          //WiFiUDP UDP_Object;
          //udp_object = new WiFiUDP(UDP_Object);
          log.printf("Begin UDP using AP UDP object  read port:%d  send port:%d\n", udp_read_port, udp_send_port);
          udp_object.begin(udp_read_port);
          log.printf("UDP instance started, listening on IP %s, UDP port %d\n", localIP.toString().c_str(), udp_read_port);              
          logScreenPrint("UDP port = ");  logScreenPrintln(String(udp_read_port));
        #endif
      #endif
      wifiSuGood = true;
    }           
    wifiSuDone = true;
 }   
  //=================================================================================================  
  #if (WIFI_PROTOCOL == 1)    //  WiFi TCP      
    bool NewOutboundTCPClient() 
    {
    static uint8_t retry = 3;
      WiFiClient newClient;        
      while (!newClient.connect(TCP_remoteIP, TCP_REMOTEPORT)) {
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
      log.print(" remote Port:"); log.println(TCP_REMOTEPORT);
      nbdelay(1000);
      logScreenPrintln("Remote server IP =");
      log.printf("%d.%d.%d.%d", TCP_remoteIP[0], TCP_remoteIP[1], TCP_remoteIP[2], TCP_remoteIP[3]);               
      
      //   logScreenPrintln(TCP_remoteIP.toString()); 
      return true;
    }
    #endif
    //=================================================================================================  
    #if (MEDIUM_IN == 2) &&  (WIFI_PROTOCOL == 2) //  WiFi && UDP - Print the remote UDP IP the first time we get it  
    void PrintRemoteIP() {
      if (FtRemIP)  {
        FtRemIP = false;
        log.print("Client connected: Remote UDP IP: "); log.print(UDP_remoteIP);
        log.print("  Remote  UDP port: "); log.println(UDP_REMOTEPORT);
        logScreenPrintln("Client connected");
        logScreenPrintln("Remote UDP IP =");
        logScreenPrintln(UDP_remoteIP.toString());
        logScreenPrintln("Remote UDP port =");
        logScreenPrintln(String(UDP_REMOTEPORT));
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
#if (defined ESP32)   && ( (MEDIUM_IN == 2) || (MEDIUM_IN == 2)) && (defined DEBUG_WIFI)
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
  #if defined DISPLAY_PRESENT  
    void setupInfoDisplayStyle()
    {
      setScreenSizeOrient(TEXT_SIZE, SCR_ORIENT);
      #if (defined ST7789_DISPLAY)      // LILYGO® TTGO T-Display ESP32 1.14" ST7789 Colour LCD
        #if (SCR_ORIENT == 0)           // portrait
          setScreenSizeOrient(1, 0);    // change text size
          display.setTextSize(1);       // and font
          display.setTextFont(0);       // Original Adafruit font 0, try 0 thru 6 
        #elif (SCR_ORIENT == 1)         // landscape
          setScreenSizeOrient(2, 3);    // change text size
          display.setTextFont(1);       // and font 
        #endif    

        display.fillScreen(SCR_BACKGROUND);
        display.setTextColor(TFT_SKYBLUE);    
            
        //display.setTextColor(TFT_WHITE);
        //display.setTextColor(TFT_BLUE);  
        //display.setTextColor(TFT_GREEN, TFT_BLACK);
    
      #elif (defined SSD1306_DISPLAY)            // all  boards with SSD1306 OLED display    
        display.clearDisplay(); 
        display.setTextColor(WHITE);  

      #elif (defined SSD1331_DISPLAY)            // T2 board with SSD1331 colour TFT display     
        //  SPI pins defined in config.h 
        display.fillScreen(BLACK);
        display.setTextColor(WHITE);  
      #elif (defined ILI9341_DISPLAY)            // ILI9341 2.8" COLOUR TFT SPI 240x320 V1.2          
        //  SPI pins defined in config.h 
        display.fillScreen(SCR_BACKGROUND);
        display.setRotation(3);          // landscape pins on the left   
        display.setCursor(0,0);
      #endif
     }
  #endif    
    //=================================== 
  #if defined DISPLAY_PRESENT 
    void displayFlightInfo() 
    {
      uint16_t xx, yy; 
      if (display_mode != flight_info) {
          setupInfoDisplayStyle();
          display_mode = flight_info; 
      }
      

      #if  (defined ILI9341_DISPLAY)

        if (millis() - info_millis > 200) {    // refresh rate
          info_millis = millis();  

          // artificial horizon
          draw_horizon(hud_roll, hud_pitch, scr_w_px, scr_h_px);
          
          setScreenSizeOrient(2, SCR_ORIENT);   // text size, screen orientation, 26 ch wide x 15 ch deep
          
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
          snprintf(snprintf_buf, snp_max, "Alt:%3.0f", F_ag);    // m 
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
          setScreenSizeOrient(1, SCR_ORIENT);   // text size, screen orientation             
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
          setScreenSizeOrient(2, SCR_ORIENT);   // text size, screen orientation, 26 ch wide x 15 ch deep
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
          #if (defined SSD1306_DISPLAY)
            display.display();
          #endif   
                 
          // Altitude 
          xx = 0;
          #if (defined SSD1306_DISPLAY) 
            yy = 1.2 * char_h_px;  
          #elif (defined SSD1331_DISPLAY)
            yy = 1.1 * char_h_px;            
          #elif (defined ST7789_DISPLAY)   
            yy = 1.8 * char_h_px;  
          #endif  
          display.setCursor(xx, yy);            
          snprintf(snprintf_buf, snp_max, "Alt %.0f", (cur.alt_ag));
          display.fillRect(xx+(4*char_w_px), yy, (4 * char_w_px), char_h_px, SCR_BACKGROUND);  // blank Alt
          display.println(snprintf_buf);  
          
          // Heading
          xx = 9 * char_w_px;
          #if (defined SSD1306_DISPLAY) 
            yy = 1.2 * char_h_px;  
          #elif (defined SSD1331_DISPLAY)
            yy = 1.1 * char_h_px;            
          #elif (defined ST7789_DISPLAY)   
            yy = 1.8 * char_h_px;  
          #endif  
          display.setCursor(xx, yy);            
          snprintf(snprintf_buf, snp_max, "Hdg %.0f", cur.hdg); 
          display.fillRect(xx+(4*char_w_px), yy, (5 * char_w_px), char_h_px, SCR_BACKGROUND); // blank Hdg
          display.println(snprintf_buf); 
                    
          // Ground Speed m/s
          xx = 0;
          #if (defined SSD1306_DISPLAY) 
            yy = 2.4 * char_h_px;  
          #elif (defined SSD1331_DISPLAY)
            yy = 2.2 * char_h_px;               
          #elif (defined ST7789_DISPLAY)   
            yy = 3.6 * char_h_px;  
          #endif               
          display.setCursor(xx, yy);            
          snprintf(snprintf_buf, snp_max, "Spd %2.1f", hud_grd_spd); 
          display.fillRect(xx+(4*char_w_px), yy, (4 * char_w_px), char_h_px, SCR_BACKGROUND);    // blank speed 
          display.println(snprintf_buf);  
                 
          // Climb - vert speed m/s
          xx = 9 * char_w_px;
          #if (defined SSD1306_DISPLAY) 
            yy = 2.4 * char_h_px;  
          #elif (defined SSD1331_DISPLAY)
            yy = 2.2 * char_h_px;               
          #elif (defined ST7789_DISPLAY)   
            yy = 3.6 * char_h_px;  
          #endif               
          display.setCursor(xx, yy);            
          snprintf(snprintf_buf, snp_max, "Clm %2.1f", hud_climb); 
          display.fillRect(xx+(4*char_w_px), yy, (4 * char_w_px), char_h_px, SCR_BACKGROUND);   // blank climb 
          display.println(snprintf_buf);   
                    
          // Volts, Amps and Ah 
          xx = 0;
          #if (defined SSD1306_DISPLAY) 
            yy = 3.6 * char_h_px;  
          #elif (defined SSD1331_DISPLAY)
            yy = 3.3 * char_h_px;                
          #elif (defined ST7789_DISPLAY)   
            yy = 5.4 * char_h_px;  
          #endif            
          display.setCursor(xx, yy);               
          snprintf(snprintf_buf, snp_max, "%2.1fV %2.0fA %2.1fAh", hud_bat1_volts * 0.1F, hud_bat1_amps * 0.1F, hud_bat1_mAh * 0.001F);     
          display.fillRect(xx, yy, scr_w_px, char_h_px, SCR_BACKGROUND); // clear the whole line  
          display.println(snprintf_buf); 

          // Latitude and Longitude
          xx = 0;
          
          #if (defined SSD1306_DISPLAY) 
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
            
          #elif (defined SSD1331_DISPLAY)       
            yy = 4.4 * char_h_px;
            display.setCursor(xx,yy);       
            snprintf(snprintf_buf, snp_max, "Lat %3.7f", cur.lat);               
            display.fillRect(xx+(4*char_w_px), yy, 12 * char_w_px, char_h_px, SCR_BACKGROUND); // clear lat
            display.println(snprintf_buf);
            yy = 5.5 * char_h_px; 
            snprintf(snprintf_buf, snp_max, "Lon %3.7f", cur.lon);  
            display.fillRect(xx+(4*char_w_px), yy, 12 * char_w_px, char_h_px, SCR_BACKGROUND); // clear lon
            display.println(snprintf_buf); 
                                                                  
          #elif (defined ST7789_DISPLAY)         
            yy = 7.2 * char_h_px;  
            display.setCursor(xx,yy);       
            snprintf(snprintf_buf, snp_max, "Lat %3.7f Lon %3.7f", cur.lat, cur.lon);        
            setScreenSizeOrient(TEXT_SIZE -1, SCR_ORIENT);  // small text size       
            display.fillRect(xx+(4*char_w_px), yy, 12 * char_w_px, char_h_px, SCR_BACKGROUND); // clear the previous data 
            display.fillRect(xx+(16*char_w_px), yy, 12 * char_w_px, char_h_px, SCR_BACKGROUND); 
            display.println(snprintf_buf);  
            setScreenSizeOrient(TEXT_SIZE, SCR_ORIENT);  // restore text size           
          #endif             
        }
      #endif    
    } 
    #endif    
    //===================================
  #if defined DISPLAY_PRESENT  
    void scrollDisplay(scroll_t up_dn) 
    {
      if (millis() - scroll_millis < 300) return;
      show_log = true;    
      scroll_millis = millis(); 
      
      if (up_dn == up) {  // towards last line painted, so lines move up
         scroll_row--;
         scroll_row = constrain(scroll_row, scr_h_ch, row);
         up_button = false; 
         paintLogScreen(scroll_row, show_last_row);   // paint down to scroll_row
      }
      if (up_dn == down) {  // towards first line painted, so lines move down
          scroll_row++; 
          scroll_row = constrain(scroll_row, scr_h_ch, row);       
          scroll_display = false; 
          paintLogScreen(scroll_row, show_last_row);  // paint down to scroll_row      
      }   
    }
  #endif      
  //=========================================  
  #if defined DISPLAY_PRESENT  
    void handleDisplayButtons() 
    {
      if (millis() - last_log_millis > 15000) { // after 15 seconds default to flight info screen
        last_log_millis = millis();             // and enable toggle button again
        show_log = false;
      }
        
      if (show_log) {
        if (infoNewPress) {     
          paintLogScreen(row, show_last_row);  // one time     
          infoNewPress = false; 
          last_log_millis = millis();
        }
      } else {            // else show flight info
        displayFlightInfo();             
      }
      
      //log.printf("busy=%d  new=%d log=%d  bounce=%d  info=%d\n", infoPressBusy, infoNewPress, show_log, info_debounce_millis, info_millis); 
      
     #if ((defined ESP32) || (defined ESP8266))   // Teensy does not have touch pins          
      if ( (Tup != -1) && (Tdn != -1) ) {         // if ESP touch pin-pair enumerated
        if (up_button) {
          scrollDisplay(up);
        }
        if (scroll_display) {     
          scrollDisplay(down);
        }     
      } else
      #endif
      
      #if ( (Pup != -1) && (Pdn != -1) )    // if digital pin-pair enumerated
        if (up_button) {                 
          scrollDisplay(up);
        }
        if (scroll_display) {
          scrollDisplay(down);
        }        
      #endif       
    }
    #endif
    //=================================== 
    // only for touch buttons 
  #if defined DISPLAY_PRESENT      
    #if ((defined ESP32) || (defined ESP8266)) 
    void IRAM_ATTR gotButtonUp(){
      up_button = true;
    }
    void IRAM_ATTR gotButtonDn()
    {
      scroll_display = true;  
    }
    #endif 
  #endif       
    //=================================================================== 
  #if (defined DISPLAY_PRESENT) && (defined ILI9341_DISPLAY)
    uint32_t draw_horizon(float roll, float pitch, int16_t width, int16_t height) 
    {
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
    #if  (defined ILI9341_DISPLAY)
    void draw_hud_arrow(int16_t x, int16_t y, int16_t home_angle, int16_t width, int16_t height) 
    {
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
   uint16_t wrap360(int16_t ang) 
   {
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
  float applyCompassAlignment(float hdg , uint8_t rotation) 
  {
    float res = 0.0;
    switch (rotation) 
    {
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
    if (res > 360.0) 
    {
      res = res - 360.0;
    }
    return  res;
  }

//===========================================================================================
void displayHeadingSource(uint8_t hs) 
{
//#if defined DEBUG_MINIMUM || defined DEBUG_ALL || defined DEBUG_BOXCOMPASS  
   
  if (hs == 1)  {
      log.printf("headingsource:%u FC GPS\n", hs); 
      logScreenPrintln("HdgSrce=FC GPS");
  }
  else if  (hs == 2) 
  { 
      log.printf("headingsource:%u FC Compass\n", hs);    
      logScreenPrintln("HdgSrce=FC Mag");
  }
  else if (hs == 3)   
  {
      log.printf("headingsource:%u Tracker Box Compass\n", hs);    
      logScreenPrintln("HdgSrce=Trackr Cmpss");
  }
  else if (hs == 4)  
  {
      log.printf("Dynamic heading source:%u Tracker Box Compass and GPS\n", headingsource); 
      logScreenPrintln("Dynamic Headg+GPS");
  }  
//#endif  
}
#if ((PROTOCOL == 8) || (PROTOCOL == 0)) || (HEADINGSOURCE == 4) // NMEA GPS or TrackerBox GPS
  //====================================================
  uint8_t daysInMonth(uint8_t mth, uint8_t yr ) {
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
      epoch += (daysInMonth(dt.mm, dt.yyyy) * 86400L);
    }    
    epoch += (dt.dd * 86400L);
    epoch += (dt.h * 3600);
    epoch += (dt.m * 60);
    epoch += dt.s;
    return epoch;
  }  
#endif
