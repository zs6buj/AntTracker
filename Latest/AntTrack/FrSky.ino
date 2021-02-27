    // 0x800 GPS
    uint32_t fr_latlong;
    int32_t  fr_lat = 0;
    int32_t  fr_lon = 0;
    uint32_t fr_velyaw;

    //FrSky Variables
    short ms2bits;
    uint32_t fr_payload;

    // 0x5002 GPS Status
    uint8_t fr_numsats;
    uint8_t fr_gps_status;           // part a
    uint8_t fr_gps_adv_status;       // part b
    uint16_t fr_hdop;                // @rotorman  2021/01/18
    int32_t fr_amsl;                // decimetres
    float   fr_famsl;               // float decimetres
    uint8_t neg;
    
    //0x5003 Batt
    int16_t fr_bat1_volts;     // dV (V * 10)
    int16_t fr_bat1_amps;      // dA )A * 10)
    uint16_t fr_bat1_mAh;
    
    //0x5008 Batt    
    uint16_t fr_bat2_mAh;
    
    //0xF101 RSSI 
    uint32_t fr_rssi; 


#if (Telemetry_In == 0) || (Telemetry_In == 3)   //  Serial or UDP

bool lonGood = false;
bool latGood = false;
bool altGood = false;
bool hdgGood = false;
bool hdopGood = false;

bool Passthru = false;
bool d_dia = false;
bool x_dia = false;
bool pt_dia = false;

bool iNav = false;

// General FrSky
boolean FT = true;

short crc=0;  
boolean crc_bad; 

//    0x410 iNav GPS Status
//uint32_t fr_gps_status;
uint8_t fr_gps_fix;
uint8_t fr_gps_homefix;
uint8_t fr_gps_homereset;
uint8_t fr_gps_accuracy;     // 0 thru 9 highest accuracy
uint16_t fr_gps_numsats;

// variables for iNav 0x410 decode
uint8_t  d1;
uint16_t dr1;
uint8_t  d2;
uint16_t dr2;
uint8_t  d3;
uint16_t dr3;
uint8_t  d4;
  
uint8_t d14;      // say 7 / 4 = 1
uint8_t dr12;     //     7 % 4 = 3
uint8_t d12;      //     3 / 2 = 1
uint8_t d11;      //     3 % 2 = 1
//.....................  
uint16_t lonDDMM;
uint16_t latDDMM;
uint16_t DD;
uint16_t MM;
uint16_t mmmm;
float MMmmmm;
char NS;   // No kidding!
char EW;  

  
    
uint32_t fr_heading;
uint32_t fr_altitude;
uint32_t fr_home;
uint16_t fr_home_dist;
float fHomeDist;
short fr_pwr;
uint32_t fr_gps;

uint8_t gpsAlt;

uint8_t fr_offset = 0;
//=======================================================================
void FrSky_Receive() {
  #if (Telemetry_In == 0)         //   FrSky Serial
    Frs_Receive_Serial();
  #endif
   
  #if (Telemetry_In == 3)         //   FrSky UDP 
    Frs_Receive_UDP();
  #endif
}
//=======================================================================
#if (Telemetry_In == 3)
void Frs_Receive_UDP() {  

  uint8_t len = frs_udp_object.parsePacket();   // packet to in buffer
  if (len == 0) return;
  for (int i = 0 ; i < len ; i++) {
    inBuf[i] = frs_udp_object.read();
    //Log.printf("byte:%X  i:%d\n", inBuf[i], i);
  }
  
  //Log.print("A " );  DisplayFrsBuffer(&inBuf[0], len-fr_offset); 
    
  if (len == 12) { // S.Port (0x7E + 0x1B) + (0x7E + 0x1B) + frame
    fr_offset = 4;
  } else
  if (len == 10) {  // S.Port 0x7E + 0x1B) + frame
    fr_offset = 2;
  } else
  if (len == 8) {  // F.Port frame
    fr_offset = 0;
  }
  //Log.printf("B ");  DisplayFrsBuffer(&inBuf[fr_offset], len-fr_offset);
  bool mycrcGood = crcGood(&inBuf[fr_offset], len-fr_offset); 

  if (mycrcGood) {  
    frGood = true;
    frGood_millis = millis();  
    #if defined Debug_All || defined Debug_FrSky_Messages_UDP
      Log.print("CRC Good C "); DisplayFrsBuffer(&inBuf[fr_offset], len-fr_offset);
    #endif 
    Frs_Decode(&inBuf[fr_offset]);   
  }
}
#endif
//=======================================================================
#if (Telemetry_In == 3)

    void crcEnd(int16_t *mycrc)  {
      *mycrc = 0xFF - *mycrc;                  // final 2s complement
      #if defined Debug_CRC
        Log.printf("crcEnd=%3X %3d\n", *mycrc, *mycrc );
      #endif  
    } 
    //=======================================================================  
    
    void crcStep(int16_t *mycrc, uint8_t b) {
       *mycrc += b;             // add in new byte
       *mycrc += *mycrc >> 8;   // add in high byte carry if any
       *mycrc &= 0xff;          // mask all but low byte, constrain to 8 bits

      #if defined Debug_CRC
         Log.printf("CRC Step: b=%3X %3d\  crc=%3X %3d\n", b, b, *mycrc, *mycrc);
      #endif
    }    
    //=======================================================================   
       
    uint8_t crcGet(uint8_t *buf, uint8_t lth)  {

      int16_t mycrc = 0;
      for (int i = 0; i < lth; i++) {
        crcStep(&mycrc, *buf++);
      }
      crcEnd(&mycrc);
      return mycrc;
    }
    //=======================================================================  
    bool crcGood(uint8_t *buf, uint8_t lth)  {
      
      uint8_t mycrc = crcGet(buf, lth);   
      uint8_t fpcrc = *(buf+lth);
      #if defined Debug_CRC    
        Log.printf("mycrc=%3X %3d  fpcrc=%3X\ %3d\n", mycrc, mycrc, fpcrc, fpcrc );
      #endif
    return (mycrc == fpcrc);

   }  

#endif
//================================================================
#if (Telemetry_In == 0)
void Frs_Receive_Serial(){    
  if (FT) {                  // Sync to the first start/stop character
    chr = NextChar();
    while (!(chr==0x7E)) {
      chr = NextChar();
     }
    FT=false;
  }
  // Candidate found 
  
  inBuf[0]=chr;            // Start-Stop character 0x7E
  inBuf[1]=NextChar();     // Sensor-ID
  
  chr=NextChar();     // Start-Stop or Data-Frame Header
  
  if (chr==0x10) {    // If data frame header
    inBuf[2]=chr;
    boolean goodPacket=serialParse();
    if (goodPacket) Frs_Decode(&inBuf[2]);
    #if defined Debug_All || defined Debug_SPort
      DisplayTheBuffer(10);
    #endif 
    chr=NextChar();   //  Should be the next Start-Stop  
    }
  else {
    #if defined Debug_All || defined Debug_SPort
      DisplayTheBuffer(2);
    #endif  
    }
    
  if (!(chr==0x7E)) FT=true;  //  If next char is not start-stop then the frame sync has been lost. Resync.
}
//***************************************************
byte NextChar() {
byte x;

  iLth=inSerial.available();     //   wait for more data
  while (iLth==0) {
    CheckForTimeouts();
    iLth=inSerial.available();
  }
  // Data is available
  hbGood = true;                     // We have a good serial connection!
  x =inSerial.read();

  return x;
}
//***************************************************
boolean serialParse() {
 crc=0;
  Add_Crc(inBuf[2]);           // data frame char into crc
 
  for (int i=3; i<=8; i++) {
    chr = NextChar();
    inBuf[i]=chr;
    Add_Crc(chr);
  }
  chr=NextChar(); 
  inBuf[9]=chr;  //  crc

  if (chr==(0xFF-crc)){
    crc_bad = false; 
 //  Log.println("CRC Good");
  }
  else {
    crc_bad=true;
//   Log.println("CRC Bad");
  }
  return !crc_bad;
} 
#endif
//***************************************************
void Frs_Decode(uint8_t *buf) {
  // Do the sensor packets according to value type
 uint16_t appID = uint16Extract(buf, 1 );
     fr_payload = uint32Extract(buf, 3);
      //Log.printf("appID:%4X\n", appID);
      switch(appID) {
                   // *****************************************************************
                //   Old D Style Hub/legacy protocol below 
                  case 0x01:                         // GPS Alt BP
                    if (!d_dia) {
                      d_dia=true;
                         LogScreenPrintln("D dialect"); 
                    }           
                    cur.alt = uint16Extract(buf, 3);
                    if (!(cur.alt==0.0000)) {
                      altGood=true; 
                      new_GPS_data = true;     
                    }
                    #if defined Debug_All || defined Print_Decoded_FrSky              
                      Log.print(" GPS Altitude 0x01=");
                      Log.println(cur.hdg,0);
                    #endif
                    break;
                  case 0x12:                        // Lon BP - before point
                    lonDDMM = uint32Extract(buf, 3);
                    #if defined Debug_All || defined Print_Decoded_FrSky              
                      Log.print(" lonDDMM 0x12=");
                      Log.println(lonDDMM);
                    #endif             
                    break;
                  case 0x13:                       // Lat BP
                    latDDMM = uint32Extract(buf, 3);
                    #if defined Debug_All || defined Print_Decoded_FrSky              
                      Log.print(" latDDMM 0x13=");
                      Log.println(latDDMM);
                    #endif           
                    break;
                  case 0x14:        
                    cur.hdg = uint16Extract(buf, 3);      // Course / Heading BP
                    if (!(cur.hdg==0.000)) hdgGood=true;
                    #if defined Debug_All || defined Print_Decoded_FrSky              
                      Log.print(" Heading 0x14=");
                      Log.println(cur.hdg,0);
                    #endif
                    break;               
                  case 0x1A:                      // Lon AP
                    mmmm = uint32Extract(buf, 3);
                    DD = lonDDMM/100;
                    MM = lonDDMM -(DD*100);
                    MMmmmm = MM + (mmmm/1E4);       
                    cur.lon = DD + (MMmmmm/60);
                    if (EW==0x57)  cur.lon = 0-cur.lon; //  "W", as opposed to "E"
                    lonGood=true;
                    new_GPS_data = true;  
                    #if defined Debug_All || defined Print_Decoded_FrSky              
                      Log.print(" Lon After Point 0x1A=");
                      Log.println(cur.lon,0);
                    #endif
                     
                    break;
                  case 0x1B:                      // Lat AP
                    mmmm = uint32Extract(buf, 3);
                    DD = latDDMM/100;
                    MM = latDDMM -(DD*100);
                    MMmmmm = MM + (mmmm/1E4);
                    cur.lat = DD + (MMmmmm/60);     
                    if (NS==0x53) cur.lat = 0-cur.lat;  //  "S", as opposed to "N" 
                    latGood=true;
                    new_GPS_data = true;
                    #if defined Debug_All || defined Print_Decoded_FrSky              
                      Log.print(" Lat After Point 0x1B=");
                      Log.println(cur.lat,0);
                    #endif
                    break;
                  case 0x22:                      // Lon E/W
                    EW = uint8Extract(buf, 3);
                    #if defined Debug_All || defined Print_Decoded_FrSky              
                      Log.print(" Lon E/W 0x22=");
                      Log.println(EW);
                    #endif
                    break;
                  case 0x23:                      // Lat N/S
                    NS = uint8Extract(buf, 3);  
                    #if defined Debug_All || defined Print_Decoded_FrSky              
                      Log.print(" Lon Lat N/S 0x23=");
                      Log.println(NS);
                    #endif
                    break;

                    
                // *****************************************************************
                //   Regular S.Port X Protocol below    
                
                  case 0x100:              // Altitude
                    if (!x_dia) {
                      x_dia=true;
                         LogScreenPrintln("X dialect"); 
                    }
                    fr_altitude= uint32Extract(buf, 3);
                    cur.alt  = fr_altitude / 100;
                    if (!(cur.alt ==0)) {
                      altGood=true; 
                      new_GPS_data = true;
                    }

                    break; 
                  case 0x410:              // Tmp2 - iNav GPS status 
                    iNav=true;
                    fr_gps_status= uint32Extract(buf, 3);
                    
                    // decode to digits 1 thru 4
                    d1 = (fr_gps_status / 1000);
                    dr1 = d1 * 1000;
                    d2 = (fr_gps_status - dr1) / 100;
                    dr2 = dr1 + (d2 * 100);
                    d3 = (fr_gps_status - dr2) / 10;
                    dr3 = dr2 + (d3 * 10);
                    d4 = fr_gps_status - dr3;
                    
                    // decode to sub-digits of d1
                    d14 = d1 / 4;      // say 7 / 4 = 1
                    dr12 = d1 % 4;     //     7 % 4 = 3
                    d12 = dr12 / 2;    //     3 / 2 = 1
                    d11 = dr12 % 2;    //     3 % 2 = 1

                    fr_gps_fix = d11;
                    fr_gps_homefix = d12;
                    fr_gps_homereset = d14;
                    fr_gps_accuracy = d2;   // 0 thru 9 highest accuracy
                    fr_gps_numsats = (d3*10) + d4;

                    hdopGood = (fr_gps_accuracy > 7);  // 0 thru 9 - 9 best
                    
                    #if defined Debug_All || defined Print_Decoded_FrSky 
                      Log.print("fr_gp_fix="); Serial.print(fr_gps_fix);     
                      Log.print(" fr_gps_homefix ="); Log.print(fr_gps_homefix);
                      Log.print(" fr_gp_homereset="); Log.print(fr_gps_homereset);     
                      Log.print(" fr_gps_accuracy ="); Log.print(fr_gps_accuracy);
                      Log.print(" fr_gps_numsats="); Log.println(fr_gps_numsats); 
                    #endif  
                    break;                    

                 case 0x800:                      // Latitude and Longitude
                 
                   fr_latlong= uint32Extract(buf, 3); 
                   ms2bits = fr_latlong >> 30;
                   fr_latlong = fr_latlong & 0x3fffffff; // remove ms2bits
                   #if defined Debug_All     
                     Log.print(" ms2bits=");
                     Log.println(ms2bits);
                   #endif   
                   switch(ms2bits) {
                     case 0:   // Latitude Positive
                       cur.lat = fr_lat = fr_latlong / 6E5;     // lon always comes first                      
                       #if defined Debug_All || defined Print_Decoded_FrSky                 
                         Log.print(" FrSky 0x800 latitude=");
                         Log.println(cur.lat,7);
                       #endif
                       latGood=true;
                       new_GPS_data = true; 
                       break;
                     case 1:   // Latitude Negative       
                       cur.lat = fr_lat = 0-(fr_latlong / 6E5); 
                       #if defined Debug_All || defined Print_Decoded_FrSky            
                         Log.print(" FrSky 0x800 latitude=");
                         Log.println(cur.lat,7);  
                       #endif   

                       if (!(cur.lat==0.000000) && !(cur.lon==0.000000)){
                         latGood=true;
                         new_GPS_data = true;                        
                       }
                       break;
                     case 2:   // Longitude Positive
                       cur.lon = fr_lon = fr_latlong / 6E5;                    
                       #if defined Debug_All || defined Print_Decoded_FrSky    
                         Log.print(" FrSky 0x800 longitude=");
                         Log.println(cur.lon,7); 
                       #endif                       
                       lonGood=true;
                       new_GPS_data = true;                         
                       break;
                     case 3:   // Longitude Negative
                       cur.lon = fr_lon = 0-(fr_latlong / 6E5);  
                       #if defined Debug_All                        
                         Log.print(" FrSky 0x800 longitude=");
                         Log.println(cur.lon,7); 
                       #endif                   
                       lonGood=true;
                       new_GPS_data = true;                       
                       break;
                    }
                    break;
                  case 0x820:              // Altitude
                    fr_altitude= uint32Extract(buf, 3);
                    cur.alt  = fr_altitude / 100;
                    if (!(cur.alt ==0.0000)){
                      altGood=true; 
                      new_GPS_data = true;
                    }
                    #if defined Debug_All || defined Print_Decoded_FrSky    
                       Log.print(" FrSky 0x820 altitude=");
                       Log.println(cur.alt,1); 
                     #endif    
                    
                    break;          
                  case 0x840:              // Heading
                    fr_heading= uint32Extract(buf, 3);
                    cur.hdg = fr_heading / 100;
                    if (!(cur.hdg==0.0000)) hdgGood=true;
                    #if defined Debug_All || defined Print_Decoded_FrSky    
                       Log.print(" FrSky 0x840 heading=");
                       Log.println(cur.hdg,1); 
                     #endif               
                    break;

  
                 //   Mavlink Passthrough Protocol below    
                 //===================================================================
                 case 0xF000:   
                   break;
                 case 0xF101:                        // RSSI  
                   fr_rssi = fr_payload;
                   #if defined Print_Decoded_FrSky
                     Log.print(" FrSky F101: RSSI=");
                     Log.println(fr_rssi);
                   #endif  
                   break;
                 case 0xF103:   
                   break;
                 case 0xF104:   
                   break;
                 case 0xF105:   
                   break;                 
                  case 0x5002:
                  
                  // GPS Status &  gpsAlt
                    Passthru=true;
                    if (!pt_dia) {
                      pt_dia=true;
                      Log.println("Passthru dialect detected");                       
                      LogScreenPrintln("Passthru dialect"); 
                    }

                    fr_gps = uint32Extract(buf, 3);
                    fr_numsats = bit32Extract(fr_gps, 0, 4);
                    fr_gps_status = bit32Extract(fr_gps, 4, 2) + bit32Extract(fr_gps, 14, 2);
                    fr_hdop = bit32Extract(fr_gps, 7, 7) * TenToPwr(bit32Extract(fr_gps, 6, 1));  
                    gpsAlt = bit32Extract(fr_gps,24,7) * TenToPwr(bit32Extract(fr_gps,22,2)); //-- dm                                    
                    cur.alt  = (float)(gpsAlt) / 10;
                    neg = bit32Extract(fr_gps, 31, 1);
                    if (neg==1) cur.alt = 0 - cur.alt;
                    new_GPS_data = true;
                    hdopGood=(fr_hdop>=3) && (fr_numsats>10);
                    #if defined Debug_All || defined Print_Decoded_FrSky 
                      Log.print(" FrSky 0x5002 Num sats=");
                      Log.print(fr_numsats);
                      Log.print(" gpsStatus=");
                      Log.print(fr_gps_status);                
                      Log.print(" HDOP=");
                      Log.print(fr_hdop);                    
                      Log.print(" gpsAlt=");
                      Log.print(cur.alt, 1);
                      Log.print(" neg=");
                      Log.println(neg);   
                    #endif

                    break;
                    
                  case 0x5003:                         // Battery 1 Hz
                   fr_bat1_volts = (float)(bit32Extract(fr_payload,0,9));  // dv -> V
                   //Log.printf("mantissa:%d  10exponent:%d mutiplier:%d \n", bit32Extract(fr_payload,10,7), bit32Extract(fr_payload,9,1), TenToPwr(bit32Extract(fr_payload,9,1)) );
                   fr_bat1_amps = (bit32Extract(fr_payload,10,7) * TenToPwr(bit32Extract(fr_payload,9,1) ) );  // rounded to nearest whole A
                   fr_bat1_mAh = bit32Extract(fr_payload,17,15);
                   fr_bat1_amps *= 10;  // prep_number() divided by 10 to get A, but we want dA for consistency
                   #if defined Print_Decoded_FrSky
                     Log.print(" FrSky 5003: Battery Volts=");
                     Log.print(fr_bat1_volts, 1);
                     Log.print("  Battery Amps=");
                     Log.print((float)fr_bat1_amps, 0);
                     Log.print("  Battery mAh=");
                     Log.println(fr_bat1_mAh); 
                   #endif       
                   break;                          
                   
                  case 0x5004:                         // Home
                    fr_home = uint32Extract(buf, 3);
                    fr_home_dist = bit32Extract(fr_home,2,10) * TenToPwr(bit32Extract(fr_home,0,2));                   
                    fHomeDist = (float)fr_home_dist * 0.1;  // Not used here 
                    cur.alt = bit32Extract(fr_home,14,10) * TenToPwr(bit32Extract(fr_home,12,2)); // decimetres
                    if (bit32Extract(fr_home,24,1) == 1) 
                      cur.alt = cur.alt * -1;
                    altGood=true; 
                    #if defined Debug_All || defined Print_Decoded_FrSky 
                      Log.print(" FrSky 0x5004 Dist to home=");
                      Log.print(fHomeDist, 1);             
                      Log.print(" Rel Alt=");
                      Log.println(cur.alt,1);
                    #endif
                    break;
                      
                  case 0x5005:                      
                  // Vert and Horiz Velocity and Yaw angle (Heading)
                    fr_velyaw = uint32Extract(buf, 3);      
                    fr_velyaw = fr_home_dist = bit32Extract(fr_velyaw, 16, 11);
                    cur.hdg = fr_velyaw/10;
      
                    hdgGood=true;
                    #if defined Debug_All || defined Print_Decoded_FrSky 
                      Log.print(" FrSky 0x5005 Heading=");
                      Log.println(cur.hdg,2);
                    #endif
                    break;   
               
                   
      }

    gpsGood = hdopGood & lonGood & latGood & altGood & hdgGood ; 
    
    if (Heading_Source==1 && (gpsGood) && (!homSaved)) AutoStoreHome();  // Only need this when Heading_Source is GPS 
}

//***************************************************
void Add_Crc (uint8_t byte) {
  crc += byte;       //0-1FF
  crc += crc >> 8;   //0-100
  crc &= 0x00ff;
  crc += crc >> 8;   //0-0FF
  crc &= 0x00ff;
  }
//=================================================================================================  
uint32_t TenToPwr(uint8_t pwr) {
  uint32_t ttp = 1;
  for (int i = 1 ; i<=pwr ; i++) {
    ttp*=10;
  }
  return ttp;
}  
//***************************************************
  uint32_t bit32Extract(uint32_t dword,uint8_t displ, uint8_t lth) {
  uint32_t r = (dword & createMask(displ,(displ+lth-1))) >> displ;
//  Log.print(" Result=");
 // Log.println(r);
  return r;
}
uint32_t createMask(uint8_t lo, uint8_t hi) {
  uint32_t r = 0;
  for (unsigned i=lo; i<=hi; i++)
       r |= 1 << i;
//  Log.print(" Mask 0x=");
//  Log.println(r, HEX);      
  return r;
}  
//***************************************************

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
//***************************************************
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
//***************************************************
uint16_t uint16Extract(uint8_t *buf, int posn){
  
    //  The number starts at byte "posn" of the received packet and is two bytes long
    //  GPS payload fields are little-endian, i.e they need an end-to-end byte swap

   byte b1 = buf[posn+1];
   byte b2 = buf[posn];  
    
    // Now convert the 2 bytes into an unsigned 16bit integer
    
    uint16_t myvar = b1 << 8 | b2;
    return myvar;
}
//***************************************************
int16_t int16Extract(uint8_t *buf, int posn){
  
    //  The number starts at byte "posn" of the received packet and is two bytes long
    //  GPS payload fields are little-endian, i.e they need an end-to-end byte swap
   byte b1 = buf[posn+1];
   byte b2 = buf[posn];
    
    // Now convert the 2 bytes into a signed 16bit integer
    
    int16_t myvar = b1 << 8 | b2;
    return myvar;
}
//***************************************************
uint8_t uint8Extract(uint8_t *buf, int posn){
  
    //  The number starts at byte "posn" of the received packet and is one byte long

  byte b1 = buf[posn];
    
    // Now convert the byte into an unsigned 8 bit integer
    
   uint8_t myvar = b1;
   return myvar;
}
//***************************************************
int8_t int8Extract(uint8_t *buf, int posn){
  
    //  The number starts at byte "posn" of the received packet and is one byte long

  byte b1 = buf[posn];
    
    // Now convert the byte into an unsigned 8 bit integer
    
   int8_t myvar = b1;
   return myvar;
}
//***************************************************
#endif
