#if (Telemetry_In == 0)    //  Serial
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
uint32_t fr_gps_status;
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
// 0x800 GPS
uint32_t fr_latlong;
uint32_t fr_velyaw;
short ms2bits;

uint32_t fr_heading;
uint32_t fr_altitude;
uint32_t fr_home;
uint16_t fr_home_dist;
float fHomeDist;
short fr_pwr;
uint32_t fr_gps;
uint16_t fr_numsats;
uint8_t fr_gpsStatus;
uint8_t fr_hdop;
uint8_t fr_vdop;
uint8_t neg;
uint8_t gpsAlt;

void FrSky_Receive(){  
  if (FT) {                  // Sync to the first start/stop character
    chr = NextChar();
    while (!(chr==0x7E)) {
      chr = NextChar();
     }
    FT=false;
  }
  // Candidate found 
  
  packetBuffer[0]=chr;            // Start-Stop character 0x7E
  packetBuffer[1]=NextChar();     // Sensor-ID
  
  chr=NextChar();     // Start-Stop or Data-Frame Header
  
  if (chr==0x10) {    // If data frame header
    packetBuffer[2]=chr;
    boolean goodPacket=ParseData();
    if (goodPacket) ProcessData();
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
    //CheckForTimeouts();
    iLth=inSerial.available();
  }
  // Data is available
  hbGood = true;                     // We have a good serial connection!
  x =inSerial.read();

  return x;
}
//***************************************************
boolean ParseData() {
 crc=0;
  Add_Crc(packetBuffer[2]);           // data frame char into crc
 
  for (int i=3; i<=8; i++) {
    chr = NextChar();
    packetBuffer[i]=chr;
    Add_Crc(chr);
  }
  chr=NextChar(); 
  packetBuffer[9]=chr;  //  crc

  if (chr==(0xFF-crc)){
    crc_bad = false; 
 //  Debug.println("CRC Good");
  }
  else {
    crc_bad=true;
//   Debug.println("CRC Bad");
  }
  return !crc_bad;
} 
//***************************************************
void ProcessData() {
  // Do the sensor packets according to value type
 uint16_t ValueType = Unpack_uint16(3);

      switch(ValueType) {
                   // *****************************************************************
                //   Old D Style Hub/legacy protocol below 
                  case 0x01:                         // GPS Alt BP
                    if (!d_dia) {
                      d_dia=true;
                         OledPrintln("D dialect"); 
                    }           
                    cur.alt = Unpack_uint16(5);
                    if (!(cur.alt==0.0000)) {
                      altGood=true; 
                      new_GPS_data = true;     
                    }
                    #if defined Debug_All || defined Debug_FrSky              
                      Debug.print(" GPS Altitude 0x01=");
                      Debug.println(cur.hdg,0);
                    #endif
                    break;
                  case 0x12:                        // Lon BP - before point
                    lonDDMM = Unpack_uint32(5);
                    #if defined Debug_All || defined Debug_FrSky              
                      Debug.print(" lonDDMM 0x12=");
                      Debug.println(lonDDMM);
                    #endif             
                    break;
                  case 0x13:                       // Lat BP
                    latDDMM = Unpack_uint32(5);
                    #if defined Debug_All || defined Debug_FrSky              
                      Debug.print(" latDDMM 0x13=");
                      Debug.println(latDDMM);
                    #endif           
                    break;
                  case 0x14:        
                    cur.hdg = Unpack_uint16(5);      // Course / Heading BP
                    if (!(cur.hdg==0.000)) hdgGood=true;
                    #if defined Debug_All || defined Debug_FrSky              
                      Debug.print(" Heading 0x14=");
                      Debug.println(cur.hdg,0);
                    #endif
                    break;               
                  case 0x1A:                      // Lon AP
                    mmmm = Unpack_uint32(5);
                    DD = lonDDMM/100;
                    MM = lonDDMM -(DD*100);
                    MMmmmm = MM + (mmmm/1E4);       
                    cur.lon = DD + (MMmmmm/60);
                    if (EW==0x57)  cur.lon = 0-cur.lon; //  "W", as opposed to "E"
                    lonGood=true;
                    new_GPS_data = true;  
                    #if defined Debug_All || defined Debug_FrSky              
                      Debug.print(" Lon After Point 0x1A=");
                      Debug.println(cur.lon,0);
                    #endif
                     
                    break;
                  case 0x1B:                      // Lat AP
                    mmmm = Unpack_uint32(5);
                    DD = latDDMM/100;
                    MM = latDDMM -(DD*100);
                    MMmmmm = MM + (mmmm/1E4);
                    cur.lat = DD + (MMmmmm/60);     
                    if (NS==0x53) cur.lat = 0-cur.lat;  //  "S", as opposed to "N" 
                    latGood=true;
                    new_GPS_data = true;
                    #if defined Debug_All || defined Debug_FrSky              
                      Debug.print(" Lat After Point 0x1B=");
                      Debug.println(cur.lat,0);
                    #endif
                    break;
                  case 0x22:                      // Lon E/W
                    EW = Unpack_uint8(5);
                    #if defined Debug_All || defined Debug_FrSky              
                      Debug.print(" Lon E/W 0x22=");
                      Debug.println(EW);
                    #endif
                    break;
                  case 0x23:                      // Lat N/S
                    NS = Unpack_uint8(5);  
                    #if defined Debug_All || defined Debug_FrSky              
                      Debug.print(" Lon Lat N/S 0x23=");
                      Debug.println(NS);
                    #endif
                    break;

                    
                // *****************************************************************
                //   Regular S.Port X Protocol below    
                
                  case 0x100:              // Altitude
                    if (!x_dia) {
                      x_dia=true;
                         OledPrintln("X dialect"); 
                    }
                    fr_altitude= Unpack_uint32(5);
                    cur.alt  = fr_altitude / 100;
                    if (!(cur.alt ==0)) {
                      altGood=true; 
                      new_GPS_data = true;
                    }

                    break; 
                  case 0x410:              // Tmp2 - iNav GPS status 
                    iNav=true;
                    fr_gps_status= Unpack_uint32(5);
                    
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
                    
                    #if defined Debug_All || defined Debug_FrSky 
                      Debug.print("fr_gp_fix="); Serial.print(fr_gps_fix);     
                      Debug.print(" fr_gps_homefix ="); Debug.print(fr_gps_homefix);
                      Debug.print(" fr_gp_homereset="); Debug.print(fr_gps_homereset);     
                      Debug.print(" fr_gps_accuracy ="); Debug.print(fr_gps_accuracy);
                      Debug.print(" fr_gps_numsats="); Debug.println(fr_gps_numsats); 
                    #endif  
                    break;                    

                 case 0x800:                      // Latitude and Longitude
                   fr_latlong= Unpack_uint32(5);
                   #if defined Debug_All || defined Debug_FrSky  
                     Debug.print(" latlong=");
                     Debug.println(fr_latlong);
                   #endif   
                   ms2bits = fr_latlong >> 30;
                   fr_latlong = fr_latlong & 0x3fffffff; // remove ms2bits
                   #if defined Debug_All     
                     Debug.print(" ms2bits=");
                     Debug.println(ms2bits);
                   #endif   
                   switch(ms2bits) {
                     case 0:   // Latitude Positive
                       cur.lat = fr_latlong / 6E5;     // lon always comes first                      
                       #if defined Debug_All || defined Debug_FrSky                 
                         Debug.print(" 0x800 latitude=");
                         Debug.println(cur.lat,7);
                       #endif
                       latGood=true;
                       new_GPS_data = true; 
                       break;
                     case 1:   // Latitude Negative       
                       cur.lat = 0-(fr_latlong / 6E5); 
                       #if defined Debug_All || defined Debug_FrSky            
                         Debug.print(" 0x800 latitude=");
                         Debug.println(cur.lat,7);  
                       #endif   

                       if (!(cur.lat==0.000000) && !(cur.lon==0.000000)){
                         latGood=true;
                         new_GPS_data = true;                        
                       }
                       break;
                     case 2:   // Longitude Positive
                       cur.lon = fr_latlong / 6E5;                    
                       #if defined Debug_All || defined Debug_FrSky    
                         Debug.print(" 0x800 longitude=");
                         Debug.println(cur.lon,7); 
                       #endif                       
                       lonGood=true;
                       new_GPS_data = true;                         
                       break;
                     case 3:   // Longitude Negative
                       cur.lon = 0-(fr_latlong / 6E5);  
                       #if defined Debug_All                        
                         Debug.print(" 0x800 longitude=");
                         Debug.println(cur.lon,7); 
                       #endif                   
                       lonGood=true;
                       new_GPS_data = true;                       
                       break;
                    }
                    break;
                  case 0x820:              // Altitude
                    fr_altitude= Unpack_uint32(5);
                    cur.alt  = fr_altitude / 100;
                    if (!(cur.alt ==0.0000)){
                      altGood=true; 
                      new_GPS_data = true;
                    }
                    #if defined Debug_All || defined Debug_FrSky    
                       Debug.print(" 0x820 altitude=");
                       Debug.println(cur.alt,1); 
                     #endif    
                    
                    break;          
                  case 0x840:              // Heading
                    fr_heading= Unpack_uint32(5);
                    cur.hdg = fr_heading / 100;
                    if (!(cur.hdg==0.0000)) hdgGood=true;
                    #if defined Debug_All || defined Debug_FrSky    
                       Debug.print(" 0x840 heading=");
                       Debug.println(cur.hdg,1); 
                     #endif               
                    break;

                 // *****************************************************************    
                 //   Mavlink Passthrough Protocol below     
                  case 0x5002:
                  
                  // GPS Status &  gpsAlt
                    Passthru=true;
                    if (!pt_dia) {
                      pt_dia=true;
                         OledPrintln("Passthru dialect"); 
                    }

                    fr_gps = Unpack_uint32(5);
                    fr_numsats = bit32Extract(fr_gps, 0, 4);
                    fr_gpsStatus = bit32Extract(fr_gps, 4, 2) + bit32Extract(fr_gps, 14, 2);
                    fr_hdop = bit32Extract(fr_gps, 7, 7) * (10^bit32Extract(fr_gps, 6, 1));
                    gpsAlt = bit32Extract(fr_gps, 24, 7) * (10^bit32Extract(fr_gps, 22, 2));
                    cur.alt  = (float)(gpsAlt) / 10;
                    neg = bit32Extract(fr_gps, 31, 1);
                    if (neg==1) cur.alt = 0 - cur.alt;
                    new_GPS_data = true;
                    hdopGood=(fr_hdop>=3) && (fr_numsats>10);
                    #if defined Debug_All || defined Debug_FrSky 
                      Debug.print(" 0x5002 Num sats=");
                      Debug.print(fr_numsats);
                      Debug.print(" gpsStatus=");
                      Debug.print(fr_gpsStatus);                
                      Debug.print(" HDOP=");
                      Debug.print(fr_hdop);
                      Debug.print(" fr_vdop=");
                      Debug.print(fr_vdop);                     
                      Debug.print(" gpsAlt=");
                      Debug.print(cur.alt, 1);
                      Debug.print(" neg=");
                      Debug.println(neg);   
                    #endif

                    break;
                  case 0x5004:                         // Home
                    fr_home = Unpack_uint32(5);
                    fr_home_dist = bit32Extract(fr_home, 2, 10) * (10^bit32Extract(fr_home, 0, 2));
                    fHomeDist = (float)fr_home_dist * 0.1;  // Not used here 
                    cur.alt = bit32Extract(fr_home, 14, 10) * (10^bit32Extract(fr_home, 12, 2)) * 0.01; // metres
                    if (bit32Extract(fr_home,24,1) == 1) 
                      cur.alt = cur.alt * -1;
                    altGood=true; 
                    #if defined Debug_All || defined Debug_FrSky 
                      Debug.print(" 0x5004 Dist to home=");
                      Debug.print(fHomeDist, 1);             
                      Debug.print(" Rel Alt=");
                      Debug.println(cur.alt,1);
                    #endif
                    break;
                      
                  case 0x5005:                      
                  // Vert and Horiz Velocity and Yaw angle (Heading)
                    fr_velyaw = Unpack_uint32(5);      
                    fr_velyaw = fr_home_dist = bit32Extract(fr_velyaw, 16, 11);
                    cur.hdg = fr_velyaw/10;
      
                    hdgGood=true;
                    #if defined Debug_All || defined Debug_FrSky 
                      Debug.print(" 0x5005 Heading=");
                      Debug.println(cur.hdg,2);
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
//***************************************************
  uint32_t bit32Extract(uint32_t dword,uint8_t displ, uint8_t lth) {
  uint32_t r = (dword & createMask(displ,(displ+lth-1))) >> displ;
//  Debug.print(" Result=");
 // Debug.println(r);
  return r;
}
uint32_t createMask(uint8_t lo, uint8_t hi) {
  uint32_t r = 0;
  for (unsigned i=lo; i<=hi; i++)
       r |= 1 << i;
//  Debug.print(" Mask 0x=");
//  Debug.println(r, HEX);      
  return r;
}  
//***************************************************

uint32_t Unpack_uint32 (int posn){
  
    //  The number starts at byte "posn" of the received packet and is four bytes long.
    //  GPS payload fields are little-endian, i.e they need an end-to-end byte swap
    
   byte b1 = packetBuffer[posn+3];
   byte b2 = packetBuffer[posn+2];
   byte b3 = packetBuffer[posn+1];
   byte b4 = packetBuffer[posn]; 
   
   unsigned long highWord = b1 << 8 | b2;
   unsigned long lowWord  = b3 << 8 | b4;
    
    // Now combine the four bytes into an unsigned 32bit integer

   uint32_t myvar = highWord << 16 | lowWord;
   return myvar;
}
//***************************************************
int32_t Unpack_int32 (int posn){
  
    //  The number starts at byte "posn" of the received packet and is four bytes long.
    //  GPS payload fields are little-endian, i.e they need an end-to-end byte swap
    
   byte b1 = packetBuffer[posn+3];
   byte b2 = packetBuffer[posn+2];
   byte b3 = packetBuffer[posn+1];
   byte b4 = packetBuffer[posn]; 
   
   unsigned long highWord = b1 << 8 | b2;
   unsigned long lowWord  = b3 << 8 | b4;
   
 // Now combine the four bytes into an unsigned 32bit integer
 
   int32_t myvar = highWord << 16 | lowWord;
   return myvar;
}
//***************************************************
uint16_t Unpack_uint16 (int posn){
  
    //  The number starts at byte "posn" of the received packet and is two bytes long
    //  GPS payload fields are little-endian, i.e they need an end-to-end byte swap

   byte b1 = packetBuffer[posn+1];
   byte b2 = packetBuffer[posn];  
    
    // Now convert the 2 bytes into an unsigned 16bit integer
    
    uint16_t myvar = b1 << 8 | b2;
    return myvar;
}
//***************************************************
int16_t Unpack_int16 (int posn){
  
    //  The number starts at byte "posn" of the received packet and is two bytes long
    //  GPS payload fields are little-endian, i.e they need an end-to-end byte swap
   byte b1 = packetBuffer[posn+1];
   byte b2 = packetBuffer[posn];
    
    // Now convert the 2 bytes into a signed 16bit integer
    
    int16_t myvar = b1 << 8 | b2;
    return myvar;
}
//***************************************************
uint8_t Unpack_uint8 (int posn){
  
    //  The number starts at byte "posn" of the received packet and is one byte long

  byte b1 = packetBuffer[posn];
    
    // Now convert the byte into an unsigned 8 bit integer
    
   uint8_t myvar = b1;
   return myvar;
}
//***************************************************
int8_t Unpack_int8 (int posn){
  
    //  The number starts at byte "posn" of the received packet and is one byte long

  byte b1 = packetBuffer[posn];
    
    // Now convert the byte into an unsigned 8 bit integer
    
   int8_t myvar = b1;
   return myvar;
}
//***************************************************
#endif
