  
/*

    ZS6BUJ's Antenna Tracker

     Universal Frsky serial telemetry input version - supports X, D and Mavlink Passthrough

     Eric Stockenstrom - First code June 2017
     

This application reads serial telemetry sent from a flight controller or GPS. The module calculates where an airbourne craft is relative to the home position. From this it calculates the azimuth and elevation
of the craft, and then positions 180 degree azimuth and elevation PWM controlled servos to point a small high-gain antenna for telemetry and video.

The code is written from scratch, but I've taken ideas from Jalves' OpenDIY-AT and others. Information and ideas on other protocols was obtained frpm
GhettoProxy by Guillaume S.

The target board is an STM32F103 "Blue Pill", chosen for its relative power, small size and second (multi) serial port(s) for debugging.

To use the module, position the AntTRacker with the antenna facing the direction of your planned take off. Position the craft a few metres 
further, also facing the same heading for take-off. The flight computer will determine the magnetic heading, from which all subsequent angles 
are calculated.

NOTE: In Mavlink Passthrough mode only, relative altitude is GPS (not barometer) derived, so altitude is inaccurate until at least 10 satelites are visible. 
      More is better.

1 Power up the craft.
2 Power up the ground Raspi board.
3 Power up the AntTracker.
4 When AntTracker successfully connects to the Taranis, the LED on the front flashes slowly.
5 When more than 10 GPS satellites are visible, and HDOP is 3 or greater, and AntTracker receives its first good GPS location record, the LED flashes fast.
6 Push the home button to register the home position and heading. The LED goes on solid.
7 Enjoy your flight! The Tracker will track your craft anywhere in the hemisphere around you (but not closer than 3 metres).

The small double bi-quad antenna has excellent gain, and works well with a vertically polarised stick on the craft. Other reception sticks 
can be added for diversity, and some improvement in link resilience has been observed despite the lower gain of the other links. 
Of course it would be possible to stack double bi-quad antennas on the AntTracker, but more robust mechanicals will be called for.

v0.14 2017-05-22 Serial input version
v0.15 2017-05-30 Mod word length for 32bit MPUs like STM32
v0.18 2017-10-09 Frsky S.Port version
v0.19 2017-10-17 Include support for Frsky D hub protocol direct from Pixhawk/APM
v0.20 2017-10-20 Fix gps timeout check
v0.21 2017-11-04 Add Frsky Mavlink Passthrough
v0.22 2017-11-10 Tidy up after flight test
 */

#include <Servo.h>

boolean FT = true;
int iLth=0;
int pLth;
byte chr = 0x00;
const int packetSize = 70; 
byte packetBuffer[packetSize]; 
short crc=0;  
boolean crc_bad; 

//************* Pin Assignments
// BT Serial1 telemetry pins - RX = A3    TX = A2
// Serial for printout       - RX = A10   TX = A9

int azPWM_Pin =  7;    // A7 Azimuth
int elPWM_Pin =  8;    // A8 Elevation

int SetHomePin = 5;    // A5

int StatusLed = 6;  // A6 - Off=No good GPS yet, Flashing=Good GPS but Home not set yet, Solid = Ready to Track
int BoardLed = PC13;
int ledState = LOW; 
unsigned long ledMillis = 0;

//*************

boolean homeInitialised = false;
boolean serGood = false;
boolean lonGood = false;
boolean latGood = false;
boolean altGood = false;
boolean hdgGood = false;
boolean gpsGood = false;
boolean gpsGoodMsg = false;
boolean hdopGood = false;
boolean Passthrough = false;

//int gpsNumSats = 0;
unsigned long gpsMillis = 0;

//  variables for servos
int azPWM = 0;
int elPWM = 0;
int LastGoodpntAz = 90;
int LastGoodEl = 0;

float fLat = 0;
float fLon = 0;
float tLon = 0;
float fAlt = 0;
float fRelAlt =0;
float fvx = 0;
float fvy = 0;
float fvz = 0;
float fhdg = 0;

uint32_t fr_latlong;
uint32_t fr_velyaw;
short ms2bits;
uint32_t fr_heading;
uint32_t fr_altitude;
uint32_t fr_home;
uint16_t fr_home_dist;
short fr_pwr;
uint32_t fr_gps;
int fr_numsats;
short fr_hdop;
boolean neg;
boolean msl;

 int lonDDMM;
 int latDDMM;
 int DD;
 int MM;
 int mmmm;
 float MMmmmm;
 char NS;   // No kidding!
 char EW;

int MinDisplacement = 4;  // Distance from home before tracking starts

//AzEl declarations

float Azimuth= 90;                     
float Elevation= 0;                     
long Distance = 0;

// 3D Location vectors
struct Location {
  float lat; //long
  float lon;
  float alt;
};

struct Location home         = {
  0,0,0};   // home location

float homeHdg;

struct Location cur      = {
  0,0,0};   // current location

// Servo instances
Servo azServo;            // Azimuth
Servo elServo;            // Elevation

//***************************************************
void setup()
{

  Serial1.begin(57600);        // Telemetry input
  Serial.begin(115200);       // Print Output
  
  delay(2000);
  pinMode(SetHomePin, INPUT_PULLUP); 
  pinMode(StatusLed , OUTPUT ); 
  pinMode(BoardLed, OUTPUT);     // Board LED mimics status led
  digitalWrite(BoardLed, HIGH);  // Logic is reversed! Initialse off

  azServo.attach(azPWM_Pin);
  elServo.attach(elPWM_Pin);
  PositionServos(90, 0, 90);   // Intialise servos to az=90, el=0, homeHdg = 90;

 Serial.println("Starting up......");

  
// TestServos();   // Uncomment this code to observe how well your servos reach 0 deg and 180 deg
                 // Fine tune MaxPWM and MinPWM in Servos module
  
}
//***************************************************
//***************************************************
void loop()  {
  
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
   // DisplayTheBuffer(10); 
    chr=NextChar();   //  Should be the next Start-Stop  
    }
  //  else DisplayTheBuffer(2); 
 
  if (!(chr==0x7E)) FT=true;  //  If next char is not start-stop then the frame sync has been lost. Resync.

//  ++++++++++++++++++++

    if (Passthrough)
      gpsGood = hdopGood & lonGood & latGood & altGood & hdgGood ;
     else
      gpsGood = lonGood & latGood & altGood & hdgGood ;

    if (gpsGood) {
      gpsMillis = millis();                 // Time of last good GPS packet
      if (!gpsGoodMsg) {
        gpsGoodMsg = true;
        if (!homeInitialised)
          Serial.println("GPS lock good! Push set-home button anytime to start tracking.");
        else
          Serial.println("GPS lock good again!");
      }
      
//    if (homeInitialised && PacketGood()) {   // Resonability check not necessary because we have crc
      if (homeInitialised) {    
        GetAzEl(home.lat, home.lon, home.alt, cur.lat, cur.lon, cur.alt);
        if (Distance >= MinDisplacement) PositionServos(Azimuth, Elevation, homeHdg);
      }
  
    short SetHomeState = digitalRead(SetHomePin);
    if (SetHomeState==0 && !homeInitialised && gpsGood){   // Pin 5 is internally pulled up - normally high
 //   if (!homeInitialised && gpsGood){                    // Auto set home on first good GPS packet
      home.lat = fLat;
      home.lon = fLon;
      home.alt = fAlt;
      homeHdg = fhdg;
      homeInitialised = true;
      Serial.print("Home location set to Lat = ");
      Serial.print(fLat,7);
      Serial.print(" Lon = ");
      Serial.print(fLon,7);
      Serial.print(" Alt = ");
      Serial.print(fAlt,0); 
      Serial.print(" Hdg = ");
      Serial.print(fhdg,0); 
      Serial.println();
      }

    delay(10);
    }
}  
//***************************************************
//***************************************************
void Add_Crc (uint8_t byte) {
  crc += byte;       //0-1FF
  crc += crc >> 8;   //0-100
  crc &= 0x00ff;
  crc += crc >> 8;   //0-0FF
  crc &= 0x00ff;
  }
//***************************************************
byte NextChar() {
byte x;

  iLth=Serial1.available();     //   wait for more data
  while (iLth==0) {
    CheckForTimeouts();
    iLth=Serial1.available();
  }
  // Data is available
  serGood = true;                     // We have a good serial connection!
  x = Serial1.read();

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
 //  Serial.println("CRC Good");
  }
  else {
    crc_bad=true;
//   Serial.println("CRC Bad");
  }
  return !crc_bad;
} 
//***************************************************
void ProcessData() {
  // Do the sensor packets according to value type
 uint16_t ValueType = Unpack_uint16(3);

      switch(ValueType) {
                   // *****************************************************************
                //   Old D Style Hub Protocol below 
                  case 0x01:                         // GPS Alt BP
                    fAlt = Unpack_uint16(5);
                    if (!(fAlt==0)) altGood=true; 
                     cur.alt = fAlt;
                    //Serial.print(" GPS Altitude=");
                    //Serial.println(fAlt,0);
                    break;
                  case 0x12:                        // Lon BP
                    lonDDMM = Unpack_uint32(5);
                    break;
                  case 0x13:                       // Lat BP
                    latDDMM = Unpack_uint32(5);
                    break;
                  case 0x14:        
                    fhdg = Unpack_uint16(5);      // Course / Heading BP
                    if (!(fhdg==0)) hdgGood=true;
           //         Serial.print(" Heading=");
           //         Serial.println(fhdg,0);
                    break;               
                  case 0x1A:                      // Lon AP
                    mmmm = Unpack_uint32(5);
                    DD = lonDDMM/100;
                    MM = lonDDMM -(DD*100);
                    MMmmmm = MM + (mmmm/1E4);       
                    tLon = DD + (MMmmmm/60);
                    if (EW==0x57)  tLon = 0-tLon; //  "W", as opposed to "E"
                    // Store tLon and wait for lat to make matched pair    
                     if (!(fLon==0)) lonGood=true;
                    break;
                  case 0x1B:                      // Lat AP
                    mmmm = Unpack_uint32(5);
                    DD = latDDMM/100;
                    MM = latDDMM -(DD*100);
                    MMmmmm = MM + (mmmm/1E4);
                    fLat = DD + (MMmmmm/60);     
                    if (NS==0x53) fLat = 0-fLat;  //  "S", as opposed to "N" 
                    fLon = tLon;  // Complete the pair 
                    if (!(fLat==0) && !(fLon==0)) latGood=true;
                    cur.lat = fLat;
                    cur.lon = fLon;
                    /*
                    ShowElapsed();
                    Serial.print(" latitude=");
                    Serial.print(fLat,7);
                    Serial.print(" longitude=");
                    Serial.println(fLon,7);
                    */
                    break;
                  case 0x22:                      // Lon E/W
                    EW = Unpack_uint8(5);
                    break;
                  case 0x23:                      // Lat N/S
                    NS = Unpack_uint8(5);  
                    break;
                // *****************************************************************
                //   New S.Port Protocol below    
                 case 0x800:                      // Latitude and Longitude
                   fr_latlong= Unpack_uint32(5);
                   ms2bits = fr_latlong >> 30;
                   fr_latlong = fr_latlong & 0x3fffffff; // remove ms2bits
                   /*
                   Serial.print(" ms2bits=");
                   Serial.println(ms2bits);
                   */
                switch(ms2bits) {
                     case 0:   // Latitude Positive
                       fLat = fr_latlong / 6E5;     // Only ever update lon and lat in pairs. Lon always comes first                   
                         /*
                       Serial.print(" latitude=");
                       Serial.println(fLat,7);
                       */
                       fLon = tLon; 
                       // Update lon from temp lon below
                       cur.lat = fLat;
                       cur.lon = fLon;
                       if (!(fLat==0) && !(fLon==0)) latGood=true;
                       break;
                     case 1:   // Latitude Negative       
                       fLat = 0-(fr_latlong / 6E5);  
                       /*         
                       Serial.print(" latitude=");
                       Serial.println(fLat,7);  
                       */              
                       fLon = tLon;
                       cur.lat = fLat;
                       cur.lon = fLon;
                       if (!(fLat==0) && !(fLon==0)) latGood=true;
                       break;
                     case 2:   // Longitude Positive
                       tLon = fr_latlong / 6E5;                 
         //              Serial.print(" longitude=");
         //              Serial.println(fLon,7);                     
                         if (!(fLon==0)) lonGood=true;
                       break;
                     case 3:   // Longitude Negative
                       tLon = 0-(fr_latlong / 6E5);                 
       //                Serial.print(" longitude=");
       //                Serial.println(fLon,7);                  
                       lonGood=true;
                       break;
                     //
                    break;
                   }
                   break;
                  case 0x840:              // Heading
                    fr_heading= Unpack_uint32(5);
                    fhdg = fr_heading / 100;
                    if (!(fhdg==0)) hdgGood=true;
                    break;
                  case 0x100:              // Altitude
                    fr_altitude= Unpack_uint32(5);
                    fAlt = fr_altitude / 100;
                    cur.alt = fAlt;
                    if (!(fAlt==0)) altGood=true; 
                    msl = true;               // Tell the algorithms altitude is msl
                    break;  
                 // *****************************************************************    
                 //   Mavlink Passthrough Protocol below     
                  case 0x5002:                         // GPS Status & Alt msl (not used)
                    Passthrough=true;
                    uint8_t altmsl;
                    uint8_t vdil;
                    fr_gps = Unpack_uint32(5);
                    fr_numsats = (fr_gps & 0xf);
                    fr_hdop = (fr_gps & 0x30) >> 4;
                    vdil = (fr_gps & 0x3f800) >> 15;
                    neg = fr_home >> 31;
                    altmsl = (fr_gps & 0x7F000000) >> 24;
                    if (neg) altmsl = 0 - altmsl;
                    hdopGood=(fr_hdop>=3) && (fr_numsats>10);
                    /*
                    Serial.print(" Num sats=");
                    Serial.print(fr_numsats);
                    Serial.print(" HDOP=");
                    Serial.println(fr_hdop);
                    // Serial.print(" vdil=");
                    //Serial.print(vdil);                  
                    // Serial.print(" AltMSL=");
                    // Serial.println(altmsl);
                   */
                    break;
                  case 0x5004:                         // Home
                    fr_home = Unpack_uint32(5);
                    fr_home_dist = (fr_home & 0xFFF)>>2;
                    fr_pwr = (fr_home & 0x3000)>>12;     // 10 ^ pwr
                    fRelAlt = (fr_home & 0xFFC000) >> 14; 
                    neg = (fr_home & 0x1000000) >> 24;
                    fRelAlt /=  10;
                    if (neg) fRelAlt = 0 - fRelAlt;
                    cur.alt = fRelAlt;
                    altGood=true; 
                    msl = false;                  // Tell the algorithms altitude is relative to home
                     /*
                    Serial.print(" Dist to home=");
                    Serial.print(fr_home_dist);
                    Serial.print(" pwr=");
                    Serial.print(fr_pwr);                 
                    Serial.print(" Rel Alt=");
                    Serial.println(fRelAlt,1);
                    */
                    break;                        
                  case 0x5005:                      
                  // Vert and Horiz Velocity and Yaw angle (Heading)
                    fr_velyaw = Unpack_uint32(5);      
                    fr_velyaw = fr_velyaw & 0xFFF0000;  // Mask out top 4 bits, keep 12b, drop bottom 16b
                    fr_velyaw =  fr_velyaw >> 16;       // Drop bottom 16 bits, keep 12b  
                    fhdg = fr_velyaw/10; 
                    hdgGood=true;
                    /*
                    Serial.print(" Heading=");
                    Serial.println(fhdg,2);
                    */
                    break;                   
                   
      }
}
//***************************************************
void TestServos() {
PositionServos(90, 0, 90); 
delay(2000);

for (int i=1; i<=360; i++) {
  delay(60);
  PositionServos(i, 30, 90);   
  }
for (int i=1; i<=180; i++) {
  delay(60);
  PositionServos(90, i, 90);   
  }
  
PositionServos(90, 0, 90);   
 }
//***************************************************
void CheckForTimeouts() {
  unsigned long cMillis = millis();
    if ((gpsGood==1) && (cMillis - gpsMillis >= 5000)){
      gpsGood = 0;   // If no GPS packet for 5 seconds then GPS timeout  
      Serial.println("No GPS telemetry for 5 seconds"); 
    }
   ServiceTheStatusLed();
}
//***************************************************

void ServiceTheStatusLed() {
/*
    Serial.print("gpsGood = ");
    Serial.print(gpsGood);
    Serial.print("   serGood = ");
    Serial.print(serGood);
    Serial.print("   homeInitialised = ");
    Serial.println(homeInitialised);
 */
  if (gpsGood) {
    if (homeInitialised) 
      ledState = HIGH;
    else 
      BlinkLed(300);
    }
  else 
     if (serGood) 
       BlinkLed(1500);
     else
       ledState = LOW;
    digitalWrite(StatusLed, ledState);  
    digitalWrite(BoardLed, !ledState);
}

//***************************************************
void BlinkLed(int rate) {
  unsigned long cMillis = millis();
     if (cMillis - ledMillis >= rate) {    // blink period
        ledMillis = cMillis;
        if (ledState == LOW) {
          ledState = HIGH; }   
        else {
          ledState = LOW;  } 
      }
}

//***************************************************
boolean PacketGood() {
// Allow 1 degree of lat and lon away from home, i.e. 60 nautical miles radius at the equator
// Allow 1km up and 300m down from home altitude
if (!homeInitialised) {  //  You can't use the home co-ordinates for a reasonability test if you don't have them yet
  return true;
  exit;
  }
if (cur.lat<(home.lat-1.0) || cur.lat>(home.lat+1.0)) {  // Also works for negative lat
  Serial.print(" Bad lat = ");
  Serial.print(cur.lat,7);
  Serial.println("  Packet ignored");   
  return false; 
  exit; 
  }
  if (cur.lon<(home.lon-1.0) || cur.lon>(home.lon+1.0)) { // Also works for negative lon
  Serial.print(" Bad lon = ");
  Serial.print(cur.lon,7);
  Serial.println("  Packet ignored");  
  return false; 
  exit;  
  }
if (cur.alt<(home.alt-300) || cur.alt>(home.alt+1000)) {
  Serial.print(" Bad alt = ");
  Serial.print(cur.alt);
  Serial.println("  Packet ignored");    
  return false; 
  exit;  
  }
  if (fRelAlt<-300 || fRelAlt>1000) {
  Serial.print(" Bad RelAlt = ");
  Serial.print(fRelAlt);
  Serial.println("  Packet ignored");    
  return false; 
  exit;  
  }
  if (fhdg<0 || fhdg>360) {
  Serial.print(" Bad hdg = ");
  Serial.print(fhdg);
  Serial.println("  Packet ignored");    
  return false; 
  exit;  
  }
return true;
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
void DisplayTheBuffer (int lth){
  for ( int i = 0; i < lth; i++ ) {
    byte b = packetBuffer[i];
    if (b<=0xf) Serial.print("0");
    Serial.print(b,HEX);
    Serial.print(" ");
  }
  Serial.println();

}
//***************************************************
void DisplayField (int pos, int lth){
  for ( int i = pos; i < pos+lth; i++ ) {
    Serial.print(packetBuffer[i],HEX);
    Serial.print(" ");
  }
  Serial.print("// ");
}
//***************************************************
String TimeString (unsigned long epoch){
 int hh = (epoch  % 86400L) / 3600;   // remove the days (86400 secs per day) and div the remainer to get hrs
 int mm = (epoch  % 3600) / 60;       // calculate the minutes (3600 secs per minute)
 int ss = (epoch % 60);               // calculate the seconds

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
//***************************************************

uint8_t Unpack8 (int posn){
  
    uint8_t myvar = packetBuffer[posn];
    return myvar;
}
//***************************************************
void ShowElapsed() {
  Serial.print(" Seconds=");
  unsigned long millnow=millis();
  float fSecs = millnow / 1000;
  Serial.print(fSecs,1);
  Serial.print(" ");
}
//***************************************************
 
