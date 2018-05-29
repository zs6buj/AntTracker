 
/*

    ZS6BUJ's Antenna Tracker

     Version 0.20 - Input MSP Protcol only - serial 2400 bps

     Eric Stockenstrom - August 2017
     

This application reads serial telemetry sent from a flight controller or GPS. The module calculates where an airbourne craft is relative to the home position. From this it calculates the azimuth and elevation
of the craft, and then positions 180 degree azimuth and elevation PWM controlled servos to point a 5GHz double bi-quad antenna 
for telemetry and video.

The code is written from scratch, but I've taken ideas from Jalves' OpenDIY-AT and others. Information and ideas on other protocols was obtained from
GhettoProxy by Guillaume S.

The target board is an STM32F103 "Blue Pill", chosen for its relative power, small size and second (multi) serial port(s) for debugging.

To use the module, position the AntTRacker with the antenna facing the direction of your planned take off. Position the craft a few metres 
further, also facing the same heading for take-off. The flight computer will determine the magnetic heading, from which all subsequent angles 
are calculated.

1 Power up the craft.
2 Power up the ground Raspi board.
3 Power up the AntTracker.
4 When AntTracker successfully connects to the ground raspi, the LED on the front flashes slowly.
5 When AntTracker receives its first good GPS location record, the LED flashes fast.
6 Push the home button to register the home position and heading. The LED goes solidly on.
7 Enjoy your flight! The Tracker will track your craft anywhere in the hemisphere around you (but not closer than 3 metres).

The small double bi-quad antenna has excellent gain, and works well with a vertically polarised stick on the craft. Other reception sticks 
can be added for diversity, and some improvement in link resilience has been observed despite the lower gain of the other links. 
Of course it would be possible to stack double bi-quad antennas on the AntTracker, but more robust mechanicals will be called for.

v0.14 2017-05-22 Serial input version
v0.15 2017-05-30 Mod word length for 32bit MPUs like STM32
v0.16 2017-08-29 Remove Mavlink support. Include support for LTM.
v0.17 2017-08-31 Remove LTM support. Include MSP support. 
v0.20 2017-10-30 Fix bug in gps timeout check as per other versions

 */

#include <Servo.h>
#define MSP_RAW_GPS              106   //out message         fix, numsat, lat, lon, alt, speed, ground course
int iLth=0;
int pLth;  // Packet length
//byte chr = 0x00;
char chr =' ';
const int packetSize = 300; 
byte packetBuffer[packetSize]; 

int serGood = 0;

//************* Pin Assignments
// Serial1 telemetry pins - RX = A3     TX = A2

// Serial for printout    - RX = A10    TX = A9

int azPWM_Pin =  7;    // A7 Azimuth
int elPWM_Pin =  8;    // A8 Elevation

int SetHomePin = 5;    // A5

int StatusLed = 6;  // A6 - Off=No good GPS yet, Flashing=Good GPS but Home not set yet, Solid = Ready to Track
int ledState = LOW; 
unsigned long ledMillis = 0;

//*************

int homeInitialised = 0;
int gpsGood = 0;
int hdgGood = 0;
int gpsNumSats = 0;
unsigned long gpsMillis = 0;

//  variables for servos
int azPWM = 0;
int elPWM = 0;
int LastGoodpntAz = 90;
int LastGoodEl = 0;

float fLat = 0;
float fLon = 0;
float fAlt = 0;
float fRelAlt = 0;
float fvx = 0;
float fvy = 0;
float fvz = 0;
float fHdg = 0;
float fVBat = 0;
float fCur = 0;

int   iSpd = 0;
int   iSat = 0;
int   iFix = 0;
int   iPitch = 0;
int   iRoll = 0;
int   iBat = 0;
int   iRssi = 0;
int   iAirspeed = 0;
int   iFltMode = 0;
byte  crc;

boolean bArmed = false;
boolean bFailsafe = false;
boolean ft = true;

int MinDisplacement = 3;

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
  Serial1.begin(9600);        // Telemetry input
  
  Serial.begin(115200);       // Print Output
  
  delay(3000);
  pinMode(SetHomePin, INPUT_PULLUP); 
  pinMode(StatusLed, OUTPUT );      

  azServo.attach(azPWM_Pin);
  elServo.attach(elPWM_Pin);
  PositionServos(90, 0, 90);   // Intialise servos to az=90, el=0, homeHdg = 90;
  
  Serial.println("Starting up......");
 
}
//***************************************************
//***************************************************
void loop()  {
  
TryAgain:
  chr = NextChar();
  while (!(chr=='$')) {   // Read byte stream until you find a candidate start character 
    chr = NextChar();
  }
  // Candidate found! 
  packetBuffer[0] = chr;         // Packet start scharacter
  chr = NextChar();  
  packetBuffer[1] = chr; 
  if (!chr=='M') goto TryAgain;
  packetBuffer[2] = chr; 
  if (!chr=='<') goto TryAgain;
  // We have an incoming packet

  // Get the payload length
  for (int i = 3; i <= 6; i++) {   
    packetBuffer[i] = NextChar();
  }
  uint8_t pLth = Unpack_uint8(3); 
   
  packetBuffer[7] = NextChar();  // ID

  uint8_t id = Unpack_uint8(7);
  
  for (int i = 8; i <= (pLth+8); i++) {   // Read the payload data into buffer
    packetBuffer[i] = NextChar();
  }
   if (id=106) UnpackGPS(8);
    
  crc=NextChar();

    
  DisplayTheBuffer(40); 

  goto TryAgain;
  /*
  chr = NextChar();    // Packet type
  packetBuffer[2] = chr;
  switch (chr) {
  case 0x41:
    pLth=10;
    UnpackAttitude(pLth);
    break;
  case 0x47:
    pLth=18;
    gpsGood = UnpackGPS(pLth);
    break;
  case 0x53:
    pLth=11;
    UnpackSensors(pLth);
    break;
  default:
    goto TryAgain;    // Don't know this packet type
  }
*/

  
    if (!PacketGood() && homeInitialised==1) goto TryAgain;          // This is a reasonability check on the GPS data
 
    if (gpsGood==1 && hdgGood==1 && ft) {
      ft=false;
      if (homeInitialised ==0)
        Serial.println("GPS lock good! Push set-home button anytime to start tracking.");
      else
        Serial.println("GPS lock good again!");
    }
   

    if (homeInitialised == 1 && gpsGood == 1) {
      GetAzEl(home.lat, home.lon, home.alt, cur.lat, cur.lon, cur.alt);
      if (Distance >= MinDisplacement) PositionServos(Azimuth, Elevation, homeHdg);
    }
  
  
  int SetHomeState = digitalRead(SetHomePin);
  if (SetHomeState == 0 && gpsGood == 1 && homeInitialised ==0){     // pin 5 is pulled up - normally high

    home.lat = fLat;
    home.lon = fLon;
    home.alt = fAlt;
    homeHdg = fHdg;
    homeInitialised = 1;
    Serial.print("Home location set to Lat = ");
    Serial.print(fLat,7);
    Serial.print(" Lon = ");
    Serial.print(fLon,7);
    Serial.print(" Alt = ");
    Serial.print(fAlt,0); 
    Serial.println();
    }

  delay(10);
}
//***************************************************
//***************************************************

byte NextChar() {
byte x;
  iLth=Serial1.available();     //   wait for more data
   while (iLth==0) {
    CheckForTimeouts();
    iLth=Serial1.available();
   }
  // Data is available
  serGood = 1;                     // We have a good serial connection!
  x = Serial1.read();
  return x;
}
//***************************************************
void ParsePacket(int lth){
   for (int i = 3; i < lth; i++) {   // Read the payload into the packet buffer 
    packetBuffer[i] = NextChar();
  } 
}
//***************************************************
boolean UnpackAttitude(int lth) {
  ParsePacket(lth);
  int16_t jPitch = Unpack_int16(3);             // degrees
  int16_t jRoll = Unpack_int16(5);              // degrees 
  int16_t jHdg = Unpack_int16(7);  
  uint8_t Crc = Unpack_uint8(9); 
  uint8_t LTCrc=LTChecksum(lth);
  
  if ((!Crc==LTCrc) || (Crc==0)) return false;  // Drop empty or CRC failed packets

  iPitch = jPitch;
  iRoll = jRoll;
  if (jHdg<0) jHdg+=360;
  fHdg = jHdg;
  hdgGood=1;
 /* 
  DisplayTheBuffer(lth); 
  Serial.print(" Pitch = ");
  Serial.print(iPitch);
  Serial.print(" Roll = ");
  Serial.print(iRoll);
  Serial.print(" Heading = ");
  Serial.print(fHdg,0); 
  Serial.print(" CRC = ");
  Serial.print(Crc); 
  Serial.print(" Calc CRC = ");
  Serial.println(LTCrc); 
  */
  return true;
}
//***************************************************
boolean UnpackGPS(int lth) {
  ParsePacket(lth);

 if (MSPcmd==MSP_RAW_GPS)
  {
    uav_fix_type=read8();
    uav_satellites_visible=read8();
    uav_lat = (int32_t)read32();
    uav_lon = (int32_t)read32();
    #ifndef BARO_ALT
    uav_alt = (int32_t)(read16()*100);
    #endif

    uav_groundspeed = read16();
  }

  int32_t Lat = Unpack_int32(3);              // degrees * 1E7
  int32_t Lon = Unpack_int32(7);              // degrees * 1E7
  uint8_t Spd = Unpack_uint8(11);  
  int32_t Alt = Unpack_int32(12);             // centimetres
  uint8_t Sat = Unpack_uint8(16); 
  int Fix = (Sat & 0b00000011);     // or 0x03  mask the first 6 bits
  Sat = (Sat & 0b11111100) >> 2;    // mask the last 2 bits and shift right 2 
  uint8_t Crc = Unpack_uint8(17); 
  uint8_t LTCrc=LTChecksum(lth);
  if ((!Crc==LTCrc) || (Crc==0)) return false;  // Drop empty or CRC failed packets
  
  fLat = Lat / 1E7;
  fLon = Lon / 1E7;
  fAlt = Alt / 1E2;
  iSpd = Spd;
  iSat = Sat;
  iFix = Fix;
  gpsGood=1;
  gpsMillis = millis();                 // Time of last good GPS packet

  cur.lat = fLat;
  cur.lon = fLon;
  cur.alt = fAlt;
  /*
  DisplayTheBuffer(lth); 
  Serial.print(" Lat = ");
  Serial.print(fLat,7);
  Serial.print(" Lon = ");
  Serial.print(fLon,7);
  Serial.print(" Speed = ");
  Serial.print(iSpd); 
  Serial.print(" Alt = ");
  Serial.print(fAlt,0); 
  Serial.print(" Sats = ");
  Serial.print(iSat); 
  Serial.print(" Fix type = ");
  Serial.print(iFix); 
  Serial.print(" CRC = ");
  Serial.print(Crc); 
  Serial.print(" Calc CRC = ");
  Serial.println(LTCrc); 
  */
  return true;
}
//***************************************************
boolean UnpackSensors(int lth) {
 ParsePacket(lth);
  int16_t jVBat = Unpack_int16(3);        // mV     
  int16_t jCur = Unpack_int16(5);         // mA   
  int8_t jRssi = Unpack_int8(7);  
  uint8_t jAirspeed = Unpack_uint8(8); 
  uint8_t byt = Unpack_uint8(9);   //// last 6 bits: flight mode, 2nd bit: failsafe, 1st bit: Arm status

  iFltMode =  (byt & 0b11111100) >> 2;
  bFailsafe = (byt & 0b00000010);
  bArmed =    (byt & 0b00000001);

  uint8_t Crc = Unpack_uint8(10); 
  uint8_t LTCrc=LTChecksum(lth);
  
  if ((!Crc==LTCrc) || (Crc==0)) return false;  // Drop empty or CRC failed packets

  iRssi = jRssi;
  iAirspeed = jAirspeed;
  fVBat = jVBat / 1E3;
  fCur = jCur / 1E3;
  /*
  DisplayTheBuffer(lth); 
  Serial.print(" Bat Volts = ");
  Serial.print(fVBat,1);
  Serial.print(" Current = ");
  Serial.print(fCur, 1);
  Serial.print(" RSSI = ");
  Serial.print(iRssi); 
  Serial.print(" Airspeed = ");
  Serial.print(iAirspeed);
  Serial.print(" Flight Mode = ");
  Serial.print(iFltMode); 
  Serial.print(" ");
  Serial.print(FlightMode(iFltMode));
  Serial.print(" ");
  Serial.print(" CRC = ");
  Serial.print(Crc); 
  Serial.print(" Calc CRC = ");
  Serial.println(LTCrc); 
  */
  return true;
}
//***************************************************
uint8_t LTChecksum(int lth) {
  uint8_t LTCrc = 0x00;                      // calculate the checksum
  for (int i = 3; i < lth-1; i++) {          // exclude the crc itself
    LTCrc ^= packetBuffer[i];
    }
  return LTCrc;  
}
//***************************************************
String FlightMode(int FM) {
    switch (FM) {
  case 0:
    return "Manual";
    break;  
  case 1:
    return "Rate";
    break;
  case 2:
    return "Attitude/Angle";
    break;
  case 3:
      return "Horizon";
    break;
  case 4:
    return "Acro";
    break;
  case 5:
    return "Stabilized 1";
    break;
  case 6:
    return "Stabilized 2";
    break;
  case 7:
    return "Stabilized 3";
    break;
  case 8:
    return "Altitude Hold";
    break;
  case 9:
    return "Loiter/GPS Hold";
    break;
  case 10:
    return "Auto/Waypoints";
    break;
  case 11:
    return "Heading Hold/Head Free";
    break;
  case 12:
    return "Circle";
    break;
  case 13:
    return "RTH";
    break;
  case 14:
    return "Follow Me";
    break;
  case 15:
    return "Land";
    break;
  case 16:
    return "FlybyWire A";
    break;   
   case 17:
    return "FlybyWire B";
    break;
  case 18:
    return "Cruise";
    break;  
  case 19:
    return "Unknown";
    break;               
  default:
    return "Unknown Code";
    }
}
//***************************************************
void TestServos() {
  PositionServos(90, 0, 90); 
  for (int i=1; i<=360; i++) {
    delay(60);
    PositionServos(i, 30, 90);   
    }
  for (int i=1; i<=170; i++) {
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
  if (gpsGood == 1) {
    if (homeInitialised ==1) 
      ledState = HIGH;
    else 
      BlinkLed(300);
    }
  else 
     if (serGood == 1) 
       BlinkLed(1500);
     else
       ledState = LOW;
    digitalWrite(StatusLed, ledState);  
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
if (homeInitialised==0) {  //  You can't use the home co-ordinates for a reasonability test if you don't have them yet
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
  if (fHdg<0 || fHdg>360) {
  Serial.print(" Bad hdg = ");
  Serial.print(fHdg);
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
    
    uint16_t myvar = b1 << 8 | b2 ;
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

uint8_t Unpack8 (int posn){
  
    uint8_t myvar = packetBuffer[posn];
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
  
  /* Mavlink
  DisplayField (0,6);   // Timestamp
  DisplayField(6,4);    // Lat
  DisplayField(10,4);   // Lon
  DisplayField(14,4);   // Alt
  DisplayField(18,4);   // Rel Alt
  DisplayField(26,4);   // vx
  DisplayField(28,2);   // vy
  DisplayField(30,2);   // vz
  DisplayField(32,2);   // heading 
  Serial.println();
  */
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



