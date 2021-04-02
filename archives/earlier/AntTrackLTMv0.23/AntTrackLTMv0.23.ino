 
/*

     ZS6BUJ's Antenna Tracker

     Input LTM Protcol only - serial 2400 bps

     Eric Stockenstrom - August 2017
     

This application reads serial telemetry sent from a flight controller or GPS. The module 
calculates where an airbourne craft is relative to the home position. From this it 
calculates the azimuth and elevation of the craft, and then positions azimuth and 
elevation PWM controlled servos to point a direction high-gain antenna for telemetry, 
RC or/and and video.

If your servo pair is of the 180 degree type, be sure to comment out this line like 
this:    //#define Az_Servo_360 

Note that the elevation (180 degree) servo flips over to cover the field-of-view behind 
you when the craft enters that space.

If your servo pair comprises a 360 degree azimuth servo and 90 degree elevation servo, be 
sure to un-comment out this line like this:    #define Az_Servo_360 
360 degree code contributed by macfly1202

The code is written from scratch, but I've taken ideas from Jalves' OpenDIY-AT and others. 
Information and ideas on other protocols was obtained from GhettoProxy by Guillaume S.

The target board is an STM32F103 "Blue Pill", chosen for its relative power, small size 
and second (multi) serial port(s) for debugging. The arduino Teensy 3.x is also suitable,
but much more expensive. The arduino mini pro or similar can be made to work but is not 
recommended for perfomance reasons and lack of second (debugging) serial port.

To use the AntTRacker, position it with the antenna facing the centre of the field in front 
of you. Position the craft a few metres further, also facing the same heading for take-off. 
Tracking (movement of the antenna) will occur only when the craft is more than minDist = 4 
metres from home because accuracy increases sharply thereafter.

When your flight system includes a compass/magnetometer:

0 Be sure to comment out this line like this :  //#define No_Compass  
1 Power up the craft.
2 Power up the ground ground system.
3 Power up the AntTracker.
4 When AntTracker successfully connects to the ground system, the LED on the front flashes slowly.
5 When AntTracker receives its first good GPS location record, the LED flashes fast.
6 Make sure the front of your craft is pointing in the direction of the AntTracker antenna at rest.
  The compass heading of the craft now determines the relative heading of the AntTracker antenna.
7 Push the home button to register the home position and heading.  The LED goes solidly on.
8 Enjoy your flight! The Tracker will track your craft anywhere in the hemisphere around you.

When your flight system does NOT include a compass/magnetometer:

0 Be sure to un-comment this line like this :  #define No_Compass  
1 Power up the craft.
2 Power up the ground system.
3 Power up the AntTracker.
4 When AntTracker successfully connects to the ground system, the LED on the front flashes slowly.
5 When AntTracker receives its first 3D fix GPS location record, the LED flashes fast.
6 Pick up your craft and walk forward several metres (4 to 8) in the direction of the AntTracker antenna at rest.
  This deternines the relative heading for the AntTracker antenna.
7 Return and push the home button to register the home position and heading. The LED goes solidly on.
8 Enjoy your flight! The Tracker will track your craft anywhere in the hemisphere around you.

The small double bi-quad antenna has excellent gain, and works well with a vertically polarised stick on the craft. Other reception sticks 
can be added for diversity, and some improvement in link resilience has been observed despite the lower gain of the other links. 
Of course it would be possible to stack double bi-quad antennas on the AntTracker, but more robust mechanicals will be called for.

v0.14 2017-05-22 Serial input version
v0.15 2017-05-30 Mod word length for 32bit MPUs like STM32
v0.16 2017-08-30 Remove Mavlink support. Include support for LTM
v0.20 2017-10-30 Fix bug in gps timeout check as per other versions
v0.21 2018-07-01 Include support for 360 deg servos, craft with no GPS, limit close-to-home altitude error
v0.22 2018-07-20 Clarify compile options 
v0.23 2018-08-13 Add better debugging options
 */

#include <Servo.h>

//************************************* Please select your options here before compiling **************************
// Un-comment (activate) the options below
//#define Az_Servo_360   // Means the azimuth servo can point in a 360 deg circle, elevation servo 90 deg
                         // Default (comment out #define above) is 180 deg azimuth and 180 deg elevation 
//#define No_Compass     // Use the GPS to determine initial heading of craft, and initial heading of Tracker
//*****************************************************************************************************************


#define Debug               Serial         // USB 

//#define Debug_All
//#define Debug_Serial
#define Debug_Telemetry
#define Debug_AzEl
#define Debug_Servos 
//#define Debug_LEDs  
int16_t iLth=0;
int16_t pLth;  // Packet length
byte chr = 0x00;
const int16_t packetSize = 70; 
byte packetBuffer[packetSize]; 

int16_t telGood = 0;

//************* Pin Assignments
// Serial1 telemetry pins - RX = A3     TX = A2

// Serial for printout    - RX = A10    TX = A9

int16_t azPWM_Pin =  7;    // A7 Azimuth
int16_t elPWM_Pin =  8;    // A8 Elevation

int16_t SetHomePin = 5;    // A5

int16_t  StatusLed = 6;  // A6 - Off=No good GPS yet, Flashing=Good GPS but Home not set yet, Solid = Ready to Track
int16_t  ledState = LOW; 
uint32_t ledMillis = 0;

//*************

int16_t homeInitialised = 0;
int16_t gpsGood = 0;
int16_t hdgGood = 0;
int16_t gpsNumSats = 0;
uint32_t gpsMillis = 0;
uint32_t startup_millis = 0;

//  variables for servos
int16_t azPWM = 0;
int16_t elPWM = 0;
int16_t LastGoodpntAz = 90;
int16_t LastGoodEl = 0;

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

int16_t   iSpd = 0;
int16_t   iSat = 0;
int16_t   iFix = 0;
int16_t   iPitch = 0;
int16_t   iRoll = 0;
int16_t   iBat = 0;
int16_t   iRssi = 0;
int16_t   iAirspeed = 0;
int16_t   iFltMode = 0;

bool bArmed = false;
bool bFailsafe = false;
bool ft = true;

int16_t minDist = 4;

// 3D Location vectors
struct Location {
  float lat; //long
  float lon;
  float alt;
  float hdg;
};

struct Location hom     = {
  0,0,0,0};   // home location

struct Location cur      = {
  0,0,0,0};   // current location
  
struct Vector {
  float    az;                     
  float    el;                     
  int32_t  dist;
};

// Vector for home-to-current location
struct Vector hc_vector  = {
  90, 0, 0};

// Servo class declarations
Servo azServo;            // Azimuth
Servo elServo;            // Elevation

//***************************************************
void setup()
{
  Serial1.begin(2400);        // Telemetry input - 2400 for LTM
  
  Debug.begin(115200);       // Print Output
  
  delay(3000);
  pinMode(SetHomePin, INPUT_PULLUP); 
  pinMode(StatusLed, OUTPUT ); 

  startup_millis = millis();    

  azServo.attach(azPWM_Pin);
  elServo.attach(elPWM_Pin);
  PositionServos(90, 0, 90);   // Intialise servos to az=90, el=0, homeHdg = 90;
  
  Debug.println("Starting up......");
 
}
//***************************************************
//***************************************************
void loop()  {
  
TryAgain:
  chr = NextChar();
  while (!(chr==0x24)) {   // Read byte stream until you find a candidate LTM packet start character 
    chr = NextChar();
  }
  // Candidate found! However, it might also be payload data
  packetBuffer[0] = chr;           // packet start char
  chr = NextChar();                // start2 should be 0x54 
  if (!chr==0x54) goto TryAgain;   // otherwise reject the start signals
  packetBuffer[1] = chr;
  
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

 // DisplayTheBuffer(pLth); 
  
    if (!PacketGood() && homeInitialised==1) goto TryAgain;          // This is a reasonability check on the GPS data
 
    #ifndef No_Compass 
    if (gpsGood==1 && hdgGood==1 && ft) {
      ft=false;
      if (homeInitialised ==0)
        Debug.println("GPS lock good! Push set-home button anytime to start tracking.");
      else
        Debug.println("GPS lock good again!");
    }
    #endif

    if (homeInitialised == 1 && gpsGood == 1) {
      GetAzEl(hom, cur);
      if (hc_vector.dist >= minDist) PositionServos(hc_vector.az, hc_vector.el, hom.hdg);
    }
  
  
 uint8_t SetHomeState = digitalRead(SetHomePin);   // Check if home button is pushed

  #ifdef No_Compass  // If no compass use home established when 3D+ lock established, homGood = 1
    if ((SetHomeState == 0) && (gpsGood) && (!homeInitialised)){   
      homeInitialised = true;
      // Calculate heading as vector from home to where craft is now
      float a, la1, lo1, la2, lo2;
      lo1 = hom.lon;
      la1 = hom.lat;
      lo2 = cur.lon;
      la2 = cur.lat;
      
      lo1=lo1/180*PI;  // Degrees to radians
      la1=la1/180*PI;
      lo2=lo2/180*PI;
      la2=la2/180*PI;

      a=atan2(sin(lo2-lo1)*cos(la2), cos(la1)*sin(la2)-sin(la1)*cos(la2)*cos(lo2-lo1));
      hom.hdg=a*180/PI;   // Radians to degrees
      if (hom.hdg<0) hom.hdg=360+hom.hdg;

      hom.lat = cur.lat;
      hom.lon = cur.lon;
      hom.alt = cur.alt;

      DisplayHome();
      
    }  
  #else    // if have compass, use FC heading
  if (SetHomeState == 0 && gpsGood && !homeInitialised){     // pin 5 is pulled up - normally high
    homeInitialised = true;
    hom.lat = cur.lat;
    hom.lon = cur.lon;
    hom.alt = cur.alt;
    hom.hdg = cur.hdg;
   
    DisplayHome();
    
    }
  #endif 
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
  telGood = 1;                     // We have a good serial connection!
  x = Serial1.read();
//  DisplayByte(x);
  return x;
}
//***************************************************
void ParsePacket(int lth){
   for (int i = 3; i < lth; i++) {   // Read the payload into the packet buffer 
    packetBuffer[i] = NextChar();
  } 
}
//***************************************************
void DisplayHome() {
    #if defined Debug_All || defined Debug_AzEl
 //   Debug.print("******************************************");
    Debug.print("Home location set to Lat = "); Debug.print(hom.lat,7);
    Debug.print(" Lon = "); Debug.print(hom.lon,7);
    Debug.print(" Alt = "); Debug.print(hom.alt,0); 
    Debug.print(" hom.hdg = "); Debug.println(hom.hdg,0); 
    #endif 
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
  #if defined Debug_All || defined Debug_Telemetry
    DisplayTheBuffer(lth); 
    Debug.print(" Pitch = ");
    Debug.print(iPitch);
    Debug.print(" Roll = ");
    Debug.print(iRoll);
    Debug.print(" Heading = ");
    Debug.print(fHdg,0); 
    Debug.print(" CRC = ");
    Debug.print(Crc); 
    Debug.print(" Calc CRC = ");
    Debug.println(LTCrc); 
  #endif

  return true;
}
//***************************************************
boolean UnpackGPS(int lth) {
  ParsePacket(lth);
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
  if ((!(Lat == 0)) || (!(Lon == 0))) {
    gpsGood=1;
  }
  gpsMillis = millis();                 // Time of last good GPS packet

  cur.lat = fLat;
  cur.lon = fLon;
  cur.alt = fAlt;
  #if defined Debug_All || defined Debug_Telemetry
    DisplayTheBuffer(lth); 
    Debug.print(" Lat = ");
    Debug.print(fLat,7);
    Debug.print(" Lon = ");
    Debug.print(fLon,7);
    Debug.print(" Speed = ");
    Debug.print(iSpd); 
    Debug.print(" Alt = ");
    Debug.print(fAlt,0); 
    Debug.print(" Sats = ");
    Debug.print(iSat); 
    Debug.print(" Fix type = ");
    Debug.print(iFix); 
    Debug.print(" CRC = ");
    Debug.print(Crc); 
    Debug.print(" Calc CRC = ");
    Debug.print(LTCrc); 
    Debug.print(" gpsGood = ");
    Debug.println(gpsGood); 
  #endif
  return gpsGood;
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
  #if defined Debug_All || defined Debug_Telemetry
    DisplayTheBuffer(lth); 
    Debug.print(" Bat Volts = ");
    Debug.print(fVBat,1);
    Debug.print(" Current = ");
    Debug.print(fCur, 1);
    Debug.print(" RSSI = ");
    Debug.print(iRssi); 
    Debug.print(" Airspeed = ");
    Debug.print(iAirspeed);
    Debug.print(" Flight Mode = ");
    Debug.print(iFltMode); 
    Debug.print(" ");
    Debug.print(FlightMode(iFltMode));
    Debug.print(" ");
    Debug.print(" CRC = ");
    Debug.print(Crc); 
    Debug.print(" Calc CRC = ");
    Debug.println(LTCrc); 
  #endif
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
      Debug.println("No GPS telemetry for 5 seconds"); 
    }
   ServiceTheStatusLed();
}
//***************************************************

void ServiceTheStatusLed() {
  #if defined Debug_All || defined Debug_LEDs
    Debug.print("gpsGood = ");
    Debug.print(gpsGood);
    Debug.print("   telGood = ");
    Debug.print(telGood);
    Debug.print("   homeInitialised = ");
    Debug.println(homeInitialised);
  #endif 
  if (gpsGood == 1) {
    if (homeInitialised ==1) 
      ledState = HIGH;
    else 
      BlinkLed(300);
    }
  else 
     if (telGood == 1) 
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
if (cur.lat<(hom.lat-1.0) || cur.lat>(hom.lat+1.0)) {  // Also works for negative lat
  Debug.print(" Bad lat = ");
  Debug.print(cur.lat,7);
  Debug.println("  Packet ignored");   
  return false; 
  exit; 
  }
  if (cur.lon<(hom.lon-1.0) || cur.lon>(hom.lon+1.0)) { // Also works for negative lon
  Debug.print(" Bad lon = ");
  Debug.print(cur.lon,7);
  Debug.println("  Packet ignored");  
  return false; 
  exit;  
  }
if (cur.alt<(hom.alt-300) || cur.alt>(hom.alt+1000)) {
  Debug.print(" Bad alt = ");
  Debug.print(cur.alt);
  Debug.println("  Packet ignored");    
  return false; 
  exit;  
  }
  if (fRelAlt<-300 || fRelAlt>1000) {
  Debug.print(" Bad RelAlt = ");
  Debug.print(fRelAlt);
  Debug.println("  Packet ignored");    
  return false; 
  exit;  
  }
  if (fHdg<0 || fHdg>360) {
  Debug.print(" Bad hdg = ");
  Debug.print(fHdg);
  Debug.println("  Packet ignored");    
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
  #if defined Debug_All || defined Debug_Serial

  for ( int i = 0; i < lth; i++ ) {
    byte b = packetBuffer[i];
    if (b<=0xf) Debug.print("0");
    Debug.print(b,HEX);
    Debug.print(" ");
  }
  Debug.println();
  
  #endif
}
//***************************************************
void DisplayField (int pos, int lth){
  for ( int i = pos; i < pos+lth; i++ ) {
    Debug.print(packetBuffer[i],HEX);
    Debug.print(" ");
  }
  Debug.print("// ");
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
void DisplayByte(byte b) {
  if (b<=0xf) Debug.print("0");
  Debug.print(b,HEX);
  Debug.print(" ");
}
