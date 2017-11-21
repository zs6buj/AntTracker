  
/*

    ZS6BUJ's Antenna Tracker

     Version 0.15 - Serial telemetry input version, small change from v0.14 to accommodate 32 bit words

     Eric Stockenstrom - June 2017
     

This application reads serial telemetry sent from a flight controller or GPS. The module calculates where an airbourne craft is relative to the home position. From this it calculates the azimuth and elevation
of the craft, and then positions 180 degree azimuth and elevation PWM controlled servos to point a 5GHz double bi-quad antenna 
for telemetry and video.

The code is written from scratch, but I've taken ideas from Jalves' OpenDIY-AT and others. Information and ideas on other protocols was obtained frpm
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
v0.15 2017-09-26 Mavlink bluetooth version
 */

#include <Servo.h>

int iLth=0;
int pLth;
byte chr = 0x00;
const int packetSize = 70; 
byte packetBuffer[packetSize]; 

int serGood = 0;

//************* Pin Assignments
// BT Serial1 telemetry pins - RX = A3    TX = A2
// Serial for printout       - RX = A10   TX = A9

int azPWM_Pin =  7;    // A7 Azimuth
int elPWM_Pin =  8;    // A8 Elevation

int SetHomePin = 5;    // A5

int StatusLed = 6;  // A6 - Off=No good GPS yet, Flashing=Good GPS but Home not set yet, Solid = Ready to Track
int ledState = LOW; 
unsigned long ledMillis = 0;

//*************

int homeInitialised = 0;
int gpsGood = 0;
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
float fRelAlt =0;
float fvx = 0;
float fvy = 0;
float fvz = 0;
float fhdg = 0;

int MinDisplacement = 7;  // Distance from home before tracking starts

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
  // Using HC-06 bluetooth model front-end
  Serial1.begin(9600);               //  HC-06 bluetooth module default speed
  delay(200); 
 // Serial1.print("AT+NAMEzs6buj");  //  Optional - Configure your HC-06 bluetooth name
  Serial1.print("AT+BAUD7");         // Set the HC-06 speed to Mavlink default speed 57600 bps
  delay(3000);                       // Wait for HC-06 reboot
  
// Now proceed as per normal serial link
  
  Serial1.begin(57600);       // Telemetry input
  Serial.begin(115200);       // Print Output
  
  delay(2000);
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
  while (!(chr==0xFE)) {   // Read byte stream until you find a candidate mavlink packet start character 
    chr = NextChar();
  }
  // Candidate found! However, it might also be payload data
  packetBuffer[0] = chr;         // Packet start sign
  pLth = NextChar();             // Payload length
  int pl = pLth;
  if (pl<8 || pl>37) {goto TryAgain;} // This one can't be a valid candidate either
  // Payload length - 0x1C or #28 for gps packet
  packetBuffer[1] = pLth;        // Should be 28 for GPS Pos packet
  packetBuffer[2] = NextChar();  // Packet sequence
  packetBuffer[3] = NextChar();  // System ID
  packetBuffer[4] = NextChar();  // Component ID
  packetBuffer[5] = NextChar();  // Message ID
//            [6] through [34]   // Payload
//            [35] and [36]      // CRC low byte then high byte

  pl = (pLth+8);                 // Total length = prefix + payload + chksum should be 36 for GPS Pos packet

    
  for (int i = 6; i < (pl); i++) {   // Read the payload into our packet buffer +2 for chksum
    packetBuffer[i] = NextChar();
  }

 // DisplayTheBuffer(pl); 

  if (packetBuffer[5] == 0x21) {               // or #33, then it is a GPS Position packet
   
    UnpackGpsPosition(pLth);  
    if (!PacketGood() && homeInitialised==1) goto TryAgain;          // This is a reasonability check on the GPS data
 
    //DisplayTheBuffer(pl);  

    if (gpsGood==0) {
      gpsGood = 1;
      if (homeInitialised ==0)
        Serial.println("GPS lock good! Push set-home button anytime to start tracking.");
      else
        Serial.println("GPS lock good again!");
    }
    gpsMillis = millis();                 // Time of last good GPS packet

    if (homeInitialised == 1 && gpsGood == 1) {
      GetAzEl(home.lat, home.lon, home.alt, cur.lat, cur.lon, cur.alt);
      if (Distance >= MinDisplacement) PositionServos(Azimuth, Elevation, homeHdg);
    }
  }
  
  int SetHomeState = digitalRead(SetHomePin);
  if (SetHomeState == 0 && gpsGood == 1 && homeInitialised ==0){     // pin 5 is pulled up - normally high
 //   if (gpsGood == 1 && homeInitialised ==0){     // Auto set home on first good GPS packet
    home.lat = fLat;
    home.lon = fLon;
    home.alt = fAlt;
    homeHdg = fhdg;
    homeInitialised = 1;
    Serial.print("Home location set to Lat = ");
    Serial.print(fLat,7);
    Serial.print(" Lon = ");
    Serial.print(fLon,7);
    Serial.print(" Alt = ");
    Serial.print(fAlt,0); 
    Serial.println();
    }

      
  if (gpsGood==1) CheckGpsTimeout();  
  ServiceTheStatusLed();
  delay(10);
}
//***************************************************
//***************************************************

byte NextChar() {
byte x;

  iLth=Serial1.available();     //   wait for more data
  while (iLth==0) iLth=Serial1.available();
   
  // Data is available
  serGood = 1;                     // We have a good serial connection!
  x = Serial1.read();

  return x;
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
void CheckGpsTimeout() {
  unsigned long cMillis = millis();
    if (cMillis - gpsMillis >= 10000){
      gpsGood = 0;   // If no GPS packet for 10 seconds then GPS timeout  
      Serial.println("GPS signal lost!"); 
    }
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
void UnpackGpsPosition (int lth){
  
    uint32_t TimeStamp = Unpack_uint32(6)/1000;  // millseconds
    int32_t Lat = Unpack_int32(10);              // degrees * 1E7
    int32_t Lon = Unpack_int32(14);              // degrees * 1E7
    int32_t Alt = Unpack_int32(18);              // millimetres
    int32_t RelAlt = Unpack_int32(22);           // millimetres
    int16_t vx = Unpack_int16(26);               // m/s * 100
    int16_t vy = Unpack_int16(28);               // m/s * 100
    int16_t vz = Unpack_int16(30);               // m/s * 100
    uint16_t hdg = Unpack_uint16(32);            // degrees * 100, 0.0..359.99 degrees

    fLat = Lat / 1E7;
    fLon = Lon / 1E7;
    fAlt = Alt / 1E3;
    fRelAlt = RelAlt / 1E3;
    fvx = vx / 100;
    fvy = vy / 100;
    fvz = vz / 100;
    fhdg = hdg / 100;

    cur.lat = fLat;
    cur.lon = fLon;
    cur.alt = fAlt;
   

    Serial.print("Time stamp = ");
    Serial.print(TimeString(TimeStamp));
    Serial.print(" Lat = ");
    Serial.print(fLat,7);
    Serial.print(" Lon = ");
    Serial.print(fLon,7);
    Serial.print(" Alt = ");
    Serial.print(fAlt,0); 
    Serial.print(" Rel Alt = ");
    Serial.print(fRelAlt,0); 
    Serial.print(" vx = ");
    Serial.print(fvx); 
    Serial.print(" vy = ");
    Serial.print(fvy); 
    Serial.print(" vz = ");
    Serial.print(fvz); 
    Serial.print(" hdg = ");
    Serial.println(fhdg,0); 
    Serial.println();

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

