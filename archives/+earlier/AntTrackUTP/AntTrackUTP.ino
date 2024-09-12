/*

Eric's WiFi Antenna Tracker

Eric Stockenstrom - February 2017

This module uses the standard mavlink UDP packets sent from a Pixhawk or APM based airbourne 2.4 GHz WiFi telemetry module 
to calculate where an airbourne craft is relative to the home position. From this it calculates the azimmuth and elevation
of the craft, and then positions 180 degree azimuth and elevation PWM style servos to point a 5GHz double bi_quad antenna 
for telemetry and video.

The code is written from scratch, but I've borrow ideas from Jalves' OpenDIY-AT and other innovators that went before. 

The target board is an ESP-201, and the module performed well in bench and flight tests.
 
See  https://pixhawk.org/peripherals/8266 for information on flashing your own wifi telemtery module.

Version 1.5 of the EZ-Wifibroadcast project is expected the deliver mavlink telemetry out of the ground station Raspi board,
at which time I'll modify this module accordingly.

Alternatively, you could also T udp packets from a 3DR style telemetry link destined for QGroundcontrol or Mission Planner using 
mavproxy.

v0.01 2017-02-15 Uses EricsMavlinkUDPClient07 as the starting point
v0.02 Ready to test
v0.03 2017-02-16 UDP to Az, El and distance all working
v0.04 2017-02-17 Include status LED and GPS signal timeout. Also GPS Status packet. But one never comes through.
v0.05 2017-02-17 Remove gps status packet.
v0.06 2017-02-18 Reverse servo direction because that's the way they happen to be mounted
v0.07 2017-02-19 Add wifi re-connect.
v0.08 2017-02-20 Initialise servos into az=90 deg, el=0 deg
v0.09 2017-03-01 Change SSID to my F550-1
v0.10 2017-03-26 Add support for 180 degree frame of reference fore and aft. i.e ant can point behind.
      Requires two 180 deg servos.
v0.11 2017-03-29 Improve wifi reconnect
v0.12 2017-05-12 Testing with 180 deg servos
v0.13 2017-05-18 Change from WiFi to UTP Ethernet input using ENC28J60
 */

#include <UIPEthernet.h>
#include <Servo.h>

// Enter a MAC address for your controller below.
// Newer Ethernet shields have a MAC address printed on a sticker on the shield
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};

unsigned int localPort = 14550;      // local port to listen for UDP packets

const int packetSize = 70; 
byte packetBuffer[packetSize]; 
int udpGood = 0;
unsigned long udpMillis = 0;

unsigned long LosMillis = 0;  // Moment of loss of wifi signal 

//************* Pin Assignments
int SetHomePin = 5;

int StatusLed = 6;  // Off=No good GPS yet, Flashing=Good GPS but Home not set yet, Solid = Ready to Track
int ledState = LOW; 
unsigned long ledMillis = 0;

int azPWM_Pin = 7;    // Azimuth
int elPWM_Pin = 8;    // Elevation

// ENC28J60 pins for Mini Pro and Uno
// SS  Pin 10 - purple
// SI  Pin 11 - yellow
// SO  Pin 12 - green
// SCK Pin 13 - blue
// RST RST    - orange (optional)
// VCC 3v3    - red
// GND        - black
//************* Pin Assignments ^^^^^^

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

//*************************
// UDP instance 
EthernetUDP udp;
// Servo instances
Servo azServo;            // Azimuth
Servo elServo;            // Elevation

//***************************************************
void setup()
{

  pinMode(SetHomePin, INPUT_PULLUP); // Set pin 5 as an input w/ pull-up for Set-Home
  pinMode(StatusLed, OUTPUT );       // Pin 13
  
  Serial.begin(115200);
  
  azServo.attach(azPWM_Pin);
  elServo.attach(elPWM_Pin);
  PositionServos(90, 0, 90);   // Intialise servos to az=90, el=0, homeHdg = 90;
  
 Serial.println("Starting up......");
  // start Ethernet and UDP
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    // no point in carrying on, so do nothing forevermore:
    for (;;)
      ;
  }

  Serial.print("Ethernet connected with IP address ");
  Serial.println(Ethernet.localIP());

  Serial.println("Starting UDP");
  udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(localPort);
  udpGood = 1;                     // We have a good UDP connection!


}
//***************************************************
//***************************************************
void loop()  {
  
  // wait for packets
  int cb = udp.parsePacket();
  if (!cb) {
   // Serial.println("no packet yet");
  }
  else {
    //   Serial.print("packet received, length=");
    //   Serial.println(cb);
    // We've received a packet, read the data from it
    udp.read(packetBuffer, packetSize); // read the packet into the buffer
    DisplayTheBuffer(cb);  
    if (packetBuffer[5] == 0x21) {            // or #33, then it is a GPS Position packet
      if (gpsGood==0) {
        gpsGood = 1;
        if (homeInitialised ==0)
          Serial.println("GPS lock good! Push set-home button anytime to start tracking.");
        else
          Serial.println("GPS lock good again!");
      }
      gpsMillis = millis();                 // Time of last good GPS packet
      UnpackGpsPosition(cb);
      if (homeInitialised == 1 && gpsGood == 1) {
        GetAzEl(home.lat, home.lon, home.alt, cur.lat, cur.lon, cur.alt);
        if (Distance >= MinDisplacement) PositionServos(Azimuth, Elevation, homeHdg);
      }
    }
  //  int SetHomeState = digitalRead(SetHomePin);
  //  if (SetHomeState == 0 && gpsGood == 1 && homeInitialised ==0){     // pin 5 is pulled up - normally high
    if (gpsGood == 1 && homeInitialised ==0){     // Auto set home on first good GPS packet
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
}
//***************************************************
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
    Serial.print("   udpGood = ");
    Serial.print(udpGood);
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
     if (udpGood == 1) 
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
uint32_t Unpack_uint32 (int posn){
  
    //  The number starts at byte "posn" of the received packet and is four bytes,
    //  or two words, long. First, extract the two words
    //  GPS payload fields are little-endian, i.e they need an end-to-end byte swap
    
    unsigned long highWord = word(packetBuffer[posn+3], packetBuffer[posn+2]);
    unsigned long lowWord = word(packetBuffer[posn+1], packetBuffer[posn]);
    
    // Now combine the four bytes (two words) into an unsigned 32bit integer
    
    uint32_t myvar = highWord << 16 | lowWord;
    return myvar;
}
//***************************************************
int32_t Unpack_int32 (int posn){
  
    //  The number starts at byte "posn" of the received packet and is four bytes,
    //  or two words, long. First, extract the two words
    //  GPS payload fields are little-endian, i.e they need an end-to-end byte swap
    
    unsigned long highWord = word(packetBuffer[posn+3], packetBuffer[posn+2]);
    unsigned long lowWord = word(packetBuffer[posn+1], packetBuffer[posn]);
    
    // Now combine the four bytes (two words) into a signed 32bit integer
    
    uint32_t myvar = highWord << 16 | lowWord;
    return myvar;
}
//***************************************************
uint16_t Unpack_uint16 (int posn){
  
    //  The number starts at byte "posn" of the received packet and is two bytes,
    //  or one word, long. First, extract the word
    //  GPS payload fields are little-endian, i.e they need an end-to-end byte swap
    
    unsigned long theWord = word(packetBuffer[posn+1], packetBuffer[posn]);
    
    // Now convert the 2 bytes / 1 word into an unsigned 16bit integer
    
    uint16_t myvar = theWord; 
    return myvar;
}
//***************************************************
int16_t Unpack_int16 (int posn){
  
    //  The number starts at byte "posn" of the received packet and is two bytes,
    //  or one word, long. First, extract the word
    //  GPS payload fields are little-endian, i.e they need an end-to-end byte swap
    
    unsigned long theWord = word(packetBuffer[posn+1], packetBuffer[posn]);
    
    // Now convert the 2 bytes / 1 word into a signed 16bit integer
    
    int16_t myvar = theWord;
    return myvar;
}
//***************************************************
uint8_t Unpack_uint8 (int posn){
  
    //  The number starts at byte "posn" of the received packet and is one bytes long

    unsigned long theWord = packetBuffer[posn];
    
    // Now convert the byte / 1 word into an unsigned 8 bit integer
    
    uint8_t myvar = theWord;
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
//Unused for now
//***************************************************
uint64_t Unpack64 (int posn){
  
     //the number starts at byte "posn" of the received packet and is eight bytes,
    // or four words, long. First, extract the four words:

    unsigned long highWord1 = word(packetBuffer[posn+7], packetBuffer[posn+6]);
    unsigned long highWord2 = word(packetBuffer[posn+5], packetBuffer[posn+4]);
    unsigned long lowWord1 = word(packetBuffer[posn+3], packetBuffer[posn+2]);
    unsigned long lowWord2 = word(packetBuffer[posn+1], packetBuffer[posn]);
    
    // combine the eight bytes (four words) into a unsigned 64bit integer
    // this is the timestamp (microseconds since Jan 1 1900):
    
    uint64_t myvar = highWord1 << 48 |highWord2 << 32 | lowWord1  <<16 | lowWord2;
    return myvar;
}

//***************************************************
uint8_t Unpack8 (int posn){
  
    uint8_t myvar = packetBuffer[posn];
    return myvar;
}

