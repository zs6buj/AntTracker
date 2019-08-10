
/*

    ZS6BUJ's Antenna Tracker - Wifi UDP Version

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
v0.20 2017-10-20 Fix gps timeout check
*/

    #include <WiFi.h>
//#include <WiFiAP.h> 
 #include <WiFiUDP.h>
#include <ESP32_Servo.h>

//char ssid[] = "PixRacer";            // your network SSID (name)
//char pass[] = "pixracer";            // your network password

char ssid[] = "EZ-WifiBroadcast";            // your network SSID (name)
char pass[] = "wifibroadcast";            // your network password

unsigned int localPort = 14550;      // local port to listen for UDP packets

const int packetSize = 70; 
byte packetBuffer[packetSize]; 
int udpGood = 0;
unsigned long udpMillis = 0;

unsigned long LosMillis = 0;  // Moment of loss of wifi signal 

int SetHomePin = 5;

int StatusLed = 13;  // Off=No good GPS yet, Flashing=Good GPS but Home not set yet, Solid = Ready to Track
int ledState = LOW; 
unsigned long ledMillis = 0;

int azPWM_Pin = 14;    // Azimuth
int elPWM_Pin = 12;    // Elevation

int homeInitialised = 0;
int gpsGood = 0;
int gpsNumSats = 0;
unsigned long gpsMillis = 0;

float fLat = 0;
float fLon = 0;
float fAlt = 0;
float fRelAlt =0;
float fvx = 0;
float fvy = 0;
float fvz = 0;
float fhdg = 0;

int MinDisplacement = 4;

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
WiFiUDP udp;
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
  
 // TestServos();   // Uncomment this code to observe how well your servos reach 0 deg and 180 deg
   
  // Connecting to PixRacer
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  Serial.println("Starting UDP");
  udp.begin(localPort);

  udpGood = 1;                     // We have a good UDP connection!


}
//***************************************************
//***************************************************
void loop()  {

//  CheckForWiFiDisconnect();
  
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
      if (gpsGood ==0) {
        gpsGood = 1;
        Serial.println("GPS lock good! Push set-home button anytime to start tracking.");
      }
      gpsMillis = millis();                 // Time of last good GPS packet
      UnpackGpsPosition(cb);
      if (homeInitialised == 1 && gpsGood ==1) {
        GetAzEl(home.lat, home.lon, home.alt, cur.lat, cur.lon, cur.alt);
        if (Distance >= MinDisplacement) PositionServos(Azimuth, Elevation, homeHdg);
      }
    }
    int SetHomeState = digitalRead(SetHomePin);
    if (SetHomeState == 0 && gpsGood == 1 && homeInitialised ==0){     // pin 12 is pulled up - normally high
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

void CheckForWiFiDisconnect() {
  long Los;
  if (WiFi.status() != WL_CONNECTED) { // If we lose the wifi connection try to re-connect
    gpsGood = 0;
    udpGood = 0;
    Serial.println(" ");
    Serial.println("WiFi link lost! Trying to reconnect. ");
    LosMillis = millis();   // Moment of loss of signal 
    int r = 0;
    while (ReconnectWiFi()==false){   // Retry immediately in this function
      r++;
      if (r>12) while(true) delay(1000); // Give up after 12 mins
      Serial.print("Seconds to try again ");
      Los = 60 - ((millis() - LosMillis)/1000);  // Seconds left of 0 min since loss of signal
      while (Los >0) {  // Then retry every 1 minutes
        Serial.println(Los);
        delay(1000);
        Los = 60 - ((millis() - LosMillis)/1000);
      }
    LosMillis = millis();  // Reset the 10 minute count-down     
    }
   udpGood = 1;    // We have a good UDP connection again
  }
 
}

//***************************************************

boolean ReconnectWiFi()  {
 int q = 0;

  while (WiFi.status() != WL_CONNECTED) { 
    ServiceTheStatusLed();
    delay ( 1000 );             //  Retry every second
    Serial.print ( "." );
    q ++;
    if ( q >120) {
      Serial.println(" ");
      Serial.println("Failed to connect ");
      return false; // Give up after 2 minutes
      exit;
    }
  }
  Serial.println("");
  Serial.println("Reconnected!");
  return true;
  
}

//***************************************************
void ServiceTheStatusLed() {
 //   Serial.print("gpsGood = ");
  //  Serial.print(gpsGood);
 //   Serial.print("   udpGood = ");
 //   Serial.println(udpGood);
    
  if (gpsGood == 1) {
    if (homeInitialised ==1) 
      ledState = HIGH;
    else 
      BlinkLed(300);
    }
  else 
     if (udpGood = 1) 
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
   
/*
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
*/
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
