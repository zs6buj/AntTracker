
/*  *****************************************************************************

    ZS6BUJ's Antenna Tracker

    Eric Stockenstrom - Original code June 2017  
                        Version 2 March 2019

    This program is free software. You may redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation. See here <http://www.gnu.org/licenses>

    The application was written in the hope that it will be useful, but it comes
    without any warranty or implied warranty of merchantability or fitness 
    for a particular purpose 
    
   *****************************************************************************

   Protocols support: 
   
    Mavlink 1
    Mavlink 2
    FrSky D legacy
    FrSky X S.Port
    FrSky Passthrough (through Mavlink)
    LTM

    MSP and GPS are outstanding.
    
   The code is written from scratch, but I've taken ideas from Jalves' OpenDIY-AT and others. 
   Information and ideas on other protocols was obtained from GhettoProxy by Guillaume S.

   The target board is an STM32F103 "Blue Pill", chosen for its relative power, small size 
   and second (multi) serial port(s) for debugging. The arduino Teensy 3.x is also suitable,
   but much more expensive. The arduino mini pro or similar can be made to work but is not 
   recommended for performance reasons and lack of second (debugging) serial port.
   
   The application reads serial telemetry sent from a flight controller or GPS. It 
   calculates where an airbourne craft is in three-dimensional space relative to the home 
   position. From this it calculates the azimuth and elevation of the craft, and then positions 
   azimuth and elevation PWM controlled servos to point a directional high-gain antenna for 
   telemetry, RC and/or video.

   Version 2 samples telemetry in, and automatically determines speed (baud) and protocol, and 
   supports an OLED display
   
   After the craft and the tracker have been powered on, and any telemetry is received by 
   the tracker, the tracker's LED flashes slowly. After a good GPS lock is received, the 
   tracker's LED flashes fast and an appropriate message is displayed on the OLED display.

   Now, in order to establish the home position, push the Set_Home button. The LED goes on 
   solid and an appropriate message is displayed on the OLED display.(If your craft has no 
   FC and compass, see Heading_Source discussion below);

   For the tracker to position the servos relative to the compass direction of the craft,
   it needs to know the compass direction in degrees where the tracker antenna is pointing 
   at rest. For convenience we call this the Heading_Source. Three possible heading sources 
   are available:

    1) Flight Computer - use this if your craft has a Flight Computer with a compass
 
    Align the craft's heading (where the nose points) with the direction of the tracker 
    antenna, and press the Set_Home button. The tracker will use the heading of the craft 
    obtained from the craft's Flight Computer via telemetry.
    
    2) GPS - use this if your craft has a GPS, but no FC or compass
 
    Position the tracker box such that the antenna faces the field straight ahead. Power up
    the craft and tracker, and wait for a good GPS lock. The tracker's LED will flash fast. 
    Walk several metres (say 5) straight ahead with the craft, place it on the ground, 
    return and press the Set_Home button. The tracker calculates the compass direction of a 
    vector (line) from where the craft was at the first GPS lock, to where it was when the 
    Set_Home button was pressed, then uses the vector as the assumed direction of the tracker 
    antenna. 
    
    3) Tracker's Own Compass - use this if your craft has a GPS, but no FC or compass
 
    Fasten a suitable magnetometer anywhere on the tracker box, facing the direction of the 
    antenna at rest.

  Note that tracking (movement of the antenna) will occur only when the craft is more than 
  minDist = 4 metres from home because accuracy increases sharply thereafter.   
    
  Before you build/compile the tracker firmware, be sure to un-comment the appropriate 
  #define macro according to your choice of heading source:

  //#define Heading_Source   1  // GPS 
  //#define Heading_Source   2  // Flight Computer 
  #define Heading_Source   3  // Tracker_Compass   

  If your servo pair is of the 180 degree type, be sure to comment out line that looks like
  this:    //#define Az_Servo_360 

  Note that the elevation (180 degree) servo flips over to cover the field-of-view behind 
  you when the craft enters that space.

  If your servo pair comprises a 360 degree azimuth servo and 90 degree elevation servo, please
  un-comment the line that looks like this:    #define Az_Servo_360 - 360 degree code contributed by macfly1202
  
  A small double bi-quad antenna has excellent gain, and works well with a vertically polarised antenna
  on the craft. Other antennas can be added for diversity, and some improvement in link resilience has 
  been observed despite the possibly lower gain of the other links. Of course it is possible to stack 
  double bi-quad antennas on the tracker, but more robust mechanicals will be called for.

  STM32F103C Blue-Pill wiring:
  
    Debug monitor   Serial(0)                -->TX1 Pin A9
                                             <--RX1 Pin A10  
                              
    Telemetry-in    Serial1                  -->TX2 Pin A2  (optional) 
                                             <--RX2 Pin A3 
                              
    i2C bus OLED and (optional)Compass          SCL Pin B6                          
                                                SDA Pin B7 
                                 
    Azimuth Servo                               Pin A7 
    Elevation Servo                             Pin A8 

    SetHome button                              Pin A5

    StatusLed                                   Pin A6 - Off=No good GPS yet, flashing=good GPS but home not set yet, solid = ready to track
                                  
    Vcc - 3.3V      IMPORTANT!
    Gnd                              

Legacy
v0.37 2019-02-12 Reintroduce PacketGood() location reasonability test to mitigate telemetry errors.
v0.38 OLED support added

v2.00 2019-03-03 Unified version auto detect telemetry speed and protocol. Mavlink 1 & 2, FrSky (D, S.Port and Passthru), LTM
      MSP and GPS support outstanding
 */

#include <Servo.h>
#include "c_library_v2\ardupilotmega\mavlink.h"  // Mavlink 2 library

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_SSD1306_STM32.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

#define inSerial            Serial1  // General telemetry input

//************************************* Please select your options here before compiling **************************
// Un-comment (activate) the options below
//#define Az_Servo_360   // Means the azimuth servo can point in a 360 deg circle, elevation servo 90 deg
                         // Default (comment out #define above) is 180 deg azimuth and 180 deg elevation 
//#define Heading_Source   1  // GPS 
#define Heading_Source   2  // Flight Computer 
//#define Heading_Source   3  // Tracker_Compass                         
//#define Setup_BT       // Sets up a previously unused BT-06 BT slave module
//#define QLRS           // Un-comment if you use the QLRS variant of Mavlink 
//*****************************************************************************************************************

#define Debug_Minimum    //  Leave this as is unless you need the serial port for something else
#define Debug_Status
//#define Debug_All
//#define Debug_Telemetry
//#define Debug_Baud
//#define Debug_AzEl
//#define Debug_Servos 
//#define Debug_LEDs
//#define Debug_Compass                            
 
//#define Mav_Debug_Heartbeat      
//#define Mav_Debug_GPS_Raw
//#define Mav_Debug_GPS_Int 
//#define Debug_Mav_Buffer  

//#define Debug_FrSky
//#define Debug_LTM
//#define Debug_MSP

// Protocol determination ***************
uint32_t baud = 0;
uint8_t  proto = 0;

// My OLED vaiables *********************
const uint8_t max_row = 8;
const uint8_t max_col = 21;

struct OLED_line {
  char OLx[max_col+1]; // plus 0x00
  };
  
 OLED_line OL[max_row]; 

uint8_t y = 0;

// ************************************

uint8_t azPWM_Pin =  7;    // A7 azimuth servo
uint8_t elPWM_Pin =  8;    // A8 elevation servo

uint8_t SetHomePin = 5;    // A5

uint8_t  StatusLed = 6;     // A6 - Off=No good GPS yet, flashing=good GPS but home not set yet, solid = ready to track
uint8_t  BoardLed = PC13;
uint8_t  ledState = LOW; 
uint32_t led_millis = 0;
uint32_t startup_millis = 0;
uint32_t display_millis = 0;
//*************
  
bool  telGood=false;
bool  gpsGood = false;
bool  cpsGood = false;
bool  homGood=false;    
bool  homeInitialised = false;
bool  new_GPS_data = false;

//  common variables for FrSky, LTM and MSP
int16_t iLth=0;
int16_t pLth;  // Packet length
byte chr = 0x00;
const int packetSize = 70; 
byte packetBuffer[packetSize]; 

//  variables for servos
int16_t azPWM = 0;
int16_t elPWM = 0;
int16_t LastGoodpntAz = 90;
int16_t LastGoodEl = 0;

bool ft = true;
uint8_t minDist = 4;  // dist from home before tracking starts

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
void setup() {

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64 version)
  display.clearDisplay();
  
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  
  OledDisplayln("AntTracker by zs6buj");
  OledDisplayln(""); // move down past yellow 

 #if defined Debug_Minimum || defined Debug_All || defined Debug_Status || defined Debug_LEDs  || defined Mav_Debug_Heartbeat || \
        defined Mav_Debug_GPS_Raw || defined Mav_Debug_GPS_Int || defined Debug_Servos || defined Debug_Compass || defined Debug_Telemetry
    #define Debug               Serial         // USB 
    Debug.begin(115200);                       // Debug monitor output
    delay(2000);
    Debug.println("Starting up......");
  #endif
  
  #ifdef Setup_BT
    // Using HC-06 bluetooth model front-end
  
    OledDisplayln("Setting up BlueTooth");
  
    inSerial.begin(57600);               // If speed already set to 57600, else it will ignore the next command
    Serial1.print("AT+BAUD4");           // Set the HC-06 speed to default speed 9600 bps for AT command mode
    delay(3000);                         // Wait for HC-06 reboot
  
    inSerial.begin(9600);               //  HC-06 bluetooth module default speed for AT command mode
    delay(200); 
    Serial1.print("AT+NAMEAntTrack");    //  Optional - Configure your HC-06 bluetooth name
    delay(200); 
    Serial1.print("AT+BAUD7");           // Set the speed to Mavlink default speed 57600 bps
    delay(3000);                         // Wait for HC-06 reboot
    // Now proceed as per normal serial link
  #endif
  
  startup_millis = millis();
  pinMode(SetHomePin, INPUT_PULLUP); 
  pinMode(StatusLed, OUTPUT );  
  pinMode(BoardLed, OUTPUT);     // Board LED mimics status led
  digitalWrite(BoardLed, HIGH);  // Logic is reversed! Initialse off    

  azServo.attach(azPWM_Pin);
  elServo.attach(elPWM_Pin);
  PositionServos(90, 0, 90);   // Intialise servos to az=90, el=0, hom.hdg = 90;

  DisplayHeadingSource();

  #ifdef QLRS
    Debug.println("QLRS variant of Mavlink expected"); 
    OledDisplayln("QLRS Mavlink expected");
  #endif  
  
  #if (Heading_Source == 3)  // Tracker_Compass

    cpsGood = Initialise_Compass();  // Check if we have a compass on the Tracker
    
    if (!(cpsGood)) {
      #if defined Debug_Minimum || defined Debug_All || defined Debug_Compass  
        Debug.println("Heading_Source = Tracker's Own Compass, but no compass found! Aborting."); 
        OledDisplayln("No compass found!");
      #endif  
      while (1) delay (1000);  // Wait here forever
    }
   
    #if defined Debug_All || defined Debug_Compass
      Debug.println("Display tracker heading for 20 seconds");
      for (int i=1; i<20;i++) {
        GetMagHeading();
        delay(1000);
      }
    #endif
    
  #endif
  
  //TestServos();   // Uncomment this code to observe how well your servos reach their specified limits
                    // Fine tune MaxPWM and MinPWM in Servos module
                               
  proto = GetProtocol();
  inSerial.begin(baud);       // baud determined by GetProtocol()  
  // expect 57600 for Mavlink and FrSky, 2400 for LTM, 9600 for MSP
  switch(proto) {
    
    case 1:    // Mavlink 1
      OledDisplayln("Mavlink 1 found"); 
      break;
    case 2:    // Mavlink 2
      OledDisplayln("Mavlink 2 found"); 
      break;
    case 3:    // FrSky protocol  
      OledDisplayln("FrSky protocol found");           
      break;
    case 4:    // LTM protocol found  
      OledDisplayln("LTM protocol found");    
      break;     
    case 5:    // MSP protocol found 
      OledDisplayln("MSP protocol found"); 
      break; 
    default:   // Unknown protocol 
      OledDisplayln("Unknown protocol!");
      OledDisplayln("Aborting....");        
      while(1) delay(1000);  // wait here forever
                          
  }                  
}
//***************************************************
//***************************************************
void loop()  {

    switch(proto) {
    
      case 1:    // Mavlink 1
        MavLink_Receive();                      
        break;
      case 2:    // Mavlink 2
        MavLink_Receive();                      
        break;
      case 3:    // FrSky
        FrSky_Receive();                     
        break;
      case 4:    // LTM   
        LTM_Receive();   
        break;  
      case 5:    // MSP
 //       MSP_Receive(); 
        break;

        break;
      default:   // Unknown protocol 
        OledDisplayln("Unknown protocol!");  
        while(1) delay(1000);  // wait here forever                     
    }  
    
    ServiceTheStatusLed();
    
    #if (Heading_Source == 2)  || (Heading_Source == 3) // Flight Computer or Tracker's Own Compass
      if (gpsGood==1 && ft) {
        ft=false;
        if (homeInitialised ==0) {
          Debug.println("GPS lock good! Push set-home button anytime to start tracking.");  
          OledDisplayln("GPS lock good! Push");
          OledDisplayln("home button");
        }
        else {
          Debug.println("GPS lock good again!");
          OledDisplayln("GPS lock good again!");   
        } 
      }
    #endif

    if (telGood && homeInitialised && PacketGood() && new_GPS_data) {  //  every time there is new GPS data 
      GetAzEl(hom, cur);
      if (hc_vector.dist >= minDist) PositionServos(hc_vector.az, hc_vector.el, hom.hdg);  // Relative to home heading
      new_GPS_data = false;
    }
 
  
  uint8_t SetHomeState = digitalRead(SetHomePin);            // Check if home button is pushed
  if (SetHomeState == 0 && gpsGood && !homeInitialised){     // pin 5 is pulled up - normally high
    SetHomeParameters();
  }
}
//***************************************************
//***************************************************
void SetHomeParameters() {
  
    #if (Heading_Source == 1)  // GPS 
      if (homGood) {            // Use home established when 3D+ lock established, homGood = 1 
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
        
        homeInitialised = true;
        DisplayHome();
      }
    
    #elif  (Heading_Source == 2)  // Flight Computer  
      hom.hdg = cur.hdg;
      
      hom.lat = cur.lat;
      hom.lon = cur.lon;
      hom.alt = cur.alt;
      
      homeInitialised = true;
      DisplayHome();
    
    #elif (Heading_Source == 3)  // Tracker's Own Compass 
      hom.hdg = GetMagHeading(); 
      
      hom.lat = cur.lat;
      hom.lon = cur.lon;
      hom.alt = cur.alt;
      
      homeInitialised = true;
      DisplayHome(); 
    
    #else  
      #error You must define at least one Heading_Source !
    #endif 
    

}
//***************************************************
void DisplayHome() { 
  OledDisplayln("Home location set");
  OledDisplayln("Tracking now active!");
  #if defined Debug_Minimum || defined Debug_All || defined Debug_AzEl
 //   Debug.print("******************************************");
    Debug.print("Home location set to Lat = "); Debug.print(hom.lat,7);
    Debug.print(" Lon = "); Debug.print(hom.lon,7);
    Debug.print(" Alt = "); Debug.print(hom.alt,0); 
    Debug.print(" hom.hdg = "); Debug.print(hom.hdg,0); 
 //   DisplayHeadingSource();
  #endif 
}
//***************************************************
void DisplayHeadingSource() {
#if defined Debug_Minimum || defined Debug_All || defined Debug_Compass  
  #if (Heading_Source == 1)  
      Debug.println(  "Heading_Source = Craft's GPS"); 
   // OledDisplayln("123456789012345678901");  
      OledDisplayln("HdgSource=Craft's GPS");
  #elif  (Heading_Source == 2)  
      Debug.println("  Heading_Source = Flight Computer");
      OledDisplayln("Heading_Source = FC");
  #elif (Heading_Source == 3)  
      Debug.println("  Heading_Source = Tracker's Own Compass"); 
      OledDisplayln("HdgSource=Tracker GPS");
  #endif 
#endif  
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

void ServiceTheStatusLed() {
  #ifdef Debug_LEDs
    Debug.print("telGood = ");
    Debug.print(telGood);
    Debug.print("   gpsGood = ");
    Debug.print(gpsGood);
    Debug.print("   homeInitialised = ");
    Debug.println(homeInitialised);
 #endif

  if (gpsGood) {
    if (homeInitialised) 
      ledState = HIGH;
    else 
      BlinkLed(100);
    }
  else 
     if (telGood) 
       BlinkLed(1300);
     else
       ledState = LOW;
       
    digitalWrite(StatusLed, ledState);  
    digitalWrite(BoardLed, !ledState);
}

//***************************************************
void BlinkLed(uint16_t rate) {
  unsigned long cMillis = millis();
     if (cMillis - led_millis >= rate) {    // blink period
        led_millis = cMillis;
        if (ledState == LOW) {
          ledState = HIGH; }   
        else {
          ledState = LOW;  } 
      }
}

//***************************************************
String TimeString (unsigned long epoch){
 uint8_t hh = (epoch  % 86400L) / 3600;   // remove the days (86400 secs per day) and div the remainer to get hrs
 uint8_t mm = (epoch  % 3600) / 60;       // calculate the minutes (3600 secs per minute)
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
//***************************************************
void PrintMavBuffer (const void *object){
  byte b; 
  int lth;

  const unsigned char * const bytes = static_cast<const unsigned char *>(object);
  b= bytes[3];
  lth=2+6+b+2;                  // total length  = crc + header + payload length + (crc again for visbility)
  for ( int i = 1; i < lth; i++ ) {
    DisplayByte(bytes[i]);
  }
  Debug.println();
}
//***************************************************
void DisplayTheBuffer (int lth){
  for ( int i = 0; i < lth; i++ ) {
    DisplayByte(packetBuffer[i]);
  }
  Debug.println();
}

void DisplayByte(byte b) {
    if (b<=0xf) Debug.print("0");
    Debug.print(b,HEX);
    Debug.print(" ");
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
  Debug.print(cur.alt, 0);
  Debug.println("  Packet ignored");    
  return false; 
  exit;  
  }
if ((cur.alt-hom.alt)<-300 || (cur.alt-hom.alt)>1000) {
  Debug.print(" Bad RelAlt = ");
  Debug.print((cur.alt-hom.alt), 0);
  Debug.println("  Packet ignored");    
  return false; 
  exit;  
  }
if (cur.hdg<0 || cur.hdg>360) {
  Debug.print(" Bad hdg = ");
  Debug.print(cur.hdg, 0);
  Debug.println("  Packet ignored");    
  return false; 
  exit;  
  }
return true;
}
//***************************************************
void OledDisplayln(String S) {
  if (y>(max_row-1)) {  // last line    0 thru 7
    
    display.clearDisplay();
    display.setCursor(0,0);
    
    for (int i = 0; i < (max_row-1); i++) {     // leave space for new line at the bottom
      if (i > 2) {   // don't scroll the 2 heading lines
        memset(OL[i].OLx, '\0', sizeof(OL[i].OLx));  // flush 
        strncpy(OL[i].OLx, OL[i+1].OLx, sizeof(OL[i+1].OLx));
      }
      display.println(OL[i].OLx);
    }
    display.display();
    y=max_row-1;
  }
  
  display.println(S);
  display.display();
  strncpy(OL[y].OLx, S.c_str(), 21 );

  y++;
}
