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
   Boards supported:
   
   ESP32 dev Board
   STM32 Blue Pill
   Maple Mini
   
   Protocols supported: 
   
    Mavlink 1
    Mavlink 2
    FrSky D legacy
    FrSky X S.Port
    FrSky Passthrough (through Mavlink)
    LTM
    NMEA GPS

    MSP remains outstanding.
    
   The code is written from scratch, but I've taken ideas from Jalves' OpenDIY-AT and others. 
   Information and ideas on other protocols was obtained from GhettoProxy by Guillaume S.
   
   The application reads serial telemetry sent from a flight controller or GPS. It 
   calculates where an airbourne craft is in three-dimensional space, relative to the home 
   position. From this it calculates the azimuth and elevation of the craft, and then positions 
   azimuth and elevation PWM controlled servos to point a directional high-gain antenna for 
   telemetry, RC and/or video.

   Version 2 samples telemetry in, and automatically determines speed (baud) and protocol, and 
   supports an OLED display. The ESP board will support WiFi or Bluetooth telemetry input.
   
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
    
  Before you build/compile the tracker firmware, be sure to modify the avalue of the 
  Heading_Source constant according to your choice of heading source:

  const uint8_t Heading_Source =  1;  // 1=GPS, 2=Flight Computer, 3=Tracker_Compass   

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

    Connections to Blue Pill STM32F103C  are:
   
    0) USB/TTL Serial   -->TX1 Pin A9   Flashing and serial monitor for debug
    0) USB/TTL Serial   -->RX1 Pin A10 
    
    1) Serial1          -->TX2 Pin A2   Telemetry-in 
    2) Serial1          <--RX2 Pin A3   Telemetry-in 
    
    3) i2C bus             SCL Pin B6   OLED and (optional)Compass
    4) i2C bus             SDA Pin B7   OLED and (optional)Compass 
     
    5) Azimuth Servo           Pin A7  
    6) Elevation Servo         Pin A8 

    7) SetHome button          Pin A5
    8) StatusLed               Pin A6 - Off=No good GPS yet, flashing=good GPS but home not set yet, solid = ready to track
    
    9) Vcc 3.3V !   IMPORTANT!
    8) GND

    Connections to Maple Mini STM32F103C are:
   
    0) USB/TTL                               Flashing and Debugging
    
    1) Serial1          -->TX1 Pin A9 (26)   Serial telemetry in
    2) Serial1          <--RX1 Pin A10(25)   Serial telemetry in
    
    3) i2C bus             SCL Pin B6 (16)  OLED and (optional)Compass
    4) i2C bus             SDA Pin B7 (15)  OLED and (optional)Compass 
     
    5) Azimuth Servo           Pin A7  
    6) Elevation Servo         Pin A8 

    7) SetHome button          Pin A5
    8) StatusLed               Pin A6 - Off=No good GPS yet, flashing=good GPS but home not set yet, solid = ready to track
    
    9) Vcc 3.3V !   IMPORTANT!
    8) GND                             

   Connections to ESP32 Dev Board are: 
  
    0) USB           UART0                    Flashing and serial monitor for debug
    1)               UART1   <--rx1 pin d12   Flexible
    2)               UART1   -->tx1 pin d14   Flexible
    3) Mavlink       UART2   <--rx2 pin d9    Mavlink source to ESP32
    4)               UART2   -->tx2 pin d10   Mavlink source from ESP32     
    3) i2C bus                  SCL pin B6 (16)  OLED and (optional)Compass
    4) i2C bus                  SDA pin B7 (15)  OLED and (optional)Compass 
     
    5) Azimuth Servo                Pin A7  
    6) Elevation Servo              Pin A8 

    7) SetHome button               Pin A5
    8) StatusLed                    Pin 13 - Off=No good GPS yet, flashing=good GPS but home not set yet, solid = ready to track
               
    3) Vcc 3.3V !
    4) GN                           

Legacy
v0.37 2019-02-12 Reintroduce PacketGood() location reasonability test to mitigate telemetry errors.
v0.38 OLED support added

v2.00 2019-03-03 Unified version auto detect telemetry speed and protocol. Mavlink 1 & 2, FrSky (D, S.Port and Passthru), LTM
      MSP and GPS support outstanding
v2.01 Support for NMEA GPS added     
v2.05 2019/03/17 UNTESTED BETA Support for retoring home settings from EEPROM after power interruption  
v2.06 2019/06/25 Improved auto baud rate detaction. Define rxPin correctly for Maple Mini
v2.07 2019/08/10 Tidy up mavlink library paths
v2.08 2019/08/13 Fix mavlink ap_fixtype >2 before using lat and lon. Scroll entire Oled screen.
v2.09 2019/08/16 Supports ESP32 Dev Board - BETA
 */

// ******************************************* Auto Determine Target Board *****************************************
//
//                Don't change anything here
//
#if defined (__MK20DX256__) 
  #define Target_Board   0      // Teensy 3.1 and 3.2    
      
#elif defined (__BluePill_F103C8__) ||  defined (MCU_STM32F103RB)
  #define Target_Board   1      // Blue Pill STM32F103C  
         
#elif defined STM32_MEDIUM_DENSITY
  #define Target_Board   2      // Maple_Mini STM32F103C  
     
#elif defined STM32_HIGH_DENSITY
  // LeafLabs high density
  #define Target_Board   2      // Maple_Mini 
  
#elif defined ESP32
  #define Target_Board   3      // Espressif ESP32 Dev Module
  
#else
  #error "No board type defined!"
#endif
// *****************************************************************************************************************

#include <mavlink_types.h>
#include <common/mavlink.h>
#include <ardupilotmega/ardupilotmega.h> 

#include <SPI.h>
#include <Wire.h>

//********************************** E S P 3 2 *********************************  
#if (Target_Board == 3) // ESP32
  #include <ESP32_Servo.h>  
  #include <Adafruit_SSD1306.h>  //#define SSD1306_128_64 ///< DEPRECATED: old way to specify 128x64 screen
  // Dev board default SDA = 21;  
  // Dev board default SCL = 22;  
  #define SCREEN_WIDTH 128 // OLED display width, in pixels
  #define SCREEN_HEIGHT 64 // OLED display height, in pixels
  // 8 rows of 21 characters
  // Declaration for an SSD1306 display connected to I2C 
  #define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)
  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
  #define BuiltinLed  02  

  // For info: Avoid SPI pins - generally   CS=5    MOSI=23   MISO=19   SCK=18  
  uint8_t SetHomePin = 15;   
  uint8_t StatusLed  = 13;    // Off=No good GPS yet, flashing=good GPS but home not set yet, solid = ready to track               
  uint8_t azPWM_Pin  = 36;    // azimuth servo
  uint8_t elPWM_Pin  = 39;    // elevation servo 

  uint8_t rxPin = 16;  
  uint8_t txPin = 17;  
  
#endif

//******************************* S T M 3 2 F 1 0 3 ******************************
#if (Target_Board == 0) || (Target_Board == 1)|| (Target_Board == 2)
  #include <Servo.h>
  #include <Adafruit_SSD1306_STM32.h>

  #define OLED_RESET 4
  Adafruit_SSD1306 display(OLED_RESET);

  uint8_t SetHomePin = 5;    // A5
  uint8_t  StatusLed = 6;    // A6 - Off=No good GPS yet, flashing=good GPS but home not set yet, solid = ready to track
  
  uint8_t azPWM_Pin =  7;    // A7 azimuth servo
  uint8_t elPWM_Pin =  8;    // A8 elevation servo

  #if (Target_Board == 1) // Blue Pill
    uint8_t rxPin = 3;  
    uint8_t txPin = 2; 
    #define BuiltinLed  PC13    // same as reserved constant LED_BUILTIN
  #elif (Target_Board == 2)     //  Maple Mini
    uint8_t txPin = 26;
    uint8_t rxPin = 25; 
    #define BuiltinLed  33      // PB1
  #endif
  
#endif

#ifndef Target_Board
  #error Please choose at least one target board
#endif
//************************************* Please select your options here before compiling **************************

// Un-comment (activate) the options below
//#define Az_Servo_360   // Means the azimuth servo can point in a 360 deg circle, elevation servo 90 deg
                         // Default (comment out #define above) is 180 deg azimuth and 180 deg elevation 
                         
const uint8_t Heading_Source =  2;  // 1=GPS, 2=Flight Computer, 3=Tracker_Compass   
                     
//#define QLRS           // Un-comment if you use the QLRS variant of Mavlink 
//*****************************************************************************************************************


#define Debug               Serial         // USB  
#define inSerial            Serial2        // ESP32 General telemetry input

          
//*****************************************************************************************************************

#define Debug_Minimum    //  Leave this as is unless you need the serial port for something else
#define Debug_Status
//#define Debug_All
//#define Debug_Telemetry
//#define Debug_Protocol
//#define Debug_Baud
//#define Debug_AzEl
//#define Debug_Servos 
//#define Debug_LEDs
//#define Debug_Compass                            
 
//#define Debug_Mav_Heartbeat      
//#define Debug_Mav_GPS
//#define Debug_Mav_Buffer  

//#define Debug_FrSky
//#define Debug_LTM
//#define Debug_MSP
//#define Debug_NMEA_GPS

//#define Debug_EEPROM
//#define Debug_Time 
//#define Debug_Home
// Protocol determination ***************
uint32_t baud = 0;
uint8_t  protocol = 0;

// OLED declarations *************************

#define max_col  21
#define max_row   8
// 8 rows of 21 characters

struct OLED_line {
  char OLx[max_col];
  };
  
 OLED_line OL[max_row]; 

uint8_t row = 0;

// ************************************

bool    rxFT = true;
uint32_t val = 0;
uint16_t addr = 0;
uint8_t  ledState = LOW; 
uint32_t millisLED = 0;
uint32_t millisStartup = 0;
uint32_t millisDisplay = 0;
uint32_t millisGPS = 0;
uint32_t millisStore = 0;
uint32_t millisSync = 0;
uint32_t epochSync = 0;
//*************
  
bool  hbGood=false;
bool  gpsGood = false;
bool  cpsGood = false;
bool  timeGood = false;
bool  homSaved = false;    
bool  homeInitialised = false;
bool  new_GPS_data = false;
bool  ftGetBaud = true;
bool  lostPowerCheckDone = false;
bool  timeEnabled = false;

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


  
// Create Servo objects
Servo azServo;            // Azimuth
Servo elServo;            // Elevation


//***************************************************
void setup() {
  Debug.begin(115200);                       // Debug monitor output
  delay(2000);
  Debug.println("Starting up......");
  
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  
  display.clearDisplay();
  
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  
  OledDisplayln("AntTracker by zs6buj");
  OledDisplayln(""); // move down past yellow 

  Debug.print("Target Board is ");
  #if (Target_Board == 0) // Teensy3x
    Debug.println("Teensy 3.x");
    OledDisplayln("Teensy 3.x");
  #elif (Target_Board == 1) // Blue Pill
    Debug.println("Blue Pill STM32F103C");
    OledDisplayln("Blue Pill STM32F103C");
  #elif (Target_Board == 2) //  Maple Mini
    Debug.println("Maple Mini STM32F103C");
    OledDisplayln("Maple Mini STM32F103C");
  #elif (Target_Board == 3) //  ESP32 Dev Module
    Debug.println("ESP32 Dev Module");
    OledDisplayln("ESP32 Dev Module");
  #endif

  EEPROM_Setup();
  
  millisStartup = millis();
  pinMode(SetHomePin, INPUT_PULLUP); 
  pinMode(StatusLed, OUTPUT ); 
  pinMode(BuiltinLed, OUTPUT);     // Board LED mimics status led
  digitalWrite(BuiltinLed, LOW);  // Logic is NOT reversed! Initialse off    

  azServo.attach(azPWM_Pin);
  elServo.attach(elPWM_Pin);
  PositionServos(90, 0, 90);   // Intialise servos to az=90, el=0, hom.hdg = 90;

  DisplayHeadingSource();

  #ifdef QLRS
    Debug.println("QLRS variant of Mavlink expected"); 
    OledDisplayln("QLRS Mavlink expected");
  #endif  
  
  if (Heading_Source == 3)  {// Tracker_Compass

    cpsGood = Initialise_Compass();  // Check if we have a compass on the Tracker
    
    if (!(cpsGood)) {
      OledDisplayln("No compass found!");
      
      #if defined Debug_Minimum || defined Debug_All || defined Debug_Compass  
        Debug.println("Heading_Source = Tracker's Own Compass, but no compass found! Aborting."); 
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
  }
    
  //TestServos();   // Uncomment this code to observe how well your servos reach their specified limits
                    // Fine tune MaxPWM and MinPWM in Servos module
                              
  protocol = GetProtocol();
  
  inSerial.begin(baud);       // baud determined by GetProtocol()  
  // expect 57600 for Mavlink and FrSky, 2400 for LTM, 9600 for MSP & GPS
  switch(protocol) {
    
    case 1:    // Mavlink 1
      OledDisplayln("Mavlink 1 found");
      Debug.println("Mavlink 1 found"); 
      timeEnabled = true;
      break;
    case 2:    // Mavlink 2
      OledDisplayln("Mavlink 2 found");
      Debug.println("Mavlink 2 found");
      timeEnabled = true;
      break;
    case 3:    // FrSky protocol  
      OledDisplayln("FrSky protocol found");
      Debug.println("FrSky protocol found");       
      break;
    case 4:    // LTM protocol found  
      OledDisplayln("LTM protocol found"); 
      Debug.println("LTM protocol found");       
      break;     
    case 5:    // MSP protocol found 
      OledDisplayln("MSP protocol found"); 
      Debug.println("MSP protocol found");   
      break; 
    case 6:    // GPS NMEA protocol found 
      OledDisplayln("NMEA protocol found"); 
      Debug.println("NMEA protocol found"); 
      set_up_GPS(); 
      if (Heading_Source == 2) {  // Flight Computer !! If we have found the NMEA protol then FC is unlikely
        OledDisplayln("Check heading source!");
        OledDisplayln("Aborting...");
        Debug.println("Check heading source! Aborting...");
        while (1) delay (1000);  // Wait here forever
      }
      timeEnabled = true;   
      break;     
    default:   // Unknown protocol 
      OledDisplayln("Unknown protocol!");
      OledDisplayln("Aborting....");        
      while(1) delay(1000);  // wait here forever
                          
  } 
  OledDisplayln("Waiting for GPS lock");                 
}
//***************************************************
//***************************************************
void loop()  {

    switch(protocol) {
    
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
 //     MSP_Receive(); 
        break;
      case 6:    // GPS
        GPS_Receive(); 
        break;
      default:   // Unknown protocol 
        OledDisplayln("Unknown protocol!");  
        while(1) delay(1000);  // wait here forever                     
    }  
    
    ServiceTheStatusLed();  

    if ( ((timeEnabled) && (lostPowerCheckDone)) || (!timeEnabled)  ) {
      if (gpsGood==1 && ft) {
        ft=false;
        if (homeInitialised==0) {
          if (Heading_Source == 1)  { // GPS
            Debug.println("GPS lock good! Walk straight ahead 10m then push home button");  
            OledDisplayln("GPS lock good. Carry ");
            OledDisplayln("craft fwrd 10m. Place");
            OledDisplayln(" and push home button");
          }
         else {  // FC & own tracker compass
            Debug.println("GPS lock good! Push set-home button anytime to start tracking.");  
            OledDisplayln("GPS lock good! Push");
            OledDisplayln("home button");
          }
        }
      }
    }

  if (homeInitialised)
    if (hbGood && gpsGood && PacketGood() && new_GPS_data) {  //  every time there is new GPS data 
      GetAzEl(hom, cur);
      if (hc_vector.dist >= minDist) PositionServos(hc_vector.az, hc_vector.el, hom.hdg);  // Relative to home heading
      new_GPS_data = false;
    }
 
  uint8_t SetHomeState = digitalRead(SetHomePin);            // Check if home button is pushed = 0
  if (SetHomeState == 0 && gpsGood && !homeInitialised){     // pin 5 is pulled up - normally high
    FinalStoreHome();
  }

  if ((lostPowerCheckDone) && (timeGood) && (millis() - millisStore) > 60000) {  // every 60 seconds
    StoreEpochPeriodic();
    millisStore = millis();
  }

}
//***************************************************
//***************************************************
void FinalStoreHome() {

   switch(Heading_Source) {
    
      case 1:    // // GPS 
        if (homSaved) {            // Use home established when 3D+ lock established, homSaved = 1 
          // Calculate heading as vector from home to where craft is now
          float a, la1, lo1, la2, lo2;
          lo1 = hom.lon; // From AutoStoreHome()
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

          OledDisplayln("Heading calculated");
          OledDisplayln("Tracking now active!");
        
          homeInitialised = true;

        }                  
        break;
      case 2:    // Flight Computer  

        hom.lat = cur.lat;
        hom.lon = cur.lon;
        hom.alt = cur.alt;
        hom.hdg = cur.hdg;  // from FC
        
        homeInitialised = true;
        DisplayHome();                  
        break;
      case 3:    // Tracker's Own Compass     
        hom.lat = cur.lat;
        hom.lon = cur.lon;
        hom.alt = cur.alt;
        hom.hdg = GetMagHeading(); // From own compass
           
        homeInitialised = true;
        DisplayHome();                  
        break;

      default:   // Unknown protocol 
        OledDisplayln("No Heading_Source!");
        OledDisplayln("Aborting");
        while(1) delay(1000);  // wait here forever                     
    }
     SaveHomeToFlash(); 
 }
//***************************************************
void AutoStoreHome() {
  #ifdef Debug_Status
    Debug.println("homSaved=true");  
  #endif
  hom.lat = cur.lat;
  hom.lon = cur.lon;
  hom.alt = cur.alt;
  
  OledDisplayln("Home loctn auto-stord"); 

  #if defined Debug_All || defined Debug_Status
    Debug.print("Home auto stored: ");       
    Debug.print("hom.lat=");  Debug.print(hom.lat, 7);
    Debug.print(" hom.lon="); Debug.print(hom.lon, 7 );        
    Debug.print(" hom.alt="); Debug.println(hom.alt, 1);                 
 #endif 
   
 // homSaved set true in SaveHomeToFlash
 SaveHomeToFlash();  
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
    Debug.print(" hom.hdg = "); Debug.println(hom.hdg,0); 
 //   DisplayHeadingSource();
  #endif 
}
//***************************************************
void DisplayHeadingSource() {
#if defined Debug_Minimum || defined Debug_All || defined Debug_Compass  
  if (Heading_Source == 1)  {
      Debug.println("Heading_Source = Craft's GPS"); 
      OledDisplayln("HdgSource=Craft's GPS");
  }
  else if  (Heading_Source == 2) { 
      Debug.println("Heading_Source = Flight Computer");
      OledDisplayln("Heading_Source = FC");
  }
  else if (Heading_Source == 3)  {
      Debug.println("Heading_Source = Tracker's Own Compass"); 
      OledDisplayln("HdgSource=Tracker GPS");
  }
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
    Debug.print("hbGood = ");
    Debug.print(hbGood);
    Debug.print("   gpsGood = ");
    Debug.print(gpsGood);
    Debug.print("   homeInitialised = ");
    Debug.println(homeInitialised);
 #endif

  if (gpsGood) {
    if (homeInitialised) 
      ledState = HIGH;
    else 
      BlinkLed(500);
    }
  else 
     if (hbGood) 
       BlinkLed(1300);
     else
       ledState = LOW;
       
    digitalWrite(StatusLed, ledState);  
    digitalWrite(BuiltinLed, ledState);
}

//***************************************************
void BlinkLed(uint16_t period) {
  uint32_t cMillis = millis();
     if (cMillis - millisLED >= period) {    // blink period
        millisLED = cMillis;
        if (ledState == LOW) {
          ledState = HIGH; }   
        else {
          ledState = LOW;  } 
      }
}

//***************************************************
String TimeString (unsigned long epoch){
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
  Debug.print(" Bad lat! cur.lat=");
  Debug.print(cur.lat,7);  
  Debug.print(" hom.lat=");Debug.print(hom.lat,7);
  Debug.println("  Packet ignored");   
  return false; 
  exit; 
  }
if (cur.lon<(hom.lon-1.0) || cur.lon>(hom.lon+1.0)) { // Also works for negative lon
  Debug.print(" Bad lon! cur.lon=");
  Debug.print(cur.lon,7);  
  Debug.print(" hom.lon=");Debug.print(hom.lon,7);
  Debug.println("  Packet ignored");  
  return false; 
  exit;  
  }
if (cur.alt<(hom.alt-300) || cur.alt>(hom.alt+1000)) {
  Debug.print(" Bad alt! cur.alt=");
  Debug.print(cur.alt,0);  
  Debug.print(" hom.alt=");Debug.print(hom.alt,0);
  Debug.println("  Packet ignored");    
  return false; 
  exit;  
  }
if ((cur.alt-hom.alt)<-300 || (cur.alt-hom.alt)>1000) {
  Debug.print(" Bad alt! cur.alt=");
  Debug.print(cur.alt,0);  
  Debug.print(" hom.alt=");Debug.print(hom.alt,0);
  Debug.println("  Packet ignored");    
  return false; 
  exit;  
  }
if (cur.hdg<0 || cur.hdg>360) {
  Debug.print(" Bad hdg! cur.hdg=");
  Debug.print(cur.hdg,0);  
  Debug.print(" hom.hdg=");Debug.print(hom.hdg,0);
  Debug.println("  Packet ignored");    
  return false; 
  exit;  
  }
return true;
}


//***************************************************
void CheckForTimeouts() {
  uint32_t cMillis = millis();
    if ((gpsGood==1) && (cMillis - millisGPS >= 5000)){
      gpsGood = 0;   // If no GPS packet for 5 seconds then GPS timeout 
      hbGood = 0;
      OledDisplayln("No GPS packts/timeout");     
      #if defined Debug_All || defined Debug_FrSky || defined Debug_NMEA_GPS || defined Debug_LTM
        Debug.println("No GPS telemetry for 5 seconds"); 
      #endif  
    }
   ServiceTheStatusLed();
}
//***************************************************
void OledDisplayln(String S) {

  if (row>(max_row-1)) {  // last line    0 thru max_row-1
    
    display.clearDisplay();
    display.setCursor(0,0);
    
    for (int i = 0; i < (max_row-1); i++) {     // leave space for new line at the bottom
                                                //   if (i > 1) {   // don't scroll the 2 heading lines
      if (i >= 0) {         
        memset(OL[i].OLx, '\0', sizeof(OL[i].OLx));  // flush          
        strncpy(OL[i].OLx, OL[i+1].OLx, sizeof(OL[i+1].OLx));
      }
      display.println(OL[i].OLx);
    }
    display.display();
    row=max_row-1;
  }
  display.println(S);
  display.display();
  strncpy(OL[row].OLx, S.c_str(), max_col-1 );  
  row++;
}
//***************************************************
uint32_t epochNow() {
return (epochSync + (millis() - millisSync) / 1E3);
}
//***************************************************
void LostPowerCheckAndRestore(uint32_t epochSyn) {  // this function only ever called by a time enabled protocol
  if ((!timeGood) || (epochSyn == 0)) return;
  
  epochSync = epochSyn;
  millisSync = millis();
 
  if (lostPowerCheckDone) return;

  #if defined Debug_All || defined Debug_Time || defined Debug_Home
    Debug.print("Checking for RestoreHomeFromFlash conditions:"); 
    Debug.print("  epochHome="); Debug.print(TimeString(epochHome())); 
    Debug.print("  epochNow="); Debug.println(TimeString(epochNow()));
  #endif 

  
  if ((epochNow() -  epochHome()) < 300) {  //  within 5 minutes
    RestoreHomeFromFlash();
    homeInitialised=1;           
  }
  
  lostPowerCheckDone = true;
}
//***************************************************
void SaveHomeToFlash() {

  EEPROMWritelong(0, epochNow());   // epochHome
  EEPROMWritelong(1, hom.lon*1E6);  // float to long
  EEPROMWritelong(2, hom.lat*1E6);
  EEPROMWritelong(3, hom.alt*10);
  EEPROMWritelong(4, hom.hdg*10);  
  
#if defined Debug_All || defined Debug_EEPROM || defined Debug_Time || defined Debug_Home
  Debug.print("  homSaved="); Debug.print(homSaved);
  Debug.print("  home.lon="); Debug.print(hom.lon, 6);
  Debug.print("  home.lat="); Debug.print(hom.lat, 6);
  Debug.print("  home.alt="); Debug.print(hom.alt, 1);
  Debug.print("  home.hdg="); Debug.println(hom.hdg, 1);

#endif   
}

//***************************************************
void StoreEpochPeriodic() {
  uint32_t epochPeriodic = epochNow();
  EEPROMWritelong(5, epochPeriodic); // Seconds

  if (homeInitialised) EEPROMWritelong(0, epochPeriodic); // UPDATE epochHome
  
  #if defined Debug_All || defined Debug_EEPROM || defined Debug_Time || defined Debug_Home
  Debug.print("epochPeriodic stored="); Debug.println(TimeString(epochPeriodic));

#endif  
}

//***************************************************
uint32_t epochHome() {

uint32_t epHome = EEPROMReadlong(5);

 #if defined Debug_All || defined Debug_EEPROM
   Debug.print("epochHome="); Debug.println(TimeString(epHome));

 #endif
 return epHome;    
}
//***************************************************
void RestoreHomeFromFlash() {

  hom.lon = EEPROMReadlong(1) / 1E6; //long back to float
  hom.lat = EEPROMReadlong(2) / 1E6;
  hom.alt = EEPROMReadlong(3) / 10;
  hom.hdg = EEPROMReadlong(4) / 10;
  
  Debug.print("Home data restored from Flash"); 
  OledDisplayln("Home data restored");
  OledDisplayln("from Flash. Go Fly!");
  
  #if defined Debug_All || defined Debug_EEPROM || defined Debug_Time || defined Debug_Home
    Debug.print("  home.lon="); Debug.print(hom.lon, 6);
    Debug.print("  home.lat="); Debug.print(hom.lat, 6);
    Debug.print("  home.alt="); Debug.print(hom.alt, 0);
    Debug.print("  home.hdg="); Debug.println(hom.hdg, 0);
  #endif  
}
//***************************************************
