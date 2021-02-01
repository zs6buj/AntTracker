/*  *****************************************************************************

    ZS6BUJ's Antenna Tracker

    Eric Stockenstrom - Original code June 2017  
                        Version 2 March 2019

  License and Disclaimer

 
  This software is provided under the GNU v2.0 License. All relevant restrictions apply including 
  the following: In case there is a conflict, the GNU v2.0 License is overriding. This software is 
  provided as-is in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the 
  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General 
  Public License for more details. In no event will the authors and/or contributors be held liable
  for any damages arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose, including commercial 
  applications, and to alter it and redistribute it freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software.
  2. If you use this software in a product, an acknowledgment in the product documentation would be appreciated.
  3. Altered versions must be plainly marked as such, and must not be misrepresented as being the original software.
  4. This notice may not be removed or altered from any distribution.  

  By downloading this software you are agreeing to the terms specified in this page and the spirit of thereof.
    
   *****************************************************************************
   Boards supported:
   
   ESP32 Dev Board
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

*/

#include <mavlink_types.h>
#include <common/mavlink.h>
#include <ardupilotmega/ardupilotmega.h> 

#include <SPI.h>
#include <Wire.h>

#include "config.h"


// Protocol determination
uint32_t baud = 0;
uint8_t  protocol = 0;

// OLED declarations

#define max_col  22
#define max_row   8
// 8 rows of 21 characters

struct OLED_line {
  char OLx[max_col];
  };
  
 OLED_line OL[max_row]; 

uint8_t row = 0;
uint8_t col = 0;

// ************************************

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
bool  wifiSuDone = false;
bool  wifiSuGood = false;
bool  rxFT = true;
bool  gotRecord = false; 
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
bool  gotClient = false;

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

// Create Bluetooth object
#if (Telemetry_In == 1) 
  BluetoothSerial SerialBT;
#endif


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
  
  OledPrintln("AntTracker by zs6buj");
  OledPrintln(""); // move down past yellow 

  Debug.print("Target Board is ");
  #if (Target_Board == 0) // Teensy3x
    Debug.println("Teensy 3.x");
    OledPrintln("Teensy 3.x");
  #elif (Target_Board == 1) // Blue Pill
    Debug.println("Blue Pill STM32F103C");
    OledPrintln("Blue Pill STM32F103C");
  #elif (Target_Board == 2) //  Maple Mini
    Debug.println("Maple Mini STM32F103C");
    OledPrintln("Maple Mini STM32F103C");
  #elif (Target_Board == 3) //  ESP32 Dev Module
    Debug.println("ESP32 Dev Module");
    OledPrintln("ESP32 Dev Module");
  #endif

  #if (Telemetry_In == 0)  // Serial
    Debug.println("Mavlink Serial In");
    OledPrintln("Mavlink Serial In");
  #endif  

  #if (Telemetry_In == 1)  // BT
    Debug.println("Mavlink BT In");
    OledPrintln("Mavlink BT In");
  #endif  

  #if (Telemetry_In  == 2)  // WiFi
    Debug.println("Mavlink WiFi In");
    OledPrintln("Mavlink WiFi In");
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
    OledPrintln("QLRS Mavlink expected");
  #endif  
  
  if (Heading_Source == 3)  {// Tracker_Compass

    cpsGood = Initialise_Compass();  // Check if we have a compass on the Tracker
    
    if (!(cpsGood)) {
      OledPrintln("No compass found!");
      
      #if defined Debug_Minimum || defined Debug_All || defined Debug_Compass  
        Debug.println("Heading_Source = Tracker's Own Compass, but no compass found! Aborting."); 
      #endif  
      while (1) delay (1000);  // Wait here forever
    }
   
    #if defined Debug_All || defined Debug_Compass
      Debug.println("Display tracker heading for 20 seconds");
      for (int i=1; i<=20;i++) {
        GetMagHeading();
        delay(1000);
      }
    #endif  
  }
    
  #if defined TestServos
    TestServos();   //observe how well your servos works in function of Az,El and Reference Home Heading
                    // Fine tune MaxPWM and MinPWM in Servos module
  #endif

// ************************ Setup Serial ******************************

  #if (Telemetry_In == 0)    //  Serial

    protocol = GetProtocol();
  
    inSerial.begin(baud);       
    // expect 57600 for Mavlink and FrSky, 2400 for LTM, 9600 for MSP & GPS
    switch(protocol) {
    
      case 1:    // Mavlink 1
        OledPrintln("Mavlink 1 found");
        Debug.println("Mavlink 1 found"); 
        timeEnabled = true;
        break;
      case 2:    // Mavlink 2
        OledPrintln("Mavlink 2 found");
        Debug.println("Mavlink 2 found");
        timeEnabled = true;
        break;
      case 3:    // FrSky protocol  
        OledPrintln("FrSky protocol found");
        Debug.println("FrSky protocol found");       
        break;
      case 4:    // LTM protocol found  
        OledPrintln("LTM protocol found"); 
        Debug.println("LTM protocol found");       
        break;     
      case 5:    // MSP protocol found 
        OledPrintln("MSP protocol found"); 
        Debug.println("MSP protocol found");   
        break; 
      case 6:    // GPS NMEA protocol found 
        OledPrintln("NMEA protocol found"); 
        Debug.println("NMEA protocol found"); 
        set_up_GPS(); 
        if (Heading_Source == 2) {  // Flight Computer !! If we have found the NMEA protol then FC is unlikely
          OledPrintln("Check heading source!");
          OledPrintln("Aborting...");
          Debug.println("Check heading source! Aborting...");
          while (1) delay (1000);  // Wait here forever
        }
        timeEnabled = true;   
        break;     
      default:   // Unknown protocol 
        OledPrintln("Unknown protocol!");
        OledPrintln("Aborting....");        
        while(1) delay(1000);  // wait here forever                        
    }
   
  #endif
  
// ************************ Setup Bluetooth ***************************  
  #if (Target_Board ==  3) && (Telemetry_In == 1) 
    #ifdef BT_Master_Mode
      SerialBT.begin("AntTrack", true);            
    #else
        SerialBT.begin("AntTrack");   
    #endif 
      
      bool bt_connected;

      bt_connected = SerialBT.connect(BT_Slave_Name);
     if(bt_connected) {
          Debug.println("Bluetooth connected!");
          OledPrintln("Bluetooth connected!");
      }    
      
  #endif 
  
  // ************************* Setup WiFi **************************** 
  #if (Target_Board ==  3)  
   #if (Telemetry_In == 2)     //  WiFi / Mavlink
     SetupWiFi();  
   #endif
   
 #endif  
      
}


// *******************************************************************************************

void loop() {            // For WiFi TCP/IP only
  
#if (Telemetry_In == 2)    // WiFi

  #if (WiFi_Protocol == 1)  // TCP  
    if (wifiSuGood) {
      wifi = server.available();              // listen for incoming clients 
      if(wifi) {
        Debug.println("New client connected"); 
        OledPrintln("New client ok!");      
        while (wifi.connected()) {            // loop while the client's connected
          main_loop(); 
        }
      wifi.stop();
      Debug.println("Client disconnected");
      OledPrintln("Client discnnct!");      
      } else {
         main_loop();
     } 
    }  else { 
       main_loop();
    }  
  #endif  
  
  #if (WiFi_Protocol == 2)  // UDP  
    main_loop();       
  #endif  
     
#else 
  main_loop();
#endif
  
}

//***************************************************
//***************************************************
void main_loop()  {

  #if (Telemetry_In == 1)       // Bluetooth
    Mavlink_Receive();
  #endif

  #if (Telemetry_In == 2)       //   WiFi
    #if (WiFi_Protocol == 1)  // TCP/IP
      if (gotClient) {     
        Mavlink_Receive();
      }
    #endif
    #if (WiFi_Protocol == 2)  // UDP 
      Mavlink_Receive();
    #endif   
  #endif
  
  #if (Telemetry_In == 0)      // Serial according to protocol
    switch(protocol) {
    
      case 1:    // Mavlink 1
        Mavlink_Receive();               
        break;
      case 2:    // Mavlink 2
        Mavlink_Receive();
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
        OledPrintln("Unknown protocol!");  
        while(1) delay(1000);  // wait here forever                     
    }  
   #endif
  
ServiceTheStatusLed();  

    if ( ((timeEnabled) && (lostPowerCheckDone)) || (!timeEnabled)  ) {
      if (gpsGood==1 && ft) {
        ft=false;
        if (homeInitialised==0) {
          if (Heading_Source == 1)  { // GPS
            Debug.println("GPS lock good! Walk straight ahead 10m then push home button");  
            OledPrintln("GPS lock good. Carry ");
            OledPrintln("craft fwrd 10m. Place");
            OledPrintln(" and push home button");
          }
         else {  // FC & own tracker compass
            Debug.println("GPS lock good! Push set-home button anytime to start tracking.");  
            OledPrintln("GPS lock good! Push");
            OledPrintln("home button");
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

          OledPrintln("Heading calculated");
          OledPrintln("Tracking now active!");
        
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
        OledPrintln("No Heading_Source!");
        OledPrintln("Aborting");
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

  // homSaved set true in SaveHomeToFlash
  SaveHomeToFlash();  
  homSaved = true;
  
  OledPrintln("Home loctn auto-stord"); 

  #if defined Debug_All || defined Debug_Status
    Debug.print("Home auto stored: ");       
    Debug.print("hom.lat=");  Debug.print(hom.lat, 7);
    Debug.print(" hom.lon="); Debug.print(hom.lon, 7 );        
    Debug.print(" hom.alt="); Debug.println(hom.alt, 1);                 
 #endif 
   
} 
//***************************************************
void DisplayHome() { 
  OledPrintln("Home location set");
  OledPrintln("Tracking now active!");
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
      OledPrintln("HdgSource=Craft's GPS");
  }
  else if  (Heading_Source == 2) { 
      Debug.println("Heading_Source = Flight Computer");
      OledPrintln("Heading_Source = FC");
  }
  else if (Heading_Source == 3)  {
      Debug.println("Heading_Source = Tracker's Own Compass"); 
      OledPrintln("HdgSource=Tracker GPS");
  }
#endif  
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
  }
if (cur.lat<(hom.lat-1.0) || cur.lat>(hom.lat+1.0)) {  // Also works for negative lat
  Debug.print(" Bad lat! cur.lat=");
  Debug.print(cur.lat,7);  
  Debug.print(" hom.lat=");Debug.print(hom.lat,7);
  Debug.println("  Packet ignored");   
  return false; 
  }
if (cur.lon<(hom.lon-1.0) || cur.lon>(hom.lon+1.0)) { // Also works for negative lon
  Debug.print(" Bad lon! cur.lon=");
  Debug.print(cur.lon,7);  
  Debug.print(" hom.lon=");Debug.print(hom.lon,7);
  Debug.println("  Packet ignored");  
  return false;  
  }
if (cur.alt<(hom.alt-300) || cur.alt>(hom.alt+1000)) {
  Debug.print(" Bad alt! cur.alt=");
  Debug.print(cur.alt,0);  
  Debug.print(" hom.alt=");Debug.print(hom.alt,0);
  Debug.println("  Packet ignored");    
  return false;  
  }
if ((cur.alt-hom.alt)<-300 || (cur.alt-hom.alt)>1000) {
  Debug.print(" Bad alt! cur.alt=");
  Debug.print(cur.alt,0);  
  Debug.print(" hom.alt=");Debug.print(hom.alt,0);
  Debug.println("  Packet ignored");    
  return false;  
  }
if (Heading_Source == 2) { //  Heading source from flight controller
  if (cur.hdg<0 || cur.hdg>360) {
    Debug.print(" Bad hdg! cur.hdg=");
    Debug.print(cur.hdg,0);  
    Debug.print(" hom.hdg=");Debug.print(hom.hdg,0);
    Debug.println("  Packet ignored");    
    return false;  
   }
}
  
return true;
}
//***************************************************
void CheckForTimeouts() {
  uint32_t cMillis = millis();
    if ((gpsGood==1) && (cMillis - millisGPS >= 5000)){
      gpsGood = 0;   // If no GPS packet for 5 seconds then GPS timeout 
      hbGood = 0;
      OledPrintln("No GPS packts/timeout");     
      #if defined Debug_All || defined Debug_FrSky || defined Debug_NMEA_GPS || defined Debug_LTM
        Debug.println("No GPS telemetry for 5 seconds"); 
      #endif  
    }
   ServiceTheStatusLed();
}

//***************************************************
void OledPrintln(String S) {
#if (Target_Board == 3) || (Target_Board == 4) 
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

  uint8_t lth = strlen(S.c_str());
  

    
  for (int i=0 ; i < lth ; i++ ) {
    OL[row].OLx[col] = S[i];
    col++;
    if (col > max_col-1) break;
  }
  
  for (col ; col < max_col-1; col++) {   //  flush to eol
    OL[row].OLx[col] = '\0';
  }
  col = 0;
  row++;
  
 // strncpy(OL[row].OLx, S.c_str(), max_col-1 );  
 // row++;
#endif  
}
//***************************************************
void OledPrint(String S) {
#if (Target_Board == 3) || (Target_Board == 4) 
  if (row>(max_row-1)) {  // last line    0 thru max_row-1
    
    display.clearDisplay();
    display.setCursor(0,0);
    
    for (int i = 0; i < (max_row-1); i++) {     // leave space for new line at the bottom
                                                //   if (i > 1) {   // don't scroll the 2 heading lines
      if (i >= 0) {         
        memset(OL[i].OLx, '\0', sizeof(OL[i].OLx));  // flush          
        strncpy(OL[i].OLx, OL[i+1].OLx, sizeof(OL[i+1].OLx));
      }
      display.print(OL[i].OLx);
    }
    display.display();
    row=max_row-1;
  }
  display.print(S);
  display.display();

  uint8_t lth = strlen(S.c_str());
  
  for (int i=0 ; i < lth ; i++ ) {
    OL[row].OLx[col] = S[i];
    col++;
    if (col > max_col-1) {
      break;
    }
  } 

  for (int i = col ; i < max_col-1; i++) {   //  flush to eol
    OL[row].OLx[i] = '\0';
  }
  
  if (col > max_col-1) {
    col = 0;
    row++;
   } 
#endif  
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

  if (homeInitialised) {
    EEPROMWritelong(0, epochPeriodic); // UPDATE epochHome
    #if defined Debug_All || defined Debug_EEPROM || defined Debug_Time || defined Debug_Home
      Debug.print("epochHome stored="); Debug.println(TimeString(epochPeriodic));
    #endif  
  }
  
  #if defined Debug_All || defined Debug_EEPROM || defined Debug_Time || defined Debug_Home
  Debug.print("epochPeriodic stored="); Debug.println(TimeString(epochPeriodic));
  #endif  
}

//***************************************************
uint32_t epochHome() {

uint32_t epHome = EEPROMReadlong(0);

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
  
  Debug.println("Home data restored from Flash"); 
  OledPrintln("Home data restored");
  OledPrintln("from Flash. Go Fly!");
  
  #if defined Debug_All || defined Debug_EEPROM || defined Debug_Time || defined Debug_Home
    Debug.print("  home.lon="); Debug.print(hom.lon, 6);
    Debug.print("  home.lat="); Debug.print(hom.lat, 6);
    Debug.print("  home.alt="); Debug.print(hom.alt, 0);
    Debug.print("  home.hdg="); Debug.println(hom.hdg, 0);
  #endif  
}
//***************************************************
  
#if (Telemetry_In == 2)     //  WiFi / Mavlink
 
  void SetupWiFi() { 

    bool apMode = false;  // used when STA fails to connect
    
     //*******************************  S T A T I O N   *****************************
    #if (WiFi_Mode == 2)  // STA
      uint8_t retry = 0;
      Debug.print("Trying to connect to ");  
      Debug.print(STAssid); 
      OledPrintln("WiFi trying ..");

      WiFi.disconnect(true);   // To circumvent "wifi: Set status to INIT" error bug
      delay(500);
      WiFi.mode(WIFI_STA);
      delay(500);
      
      WiFi.begin(STAssid, STApw);
      while (WiFi.status() != WL_CONNECTED){
        retry++;
        if (retry > 10) {
          Debug.println();
          Debug.println("Failed to connect in STA mode");
          OledPrintln("Failed in STA Mode");
          wifiSuDone = true;
          
          #ifdef AutoAP  
            apMode = true;            // Rather go establish an AP instead
            Debug.println("Starting AP instead.");
            OledPrintln("Starting AP instead");  
            //new from Target0815:
            Debug.println("WiFi-Reset ...");
            WiFi.mode(WIFI_MODE_NULL);    
            delay(500);             
          #endif  
          
          break;
        }
        delay(500);
        Serial.print(".");
      }
      
      if (WiFi.status() == WL_CONNECTED) {
        localIP = WiFi.localIP();
        Debug.println();
        Debug.println("WiFi connected!");
        Debug.print("Local IP address: ");
        Debug.print(localIP);

        #if   (WiFi_Protocol == 1)   // TCP
          Debug.print("  port: ");
          Debug.println(tcp_localPort);    //  UDP port is printed lower down
        #else 
          Debug.println();
        #endif 
         
        wifi_rssi = WiFi.RSSI();
        Debug.print("WiFi RSSI:");
        Debug.print(wifi_rssi);
        Debug.println(" dBm");

        OledPrintln("Connected!");
        OledPrintln(localIP.toString());
        
        #if (WiFi_Protocol == 1)   // TCP
          server.begin();                     // Not need now
          Debug.println("TCP server started");
          OledPrintln("TCP server started");
        #endif

        #if (WiFi_Protocol == 2)  // UDP
          udp.begin(udp_localPort);
          Debug.printf("UDP started, listening on IP %s, UDP port %d\n", localIP.toString().c_str(), udp_localPort);
          OledPrint("UDP port = ");  OledPrintln(String(udp_localPort));
        #endif
        
        wifiSuDone = true;
        wifiSuGood = true;
      } 
    #endif

     //*******************************  Access Point   *****************************
    #if (WiFi_Mode == 1)  // AP
      apMode = true;
    #endif

    if (apMode)   {
      WiFi.mode(WIFI_AP);
      WiFi.softAP(APssid, APpw, APchannel);
      localIP = WiFi.softAPIP();
      Debug.print("AP IP address: ");
      Debug.print (localIP); 
      Debug.print("  SSID: ");
      Debug.println(String(APssid));
      OledPrintln("WiFi AP SSID =");
      OledPrintln(String(APssid));
      
      #if (WiFi_Protocol == 1)      // TCP
        server.begin();             //  Server for TCP/IP traffic
        Debug.printf("TCP/IP started, listening on IP %s, TCP port %d\n", localIP.toString().c_str(), tcp_localPort);
        OledPrint("TCP port = ");  OledPrintln(String(tcp_localPort));
      #endif  

      #if (WiFi_Protocol == 2)      // UDP
        udp.begin(udp_localPort);
        Debug.printf("UDP started, listening on IP %s, UDP port %d\n", WiFi.softAPIP().toString().c_str(), udp_localPort);
        OledPrint("UDP port = ");  OledPrintln(String(udp_localPort));

      #endif 
      
      wifiSuGood = true;
 
    }           

      
    #ifndef Start_WiFi  // if not button override
      delay(2000);      // debounce button press
    #endif  

    wifiSuDone = true;
 }   


#if (Telemetry_In == 2) &&  (WiFi_Protocol == 2) //  WiFi && UDP - Display the remote UDP IP the first time we get it
  void DisplayRemoteIP() {
    if (FtRemIP)  {
      FtRemIP = false;
      Debug.print("Client connected: Remote UDP IP: "); Debug.print(udp_remoteIP);
      Debug.print("  Remote  UDP port: "); Debug.println(udp_remotePort);
      OledPrintln("Client connected");
      OledPrintln("Remote UDP IP =");
      OledPrintln(udp_remoteIP.toString());
      OledPrintln("Remote UDP port =");
      OledPrintln(String(udp_remotePort));
     }
  }
  #endif
  
 #endif  

//***************************************************
#if defined TestServos
  void TestServos() {
  PositionServos(90, 0, 90); 
    for (int i=1; i<=360; i++) {
      if (i<5 or (i>175 and i<185) or i>355){
        delay(150); //Take time to observe max positions
      }
      else {
        delay(20); 
      }
      PositionServos(i, 30, 90);   
    }
    digitalWrite(StatusLed,HIGH);
    delay(1000);
    digitalWrite(StatusLed,LOW);
    for (int i=1; i<=180; i++) {
      if (i<5 or i>175){
        delay(150); //Take time to observe max positions
      }
      else {
          delay(20); 
      }
      PositionServos(90, i, 90);   
    }
    PositionServos(90, 0, 90);
  
    //Test with various ReferenceHomeHeading
    digitalWrite(StatusLed,HIGH);
    delay(1000);
    digitalWrite(StatusLed,LOW);
    PositionServos(90, 0, 190); 
    for (int i=1; i<=360; i++) {
      delay(50);
      PositionServos(i, 45, 190);   
    }
    digitalWrite(StatusLed,HIGH);
    delay(1000);
    digitalWrite(StatusLed,LOW);
    for (int i=1; i<=180; i++) {
      delay(20);
      PositionServos(90, i, 190);   
    }
    digitalWrite(StatusLed,HIGH);
    delay(1000);
    digitalWrite(StatusLed,LOW);
    PositionServos(90, 0, 250); 
    for (int i=1; i<=360; i++) {
      delay(50);
      PositionServos(i, 45, 250);   
    }
    digitalWrite(StatusLed,HIGH);
    delay(1000);
    digitalWrite(StatusLed,LOW);
    for (int i=1; i<=180; i++) {
      delay(20);
      PositionServos(90, i, 250);   
    }
    digitalWrite(StatusLed,HIGH);
    delay(1000);
    digitalWrite(StatusLed,LOW);
    PositionServos(90, 0, 350); 
    for (int i=1; i<=360; i++) {
      delay(50);
      PositionServos(i, 45, 350);   
    }
    digitalWrite(StatusLed,HIGH);
    delay(1000);
    digitalWrite(StatusLed,LOW);
    for (int i=1; i<=180; i++) {
      delay(20);
      PositionServos(90, i, 350);   
    }
    PositionServos(90, 0, 350);
    digitalWrite(StatusLed,HIGH);
    delay(1000);
    digitalWrite(StatusLed,LOW);
    PositionServos(90, 0, 0);
    digitalWrite(StatusLed,HIGH);
    delay(1000);
    digitalWrite(StatusLed,LOW);
    PositionServos(90, 0, 90);
    digitalWrite(StatusLed,HIGH);
    delay(1000);
    digitalWrite(StatusLed,LOW);
    PositionServos(90, 0, 180);
    digitalWrite(StatusLed,HIGH);
    delay(1000);
    digitalWrite(StatusLed,LOW);
    PositionServos(90, 0, 270);
    digitalWrite(StatusLed,HIGH);
    delay(1000);
    digitalWrite(StatusLed,LOW);
    PositionServos(90, 0, 360);
    digitalWrite(StatusLed,HIGH);
    delay(1000);
    digitalWrite(StatusLed,LOW);
    PositionServos(90, 0, 90);
    //End of test with different ReferenceHomeHeading
  }
#endif
