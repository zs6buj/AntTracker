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
   STM32 Black Pill STM32F104
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
#include "config.h"                      // ESP_IDF libs included here
#include <ardupilotmega/mavlink.h>
#include <ardupilotmega/ardupilotmega.h>
#include <SPI.h>
#include <Wire.h>

String    pgm_path;
String    pgm_name;

bool pb_rx = true;

// Protocol determination
uint32_t baud = 0;
uint8_t  protocol = 0;


const uint8_t snp_max = 32;
char          snprintf_buf[snp_max];       // for use with snprintf() formatting of display line

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
uint32_t millisFcHheartbeat = 0;

//*************
bool  wifiSuDone = false;
bool  wifiSuGood = false;
bool  outbound_clientGood = false;
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

//=================================================================================================   
//                     F O R W A R D    D E C L A R A T I O N S
//=================================================================================================



 #if (defined ESP32) || (defined ESP8266)
   #if (defined Display_Support)
     void PaintDisplay(uint8_t, last_row_t);
     void Scroll_Display(scroll_t);
   #endif  
   void IRAM_ATTR gotButtonDn();
   void IRAM_ATTR gotButtonUp();
   void IRAM_ATTR gotWifiButton();
   void SetupWiFi();
   bool NewOutboundTCPClient();  
 #endif

 bool PacketGood();
 void StoreEpochPeriodic();
 void SaveHomeToFlash();
 void PrintByte(byte b);
 void PrintMavBuffer(const void *object);
 
//***************************************************
void setup() {
  Debug.begin(115200);
  delay(2000);
  Debug.println();
  pgm_path = __FILE__;  // ESP8266 __FILE__ macro returns pgm_name and no path
  pgm_name = pgm_path.substring(pgm_path.lastIndexOf("\\")+1);  
  pgm_name = pgm_name.substring(0, pgm_name.lastIndexOf('.'));  // remove the extension
  Debug.print("Starting "); Debug.print(pgm_name); Debug.println(" .....");
    
  #if ((defined ESP32) || (defined ESP8266)) && (defined Debug_WiFi)
   WiFi.onEvent(WiFiEventHandler);   
  #endif  
 
  #if ((defined ESP32) || (defined ESP8266)) && (defined Debug_SRAM)
    Debug.printf("Free Heap just after startup = %d\n", ESP.getFreeHeap());
  #endif  
//=================================================================================================   
//                                 S E T U P   D I S P L A Y
//=================================================================================================
  #if (defined Display_Support) 

    #if (defined ESP32)
      if ( (Tup != 99) && (Tdn != 99) ) {   // enable touch pin-pair
        touchAttachInterrupt(digitalPinToInterrupt(Tup), gotButtonUp, threshold);
        touchAttachInterrupt(digitalPinToInterrupt(Tdn), gotButtonDn, threshold);
      } else
      if ( (Pup != 99) && (Pdn != 99) ) {   // enable digital pin-pair
        pinMode(Pup, INPUT_PULLUP);
        pinMode(Pdn, INPUT_PULLUP);          
      }

    #endif  

    #if (defined ESP8266)           // interrupt on ESP8266 is momentary, not what we need
      if ( (Pup != 99) && (Pdn != 99) ) { // enable digital pin pair
        pinMode(Pup, INPUT_PULLUP);
        pinMode(Pdn, INPUT_PULLUP);
      }

    #endif 

    #if (defined ST7789_Display)      // LILYGO® TTGO T-Display ESP32 1.14" ST7789 Colour LCD
      display.init();
      if (screenOrientation == 0) {   // portrait
        display.setRotation(0);       // or 4 
        display.setTextSize(1);
        display.setTextFont(0);       // Original Adafruit font 0, try 0 thru 6
      } else 
      if (screenOrientation == 1) {   // landscape
        display.setRotation(3);       // or 1
        display.setTextSize(2);  
        display.setTextFont(1);    
      }      

      #define screenBackground TFT_BLACK
      
      display.fillScreen(screenBackground);
      display.setTextColor(TFT_SKYBLUE);    
            
  //    display.setTextColor(TFT_WHITE);
  //    display.setTextColor(TFT_BLUE);  
  //    display.setTextColor(TFT_GREEN, TFT_BLACK);
    
    #elif (defined SSD1306_Display)            // all other boards with SSD1306 OLED display
      Wire.begin(SDA, SCL);
      display.begin(SSD1306_SWITCHCAPVCC, i2cAddr); 
      display.clearDisplay(); 
      display.setTextColor(WHITE);  
      display.setTextSize(1);  
      #define screenBackground BLACK  
      #define screen_height    8     // characters not pixels
      #define screen_width    21  
    #endif
    
    display.setCursor(0,0);
  
    Debug.println("Display support activated");

  #endif
  /*
  display.setFont(Dialog_plain_8);     //  col=24 x row 8  on 128x64 display
  display.setFont(Dialog_plain_16);    //  col=13 x row=4  on 128x64 display
  */
//=================================================================================================   
    
  DisplayPrintln("AntTracker by zs6buj");
  Debug.print("Target Board is ");
  #if (defined TEENSY3X) // Teensy3x
    Debug.println("Teensy 3.x");
    DisplayPrintln("Teensy 3.x");
  #elif (defined ESP32) //  ESP32 Board
    Debug.print("ESP32 / Variant is ");
    DisplayPrintln("ESP32 / Variant is");
    #if (ESP32_Variant == 1)
      Debug.println("Dev Module");
      DisplayPrintln("Dev Module");
    #endif
    #if (ESP32_Variant == 4)
      Debug.println("Heltec Wifi Kit 32");
      DisplayPrintln("Heltec Wifi Kit 32");
    #endif
    #if (ESP32_Variant == 5)
      Debug.println("LILYGO® TTGO T-Display ESP32 1.14 inch ST7789 Colour LCD");
      DisplayPrintln("TTGO T-Display ESP32");
    #endif   
    #if (ESP32_Variant == 6)
      Debug.println("LILYGO® TTGO T2 ESP32 OLED SD");
      DisplayPrintln("TTGO T2 ESP32 SD");
    #endif     
  #elif (defined ESP8266) 
    Debug.print("ESP8266 / Variant is ");
    DisplayPrintln("ESP8266 / Variant is");  
    #if (ESP8266_Variant == 1)
      Debug.println("Lonlin Node MCU 12F");
      DisplayPrintln("Node MCU 12");
    #endif      
  #endif
  
  #if (Telemetry_In == 0)  // Serial
    Debug.println("Mavlink Serial In");
    DisplayPrintln("Mavlink Serial In");
  #endif  

  #if (Telemetry_In == 1)  // BT
    Debug.println("Mavlink BT In");
    DisplayPrintln("Mavlink BT In");
  #endif  

  #if (Telemetry_In  == 2)  // WiFi
    Debug.println("Mavlink WiFi In");
    DisplayPrintln("Mavlink WiFi In");
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
    DisplayPrintln("QLRS Mavlink expected");
  #endif  
  
  if (Heading_Source == 3)  {// Tracker_Compass

    cpsGood = Initialise_Compass();  // Check if we have a compass on the Tracker
    
    if (!(cpsGood)) {
      DisplayPrintln("No compass found!");
      
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
    
  //TestServos();   // Uncomment this code to observe how well your servos reach their specified limits
                    // Fine tune MaxPWM and MinPWM in Servos module

// ************************ Setup Serial ******************************

  #if (Telemetry_In == 0)    //  Serial

    protocol = GetProtocol();

    #if ( (defined ESP8266) || (defined ESP32) ) 
      inSerial.begin(baud, SERIAL_8N1, rxPin, txPin, frInvert); 
    #elif (defined TEENSY3X) 
      frSerial.begin(frBaud); // Teensy 3.x    tx pin hard wired
       if (frInvert) {          // For S.Port not F.Port
         UART0_C3 = 0x10;       // Invert Serial1 Tx levels
         UART0_S2 = 0x10;       // Invert Serial1 Rx levels;       
       }
       #if (defined frOneWire )  // default
         UART0_C1 = 0xA0;        // Switch Serial1 to single wire (half-duplex) mode  
       #endif    
    #else
      inSerial.begin(baud);
    #endif   
            
    // expect 57600 for Mavlink and FrSky, 2400 for LTM, 9600 for MSP & GPS
    switch(protocol) {
    
      case 1:    // Mavlink 1
        DisplayPrintln("Mavlink 1 found");
        Debug.println("Mavlink 1 found"); 
        timeEnabled = true;
        break;
      case 2:    // Mavlink 2
        DisplayPrintln("Mavlink 2 found");
        Debug.println("Mavlink 2 found");
        timeEnabled = true;
        break;
      case 3:    // FrSky protocol  
        DisplayPrintln("FrSky protocol found");
        Debug.println("FrSky protocol found");       
        break;
      case 4:    // LTM protocol found  
        DisplayPrintln("LTM protocol found"); 
        Debug.println("LTM protocol found");       
        break;     
      case 5:    // MSP protocol found 
        DisplayPrintln("MSP protocol found"); 
        Debug.println("MSP protocol found");   
        break; 
      case 6:    // GPS NMEA protocol found 
        DisplayPrintln("NMEA protocol found"); 
        Debug.println("NMEA protocol found"); 
        set_up_GPS(); 
        if (Heading_Source == 2) {  // Flight Computer !! If we have found the NMEA protol then FC is unlikely
          DisplayPrintln("Check heading source!");
          DisplayPrintln("Aborting...");
          Debug.println("Check heading source! Aborting...");
          while (1) delay (1000);  // Wait here forever
        }
        timeEnabled = true;   
        break;     
      default:   // Unknown protocol 
        DisplayPrintln("Unknown protocol!");
        DisplayPrintln("Aborting....");        
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
          DisplayPrintln("Bluetooth connected!");
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

void loop() {            
  
  #if (Telemetry_In == 1)       // Bluetooth
    Mavlink_Receive();
  #endif

  #if (Telemetry_In == 2)       //   WiFi
    #if (WiFi_Protocol == 1)    // TCP/IP
      if (outbound_clientGood) {     
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
        DisplayPrintln("Unknown protocol!");  
        while(1) delay(1000);  // wait here forever                     
    }  
   #endif

   //===============================       C h e c k   F o r   N e w   A P   C o n n e c t s
   
    #if (Telemetry_In == 2)    // WiFi
  
      AP_sta_count = WiFi.softAPgetStationNum();
      if (AP_sta_count > AP_prev_sta_count) {  // a STA device has connected to the AP
        AP_prev_sta_count = AP_sta_count;
        Debug.printf("Remote STA %d connected to our AP\n", AP_sta_count);  
        snprintf(snprintf_buf, snp_max, "New STA, total=%d", AP_sta_count);        
        DisplayPrintln(snprintf_buf); 
        #if (WiFi_Protocol == 1)  // TCP
          if (!outbound_clientGood) {// if we don't have an active tcp session, start one
            outbound_clientGood = NewOutboundTCPClient();
          } 
        #endif               
      } else 
      if (AP_sta_count < AP_prev_sta_count) {  // a STA device has disconnected from the AP
        AP_prev_sta_count = AP_sta_count;
        Debug.println("A STA disconnected from our AP");     // back in listening mode
        DisplayPrintln("A STA disconnected"); 
      }

    #endif
    
  //===============   Check For Display Scroll Button Touch / Press
  
  #if defined Display_Support
  
    #if (defined ESP32)             
      if ( (Tup != 99) && (Tdn != 99) ) {   // if touch pin-pair enabled
        if (upButton) {
          Scroll_Display(up);
        }
        if (dnButton) {
          Scroll_Display(down);
        } 
      } else
      if ( (Pup != 99) && (Pdn != 99) ) {   // if digital pin-pair enabled
        upButton = digitalRead(Pup);
        if (!upButton) {                 // low == pressed
          Scroll_Display(up);
        }
        dnButton = digitalRead(Pdn);
        if (!dnButton) {
          Scroll_Display(down);
        }        
      }
    #endif
    
    #if (defined ESP8266)                 // digital pin interrupt is momentary, not what we need
      if ( (Pup != 99) && (Pdn != 99) ) { // 99 means pins not avaliable, maybe not enough pins
        upButton = digitalRead(Pup);
        if (!upButton) {                 // low == pressed
          Scroll_Display(up);
        }
        dnButton = digitalRead(Pdn);
        if (!dnButton) {
          Scroll_Display(down);
        }    
      }
    #endif
    
  #endif

    //==================== Send Our Own Heartbeat to FC to trigger tardy telemetry

    if(millis()- millisFcHheartbeat > 2000) {  // MavToPass heartbeat to FC every 2 seconds
      millisFcHheartbeat=millis();       
      Send_FC_Heartbeat();                     // for serial must have tx pin connected to dedicated telem radio rx pin  
    }
     
   //===============================
  
    ServiceTheStatusLed();  

    if ( ((timeEnabled) && (lostPowerCheckDone)) || (!timeEnabled)  ) {
      if (gpsGood==1 && ft) {
        ft=false;
        if (homeInitialised==0) {
          if (Heading_Source == 1)  { // GPS
            Debug.println("GPS lock good! Walk straight ahead 10m then push home button");  
            DisplayPrintln("GPS lock good. Carry ");
            DisplayPrintln("craft fwrd 10m. Place");
            DisplayPrintln(" and push home button");
          }
         else {  // FC & own tracker compass
            Debug.println("GPS lock good! Push set-home button anytime to start tracking.");  
            DisplayPrintln("GPS lock good! Push");
            DisplayPrintln("home button");
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

          DisplayPrintln("Heading calculated");
          DisplayPrintln("Tracking now active!");
        
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
        DisplayPrintln("No Heading_Source!");
        DisplayPrintln("Aborting");
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
  
  DisplayPrintln("Home loctn auto-stord"); 

  #if defined Debug_All || defined Debug_Status
    Debug.print("Home auto stored: ");       
    Debug.print("hom.lat=");  Debug.print(hom.lat, 7);
    Debug.print(" hom.lon="); Debug.print(hom.lon, 7 );        
    Debug.print(" hom.alt="); Debug.println(hom.alt, 1);                 
 #endif 
   
} 
//***************************************************
void DisplayHome() { 
  DisplayPrintln("Home location set");
  DisplayPrintln("Tracking now active!");
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
      DisplayPrintln("HdgSrce=Craft's GPS");
  }
  else if  (Heading_Source == 2) { 
      Debug.println("Heading_Source = Flight Computer");
      DisplayPrintln("Heading_Source = FC");
  }
  else if (Heading_Source == 3)  {
      Debug.println("Heading_Source = Tracker's Own Compass"); 
      DisplayPrintln("HdgSrce=Tracker GPS");
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
