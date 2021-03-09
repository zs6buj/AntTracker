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
    
   typedef enum frport_type_set { f_none = 0, f_port1 = 1, f_port2 = 2, s_port = 3, f_auto = 4} frport_t;  

   typedef enum polarity_set { idle_low = 0, idle_high = 1, no_traffic = 2 } pol_t;    
    
    // Protocol determination
    uint32_t baud = 0;
    uint8_t  protocol = 0;


    const uint8_t snp_max = 32;
    char          snprintf_buf[snp_max];       // for use with snprintf() formatting of display line

    // ************************************

    uint32_t val = 0;
    uint16_t addr = 0;
    uint8_t  ledState = LOW; 

    const uint8_t timeout_secs = 8;   
    
    uint32_t millisLED = 0;
    uint32_t millisStartup = 0;
    uint32_t millisDisplay = 0;

    uint32_t millisStore = 0;
    uint32_t millisSync = 0;
    uint32_t epochSync = 0;
    uint32_t millisFcHheartbeat = 0;
    uint32_t serGood_millis = 0;
    uint32_t packetloss_millis = 0;
    
    bool      hbGood = false; 
    bool      mavGood = false;   
    bool      cpsGood = false;
    bool      timeGood = false;
    bool      frGood = false;
    bool      frPrev = false; 
    bool      pwmGood = false; 
    bool      pwmPrev = false;
    bool      gpsGood = false; 
    bool      gpsPrev = false;    
    bool      serGood = false;            

    uint32_t  frGood_millis = 0;
    uint32_t  mavGood_millis = 0;   
    uint32_t  pwmGood_millis = 0;       
    uint32_t  gpsGood_millis = 0;

    uint32_t  goodFrames = 0;
    uint32_t  badFrames = 0;

    //====================
    bool  wifiSuDone = false;
    bool  wifiSuGood = false;
    bool  outbound_clientGood = false;
    bool  rxFT = true;
    bool  gotRecord = false; 
    bool  homSaved = false;    
    bool  homeInitialised = false;
    bool  new_GPS_data = false;
    bool  ftgetBaud = true;
    bool  lostPowerCheckDone = false;
    bool  timeEnabled = false;

    //  common variables for FrSky, LTM and MSP
    int16_t iLth=0;
    int16_t pLth;  // Packet length
    byte chr = 0x00;
    byte prev_chr = 0x00;    
    const int inMax = 70; 
    byte inBuf[inMax]; 

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
     
  struct Battery {
    float    mAh;
    float    tot_mAh;
    float    avg_cA;
    float    avg_mV;
    uint32_t prv_millis;
    uint32_t tot_volts;      // sum of all samples
    uint32_t tot_mW;
    uint32_t samples;
    bool ft;
  };
  
  struct Battery bat1     = {
     0, 0, 0, 0, 0, 0, 0, true};   

  struct Battery bat2     = {
    0, 0, 0, 0, 0, 0, 0, true};   

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
   #if (defined displaySupport)
     void PaintDisplay(uint8_t, last_row_t);
     void Scroll_Display(scroll_t);
     void SetupLogDisplayStyle();     
     void DisplayFlightInfo();
         
   #endif  
   void IRAM_ATTR gotButtonDn();
   void IRAM_ATTR gotButtonUp();
   void IRAM_ATTR gotWifiButton();
   void IRAM_ATTR gotButtonInfo();
   void SetupWiFi();
   bool NewOutboundTCPClient();  
 #endif

 bool PacketGood();
 void StoreEpochPeriodic();
 void SaveHomeToFlash();
 void PrintByte(byte b);
 void PrintMavBuffer(const void *object);
 void PrintFrsBuffer(byte *, uint8_t);
 void CheckForTimeouts();
 void LostPowerCheckAndRestore(uint32_t);
 uint32_t Get_Volt_Average1(uint16_t);  
 uint32_t Get_Current_Average1(uint16_t);

//***************************************************
void setup() {
  Log.begin(115200);
  delay(2000);
  Log.println();
  pgm_path = __FILE__;  // ESP8266 __FILE__ macro returns pgm_name and no path
  pgm_name = pgm_path.substring(pgm_path.lastIndexOf("\\")+1);  
  pgm_name = pgm_name.substring(0, pgm_name.lastIndexOf('.'));  // remove the extension
  Log.print("Starting "); Log.print(pgm_name);
  #if defined STM32F103C
    Log.print(" version:"); Log.print(MAJOR_VERSION);
    Log.print("."); Log.print(MINOR_VERSION); 
    Log.print("."); Log.println(PATCH_LEVEL);      
  #else
    Log.printf(" version:%d.%02d.%02d\n", MAJOR_VERSION,  MINOR_VERSION, PATCH_LEVEL);
  #endif     
  #if ((defined ESP32) || (defined ESP8266)) && (defined Debug_WiFi)
   WiFi.onEvent(WiFiEventHandler);   
  #endif  
 
  #if ((defined ESP32) || (defined ESP8266)) && (defined Debug_SRAM)
    Log.printf("Free Heap just after startup = %d\n", ESP.getFreeHeap());
  #endif  
//=================================================================================================   
//                                   S E T U P   D I S P L A Y
//=================================================================================================
  #if (defined displaySupport) 

    #if (defined ESP32)
    
      if (Tinfo != 99)  {   // enable info touch pin-pair
        touchAttachInterrupt(digitalPinToInterrupt(Tinfo), gotButtonInfo, threshold); 
       } else
      if (Pinfo != 99)  {   // enable info digital pin
        pinMode(Pinfo, INPUT_PULLUP);   
      }  
       
      if ( (Tup != 99) && (Tdn != 99) ) {   // enable touch pin-pair
        touchAttachInterrupt(digitalPinToInterrupt(Tup), gotButtonUp, threshold);
        touchAttachInterrupt(digitalPinToInterrupt(Tdn), gotButtonDn, threshold);   
      } else
      if ( (Pup != 99) && (Pdn != 99) ) {   // enable digital pin-pair
        pinMode(Pup, INPUT_PULLUP);
        pinMode(Pdn, INPUT_PULLUP);          
      }

    #endif  

    #if ((defined ESP8266) || (defined TEENSY3X))         
      if ( (Pup != 99) && (Pdn != 99) ) { // enable digital pin pair
        pinMode(Pup, INPUT_PULLUP);
        pinMode(Pdn, INPUT_PULLUP);
      }

    #endif 

    #if (defined ST7789_Display)               // LILYGO® TTGO T-Display ESP32 1.14" ST7789 Colour LCD (135 x 240)
      display.init(); 
      #define SCR_BACKGROUND TFT_BLACK
      
    #elif (defined SSD1306_Display)            // all  boards with SSD1306 OLED display (128 x 64)
      #if not defined TEENSY3X                 // Teensy uses default SCA and SCL in teensy "pins_arduino.h"
         Wire.begin(SDA, SCL);
      #endif   
      display.begin(SSD1306_SWITCHCAPVCC, i2cAddr);  
      #define SCR_BACKGROUND BLACK   
      
    #elif (defined SSD1331_Display)            // T2 board with SSD1331 colour TFT display (96 x 64)
        //  uses software SPI pins defined in config.h 
        display.begin();
        #define SCR_BACKGROUND BLACK  
        
    #elif (defined ILI9341_Display)    // ILI9341 2.8" COLOUR TFT SPI 240x320 V1.2 (320 x 240)
        //  uses hardware SPI pins defined in config.h 
        display.begin();
        #define SCR_BACKGROUND ILI9341_BLUE
    #endif
   
    #if (SCR_ORIENT == 0)              // portrait
        Log.print("Display Setup: Portrait ");         
    #elif (SCR_ORIENT == 1)            // landscape   
        Log.println("Display support activated: Landscape "); 
    #endif

    int eol= 0;
    for (eol = 0 ; eol < SCR_W_CH ; eol++) {
      clear_line[eol] = ' ';
    }
    clear_line[eol] = 0x00;
    #if defined STM32F103C
      Log.print(SCR_H_PX); Log.print("x"); Log.print(SCR_W_PX);
      Log.print("  TEXT_SIZE:"); Log.print(TEXT_SIZE); 
      Log.print("  CHAR_W_PX:"); Log.print(CHAR_W_PX);  
      Log.print("  CHAR_H_PX:"); Log.print(CHAR_H_PX); 
      Log.print("  SCR_H_CH:"); Log.print(SCR_H_CH);  
      Log.print("  SCR_W_CH:"); Log.println(SCR_W_CH);                    
    #else
      Log.printf("%dx%d  TEXT_SIZE:%d   CHAR_W_PX:%d  CHAR_H_PX:%d   SCR_H_CH:   SCR_W_CH:%d \n", SCR_H_PX, SCR_W_PX, TEXT_SIZE, CHAR_W_PX, CHAR_H_PX, SCR_H_CH, SCR_W_CH);
    #endif  
    
    LogScreenPrintln("Starting .... ");
  #endif
  /*
  display.setFont(Dialog_plain_8);     //  col=24 x row 8  on 128x64 display
  display.setFont(Dialog_plain_16);    //  col=13 x row=4  on 128x64 display
  */
  #if ((defined ESP32) || (defined ESP8266)) && (defined Debug_SRAM)
    Log.printf("==============>Free Heap after OLED setup = %d\n", ESP.getFreeHeap());
  #endif
  
//=================================================================================================   
    
  LogScreenPrintln("AntTracker by zs6buj");
  Log.print("Target Board is ");
  #if (defined TEENSY3X) // Teensy3x
    Log.println("Teensy 3.x");
    LogScreenPrintln("Teensy 3.x");
  #elif (defined ESP32) //  ESP32 Board
    Log.print("ESP32 / Variant is ");
    LogScreenPrintln("ESP32 / Variant is");
    #if (ESP32_Variant == 1)
      Log.println("Dev Module");
      LogScreenPrintln("Dev Module");
    #endif
    #if (ESP32_Variant == 4)
      Log.println("Heltec Wifi Kit 32");
      LogScreenPrintln("Heltec Wifi Kit 32");
    #endif
    #if (ESP32_Variant == 5)
      Log.println("LILYGO® TTGO T-Display ESP32 1.14 inch ST7789 Colour LCD");
      LogScreenPrintln("TTGO T-Display ESP32");
    #endif   
    #if (ESP32_Variant == 6)
      Log.println("LILYGO® TTGO T2 ESP32 OLED SD");
      LogScreenPrintln("TTGO T2 ESP32 SD");
    #endif     
  #elif (defined ESP8266) 
    Log.print("ESP8266 / Variant is ");
    LogScreenPrintln("ESP8266 / Variant is");  
    #if (ESP8266_Variant == 1)
      Log.println("Lonlin Node MCU 12F");
      LogScreenPrintln("Node MCU 12");
    #endif      
  #endif
  
  #if (Telemetry_In == 0)  // Serial
    Log.println("Mavlink Serial In");
    LogScreenPrintln("Mavlink Serial In");
  #endif  

  #if (Telemetry_In == 1)  // BT
    Log.println("Mavlink BT In");
    LogScreenPrintln("Mavlink BT In");
  #endif  

  #if (Telemetry_In  == 2)  // WiFi
    Log.println("Mavlink WiFi In");
    LogScreenPrintln("Mavlink WiFi In");
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
    Log.println("QLRS variant of Mavlink expected"); 
    LogScreenPrintln("QLRS Mavlink expected");
  #endif  
  
  if (Heading_Source == 3)  {// Tracker_Compass

    cpsGood = Initialise_Compass();  // Check if we have a compass on the Tracker
    
    if (!(cpsGood)) {
      LogScreenPrintln("No compass found!");
      
      #if defined Debug_Minimum || defined Debug_All || defined Debug_Compass  
        Log.println("Heading_Source = Tracker's Own Compass, but no compass found! Aborting."); 
      #endif  
      while (1) delay (1000);  // Wait here forever
    }
   
    #if defined Debug_All || defined Debug_Compass
      Log.println("Display tracker heading for 20 seconds");
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

      pol_t pol = (pol_t)getPolarity(rxPin);
      bool ftp = true;
      static int8_t cdown = 30; 
      bool polGood = true;    
      while ( (pol == no_traffic) && (cdown) ){
        if (ftp) {
          Log.printf("No telem on rx pin:%d. Retrying ", rxPin);
          String s_rxPin=String(rxPin);   // integer to string
          LogScreenPrintln("No telem on rxpin:"+ s_rxPin); 
          ftp = false;
        }
        Log.print(cdown); Log.print(" ");
        pol = (pol_t)getPolarity(rxPin);
        delay(500);      
        if (cdown-- == 1 ) {
          Log.println();
          Log.println("Auto sensing abandoned. Defaulting to IDLE_HIGH 57600 b/s");
          LogScreenPrintln("Default to idle_high");
          polGood = false;
   
        }
      }
      
    if (polGood) {    // expect 57600 for Mavlink and FrSky, 2400 for LTM, 9600 for MSP & GPS
      if (pol == idle_low) {
        rxInvert = true;
        Log.printf("Serial port rx pin %d is IDLE_LOW, inverting rx polarity\n", rxPin);
      } else {
        rxInvert = false;
        Log.printf("Serial port rx pin %d is IDLE_HIGH, regular rx polarity retained\n", rxPin);        
      }     
      baud = getBaud(rxPin);
      Log.print("Baud rate detected is ");  Log.print(baud); Log.println(" b/s"); 
      String s_baud=String(baud);   // integer to string. "String" overloaded
      LogScreenPrintln("Telem at "+ s_baud);
      protocol = detectProtocol(baud);
    //Log.printf("Protocol:%d\n", protocol);
      
    } else {
      pol = idle_high;
      baud = 57600;
      protocol = 2;  
    }
    

    #if ( (defined ESP8266) || (defined ESP32) ) 
      delay(100);
      inSerial.begin(baud, SERIAL_8N1, rxPin, txPin, rxInvert); 
      delay(10);
    #elif (defined TEENSY3X) 
      frSerial.begin(frBaud); // Teensy 3.x    tx pin hard wired
       if (rxInvert) {          // For S.Port not F.Port
         UART0_C3 = 0x10;       // Invert Serial1 Tx levels
         UART0_S2 = 0x10;       // Invert Serial1 Rx levels;       
       }
       #if (defined frOneWire )  // default
         UART0_C1 = 0xA0;        // Switch Serial1 to single wire (half-duplex) mode  
       #endif    
    #else
      inSerial.begin(baud);
    #endif   


    switch(protocol) {
    
      case 1:    // Mavlink 1
        LogScreenPrintln("Mavlink 1 found");
        Log.println("Mavlink 1 found"); 
        timeEnabled = true;
        break;
      case 2:    // Mavlink 2
        LogScreenPrintln("Mavlink 2 found");
        Log.println("Mavlink 2 found");
        timeEnabled = true;
        break;
      case 3:    // FrSky S.Port 
        LogScreenPrintln("FrSky S.Port found");
        Log.println("FrSky S.Port found");       
        break;
      case 4:    // FrSky F.Port 1
        LogScreenPrintln("FrSky F.Portl found");
        Log.println("FrSky F.Port1 found");       
        break;
      case 5:    // FrSky F.Port 2
        LogScreenPrintln("FrSky F.Port2 found");
        Log.println("FrSky F.Port2 found");       
        break; 
      case 6:    // LTM protocol found  
        LogScreenPrintln("LTM protocol found"); 
        Log.println("LTM protocol found");       
        break;     
      case 7:    // MSP protocol found 
        LogScreenPrintln("MSP protocol found"); 
        Log.println("MSP protocol found");   
        break; 
      case 8:    // GPS NMEA protocol found 
        LogScreenPrintln("NMEA protocol found"); 
        Log.println("NMEA protocol found"); 
        set_up_GPS(); 
        if (Heading_Source == 2) {  // Flight Computer !! If we have found the NMEA protol then FC is unlikely
          LogScreenPrintln("Check heading source!");
          LogScreenPrintln("Aborting...");
          Log.println("Check heading source! Aborting...");
          while (1) delay (1000);  // Wait here forever
        }
        timeEnabled = true;   
        break;     
      default:   // Unknown protocol 
        LogScreenPrintln("Unknown protocol!");
        LogScreenPrintln("Aborting....");        
        while(1) delay(1000);  // wait here forever                        
    }
   
  #endif
  
// ************************ Setup Bluetooth ***************************  
  #if (defined ESP32)  && (Telemetry_In == 1) 
    #ifdef BT_Master_Mode
      SerialBT.begin("AntTrack", true);            
    #else
        SerialBT.begin("AntTrack");   
    #endif 
      
      bool bt_connected;

      bt_connected = SerialBT.connect(BT_Slave_Name);
     if(bt_connected) {
          Log.println("Bluetooth connected!");
          LogScreenPrintln("Bluetooth connected!");
      }    
      
  #endif 
  
  // ************************* Setup WiFi **************************** 
  #if (defined ESP32)  || (defined ESP8266)

   #if (Telemetry_In == 2) || (Telemetry_In == 3)  //  WiFi Mavlink or FrSky
     SetupWiFi();  
   #endif
   
 #endif  
      
}


// *******************************************************************************************

void loop() {            
  
  #if (Telemetry_In == 1)         // Bluetooth
    Mavlink_Receive();
  #endif

  #if (Telemetry_In == 2)         // Mavlink WiFi
  
    #if (WiFi_Protocol == 1)      // Mav TCP
      if (outbound_clientGood) {     
        Mavlink_Receive();
      }
    #endif
    #if (WiFi_Protocol == 2)      // Mav UDP 
      Mavlink_Receive();
    #endif   
  #endif

  #if (Telemetry_In == 3)         //   FrSky UDP
      if (wifiSuGood) {     
        FrSky_Receive(9);
      }
    #endif
  
  #if (Telemetry_In == 0)      // Serial according to detected protocol
    switch(protocol) {
    
      case 1:    // Mavlink 1
        Mavlink_Receive();               
        break;
      case 2:    // Mavlink 2
        Mavlink_Receive();
        break;
      case 3:    // S.Port
        FrSky_Receive(3);                     
        break;
       case 4:    // F.Port1
        FrSky_Receive(4);                     
        break;
       case 5:    // F.Port2
        FrSky_Receive(5);                     
        break;              
      case 6:    // LTM   
        LTM_Receive();   
        break;  
      case 7:    // MSP
 //     MSP_Receive(); 
        break;
      case 8:    // GPS
        GPS_Receive(); 
        break;
      default:   // Unknown protocol 
        LogScreenPrintln("Unknown protocol!");  
        while(1) delay(1000);  // wait here forever                     
    }  
   #endif

      //===============================       C h e c k   F o r   N e w   A P   C o n n e c t s
   
      #if (Telemetry_In == 2)    // WiFi
  
      AP_sta_count = WiFi.softAPgetStationNum();
      if (AP_sta_count > AP_prev_sta_count) {  // a STA device has connected to the AP
        AP_prev_sta_count = AP_sta_count;
        Log.printf("Remote STA %d connected to our AP\n", AP_sta_count);  
        snprintf(snprintf_buf, snp_max, "New STA, total=%d", AP_sta_count);        
        LogScreenPrintln(snprintf_buf); 
        #if (WiFi_Protocol == 1)  // TCP
          if (!outbound_clientGood) {// if we don't have an active tcp session, start one
            outbound_clientGood = NewOutboundTCPClient();
          } 
        #endif               
      } else 
      if (AP_sta_count < AP_prev_sta_count) {  // a STA device has disconnected from the AP
        AP_prev_sta_count = AP_sta_count;
        Log.println("A STA disconnected from our AP");     // back in listening mode
        LogScreenPrintln("A STA disconnected"); 
      }

      #endif
    
      //====================  Check For Display Button Touch / Press
  
      #if defined displaySupport
        HandleDisplayButtons();
      #endif   

      //==================== Send Our Own Heartbeat to FC to trigger tardy telemetry

      if(millis()- millisFcHheartbeat > 2000) {  // MavToPass heartbeat to FC every 2 seconds
      millisFcHheartbeat=millis();       
      Send_FC_Heartbeat();                     // for serial must have tx pin connected to dedicated telem radio rx pin  
      }
     
      //===============================

      CheckForTimeouts();
   
      //===============================  
    
      ServiceTheStatusLed();  
    
    //=============================== 
    if ( ((timeEnabled) && (lostPowerCheckDone)) || (!timeEnabled)  ) {
      if (gpsGood==1 && ft) {
        ft=false;
        if (homeInitialised==0) {
          if (Heading_Source == 1)  { // GPS
            Log.println("GPS lock good! Walk straight ahead 10m then push home button");  
            LogScreenPrintln("GPS lock good. Carry ");
            LogScreenPrintln("craft fwrd 10m. Place");
            LogScreenPrintln(" and push home button");
          }
         else {  // FC & own tracker compass
            Log.println("GPS lock good! Push set-home button anytime to start tracking.");  
            LogScreenPrintln("GPS lock good! Push");
            LogScreenPrintln("home button");
          }
        }
      }
    }

    if (homeInitialised) {
      if (hbGood && gpsGood && PacketGood() && new_GPS_data) {  //  every time there is new GPS data 
        GetAzEl(hom, cur);
        if (hc_vector.dist >= minDist) PositionServos(hc_vector.az, hc_vector.el, hom.hdg);  // Relative to home heading
        new_GPS_data = false;
      }
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

          LogScreenPrintln("Heading calculated");
          LogScreenPrintln("Tracking now active!");
        
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
        LogScreenPrintln("No Heading_Source!");
        LogScreenPrintln("Aborting");
        while(1) delay(1000);  // wait here forever                     
    }
     SaveHomeToFlash(); 
 }
//***************************************************
void AutoStoreHome() {
  #ifdef Debug_Status
    Log.println("homSaved=true");  
  #endif
  hom.lat = cur.lat;
  hom.lon = cur.lon;
  hom.alt = cur.alt;

  // homSaved set true in SaveHomeToFlash
  SaveHomeToFlash();  
  homSaved = true;
  
  LogScreenPrintln("Home loctn auto-stord"); 

  #if defined Debug_All || defined Debug_Status
    Log.print("Home auto stored: ");       
    Log.print("hom.lat=");  Log.print(hom.lat, 7);
    Log.print(" hom.lon="); Log.print(hom.lon, 7 );        
    Log.print(" hom.alt="); Log.println(hom.alt, 1);                 
 #endif 
   
} 
//***************************************************
void DisplayHome() { 
  LogScreenPrintln("Home location set");
  LogScreenPrintln("Tracking now active!");
  #if defined Debug_Minimum || defined Debug_All || defined Debug_AzEl
 //   Log.print("******************************************");
    Log.print("Home location set to Lat = "); Log.print(hom.lat,7);
    Log.print(" Lon = "); Log.print(hom.lon,7);
    Log.print(" Alt = "); Log.print(hom.alt,0); 
    Log.print(" hom.hdg = "); Log.println(hom.hdg,0); 
 //   DisplayHeadingSource();
  #endif 
}
//***************************************************
void DisplayHeadingSource() {
#if defined Debug_Minimum || defined Debug_All || defined Debug_Compass  
  if (Heading_Source == 1)  {
      Log.println("Heading_Source = Craft's GPS"); 
      LogScreenPrintln("HdgSrce=Craft's GPS");
  }
  else if  (Heading_Source == 2) { 
      Log.println("Heading_Source = Flight Computer");
      LogScreenPrintln("Heading_Source = FC");
  }
  else if (Heading_Source == 3)  {
      Log.println("Heading_Source = Tracker's Own Compass"); 
      LogScreenPrintln("HdgSrce=Tracker GPS");
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
    Log.print("hbGood = ");
    Log.print(hbGood);
    Log.print("   gpsGood = ");
    Log.print(gpsGood);
    Log.print("   homeInitialised = ");
    Log.println(homeInitialised);
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
