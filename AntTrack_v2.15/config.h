
/*

  Complete change log and debugging options are at the bottom of this tab
   
v2.15 2020-10-12 Proper TCP client added for outgoing (to telemetry source) connect   
                 Display scrolling added
                 TCP in tests good.

`                    
*/
// ******************************* Please select your options here before compiling *******************************
#define Device_sysid     251                     // Our Mavlink Identity - APM FC is 1, Mission Planner is 255, QGC default is 0 
#define Device_compid    MAV_COMP_ID_PERIPHERAL  // 158 Generic autopilot peripheral - APM FC is 1, MP is 190, QGC is  https://mavlink.io/en/messages/common.html

//#define Display_Support                 // Enable if you have a display attached - choose default display type below
#define SSD1306_Display                 // OLED display type - if you have a display you must define which type
//#define ST7789_Display                // TFT display type - if you have a display you must define which type 

// Choose one only of these input channels 
// How does telemetry enter the tracker?
//#define Telemetry_In  0    // Serial Port (default) - all protocols        
//#define Telemetry_In  1    // BlueTooth Classic - ESP32 and Mavlink only
#define Telemetry_In  2    // WiFi - ESP32 and Mavlink only


// NOTE: The Bluetooth class library uses a lot of application memory. During Compile/Flash
//  you may need to select Tools/Partition Scheme: "Minimal SPIFFS (1.9MB APP ...)

//#define BT_Master_Mode true    // Master connects to BT_Slave_Name --- false for BT Slave Mode
const char* BT_Slave_Name   =   "Crossfire 0277";  // Example

#define AutoBaud              // UART Serial Only - Auto detect telemetry speed

// Choose one - for ESP32 and Mavlink only
#define WiFi_Protocol 1    // TCP/IP
//#define WiFi_Protocol 2    // UDP     useful for Ez-WiFiBroadcast in STA mode

// Choose one - AP means advertise as an access point (hotspot). STA means connect to a known host AP
//#define WiFi_Mode   1  //AP            
#define WiFi_Mode   2  // STA


#define AutoAP                      // If we fail to connect in STA mode, start AP instead


// Un-comment (activate) the options below
//#define Az_Servo_360   // Means the azimuth servo can point in a 360 deg circle, elevation servo 90 deg
                         // Default (comment out #define above) is 180 deg azimuth and 180 deg elevation 
                         
const uint8_t Heading_Source =  2;  // 1=GPS, 2=Flight Computer, 3=Tracker_Compass   
                     
//#define QLRS           // Un-comment if you use the QLRS variant of Mavlink 

//*********************************************************************************************
//**********************   S E L E C T   E S P   B O A R D   V A R I A N T   ******************

#define ESP32_Variant     1    //  ESP32 Dev Module - there are several sub-variants that work
//#define ESP32_Variant     4    //  Heltec Wifi Kit 32 
//#define ESP32_Variant     5    //  LILYGO® TTGO T-Display ESP32 1.14" ST7789 Colour LCD
//#define ESP32_Variant     6    //  DON'T USE ME YET !!  LILYGO® TTGO T2 SD

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


// ****************************************** Auto Determine Target Platform **************************************
//
//                Don't change anything here
//
#if defined (__MK20DX128__) || defined(__MK20DX256__)
  #define Target_Board   0      // Teensy 3.1 and 3.2    
      
#elif defined (__BluePill_F103C8__) ||  defined (MCU_STM32F103RB)
  #define Target_Board   1      // Blue Pill STM32F103C  
         
#elif defined (STM32_MEDIUM_DENSITY) 
  #define Target_Board   2      // Maple_Mini STM32F103C  
     
#elif defined (_BOARD_MAPLE_MINI_H_)
  // LeafLabs high density
  #define Target_Board   2      // Maple_Mini 

#elif defined STM32_HIGH_DENSITY
  // LeafLabs high density
  #define Target_Board   2      // Maple_Mini 
   
#elif defined ESP32
  #define Target_Board   3      // Espressif ESP32 Dev Module

#elif defined ESP8266
  #define Target_Board   4      // Espressif ESP8266
  
#else
  #error "No board type defined!"
#endif


  #if (defined ESP32 || defined ESP8266) && (Telemetry_In == 2)
    #define wifiBuiltin   //  for this feature we need wifi support compiled in
  #endif    

   #if (defined ESP32 || defined ESP8266) && (Telemetry_In == 1)
    #define btBuiltin   //  for this feature we need bluetooth support compiled in
  #endif



//************************************** Macro Logic Checks ************************************* 
  #ifndef Target_Board
    #error Please choose at least one target board
  #endif

 #if (Target_Board == 0) 
   #error Teensy 3.x not yet supported
 #endif  

 #if (Target_Board == 1) 
   #if defined Display_Support
     #error Blue Pill  version does not yet support a display
   #endif  
 #endif  

 #if (Target_Board == 2) 
   #if defined Display_Support
     #error Maple Mini version does not yet support a display
   #endif  
 #endif  

  #if (Target_Board == 4) 
   #error ESP8266 might work but you need to work out the detail yourself
 #endif 
 
  #if (Target_Board != 3) 
     #if (Telemetry_In  == 1) || (Telemetry_In  == 2) 
       #error WiFi or Bluetooth works only on an ESP32 board
     #endif  
  #endif

  #ifndef Telemetry_In  
    #error Please choose at least one Mavlink input type, Serial, Bluetooth or WiFi 
  #endif  
  
  #if (Target_Board == 3)
    #ifndef WiFi_Mode 
      #error Please define WiFi_Mode
    #endif  
    #ifndef WiFi_Protocol
      #error Please define WiFi_Protocol
    #endif
  #endif 
// ************************* P L A T F O R M   D E P E N D E N T   S E T U P S **********************************
//********************************************* LEDS, OLED SSD1306, rx pin **************************************

  
#if (Target_Board == 0)           // Teensy3x NOT YET IMPLEMENTED !
  #define rxPin        9  
  #define txPin       10
  #define SetHomePin 
  #define StatusLed   14
  #define SetHomePin  
  #define azPWM_Pin 
  #define elPWM_Pin 
  #define BuiltinLed  13
  #undef  Display_Support 
  
  //=========================================================================   
#elif (Target_Board == 1)         // Blue Pill

  #include <Servo.h>  
  uint8_t rxPin =           PA03;  
  #define txPin             PA02  
  #define SetHomePin        PA0;    
  #define StatusLed         PA06  // Off=No good GPS yet, flashing=good GPS but home not set yet, solid = ready to track
  #define azPWM_Pin         PA07  // azimuth servo 
  #define elPWM_Pin         PA08  // elevation servo
  #define BuiltinLed        PC13  
  //=========================================================================   
#elif (Target_Board == 2)         // Maple Mini

  #include <Servo.h>  
  uint8_t rxPin =           26;  
  #define txPin             25  
  #define SetHomePin        5    
  #define StatusLed         6   // Off=No good GPS yet, flashing=good GPS but home not set yet, solid = ready to track
  #define azPWM_Pin         7   // azimuth servo 
  #define elPWM_Pin         8   // elevation servo
  #define BuiltinLed        33  // PB1   
  
#elif (Target_Board == 3)         // ESP32 Platform
// For info: Avoid SPI pins - generally   CS=5    MOSI=23   MISO=19   SCK=18  
  #include <ESP32_Servo.h>  
  //=========================================================================   
  #if (ESP32_Variant == 1)          // ESP32 Dev Module
  uint8_t rxPin =           27;  
  #define txPin             17  
  #define SetHomePin        18    
  #define StatusLed         25  // Off=No good GPS yet, flashing=good GPS but home not set yet, solid = ready to track
  #define BuiltinLed        99
  #define azPWM_Pin         32  // azimuth servo (can't be 34,35,36,39 because input only !!)
  #define elPWM_Pin         33  // elevation servo(can't be 34,35,36,39 because input only !!)
  #define BuiltinLed        02  // PB1   

    #if (defined Display_Support)   // Display type defined with # define Display_Support   
      /* Below please choose either Touch pin-pair or Digital pin-pair for display scrolling
       *  Pin == 99 means the pin-pair is not used
       */ 
      #define Pup           99        // 35 Board Button 1 to scroll the display up
      #define Pdn           99        //  0 Board Button 2 to scroll the display down    
      #define Tup           33        // 33 Touch pin to scroll the display up
      #define Tdn           32        // 32 Touch pin to scroll the display down   
      #define SDA           21        // I2C OLED board
      #define SCL           22        // I2C OLED board
      #define i2cAddr      0x3C       // I2C OLED board
    #endif   
    /*  
      SPI/CS                       Pin 05   For optional TF/SD Card Adapter
      SPI/MOSI                     Pin 23   For optional TF/SD Card Adapter
      SPI/MISO                     Pin 19   For optional TF/SD Card Adapter
      SPI/SCK                      Pin 18   For optional TF/SD Card Adapter  
    */
  #endif
  //=========================================================================   
  #if (ESP32_Variant == 4)          // Heltec Wifi Kit 32 (NOTE! 8MB) 
  uint8_t rxPin =           27;  
  #define txPin             17  
  #define SetHomePin        18    
  #define StatusLed         13  // Off=No good GPS yet, flashing=good GPS but home not set yet, solid = ready to track
  #define BuiltinLed        99 
  #define azPWM_Pin         32  // azimuth servo (can't be 34,35,36,39 because input only !!)
  #define elPWM_Pin         33  // elevation servo(can't be 34,35,36,39 because input only !!)

    #if !defined Display_Support      // I2C OLED board is built into Heltec WiFi Kit 32
      #define Display_Support
    #endif
    #if !defined SSD1306_Display    
      #define SSD1306_Display         // OLED display type - if you have a display you must define which type
    #endif 
    #undef ST7789_Display 
    /* Below please choose either Touch pin-pair or Digital pin-pair for display scrolling
     *  Pin == 99 means the pin-pair is not used
     */ 
    #define Pup           99        // Board Button 1 to scroll the display up
    #define Pdn           99        // Board Button 2 to scroll the display down    
    #define Tup           34        // 33 Touch pin to scroll the display up
    #define Tdn           35        // 32 Touch pin to scroll the display down 
       
    #define SDA           04        // I2C OLED board 
    #define SCL           15        // I2C OLED board
    #define i2cAddr      0x3C       // I2C OLED board
    #define OLED_RESET    16        // RESET here so no reset lower down    

    /*  
      SPI/CS               05   For optional TF/SD Card Adapter
      SPI/MOSI             23   For optional TF/SD Card Adapter
      SPI/MISO             19   For optional TF/SD Card Adapter
      SPI/SCK              18   For optional TF/SD Card Adapter  
    */
  int16_t  wifi_rssi;   
  #endif
  //=========================================================================   
  #if (ESP32_Variant == 5)          // LILYGO® TTGO T-Display ESP32 1.14" ST7789 Colour LCD, IDE board = "ESP32 Dev Module"
    uint8_t rxPin =           27;  
    #define txPin             17  
    #define SetHomePin        15
    #define StatusLed         25        // Add your own LED with around 1K series resistor
    #define BuiltinLed        99    
    #define azPWM_Pin         32  // azimuth servo (can't be 34,35,36,39 because input only !!)
    #define elPWM_Pin         33  // elevation servo(can't be 34,35,36,39 because input only !!)    
    #if !defined Display_Support    // I2C TFT board is built into TTGO T-Display
      #define Display_Support
    #endif
    #if !defined ST7789_Display    
      #define ST7789_Display          // TFT display type 
    #endif 
    #undef SSD1306_Display 
    /* Below please choose either Touch pin-pair or Digital pin-pair for display scrolling
     *  Pin == 99 means the pin-pair is not used
     */ 
    #define Pup            0        //  0 Board Button 1 to scroll the display up
    #define Pdn           35        // 35 Board Button 2 to scroll the display down    
    #define Tup           99        // 33 Touch pin to scroll the display up
    #define Tdn           99        // 32 Touch pin to scroll the display down   
    
    //#define screenOrientation 0     // Portrait - Select one orientation only
    #define screenOrientation 1     // Landscape
 
    #define SDA           21        // I2C TFT board 
    #define SCL           22        // I2C TFT board
    #define i2cAddr      0x3C       // I2C TFT board
  #endif
   //=========================================================================   
  #if (ESP32_Variant == 6)          // LILYGO® TTGO T2 ESP32 OLED Arduino IDE board = "ESP32 Dev Module"
    uint8_t rxPin =           27;  
    #define txPin             17  
    #define SetHomePin        15
    #define StatusLed         25        // Add your own LED with around 1K series resistor
    #define BuiltinLed        99    
    #define azPWM_Pin         14  // azimuth servo (can't be 34,35,36,39 because input only !!)
    #define elPWM_Pin         16  // elevation servo(can't be 34,35,36,39 because input only !!)    
    #if !defined Display_Support      // I2C OLED board is built into TTGO T2
      #define Display_Support
    #endif
    #if !defined SSD1306_Display    
      #define SSD1306_Display         // OLED display type
    #endif 
    #undef ST7789_Display 
    /* Below please choose either Touch pin-pair or Digital pin-pair for display scrolling
     *  Pin == 99 means the pin-pair is not used
     */ 
    #define Pup           99        // Board Button 1 to scroll the display up
    #define Pdn           99        // Board Button 2 to scroll the display down    
    #define Tup           33        // 33 Touch pin to scroll the display up
    #define Tdn           32        // 32 Touch pin to scroll the display down  
     
    #define SDA           13        // I2C OLED board 
    #define SCL           14        // I2C OLED board
    #define i2cAddr      0x3C       // I2C OLED board
  #endif 
#endif

  //=================================================================================================   
  //                      D I S P L A Y   S U P P O R T    E S P  O N L Y - for now
  //=================================================================================================  

  #if ((defined ESP32 || defined ESP8266)) && (defined Display_Support)  

    #if not defined SD_Libs_Loaded    //  by SD block
      #include <SPI.h>                // for SD card and/or Display
    #endif  

    #if (defined ST7789_Display)        // TFT display type    
      #include <TFT_eSPI.h>             // Note: This is a hardware-specific library. You must update User_Setup.h 
      TFT_eSPI display = TFT_eSPI();
      #if (screenOrientation == 0)      // portrait
        #define screen_height  20       // characters not pixels
        #define screen_width   30       // ?
      #elif (screenOrientation == 1)    // landscape
        #define screen_height   8       // characters not pixels 
        #define screen_width   20        
      #endif      

    #elif (defined SSD1306_Display)    // SSD1306 OLED display     
      #if not defined SD_Libs_Loaded   //  by SD block
        #include <Wire.h>
      #endif  
      #include <Adafruit_SSD1306.h> 
        #define SCREEN_WIDTH 128   // OLED display width, in pixels
        #define SCREEN_HEIGHT 64   // OLED display height, in pixels
        #define screen_height  8   // characters not pixels 
        #define screen_width  21        
        #ifndef OLED_RESET
          #define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)
        #endif  
      Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);      
    #endif

  #define max_col   screen_width+1  // for terminating char        
  #define max_row   30

  char snprintf_buf[max_col];   // for use with snprintf() formatting of display line
  
    static const uint16_t threshold = 40;
    volatile bool upButton = false;
    volatile bool dnButton = false;

    #if (not defined Tup) 
      #define Tup         99
    #endif

    #if (not defined Tdn) 
      #define Tdn         99
    #endif

    #if (not defined Pup) 
      #define Tup         99
    #endif

    #if (not defined Pdn) 
      #define Tdn         99
    #endif

    typedef enum scroll_set { non = 0, up = 1, down = 2 } scroll_t;
    scroll_t up_down = non; 

    typedef enum last_row_set { omit_last_row = 0, show_last_row = 1} last_row_t;
    last_row_t last_row_action;   

    struct row_t {
      char x[max_col];
      };
  
     row_t ScreenRow[max_row]; 
     
    uint8_t   row = 0;
    uint8_t   col = 0;
    uint8_t   scroll_row = 0;
    uint32_t  scroll_millis =0 ;

  #endif

  //=================================================================================================   
  //                     B L U E T O O T H   S U P P O R T -  E S P 3 2  O n l y
  //================================================================================================= 

#if (Telemetry_In == 1)     // Bluetooth
    #if (Target_Board == 3) // ESP32

    #define BT_Setup   // so that WiFi setup does not defien these shared variables again
    // Define link variables
    struct linkStatus {
      uint32_t    packets_received;
      uint32_t    packets_lost;
      uint32_t    packets_sent;
     };

      bool          hb_heard_from = false;;
      uint8_t       hb_system_id = 0;
      uint8_t       hb_comp_id = 0;
      uint8_t       hb_seq_expected = 0;
      uint32_t      hb_last_heartbeat = 0;
      linkStatus    link_status;

      #include "BluetoothSerial.h"
      #if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
        #error Bluetooth is not enabled! Please run `make menuconfig in ESP32 IDF` 
      #endif
    #else
      #error Bluetooth only available on ESP32
    #endif    
#endif

  //=================================================================================================   
  //                       W I F I   S U P P O R T - ESP32 and ES8266 Only
  //================================================================================================= 


#if (Target_Board == 3) && (Telemetry_In == 2)  // ESP32 and WiFi

    uint16_t  TCP_localPort = 5760;
    uint16_t  TCP_remotePort = 5760;    
    uint16_t  UDP_localPort = 14555;     
    uint16_t  UDP_remotePort = 14550;   
    bool      FtRemIP = true;
    uint8_t   AP_sta_count = 0;
    uint8_t   AP_prev_sta_count = 0;
    int16_t   wifi_rssi;
    
    // Define link variables
    #ifndef BT_Setup
      struct linkStatus {
        uint32_t    packets_received;
        uint32_t    packets_lost;
        uint32_t    packets_sent;
      };

       bool          hb_heard_from = false;;
       uint8_t       hb_system_id = 0;
       uint8_t       hb_comp_id = 0;
       uint8_t       hb_seq_expected = 0;
       uint32_t      hb_last_heartbeat = 0;
       linkStatus    link_status;
    #endif
   

      #include <WiFi.h>  
      #include <WiFiClient.h>
      #if (WiFi_Mode == 1)  // AP
        #include <WiFiAP.h>  
      #endif   

    
    #if (WiFi_Protocol == 2)  //  UDP
      #include <WiFiUDP.h>    // ESP32 and ESP8266
    #endif   
    
    const char    *APssid         =    "AntTracker";      // The AP SSID that we advertise  ====>
    const char    *APpw           =    "password";        // Change me!
    const uint8_t  APchannel      =    9;                 // The WiFi channel to use
//   const char    *STAssid        =     "TargetAPName";  // Target AP to connect to      <====
//    const char    *STApw          =     "targetPw";     // Change me!    

   const char    *STAssid        =     "MavToPassthru";    
   const char    *STApw          =     "password";             

   //  const char    *STAssid =     "EZ-WifiBroadcast";    
   //  const char    *STApw =       "wifibroadcast";         

   //  const char    *STAssid =     "TXMOD-54-DD-FE";   
   //  const char    *STApw =       "txmod123";      


   //====================       W i F i   O b j e c t s 

    #define max_clients    5
    uint8_t active_client_idx = 0; 

    IPAddress localIP;                           // tcp and UDP
    IPAddress TCP_remoteIP(192,168,4,1);         // when we connect to a server in tcp client mode, put the server IP here  

    #if (WiFi_Protocol == 1)     // TCP
    
     // WiFiClient TCPclient; 

      WiFiClient *clients[max_clients] = {NULL};   // pointers to TCP client objects (we only need one in this application
    
     // WiFiServer TCPserver(TCP_localPort);
    
    #endif 
    
    #if (WiFi_Protocol == 2)     //  UDP
   
      IPAddress UDP_remoteIP(192, 168, 1, 255);    // default to broadcast, but likey to change after connect               
      uint8_t   UDP_remoteIP_B3;                   // last byte of remote UDP client IP  
      WiFiUDP UDP;       // Create UDP object    
        
    #endif   

#endif
  


//************************************************************************** 
//********************************** Serial ********************************

  #define Debug               Serial         // USB  
  #if (Telemetry_In == 0)     // Serial
    #define inSerial            Serial2        // ESP32 General telemetry input   
  #endif  


// ******************************** D E B U G G I N G   O P T I O N S ***************************************

#define Debug_Minimum    //  Leave this as is unless you need the serial port for something else
#define Debug_Status
//#define Debug_All

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

//#define Debug_BT
//#define Debug_WiFi
//#define Debug_FC_Write
//#define Debug_Input

// *****************************************************************************************************************

/*
*****************************************************************************************************************  
Change log:
                                    
Legacy
v0.37 2019-02-12 Reintroduce PacketGood() location reasonability test to mitigate telemetry errors.
v0.38 OLED support added

v2.00 2019-03-03 Unified version auto detect telemetry speed and protocol. Mavlink 1 & 2, FrSky (D, S.Port and Passthru), LTM
      MSP and GPS support outstanding
v2.01 Support for NMEA GPS added     
v2.05 2019/03/17 UNTESTED BETA Support for retoring home settings from EEPROM after power interruption  
v2.06 2019/06/25 Improved auto baud rate detection. Define rxPin correctly for Maple Mini
v2.07 2019/08/10 Tidy up mavlink library paths
v2.08 2019/08/13 Fix mavlink ap_fixtype >2 before using lat and lon. Scroll entire Oled screen.
v2.09 2019/08/16 Supports ESP32 Dev Board - BETA
v2.10 2019/08/23 Improve response in baud detect when no telemetry present.
v2.11 2019/09/06 Mavlink WiFi and Bluetooth input added.  BT not tested.
v2.12 2019/09/18 Store most recent mavlink GPS millis.  
v2.13 2019-11-18 Update WiFi, BT (master/slave), board variants setup, autobaud, OledPrint to Mav2Pt v2.46  
v2.14 2019-12-11 When Heading _Source == 3 (compass on tracker), don't need to check ap_hdg in PacketGood()
      2020-01-02 Sloppy exits removed. :)          
*/
