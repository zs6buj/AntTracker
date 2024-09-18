
/*

  Complete change log and debugging options are at the bottom of this tab
   
v2.13 2019-11-18 Update WiFi, BT (master/slave), board variants setup, autobaud, OledPrint to Mav2Pt v2.46  
`                    
*/
// ******************************* Please select your options here before compiling *******************************

// Choose one only of these input channels 
// How does telemetry enter the tracker?
#define Telemetry_In  0    // Serial Port (default) - all protocols        
//#define Telemetry_In  1    // BlueTooth Classic - ESP32 and Mavlink only
//#define Telemetry_In  2    // WiFi - ESP32 and Mavlink only


// NOTE: The Bluetooth class library uses a great deal of application memory. During Compile/Flash
//  you may need to select Tools/Partition Scheme: "Minimal SPIFFS (1.9MB APP ...)

//#define BT_Master_Mode true    // Master connects to BT_Slave_Name --- false for BT Slave Mode
const char* BT_Slave_Name   =   "Crossfire 0277";  // Example

#define AutoBaud              // UART Serial Only - Auto detect telemetry speed

// Choose one - for ESP32 and Mavlink only
//#define WiFi_Protocol 1    // TCP/IP
#define WiFi_Protocol 2    // UDP     useful for Ez-WiFiBroadcast in STA mode

// Choose one - AP means advertise as an access point (hotspot). STA means connect to a known host
//#define WiFi_Mode   1  //AP            
#define WiFi_Mode   2  // STA


#define AutoAP                      // If we fail to connect in STA mode, start AP instead


// Un-comment (activate) the options below
//#define Az_Servo_360   // Means the azimuth servo can point in a 360 deg circle, elevation servo 90 deg
                         // Default (comment out #define above) is 180 deg azimuth and 180 deg elevation 
                         
const uint8_t Heading_Source =  3;  // 1=GPS, 2=Flight Computer, 3=Tracker_Compass   
                     
//#define QLRS           // Un-comment if you use the QLRS variant of Mavlink 

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




//*********************************************************************************************
//**********************   S E L E C T   E S P   B O A R D   V A R I A N T   ******************

#define ESP32_Variant     1    //  ESP32 Dev Module - there are several sub-variants that work
//#define ESP32_Variant     2    //  Wemos® LOLIN ESP32-WROOM-32_OLED_Dual_26p
//#define ESP32_Variant     3    //  Dragonlink V3 slim with internal ESP32 - contributed by Noircogi
//#define ESP32_Variant     4    //  Heltec Wifi Kit 32 - contributed by Noircogi

#define ESP8266_Variant   1   // Node MFU 12F


//************************************** Macro Logic Checks ************************************* 
  #ifndef Target_Board
    #error Please choose at least one target board
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
  #define rxPin       09  
  #define txPin       10
  #define SetHomePin 
  #define StatusLed   14
  #define SetHomePin  
  #define azPWM_Pin 
  #define elPWM_Pin 
  #define BuiltinLed  13

#elif (Target_Board == 1)         // Blue Pill

  #include <Servo.h>  
  uint8_t rxPin =           PA03;  
  #define txPin             PA02  
  #define SetHomePin        PA0;    
  #define StatusLed         PA06  // Off=No good GPS yet, flashing=good GPS but home not set yet, solid = ready to track
  #define azPWM_Pin         PA07  // azimuth servo 
  #define elPWM_Pin         PA08  // elevation servo
  #define BuiltinLed        PC13  
  
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
  
  #if (ESP32_Variant == 1)          // ESP32 Dev Module
  uint8_t rxPin =           16;  
  #define txPin             17;  
  #define SetHomePin        15    
  #define StatusLed         13  // Off=No good GPS yet, flashing=good GPS but home not set yet, solid = ready to track
  #define azPWM_Pin         32  // azimuth servo (can't be 34,35,36,39 because input only !!)
  #define elPWM_Pin         33  // elevation servo(can't be 34,35,36,39 because input only !!)
  #define BuiltinLed        02  // PB1   
  
  int16_t  wifi_rssi;   
  #endif
#endif

// *************************************    D E F I N E   O L E D    *****************************
#if (Target_Board == 3) // ESP32  
  #include <SPI.h>                // for OLED
  #include <Wire.h>
  #include <Adafruit_SSD1306.h> 

  #define SCREEN_WIDTH 128 // OLED display width, in pixels
  #define SCREEN_HEIGHT 64 // OLED display height, in pixels
  // 8 rows of 21 characters

  // Declaration for an SSD1306 I2C display
  #ifndef OLED_RESET
    #define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)
  #endif  
  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
  
#endif

#if (Target_Board == 1)|| (Target_Board == 2) // Blue Pill or Maple Mini
  #include <Servo.h>
  #include <Adafruit_SSD1306_STM32.h>

  #define OLED_RESET 4
  Adafruit_SSD1306 display(OLED_RESET);

#endif

//************************************************************************** 
//**************************** Bluetooth - ESP32 Only **********************

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

///************************************************************************** 
//***************************** WiFi - ESP32 Only ***************************


#if (Target_Board == 3) && (Telemetry_In == 2)  // ESP32 and WiFi

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

   const char    *STAssid        =     "OmegaOffice";    
   const char    *STApw          =     "";             

   //  const char    *STAssid =     "EZ-WifiBroadcast";    
   //  const char    *STApw =       "wifibroadcast";         

   //  const char    *STAssid =     "TXMOD-54-DD-FE";   
   //  const char    *STApw =       "txmod123";      


   // AP and STA below

    WiFiClient wifi;   
    
    #if (WiFi_Protocol == 1)     // TCP
      uint16_t tcp_localPort = 5760;  
      WiFiServer server(tcp_localPort);
    #endif 
    
    #if (WiFi_Protocol == 2)     //  UDP
   
      uint16_t udp_localPort = 14555;     // This ESP32 or ESP8266
      uint16_t udp_remotePort = 14550;    // GCS like QGC      
      bool FtRemIP = true;
      #if   (WiFi_Mode == 1)  // AP
        IPAddress udp_remoteIP(192, 168, 4, 2);       // We hand out this IP to the first client via DHCP
      #elif (WiFi_Mode == 2)  // STA
        IPAddress udp_remoteIP(192, 168, 1, 255);    // UDP broadcast on your likely LAN subnet
      #endif       

      WiFiUDP udp;       // Create udp object    
        
    #endif   
    
    IPAddress localIP;

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
//#define Debug_Input
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
*/
