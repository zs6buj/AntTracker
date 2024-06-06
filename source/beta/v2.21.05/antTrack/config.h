//================================================================================================= 
//================================================================================================= 
//
//                                    C O N F I G U R A T I O N 

#define MAJOR_VERSION       2
#define MINOR_VERSION      21
#define PATCH_LEVEL         5

//=============================================================================================
//================== Please select your options below before compiling ========================
//=============================================================================================

#define DEVICE_SYSID     251                     // Our Mavlink Identity - APM FC is 1, Mission Planner is 255, QGC default is 0 
#define DEVICE_COMPID    MAV_COMP_ID_PERIPHERAL  // 158 Generic autopilot peripheral - APM FC is 1, MP is 190, QGC is  https://mavlink.io/en/messages/common.html


//=============================================================================================
//=====================   S E L E C T   E S P   B O A R D   V A R I A N T   ===================
//=============================================================================================
//#define ESP32_Variant     1    //  ESP32 Dev Module - there are several sub-variants that work
//#define ESP32_Variant     4    //  Heltec Wifi Kit 32 
#define ESP32_Variant     5    //  LILYGO® TTGO T-Display ESP32 1.14" ST7789 Colour LCD, IDE board = "ESP32 Dev Module"
//#define ESP32_Variant     6    // LILYGO® TTGO T2 ESP32 OLED Arduino IDE board = "ESP32 Dev Module"
//#define ESP32_Variant     7    // ESP32 Dev Module with ILI9341 2.8" colour TFT SPI 240x320 NOT TESTED _ DON'T USE YET


//=============================================================================================
//====================  I N P U T   M E D I U M    How does telemetPROTOCOLry enter the tracker?
//=============================================================================================
// Choose one only of these input channels 
#define MEDIUM_IN  1    // UART (Serial)       
//#define MEDIUM_IN  2    // WiFi - ESP only
//#define MEDIUM_IN  3    // Bluetooth (Serial) - ESP32 only

//=============================================================================================
//================================  T E L E M E T R Y   P R O T O C O L  ======================
//=============================================================================================
// Select only one telemetry PROTOCOL here
//#define PROTOCOL 0     // AUTO detect protocol
//#define PROTOCOL 1     // Mavlink 1
//#define PROTOCOL 2     // Mavlink 2
//#define PROTOCOL 3     // FrSky S.Port
//#define PROTOCOL 4     // FrSky F.Port 1
//#define PROTOCOL 5     // FrSky F.Port 2
//#define PROTOCOL 6     // LTM
//#define PROTOCOL 7     // MSP
//#define PROTOCOL 8     // GPS NMEA
#define PROTOCOL 9     // CRFS / ELRS

//=============================================================================================
//==================================  H E A D I N G   S O U R C E  ============================
//=============================================================================================
// Select one heading source. We need this to relate the external world of co-ordinates to the internal tracker co_ordinates.
//#define HEADINGSOURCE  1     // 1=Flight Computer GPS, 
#define HEADINGSOURCE  2     // 2=Flight Computer Compass
//#define HEADINGSOURCE  3     // 3=Trackerbox_Compass 
//#define HEADINGSOURCE  4     // 4=Trackerbox_GPS_And_Compass
// Select GPS module serial link speed. Many GPS modules are capable of using multiple serial speed out from the box.
// This information should be provided by the manufacturer.
// If not defined, speed will be selected automatically.
 #define BOX_GPS_BAUD 9600

// Select compass type. This information should be provided by the manufacturer.
//#define HMC5883L
#define QMC5883L

// Select compass declination. Consult http://www.magnetic-declination.com/  to check your zone declination value.
//#define Compass_Declination -0.34

// Bohan's PR
// Select compass orientation. Some of the available GPS/Compass boards have their compass oriented in non-standard way,
// the correct re-orientation information should be provided by the manufacturer.
// Available options: ALIGN_DEFAULT, CW0_DEG, CW90_DEG, CW180_DEG, CW270_DEG, CW0_DEG_FLIP, CW90_DEG_FLIP, CW180_DEG_FLIP, CW270_DEG_FLIP
//#define BOX_COMPASS_ALIGN       ALIGN_DEFAULT
//#define BOX_COMPASS_ALIGN       CW90_DEG
//#define BOX_COMPASS_ALIGN       CW90_DEG_FLIP
// If the tracker box has a GPS AND a compass attached, we support a moving tracker. For example,
// the tracker could be on one moving vehicle and track a second moving vehicle, or a 'plane could 
// always point an antenna at home base.

//=============================================================================================
//                              S E T   H O M E   O N   A R M
//#define SET_HOME_AT_ARM_TIME      // else set home location with push button
//=============================================================================================

#define home_decay_secs 60  // Home data decay secs. if power lost and restored within decay secs, Home is restored from NVM.

//=============================================================================================
//============================    B L U E T O O T H   S E T T I N G S   =======================  
//=============================================================================================
// NOTE: The Bluetooth class library uses a lot of application memory. During Compile/Flash
//  you may need to select Tools/Partition Scheme: "Minimal SPIFFS (1.9MB APP ...) in the Arduino IDE

#define BT_MODE  1           // Master Mode - active, initiate connection with slave (name)
//#define BT_MODE  2           // Slave Mode - passive, advertise our hostname & wait for master to connect to us
const char* BT_Slave_Name   =   "btslavename"; 

//=============================================================================================
//=================================  O T H E R   S E T T I N G S   ============================
//=============================================================================================
//#define QLRS           // Un-comment if you use the QLRS variant of Mavlink 

#define Data_Streams_Enabled        // Requests data streams from FC. Requires both rx and tx lines to FC. Rather set SRn in Mission Planner


//=============================================================================================
//===================================  S E R V O   S E T T I N G S  ===========================
//=============================================================================================

  //#define TEST_SERVOS      // Move servos through their limits, then try box_hdg every 45 degrees of circle

  //#define Az_Servo_360   // Means the azimuth servo can point in a 360 deg circle, elevation servo 90 deg
                           // Default (comment out #define above) is 180 deg azimuth and flip over 180 deg elevation 
                           
  #define azStart  90       // 0 deg = left, 90 deg = straight ahead, 180 deg = right, 270 deg = behind
  #define elStart   0       // 0 = horizontal, 90 = vertical

  #define Servo_Slowdown 0  // Default 0 - Try 5, 10..  ms/degree to limit angular velocity

  // Set the degree range of the servos here. Do not adjust servo mechanical limits here.                         
  #if defined Az_Servo_360   // 1 x 360, 1 x 90 (or 180) servos  
    int16_t minAz = 0;          // Az lower limit in degrees, left of tracker facing flying field
    int16_t maxAz = 359;        // Az upper limit in degrees
    int16_t minEl = 0;          // El lower limit in degrees, horizontal 
    int16_t maxEl = 90;         // El upper limit in degrees, straight up
  #else                      // 2 x 180 deg servos
    int16_t minAz = 0;          // Az lower limit in degrees, left of tracker facing flying field
    int16_t maxAz = 180;        // Az upper limit in degrees, right of tracker facing flying field
    int16_t minEl = 0;          // El lower limit in degrees, horizontal and forward
    int16_t maxEl = 180;        // El upper limit in degrees, horizontal and rearward
  #endif 

  // Sometimes the mechanical movement of a servo is reversed due to the orientation of its mounting
  // Its movement may be reversed here to compensate
  #define ReverseAzimuth          // my azimuth servo has a reversed action
  //#define ReverseElevation


  // Default values for SG90 servos; 500 and 2400
  // My 180 deg servos have a PWM range of 700 through 2300 microseconds. Your's may differ. 
  // ADJUST THE MECHANICAL LIMITS OF MOVEMENT OF YOUR SERVOS HERE BELOW

  #if defined STM32F1xx      // my STM32 based tracker has different servos
    uint16_t minAzPWM = 600;   // right (because mine is reversed)
    uint16_t maxAzPWM = 2300;  // left   
    uint16_t minElPWM = 700;   // front 
    uint16_t maxElPWM = 2300;  // back
  #else
    uint16_t minAzPWM = 625;   // right (because mine is reversed)
    uint16_t maxAzPWM = 2235;  // left 
    uint16_t minElPWM = 600;   // front
    uint16_t maxElPWM = 2257;  // back  
  #endif


//=============================================================================================
//================================   W I F I   S E T T I N G S  ===============================  
//=============================================================================================

#define Start_WiFi                              // Start WiFi at startup, override startWiFi pin

#define HostName             "Frs2BT"        // This translator's host name
#define APssid               "AntTrackAP"     // The AP SSID that we advertise         ====>
#define APpw                 "12345678"         // Change me! Must be >= 8 chars
#define APchannel            9                  // The wifi channel to use for our AP
#define STAssid              "FrSkyToWiFi"    // Target AP to connect to (in STA mode) <====
#define STApw                "password"         // Target AP password (in STA mode). Must be >= 8 chars      

// Choose one default mode for ESP only - AP means advertise as an access point (hotspot). STA means connect to a known host
//#define WiFi_Mode   1  //AP            
#define WiFi_Mode   2  // STA
//#define WiFi_Mode   3  // (STA>AP) STA failover to AP 

// Choose one default protocol - for ESP only
//#define WIFI_PROTOCOL 1    // TCP/IP
#define WIFI_PROTOCOL 2    // UDP 

//  const char    *STAssid =     "EZ-WifiBroadcast";    
//  const char    *STApw =       "wifibroadcast";         

//  const char    *STAssid =     "TXMOD-54-DD-FE";   
//  const char    *STApw =       "txmod123"; 

uint16_t  TCP_localPort = 5760;    // You listen on this port, read port    
uint16_t  TCP_remotePort = 5760;   // You send to this port, send port
uint16_t  UDP_localPort = 14555;    // Mav readPort,  (default 14555) remote host (like MP and QGC) expects to send to this port - Frsky +1
uint16_t  UDP_remotePort = 14550;   // Mav sendPort,  (default 14550) remote host reads from this port - FrSky +1

//=============================================================================================
//============================= Auto Determine Target Platform ================================
//=============================================================================================
//
//                Don't change anything here
//
#if defined (__MK20DX128__) || defined(__MK20DX256__)
  #define TEENSY3X
  #define Target_Board   0      // Teensy 3.1 and 3.2    


// Using official stmicroelectronics lib: https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json 
#elif defined (STM32F1xx)   // in boards.txt / build.variant, like GenF1.menu.pnum.BLUEPILL_F103C8.build.variant=STM32F1xx/F103C8T_F103CB(T-U)
  #define Target_Board   1      // Blue Pill STM32F103C8  

#elif defined ESP32
  #define Target_Board   3      // Espressif ESP32 Dev Module

#elif defined ESP8266
  #define Target_Board   4      // Espressif ESP8266
  
#else
  #error "No board type defined!"
#endif


  #if (defined ESP32 || defined ESP8266) && (MEDIUM_IN == 2) 
    #define wifiBuiltin   //  for this feature we need wifi support compiled in
  #endif    

   #if (defined ESP32 || defined ESP8266) && (MEDIUM_IN == 3) 
    #define btBuiltin   //  for this feature we need bluetooth support compiled in
  #endif

 #if ( (defined ESP32) && (MEDIUM_IN  == 3) )
   #if ( (BT_MODE != 1) && (BT_MODE != 2) )
       #error "Please define a bluetooth mode"
   #endif
 #endif  

  
//============================================================================================= 
//======================================= Macro Logic Checks ==================================
//============================================================================================= 
  #ifndef Target_Board
    #error Please choose at least one target board
  #endif

  #if (Target_Board == 0) 
    //#error Teensy 3.x not yet supported. 
  #endif  

  #if (Target_Board == 1) 
    #if defined displaySupport
  //   #error STM32F1xx  version does not yet support a display
    #endif  
  #endif  

  #if not defined HEADINGSOURCE
   #error Please define a HEADINGSOURCE
  #endif
  #if ((HEADINGSOURCE == 3) || (HEADINGSOURCE == 4))
    #if ( (not defined HMC5883L) && (not defined QMC5883L) )
      #error Please define a compass type 
    #endif
  #endif  

  uint8_t headingsource = HEADINGSOURCE;
 
  #if (Target_Board == 4) 
    #error ESP8266 should work but you need to work out the detail yourself
  #endif 

  #ifndef MEDIUM_IN  
    #error Please choose at least one Mavlink input type, Serial, Bluetooth or WiFi 
  #endif
  #if not defined ESP32 
     #if (MEDIUM_IN  == 2) || (MEDIUM_IN  == 3) 
       #error WiFi or Bluetooth works only on an ESP32 board, rather select serial in
     #endif  
  #endif
  
  #if (defined ESP32)
    #ifndef WiFi_Mode 
      #error Please define WiFi_Mode
    #endif  
    #ifndef WIFI_PROTOCOL
      #error Please define WIFI_PROTOCOL
    #endif 
  #endif

  #if not defined PROTOCOL 
     #error Please define a telemetry protocol  
  #endif 
//=============================================================================================  
//======================= P L A T F O R M   D E P E N D E N T   S E T U P S ===================
//=============================================================================================

#if (Target_Board == 0)           //   (TEENSY3X) 
  #include <PWMServo.h>     
  #define in_rxPin        0       // rx1 tx1 - Serial1
  #define in_txPin        1
  int8_t boxgps_rxPin =   9;      // rx2 tx2 - Serial2 for tracker box GPS if applicable
  #define boxgps_txPin   10  
  bool in_invert = true;           // ONLY FOR FrSky S.Port, NOT F.Port, NOT MAVLINK     
  #define frOneWire     true      // ONLY FOR FrSky S.Port
  #define SetHomePin     11
  #define StatusLed      14
  #define azPWM_Pin       5
  #define elPWM_Pin       6
  #define BuiltinLed     13
  #undef  displaySupport 
  #define SDA            17  // I2C OLED board and/or Compass - default can be changed in Wire.h 
  #define SCL            16  // I2C OLED board and/or Compass - default can be changed in Wire.h 
  //=========================================================================   
  
#elif (Target_Board == 1)         // STM32F1xx Blue Pill

  #include <Servo.h>  
  #include <Wire.h>
                         // PA10  // rx1 Serial(0) flash and monitor    
                         // PA9   // tx1 Serial(0) flash and monitor
  int8_t in_rxPin =         PA3;  // rx2 Serial1
  #define in_txPin          PA2   // tx2 Serial1
  int8_t boxgps_rxPin =     PB11;  // rx3 Serial2
  #define boxgps_txPin      PB10   // tx3 Serial2
  bool in_invert = false;           // ONLY FOR FrSky S.Port, NOT F.Port, NOT MAVLINK    
  #define SetHomePin        PA5    //PA0    
  #define StatusLed         PA6  // Off=No good GPS yet, flashing=good GPS but home not set yet, solid = ready to track
  #define azPWM_Pin         PA7  // azimuth servo 
  #define elPWM_Pin         PA8  // elevation servo
  #define BuiltinLed        PC13  
  #define SDA               PB7  // I2C OLED board and/or Compass - default can be changed in Wire.h 
  #define SCL               PB6  // I2C OLED board and/or Compass - default can be changed in Wire.h 
  //=========================================================================  
   
#elif (Target_Board == 2)    // Maple Mini

  #include <Servo.h>  
  int8_t in_rxPin =        26;  // rx1 Serial1
  #define in_txPin         25   // tx1 Serial1
  int8_t boxgps_rxPin =     8;  // rx2 Serial2
  #define boxgps_txPin      9   // tx2 Serial2
   bool in_invert = true;        // ONLY FOR FrSky S.Port, NOT F.Port, NOT MAVLINK    
  #define SetHomePin        5    
  #define StatusLed         6   // Off=No good GPS yet, flashing=good GPS but home not set yet, solid = ready to track
  #define azPWM_Pin         4   // azimuth servo 
  #define elPWM_Pin         5   // elevation servo  consider pin 10  this pin=rx2 (9=tx2)
  #define BuiltinLed       33   // PB1   
  #define SDA              15   // I2C OLED board and/or Compass - default must be changed in Wire.h 
  #define SCL              16   // I2C OLED board and/or Compass - default must be changed in Wire.h 
  
#elif (Target_Board == 3)         // ESP32 Platform
// For info: Avoid SPI pins - generally   CS=5    MOSI=23   MISO=19   SCK=18  
  #include <ESP32_Servo.h>  
  //=========================================================================  
   
  #if (ESP32_Variant == 1)          // ESP32 Dev Module
  int8_t in_rxPin =         27;  // uart1
  #define in_txPin          17 
  int8_t boxgps_rxPin =     13;  // uart2 for tracker box GPS if applicable
  #define boxgps_txPin       4  
  bool in_invert = true;          // ONLY FOR FrSky S.Port and CRSF, NOT F.Port, NOT MAVLINK
  #define SetHomePin        12   // LOW == pushed    
  #define StatusLed         25   // Off=No good GPS yet, flashing=good GPS but home not set yet, solid = ready to track
  #define BuiltinLed        -1
  #define azPWM_Pin         32   // azimuth servo (can't be 34,35,36,39 because input only !!)
  #define elPWM_Pin         33   // elevation servo(can't be 34,35,36,39 because input only !!)
  #define BuiltinLed        02   // PB1   

    #define displaySupport       // uncomment me if you have a SSD1306 display
    #if (defined displaySupport)   // Display type defined with # define displaySupport   
      #define SSD1306_Display         // OLED display type    
      /* Below please choose either Touch pin-pair or Digital pin-pair for display scrolling
       *  Pin == -1 means the pin-pair is not used
       */ 
      #define Pup           -1        // Board Button 1 to scroll the display up
      #define Pdn           -1        // Board Button 2 to scroll the display down   
      #define Tup           33        // Touch pin to scroll the display up
      #define Tdn           32        // Touch pin to scroll the display down   
          
      #define SDA           21        // I2C OLED board and/or Compass
      #define SCL           22        // I2C OLED board and/or Compass
      #define display_i2c_addr      0x3C       // I2C OLED board
    #endif   
    /*  
      SPI/CS                       Pin 05   For optional TF/SD Card Adapter
      SPI/MOSI                     Pin 23   For optional TF/SD Card Adapter
      SPI/MISO                     Pin 19   For optional TF/SD Card Adapter
      SPI/SCK                      Pin 18   For optional TF/SD Card Adapter  
    */
  #endif
  //========================================================================= 
    
#if (ESP32_Variant == 4) // Heltec Wifi Kit 32 V3 (S3) (thanks to Marc Dornan)
  #define MavStatusLed 35 // Onboard LED
  #define InvertMavLed false
  #define BufStatusLed -1 // none
  #define fc_rxPin 44 // Mavlink serial rx2
  #define fc_txPin 43 // Mavlink serial tx2
  #define fr_rxPin 26 // FPort rx1 - (NOTE: DON’T use pin 12! boot fails if pulled high)
  #define fr_txPin 48 // FPort tx1 - Use me in single wire mode
  // no GCS serial set up here yet
  #define sbus_rxPin -1 // not used - don’t care
  #define sbus_txPin -1 // ?Optional SBUS out pin
  #define startWiFiPin -1 // Trigger WiFi startup
  #define resetEepromPin -1 // 5, -1=none use non digital touch pin
  #if !defined displaySupport // I2C OLED board is built into Heltec WiFi Kit 32
    #define displaySupport
  #endif
  #define SSD1306_Display // OLED display type
  #define SCR_ORIENT 1 // 1 Landscape or 0 Portrait
  /* Below please choose either Touch pin-pair or Digital pin-pair for display scrolling
  * Pin == -1 means the pin-pair is not used
  */
  #define Pup -1 // Board Button to scroll the display up
  #define Pdn -1 // Board Button to scroll the display down
  #define Tup 45 // 33 Touch pin to scroll the display up
  #define Tdn 46 // 32 Touch pin to scroll the display down
  #define SDA 17 // I2C OLED board
  #define SCL 18 // I2C OLED board
  #define i2cAddr 0x3C // I2C OLED board
  #define OLED_RESET 21 // RESET here so no reset lower down

    /*  
      SPI/CS               05   For optional TF/SD Card Adapter
      SPI/MOSI             23   For optional TF/SD Card Adapter
      SPI/MISO             19   For optional TF/SD Card Adapter
      SPI/SCK              18   For optional TF/SD Card Adapter  
    */
  #endif
  //========================================================================= 
    
  #if (ESP32_Variant == 5)          // LILYGO® TTGO T-Display ESP32 1.14" ST7789 Colour LCD, IDE board = "ESP32 Dev Module"
    int8_t in_rxPin =         27;       // uart1 for general serial in, including flight gps
    #define in_txPin          17 
    int8_t boxgps_rxPin =     13;       // uart2 for tracker box GPS if applicable
    #define boxgps_txPin      15
    bool in_invert = false;              // ONLY FOR FrSky S.Port, NOT F.Port, NOT MAVLINK
    #define SetHomePin        12
    #define StatusLed         25        // Add your own LED with around 1K series resistor
    #define BuiltinLed        -1    
    #define azPWM_Pin         32  // azimuth servo (can't be 34,35,36,39 because input only !!)
    #define elPWM_Pin         33  // elevation servo(can't be 34,35,36,39 because input only !!)    
    #define startWiFiPin      -1  // -1=none. No input pin available (non touch!) Could use touchpin with a bit of messy work.    
    
    #if !defined displaySupport    // I2C TFT board is built into TTGO T-Display
      #define displaySupport
    #endif    
    #define ST7789_Display          // TFT display type - if you have a display you must define which type
    #define SCR_ORIENT   1          // 1 Landscape or 0 Portrait    
    
    /* Below please choose either Touch pin-pair or Digital pin-pair for display scrolling
     *  Pin == -1 means the pin-pair is not used
     */  
    #define Pup            0        //  0 Board Button 1 to scroll the display up
    #define Pdn           35        // 35 Board Button 2 to scroll the display down      
    #define Tup           -1        // 33 Touch pin to scroll the display up
    #define Tdn           -1        // 32 Touch pin to scroll the display down   
    
    #define SCR_ORIENT     1        // 1 Landscape or 0 Portrait
 
    #define SDA           21        // I2C TFT board and/or Compass (grey wire)
    #define SCL           22        // I2C TFT board and/or Compass (brown wire)
    //#define display_i2c_addr      0x3C     
    //#define compass_i2c_addr      0x1E   // 0x1E for HMC5883L   0x0D for QMC5883
  #endif
   //========================================================================= 
     
  #if (ESP32_Variant == 6)          // LILYGO® TTGO T2 ESP32 OLED Arduino IDE board = "ESP32 Dev Module"
    uint8_t in_rxPin =        17;       // uart1 for general serial in, including flight gps
    #define in_txPin          18 
    uint8_t boxgps_rxPin =    19;       // uart2 for tracker box GPS
    #define boxgps_txPin      21  
    bool in_invert = true;               // ONLY FOR FrSky S.Port, NOT F.Port, NOT MAVLINK
    #define SetHomePin        15
    #define StatusLed         25        // Add your own LED with around 1K series resistor
    #define BuiltinLed        -1    
    #define azPWM_Pin         14  // azimuth servo (can't be 34,35,36,39 because input only !!)
    #define elPWM_Pin         16  // elevation servo(can't be 34,35,36,39 because input only !!)    
    #if !defined displaySupport      // I2C OLED board is built into TTGO T2
      #define displaySupport
    #endif
    #if !defined SSD1306_Display    
      #define SSD1306_Display         // OLED display type
    #endif 
    #undef ST7789_Display 
    /* Below please choose either Touch pin-pair or Digital pin-pair for display scrolling
     *  Pin == -1 means the pin-pair is not used
     */        
    #define Pup           -1        // Board Button 1 to scroll the display up
    #define Pdn           -1        // Board Button 2 to scroll the display down          
    #define Tup           33        // 33 Touch pin to scroll the display up
    #define Tdn           32        // 32 Touch pin to scroll the display down  
     
    #define SDA           13        // I2C OLED board 
    #define SCL           14        // I2C OLED board
    #define display_i2c_addr      0x3C       // I2C OLED board
  #endif 
  //========================================================================= 
   
  #if (ESP32_Variant == 7)          // ESP32 Dev Module with ILI9341 2.8" colour TFT SPI 240x320
    uint8_t in_rxPin =        16;       // uart1 for general serial in, including flight gps
    #define in_txPin          17 
    uint8_t boxgps_rxPin =    13;       // uart2 for tracker box GPS if applicable
    #define boxgps_txPin       4
    bool in_invert = false;              // ONLY FOR FrSky S.Port, NOT F.Port, NOT MAVLINK
    #define SetHomePin         5
    #define StatusLed          2        // Add your own LED with around 1K series resistor
    #define BuiltinLed        -1    
    #define azPWM_Pin         32  // azimuth servo (can't be 34,35,36,39 because input only !!)
    #define elPWM_Pin         33  // elevation servo(can't be 34,35,36,39 because input only !!)    
    #define startWiFiPin  -1      // -1=none. No input pin available (non touch!) Could use touchpin with a bit of messy work.
    #if !defined displaySupport     
      #define displaySupport
    #endif
    #define ILI9341_Display         // ILI9341 2.8" COLOUR TFT SPI 240x320 V1.2  
    #define SCLK          18        // blue wire on my test setup
    #define MOSI          23        // yellow wire
    #define CS            25        // white wire
    #define DC            26        // green wire  
    #define RST           27        // brown wire
    // LED=3.3V,  Vcc=5v,  Gnd 
    // MISO                not used by Adafruit     
    
    /* Below please choose either Touch pin-pair or Digital pin-pair for display scrolling
     *  Pin == -1 means the pin-pair is not used
     */ 
               
    #define Pup           -1        // 35 Board Button 1 to scroll the display up
    #define Pdn           -1        //  0 Board Button 2 to scroll the display down     
    #define Tup           12        // Touch pin to scroll the display up
    #define Tdn           14        // Touch pin to scroll the display down   

    #define SDA           21        // I2C for tracker box compass
    #define SCL           22        // I2C 

  #endif     
#endif

  //=================================================================================================   
  //==================================  C O M P A S S    S U P P O R T ==============================  
  //=================================================================================================  
  #if (HEADINGSOURCE  == 3) || (HEADINGSOURCE  == 4)     // 3 = TracerBox_Compass 4=Trackerbox_GPS_And_Compass
    #define Wire_Loaded
    #include <Wire.h>
  #endif
  //=================================================================================================   
  //===================================   D I S P L A Y   S U P P O R T  ============================ 
  //=================================================================================================  

  #if defined displaySupport
    #if (!( (defined SSD1306_Display) || (defined SSD1331_Display) || (defined ST7789_Display) || (defined ILI9341_Display) ))
      #error please define a display type in your board variant configuration, or disable displaySupport
    #endif   

    #if not defined SD_Libs_Loaded    // by SD block
      #include <SPI.h>                // for SD card and/or Display
    #endif  
    #if not defined Wire_Loaded
      #include <Wire.h>
    #endif  
    
    #if (ESP32_Variant == 6)
      // Generic colour definitions
      #define BLACK           0x0000
      #define BLUE            0x001F
      #define RED             0xF800
      #define GREEN           0x07E0
      #define CYAN            0x07FF
      #define MAGENTA         0xF81F
      #define YELLOW          0xFFE0
      #define WHITE           0xFFFF 
    #endif

  //==========================================================
    
    #if (defined ST7789_Display)      // TTGO T_Display 1.14 TFT display 135 x 240 SPI
      #include <TFT_eSPI.h>           // Remember to select the T_Display board in   in TFT_eSPI library (135 x 240) 
      TFT_eSPI display = TFT_eSPI();
      #define SCR_W_PX      135       // OLED display width, in pixels - always define in portrait
      #define SCR_H_PX      240       // OLED display height, in pixels
      //#define SCR_ORIENT  1
      
      #if (SCR_ORIENT == 0)           // portrait
        #define TEXT_SIZE     1                
      #elif (SCR_ORIENT == 1)         // landscape
        #define TEXT_SIZE     2                       
      #endif 
      #define SCR_BACKGROUND TFT_BLACK  
      
    //==========================================================
    
    #elif (defined SSD1306_Display)    // SSD1306 OLED display     (128 x 64) 
      #include <Adafruit_GFX.h>
      #include <Adafruit_SSD1306.h> 
      #define SCR_W_PX 64             // OLED display width, in pixels - always define in portrait
      #define SCR_H_PX 128            // OLED display height, in pixels
      #define SCR_ORIENT  1           // landscape
      
      #if (SCR_ORIENT == 0)           // portrait
        #define TEXT_SIZE     0                
      #elif (SCR_ORIENT == 1)         // landscape
        #define TEXT_SIZE     1                       
      #endif 
 
      #ifndef OLED_RESET
        #define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)
      #endif  
      Adafruit_SSD1306 display(SCR_H_PX, SCR_W_PX, &Wire, OLED_RESET); // 128, 64
      #define SCR_BACKGROUND BLACK      
    //==========================================================  
    #elif (defined SSD1331_Display)    // SSD1331 0.95" TTGO T2 colour TFT display (96 x 64)
      #include <Adafruit_GFX.h>
      #include <Adafruit_SSD1331.h>
      // always define portrait      
      #define SCR_W_PX    64     // OLED display width, in pixels- always define in portrait
      #define SCR_H_PX    96     // OLED display height, in pixels
      #define SCR_ORIENT   1     // 0 portrait  1 landscape
      #define TEXT_SIZE    1

      Adafruit_SSD1331 display = Adafruit_SSD1331(CS, DC, MOSI, SCLK, RST);  
      #define SCR_BACKGROUND BLACK 
    //========================================================== 
    #elif  (defined ILI9341_Display)    // ILI9341 2.8" COLOUR TFT SPI 240x320 V1.2  
      #include "Adafruit_GFX.h"   
      #include "Adafruit_ILI9341.h"
      // Uses hardware SPI
      
      Adafruit_ILI9341 display = Adafruit_ILI9341(CS, DC, RST);    // LED=3.3V,  Vcc=5v,  Gnd 
     // Adafruit_ILI9341 display = Adafruit_ILI9341 (CS, DC, MOSI, SCLK, RST);     
      #define SCR_ORIENT   1            // 0 for portrait or 1 for landscape
      #define TEXT_SIZE    2            // default, may be changed on the fly
   
      #define SCR_W_PX  240             //  always define in portrait
      #define SCR_H_PX  320   
      #define SCR_BACKGROUND ILI9341_BLUE  
      #define HUD_ARROW_OFFSET 999      // no offset=default, else degrees offset for hud arrow
    #endif   
    //==========================================================   

    #if (!(defined SCR_ORIENT) )
      #error please define a desired screen orientation,  0 portrait or 1 landscape for your board variant or display type
    #endif 

    typedef enum display_mode_set { logg = 1 , flight_info = 2 } display_mode_t;
    
    display_mode_t      display_mode;     
        
    bool infoPressBusy = false;
    bool infoNewPress = false;         
    bool infoPressed = false;
    bool show_log = true;    
    uint32_t info_debounce_millis = 0; 
    uint32_t  info_millis = 0; 
    uint32_t  last_log_millis = 0;
    const uint16_t db_period = 1000; // debounce period mS

    uint16_t scr_h_px = 0;
    uint16_t scr_w_px = 0;
    uint16_t scr_w_ch = 0;
    uint16_t scr_h_ch = 0;
    uint16_t char_h_px = 0;
    uint16_t char_w_px = 0;
    
    // allocate space for screen buffer   
    #define max_col   64  // +1 for terminating line 0x00        
    #define max_row   64
    
    char clear_line[max_col];
  
    static const uint16_t threshold = 40;
    volatile bool upButton = false;
    volatile bool dnButton = false;
    volatile bool infoButton = false;
    
    #if (not defined Tup) 
      #define Tup         -1
    #endif

    #if (not defined Tdn) 
      #define Tdn         -1
    #endif

    #if (not defined Pup) 
      #define Tup         -1
    #endif

    #if (not defined Pdn) 
      #define Tdn         -1
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

  #endif   // end of Display_Support
  //=================================================================================================   
  //======================== B L U E T O O T H   S U P P O R T -  E S P 3 2  O n l y ================
  //================================================================================================= 

#if (MEDIUM_IN == 3)   // Bluetooth
  #if (defined ESP32) 
    #define BT_Setup   // so that WiFi setup does not define these shared variables again
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
      //#include "esp_gap_bt_api.h"
      #if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
      #error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
      #endif
  #else
      #error Bluetooth only available on ESP32
  #endif    
#endif

  //=================================================================================================   
  //=========================  W I F I   S U P P O R T - ESP32 and ES8266 Only ======================
  //================================================================================================= 

    int16_t   wifi_rssi;
    
#if (defined ESP32) && (MEDIUM_IN == 2)  // ESP32 and WiFi 
  
    bool      FtRemIP = true;
    uint8_t   AP_sta_count = 0;
    uint8_t   AP_prev_sta_count = 0;

    #ifndef BT_Setup
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
    #endif
    
    #include <WiFi.h>  
    #include <WiFiClient.h>

    #include <WiFiAP.h>  // SoftAP
  
    #if (WIFI_PROTOCOL == 2)    //  UDP libs
      #include <WiFiUDP.h>      // ESP32 and ESP8266
    #endif    

  #if (PROTOCOL == 1) || (PROTOCOL == 2) 
    #define MAVLINK
  #endif  

  #if (PROTOCOL == 3) || (PROTOCOL == 4) || (PROTOCOL == 5)
    #define FRSKY
  #endif  
   //====================       W i F i   O b j e c t s 

    #define max_clients    5
    uint8_t active_client_idx = 0; 

    IPAddress localIP;                           // tcp and UDP
    IPAddress TCP_remoteIP(192,168,4,1);         // when we connect to a server in tcp client mode, put the server IP here  

    #if (WIFI_PROTOCOL == 1)     // TCP  
      WiFiClient *clients[max_clients] = {NULL};   // pointers to TCP client objects (we only need one in this application    
    #endif 
    
    #if (WIFI_PROTOCOL == 2)    //  UDP 
      IPAddress UDP_remoteIP(192, 168, 1, 255);    // default to broadcast, but likey to change after connect               
      uint8_t   UDP_remoteIP_B3;                   // last byte of remote UDP client IP  
      #if (defined MAVLINK)      
        WiFiUDP   mav_udp_object;                    // Instantiate Mavlink UDP object    
      #endif
      #if (defined FRSKY)
        WiFiUDP   frs_udp_object;                    // Object for FrSky telemetry input
      #endif     
    #endif   

#endif  // end of WiFi
  
  //================================================================================================= 
  //================================   U A R T and  B T  S U P P O R T   ============================    
  //================================================================================================= 

  #if defined STM32F1xx         // USART1, USART2 and USART3
    #if defined NoGenericSerial
      // NOTE NOTE NOTE NOTE! In IDE select Tools/U(S)ART support: "Enabled (no generic 'Serial')"
      // Now we map USARTS in a sensible way
      HardwareSerial Serial(USART1);  //rx1=PA10 tx1=PA9  - for flashing and monitor
      //HardwareSerial InSerial(USART2); //rx2=PA3  tx2=PA2  - for telemetry in
      HardwareSerial Serial2(USART3); //rx3=PB11 tx3=PB10 - for GPS if present
    #else
      #define inSerial  Serial1
    #endif 
////    #include <SoftwareSerial.h>  
////    SoftwareSerial inSerial(in_rxPin, in_txPin, in_invert); // RX=10, TX=11 
  #endif

  #define log                   Serial         // USB / Serial 

  #if defined ESP32               // U0UXD, U1UXD and U2UXD  UART0, UART1, and UART2
    #if (MEDIUM_IN == 1)          // UART Serial
      HardwareSerial inSerial(1);                                                 
    #elif (MEDIUM_IN == 3)        // BLUETOOTH Serial
      BluetoothSerial inSerial; 
    #endif  
  #endif
 
   #if (PROTOCOL == 8) || (HEADINGSOURCE == 4)  // NMEA GPS or Box GPS
    #include <TinyGPS++.h>
  #endif

  #if (HEADINGSOURCE == 4) 
    #define boxgpsSerial           Serial2        // ESP32 and STM32F1xx Tracker box GPS
  #endif  
  
