//================================================================================================= 
//================================================================================================= 
//
//                                    C O N F I G U R A T I O N 

#define MAJOR_VERSION       2
#define MINOR_VERSION      22
#define PATCH_LEVEL         1

//=============================================================================================
//================== Please select your options below before compiling ========================
//=============================================================================================

#define DEVICE_SYSID     251                     // Our Mavlink Identity - APM FC is 1, Mission Planner is 255, QGC default is 0 
#define DEVICE_COMPID    MAV_COMP_ID_PERIPHERAL  // 158 Generic autopilot peripheral - APM FC is 1, MP is 190, QGC is  https://mavlink.io/en/messages/common.html

//=============================================================================================
//========================      S E L E C T   M O T O R   T Y P E      ========================
//=============================================================================================
// Select only one motor type
//#define STEPPERS     
#define SERVOS

//=============================================================================================
//=====================   S E L E C T   E S P   B O A R D   V A R I A N T   ===================
//=============================================================================================
//#define ESP32_VARIANT     1    //  ESP32 Dev Module - there are several sub-variants that work
//#define ESP32_VARIANT     4    //  Heltec Wifi Kit 32 
#define ESP32_VARIANT     5    //  LILYGO® TTGO T-DISPLAY ESP32 1.14" ST7789 Colour LCD, IDE board = "ESP32 Dev Module"
//#define ESP32_VARIANT     6    // LILYGO® TTGO T2 ESP32 OLED Arduino IDE board = "ESP32 Dev Module"
//#define ESP32_VARIANT      7    // ESP32 Dev Module with ILI9341 2.8" colour TFT SPI 240x320 NOT TESTED _ DON'T USE YET

//=============================================================================================
//====================  I N P U T   M E D I U M    How does telemetry enter the tracker?
//=============================================================================================
// Choose only one input medium 
//#define MEDIUM_IN  1    // UART (Serial)       
//#define MEDIUM_IN  2    // WiFi UDP - ESP only
//#define MEDIUM_IN  3    // Bluetooth Classic - ESP32 only
//#define MEDIUM_IN  4    // Bluetooth Low Energy (BLE4)- ESP32 only
#define MEDIUM_IN  5    // ESP_NOW

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
#define PROTOCOL 9     // CRFS

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

//=============================================================================================
//                              S E T   H O M E   O N   A R M
//#define SET_HOME_AT_ARM_TIME      // else set home location with push button
//=============================================================================================

#define HOME_DECAY_SECS 60  // Home data decay secs. if power lost and restored within decay secs, Home is restored from NVM.

//=============================================================================================
//================================    BLUETOOTH CLASSIC SETTINGS   ============================  
//=============================================================================================
// NOTE: The Bluetooth class library uses a lot of application memory. During Compile/Flash
//  you may need to select Tools/Partition Scheme: "Minimal SPIFFS (1.9MB APP ...) in the Arduino IDE
#if (MEDIUM_IN == 3)
  #define BT_MODE  1           // Master Mode - active, initiate connection with slave (name)
  //#define BT_MODE  2           // Slave Mode - passive, advertise our hostname & wait for master to connect to us
  const char* BT_Slave_Name   =   "btslavename"; 
#endif
//=============================================================================================
//==============================    BLUETOOTH LOW  ENERGY SETTINGS   ========================== 
//=============================================================================================
#if (MEDIUM_IN == 4)
  //#define SERVICE_UUID "91bad492-b950-4226-aa2b-4ede9fa42f59"
  //#define CHARACTERISTIC_01_UUID "cba1d466-344c-4be3-ab3f-189f80dd7518"
  #define SERVICE_UUID "0000fff0-0000-1000-8000-00805f9b34fb"            // use me for Frsky Tandem
  #define CHARACTERISTIC_MSG_UUID "0000fff6-0000-1000-8000-00805f9b34fb"   // use me for Frsky Tandem
  #define bleServerName "Hello"  // use me for Frsky Tandem
#endif
//=============================================================================================
//====================================    ESP_NOW SETTINGS   ================================== 
//=============================================================================================
#if (MEDIUM_IN == 5)
// future?
#endif
//=============================================================================================
//=================================  O T H E R   S E T T I N G S   ============================
//=============================================================================================
//#define QLRS           // Un-comment if you use the QLRS variant of Mavlink 

#define DATA_STREAMS_ENABLED        // Requests data streams from FC. Requires both rx and tx lines to FC. Rather set SRn in Mission Planner


//=============================================================================================
//===================================  M O T O R   S E T T I N G S  ===========================
//=============================================================================================

  //#define TEST_MOTORS    // Move servos/steppers through their limits, then try box_hdg every 45 degrees of circle

  //#define AZ_SERVO_360   // Means the azimuth servo can point in a 360 deg circle, elevation servo 90 deg
                           // Default (comment out #define above) is 180 deg azimuth and flip over 180 deg elevation 
  #if (defined STEPPERS)
    #define azMidFront  0   // 270 deg = left, 0 deg = straight ahead, 90 deg = right, 180 deg = behind
  #endif 
  #if (defined SERVOS)                   
    #define azMidFront  90  // 0 deg = left, 90 deg = straight ahead, 180 deg = right, 270 deg = behind
  #endif
  #define elStart   0       // 0 = horizontal, 90 = vertical

  /*    See SERVO_SPEED below     */

  // Set the degree range of the motors here. Do not adjust servo mechanical limits here. See MIN/MAXPULSEWIDTH below.                        
  #if defined AZ_SERVO_360   // 1 x 360, 1 x 90 (or 180) motors  
    int16_t minAz = 0;          // Az lower limit in degrees, left of tracker facing flying field
    int16_t maxAz = 359;        // Az upper limit in degrees
    int16_t minEl = 0;          // El lower limit in degrees, horizontal 
    int16_t maxEl = 90;         // El upper limit in degrees, straight up
  #else                      // 2 x 180 deg motors
    int16_t minAz = 0;          // Az lower limit in degrees, left of tracker facing flying field
    int16_t maxAz = 180;        // Az upper limit in degrees, right of tracker facing flying field
    int16_t minEl = 0;          // El lower limit in degrees, horizontal and forward
    int16_t maxEl = 180;        // El upper limit in degrees, horizontal and rearward
  #endif 

  // Sometimes the mechanical movement of a servo is reversed due to the orientation of its mounting
  // Its movement may be reversed here to compensate
  #define REVERSEAZIMUTH          // my azimuth servo has a reversed action
  //#define REVERSEELEVATION


  // Default values for SG90 servos; 500 and 2400
  // My 180 deg servos have a PWM range of 700 through 2300 microseconds. Your's may differ. 
  // ADJUST THE MECHANICAL LIMITS OF MOVEMENT OF YOUR SERVOS/STEPPERS HERE BELOW

  #if defined STM32F1xx      // my STM32 based tracker has different servos
    const uint16_t minAzPWM = 600;   // right (because mine is reversed)
    const uint16_t maxAzPWM = 2300;  // left   
    const uint16_t minElPWM = 700;   // front 
    const uint16_t maxElPWM = 2300;  // back
  #else
    const uint16_t minAzPWM = 625;   // right (because mine is reversed)
    const uint16_t maxAzPWM = 2235;  // left 
    const uint16_t minElPWM = 600;   // front
    const uint16_t maxElPWM = 2257;  // back  
  #endif

  #if defined SERVOS
    // adjust your servo movement limits here
    // Find these settings in MobaTools.h and change them, or comment them out
    #define MINPULSEWIDTH   613   
    #define MAXPULSEWIDTH   2215  
    #define SERVO_SPEED     100  // Default = 0 - Try 5, 20, 50, 100, 300
  #endif
  #if defined STEPPERS
    int8_t st_direction = 1;                      // CW or -1 CCW
    const uint8_t st_speed = 10;                  // relative speed recommended range 1 - 30
    const float st_ramp_ratio = 0.2;              // portion of angular range for speed ramp_up and ramp down
    const uint16_t st_gear_ratio = 10;            // Inverse. So 10 means 1/10. Gear ratio of motor gearbox. 1 if no gearing
    const int aStepRev = 1600 * st_gear_ratio;    // 1600 steps per revolution
    std::string s_dir; 
  #endif
  #include <MobaTools.h>  // should work with Teensy and STM32 - test this

//=============================================================================================
//================================   W I F I   S E T T I N G S  ===============================  
//=============================================================================================

#define HostName    "crsfUDP"  
#define APssid      "crsfUDP"
#define APpw        "password"         // Change me! Must be >= 8 chars  
#define STAssid     "crsfUDP"
#define STApw       "password"         // Change me! Must be >= 8 chars 
#define APchannel            9                  // The wifi channel to use for our AP
 
// Choose one default mode for ESP only - AP means advertise as an access point (hotspot). STA means connect to a known host
//#define WIFI_MODE   1  //AP            
//#define WIFI_MODE   2  // STA
#define WIFI_MODE   3  // (STA>AP) STA failover to AP 

// Choose one default protocol - for ESP only
//#define WIFI_PROTOCOL 1    // TCP/IP not used here
#define WIFI_PROTOCOL 2    // UDP 

//  const char    *STAssid =     "EZ-WifiBroadcast";    
//  const char    *STApw =       "wifibroadcast";         
//  const char    *STAssid =     "TXMOD-54-DD-FE";   
//  const char    *STApw =       "txmod123"; 

uint16_t  TCP_LOCALPORT = 5760;     // Read port. You listen on this port. 
uint16_t  TCP_REMOTEPORT = 5760;    // Send port. You send to this port.
uint16_t  UDP_LOCALPORT = 14555;    // readPort,  (default 14555) remote host (like MP and QGC) expects to send to this port - Frsky +1
uint16_t  UDP_REMOTEPORT = 14550;   // sendPort,  (default 14550) remote host reads from this port - FrSky +1
uint16_t  udp_read_port = 0;
uint16_t  udp_send_port = 0;

//=============================================================================================
//============================= Auto Determine Target Platform ================================
//=============================================================================================
//
//                Don't change anything here
//
#if defined (__MK20DX128__) || defined(__MK20DX256__)
  #define TEENSY3X
  #define TARGET_BOARD   0      // Teensy 3.1 and 3.2    


// Using official stmicroelectronics lib: https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json 
#elif defined (STM32F1xx)   // in boards.txt / build.variant, like GenF1.menu.pnum.BLUEPILL_F103C8.build.variant=STM32F1xx/F103C8T_F103CB(T-U)
  #define TARGET_BOARD   1      // Blue Pill STM32F103C8  

#elif defined ESP32
  #define TARGET_BOARD   3      // Espressif ESP32 Dev Module

#elif defined ESP8266
  #define TARGET_BOARD   4      // Espressif ESP8266
  
#else
  #error "No board type defined!"
#endif


  #if (defined ESP32 || defined ESP8266) && (MEDIUM_IN == 2) 
    #define wifiBuiltin   //  for this feature we need wifi support compiled in
  #endif    

  #if (defined ESP32) && (MEDIUM_IN == 3) 
    #define btBuiltin   //  for this feature we need BT support compiled in
  #endif
  #if (defined ESP32) && (MEDIUM_IN == 4)
    #define bleBuiltin   //  for this feature we need BLEsupport compiled in
  #endif

 #if ( (defined ESP32) && (MEDIUM_IN  == 3) )
   #if ( (BT_MODE != 1) && (BT_MODE != 2) )
       #error "Please define a bluetooth mode"
   #endif
 #endif  


//=============================================================================================  
//======================= P L A T F O R M   D E P E N D E N T   S E T U P S ===================
//=============================================================================================

#if (TARGET_BOARD == 0)           //   (TEENSY3X) 
  #include <PWMServo.h>     
  #define in_rxPin        0       // rx1 tx1 - Serial1
  #define in_txPin        1
  int8_t boxgps_rxPin =   9;      // rx2 tx2 - Serial2 for tracker box GPS if applicable
  #define boxgps_txPin   10  
  bool in_invert = true;           // ONLY FOR FrSky S.Port, NOT F.Port, NOT MAVLINK     
  #define frOneWire     true      // ONLY FOR FrSky S.Port
  #define setPin         11
  #define StatusLed      14
  #define azPWM_Pin       5
  #define elPWM_Pin       6
  #define BuiltinLed     13
  #undef   
  #define SDA            17  // I2C OLED board and/or Compass - default can be changed in Wire.h 
  #define SCL            16  // I2C OLED board and/or Compass - default can be changed in Wire.h 
  //=========================================================================   
  
#elif (TARGET_BOARD == 1)         // STM32F1xx Blue Pill


  #include <Wire.h>
                         // PA10  // rx1 Serial(0) flash and monitor    
                         // PA9   // tx1 Serial(0) flash and monitor
  int8_t in_rxPin =         PA3;  // rx2 Serial1
  #define in_txPin          PA2   // tx2 Serial1
  int8_t boxgps_rxPin =     PB11;  // rx3 Serial2
  #define boxgps_txPin      PB10   // tx3 Serial2
  bool in_invert = false;           // ONLY FOR FrSky S.Port, NOT F.Port, NOT MAVLINK    
  #define setPin        PA5    //PA0    
  #define StatusLed         PA6  // Off=No good GPS yet, flashing=good GPS but home not set yet, solid = ready to track
  #define azPWM_Pin         PA7  // azimuth servo 
  #define elPWM_Pin         PA8  // elevation servo
  #define BuiltinLed        PC13  
  #define SDA               PB7  // I2C OLED board and/or Compass - default can be changed in Wire.h 
  #define SCL               PB6  // I2C OLED board and/or Compass - default can be changed in Wire.h 
  //=========================================================================  
   
#elif (TARGET_BOARD == 2)    // Maple Mini

  int8_t in_rxPin =        26;  // rx1 Serial1
  #define in_txPin         25   // tx1 Serial1
  int8_t boxgps_rxPin =     8;  // rx2 Serial2
  #define boxgps_txPin      9   // tx2 Serial2
   bool in_invert = true;        // ONLY FOR FrSky S.Port, NOT F.Port, NOT MAVLINK    
  #define setPin        5    
  #define StatusLed         6   // Off=No good GPS yet, flashing=good GPS but home not set yet, solid = ready to track
  #define azPWM_Pin         4   // azimuth servo 
  #define elPWM_Pin         5   // elevation servo  consider pin 10  this pin=rx2 (9=tx2)
  #define BuiltinLed       33   // PB1   
  #define SDA              15   // I2C OLED board and/or Compass - default must be changed in Wire.h 
  #define SCL              16   // I2C OLED board and/or Compass - default must be changed in Wire.h 
  
#elif (TARGET_BOARD == 3)         // ESP32 Platform
  // For info: Avoid SPI pins - generally   CS=5    MOSI=23   MISO=19   SCK=18  
  //=========================================================================  
   
  #if (ESP32_VARIANT == 1)          // ESP32 Dev Module
    int8_t in_rxPin =         27;  // uart1
    #define in_txPin          17 
    int8_t boxgps_rxPin =     13;  // uart2 for tracker box GPS if applicable
    #define boxgps_txPin       4  
    bool in_invert = true;          // ONLY FOR FrSky S.Port and CRSF, NOT F.Port, NOT MAVLINK
    #define setPin        12   // LOW == pushed    
    #define StatusLed         25   // Off=No good GPS yet, flashing=good GPS but home not set yet, solid = ready to track
    #define BuiltinLed        -1
    #define azPWM_Pin         32   // azimuth servo (can't be 34,35,36,39 because input only !!)
    #define elPWM_Pin         33   // elevation servo(can't be 34,35,36,39 because input only !!)
    #define BuiltinLed        02   // PB1   
    #define SSD1306_DISPLAY         // OLED display type    
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
    /*  
      SPI/CS                       Pin 05   For optional TF/SD Card Adapter
      SPI/MOSI                     Pin 23   For optional TF/SD Card Adapter
      SPI/MISO                     Pin 19   For optional TF/SD Card Adapter
      SPI/SCK                      Pin 18   For optional TF/SD Card Adapter  
    */
  #endif
  //========================================================================= 
    
#if (ESP32_VARIANT == 4) // Heltec Wifi Kit 32 V3 (S3) (thanks to Marc Dornan)
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
  #define resetEepromPin -1 // 5, -1=none use non digital touch pin
  #define SSD1306_DISPLAY // OLED display type
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
    
  #if (ESP32_VARIANT == 5)  /* LILYGO® TTGO T-DISPLAY ESP32 1.14" ST7789 Colour LCD, IDE board = "ESP32 Dev Module"*/
    int8_t in_rxPin =         27;   // uart1 for general serial in, including flight gps
    #define in_txPin          17 
    #if (HEADINGSOURCE == 4)        // Box GPS and Compass
      int8_t boxgps_rxPin =   13;   // uart2 for tracker box GPS if applicable
      #define boxgps_txPin    -1    // not used
    #endif
    bool in_invert = false;       // ONLY FOR FrSky S.Port, NOT F.Port, NOT MAVLINK
    #define StatusLed         25  // Add your own LED with around 1K series resistor
    #define BuiltinLed        -1    
    #define setPin            12  // black
    #if defined SERVOS
      #define azPWM_Pin       32  // azimuth servo (can't be 34,35,36,39 because input only !!)
      #define elPWM_Pin       33  // elevation servo(can't be 34,35,36,39 because input only !!)  
    #endif
    #if defined STEPPERS
      #define adjustPin       26  // white    
      #define azStepPin       32  // orange
      #define azDirPin        33  // grey
      #define elStepPin        2  // brown
      #define elDirPin        15  // purple
    #endif
    
    #define ST7789_DISPLAY          // TFT display type - if you have a display you must define which type
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
     
  #if (ESP32_VARIANT == 6)          // LILYGO® TTGO T2 ESP32 OLED Arduino IDE board = "ESP32 Dev Module"
    uint8_t in_rxPin =        17;       // uart1 for general serial in, including flight gps
    #define in_txPin          18 
    uint8_t boxgps_rxPin =    19;       // uart2 for tracker box GPS
    #define boxgps_txPin      21  
    bool in_invert = true;               // ONLY FOR FrSky S.Port, NOT F.Port, NOT MAVLINK
    #define setPin        15
    #define StatusLed         25        // Add your own LED with around 1K series resistor
    #define BuiltinLed        -1    
    #define azPWM_Pin         14  // azimuth servo (can't be 34,35,36,39 because input only !!)
    #define elPWM_Pin         16  // elevation servo(can't be 34,35,36,39 because input only !!)    

    #if !defined SSD1306_DISPLAY    
      #define SSD1306_DISPLAY         // OLED display type
    #endif 
    #undef ST7789_DISPLAY 
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
   
  #if (ESP32_VARIANT == 7)          // ESP32 Dev Module with ILI9341 2.8" colour TFT SPI 240x320
    uint8_t in_rxPin =        16;       // uart1 for general serial in, including flight gps
    #define in_txPin          17 
    uint8_t boxgps_rxPin =    13;       // uart2 for tracker box GPS if applicable
    #define boxgps_txPin       4
    bool in_invert = false;              // ONLY FOR FrSky S.Port, NOT F.Port, NOT MAVLINK
    #define setPin         5
    #define StatusLed          2        // Add your own LED with around 1K series resistor
    #define BuiltinLed        -1    
    #define azPWM_Pin         32    // azimuth servo (can't be 34,35,36,39 because input only !!)
    #define elPWM_Pin         33    // elevation servo(can't be 34,35,36,39 because input only !!)    
    #define ILI9341_DISPLAY         // ILI9341 2.8" COLOUR TFT SPI 240x320 V1.2  
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
//============================================================================================= 
//======================================= Macro Logic Checks ==================================
//============================================================================================= 
  #if defined STEPPERS
    #ifndef AZ_SERVO_360
      #define AZ_SERVO_360
    #endif
  #endif

  #ifndef setPin
      #error Please define a setPin for your board
  #endif
  #if (setPin == -1)
      #error Please define a valid setPin. setPin is mandatory.
  #endif

  #if (defined STEPPERS) && (defined SERVOS)    
      #error Please choose only one one motor_type
  #endif

  #if  (not defined STEPPERS) && (not defined SERVOS)    
      #error Please choose a motor_type
  #endif

  #ifndef TARGET_BOARD
    #error Please choose at least one target board
  #endif

  #if (TARGET_BOARD == 0) 
    //#error Teensy 3.x 
  #endif  

  #if (TARGET_BOARD == 1) 
    #if defined 
  //   #error STM32F1xx  version does not yet support a display
    #endif  
  #endif  

  #if (WIFI_PROTOCOL == 1)
    #if (PROTOCOL != 1) && (PROTOCOL != 2)
      #error WiFi TCP only works for Mavlink at this time
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
 
  #if (TARGET_BOARD == 4) 
    #error ESP8266 should work but you need to work out the detail yourself
  #endif 

  #ifndef MEDIUM_IN  
    #error Please choose at least one input medium, Serial, Bluetooth, BLE or WiFi 
  #endif
  #if not defined ESP32 
     #if (MEDIUM_IN  == 2) || (MEDIUM_IN  == 3) || (MEDIUM_IN  == 4) 
       #error WiFi or Bluetooth works only on an ESP32 board, rather select serial in
     #endif  
  #endif
  
  #if (defined ESP32)
    #ifndef WIFI_MODE 
      #error Please define WIFI_MODE
    #endif  
    #ifndef WIFI_PROTOCOL
      #error Please define WIFI_PROTOCOL
    #endif 
  #endif

  #if not defined PROTOCOL 
     #error Please define a telemetry protocol  
  #endif 

  #if ((not defined ESP32) && (not defined ESP8266))
    #if (MEDIUM_IN == 5)
      #error "ESP_NOW requires an ESP32 or ESP8266"
    #endif
  #endif

  #if (MEDIUM_IN == 5)  // ESP_NOW
    #if (PROTOCOL != 9)
      #error "Sorry ESP_NOW only works with CRSF backpack protocol at present"
    #endif
  #endif
  //=================================================================================================   
  //==================================   B U T T O N   S U P P O R T  =============================== 
  //=================================================================================================  
  #include <ezButton.h>
    #if ( (Pup != -1) && (Pdn != -1) ) 
      ezButton upButton(Pup);
      ezButton dnButton(Pdn);
    #endif   
  ezButton setButton(setPin);
  #if defined STEPPERS
    ezButton adjustButton(adjustPin);       // instantiate ezButton objects
  #endif  
  //=================================================================================================   
  //==================================  C O M P A S S    S U P P O R T ==============================  
  //=================================================================================================  
  #if (HEADINGSOURCE  == 3) || (HEADINGSOURCE  == 4)     // 3 = TracerBox_Compass 4=Trackerbox_GPS_And_Compass
    #define Wire_Loaded
    #include <Wire.h>
  #endif
  //=================================================================================================   
  //=================================   D I S P L A Y   S U P P O R T  ============================ 
  //=================================================================================================  

  #if defined ESP32_VARIANT
    #if (!( (defined SSD1306_DISPLAY) || (defined SSD1331_DISPLAY) || (defined ST7789_DISPLAY) || (defined ILI9341_DISPLAY) ))
      #error please define a display type in your board variant configuration, or disable 
    #endif   

    #if not defined SD_Libs_Loaded    // by SD block
      #include <SPI.h>                // for SD card and/or Display
    #endif  
    #if not defined Wire_Loaded
      #include <Wire.h>
    #endif  
    
     #if (ESP32_VARIANT == 6)
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
    
    #if (defined ST7789_DISPLAY)      // TTGO T_DISPLAY 1.14 TFT display 135 x 240 SPI
      #include <TFT_eSPI.h>           // Remember to select the T_DISPLAY board in   in TFT_eSPI library (135 x 240) 
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
    
    #elif (defined SSD1306_DISPLAY)    // SSD1306 OLED display     (128 x 64) 
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
    #elif (defined SSD1331_DISPLAY)    // SSD1331 0.95" TTGO T2 colour TFT display (96 x 64)
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
    #elif  (defined ILI9341_DISPLAY)    // ILI9341 2.8" COLOUR TFT SPI 240x320 V1.2  
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
    volatile bool up_button = false;
    volatile bool scroll_display = false;

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
  //===================================================================================
  //======================== BLUETOOTH  CLASSIC SUPPORT -  ESP32  Only ================
  //===================================================================================

#if (MEDIUM_IN == 3) // Bluetooth Classic
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
      #if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
      #error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
      #endif
  #else
      #error Bluetooth only available on ESP32
  #endif    
#endif
  //===================================================================================
  //================== BLUETOOTH  LOW ENERGY (BLE4) SUPPORT -  ESP32  Only ============
  //===================================================================================

#if (MEDIUM_IN == 4) // BLUETOOTH LOW ENERGY (BLE4)
  #if (defined ESP32) 
    #define BLE_Setup   // so that WiFi setup does not define these shared variables again
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
      #include "BLEDevice.h"
      #if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
      #error BLE is not enabled! Please run `make menuconfig` to and enable it
      #endif
  #else
      #error BLE only available on ESP32
  #endif  

  static BLEUUID bmeServiceUUID(SERVICE_UUID);
  static BLEUUID msgCharacteristicUUID(CHARACTERISTIC_MSG_UUID);
  static boolean doConnect = false;
  static boolean BLEconnected = false;
  static BLEAddress *pServerAddress;
  static BLERemoteCharacteristic* msgCharacteristic;
  const uint8_t notificationOn[] = {0x1, 0x0};
  const uint8_t notificationOff[] = {0x0, 0x0};
  const uint8_t maxbuf = 64;
  uint8_t msgBuf[maxbuf];
  boolean newMsg = false;
  uint8_t newLen = 0;
  uint8_t *ppData = 0;

#endif // end of BLE4

  //=================================================================================================   
  //=========================  W I F I   S U P P O R T - ESP32 and ES8266 Only ======================
  //================================================================================================= 

    int16_t   wifi_rssi;
    
#if (defined ESP32) && (MEDIUM_IN == 2)  // ESP32 and WiFi 
  
    bool      FtRemIP = true;
    bool showRemoteIP = true;

    #ifndef BT_Setup && (not defined BLE_Setup)
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

    #define max_clients    6
    uint8_t active_udpremoteip_idx = 0;     // active remote ip 
    uint8_t active_client_obj_idx = 0;

    IPAddress localIP;                             // tcp and UDP
    IPAddress TCP_remoteIP(192,168,4,1);           // when we connect to a server in tcp client mode, put the server IP here  

    #if (WIFI_PROTOCOL == 1)     // TCP  
      WiFiClient *clients[max_clients] = {NULL};   // pointers to TCP client objects (we only need one in this application    
    #endif 
    
    #if (WIFI_PROTOCOL == 2)    //  UDP 
      IPAddress UDP_remoteIP(192, 168, 1, 255);    // default is broadcast, but likey to change after connect 
      IPAddress udpremoteip[max_clients];          // table of remote UDP client IPs            
      uint8_t   UDP_remoteIP_B3;                   // last byte of remote UDP client IP  
      WiFiUDP   udp_object;                        // Instantiate UDP object        
    #endif   

#endif  // end of WiFi

  //=================================================================================================   
  //=========================  ESP_NOW   S U P P O R T - ESP32 and ES8266 Only ======================
  //================================================================================================= 
#if (MEDIUM_IN == 5)
  #include <esp_now.h>
  #include <WiFi.h>
  bool espnow_received = false;
  int16_t  espnow_len = 0;
#endif
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
 
   #if ((PROTOCOL == 8) || (PROTOCOL == 0)) || (HEADINGSOURCE == 4)  // NMEA GPS or Box GPS
    #include <TinyGPS++.h>
  #endif

  #if (HEADINGSOURCE == 4) 
    #define boxgpsSerial           Serial2        // Tracker box GPS
  #endif  
  
