 //================================================================================================= 
//================================================================================================= 
//
//                                    C O N F I G U R A T I O N 

#define MAJOR_VERSION       2
#define MINOR_VERSION      22
#define PATCH_LEVEL        31

//=============================================================================================
//================== Please select your options below before compiling ========================
//=============================================================================================

#define DEVICE_SYSID     251                     // Our Mavlink Identity - APM FC is 1, Mission Planner is 255, QGC default is 0 
#define DEVICE_COMPID    MAV_COMP_ID_PERIPHERAL  // 158 Generic autopilot peripheral - APM FC is 1, MP is 190, QGC is  https://mavlink.io/en/messages/common.html

//=============================================================================================
//============================    D E T E C T   M C U   F A M I L Y   =========================
//=============================================================================================
//                Don't change anything here
//
#if defined (__MK20DX128__) || defined(__MK20DX256__)
  #define TEENSY3X  
  #define TARGET_FAMILY   1      // Teensy 3.1 and 3.2  
  #define TEENSY_VARIANT 1    
#endif  
#if defined(__IMXRT1052__) || defined(__IMXRT1062__)
  #define TEENSY4X  
  #define TARGET_FAMILY   1      // Teensy 4.2  
  #define TEENSY_VARIANT 2  
#endif    
// Using official stmicroelectronics lib: https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json 
#if defined (STM32F1xx)   // in boards.txt / build.variant, like GenF1.menu.pnum.BLUEPILL_F103C8.build.variant=STM32F1xx/F103C8T_F103CB(T-U)
  #define TARGET_FAMILY   2      // Blue Pill STM32F103C8 
  #define STM32_VARIANT  1
#endif    
#if defined ESP32
  #define TARGET_FAMILY   3      // Espressif ESP32 
#endif    
#if defined ESP8266
  #define TARGET_FAMILY   4      // Espressif ESP8266
#endif 
#if defined(CONFIG_IDF_TARGET_ESP32C3)
  #define ESP32CX 3
#elif CONFIG_IDF_TARGET_ESP32C6 
#define ESP32CX  6   
#elif CONFIG_IDF_TARGET_ESP32S2
#define ESP32Sx  2        
#elif CONFIG_IDF_TARGET_ESP32S3
#define ESP32Sx 3   
#elif CONFIG_IDF_TARGET_ESP32S6
#define ESP32Sx 6        
#elif CONFIG_IDF_TARGET_ESP32H2
  #define ESP32Hx 2   
#endif

//=============================================================================================
//========================      S E L E C T   M O T O R   T Y P E      ========================
//=============================================================================================
 
// Select only one motor type
//#define STEPPERS 
#define SERVOS

//=================================================================================================                            
//===========================     S E L E C T   B O A R D   V A R I A N T     =====================  
//=================================================================================================

// Board is derived from board selected in IDE, variant is selected here
// Select only 1 variant

//#define TEENSY_VARIANT    1      //   (TEENSY3x) 
//#define TEENSY_VARIANT    2      //   (TEENSY4x
//#define STM32_VARIANT     1      // STM32 Blue Pill F103C8 with 128k
//#define ESP32_VARIANT     1    //  ESP32 Dev Module
//#define ESP32_VARIANT     4    // Heltec Wifi Kit 32 V3 (S3) (thanks to Marc Dornan)
//#define ESP32_VARIANT     5    //  LILYGO® TTGO T-DISPLAY ESP32 1.14" ST7789 Colour LCD, IDE board = "ESP32 Dev Module"
//#define ESP32_VARIANT     6    // LILYGO® TTGO T2 ESP32 OLED Arduino IDE board = "ESP32 Dev Module"
//#define ESP32_VARIANT     7    // ESP32 Dev Module with ILI9341 2.8" colour TFT SPI 240x320 NOT TESTED _ DON'T USE YET
//#define ESP32_VARIANT     8    // Lilygo T-Display-S3 ESP32-S3 1.9 in ST7789V LCD no touch screen 
//#define ESP32_VARIANT     9    // ESP32-S3-DEVKIT-1 no display
//#define ESP32_VARIANT    10    // ESP32-C3 Supermini
//#define ESP8266_VARIANT   1    // Lolin NodeMCU 12E/F board - Select NodeMCU 12E in IDE 

/*
                PLATFORMIO BOARD VARIANT SELECTION OVERRIDE
*/
#if defined PIO_BUILD
  #if (defined PIO_ESP32_VARIANT)
    #undef ESP32_VARIANT
  #endif
  #define ESP32_VARIANT PIO_ESP32_VARIANT
#endif  
#if defined PIO_BUILD
  #if (defined PIO_ESP8266_VARIANT)
    #undef ESP8266_VARIANT
  #endif
  #define ESP8266_VARIANT PIO_ESP8266_VARIANT
#endif  
#if defined PIO_BUILD
  #if (defined PIO_STM32_VARIANT)
    #undef STM32_VARIANT
  #endif
  #define STM32_VARIANT PIO_STM32_VARIANT
#endif  
//=============================================================================================
//====================  I N P U T   M E D I U M    How does telemetry enter the tracker?
//=============================================================================================
// Choose only one input medium 
//#define MEDIUM_IN  1    // UART (Serial)   Teensy, STM32F103, ESP8266 and ESP32   
#define MEDIUM_IN  2    // WiFi UDP - ESP only
//#define MEDIUM_IN  3    // Bluetooth Classic - ESP32 only
//#define MEDIUM_IN  4    // Bluetooth Low Energy (BLE4.2)- ESP32 only
//#define MEDIUM_IN  5    // ESPNOW - ESP32 and CRSF protocol only

//=============================================================================================
//================================  T E L E M E T R Y   P R O T O C O L  ======================
//=============================================================================================
// Select only one telemetry PROTOCOL here
//#define PROTOCOL 0     // AUTO detect protocol   at present UART, BT Classic or BLE only
//#define PROTOCOL 1     // Mavlink 1
//#define PROTOCOL 2     // Mavlink 2
//#define PROTOCOL 3     // FrSky S.Port
//#define PROTOCOL 4     // FrSky F.Port 1
//#define PROTOCOL 5     // FrSky F.Port 2
//#define PROTOCOL 6     // LTM
//#define PROTOCOL 7     // MSP - Not supported - Possible future support
//#define PROTOCOL 8     // GPS NMEA
#define PROTOCOL 9     // CRFS

//=============================================================================================
//==================================  H E A D I N G   S O U R C E  ============================
//=============================================================================================
// Select one heading source. We need this to relate the external world of co-ordinates to the internal tracker co_ordinates.
//#define HEADINGSOURCE  1     // 1=Flight Computer GPS
#define HEADINGSOURCE  2     // 2=Flight Computer Compass
//#define HEADINGSOURCE  3     // 3=Trackerbox_Compass 
//#define HEADINGSOURCE  4     // 4=Trackerbox_GPS_And_Compass
// Select GPS module serial link speed. Many GPS modules are capable of using multiple serial speed out from the box.
// This information should be provided by the manufacturer.
// If not defined, speed will be selected automatically.
 #define BOX_GPS_BAUD 115200

// Select compass type. This information should be provided by the manufacturer.
#define HMC5883L
//#define QMC5883L

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
  const char* BT_Slave_Name   =   "TX16S"; 
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
//============================    ESPNOW  ELRS BACKPACK  SETTINGS   ===========================
//=============================================================================================
// Please use ExpressLRS Configurator Runtime Options to obtain your UID (unique MAC hashed 
// from binding_phrase) Insert the six numbers between the curly brackets below
#if (MEDIUM_IN == 5)
  const uint8_t myUID[6] = {231,140,80,73,149,3}; // derived from your hashed binding phrase
  //const uint8_t myUID[6] = {208,200,225,230,184,24}; // this is my test UID. It will not work for you
#endif
//=============================================================================================
//=================================  O T H E R   S E T T I N G S   ============================
//=============================================================================================
//#define QLRS           // Un-comment if you use the QLRS variant of Mavlink 

//#define DATA_STREAMS_ENABLED   // Mavlink only - Requests data streams from FC. Requires both rx and tx lines to FC. Rather set SRn in Mission Planner

//=================================================================================================   
//================================   M O T O R   L I B R A R I E S   ============================== 
//=================================================================================================  
#if defined TEENSY3X         
  #include <PWMServo.h>  
  #define _PWMSERVO   
#elif defined STM32F1xx
  #include <MobaTools.h>
  #define _MOBASERVO     
#elif defined ESP32 
  // if ESP32Cx or Sx MCU, then Mobatools does not support! Use other libraries
  #if (defined ESP32CX) || (defined ESP32SX)
    #if defined STEPPERS
      #include <AccelStepper.h> 
      #define _ACCELSTEP    
    #elif defined SERVOS
      #include <ESP32Servo.h> 
      #define _ESPSERVO   
    #endif  
  #else                    // regular ESP32, not Cx or Sx
    #include <MobaTools.h>  
    #if defined STEPPERS
      #define _MOBASTEP  
    #elif defined SERVOS
      #define _MOBASERVO   
    #endif  
  #endif  
#elif (defined ESP8266)  
  #include <MobaTools.h>    
  #if defined STEPPERS
    #define _MOBASTEP 
  #else
    #define _MOBASERVO
   // #include <Servo.h> // investgate ESP8266_ISR_Servo library
   // #define _8266SERVO  // Arduino/Espressif lib   
  #endif          
#endif
//=============================================================================================
//===================================  M O T O R   S E T T I N G S  ===========================
//=============================================================================================

  #define TEST_MOTORS    // Move servos/steppers through their limits, then try box_hdg every 45 degrees of circle

  #define AZ_SERVO_360   // Means the azimuth servo can point in a 360 deg circle, elevation servo 90 deg
                           // Default (comment out #define above) is 180 deg azimuth and flip over 180 deg elevation 

  #define azMidFront  90    // 0 deg = left, 90 deg = straight ahead, 180 deg = right, 270 deg = behind
  #define elStart      0    // 0 = horizontal, 90 = vertical

  #if defined STEPPERS
    typedef enum direction_set {none= 0, CW = 1, CCW = -1 } direction_t;
      direction_t st_direction = CW;

    const uint16_t st_gear_ratio = 2;  // Inverse. So 10 means 1/10. Gear ratio of motor gearbox. 1 if no gearing      
    std::string s_dir;   
  #endif
  #if defined SERVOS
    #define SERVO_SPEED     25   // 5=very slow, 0=no effect
  /* MobaTools QUOTE
      "Set the speed of movement of the servo. 'Speed' is the value by which the pulse length changes 
      every 20ms when the servo moves. For example, Speed=20 means that 8 seconds are needed to change
      the pulse length from from 1ms to 2ms"
  */
  #endif

  #if defined _MOBASTEP        
    //st_direction = CW;                           
    const uint8_t mbst_speed = 30;                  // relative speed recommended range 1 - 30
    const float mbst_ramp = 1;                    // portion of angular range for speed ramp_up and ramp down
    const int mbstStepRev = 1600 * st_gear_ratio; // 1600 steps per revolution
  #endif
  #if defined _ACCELSTEP
    uint32_t acst_button_millis = 0;
    int16_t acst_deg = 0;
    int16_t acst_az_step = 0;
    int16_t acst_el_step = 0;
    const int16_t max_steps = 3200;       // steps per 360deg, set TB6600 same (200 base steps x 16 or 32)
    const float acst_steps_per_deg = (float)max_steps / (float)360;
    bool  acst_adjustButtonActive = false;
    const uint16_t acst_max_speed = 1000;      // steps/s - 6400 == 60RPM, 1rp/s
    const uint16_t acst_accel = 2000;          // steps/s/s
   
  #endif
  #if defined _MOBASERVO
    /* 
      if required, you must adjust your servo movement limits in /lib/MobaTools.h around line 207

      #define MINPULSEWIDTH   550  
      #define MAXPULSEWIDTH   2600
    */
  #endif

  // set the degree range of the motors here                     
  // motor movement is constrained by these values
  #if defined AZ_SERVO_360   // 1 x 360, 1 x 90 (or 180) motors  
    const int16_t minAz = 0;          // Az lower limit in degrees, left of tracker facing flying field
    const int16_t maxAz = 359;        // Az upper limit in degrees
    const int16_t minEl = -10;          // El lower limit in degrees, horizontal 
    const int16_t maxEl = 85;         // El upper limit in degrees, straight up
  #else                      // 2 x 180 deg motors
    const int16_t minAz = 0;          // Az lower limit in degrees, left of tracker facing flying field
    const int16_t maxAz = 180;        // Az upper limit in degrees, right of tracker facing flying field
    const int16_t minEl = 15;         // 0 El lower limit in degrees, horizontal and forward
    const int16_t maxEl = 150;        // 180 El upper limit in degrees, horizontal and rearward
  #endif 

  // Sometimes the mechanical movement of a motor is reversed due to the orientation of its mounting
  // Its movement may be reversed here to compensate
  //#define REVERSEAZIMUTH          // my azimuth servo has a reversed action
  //#define REVERSEELEVATION
  // Default values for SG90 servos; 500 and 2400
  // My 180 deg servos have a PWM range of 700 through 2300 microseconds. Your's may differ. 
  // ADJUST THE MECHANICAL LIMITS OF MOVEMENT OF YOUR SERVOS HERE BELOW

  const uint16_t minAzPWM = 625;   // right (because mine is reversed)
  const uint16_t maxAzPWM = 2235;  // left 
  const uint16_t minElPWM = 600;   // front
  const uint16_t maxElPWM = 2257;  // back  

//=============================================================================================
//================================   W I F I   S E T T I N G S  ===============================  
//=============================================================================================

#define HostName    "Tracker"  
#define APssid      "Tracker"
#define APpw        "password"         // Change me! Must be >= 8 chars  
#define STAssid     "MavToUDP"
#define STApw       "password"         // Change me! Must be >= 8 chars 
#define APchannel            9                  // The wifi channel to use for our AP
 
// Choose one default mode for ESP only - AP means advertise as an access point (hotspot). STA means connect to a known host
#define WIFI_MODE  1  //AP            
//#define WIFI_MODE   2  // STA
//#define WIFI_MODE   3  // (STA>AP) STA failover to AP 
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
//========================   Determine Board Group Capability  ================================
//=============================================================================================

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

#if (TARGET_FAMILY == 1)           //   (TEENSY) 
  #if (TEENSY_VARIANT == 1)          // TEENSY 3.1/3.2
    #define in_rxPin        0       // rx1 tx1 - Serial1
    #define in_txPin        1
    #if (HEADINGSOURCE == 4)  // Tracker box GPS 
      int8_t boxgps_rxPin =   9;      // rx2 tx2 - Serial2 for tracker box GPS if applicable
      #define boxgps_txPin   10  
    #endif  
    bool in_invert = false;          // ONLY FOR FrSky S.Port, NOT F.Port, NOT MAVLINK     
    #define frOneWire     true      // ONLY FOR FrSky S.Port
    #define set_Pin         11
    #define StatusLed      14
    #define BuiltinLed     -1     // none, but must be defined!
    #define azPWM_Pin       5
    #define elPWM_Pin       6
    #define BuiltinLed     13
    #define SSD1306_DISPLAY         // Optional OLED display type    
    /* Below please choose either Touch pin-pair or Digital pin-pair for display scrolling
      *  Pin == -1 means the pin-pair is not used
      */ 
    #define SDA            17  // I2C OLED board and/or Compass - default can be changed in Wire.h 
    #define SCL            16  // I2C OLED board and/or Compass - default can be changed in Wire.h 
  #endif  
  //========================================================================= 
  #if (TEENSY_VARIANT == 2)          // Teensy4x
    #define in_rxPin        0       // rx1 tx1 - Serial1
    #define in_txPin        1
    #if (HEADINGSOURCE == 4)  // Tracker box GPS 
      int8_t boxgps_rxPin =   9;      // rx2 tx2 - Serial2 for tracker box GPS if applicable
      #define boxgps_txPin   10 
    #endif  
    bool in_invert = true;          // ONLY FOR FrSky S.Port, NOT F.Port, NOT MAVLINK     
    #define frOneWire     true      // ONLY FOR FrSky S.Port
    #define set_Pin         11
    #define StatusLed      14
    #define BuiltinLed     -1     // none, but must be defined!
    #define azPWM_Pin       5
    #define elPWM_Pin       6
    #define BuiltinLed     13
    #define SSD1306_DISPLAY         // Optional OLED display type    
    /* Below please choose either Touch pin-pair or Digital pin-pair for display scrolling
      *  Pin == -1 means the pin-pair is not used
      */ 
    #define SDA            17  // I2C OLED board and/or Compass - default can be changed in Wire.h 
    #define SCL            16  // I2C OLED board and/or Compass - default can be changed in Wire.h 
  #endif  
//=========================================================================   

#elif (TARGET_FAMILY == 2)         // STM32F1xx Blue Pill
  #if (STM32_VARIANT == 1)
    // There are three USART peripherals on the STM32F103, rx1/tx1 (flash, monitor), rx2/tx2, rx3/tx3
                          // PA10   // rx1 Serial(0) flash and monitor    
                          // PA9    // tx1 Serial(0) flash and monitor
    int8_t in_rxPin =         PA3;   // rx2 Serial1
    #define in_txPin          PA2    // need for mavlink tx2 Serial1
    #if (HEADINGSOURCE == 4)  // Tracker box GPS 
      int8_t boxgps_rxPin =     PB11;  // rx3 Serial2
      #define boxgps_txPin      -1     // not needed tx3 Serial2
    #endif  
    bool in_invert = false;          // ONLY FOR FrSky S.Port, NOT F.Port, NOT MAVLINK    
    #define set_Pin           PA5    //PA0    
    #define StatusLed         PA6    // Off=No good GPS yet, flashing=good GPS but home not set yet, solid = ready to track
    #define BuiltinLed        -1     // none, but must be defined!
    #define azPWM_Pin         PA7    // azimuth servo 
    #define elPWM_Pin         PA8    // elevation servo
    #define BuiltinLed        PC13  
    #define SSD1306_DISPLAY          // Optional OLED display type  
    #define Pup               PA2    // Button 1 to scroll the display up
    #define Pdn               PA4    // Button 2 to scroll the display down   

    #define SDA               PB7  // I2C OLED board and/or Compass - default can be changed in Wire.h 
    #define SCL               PB6  // I2C OLED board and/or Compass - default can be changed in Wire.h 
  #endif  
  //=========================================================================  
   
#elif (TARGET_FAMILY == 3)         // ESP32 Platform

  // For info: Avoid SPI pins - generally   CS=5    MOSI=23   MISO=19   SCK=18  
  /*
  #if defined LED_BUILTIN
    #define BuiltinLed LED_BUILTIN
  #else
    #define BuiltinLed        -1
  #endif
  */
  #if (ESP32_VARIANT == 1)          // ESP32 Dev Module
    #if MEDIUM_IN  == 1             // UART
      int8_t in_rxPin =         27;  // uart1
      #define in_txPin          17 
    #endif
    #if (HEADINGSOURCE == 4)  // Tracker box GPS 
      int8_t boxgps_rxPin =     13;  // uart2 for tracker box GPS if applicable
      #define boxgps_txPin       4  
    #endif  
    bool in_invert = false;         // ONLY FOR FrSky S.Port, NOT F.Port, NOT MAVLINK
    #define set_Pin           12   // LOW == pushed    
    #define StatusLed         2    // Off=No good GPS yet, flashing=good GPS but home not set yet, solid = ready to track
    #define BuiltinLed        -1     // none, but must be defined!
    #if defined SERVOS
      #define azPWM_Pin       32  // azimuth servo (can't be 34,35,36,39 because input only !!)
      #define elPWM_Pin       33  // elevation servo(can't be 34,35,36,39 because input only !!)  
    #endif
    #if defined STEPPERS
      #define adjustPin       26  // white    
      #define azPulsePin      32  // orange
      #define azDirPin        33  // grey
      #define elPulsePin       2  // brown
      #define elDirPin        15  // purple
    #endif

    #define SSD1306_DISPLAY         // OLED display type    
    /* Below please choose either Touch pin-pair or Digital pin-pair for display scrolling
      *  Pin == -1 means the pin-pair is not used
      */ 
    #define Pup            0        // 34 Board Button 1 to scroll the display up
    #define Pdn           35        // Board Button 2 to scroll the display down   
    #define Tup           -1        // Touch pin to scroll the display up
    #define Tdn           -1        // Touch pin to scroll the display down   
        
    #define SDA           21        // I2C OLED board and/or Compass
    #define SCL           22        // I2C OLED board and/or Compass

  #endif
  //========================================================================= 
    
  #if (ESP32_VARIANT == 4) // Heltec Wifi Kit 32 V3 (S3) 
    // NOT TESTED YET
    // For info: Avoid SPI pins - generally   CS=5    MOSI=23   MISO=19   SCK=18  
    #if defined LED_BUILTIN
      #define BuiltinLed LED_BUILTIN
    #else
      #define BuiltinLed        -1
    #endif
    #if MEDIUM_IN  == 1             // UART
      int8_t in_rxPin =         27;  // uart1
      #define in_txPin          17 
    #endif
      #if (HEADINGSOURCE == 4)  // Tracker box GPS 
      int8_t boxgps_rxPin =     13;  // uart2 for tracker box GPS if applicable
      #define boxgps_txPin       4 
    #endif   
    bool in_invert = false;         // ONLY FOR FrSky S.Port, NOT F.Port, NOT MAVLINK
    #define set_Pin           12   // LOW == pushed    
    #define StatusLed         25   // Off=No good GPS yet, flashing=good GPS but home not set yet, solid = ready to track
    #define BuiltinLed        -1     // none, but must be defined!
    #if defined SERVOS
      #define azPWM_Pin       32  // azimuth servo (can't be 34,35,36,39 because input only !!)
      #define elPWM_Pin       33  // elevation servo(can't be 34,35,36,39 because input only !!)  
    #endif
    #if defined STEPPERS
      #define adjustPin       26  // white    
      #define azPulsePin      32  // orange
      #define azDirPin        33  // grey
      #define elPulsePin       2  // brown
      #define elDirPin        15  // purple
    #endif

    #define SSD1306_DISPLAY         // OLED display type    
    /* Below please choose either Touch pin-pair or Digital pin-pair for display scrolling
      *  Pin == -1 means the pin-pair is not used
      */ 
    #define Pup            0        // 34 Board Button 1 to scroll the display up
    #define Pdn           35        // Board Button 2 to scroll the display down   
    #define Tup           -1        // Touch pin to scroll the display up
    #define Tdn           -1        // Touch pin to scroll the display down   
        
    #define SDA           21        // I2C OLED board and/or Compass
    #define SCL           22        // I2C OLED board and/or Compass
    */
  #endif
  //========================================================================= 
      
  #if (ESP32_VARIANT == 5)  /* LILYGO® TTGO T-DISPLAY ESP32 1.14" ST7789 Colour LCD, IDE board = "ESP32 Dev Module"*/
    #if MEDIUM_IN  == 1             // UART
      int8_t in_rxPin =         27;  // uart1
      #define in_txPin          17 
      bool in_invert = false;       // ONLY FOR FrSky S.Port, NOT F.Port, NOT MAVLINK     
    #endif
    #if (HEADINGSOURCE == 4)        // Box GPS and Compass
      int8_t boxgps_rxPin =   13;   // uart2 for tracker box GPS if applicable
      #define boxgps_txPin    -1    // not used
    #endif
    #define StatusLed         25  // Add your own LED with around 1K series resistor
    #define BuiltinLed        -1  // none, but must be defined!
    #define set_Pin           12  // black
    #if defined SERVOS
      #define azPWM_Pin       32  // azimuth servo (can't be 34,35,36,39 because input only !!)
      #define elPWM_Pin       33  // elevation servo(can't be 34,35,36,39 because input only !!)  
    #endif
    #if defined STEPPERS
      #define adjustPin       26  // white    
      #define azPulsePin      32  // orange
      #define azDirPin        33  // grey
      #define elPulsePin       2  // brown
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
    //#define compass_i2c_addr      0x1E   // 0x1E for HMC5883L   0x0D for QMC5883
  #endif
  //========================================================================= 
    
  #if (ESP32_VARIANT == 6)          // LILYGO® TTGO T2 ESP32 OLED Arduino IDE board = "ESP32 Dev Module"
    #if MEDIUM_IN  == 1             // UART
      int8_t in_rxPin =         17;  // uart1
      #define in_txPin          18  
    #endif
    #if (HEADINGSOURCE == 4)  // Tracker box GPS   
      uint8_t boxgps_rxPin =    19;       // uart2 for tracker box GPS
      #define boxgps_txPin      21  
    #endif  
    bool in_invert = true;           // ONLY FOR FrSky S.Port, NOT F.Port, NOT MAVLINK
    #define set_Pin           15
    #define StatusLed         25  // Add your own LED with around 1K series resistor
    #define BuiltinLed        -1  // none, but must be defined!
    #define azPWM_Pin         14  // azimuth servo (can't be 34,35,36,39 because input only !!)
    #define elPWM_Pin         16  // elevation servo(can't be 34,35,36,39 because input only !!)    

    #if !defined SSD1306_DISPLAY    
      #define SSD1306_DISPLAY         // OLED display type
    #endif 
    /* Below please choose either Touch pin-pair or Digital pin-pair for display scrolling
    *  Pin == -1 means the pin-pair is not used
    */        
    #define Pup            0        // Board Button 1 to scroll the display up
    #define Pdn           35        // Board Button 2 to scroll the display down          
    #define Tup           -1        // 33 Touch pin to scroll the display up
    #define Tdn           -1        // 32 Touch pin to scroll the display down  
    
    #define SDA           13        // I2C OLED board 
    #define SCL           14        // I2C OLED board
  #endif 
  //========================================================================= 
  
  #if (ESP32_VARIANT == 7)          // ESP32 Dev Module with ILI9341 2.8" colour TFT SPI 240x320
    #if MEDIUM_IN  == 1             // UART
      int8_t in_rxPin =         16;  // uart1
      #define in_txPin          17    
    #endif
    #if (HEADINGSOURCE == 4)  // Tracker box GPS     
      uint8_t boxgps_rxPin =    13;       // uart2 for tracker box GPS if applicable
      #define boxgps_txPin       4
    #endif  
    bool in_invert = false;              // ONLY FOR FrSky S.Port, NOT F.Port, NOT MAVLINK
    #define set_Pin            5
    #define StatusLed          2        // Add your own LED with around 1K series resistor  
    #define BuiltinLed        -1    // none, but must be defined!
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
  //=========================================================================   

  #if (ESP32_VARIANT == 8)  // Lilygo T-Display-S3 ESP32-S3 1.9 in ST7789V LCD no touch screen 
  /*   
  see C:\Users\<user>\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.10\variants\lilygo_t_display_s3\pins_arduino.h                                
  uart0(flash & monitor), uart1 and uart2(map pins)   

  static const uint8_t TX = 43;      // u0txd default
  static const uint8_t RX = 44;      // u0rxd
  */
    #if MEDIUM_IN  == 1             // UART
      int8_t in_rxPin =         10;  // uart1
      #define in_txPin          11  
    #endif
    #if (HEADINGSOURCE == 4)  // Tracker box GPS     
      int8_t boxgps_rxPin =     13;  // uart2 for tracker box GPS if applicable
      #define boxgps_txPin      -1   // not needed
    #endif  
    bool in_invert = false;          // ONLY FOR FrSky S.Port, NOT F.Port, NOT MAVLINK
    #define set_Pin            1     // yellow LOW == pushed  
    #define StatusLed          2     // purple  Off=No good GPS yet, flashing=good GPS but home not set yet, solid = ready to track
    #define BuiltinLed        -1     // none, but must be defined!
    /*
    The ESP32-S3 board has 8 PWM channels (all GPIO pins except the input-only pins 34-39) 
    */
    #if defined SERVOS
      #define azPWM_Pin       12     // azimuth servo (can't be 34,35,36,39 because input only !!)
      #define elPWM_Pin       11     // elevation servo(can't be 34,35,36,39 because input only !!)  
    #endif
    #if defined STEPPERS
      #define adjustPin       12     // white    
      #define azPulsePin      11     // orange
      #define azDirPin        10     // light blue
      #define elPulsePin      18     // green
      #define elDirPin        21     // blue
    #endif

    #define ST7789_DISPLAY            // 170 x 320 dot 262K Color TFT LCD
    #define SCR_ORIENT     1          // 1 Landscape or 0 Portrait 
    
    #define Pup            0           // Hardwired board Button 1 to scroll the display up
    #define Pdn           14           // Hardwired board Button 2 to scroll the display down   
    #define Tup           -1           // Touch pin to scroll the display up
    #define Tdn           -1           // Touch pin to scroll the display down   
    
    /* 
    Available Pins on Lilygo T-Display_S3
    1, 2, 3   10, 11, 12, 13   16, 17, 18    21    43, 44
    
    Button 1 and 2    0, 14
    From variants\ lilygo_t_display_s3_pins_arduino.h 
    static const uint8_t SDA = 18;
    static const uint8_t SCL = 17; 

    MOBATOOLS SPI (BUT WE DON'T NEED THIS, And hope Moba does not meddle with these pins)
    #define SPI_USED    HSPI
    #define MOSI        13
    #define SCK         14 (Used by Button2)
    #define SS          15

      SPI FLASH
      Do not use these pins in your projects.
        GPIO 26
        GPIO 27
        GPIO 28
        GPIO 29
        GPIO 30
        GPIO 31
        GPIO 32
     ST7789V Display reserved pins
      LCD_Power_on      15
      LCD_BL            38 Backlight
      LCD_D0            39
      LCD_D1            40 
      LCD_D2            41
      LCD_D3            42
      LCD_D4            45 
      LCD_D5            46
      LCD_D6            47
      LCD_D7            48
      LCD_WR            08
      LCD_RD            09
      LCD_DC            07
      LCD_CS            06
      LCD_RES           05                 
    */
  #endif  
  //=========================================================================   

  #if (ESP32_VARIANT == 9)  // ESP32-S3-DEVKIT-1 no display (at this time) 
  /*   
  see C:\Users\<user>\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.10\variants\lilygo_t_display_s3\pins_arduino.h                                
  uart0(flash & monitor), uart1 and uart2(map pins)   

  static const uint8_t TX = 43;      // u0txd default
  static const uint8_t RX = 44;      // u0rxd
  */
    #if MEDIUM_IN  == 1             // UART
      int8_t in_rxPin =         10;  // uart1
      #define in_txPin          11   
    #endif
    #if (HEADINGSOURCE == 4)  // Tracker box GPS     
      int8_t boxgps_rxPin =     13;  // uart2 for tracker box GPS if applicable
      #define boxgps_txPin      -1   // not needed
    #endif  
    bool in_invert = false;          // ONLY FOR FrSky S.Port, NOT F.Port, NOT MAVLINK
    #define set_Pin            1     // yellow LOW == pushed  
    #define StatusLed          2     // purple  Off=No good GPS yet, flashing=good GPS but home not set yet, solid = ready to track
    #define BuiltinLed        -1     // none, but must be defined!
    /*
    The ESP32-S3 board has 8 PWM channels (all GPIO pins except the input-only pins 34-39) 
    */
    #if defined SERVOS
      #define azPWM_Pin       12     // azimuth servo (can't be 34,35,36,39 because input only !!)
      #define elPWM_Pin       11     // elevation servo(can't be 34,35,36,39 because input only !!)  
    #endif
    #if defined STEPPERS
      #define adjustPin       12     // white    
      #define azPulsePin      11     // orange
      #define azDirPin        10     // light blue
      #define elPulsePin      18     // green
      #define elDirPin        21     // blue
    #endif

    /*#define ST7789_DISPLAY            // 170 x 320 dot 262K Color TFT LCD
    #define SCR_ORIENT     1          // 1 Landscape or 0 Portrait 
    
    #define Pup            0           // Hardwired board Button 1 to scroll the display up
    #define Pdn           14           // Hardwired board Button 2 to scroll the display down   
    #define Tup           -1           // Touch pin to scroll the display up
    #define Tdn           -1           // Touch pin to scroll the display down   
    */
    /* 
    Available Pins on Lilygo T-Display_S3
    1, 2, 3   10, 11, 12, 13   16, 17, 18    21    43, 44
    
    static const uint8_t SDA = 18;
    static const uint8_t SCL = 17; 

    MOBATOOLS SPI (BUT WE DON'T NEED THIS, And hope Moba does not meddle with these pins)
    #define SPI_USED    HSPI
    #define MOSI        13
    #define SCK         14 (Used by Button2)
    #define SS          15

      SPI FLASH
      Do not use these pins in your projects.
        GPIO 26
        GPIO 27
        GPIO 28
        GPIO 29
        GPIO 30
        GPIO 31
        GPIO 32
       
    */
  #endif  
  #if (ESP32_VARIANT == 10)  // ESP32-C3-Supermini no display (at this time) 
  /*   
    PWM pins GPIO0 thru GPIO21
    UART0:
        TX: GPIO21
        RX: GPIO20
    UART1:
        TX: GPIO19
        RX: GPIO18   
    NO UART2 - use SoftareSerial   
    Colour PWM LED GPIO8 
    SDA_PIN 8
    SCL_PIN 9
  */
    #if MEDIUM_IN  == 1             // UART
      int8_t in_rxPin =         18;  // uart1
      #define in_txPin          19    
    #endif
    #if (HEADINGSOURCE == 4)  // Tracker box GPS   
    // Must use softwareSerial  
      int8_t boxgps_rxPin =     13;  // uart2 for tracker box GPS if applicable
      #define boxgps_txPin      -1   // not needed
    #endif  
    bool in_invert = false;          // ONLY FOR FrSky S.Port, NOT F.Port, NOT MAVLINK
    #define set_Pin             1     // yellow LOW == pushed  
    #define StatusLed          8     // purple  Off=No good GPS yet, flashing=good GPS but home not set yet, solid = ready to track
    #define BuiltinLed        -1     // none, but must be defined!
    // pwm pins on C3 are GPIO0 thru GPIO21
    #if defined SERVOS
      #define azPWM_Pin        2     // D0 azimuth servo 
      #define elPWM_Pin        3     // D1 elevation servo  
    #endif
    #if defined STEPPERS
      #define adjustPin        2     // white    
      #define azPulsePin       3     // orange
      #define azDirPin         6     // light blue
      #define elPulsePin       7     // green
      #define elDirPin        10     // blue
    #endif

  #endif    

#elif (TARGET_FAMILY == 4)         // ESP8266 Platform            

  /*The ESP8266 has two hardware UARTS (Serial ports): UART0 on pins 1 and 3 (TX0 and RX0 resp.), 
    and UART1 on pins 2 and 8 (TX1 and RX1 resp.), however, GPIO8 is used to connect the flash chip
  */  

  //========================================================================= 
  #if (ESP8266_VARIANT == 1)        // Lolin NodeMCU 12E/F board - Select NodeMCU 12E in IDE 
    /*                        D0  flash
                              D1  tx0 Monitor
                              D2  tx1 
                              D3  rx0
                              D8  rx1 and memory
                I2C default   D1 SCL                                 
                              D2 SDA 
           
    */
    #if MEDIUM_IN  == 1             // UART
      int8_t in_rxPin =        D8;  // SoftwareSerial 1
      define in_txPin          D3   // for mavlink only
    #endif   
    #if (HEADINGSOURCE == 4)  // Tracker box GPS 
      int8_t boxgps_rxPin =     D5;  // alt D8 SoftwareSerial 2 for tracker box GPS if applicable
      #define boxgps_txPin      D6   // alt D9
    #endif
    bool in_invert = false;         // ONLY FOR FrSky S.Port, NOT F.Port, NOT MAVLINK
    #define set_Pin           D7   // LOW == pushed    
    #define StatusLed         D4   // Off=No good GPS yet, flashing=good GPS but home not set yet, solid = ready to track
    #define BuiltinLed        -1   // none, but must be defined!
                                   // Also TXD1 - Serial-1 debug log out SHARED WITH BOARD LED  
    //#define INVERT_LED             // NodeMCU LED action is inverse                  
    #if defined SERVOS
      #define azPWM_Pin       D5  // azimuth servo
      #define elPWM_Pin       D6  // connected to flash elevation servo
    #endif
    #if defined STEPPERS  // choose pins, these are not tested at this pont
      #define adjustPin       D10  // white    
      #define azPulsePin      D5  // orange
      #define azDirPin        D6  // grey
      #define elPulsePin      D9  // brown
      #define elDirPin        D10  // purple
    #endif

    #define SSD1306_DISPLAY         // OLED display type    
    #if (defined SSD1306_DISPLAY)
      /*    digital pin-pair for display scrolling    */ 
      #define Pup           -1                  // Button 1 to scroll the display up
      #define Pdn           -1                  // Button 2 to scroll the display down     
      #define SDA           D2        // D4 I2C OLED D4board and/or Compass
      #define SCL           D1        // D5 I2C OLED board and/or Compass
    #endif  
  #endif
#endif
//============================================================================================= 
//======================================= Macro Logic Checks ==================================
//============================================================================================= 

  #if (PROTOCOL == 7) 
    #error MSP not presently supported
  #endif

  #if defined STEPPERS
    #ifndef AZ_SERVO_360
      #define AZ_SERVO_360
    #endif
  #endif

  #ifndef set_Pin
      #error Please define a set_Pin for your board
  #endif
  #if (set_Pin == -1)
      #error Please define a valid set_Pin. set_Pin is mandatory.
  #endif

  #if (defined STEPPERS) && (defined SERVOS)    
      #error Please choose only one one motor_type
  #endif

  #if  (not defined STEPPERS) && (not defined SERVOS)    
      #error Please choose a motor_type
  #endif

  #ifndef TARGET_FAMILY
    #error Please choose at least one target board
  #endif

  #if (TARGET_FAMILY == 1) 
    //#error Teensy 3.x/4.x 
  #endif  

  #if (TARGET_FAMILY == 2) // STM32F1xx
  //   #error STM32F1xx  
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
 

  #ifndef MEDIUM_IN  
    #error Please choose at least one input medium, Serial, Bluetooth, BLE or WiFi 
  #endif
  #if (not defined ESP32) && (not defined ESP8266) 
     #if (MEDIUM_IN  == 2) 
       #error WiFi only works on an ESP32 or ES8266 board
     #endif  
  #endif
  #if (not defined ESP32) 
     #if (MEDIUM_IN  == 3) || (MEDIUM_IN == 4) 
       #error Bluetooth works only on an ESP32 board
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
    #if (MEDIUM_IN == 2)
      #error "WiFi UDP requires a ESP32 or ESP8266"
    #endif
  #endif

  #if (not defined ESP32) 
    #if (MEDIUM_IN == 3)
      #error "Bluetooth requires an ESP32"
    #endif
  #endif

  #if (not defined ESP32)
    #if (MEDIUM_IN == 4)
      #error "BLE 4.2 requires an ESP32" 
    #endif
  #endif

  #if ((not defined ESP32) && (not defined ESP8266))
    #if (MEDIUM_IN == 5)
      #error "ESPNOW requires an ESP32 or ESP8266"
    #endif
  #endif

  #if (MEDIUM_IN == 5) // ESPNOW
    #if  (PROTOCOL != 9)
      #error "Sorry ESPNOW only works with CRSF backpack protocol at present"
    #endif
  #endif

  #if ( (defined SSD1306_DISPLAY) || (defined SSD1331_DISPLAY) || (defined ST7789_DISPLAY) || (defined ILI9341_DISPLAY) )
    #define DISPLAY_PRESENT
  #endif  

  #if (PROTOCOL == 0)
    #if (MEDIUM_IN!= 1) && (MEDIUM_IN!= 3) && (MEDIUM_IN!= 4)
      #error AUTO detect protocol only avaiable for UART, BT Classic or BLE at present
    #endif
  #endif  
  //=================================================================================================   
  //==================================   E E P R O M   S U P P O R T  =============================== 
  //================================================================================================= 

  #include <EEPROM.h>                        // same lib name for esp, stm32
  const uint8_t home_eeprom_offset = 0;      // ESPNOW uses 7B, 24  thru 30
  const uint8_t espnow_eeprom_offset = 24;   // "home" uses (6 x 4B), 0  thru 23
  //=================================================================================================   
  //==================================   B U T T O N   S U P P O R T  =============================== 
  //=================================================================================================  
  #include <ezButton.h>
  #if defined DISPLAY_PRESENT
    #if ( (Pup != -1) && (Pdn != -1) ) 
      ezButton upButton(Pup);
      ezButton dnButton(Pdn);
    #endif 
  #endif    
  ezButton setButton(set_Pin);
  #if defined STEPPERS
    ezButton adjustButton(adjustPin);       // instantiate ezButton objects
  #endif  

  //=================================================================================================   
  //==================================  C O M P A S S    S U P P O R T ==============================  
  //=================================================================================================  
  #if (HEADINGSOURCE  == 3) || (HEADINGSOURCE  == 4)     // 3 = TracerBox_Compass 4=Trackerbox_GPS_And_Compass
    #include <Wire.h>
    #define WIRE_LOADED    
  #endif
  //=================================================================================================   
  //=================================   D I S P L A Y   S U P P O R T  ============================ 
  //=================================================================================================  

  #if defined DISPLAY_PRESENT
    
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
      #if (ESP32_VARIANT == 8)
      // TTGO T_DISPLAY_S3 1.90 170 x 320 Touch Color TFT LCD SPI
        #define SCR_W_PX      170       // Display width, in pixels - always define in portrait
        #define SCR_H_PX      320       // Display height, in pixels
      #else  
        // TTGO T_DISPLAY 1.14 TFT display 135 x 240 SPI
        #define SCR_W_PX      135       // Display width, in pixels - always define in portrait
        #define SCR_H_PX      240       // Display height, in pixels
      #endif  
      //#define SCR_ORIENT  1
      
      #if (SCR_ORIENT == 0)           // portrait
        #define TEXT_SIZE     1                
      #elif (SCR_ORIENT == 1)         // landscape
        #define TEXT_SIZE     2                       
      #endif 
      #define SCR_BACKGROUND        TFT_BLACK  

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
      #define display_i2c_addr      0x3C       // I2C OLED board
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
  boolean ble_received = false;
  uint8_t ble_len = 0;
  uint8_t *ppData = 0;

#endif // end of BLE4

  //=================================================================================================   
  //=========================  W I F I   S U P P O R T - ESP32 and ES8266 Only ======================
  //================================================================================================= 

    int16_t   wifi_rssi;
    
#if ((defined ESP32) ||(defined ESP8266)) && (MEDIUM_IN == 2)  // ESP and WiFi 

    #if (not defined BT_Setup) && (not defined BLE_Setup)
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
    
    #if defined ESP32
        #include <WiFi.h> 
        #include <WiFiAP.h>  // SoftAP        
    #endif   
    #if defined ESP8266
        #include <ESP8266WiFi.h> 
    #endif    
    #include <WiFiClient.h>
  
    #if (WIFI_PROTOCOL == 2)    //  UDP libs
      #include <WiFiUdp.h>      // ESP32 and ESP8266
    #endif    

  #if (PROTOCOL == 1) || (PROTOCOL == 2) 
    #define MAVLINK
  #endif  

  #if (PROTOCOL == 3) || (PROTOCOL == 4) || (PROTOCOL == 5)
    #define FRSKY
  #endif  
   //====================       W i F i   O b j e c t s 

    #define max_clients    6
       uint8_t active_client_idx = 0;
       uint8_t active_udpremoteip_idx = 0; // active remote ip
       uint8_t active_client_obj_idx = 0;

       IPAddress localIP;                      // tcp and UDP
       IPAddress TCP_remoteIP(192, 168, 4, 1); // when we connect to a server in tcp client mode, put the server IP here

#if (WIFI_PROTOCOL == 1)                   // TCP
       WiFiClient *clients[max_clients] = {NULL}; // pointers to TCP client objects (we only need one in this application
#endif

#if (WIFI_PROTOCOL == 2)                  //  UDP
       IPAddress UDP_remoteIP(192, 168, 1, 255); // default is broadcast, but likey to change after connect
       IPAddress udpremoteip[max_clients];       // table of remote UDP client IPs
       uint8_t UDP_remoteIP_B3;                  // last byte of remote UDP client IP
       WiFiUDP udp_object;                       // Instantiate UDP object
#endif

#endif // end of WiFi

       //=================================================================================================
       //==============================   ESPNOW   S U P P O R T - ESP32 Only  ===========================
       //=================================================================================================
#if (MEDIUM_IN == 5)
      #include <ESP_NOW.h>
      #include <esp_wifi.h>
      #include <WiFi.h>

      esp_now_peer_info_t peerInfo;
      bool espnow_received = false;
      int16_t espnow_len = 0;
      int16_t crsf_len = 0;
      uint8_t soft_mac[6];
      uint8_t have_eeprom_mac = 0xcc; // initial value
#endif
       //=================================================================================================
       //================================   U A R T and  B T  S U P P O R T   ============================
       //=================================================================================================

#if defined STM32F1xx // USART1, USART2 and USART3
                                       // #define NO_GENERIC_SERIAL  //
#if defined NO_GENERIC_SERIAL
                                       // NOTE NOTE NOTE NOTE! In IDE select Tools/U(S)ART support: "Enabled (no generic 'Serial')"
       HardwareSerial Serial(USART1);   // Serial rx1=PA10 tx1=PA9  - for flashing and monitor
       HardwareSerial inSerial(USART2); // Serial1 rx2=PA3  tx2=PA2  - for telemetry in
       HardwareSerial Serial2(USART3);  // Serial2 rx3=PB11 tx3=PB10 - for GPS if present
#else
      HardwareSerial inSerial(USART2); //Serial1 rx2=PA3  tx2=PA2  - for telemetry in
      HardwareSerial Serial2(USART3);  //Serial2 rx3=PB11 tx3=PB10 - for GPS if present
#endif
#endif

#define log Serial          // USB / Serial

#if defined ESP32           // U0UXD, U1UXD and U2UXD  UART0, UART1, and UART2
#if (MEDIUM_IN == 1)        // UART Serial
       HardwareSerial inSerial(1); // UART1
#elif (MEDIUM_IN == 3)      // BLUETOOTH Serial
      BluetoothSerial inSerial;
#endif
#endif

#if (defined ESP8266) // Always SoftwareSerial
  #if (MEDIUM_IN == 1)  // UART Serial
  #include <SoftwareSerial.h>
  #define SWSERIAL_INCLUDED
        SoftwareSerial inSerial(in_rxPin, in_txPin, in_invert);
  #elif (MEDIUM_IN == 3) // BLUETOOTH Serial
        BluetoothSerial inSerial;
  #endif
#endif
#if ((PROTOCOL == 8) || (PROTOCOL == 0)) || (HEADINGSOURCE == 4) // NMEA GPS or Box GPS
  #include <TinyGPS++.h>
  struct compdate 
  {
    uint16_t yyyy;  
    uint8_t mm, dd, h, m, s;
  };
struct compdate dt = {0,0,0,0,0,0};
#endif

#if (HEADINGSOURCE == 4)
  #if (defined ESP8266) // Always SoftwareSerial
  #ifndef SWSERIAL_INCLUDED
  #include <SoftwareSerial.h>
  #define SWSERIAL_INCLUDED
        SoftwareSerial boxgpsSerial(boxgps_rxPin, boxgps_txPin);
  #endif
  #else
    #define boxgpsSerial Serial2 // Tracker box GPS
  #endif
#endif
