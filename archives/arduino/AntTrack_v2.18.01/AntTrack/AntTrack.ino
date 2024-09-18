/*================================================================================================= 

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
    
   ========================================================================
   Boards supported:
   
   ESP32 boards - variety of variants
   STM32 Blue Pill 
   STM32 Black Pill STM32F104
   Maple Mini
   
   Protocols supported: 
   
    Mavlink 1
    Mavlink 2
    FrSky D legacy
    FrSky X, S and F.Port
    FrSky Passthrough (through Mavlink)
    LTM
    NMEA GPS
    UDP Mavlink and FrSky
    MSP remains outstanding.
    
   The code is written from scratch, but I've taken ideas from Jalves' OpenDIY-AT and others. 
   Information and ideas on other protocols was obtained from GhettoProxy by Guillaume S.
   
   The application reads serial telemetry sent from a flight controller or Flight_GPS. It 
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
   FC and compass, see headingSource discussion below);

   For the tracker to position the servos relative to the compass direction of the craft,
   it needs to know the compass direction in degrees where the tracker antenna is pointing 
   at rest. For convenience we call this the headingSource. Three possible heading sources 
   are available:

    1) Flight_GPS - use this if your craft has a GPS, but no FC or compass
 
    Position the tracker box such that the antenna faces the field straight ahead. Power up
    the craft and tracker, and wait for a good GPS lock. The tracker's LED will flash fast. 
    Walk several metres (say 5) straight ahead with the craft, place it on the ground, 
    return and press the Set_Home button. The tracker calculates the compass direction of a 
    vector (line) from where the craft was at the first GPS lock, to where it was when the 
    Set_Home button was pressed, then uses the vector as the assumed direction of the tracker 
    antenna. 

    2) Flight Computer - use this if your craft has a Flight Computer with a compass
 
    Align the craft's heading (where the nose points) with the direction of the tracker 
    antenna, and press the Set_Home button. The tracker will use the heading of the craft 
    obtained from the craft's Flight Computer via telemetry.
       
    3) Tracker's Own Compass - use this if your craft has no compass sensor.
 
    Fasten a suitable magnetometer anywhere on the tracker box, facing the direction of the 
    antenna at rest.

    4) Trackers Own GPS and Compass - use this if you want to track a moving vehicle from a second
    moving vehicle. This could also be used by a tracker on a 'plane to always point an antenna at home,
    or a second vehicle.

  Note that tracking (movement of the antenna) will occur only when the craft is more than 
  minDist = 4 metres from home because accuracy increases sharply thereafter.   
    
  Before you build/compile the tracker firmware, be sure to modify the value of the 
  headingSource constant according to your choice of heading source:

  #define headingSource   2  // 1=Flight_GPS, 2=Flight_Computer, 3=Tracker_Compass  4=Tracker_GPS_And_Compass 

  If your servo pair is of the 180 degree type, be sure to comment out line that looks like
  this:    //#define Az_Servo_360 

  Note that the elevation (180 degree) servo flips over to cover the field-of-view behind 
  you when the craft enters that space.

  If your servo pair comprises a 360 degree azimuth servo and 90 degree elevation servo, please
  un-comment the line that looks like this:    #define Az_Servo_360. 360 degree code contributed by macfly1202
  
  A small double bi-quad antenna has excellent gain, and works well with a vertically polarised antenna
  on the craft. Other antennas can be added for diversity, and some improvement in link resilience has 
  been observed despite the possibly lower gain of the other links. Of course it is possible to stack 
  double bi-quad antennas on the tracker, but more robust mechanicals will be called for.

  Pin connections are defined in config.h
   

*/
//#include <stdio.h>
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
    uint32_t inBaud = 0;    // Includes flight GPS
    uint32_t gpsBaud = 0;   // Tracker attached GPS, not flight GPS
    uint8_t  protocol = 0;

    const uint8_t snp_max = 74;
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
    uint32_t box_loc_millis = 0;
    uint32_t rds_millis = 0;
       
    bool      hbGood = false; 
    bool      mavGood = false;   
    bool      timeGood = false;
    bool      frGood = false;
    bool      frPrev = false; 
    bool      pwmGood = false; 
    bool      pwmPrev = false;
    bool      gpsGood = false; 
    bool      gpsPrev = false;    
    bool      serGood = false;            
    bool      boxgpsGood = false; 
    bool      boxgpsPrev = false; 
    bool      boxmagGood = false;
        
    uint32_t  frGood_millis = 0;
    uint32_t  hbGood_millis = 0;   
    uint32_t  pwmGood_millis = 0;       
    uint32_t  gpsGood_millis = 0;
    uint32_t  boxgpsGood_millis = 0;
    uint32_t  goodFrames = 0;
    uint32_t  badFrames = 0;

    //====================
    bool  wifiSuDone = false;
    bool  wifiSuGood = false;
    bool  btSuGood = false;
    bool  outbound_clientGood = false;
    bool  rxFT = true;
    bool  gotRecord = false; 
    bool  homSaved = false;    
    bool  homeInitialised = false;
    bool  new_GPS_data = false;
    bool  new_boxGPS_data = false;   
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
     float alt_ag;     
    };

    struct Location hom     = {
      0,0,0,0};   // home location

    struct Location cur      = {
      0,0,0,0};   // current location

    struct Location boxGPS      = {             // tracker box
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
  
    #if (defined TEENSY3X)     // Teensy 3.2
      PWMServo azServo;        // Azimuth
      PWMServo elServo;        // Elevation    
    #else
      Servo azServo;            // Azimuth
      Servo elServo;            // Elevation
    #endif

    // Create Bluetooth object
    #if (Telemetry_In == 1) || (Telemetry_In == 4) 
      BluetoothSerial SerialBT;
    #endif


//=================================================================================================   
//                     F O R W A R D    D E C L A R A T I O N S
//=================================================================================================

   #if (defined displaySupport)
     void PaintDisplay(uint8_t, last_row_t);
     void Scroll_Display(scroll_t);
     void SetupLogDisplayStyle();    
     void LogScreenPrintln(String S);   
     void LogScreenPrint(String S);     
     void DisplayFlightInfo();
     void HandleDisplayButtons();         
   #endif  
   
 #if (defined ESP32) || (defined ESP8266)   
   void IRAM_ATTR gotButtonDn();
   void IRAM_ATTR gotButtonUp();
   void IRAM_ATTR gotWifiButton();
   void IRAM_ATTR gotButtonInfo();
   void SetupWiFi();
   bool NewOutboundTCPClient();  
 #endif

 void pointServos(int16_t, int16_t, int16_t);
 void Send_FC_Heartbeat();
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
 uint32_t epochNow(); 
 float RadToDeg(float);
 uint32_t getBaud(uint8_t);

 
//***************************************************
void setup() {
  Log.begin(115200);
  delay(2000);
  Log.println();
  pgm_path = __FILE__;  // ESP8266 __FILE__ macro returns pgm_name and no path
  pgm_name = pgm_path.substring(pgm_path.lastIndexOf("\\")+1);  
  pgm_name = pgm_name.substring(0, pgm_name.lastIndexOf('.'));  // remove the extension
  Log.print("Starting "); Log.print(pgm_name);
  snprintf(snprintf_buf, snp_max, " version:%d.%02d.%02d\n", MAJOR_VERSION,  MINOR_VERSION, PATCH_LEVEL);
  Log.print(snprintf_buf);

   #if (defined ESP32) && ( (Telemetry_In == 2) || (Telemetry_In == 3)) && (defined Debug_WiFi)
   WiFi.onEvent(WiFiEventHandler);   
  #endif  
 
  #if ((defined ESP32) || (defined ESP8266)) && (defined Debug_SRAM)
    Log.printf("Free Heap just after startup = %d\n", ESP.getFreeHeap());
  #endif  
// ======================== Setup I2C ==============================
  #if (( defined displaySupport) && (defined SSD1306_Display) )   // SSD1306 display
    Wire.begin(SDA, SCL);  
  #elif ( (Heading_Source == 3) || (Heading_Source == 4) )        // Compass
    Wire.begin(SDA, SCL);  
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
      display.begin(SSD1306_SWITCHCAPVCC, display_i2c_addr);         
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
    for (eol = 0 ; eol < scr_w_ch ; eol++) {
      clear_line[eol] = ' ';
    }
    clear_line[eol] = 0x00;

    SetScreenSizeOrient(TEXT_SIZE, SCR_ORIENT);

    Log.printf("%dx%d  text_size=%d  char_w_px=%d  char_h_px=%d  scr_h_ch=%d  scr_w_ch=%d\n", scr_h_px, scr_w_px, TEXT_SIZE, char_w_px, char_h_px, scr_h_ch, scr_w_ch);
    
    LogScreenPrintln("Starting .... ");
  #else
    Log.println("No display support selected or built-in");    
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
    #if (ESP32_Variant == 7)
      Log.println("Dev Module with ILI9341 2.8in COLOUR TFT SPI");
      LogScreenPrintln("Dev module + TFT");
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
    Log.println("Serial Telemetry In");
    LogScreenPrintln("Serial Telem In");
  #endif  

  #if (Telemetry_In == 1)  // BT
    Log.println("Mavlink BT In");
    LogScreenPrintln("Mavlink BT In");
  #endif  

  #if (Telemetry_In  == 2)  // WiFi
    Log.println("Mavlink WiFi In");
    LogScreenPrintln("Mavlink WiFi In");
  #endif  

  Log.print("Selected protocol is ");
  #if (PROTOCOL == 0)   // AUTO
    Log.println("AUTO");  
  #elif (PROTOCOL == 1)   // Mavlink 1
    Log.println("Mavlink 1");
  #elif (PROTOCOL == 2) // Mavlink 2
    Log.println("Mavlink 2");
  #elif (PROTOCOL == 3) // FrSky S.Port
    Log.println("S.Port");
  #elif (PROTOCOL == 4) // FrSky F.Port 1
    Log.println("F.Port 1");
  #elif (PROTOCOL == 5) // FrSky F.Port 2
    Log.println("F.Port 2");
  #elif (PROTOCOL == 6) // LTM
    Log.println("LTM");
  #elif (PROTOCOL == 7)  // MSP
    Log.println("MSP");
  #elif (PROTOCOL == 8)  // GPS NMEA
    Log.println("GPS NMEA");
  #endif
  
  EEPROM_Setup();
  
  millisStartup = millis();
  pinMode(SetHomePin, INPUT_PULLUP); 
  pinMode(StatusLed, OUTPUT ); 
  pinMode(BuiltinLed, OUTPUT);     // Board LED mimics status led
  digitalWrite(BuiltinLed, LOW);  // Logic is NOT reversed! Initialse off    

  DisplayHeadingSource();

  #ifdef QLRS
    Log.println("QLRS variant of Mavlink expected"); 
    LogScreenPrintln("QLRS Mavlink expected");
  #endif  
  
  if ( (headingSource == 3) || ( headingSource == 4) ) {  // Tracker_Compass or (GPS + Compass)

    #if defined HMC5883L  
      Log.println("Compass type HMC5883L expected"); 
      LogScreenPrintln("Compass:HMC5883L");    
    #elif  defined QMC5883L
      Log.println("Compass type QMC5883L expected"); 
      LogScreenPrintln("Compass:QMC5883L");    
    #endif        

    boxmagGood = initialiseCompass();  // Also checks if we have a compass on the Tracker 

   /*
    #if defined Debug_All || defined Debug_boxCompass
      Log.println("Display tracker heading for 20 seconds");
      for (int i=1; i<=20;i++) {
        getTrackerboxHeading();
        delay(1000);
      }
    #endif  
    */
    
  }

  // =============================== Setup SERVOS  ==================================
  
  #if (defined TEENSY3X) || (defined ESP32) || (defined ESP8266)     // NOTE: myservo.attach(pin, 1000, 2000);
    azServo.attach(azPWM_Pin, minAzPWM, maxAzPWM); 
    elServo.attach(elPWM_Pin, minElPWM, maxElPWM);     
  #else
    azServo.attach(azPWM_Pin);
    elServo.attach(elPWM_Pin);
  #endif
  
  moveServos(90, 0);   // Intialise servos to az=90 (straight ahead) , el=0 (horizontal)

  #if defined Debug_Servos  
    Log.println("Testing Servos");
   // TestServos();    // Fine tune MaxPWM and MinPWM in Config tab
  #endif  
                 

// ======================== Setup Serial ==============================
  #if (Heading_Source == 4)  // Tracker box  GPS on Serial2

    #if ( (defined ESP8266) || (defined ESP32) )
      gpsBaud = getBaud(gps_rxPin);
      //Log.print("Tracker box GPS baud rate detected is ");  Log.print(gpsBaud); Log.println(" b/s"); 
      Log.printf("Tracker box GPS baud rate detected is %db/s\n", gpsBaud); 
      String s_baud=String(gpsBaud);   // integer to string. "String" overloaded
      LogScreenPrintln("Box GPS at "+ s_baud);
     
      delay(100);
      gpsSerial.begin(gpsBaud, SERIAL_8N1, gps_rxPin, gps_txPin); 
      delay(10);
    #else
      gpsSerial.begin(gpsBaud);
    #endif
    
  #endif

  #if (Telemetry_In == 0)    //  Serial telemetry in

    protocol = PROTOCOL;
    
    #if (PROTOCOL == 1)   // Mavlink 1
      rxInvert = false;
      inBaud = 57600;
      timeEnabled = true;  
    #elif (PROTOCOL == 2) // Mavlink 2
      rxInvert = false;
      inBaud = 57600;
      timeEnabled = true;         
    #elif (PROTOCOL == 3) // FrSky S.Port
      rxInvert = true;
      inBaud = 57600;
    #elif (PROTOCOL == 4) // FrSky F.Port 1
      rxInvert = false;
      inBaud = 115200; 
    #elif (PROTOCOL == 5) // FrSky F.Port 2
      rxInvert = false;
      inBaud = 115200;   
    #elif (PROTOCOL == 6) // LTM
      rxInvert = false;
      inBaud = 2400; 
    #elif (PROTOCOL == 7)  // MSP
      rxInvert = false;
      inBaud = 9600; 
    #elif (PROTOCOL == 8)  // GPS NMEA
      rxInvert = false;
      inBaud = 9600; 
      timeEnabled = true;          
    #elif (PROTOCOL == 0) // AUTO

      // determine polarity of the telemetry - idle high (normal) or idle low (like S.Port)
      pol_t pol = (pol_t)getPolarity(in_rxPin);
      bool ftp = true;
      static int8_t cdown = 30; 
      bool polGood = true;    
      while ( (pol == no_traffic) && (cdown) ){
        if (ftp) {
          snprintf(snprintf_buf, snp_max, "No telem on rx pin:%d. Retrying ", in_rxPin);
          Log.print(snprintf_buf);
          String s_in_rxPin=String(in_rxPin);   // integer to string
          LogScreenPrintln("No telem on rxpin:"+ s_in_rxPin); 
          ftp = false;
        }
        Log.print(cdown); Log.print(" ");
        pol = (pol_t)getPolarity(in_rxPin);
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
          snprintf(snprintf_buf, snp_max, "Serial port rx pin %d is IDLE_LOW, inverting rx polarity\n", in_rxPin);
          Log.print(snprintf_buf);
        } else {
          rxInvert = false;
          snprintf(snprintf_buf, snp_max, "Serial port rx pin %d is IDLE_HIGH, regular rx polarity retained\n", in_rxPin);     
          Log.print(snprintf_buf);   
        }  

         // Determine Baud
        inBaud = getBaud(in_rxPin);
        Log.print("Serial input baud rate detected is ");  Log.print(inBaud); Log.println(" b/s"); 
        String s_baud=String(inBaud);   // integer to string. "String" overloaded
        LogScreenPrintln("Telem at "+ s_baud);
   
        protocol = detectProtocol(inBaud);
      //snprintf(snprintf_buf, snp_max, "Protocol:%d\n", protocol);
      //Log.print(snprintf_buf);
      
      } else {
        pol = idle_high;
        inBaud = 57600;
        protocol = 2;  
      }
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
        Setup_inGPS(); 
        if (headingSource == 2) {  // Flight Computer !! If we have found the NMEA protol then FC is unlikely
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

    #endif // end of protocol selection

    
    #if ( (defined ESP8266) || (defined ESP32) ) 
      delay(100);
      inSerial.begin(inBaud, SERIAL_8N1, in_rxPin, in_txPin, rxInvert); 
      delay(50);
    #elif (defined TEENSY3X) 
      inSerial.begin(inBaud); // Teensy 3.x    rx tx pins hard wired
       if (rxInvert) {          // For S.Port not F.Port
         UART0_C3 = 0x10;       // Invert Serial1 Tx levels
         UART0_S2 = 0x10;       // Invert Serial1 Rx levels;       
       }
       #if (defined frOneWire )  // default
         UART0_C1 = 0xA0;        // Switch Serial1 to single wire (half-duplex) mode  
       #endif    
    #else
      inSerial.begin(inBaud);
    #endif   
  
  #endif
  
// ************************ Setup Bluetooth ***************************  
  #if (defined ESP32)  && ( (Telemetry_In == 1) || (Telemetry_In == 4) )
    #ifdef BT_Master_Mode
      SerialBT.begin("AntTrack", true);            
    #else
        SerialBT.begin("AntTrack");   
    #endif 
      
    bool bt_connected;

    bt_connected = SerialBT.connect(BT_Slave_Name);
    if(bt_connected) {
      btSuGood = true;
      Log.println("Bluetooth connected!");
      LogScreenPrintln("Bluetooth connected!");
    } else {
      Log.println("Bluetooth NOT connected!");
      LogScreenPrintln("BT NOT connected!");    
    }
      
  #endif 
  
  // ================================  Setup WiFi  ====================================
  #if (defined ESP32)  || (defined ESP8266)

   #if (Telemetry_In == 2) || (Telemetry_In == 3)  //  WiFi Mavlink or FrSky
     SetupWiFi();  
     
   #endif
   
 #endif  
      
}

// ==================================================================================

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

  #if (Telemetry_In == 4)         //   FrSky BT
      if (btSuGood) {     
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

      //===============================      Service Tracker Box Compass and GPS if they exist
      #if (Heading_Source == 4) 

        if ( (hbGood) && ((millis() - box_loc_millis) > 500) ) {  // 2 Hz
          box_loc_millis = millis();
          if(boxmagGood) {
            hom.hdg = getTrackerboxHeading();    // hdg
          }
          getTrackerboxLocation();               // lat, lon alt
        }

      #endif
      //===============================       C h e c k   F o r   N e w   A P   C o n n e c t s
   
      #if (Telemetry_In == 2)    // WiFi
  
        AP_sta_count = WiFi.softAPgetStationNum();
        if (AP_sta_count > AP_prev_sta_count) {  // a STA device has connected to the AP
        AP_prev_sta_count = AP_sta_count;
        snprintf(snprintf_buf, snp_max, "Remote STA %d connected to our AP\n", AP_sta_count);  
        Log.print(snprintf_buf);
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

      CheckForTimeouts();          // and service status LED
      
     //==================== Data Streaming Option
  
     #ifdef Data_Streams_Enabled 
       if(mavGood) {                      // If we have a link, request data streams from MavLink every 30s
         if(millis()- rds_millis > 30000) {
           rds_millis=millis();
           //Log.println("Requesting data streams"); 
           //LogScreenPrintln("Reqstg datastreams");    
           RequestDataStreams();   // must have Teensy Tx connected to Taranis/FC rx  (When SRx not enumerated)
         }
      }
    #endif 
    //===============================  H A N D L E   H O M E   L O C A T I O N

    //       D Y N A M I C   H O M E   L O C A T I O N
     
    #if (Heading_Source == 4)        // moving home location
        if (hbGood && gpsGood && boxgpsGood && PacketGood() && new_GPS_data && new_boxGPS_data) {  //  every time there is new GPS data 
          static bool first_dynamic_home = true;
          new_boxGPS_data = false; 
          if (boxmagGood) {
            if (first_dynamic_home) {
              first_dynamic_home = false;
              Log.println("Dynamic tracking (moving home) started");  // moving home tracking
              LogScreenPrintln("Dynamic tracker ok!");
            }        
            getAzEl(hom, cur);   
            if (hc_vector.dist >= minDist) pointServos((uint16_t)hc_vector.az, (uint16_t)hc_vector.el, (uint16_t)hom.hdg);  // Relative to home heading
            new_GPS_data = false;        // cur. location
            new_boxGPS_data = false;     // moving hom. location          
          } else {
            if (first_dynamic_home) {
              first_dynamic_home = false;
              Log.println("Dynamic tracking (moving home) failed. No good tracker box compass!");  
              LogScreenPrintln("Dynamic trackr bad!");
            }       
          }

       }

    //      S T A T I C   H O M E   L O C A T I O N
    
    #else   // end of tracker box gps moving home location, start of static home location

      if (timeGood) LostPowerCheckAndRestore(epochNow());  // only if active timeEnabled protocol
       
      if ( ((timeEnabled) && (lostPowerCheckDone)) || (!timeEnabled)  ) {
        if (gpsGood==1 && ft) {

          if (homeInitialised==0) {
            if (headingSource == 1)  { // GPS
              if (homSaved) {
                ft=false;
                Log.println("Flight GPS lock good! Walk straight ahead 10m then push home button");  
                LogScreenPrintln("GPS lock good. Carry");
                LogScreenPrintln("craft fwrd 10m. Place");
                LogScreenPrintln("and push home button");
              }
            }
           else {  // FC & own tracker compass
              ft=false;
              snprintf(snprintf_buf, snp_max, "GPS lock good! Push set-home button (pin:%d) anytime to start tracking \n", SetHomePin);  
              //Log.printf("GPS lock good! Push set-home button (pin:%d) anytime to start tracking\n", SetHomePin); 
              Log.print(snprintf_buf);
              LogScreenPrintln("GPS lock good! Push");
              LogScreenPrintln("home button");
            }
          }
        }
      }

      if (homeInitialised) {
        if (hbGood && gpsGood && PacketGood() && new_GPS_data) {  //  every time there is new GPS data 
          getAzEl(hom, cur);
          if (hc_vector.dist >= minDist) pointServos((int16_t)hc_vector.az, (int16_t)hc_vector.el, (int16_t)hom.hdg);  // Relative to home heading
          new_GPS_data = false;
        }
      }
      uint8_t SetHomeState = digitalRead(SetHomePin);            // Check if home button is pushed = 0
      if (headingSource==1) {
        if (SetHomeState == 0 && gpsGood && homSaved && !homeInitialised){     // SetHomePin is pulled up - normally high
          FinalStoreHome();  
        }     
      } else
      if (headingSource==2) {      
        if (SetHomeState == 0 && gpsGood && !homeInitialised){     // SetHomePin is pulled up - normally high
          FinalStoreHome();
        }
      }
      if ((lostPowerCheckDone) && (timeGood) && (millis() - millisStore) > 60000) {  // every 60 seconds
        StoreEpochPeriodic();
        millisStore = millis();
      }
      
    #endif   // end of static home

} // end of main loop
//***************************************************
//***************************************************
void FinalStoreHome() {

   switch(headingSource) {
    
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
          Log.print("Heading calculated = "); Log.print(hom.hdg, 1);
          LogScreenPrintln("Heading calculated");      
          Log.println(" Tracking now active!");
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
        hom.hdg = getTrackerboxHeading(); // From own compass
           
        homeInitialised = true;
        DisplayHome();                  
        break;
      case 4:    // Tracker's Own GPS/Compass     
        Log.println("Tracker's Own GPS/Compass. Should never get here because home is dynamic!");
        Log.println("Aborting. Check logic");       
        LogScreenPrintln("Aborting");
        while(1) delay(1000);  // wait here forever           
        break;
      default:   // Unknown protocol 
        Log.println("No headingSource!");
        LogScreenPrintln("No headingSource!");
        LogScreenPrintln("Aborting");
        while(1) delay(1000);  // wait here forever                     
    }
     SaveHomeToFlash(); 
 }
//***************************************************
void AutoStoreHome() {

  hom.lat = cur.lat;
  hom.lon = cur.lon;
  hom.alt = cur.alt;

  // homSaved set true in SaveHomeToFlash
  SaveHomeToFlash();  
  homSaved = true;
  
  LogScreenPrintln("Home loctn auto-stord"); 

  #if defined Debug_All || defined Debug_Status
    Log.print("Static home location AUTO set to ");       
    Log.print("Lat = ");  Log.print(hom.lat, 7);
    Log.print(" Lon = "); Log.print(hom.lon, 7 );        
    Log.print(" Alt = "); Log.println(hom.alt, 1);                 
 #endif 
   
} 
//***************************************************
void DisplayHome() { 
  LogScreenPrintln("Home location set");
  LogScreenPrintln("Tracking now active!");
  #if defined Debug_Minimum || defined Debug_All || defined Debug_AzEl
    Log.print("Static home location set to Lat = "); Log.print(hom.lat,7);
    Log.print(" Lon = "); Log.print(hom.lon,7);
    Log.print(" Alt = "); Log.print(hom.alt,0); 
    Log.print(" Hdg = "); Log.println(hom.hdg,0); 
    LogScreenPrintln("Home set success!");    
 //   DisplayHeadingSource();
  #endif 
}
//***************************************************
void DisplayHeadingSource() {
#if defined Debug_Minimum || defined Debug_All || defined Debug_boxCompass  
  if (headingSource == 1)  {
      Log.println("headingSource = FC GPS"); 
      LogScreenPrintln("HdgSrce=FC GPS");
  }
  else if  (headingSource == 2) { 
      Log.println("Heading source = FC Compass");
      LogScreenPrintln("Headg srce=FC Mag");
  }
  else if (headingSource == 3)   {
      Log.println("headingSource = Tracker Box Compass"); 
      LogScreenPrintln("HdgSrce=Trackr Cmpss");
  }
  else if (headingSource == 4)  {
      Log.println("Dynamic heading source = Tracker Box Compass"); 
      LogScreenPrintln("Dynamic HeadingSrce");
  }  

#endif  
}

//=====================================================

void ServiceTheStatusLed() {
  #ifdef Debug_LEDs
    Log.print("hbGood = ");
    Log.print(hbGood);
    Log.print("   gpsGood = ");
    Log.print(gpsGood);
    Log.print("   boxgpsGood = ");
    Log.print(boxgpsGood);   
    Log.print("   boxmagGood = ");
    Log.print(boxmagGood);       
    Log.print("   homeInitialised = ");
    Log.println(homeInitialised);
 #endif

  if (gpsGood) {
    if ( (homeInitialised) || ( (boxgpsGood) && boxmagGood) )
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
