#include <Arduino.h>
/*================================================================================================= 

    ZS6BUJ's Antenna Tracker

    Eric Stockenstrom - Original code June 2017  
                        Version 2     March 2019
                        Version 2.22  August 2024

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
  By downloading this software you are agreeing to the terms specified in this page and the spirit of thereof.

   Boards supported:
    Advanced features are only available on the ESP32 boards
      ESP8266
      ESP32 boards - number of variants
    Teensy 3.x and 4.x

   Input Media
    Uart
    WiFi
    Bluetooth Classic
    BLE 4.2
    ESP_NOW (ELRS Backpack) from v2.22.04

   Protocols supported: 
    Mavlink 1
    Mavlink 2
    FrSky D legacy
    FrSky X, S and F.Port
    FrSky Passthrough (through Mavlink)
    LTM
    NMEA GPS
    UDP Mavlink and FrSky
    CRSF / ELRS  - as of Nov 2023
    
   The code is written from scratch, but I've taken ideas from Jalves' OpenDIY-AT and others. 
   
   The application reads telemetry sent from a flight controller or Flight_GPS. It 
   calculates where an airbourne craft is in three-dimensional space, relative to the home 
   position. From this it calculates the azimuth and elevation of the craft, and then positions 
   azimuth and elevation PWM controlled servos or stepper motors to point a directional 
   high-gain antenna for radio telemetry, RC control and/or video.

   If stepper motors are selected, please setup speed, ramp_ratio and gear_ratio(if applicable).
   Ramp ratio is used to adjust the rate of acceleration and deceleration of the motor, and hence
   the mass of the antenna. After booting the program will enter a "set mid-front" phase.
   This is necessary to set a nominal zero-degrees or start point for the azimuth stepper motor.
   Use #define azMidFront in motor settings (config.h) to set to 0 (or 90) degrees. Use the Adjust
   button to move the antenna to the nominal middle of the field-of-flight in front of the pilot. 
   When you are satisfied, push the Set button.

   Version 2 samples UART telemetry in, and automatically determines speed (baud) and protocol,
   and supports an OLED display. The ESP board will support UART, WiFi, Bluetooth and BLE 
   telemetry input.
   
   After the craft and the tracker have been powered on, and any telemetry is received by 
   the tracker, the tracker's LED flashes slowly. After a good GPS lock is received, the 
   tracker's LED flashes fast and an appropriate message is displayed on the OLED display.
   We are now in the "set-home" phase.

   In order to establish the home position (latitude, longitude and altitude), push the Set 
   button. The LED goes on solid and an appropriate message is displayed on the OLED display. 
   (If your craft has no FC and compass, see HEADINGSOURCE discussion below);

   For the tracker to position the servos/steppers relative to the compass direction of the 
   craft, it needs to know the compass direction in degrees where the tracker antenna is 
   pointing at rest. For convenience we call this the HEADINGSOURCE. Three possible heading 
   sources are available:

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
       
    3) Tracker's Own Compass - use this if your 'craft has no compass sensor but you have 
    a compass attached to the tracker box,
 
    Fasten a suitable magnetometer anywhere on the tracker box, facing the direction of the 
    antenna at rest.

    4) Trackers Own GPS and Compass - use this if you want to track a moving vehicle from a
    second moving vehicle. This could also be used by a tracker on a 'plane to always point
    an antenna at home, or a second vehicle.

  Note that tracking (movement of the antenna) will occur only when the craft is more than 
  minDist = 4 metres from home because accuracy increases sharply thereafter.   
    
  Before you build/compile the tracker firmware, be sure to modify the value of the 
  HEADINGSOURCE constant according to your choice of heading source:

  #define HEADINGSOURCE   2  // 1=Flight_GPS, 2=Flight_Computer, 3=Tracker_Compass  4=Tracker_GPS_And_Compass 

  If your servo pair is of the 180 degree type, be sure to comment out line that looks like
  this:    //#define AZ_SERVO_360 

  Note that the elevation (180 degree) servo flips over to cover the field-of-view behind 
  you when the craft enters that space.

  If your servo pair comprises a 360 degree azimuth servo and 90 degree elevation servo, please
  un-comment the line that looks like this:    #define AZ_SERVO_360. 360 degree code contributed by macfly1202
  
  A small double bi-quad antenna has excellent gain, and works well with a vertically polarised antenna
  on the craft. Other antennas can be added for diversity, and some improvement in link resilience has 
  been observed despite the possibly lower gain of the other links. Of course it is possible to stack 
  double bi-quad antennas on the tracker, but more robust mechanicals might be called for.

  Pin connections are defined in config.h
   
*/
#include <iostream>
#include <string>
#include <mavlink_types.h>
#include "config.h"                      // ESP_IDF libs included here
#include "debug.h"
#include <globals.h>
#if (PROTOCOL == 1) || (PROTOCOL == 2)  || (PROTOCOL == 0)
  #include <ardupilotmega/mavlink.h>
  #include <ardupilotmega/ardupilotmega.h>
#endif
#if (PROTOCOL == 9) || (PROTOCOL == 0)
  #include <terseCRSF.h>  // https://github.com/zs6buj/terseCRSF   use v 0.0.6 or later
  CRSF   crsf;            // instantiate crsf object
#endif

#if defined PIO_BUILD
  #include <utilities.cpp>
  #include <azEl.cpp>
  #include <boxGPS.cpp>
  #include <compass.cpp>
  #include <crsf.cpp>
  #include <frsky.cpp>
  #include <getProtocol.cpp>
  #include <ltm.cpp>
  #include <mavlink.cpp>
  #include <motors.cpp>
  #include <nmeaGPS.cpp>
#else 
  void IRAM_ATTR gotButtonUp();
  void IRAM_ATTR gotButtonDn();
#endif  

  //================================================================
  #if (MEDIUM_IN == 4)  // BLE 4.2
    class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks 
    {
      void onDisconnect(BLEClient* pclient) 
      {
        BLEconnected = false;
        log.println("Device disconnected\nScanning to reconnect");
        BLEScan* pBLEScan = BLEDevice::getScan();
        pBLEScan->start(0);
      }
      void onResult(BLEAdvertisedDevice advertisedDevice) 
      {
        //log.printf("Device:%s\n", advertisedDevice.toString().c_str());
        if (advertisedDevice.getName() == bleServerName)
        {                                                                 // Check if the name of the advertised device matches
          log.printf("Device:%s found\n", advertisedDevice.toString().c_str());
          advertisedDevice.getScan()->stop();                             // Stop scanning, we found our device
          pServerAddress = new BLEAddress(advertisedDevice.getAddress()); // Address of advertiser
          doConnect = true;                                               // Set flag indicating we are ready to connect
          log.printf("our device found, connecting to %S...\n", bleServerName);
        }
      }
    };
  #endif
  //================================================================
  #if (MEDIUM_IN == 5)  // ESPNOW
    // callback function that will be executed when data is received
    void OnDataRecv(const uint8_t * mac, uint8_t *data, int len) 
    {
      espnow_len = len;
      crsf_len = len - 8;
      espnow_received = true;
      #if (defined DEBUG_ESPNOW)
        log.printf("ESPNow: len:%u, ", len);
        printBytes(&*data, len);
      #endif  
      memcpy(&crsf.crsf_buf, &*(data+8), sizeof(crsf.crsf_buf));
    }
  #endif  
  //====================================
  void buttonEvents()
  {
    #if defined DISPLAY_PRESENT
      #if ( (Pup != -1) && (Pdn != -1) ) 
        if (upButton.isPressed())
        {
          up_button = true;
        }
        if(dnButton.isPressed())
        {
          scroll_display = true;
        }
      #endif  
    #endif
    #if defined STEPPERS
      if (phase == set_midfront)
      {
        if (adjustButton.isPressed()) 
        {
          azStepper.setSpeedSteps(1e4);
          azStepper.rotate(st_direction);
          s_dir = (st_direction == 1) ? "CW" : "CCW";
          std::cout << "dir:" << s_dir << std::endl;
        }
        if (adjustButton.isReleased())
        {
          azStepper.stop();
          delay(10);                              // wait for async .stop to finish
          st_direction = (st_direction == 1) ? -1 : 1;  // ternary toggle direction
          s_dir = (st_direction == 1) ? "CW" : "CCW";
          std::cout << "dir:" << s_dir << std::endl;           
          azStepper.setSpeedSteps((st_speed * 1e4),  (uint32_t)(aStepRev * st_ramp_ratio));
        }  
      }
    #endif  
    #if defined STEPPERS
      if(setButton.isPressed())
      {
        if (phase == set_midfront)
        {
          azStepper.setZero();                    // present position is set as 0 angle (zeropoint)
          log.print("MidFront is "); log.print(azMidFront); log.println(" deg");
          log.println("Starting set_home phase");
          logScreenPrintln("Set_home phase");
          phase = set_home;  
        } 
      } 
    #endif  
    /*   for SERVOS, setButton.isPressed() is also checked in STATIC HOME LOCATION snippet in main()  */
  }

//===========================================================
void setup() 
{ 
  log.begin(115200);
  delay(2000);
  #if defined ESP32
    pgm_path = __FILE__;  // ESP8266 __FILE__ macro returns pgm_name and no path
    pgm_name = pgm_path.substring(pgm_path.lastIndexOf("\\")+1);  
    pgm_name = pgm_name.substring(0, pgm_name.lastIndexOf('.'));  // remove the extension
  #elif defined ESP8266  
    pgm_name = __FILE__;  // ESP8266 __FILE__ macro returns pgm_name and no path
  #endif
  log.print("\nStarting ");  
    #if (defined ESP32) || (defined ESP8266)
    log.println(pgm_name);
  #elif defined STM32F1xx
     log.println("antTrack");  
  #endif  
  log.printf("Version:%d.%02d.%02d\n", MAJOR_VERSION,  MINOR_VERSION, PATCH_LEVEL);

  #if (defined ESP32) && (MEDIUM_IN == 2) && (defined DEBUG_WIFI)
    WiFi.onEvent(WiFiEventHandler);   
  #endif  

   protocol = PROTOCOL;
   medium_in = MEDIUM_IN;
   wifi_protocol = WIFI_PROTOCOL;

  #if ((defined ESP32) || (defined ESP8266)) && (defined DEBUG_SRAM)
    log.printf("Free Heap just after startup = %d\n", ESP.getFreeHeap());  
  #endif  

// =======================  S E T U P   E E P R O M   =======================

  #if (defined ESP32)   // no eeprom setup for stm32f103, teensy(?)
      if (!EEPROM.begin(EEPROM_SIZE)) { // We use 0 thru 4 for "home", and 5 thru 11 for ESPNOW
      log.println("EEPROM failed to initialise"); 
      logScreenPrintln("EEPROM init failed!");
      while (true) delay(1000);
    }
  #elif defined ESP8266
    EEPROM.begin(EEPROM_SIZE);
  #endif  
    #if defined DEBUG_EEPROM
      displayEEPROM(); 
    #endif 

  // =======================  S E T U P   B U T T O N S   =======================
  #if defined DISPLAY_PRESENT
    #if ( (Pup != -1) && (Pdn != -1) ) 
      ezButton upButton(Pup);
      upButton.setDebounceTime(200); // mS
      ezButton dnButton(Pdn);
      dnButton.setDebounceTime(200); 
    #endif   
  #endif  
  ezButton setButton(setPin);
  setButton.setDebounceTime(200); // mS
  #if defined STEPPERS
    ezButton adjustButton(adjustPin);
    adjustButton.setDebounceTime(2000);  
  #endif  
  // ======================== Setup I2C ==============================
  #if (defined SSD1306_DISPLAY) ||  (HEADINGSOURCE == 3) || (HEADINGSOURCE == 4)
    #define NEED_I2C
  #endif
  #if (( defined ESP32 ) || (defined ESP8266) || (defined STM32F1xx))
    #if (defined NEED_I2C)   
      log.printf("Setting up Wire I2C: SDA:%u, SCL:%u\n", SDA, SCL); 
      Wire.begin(SDA, SCL);  
      //scanI2C(); 
    #endif
  #else  // Teensy
    #if (NEED_I2C)
      log.println("Default I2C pins are defined in Wire.h");
    #endif
  #endif  
  //=================================================================================================   
  //                                   S E T U P   D I S P L A Y
  //=================================================================================================
  #if (defined DISPLAY_PRESENT) 
    #if (defined ESP32)
      if ( (Tup != -1) && (Tdn != -1) ) 
      {   // enable touch gpio-pair
        touchAttachInterrupt(digitalPinToInterrupt(Tup), gotButtonUp, threshold);
        touchAttachInterrupt(digitalPinToInterrupt(Tdn), gotButtonDn, threshold);   
      } 
    #endif  

    #if (defined ST7789_DISPLAY)               // LILYGO® TTGO T-Display ESP32 1.14" ST7789 Colour LCD (135 x 240)
      display.init(); 
      #define SCR_BACKGROUND TFT_BLACK
      
    #elif (defined SSD1306_DISPLAY)            // all  boards with SSD1306 OLED display (128 x 64)
      #if not defined TEENSY3X                 // Teensy uses default SCA and SCL in teensy "pins_arduino.h" 
      #endif   
      display.begin(SSD1306_SWITCHCAPVCC, display_i2c_addr);         
      #define SCR_BACKGROUND BLACK   
      
    #elif (defined SSD1331_DISPLAY)            // T2 board with SSD1331 colour TFT display (96 x 64)
        //  uses software SPI pins defined in config.h 
        display.begin();
        #define SCR_BACKGROUND BLACK  
        
    #elif (defined ILI9341_DISPLAY)    // ILI9341 2.8" COLOUR TFT SPI 240x320 V1.2 (320 x 240)
        //  uses hardware SPI pins defined in config.h 
        display.begin();
        #define SCR_BACKGROUND ILI9341_BLUE
    #endif
   
    #if (SCR_ORIENT == 0)              // portrait
        log.print("Display Setup: Portrait ");       
    #elif (SCR_ORIENT == 1)            // landscape   
        log.println("Display support activated: Landscape "); 
    #endif

    int eol= 0;
    for (eol = 0 ; eol < scr_w_ch ; eol++) {
      clear_line[eol] = ' ';
    }
    clear_line[eol] = 0x00;

    setScreenSizeOrient(TEXT_SIZE, SCR_ORIENT);

    log.printf("%dx%d  text_size=%d  char_w_px=%d  char_h_px=%d  scr_h_ch=%d  scr_w_ch=%d\n", scr_h_px, scr_w_px, TEXT_SIZE, char_w_px, char_h_px, scr_h_ch, scr_w_ch);
    logScreenPrintln("Starting .... ");
  #else
    log.println("No display support selected or built-in");    
  #endif
  /*
  display.setFont(Dialog_plain_8);     //  col=24 x row 8  on 128x64 display
  display.setFont(Dialog_plain_16);    //  col=13 x row=4  on 128x64 display
  */
  #if ((defined ESP32) || (defined ESP8266)) && (defined DEBUG_SRAM)
    log.printf("==============>Free Heap after OLED setup = %d\n", ESP.getFreeHeap());
  #endif
  //=================================================================================================   
    
  logScreenPrintln("AntTracker by zs6buj");
  log.printf("Target Board = %u\n", TARGET_BOARD);   
  #if (defined TEENSY3X) // Teensy3x
    log.println("Teensy 3.x");
    logScreenPrintln("Teensy 3.x");
  #elif (defined TEENSY4X)
    log.println("Teensy 4.x");  
  #elif (defined STM32F1xx)
    log.println("STM32F1xx");  
  #elif (defined ESP32) //  ESP32 Board
  delay(10);
    log.print("ESP32/Variant is ");
    logScreenPrintln("ESP32/Variant is");
    #if (ESP32_VARIANT == 1)
      log.println("Dev Module");
      logScreenPrintln("Dev Module");
    #endif
    #if (ESP32_VARIANT == 4)
      log.println("Heltec Wifi Kit 32");
      logScreenPrintln("Heltec Wifi Kit 32");
    #endif
    #if (ESP32_VARIANT == 5)
      log.println("LILYGO® TTGO T-DISPLAY ESP32 1.14 inch ST7789 Colour LCD");
      logScreenPrintln("TTGO T-DISPLAY ESP32");
    #endif   
    #if (ESP32_VARIANT == 6)
      log.println("LILYGO® TTGO T2 ESP32 OLED SD");
      logScreenPrintln("TTGO T2 ESP32 SD");
    #endif 
    #if (ESP32_VARIANT == 7)
      log.println("Dev Module with ILI9341 2.8in COLOUR TFT SPI");
      logScreenPrintln("Dev module + TFT");
    #endif      
  #elif (defined ESP8266) 
    log.print("ESP8266 / Variant is ");
    logScreenPrintln("ESP8266 / Variant is");  
    #if (ESP8266_VARIANT == 1)
      log.println("Lonlin Node MCU 12F");
      logScreenPrintln("Node MCU 12");
    #endif      (MEDIUM_IN == 1)
  #endif

  #if (MEDIUM_IN == 1)  // UART serial
    log.println("Expecting UART Telemetry In");
    logScreenPrintln("UART Telem In");
  #endif  

  #if (MEDIUM_IN  == 2)  // UDP - ESP only
    log.println("Expecting UDP In");
    logScreenPrintln("UDP In");
  #endif  

  #if (MEDIUM_IN == 3)  // Bluetooth Serial - ESP32 only
    log.println("Expecting Bluetooth In");
    logScreenPrintln("Expect BT In");
  #endif  
  
  #if (MEDIUM_IN == 4)  // GBLE4 - ESP32 only
    log.println("Expecting BLE In");
    logScreenPrintln("Expect BLE In");
  #endif 

  #if (MEDIUM_IN == 5)  // ESPNOW - ESP  only
    log.println("Expecting ESPNOW In");
    logScreenPrintln("Expect ESPNOW In");
  #endif 

  log.print("Selected protocol is ");
  #if (PROTOCOL == 0)   // AUTO - does not detect CRSF/ELRS - please select PROTOCOL == 9
    log.println("AUTO");  
  #elif (PROTOCOL == 1)  // Mavlink 1
    log.println("Mavlink 1");
  #elif (PROTOCOL == 2)  // Mavlink 2
    log.println("Mavlink 2");
  #elif (PROTOCOL == 3)  // FrSky S.Port
    log.println("S.Port");
  #elif (PROTOCOL == 4)  // FrSky F.Port 1
    log.println("F.Port 1");
  #elif (PROTOCOL == 5)  // FrSky F.Port 2
    log.println("F.Port 2");
  #elif (PROTOCOL == 6)  // LTM
    log.println("LTM");
  #elif (PROTOCOL == 7)  // MSP
    log.println("MSP - not supported");
  #elif (PROTOCOL == 8)  // GPS NMEA
    log.println("GPS NMEA");
  #elif (PROTOCOL == 9)  // CRSF / ELRS
    log.println("CRSF");    
  #endif

  #if defined SERVOS
    log.println("Expecting SERVO motors");
  #endif
  #if defined STEPPERS
    log.println("Expecting STEPPERS motors");
  #endif

  millisStartup = millis();
  pinMode(StatusLed, OUTPUT ); 
  if (BuiltinLed != -1) {
    pinMode(BuiltinLed, OUTPUT);     // Board LED mimics status led
    digitalWrite(BuiltinLed, LOW);   // Logic is NOT reversed! Initialse off    
  }
  displayHeadingSource(headingsource);
  #ifdef QLRS
    log.println("QLRS variant of Mavlink expected"); 
    logScreenPrintln("QLRS Mavlink expected");
  #endif  
 #if (HEADINGSOURCE  == 3) || (HEADINGSOURCE  == 4) // Tracker_Compass or (GPS + Compass)
    #if defined HMC5883L  
      log.println("Compass type HMC5883L selected"); 
      logScreenPrintln("Compass:HMC5883L");    
    #elif  defined QMC5883L
      log.println("Compass type QMC5883L selected"); 
      logScreenPrintln("Compass:QMC5883L");    
    #endif        
    boxmagGood = initialiseCompass();  // Also checks if we have a compass on the Tracker 
    #if defined DEBUG_BOXCOMPASS
      log.println("Display tracker heading for 20 seconds");
      for (int i=1; i<=20;i++) {
        getTrackerboxHeading();
        delay(1000);
      }
    #endif  
  #endif

  // ====================== SETUP SERVOS or STEPPERS  ===========================
  log.printf("midFront is assumed to be %u deg\n", azMidFront); 
  #if defined SERVOS
    azServo.attach(azPWM_Pin); 
    elServo.attach(elPWM_Pin); 
    #if (defined ESP32) || (defined ESP8266)
      azServo.setSpeed(SERVO_SPEED);   
      elServo.setSpeed(SERVO_SPEED); 
    #endif  
  
    phase = set_home;
    log.println("Starting set_home phase");
    delay(100);
    logScreenPrintln("Set_home phase");
    #if (defined ESP32) || (defined ESP8266)
      if (SERVO_SPEED > 0) 
      {
        log.printf("Servo speed is %u\n", SERVO_SPEED);       
      } else
      {
        log.println("Servo speed is default");
      }
    #endif  
  #endif
  #if (defined STEPPERS) && ((defined ESP32) || (defined ESP8266))
    azStepper.attach(azStepPin, azDirPin);
    azStepper.setSpeedSteps((st_speed * 1e4),  (uint32_t)(aStepRev * st_ramp_ratio)); // speed10== steps in 10s, ramp
    //azStepper.attachEnable(azEnaPin, 200, LOW ); // (pinEna, delay ms before start, active hi or low);
    elStepper.attach(elStepPin, elDirPin);
    elStepper.setSpeedSteps((st_speed * 1e4),  (uint32_t)(aStepRev * st_ramp_ratio)); 
    //elStepper.attachEnable(elEnaPin, 200, LOW );   // save a scarce  
    phase = set_midfront;
    log.printf("Servo speed:%u  ramp_ratio:%1.1f  gear_ratio:1/%u\n", st_speed, st_ramp_ratio, st_gear_ratio);
  #endif

  // ======================== Setup Bluetooth Serial ==========================    
  #if defined ESP32
    #if (MEDIUM_IN == 3)  // Bluetooth Serial
      if (!btSuGood)      // not already set up by auto protocol detect
      {
        #if (BT_MODE == 1)     // 1 master mode, connect to slave name
          log.printf("Bluetooth master mode, looking for slave name \"%s\"\n", BT_Slave_Name);          
          logScreenPrintln("BT master conncting");      
          inSerial.begin(BT_Slave_Name, true);            
        #else                  // 2 slave mode, advertise slave name
            log.printf("Bluetooth slave mode advertising slave name \"%s\"\n", BT_Slave_Name);            
            logScreenPrintln("BT slave ready");   
            inSerialBT.begin(BT_Slave_Name);   
        #endif 
        bt_connected = inSerial.connect(BT_Slave_Name);
        while(!bt_connected) {
          log.print("midFront is "); log.print(azMidFront); log.println("degrees");
          logScreenPrintChar('.');  
          delay(1000);
          bt_connected = inSerial.connect(BT_Slave_Name);       
        }  
        if(bt_connected) {
          btSuGood = true;       
          log.println("Bluetooth connected!");
          logScreenPrintln("BT connected!");
        } else {
          log.println("Bluetooth NOT connected!");
          logScreenPrintln("BT NOT connected");    
        } 

        #if (PROTOCOL == 9) // CRSF
          crsf.initialise(inSerial);  // initialise pointer to Stream &port
        #endif
      }
    #endif // end BT
    #endif // end of ESP32    
    // ======================== Setup Bluetooth Low Energy 4.2 ==========================    
  #if defined ESP32
    #if (MEDIUM_IN == 4)  // BLE 4.2
      //Init BLE device
      BLEDevice::init("");
      // Retrieve a Scanner and set the callback we want to use to be informed when we
      // have detected a new device.  Specify that we want active scanning and start the
      // scan to run asynchronously allways
      log.println("Starting BLE Client");
      log.printf("Scanning for device \"%s\" forever\n", bleServerName);
      logScreenPrintln("BLE scanning..");
      BLEScan* pBLEScan = BLEDevice::getScan();
      pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
      pBLEScan->setActiveScan(true);
      pBLEScan->start(0);  // scan forever
      
    #endif // end BLE 4.2       
  #endif // ESP32
      // ===================   Setup ESPNOW (ELRS Backpack)  ======================   
  #if defined ESP32
    #if (MEDIUM_IN == 5)  // ESPNOW
      if ((myUID[0] == 0) && (myUID[1] == 0) && (myUID[2] == 0) && 
      (myUID[3] == 0) && (myUID[4] == 0) && (myUID[5] == 0)) 
      {
        log.println("Your myUID IS ZERO. Please enter valid MAC values and recompile");
        while(1) delay(1000);  
      }
      uint8_t offset = espnow_eeprom_offset;  // ==24, "home" uses 0 thru 19
      have_eeprom_mac = EEPROMreadByte(offset);
      //if (have_eeprom_mac != 0xfd) for future
      //{
      //  log.println("No valid soft_mac in eeprom");
        macToEeprom(myUID);
      //}
      eepromToMac(soft_mac);
      setSoftMACAddress();
      WiFi.mode(WIFI_STA); // backpack is AP, so we want to be STA
      if (esp_now_init() != ESP_OK) 
      {
        log.println("Error initializing ESP-NOW");
        return;
      }
      memcpy(peerInfo.peer_addr, soft_mac, 6);
      peerInfo.channel = 0;
      peerInfo.encrypt = false;
      if (esp_now_add_peer(&peerInfo) != ESP_OK)
      {
        log.println("ESP-NOW failed to add peer");
        return;
      }
      esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv)); // register esp-now callback
      
    #endif // end ESPNOW     
  #endif // ESP32
  // ======================== S e t u p   T r a c k e r b o x   G P S  =================
  #if (HEADINGSOURCE == 4)  // Tracker box GPS 
    #if ( (defined ESP8266) || (defined ESP32) )
      #if defined BOX_GPS_BAUD 
        boxgpsBaud = BOX_GPS_BAUD;
      #else
        log.print("Getting box-gps baud: ");
        boxgpsBaud = getBaud(gps_rxPin);
      #endif
      log.printf("Tracker box GPS baud rate is %db/s\n", boxgpsBaud);       
      String s_baud=String(boxgpsBaud);   // integer to string. "String" overloaded
      logScreenPrintln("Box GPS at "+ s_baud);
      delay(100);
      #ifdef SWSERIAL_INCLUDED
        boxgpsSerial.begin(boxgpsBaud);
      #else
        boxgpsSerial.begin(boxgpsBaud, SERIAL_8N1, boxgps_rxPin, boxgps_txPin); //GPS on Serial2
      #endif  
      delay(10);
    #else
      boxgpsSerial.begin(boxgpsBaud);                                   // GPS on default Serial2 (UART3)
    #endif 
  #endif // end of Tracker box  
  
  // ==================    S e t u p   U A R T   S e r i a l   ==============
  #if (MEDIUM_IN == 1)    //UART
    #if (PROTOCOL == 1)   // Mavlink 1
      in_invert = false;
      inBaud = 57600;
      timeEnabled = true;  
    #elif (PROTOCOL == 2) // Mavlink 2
      in_invert = false;
      inBaud = 57600;
      timeEnabled = true;         
    #elif (PROTOCOL == 3) // FrSky S.Port
      in_invert = true;
      inBaud = 57600;
    #elif (PROTOCOL == 4) // FrSky F.Port 1
      in_invert = false;
      inBaud = 115200; 
    #elif (PROTOCOL == 5) // FrSky F.Port 2
      in_invert = false;
      inBaud = 115200;   
    #elif (PROTOCOL == 6) // LTM
      in_invert = false;
      inBaud = 2400; 
    #elif (PROTOCOL == 7)  // MSP
      in_invert = false;
      inBaud = 9600; 
    #elif (PROTOCOL == 8)  // GPS NMEA
      in_invert = false;
      inBaud = 9600; 
      timeEnabled = true;    
    #elif (PROTOCOL == 9)  // CRSF 
      in_invert = false;
      #if (TELEMETRY_SOURCE  == 1)      // BetaFlight
        inBaud = 400000; 
      #elif (TELEMETRY_SOURCE  == 2)    // EdgeTX/OpenTx
        inBaud = 115200; 
      #endif 
    #endif
  #endif  // end of MEDIUM_IN == 1  UART

  // ========================   Optionally  Detect  Baud and Protocol  ==============================
  #if (PROTOCOL == 0) // AUTO
      #if (MEDIUM_IN == 1)  // UART

        // determine polarity of the telemetry - idle high (normal) or idle low (like S.Port)
        pol_t pol = (pol_t)getPolarity(in_rxPin);
        bool ftp = true;
        static int8_t cdown = 30; 
        bool polGood = true;    
        while ( (pol == no_traffic) && (cdown) )
        {
          if (ftp) {
            log.printf("No telem on rx gpio:%d. Retrying ", in_rxPin);
            String s_in_rxPin=String(in_rxPin);   // integer to string
            logScreenPrintln("No telem on rxpin:"+ s_in_rxPin); 
            ftp = false;
          }
          log.print(cdown); log.print(" ");
          pol = (pol_t)getPolarity(in_rxPin);
          delay(500);      
          if (cdown-- == 1 ) {
            log.println();
            log.println("Auto sensing abandoned. Defaulting to IDLE_HIGH 57600 b/s");
            logScreenPrintln("Default to idle_high");
            polGood = false;
          }
        } 
        if (polGood) 
        {    // expect 57600 for Mavlink and FrSky, 2400 for LTM, 9600 for MSP & GPS
          if (pol == idle_low) 
          {
            in_invert = true;
            log.printf("Serial port rx gpio %d is IDLE_LOW, inverting rx polarity\n", in_rxPin);
          } else {
            in_invert = false;
            log.printf("Serial port rx gpio %d is IDLE_HIGH, regular rx polarity retained\n", in_rxPin);     
          }  

          // Determine Baud ===========================================
          inBaud = getBaud(in_rxPin);
          log.print("Serial input baud rate detected is ");  log.print(inBaud); log.println(" b/s"); 
          String s_baud=String(inBaud);   // integer to string. "String" overloaded
          logScreenPrintln("Telem at "+ s_baud);
          
        } else 
        {    // default
            pol = idle_high;
            inBaud = 57600; 
        }
      #endif // end of MEDIUM_IN == 1 UART

      // ======================== Optionally Detect Protocol ==============================
      #if (MEDIUM_IN == 1) || (MEDIUM_IN == 3) || (MEDIUM_IN == 4)// UART or BT or BLE 4.2

        protocol = detectProtocol(inBaud);
        switch(protocol) { 
        case 0:    // No known protocol found
          log.println("No protocol found. Aborting ...."); 
          logScreenPrintln("No protocol!");
          logScreenPrintln("Aborting....");        
          while(1) delay(1000);  // wait here forever  
          break;   
        case 1:    // Mavlink 1
          logScreenPrintln("Mavlink 1 found");
          log.println("Mavlink 1 found"); 
          timeEnabled = true;
          break;
        case 2:    // Mavlink 2
          logScreenPrintln("Mavlink 2 found");
          log.println("Mavlink 2 found");
          timeEnabled = true;
          break;
        case 3:    // FrSky S.Port 
          logScreenPrintln("FrSky S.Port found");
          log.println("FrSky S.Port found");       
          break;
        case 4:    // FrSky F.Port 1
          logScreenPrintln("FrSky F.Portl found");
          log.println("FrSky F.Port1 found");       
          break;
        case 5:    // FrSky F.Port 2
          logScreenPrintln("FrSky F.Port2 found");
          log.println("FrSky F.Port2 found");       
          break; 
        case 6:    // LTM protocol found  
          logScreenPrintln("LTM protocol found"); 
          log.println("LTM protocol found");       
          break;     
        case 7:    // MSP protocol found 
          logScreenPrintln("MSP protocol found"); 
          log.println("MSP protocol found");   
          break; 
        case 8:    // GPS NMEA protocol found 
          logScreenPrintln("NMEA protocol found"); 
          log.println("NMEA protocol found"); 
          timeEnabled = true;   
          break; 
        case 9:    // CRSF protocol found 
          logScreenPrintln("CRSF protocol found"); 
          log.println("CRSF protocol found"); 
          crsf.initialise(inSerial);  // initialise pointer to Stream &port 
          break;                    
        default:   // Unknown protocol 
          log.println("No protocol found, aborting ...."); 
          logScreenPrintln("No protocol!");
          logScreenPrintln("Aborting....");        
          while(1) delay(1000);  // wait here forever
          break;                        
        }
      #else // if not auto detect protocol, use selected protocol
        protocol = PROTOCOL;
      #endif // end of protocol selection 

  #endif // end of AUTO PROTOCOL == 0
//========================  S e t u p   U A R T ==============================
  #if (MEDIUM_IN == 1)
    delay(100);
    #if (defined ESP32) 
        inSerial.begin(inBaud, SERIAL_8N1, in_rxPin, in_txPin, in_invert); 
        log.printf("inSerial baud:%u  rxPin:%d  txPin:%d  invert:%u\n", inBaud, in_rxPin, in_txPin, in_invert);
        logScreenPrint("UART_in ok, baud:");  logScreenPrintln(String(inBaud));     
    #elif (defined ESP8266)
      inSerial.begin(inBaud);
      log.printf("Softwareserial: inSerial baud:%u\n", inBaud);
      logScreenPrint("Serial_in ok, baud:");  logScreenPrintln(String(inBaud));    
    #elif (defined TEENSY3X) 
      inSerial.begin(inBaud); // Teensy 3.x    rx tx pins hard wired
        if (in_invert) {          // For S.Port not F.Port
          UART0_C3 = 0x10;       // Invert Serial1 Tx levels
          UART0_S2 = 0x10;       // Invert Serial1 Rx levels;       
        }
        #if (defined frOneWire )  // default
          UART0_C1 = 0xA0;        // Switch Serial1 to single wire (half-duplex) mode  
        #endif    
    #elif (defined STM32F1xx) 
      inSerial.begin(inBaud);
      log.printf("inSerial baud:%u\n", inBaud);
      //logScreenPrintln("UARTin baud:");  
      //logScreenPrintln(String(inBaud));  
    #endif   
    delay(50); 
    #if (PROTOCOL == 9)  // CRSF 
      crsf.initialise(inSerial);  // initialise pointer to Stream &port    
      log.println("CRSF initialised");
    #endif   
  #endif  
  // ================================  Setup WiFi  ====================================
  #if (defined ESP32)  || (defined ESP8266)
    #if (MEDIUM_IN == 2)  //  WiFi
      setupWiFi();    
    #endif  
  #endif   
 //===========================================================================================
  #if defined STEPPERS
    log.println("Please use adjust button then set midFront position");
    logScreenPrintln("Adjust midfront pos");
  #endif

  log.println("Waiting for input telemetry");
  logScreenPrintln("Waiting for telem"); 
} // end of setup()

//===========================================================================================
//===========================================================================================
#if (defined ESP32) && (MEDIUM_IN == 3)   // Bluetooth Classic
  void checkBT_reconnect()
  {
    if (inSerial.available())
    { 
      btGood = true;
      btGood_millis = millis();
    } else
    {
      if (!btGood)  // check if still connected
      {
        bt_connected = inSerial.connect(BT_Slave_Name); // it tries here for a few seconds while no connection
        while(!bt_connected) //  if not connected, try to reconnect
        {
          static bool ft0 = true;
          if (ft0) 
          {
            ft0 = false;
            log.print("BT disconnected, retrying");
            logScreenPrintChar('BT disco, retry');  
          }
          log.print(".");
          logScreenPrintChar('.');  
          //delay(100); // give OS some time
          bt_connected = inSerial.connect(BT_Slave_Name);       
        } 
      if (bt_connected) 
        {   
          static bool ft1 = true;
          if (ft1)
          {
            ft1 = false;
            log.println("\nBluetooth re-connected!");
            logScreenPrintln("BT reconnected!");
          }
        } 
      }
    }
  }
#endif
//===========================================================================================
void displayHome() 
{ 
  #if defined DEBUG_MINIMUM || defined DEBUG_ALL || defined DEBUG_AZEL
    log.print("Static home location set to Lat = "); log.print(hom.lat,7);
    log.print(" Lon = "); log.print(hom.lon,7);
    log.print(" Alt = "); log.print(hom.alt,0); 
    log.print(" Hdg = "); log.println(hom.hdg,0); 
    logScreenPrintln("Home set success"); 
    logScreenPrintln("Tracking active!");   
 //   displayHeadingSource();
  #endif
}   
//===========================================================================================
void firstStoreHome() 
{
  hom.lat = cur.lat;
  hom.lon = cur.lon;
  hom.alt = cur.alt;

  // firstHomeStored set true in saveHomeToFlash
  saveHomeToFlash();  
  firstHomeStored = true;

  #if defined DEBUG_ALL || defined DEBUG_STATUS
    log.print("Static home location AUTO set to ");       
    log.print("Lat = ");  log.print(hom.lat, 7);
    log.print(" Lon = "); log.print(hom.lon, 7 );        
    log.print(" Alt = "); log.println(hom.alt, 1);                 
 #endif 
} 
//===========================================================================================
void finalStoreHome() 
{
  if (headingsource == 1) // GPS
  {  
      if (firstHomeStored) 
      {            // Use home established when 3D+ lock established, firstHomeStored = 1 
        // Calculate heading as vector from home to where craft is now
        float a, la1, lo1, la2, lo2;
        lo1 = hom.lon; // From firstStoreHome()
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
        log.print("Heading calculated = "); log.println(hom.hdg, 1);
        logScreenPrintln("Heading calculated");      

        hom.lat = cur.lat;
        hom.lon = cur.lon;
        hom.alt = cur.alt;
        hom.hdg = cur.hdg;  // from FC
        finalHomeStored = true;
        displayHome();  
      }  
  } else
  {                
    if (headingsource == 2) // Flight computer compass 
    { 
          hom.lat = cur.lat;
          hom.lon = cur.lon;
          hom.alt = cur.alt;
          hom.hdg = cur.hdg;  // from FC
          finalHomeStored = true;
          displayHome();                  
    } else
    {
      if (headingsource == 3)  // Trackerbox Compass  
      {    
            hom.lat = cur.lat;
            hom.lon = cur.lon;
            hom.alt = cur.alt;
            #if (headingsource  == 3) || (headingsource  == 4)
              hom.hdg = getTrackerboxHeading(); // From own compass 
            #endif          
            finalHomeStored = true;
            displayHome();                  
      } else
      { 
        if (headingsource == 4) // Trackerbox GPS/Compass 
        {    
              log.println("Trackerbox GPS/Compass. Should never get here because home is dynamic!");
              log.println("Aborting. Check logic");       
              logScreenPrintln("Aborting");
              while(1) delay(1000);  // wait here forever    
        } else 
        {      // Unknown protocol 
              log.println("No headingsource!");
              logScreenPrintln("No headingsource!");
              logScreenPrintln("Aborting");
              while(1) delay(1000);  // wait here forever                     
        }
      }  
    }  
  }  
  saveHomeToFlash(); 

}
//===========================================================================================
//===========================================================================================
void loop() 
{     
  //log.print(".");
  #if defined DISPLAY_PRESENT
    upButton.loop();
    dnButton.loop();
  #endif
  setButton.loop();
  #if defined STEPPERS
    adjustButton.loop(); 
  #endif  
  buttonEvents();

  static bool testedMotors = false;
  if ((phase == set_home) && (!testedMotors))
  {
    testedMotors = true;
    #if defined TEST_MOTORS   
      log.println("Testing Servos/Steppers");
      logScreenPrintln("Testing Motors");     
      testMotors();  // Fine tune MaxPWM and MinPWM in config.h to achieve expected movement limits, like 0 and 180
    #endif 
    moveMotors(azMidFront, elStart);   // Move Motors to "midfront/start position
  }
  checkStatusAndTimeouts();          // and service status LED

  #if (MEDiUM_IN == 2)              // WiFi
    serviceWiFiRoutines(); 
  #endif

  #if (MEDIUM_IN == 4)              // BLE4
    if (doConnect == true) 
    { 
      if (connectToServer(*pServerAddress)) // this means if go connect was successfully, NOT if still connected!
      {
        log.printf("fully connected to BLE server %S\n", bleServerName);
        char temp[24];
        sprintf(temp, "BLE cnct to %s", bleServerName);
        logScreenPrintln(temp); 
        BLEconnected = true;
        bleSuGood = true;
      } else 
      {
        log.println("failed to connect to the server. restart to scan for server again.");
      }
      doConnect = false;
    }
  #endif

  #if (defined ESP32) || (defined (ESP8266))     
    if (medium_in == 2)  // WiFi
    {
      wifiGood = false;
      if (wifi_protocol == 1)  // TCP
      {
        wifiGood = (wifiSuGood && outbound_clientGood);
      } else
      if (wifi_protocol == 2)  // UDP
      {
        wifiGood = wifiSuGood ;
      }
    }
  #endif

  if ( (medium_in == 1) || ((medium_in == 2) && wifiGood) || 
     ( (medium_in == 3) && btSuGood)   ||
     ( (medium_in == 4) && bleSuGood)  ||
     ( medium_in == 5)  )
  {
      #if (defined ESP32) && (MEDIUM_IN == 3)  
          checkBT_reconnect();
      #endif
        switch(protocol) { 
        case 1:    // Mavlink 1
          #if (PROTOCOL == 1) || (PROTOCOL == 2)  || (PROTOCOL == 0)  
            Mavlink_Receive();    
          #endif  
          break;
        case 2:    // Mavlink 2
          #if (PROTOCOL == 1) || (PROTOCOL == 2)  || (PROTOCOL == 0)  
            Mavlink_Receive();    
          #endif    
          break;
        case 3 ... 5:    // FrSky S.Port, F.Port 1,  F.Port 2
          #if (PROTOCOL == 3) || (PROTOCOL == 4)  || (PROTOCOL == 5) || (PROTOCOL == 0)
            FrSky_Receive(protocol);  
          #endif     
          break;
        case 6:    // LTM 
          #if (PROTOCOL == 6) || (PROTOCOL == 0)        
            LTM_Receive();    
          #endif
          break;     
        case 7:    // MSP 
          #if (PROTOCOL == 7) || (PROTOCOL == 0)        
            // MSP_Receive(); 
          #endif
          break; 
        case 8:    // GPS NMEA 
          #if (PROTOCOL == 8) || (PROTOCOL == 0)          
            NMEA_GPS_Receive();   
          #endif
          break; 
        case 9:    // CRSF 
          #if (PROTOCOL == 9) || (PROTOCOL == 0)          
            CRSF_Receive(); 
          #endif  
          break;                    
        default:   // Unknown protocol 
          log.printf("Unknown protocol:%X, aborting ....", protocol);        
          logScreenPrintln("Unknown protocol!");
          logScreenPrintln("Aborting....");        
          while(1) delay(1000);  // wait here forever                        
        }

  }  
  //===============================      Service Tracker Box Compass and GPS if they exist
  #if (HEADINGSOURCE == 3) || (HEADINGSOURCE == 4) 
    if ( ((millis() - box_loc_millis) > 500) ) 
    {  // 2 Hz
      box_loc_millis = millis();
      if(boxmagGood) {
        hom.hdg = getTrackerboxHeading();      // heading
        boxhdgGood = true;
      }
      #if (HEADINGSOURCE == 4)
        getTrackerboxLocation();               // lat, lon alt
      #endif  
    }
  #endif
  //===============================       C h e c k   F o r   N e w   A P   C o n n e c t s
  /*
  #if (MEDIUM_IN == 2)    // WiFi
    static uint8_t   AP_prev_sta_count = 0;
    uint8_t AP_sta_count = WiFi.softAPgetStationNum();
    log.printf(" \n", );
    if (AP_sta_count > AP_prev_sta_count) 
    {  // a STA device has connected to the AP
      AP_prev_sta_count = AP_sta_count;
      log.printf("Remote STA %d connected to our AP\n", AP_sta_count);  
      snprintf(snprintf_buf, snp_max, "New STA, total=%d", AP_sta_count);        
      logScreenPrintln(snprintf_buf); 
      #if (WIFI_PROTOCOL == 1)  // TCP
        if (!outbound_clientGood) 
        {// if we don't have an active tcp session, start one
          outbound_clientGood = NewOutboundTCPClient();
        } 
      #endif               
    } else 
    if (AP_sta_count < AP_prev_sta_count) 
    {  // a STA device has disconnected from the AP
      AP_prev_sta_count = AP_sta_count;
      log.println("A STA disconnected from our AP");     // back in listening mode
      logScreenPrintln("A STA disconnected"); 
    }
  #endif
  */
  //====================  Check For Display Button Touch / Press
  #if defined DISPLAY_PRESENT
    handleDisplayButtons();
  #endif   
  //==================== Data Streaming Option
  #if (PROTOCOL == 1) || (PROTOCOL == 2)  || (PROTOCOL == 0)  
    #ifdef DATA_STREAMS_ENABLED 
      if(mavGood) 
      {                      // If we have a link, request data streams from MavLink every 30s
        if(millis()- rds_millis > 30000) 
        {
          rds_millis=millis();
          //log.println("Requesting data streams"); 
          //logScreenPrintln("Reqstg datastreams");    
          RequestDataStreams();   // must have Teensy Tx connected to Taranis/FC rx  (When SRx not enumerated)
        }
      }
  #endif 
#endif  
  //===============================  H A N D L E   H O M E   L O C A T I O N

  //       D Y N A M I C   H O M E   L O C A T I O N
  //log.printf("HEADINGSOURCE:%u  hbG:%u  gpsG:%u  boxgpsG:%u  PacketG:%u  new_GPS_data:%u  new_boxGPS_data:%u \n", 
  //         HEADINGSOURCE, telemGood, gpsGood, boxgpsGood, PacketGood(), new_GPS_data, new_boxGPS_data);          
  #if (HEADINGSOURCE == 4)        // Trackerbox_GPS_And_Compass - possible moving home location
      if (telemGood && gpsGood && boxgpsGood && PacketGood() && new_GPS_data && new_boxGPS_data) 
      {  //  every time there is new GPS data 
        static bool first_dynamic_home = true;
        new_boxGPS_data = false; 
        if (boxmagGood) 
        {
          if (first_dynamic_home) 
          {
            first_dynamic_home = false;
            log.println("Dynamic tracking (moving home) started");  // moving home tracking
            logScreenPrintln("Dynamic tracker ok!");
          }        
          getAzEl(hom, cur);   
          if ( (hc_vector.dist >= minDist) || ((int)cur.alt_ag >= minAltAg) ) pointMotors((uint16_t)hc_vector.az, (uint16_t)hc_vector.el, (uint16_t)hom.hdg);  // Relative to home heading
          new_GPS_data = false;        // cur. location
          new_boxGPS_data = false;     // moving hom. location          
        } else {
          if (first_dynamic_home) {
            first_dynamic_home = false;
            log.println("Dynamic tracking (moving home) failed. No good tracker box compass!");  
            logScreenPrintln("Dynamic trackr bad!");
          }       
        }
      }
  //      S T A T I C   H O M E   L O C A T I O N
  #else   // end of tracker box gps moving home location, start of static home location, HEADINGSOURCE 1, 2 and 3
    if (timeGood) lostPowerCheckAndRestore(epochNow());  // only if active timeEnabled protocol
      
    //log.printf("finalHomeStored:%u  timeEnabled:%u  lostPowerCheckDone:%u  firstHomeStored:%u  homeButtonPushed:%u\n", 
    //        finalHomeStored, timeEnabled, lostPowerCheckDone, firstHomeStored, homeButtonPushed());     
                    
    if ( (!finalHomeStored) && ( ((timeEnabled) && (lostPowerCheckDone)) || (!timeEnabled) ) ) 
    {  // final home not yet stored

      if ((headingsource == 1) && (gpsGood) ) 
      {                                                  // if FC GPS      
        if (!firstHomeStored) 
        {  
          firstStoreHome();          // to get the first location, now go get the second location
          log.println("To get heading, carry craft straight ahead 10m, put down, then return and push tracker home button");  
          logScreenPrintln("Carry craft fwrd ");
          logScreenPrintln("10m. Place and ");
          logScreenPrintln("push home button");
        } else 
        {   // first home stored, now check for buttom push   
          if ((setButton.isPressed()) && (phase == set_home) )
          {
            finalStoreHome(); 
            phase = set_done;
            log.println("Ending set_home phase");
            logScreenPrintln("Set_home phase end");       
          }      
        }  // end of check for button push  

      } else  // end of heading source == FC GPS
      {
        bool sh_armFlag = false;
        #if defined SET_HOME_AT_ARM_TIME  
          sh_armFlag = true;
        #endif
        //log.printf("headingsource:%u  hdgGood:%u sh_armFlag:%u\n", headingsource, hdgGood, sh_armFlag);
        if ( ((headingsource == 2) && (hdgGood)) || ( ((headingsource == 3) || (headingsource == 4)) && (boxhdgGood) ) ) 
        {  // if FC compass or Trackerbox compass 

          //log.printf("sh_armFlag:%u  motArmed:%u  gpsfixGood:%u  ft:%u  hbp:%u\n", sh_armFlag, motArmed, gpsfixGood, ft, homeButtonPushed());              
          if (sh_armFlag) // if set home at arm time
          {                    
            if ( motArmed && gpsfixGood ) 
            { 
              finalStoreHome();         // if motors armed for the first time, and good GPS fix, then mark this spot as home
            } 
          } else                        // if not set home at arm time 
          if (gpsfixGood) 
          {                                  
            static bool ft = true;                         
            if (ft) 
            {
              ft=false;           
              log.printf("GPS lock good! Push set-home button (gpio:%d) anytime to start tracking *************\n", setPin);  
              logScreenPrintln("GPS lock good! Push");
              logScreenPrintln("home button");        
            } else 
            {
              if ((setButton.isPressed()) && (phase == set_home))
              {
                finalStoreHome(); 
                phase == set_done; 
                log.println("Ending set-home phase" );    
                logScreenPrintln("Set_home phase end");                 
              }  
            }
          }      
        } 
      }
    } // final home already stored
    #endif   // end of static home
  
    // Antenna pointing is done from code block below
    //=======================>
    if (finalHomeStored) 
    {
      if (telemGood && gpsGood && PacketGood() && new_GPS_data) 
      {  //  every time there is new GPS data 
        getAzEl(hom, cur);
        if ( (hc_vector.dist >= minDist) || ((int)cur.alt_ag >= minAltAg) ) 
            pointMotors((uint16_t)hc_vector.az, (uint16_t)hc_vector.el, (uint16_t)hom.hdg);  // Motors pointed relative to "home midfront" heading >>>>>>
        new_GPS_data = false;
      }
    }
    if ((lostPowerCheckDone) && (timeGood) && (millis() - millisStore) > 60000) 
    {  // every 60 seconds
      storeEpochPeriodic();
      millisStore = millis();
    }

} // end of main loop
//===========================================================================================
//===========================================================================================
//===========================================================================================
