         

//================================================================================================= 
//================================  C h a n g e   L o g  ========================================== 
//================================================================================================= 
/*

Change log:
                                    
Legacy
v0.37 2019-02-12 Reintroduce PacketGood() location reasonability test to mitigate telemetry errors.
v0.38 OLED support added

v2.00 2019-03-03 Unified version auto detect telemetry speed and protocol. Mavlink 1 & 2, FrSky (D, S.Port and Passthru), LTM
      MSP and GPS support outstanding
v2.01 Support for NMEA GPS added     
v2.05 2019/03/17 UNTESTED BETA Support for retoring home settings from EEPROM after power interruption  
v2.06 2019/06/25 Improved auto baud rate detection. Define in_rxPin correctly for Maple Mini
v2.07 2019/08/10 Tidy up mavlink library paths
v2.08 2019/08/13 Fix mavlink ap_fixtype >2 before using lat and lon. Scroll entire Oled screen.
v2.09 2019/08/16 Supports ESP32 Dev Board - BETA
v2.10 2019/08/23 Improve response in baud detect when no telemetry present.
v2.11 2019/09/06 Mavlink WiFi and Bluetooth input added.  BT not tested.
v2.12 2019/09/18 Store most recent mavlink GPS millis.  
v2.13 2019-11-18 Update WiFi, BT (master/slave), board variants setup, autobaud, OledPrint to Mav2Pt v2.46  
v2.14 2019-12-11 When Heading _Source == 3 (compass on tracker), don't need to check ap_hdg in PacketGood()
      2020-01-02 Sloppy exits removed. :)          
v2.15 2020-10-12 Proper TCP client added for outgoing (to telemetry source) connect   
                 Display scrolling added
                 TCP in tests good.     
v2.15.3 2020-11-08 Fix 10 to power calc for FrSky X.  
v2.15.4 2020-11-12 Patch by mric3412 (pointServos bug in some home heading cases) properly included. 
v2.15.5 2021-02-01 Add hardware signal inversion for S.Port input. Notified by @mello73.   
v2.16.0   2021-02-22 Adopt GitHub Tags
          2021-02-27 Add HUD display. Add FrSky UDP telemetry in
V2.16.1   2021-03-03 Add support for F.Port, auto inversion 
V2.16.2   2021-03-04 Debug FrSky frame decode (fix offset)
                     Fix HUD RSSI blank    
V2.16.3   2021-03-04 Always auto detect serial speed. Always sense polarity, autobaud and detect protocol
          2021-03-09 Include sport, fport1 and fport2. 
          2021-03-09 Tidy up.           
V2.17.0   2021-03-16 Add "GPS on the Tracker" option, aka movable tracker.  
          2021-03-17 Use ap33_alt_ag instead of ap33_amsl. Still calcs relative alt above field.   
          2021-03-24 Beta. Support dynamic (moving) tracker with mag and GPS on the box. 
                     Bug fixes.  
V2.17.1   2021-03-29 Rationalise patches and simplify servo code  
V2.17.2   2021-04-02 Clean build of STM32F103, Maple Mini and Teensy 3.x code.   
V2.17.3   2021-04-07 Clean compile and test - WiFi UDP in    
          2021-04-09 ESP Servo lib, degrees not PWM like STM32
V2.17.4   2021-04-16 Clean compile Mavlink BT input option    
V2.17.5   2021-04-17 Alpha code - not tested:  Add FrSky BT input option       
V2.17.6   2021-04-19 For Frsky input only, hbGood = gpsGood    
V2.17.7   2021-04-19 pan-sattan : #REVERSEELEVATION typo 
                     Complete coding for HMC5883L magnetometer   
 
V2.17.8   2021-05-10 Declare WiFi.onEvent() only when WiFi input option selected 
v2.17.9   2021-06-13 Fix syntax erros with some configurations 
v2.17.10  2021-06-20 Pre-select protocol option     
v2.18.00 2021-06-24 S.Port input tested good
                    Upgrade display code
                    Add Mavlink #define DATA_STREAMS_ENABLED to data streams from FC
                    For FrSky input, renew gpsGood_millis
                    Add Adafruit_BusIO library
v2.18.01 2021-06-25 Fix S.Port altitude. Use 0x5004.
v2.18.02 2021-07-27 Tidy up FrSky UDP telemetry input.
v2.18.03 2021-08-02 BT option syntax
v2.18.04 2021-08-05 Set up with FrSky BT options
v2.18.05 2021-08-09 Rewrite old 0x410 decode 
v2.18.06 2021-08-11 Revert to v2.18.04. Rename 32b pt_gps_status pt410_gps_status.
v2.18.07 2021-08-20 Add debug for FrSky GPS & Mag Status
v2.18.08 2021-09-06 Add debug for FrSky D-Style Flight Mode 0x400
         2021-09-07 Invert ESP32 home button sense and change pin number  
v2.18.09 2021-09-08 Add Set_Home_On_Arm option for testing  
         2021-09-09 Tidy up Mavlink BT in.
         2021-09-10 Wait forever for BT connection.  
                    Fix FrSky 0x400 motArmed.  
v2.19.00 2021-11-09 Tinfo and Pinfo pins deprecated, removed code 
v2.19.01 2021-12-08 Resurrect STM32f103C version. Replace printf with snprintf. I2C pins in Wire.h     
v2.19.2  2021-12-10 Minor STM32 fixes. Activate some debugging.  
v2.19.3  2022-01-03 Changes to support the official STM32 OEM core - RECOMMENDED !
                    OEM servo lib uses degrees
                    OEM EEPROM lib same as ESP32
                    Mavlink send gets it's own msg buffer (stops receive msg buffer tainting, repeating Motos Armed/Disarmed)
v2.19.4  2022-01-07 Add servo slowdown factor
                    Improve servo testing 
                    Fix set home button press on STM32F1xx                                         
v2.19.5  2022-01-09 Define hud offset   
v2.19.6  2022-01-19 Add HUD support for iNav, part 1                                             
v2.19.7  2022-01-20 Add iNav speed, pitch and roll for HUD 
v2.19.7             Add climb  
v2.19.8  2022-03-09 Merge Bohan's code for box compass alignment.
                    Arrange Library folders  
v2.19.9  2022-09-01 Add option macro for NoGenericSerial  
         2023-04-03 Config.h and binary for tibean    
v2.19.10 2023-04-06 Refined BOX_COMPASS_ALIGN routine     
                    Fixed "E (2613) gpio" 
v2.20.0  2023-10-01 Add CRSF/ELRS. Refresh code layout. https://github.com/zs6buj/terseCRSF
v2.20.1  2023-11-10 Add I2C bus scan for diagnostics  
                    Switch to QMC5883LCompass.h Library  
                    Fix boardled error msg 
v2.20.2  2023-11-19 New crsf library, with CRC check added
v2.20.3  2023-11-29 Upgrade to terseCRSF v0.0.3. 
v2.20.4  2024-02-07 Patches as per casfra96 for LTM.
v2.20.5  2024-03-12 Beta - Generic BT input for CRSF (and others in future)   
v2.20.6  2024-03-22 PR oldrootbeer - Fix error in FLIPPED box compass allignment  

v2.21.00 2024-05-08 Support selection of "medium", being UART or WiFi or BT indpendent of
                    protocol. Some restructuring. Beta, needs testing. 
                    CRSF + (Serial or BT) tested good.
v2.21.01 2024-05-22 CRSF - support EdgeTX/OpenTX - frame sync byte and baud.
                    Add auto-detect CRSF protocol
v2.21.02 2024-05-23 Add BT auto reconnect. Add motArmed detect for crsf (needs checking).
v2.21.04 2024-06-03 Update some libraries
v2.21.05 2024-06-05 Revert to ESP32 core 2.0.17 (3.0.0 breaks TFT_eSPI lib)
                    Also, UART code fix.
v2.21.06 2024-06-06 Add general WiFi (UDP) for all protocols except crsf(for now)     
v2.21.07 2024-07-01 Add and test UDP for CRSF. 
v2.21.08 2024-07-14 Start adding BLR support for FrSky and CRSF   
         2024-07-15 add missing logic, if (connectToServer(*pServerAddress)) 
v2.21.08 2024-07-17 further debugging and testing FrSky BLE   
v2.21.10 2024-07-18 restore line overwritten in  config        
V2.21.10b 2024-07-21 Add some debugging code     
        c            debug CRC
        d 2024-07-23 Temporarily disable CRC    
        e 2024-07-25 Utilities line 1935 memcpy(msgBuf, pData, length);   
v2.21.11  2024-07-28 Unstuff frsky byte stuff     
V2.21.11b 2024-07-29 Reduce debug verbosity   
v2.21.11c 2024-07-30 Remove notify ble propery, not needed.
                     Further reduce debug verbosity
                     Add display support for BLE
v2.21.11d 2024-07-31 Disable the action of BLEAdvertisedDeviceCallbacks() after it finds our device.  
v2.21.11e 2024-08-02 maxbuf = 64; (was 24)                   
// IMPORTANT - versions below are all built with esp32 core 3.0.3 or later
v2.21.12  2024-08-07 Replace servo library with MobaTools servo library. 
                     Support servo speed control and asynchronous working.
v2.21.13             Add support stepper motors. Switch to ezButons library 
v2.22.00  2024-08-20 Clean up udp objects. Testing   
v2.22.01  2024-08-22              
*/               