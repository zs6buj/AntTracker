#include <Arduino.h>
#include <terseCRSF.h> 
// Select RC or telemetry, telem source-type and any debug macros in terseCRSF.h

#if defined RC_BUILD     // Radio Control Build
    #define crsf_rxPin      13      // Signal pin transmitter in bay tx
    #define crsf_txPin      14      // GREEN tx to FC LIGHT BLUE 
    #define crsf_invert     true
  #if defined SUPPORT_SBUS_OUT    
    #define sbus_uart       2      // Serial2
    HardwareSerial sbusSerial(sbus_uart);       // instantiate Serial object
    #define sbus_rxPin      -1      // RX1 SBUS not used - don't care 
    #define sbus_txPin      15      // TX1 SBUS out 
    #define sbus_invert     true  
    sbmode_t sbus_mode = sbm_normal; //normal - baud - 100000b/s, fast - baud = 200000b/s
  #endif     
#else                   // Telemetry Build
    #define crsf_invert     false
    #define crsf_rxPin      27      //16 YELLOW rx from GREEN FC tx
    #define crsf_txPin      17      // GREEN tx to YELLOW FC rx    
#endif

#define log   Serial

#define crsf_uart            1              // Serial1
#if (TELEMETRY_SOURCE  == 1)                // Telemetry from BetaFlight/CF
  #define crsf_baud          420000
#elif (TELEMETRY_SOURCE  == 2)              // EdgeTX/OpenTx
  #define crsf_baud          115200         // Telemetry from RadioMaster TX16S AUX2
#endif

HardwareSerial crsfSerial(crsf_uart);       // instantiate Serial object

CRSF crsf;                                  // instantiate CRSF object

void printLoop1(bool newline)
{
  static uint32_t prev_lp1_millis = 0;
  uint32_t now_millis = millis();
  uint32_t period = now_millis - prev_lp1_millis;
  log.printf("Loop1 %3dmS ", period);
  if (newline)
    log.println();
  prev_lp1_millis = now_millis;
}

void setup() {
  log.begin(115200);
  delay(2000);

  crsfSerial.begin(crsf_baud, SERIAL_8N1, crsf_rxPin, crsf_txPin, crsf_invert);
  log.printf("CRFS uart:%u  baud:%u  rxPin:%u  txPin:%u  invert:%u\n", crsf_uart, crsf_baud, crsf_rxPin, crsf_txPin, crsf_invert);
  crsf.initialise(crsfSerial);

#if defined SUPPORT_SBUS_OUT
    uint32_t sbus_baud = 0;
    if (sbus_mode == sbm_fast)
    {
      sbus_baud = 200000;
      log.println("SBUS fast 200000b/s");
    }
    else
    {
      sbus_baud = 100000;  
      log.println("SBUS normal 100000 b/s"); 
    }  
    delay(100);
    sbusSerial.begin(sbus_baud, SERIAL_8E2, sbus_rxPin, sbus_txPin, sbus_invert); 
    log.printf("SBUS uart:%u  baud:%u sending on txPin:%u  invert:%u\n", sbus_uart, sbus_baud, sbus_txPin, sbus_invert);
    delay(100);
    crsf.sbus_initialise(sbusSerial);
#endif
}

void loop() 
{
  crsf.printLinkStats();    // optional

  if (crsf.readCrsfFrame(crsf.frame_lth))  // exposes discovered frame_lth if needed
  {
#if defined SHOW_LOOP_PERIOD
  printLoop1(true);
#endif

#if defined RC_BUILD
  crsf.decodeRC(); // remember to lose the prefix sync byte
#if defined DEMO_PWM_VALUES
      crsf.printPWM(&*crsf.pwm_val, crsf.max_ch);
#endif
#if defined DEMO_SBUS
  log.print("SBUS:");
  crsf.printBytes(&*crsf.sb_bytes, 25);
#endif 

#else   // TELEMETRY BUILD
    int16_t len = 0;
    uint8_t crsf_id = crsf.decodeTelemetry(&*crsf.crsf_buf, len);

    if (crsf_id == GPS_ID) 
    {
      #if defined DEMO_CRSF_GPS    
      log.print("GPS id:");
      crsf.printByte(crsf_id, ' ');
      log.printf("lat:%2.7f  lon:%2.7f", crsf.gpsF_lat, crsf.gpsF_lon);
      log.printf("  ground_spd:%.1fkm/hr", crsf.gpsF_groundspeed);
      log.printf("  hdg:%.2fdeg", crsf.gpsF_heading);
      log.printf("  alt:%dm", crsf.gps_altitude);
      log.printf("  sats:%d\n", crsf.gps_sats); 
 #endif     
    }

    if (crsf_id == BATTERY_ID) // 0x08
    { 
#if defined DEMO_CRSF_BATTERY         
      log.print("BATTERY id:");
      crsf.printByte(crsf_id, ' ');
      log.printf("volts:%2.1f", crsf.batF_voltage);
      log.printf("  amps:%3.1f", crsf.batF_current);
      log.printf("  Ah_drawn:%3.1f", crsf.batF_fuel_drawn);
      log.printf("  remaining:%3u%%\n", crsf.bat_remaining);
#endif 
    }
   
    if (crsf_id == LINK_ID) // 0x14 Link statistics
    {
#if defined DEMO_CRSF_LINK 
      log.print("LINK id:");     
      crsf.printByte(crsf_id, ' ');
      log.printf("  up_rssi_ant_1:%ddB", crsf.link_up_rssi_ant_1 * -1);  
      log.printf("  up_rssi_ant_2:%ddB", crsf.link_up_rssi_ant_2 * -1);  
      log.printf("  up_quality:%d%%", crsf.link_up_quality);  
      log.printf("  up_snr:%ddB", crsf.link_up_snr);
      log.printf("  diversity_active_ant:%d", crsf.link_diversity_active_ant);  
      log.printf("  rf_mode:%d", crsf.link_rf_mode);  
      log.printf("  up_tx_power:%d", crsf.link_up_tx_power);  
      log.printf("  dn_rssi:%ddB", crsf.link_dn_rssi * -1);  
      log.printf("  dn_quality:%d%%", crsf.link_dn_quality);  
      log.printf("  up_rssi_ant_1:%d", crsf.link_up_rssi_ant_1);  
      log.printf("  link_dn_snr:%ddB\n", crsf.link_dn_snr);        
#endif      
    }
    if (crsf_id == ATTITUDE_ID) // 0x1E
    {
#if defined DEMO_CRSF_ATTITUDE 
      log.print("ATTITUDE id:");
      crsf.printByte(crsf_id, ' '); 
      log.printf("pitch:%3.1fdeg", crsf.attiF_pitch);
      log.printf("  roll:%3.1fdeg", crsf.attiF_roll);
      log.printf("  yaw:%3.1fdeg\n", crsf.attiF_yaw);  
#endif          
    }    

    if (crsf_id == FLIGHT_MODE_ID)
    {
#if defined DEMO_CRSF_FLIGHT_MODE 
      log.print("FLIGHT_MODE id:");
      crsf.printByte(crsf_id, ' ');
      log.printf("lth:%u %s\n", crsf.flight_mode_lth, crsf.flightMode.c_str());
#endif
    }
#endif // end of Telemetry
  }
}
