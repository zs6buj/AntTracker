//version 0.0.5
#include <terseCRSF.h>  
// Select RC or telemetry, telem source-type and any debug macros in terseCRSF.h

#if defined RC_BUILD
    #define crsf_rxPin      A3      // Signal tx pin, transmitter, in back bay
    #define crsf_txPin      A2      // 
    #define crsf_invert     true
#else
    #define crsf_invert     false
    #define crsf_rxPin      A3      
    #define crsf_txPin      A2       
#endif

#define log   Serial

//#define crsf_uart            1              // Serial1
#if (TELEMETRY_SOURCE  == 1)                // Telemetry from BetaFlight/CF
  #define crsf_baud          420000
#elif (TELEMETRY_SOURCE  == 2)              // EdgeTX/OpenTx
  #define crsf_baud          115200         // Telemetry from RadioMaster TX16S AUX2
#endif
      HardwareSerial crsfSerial(USART2); //Serial 1rx2=PA3  tx2=PA2  - for telemetry in
//HardwareSerial crsfSerial(crsf_uart);       // instantiate Serial object

CRSF crsf;            // instantiate CRSF object

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
  log.println("Starting...");
  crsfSerial.begin(crsf_baud);
  log.printf("baud:%u\n", crsf_baud);
  crsf.initialise(crsfSerial);  
}

void loop() 
{
  if (crsf.readCrsfFrame(crsf.frame_lth))  // exposes frame_lth
  {
#if defined SHOW_LOOP_PERIOD
  printLoop1(true);
#endif

#if defined RC_BUILD
      crsf.decodeRC();   // remember to lose the prefix sync byte
#if defined DEMO_PWM_VALUES
      crsf.printPWM(&*crsf.pwm_val, crsf.max_ch);
#endif
#if defined DEMO_SBUS
  log.print("SBUS:");
  crsf.printBytes(&*crsf.sb_bytes, 25);
#endif 

#else   // TELEMETRY BUILD
    uint16_t len = crsf.frame_lth;
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
  
    if (crsf_id == BATTERY_ID) 
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

    if (crsf_id == ATTITUDE_ID)
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
