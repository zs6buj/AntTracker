    String    pgm_path;
    String    pgm_name;

    bool pb_rx = true;
      
    typedef enum frport_type_set { f_none = 0, f_port1 = 1, f_port2 = 2, s_port = 3, f_auto = 4} frport_t;  
    typedef enum polarity_set { idle_low = 0, idle_high = 1, no_traffic = 2 } pol_t;
    typedef enum phase_set {set_midfront = 0, set_home = 1, set_done = 3} phase_t;
    phase_t phase;

    enum BOX_COMPASS_ALIGN 
    {
      ALIGN_DEFAULT = 0,
      CW0_DEG = 1,
      CW90_DEG = 2,
      CW180_DEG = 3,
      CW270_DEG = 4,
      CW0_DEG_FLIP = 5,
      CW90_DEG_FLIP = 6,
      CW180_DEG_FLIP = 7,
      CW270_DEG_FLIP = 8
    };
    
    uint32_t inBaud = 0;       // Includes flight GPS
    uint32_t boxgpsBaud = 0;   // Tracker attached GPS, not flight GPS

    uint8_t protocol = 0;
    uint8_t medium_in = 0;
    uint8_t wifi_protocol = 0;
    
    const uint8_t snp_max = 128;
    char          snprintf_buf[snp_max];       // for use with snprintf() formatting of display line

    // ************************************

    uint32_t val = 0;
    uint16_t addr = 0;
    uint8_t  ledState = LOW; 
    const uint8_t timeout_secs = 12;   

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

    bool      telemGood = false;   
    bool      telemPrev = false;    
    bool      hbGood = false; 
    bool      mavGood = false;   
    bool      timeGood = false;
    bool      frGood = false;
    bool      frPrev = false; 
    bool      motPrev = false;     
    bool      pwmGood = false; 
    bool      pwmPrev = false;
    bool      gpsGood = false; 
    bool      gpsPrev = false;
    bool      bt_connected = false;  
    bool      btGood = false; 
    bool      btPrev = false;  
    bool      hdgGood = false;       
    bool      serGood = false; 
    bool      lonGood = false;
    bool      latGood = false;
    bool      altGood = false;               
    bool      boxgpsGood = false; 
    bool      boxgpsPrev = false; 
    bool      boxmagGood = false;
    bool      boxhdgGood = false; 
    bool      motArmed = false;   // motors armed flag
    bool      gpsfixGood = false;

    uint32_t  telem_millis = 0;         
    uint32_t  frGood_millis = 0;
    uint32_t  hbGood_millis = 0;   
    uint32_t  pwmGood_millis = 0;       
    uint32_t  gpsGood_millis = 0;
    uint32_t  btGood_millis = 0;
    uint32_t  boxgpsGood_millis = 0;
    uint32_t  goodFrames = 0;
    uint32_t  badFrames = 0;
    uint32_t  wifi_retry_millis = 0;
    uint32_t  wifi_recv_millis = 0;
    //====================
    bool  wifiSuDone = false;
    bool  wifiSuGood = false;
    bool  wifiGood = false;
    bool  wifi_recv_good = false;
    bool  btSuGood = false;
    bool  bleSuGood = false;
    bool  wifiApConnected = false;
    bool  wifiStaConnected = false;
    bool  outbound_clientGood = false;
    bool  rxFT = true;
    bool  gotRecord = false; 
    bool  firstHomeStored = false;    
    bool  finalHomeStored = false;
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
    
    uint8_t minDist = 4;  // dist from home where tracking starts OR
    uint8_t minAltAg = 4; // alt ag where tracking starts
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

  //  HUD variables
  uint8_t  hud_num_sats = 0; 
  uint32_t hud_rssi = 0; 
  float    hud_pitch = 0;
  float    hud_roll = 0; 
  float    hud_yaw = 0;
  float    hud_grd_spd = 0;
  float    hud_climb = 0;          // m/s
  int16_t  hud_bat1_volts = 0;     // dV (V * 10)
  int16_t  hud_bat1_amps = 0;      // dA (A * 10)
  uint16_t hud_bat1_mAh = 0;
  int16_t  hud_bat2_volts = 0;     // dV (V * 10)
  int16_t  hud_bat2_amps = 0;      // dA (A * 10)
  uint16_t hud_bat2_mAh = 0;

//        E E P R O M   
  #define EEPROM_SIZE 32    // 0 thru 30 used (31B) 

  // Create MOTOR TYPE objects 
  #if defined SERVOS
    #if defined TEENSY3X            // Teensy 3.2
      PWMServo azServo;             // Azimuth
      PWMServo elServo;             // Elevation   
    #elif defined STM32F1xx
      Servo azServo;               
      Servo elServo;                  
    #elif (defined ESP32) || (defined ESP8266)   
      MoToServo azServo;            
      MoToServo elServo;            
    #endif
  #endif
  #if defined STEPPERS
    MoToStepper azStepper(aStepRev,  STEPDIR); 
    MoToStepper elStepper(aStepRev,  STEPDIR);  
  #endif