/*
    Example to demonstrate WiFi UDP input
*/

#include <Arduino.h>
#include <terseCRSF.h> 
// Select RC or telemetry build, telem source-type and any debug macros in terseCRSF.h

// Choose only one input medium 
#define MEDIUM_IN  1    // UART (Serial)       
//#define MEDIUM_IN  2    // WiFi UDP - ESP only
//#define MEDIUM_IN  3    // Bluetooth (Serial) - ESP32 only

#if (MEDIUM_IN  == 1)      // UART select
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

  #define crsf_uart            1              // Serial1
  #if (TELEMETRY_SOURCE  == 1)                // Telemetry from BetaFlight/CF
    #define crsf_baud          420000
  #elif (TELEMETRY_SOURCE  == 2)              // EdgeTX/OpenTx
    #define crsf_baud          115200         // Telemetry from RadioMaster TX16S AUX2
  #endif

  HardwareSerial crsfSerial(crsf_uart);       // instantiate UART object

#endif  // end of UART select

#if (MEDIUM_IN  == 2)      // WiFi UDP select
    //#define WIFI_MODE   1  //AP
    #define WIFI_MODE   2  // STA

    #include <WiFi.h>  // includes UDP class
    #include <WebServer.h>
    #include <WiFiAP.h> 
    
    #define HostName    "crsfUDP"  
    #define APssid      "crsfUDP"
    #define APpw        "password"         // Change me! Must be >= 8 chars  
    #define STAssid     "crsfUDP"
    #define STApw       "password"         // Change me! Must be >= 8 chars   

    IPAddress AP_default_IP(192, 168, 4, 1);
    IPAddress AP_gateway(192, 168, 4, 1);
    IPAddress AP_mask(255, 255, 255, 0);

    IPAddress localIP;                            // tcp and udp
    IPAddress TCP_REMOTEIP(192, 168, 4, 1);      

    uint16_t  UDP_LOCALPORT = 14555;    // readPort 
    uint16_t  UDP_REMOTEPORT = 14550;   // sendPort 
    uint16_t  udp_read_port = 0;
    uint16_t  udp_send_port = 0;

    bool wifiSuGood = false;
    bool wifiApConnected = false;
    bool wifiStaConnected = false;
    bool inbound_clientGood = false;
    bool wifiDisconnected = false;
    bool showRemoteIP = true;
    bool wifi_recv_good = false;
    uint32_t wifi_retry_millis = 0;
    uint32_t wifi_recv_millis = 0;

    //====================       I n s t a n t i a t e   W i F i   O b j e c t s
    
    #define max_clients    6
    uint8_t active_udp_obj_idx = 0;         // for UDP
    uint8_t active_udpremoteip_idx = 0;     // active remote ip 
    uint8_t active_client_obj_idx = 0;

    IPAddress UDP_remoteIP(192, 168, 1, 255);     // default to broadcast unless (not defined UDP_Broadcast)
    IPAddress udpremoteip[max_clients];           // table of remote UDP client IPs
    WiFiUDP *udp_object[2] = {NULL};              // pointers to UDP objects for STA and AP modes
#endif // end of WiFi UDP select

#if (MEDIUM_IN  == 3)      // BT Classic select
  #include "BluetoothSerial.h"
  #define BT_MODE  1           // Master Mode - active, initiate connection with slave (name)
  //#define BT_MODE  2           // Slave Mode - passive, advertise our hostname & wait for master to connect to us
  const char* BT_Slave_Name   =   "btslavename"; 
  BluetoothSerial inSerial; 
  bool  btSuGood = false;
  bool      bt_connected = false;  
  bool      btGood = false; 
  bool      btPrev = false;  
#endif

#define log   Serial

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

#if (MEDIUM_IN  == 2)      // WiFi UDP select
  void printRemoteIP()
  {
    if (showRemoteIP)
    {
      showRemoteIP = false;
      log.print("UDP client identified, remote IP: ");
      log.print(UDP_remoteIP);
      log.print(", remote port: ");
      log.println(UDP_REMOTEPORT);
    }
  }
  void serviceWiFiRoutines()
  {
    // Report stations connected to/from our AP
    uint8_t AP_sta_count = WiFi.softAPgetStationNum();
    static uint8_t AP_prev_sta_count = 0;
    wifiApConnected = (AP_sta_count > 0);
    if (AP_sta_count > AP_prev_sta_count)
    {
      AP_prev_sta_count = AP_sta_count;
      log.printf("Remote STA %d connected to our AP\n", AP_sta_count);
    }
    else if (AP_sta_count < AP_prev_sta_count)
    { // a device has disconnected from the AP
      AP_prev_sta_count = AP_sta_count;
      log.println("A STA disconnected from our AP"); // back in listening mode
    }
  }
  uint16_t read_UDP()
  {
    if (!wifiSuGood)
      return 0;
    if ((active_udp_obj_idx == 0) && (!(wifiStaConnected)))
      return 0;
    if ((active_udp_obj_idx == 1) && (!(wifiApConnected)))
      return 0;    
    // 2 possible udp objects, STA [0]    and    AP [1]
    #if (WIFI_MODE == 2)  // STA
      active_udp_obj_idx = 0;             // Use STA UDP object for FC read
      udp_read_port = UDP_REMOTEPORT; // used by printRemoteIP() only. read port set by ->begin(udp_read_port)).
      udp_send_port = UDP_LOCALPORT;
      // log.printf("readFC() read port:%u    send port:%u\n", udp_read_port, udp_send_port);
    #endif
    #if (WIFI_MODE == 1)  // AP
        active_udp_obj_idx = 1; // Use AP UDP object for FC read
        udp_read_port = UDP_LOCALPORT;
        udp_send_port = UDP_REMOTEPORT;
    #endif
    int16_t packetSize = udp_object[active_udp_obj_idx]->parsePacket();
    // esp sometimes reboots here: WiFiUDP.cpp line 213 char * buf = new char[1460];
    // log.printf("Read UDP object=%d port:%d  len:%d\n", active_udp_obj_idx, udp_read_port, len);
    if (packetSize)
      {
        uint16_t len = udp_object[active_udp_obj_idx]->read(&*crsf.crsf_buf, 64);
        log.printf("len:%2u:", len);
        if (len >= 0)
        {
          wifi_recv_millis = millis();
          wifi_recv_good = true;
          //printBuffer(buff);
          #if (not defined UDP_Broadcast)
            UDP_remoteIP = udp_object[active_udp_obj_idx]->remoteIP();
            bool in_table = false;
              for (int i = 1; i < max_clients; i++)
              {
                if (udpremoteip[i] == UDP_remoteIP)
                { // IP already in the table
                  //    log.printf("%s already in table\n", UDP_remoteIP.toString().c_str() );
                  in_table = true;
                  break;
                }
              }
              if (!in_table)
              { // if not in table, add it into empty slot, but not [0] reserved for otgoing (FC side)
                for (int i = 1; i < max_clients; i++)
                {
                  if ((udpremoteip[i][0] == 0) || (udpremoteip[i][3] == 255))
                  {                                // overwrite empty or broadcast ips
                    udpremoteip[i] = UDP_remoteIP; // remember unique IP of remote udp client so we can target it
                    log.printf("%s client inserted in UDP client table\n", UDP_remoteIP.toString().c_str());
                    showRemoteIP = true;
                    break;
                  }
                }
              }
          #endif
          printRemoteIP();
          return len;
        }  
      }
      return 0;
  }
  void startAccessPoint()
  {
    log.printf("WiFi mode set to WIFI_AP %s\n", WiFi.mode(WIFI_AP) ? "" : "Failed!");
    // WiFi.softAP(const char* ssid, const char* password, int channel, int ssid_hidden, int max_connection)
    WiFi.softAP(APssid, APpw);
    delay(100);
    log.print("AP_default_IP:");
    log.print(AP_default_IP); // these print statement give the module time to complete the above setting
    log.print("  AP_gateway:");
    log.print(AP_gateway);
    log.print("  AP_mask:");
    log.println(AP_mask);

    WiFi.softAPConfig(AP_default_IP, AP_gateway, AP_mask);

    localIP = WiFi.softAPIP();
    log.print("AP IP address: ");
    log.print(localIP);
    log.printf(" SSID:%s\n", APssid);

    // regular AP
    udp_read_port = UDP_LOCALPORT;
    udp_send_port = UDP_REMOTEPORT;

    // Start UDP Object
    WiFiUDP UDP_Object;
    udp_object[1] = new WiFiUDP(UDP_Object);
    log.printf("Begin UDP using AP UDP object  read port:%d  send port:%d\n", udp_read_port, udp_send_port);

    udp_object[1]->begin(udp_read_port); // there are 2 possible udp objects, STA [0]    and    AP [1]
    UDP_remoteIP = WiFi.softAPIP();
    UDP_remoteIP[3] = 255; // broadcast until we know which ip to target

    // Now initialise the first entry of the udp targeted ip table
    udpremoteip[0] = UDP_remoteIP;
    udpremoteip[1] = UDP_remoteIP;

    log.printf("UDP for AP started, local %s   remote %s\n", WiFi.softAPIP().toString().c_str(),
                UDP_remoteIP.toString().c_str());

    wifiSuGood = true;
  }

  bool startStation()
  {
    uint8_t retry = 0;
    WiFi.disconnect(true); // To circumvent "wifi: Set status to INIT" error bug
    delay(500);
    if (WiFi.mode(WIFI_STA))
    {
      log.println("WiFi mode set to STA sucessfully");
    } else
    {
      log.println("WiFi mode set to STA failed!");
    }
    log.print("Trying to connect to ");
    log.print(STAssid);
    delay(500);

    WiFi.begin(STAssid, STApw);
    while (WiFi.status() != WL_CONNECTED)
    {
      retry++;
      static uint32_t max_retry = 30;
      if (retry >= max_retry)
      {
        log.println();
        log.println("Failed to connect in STA mode");
        return false;
      }
      delay(1000);
      log.print(".");
    }   
    if (WiFi.status() == WL_CONNECTED)
    {
      log.println();
      log.println("WiFi connected!");
      wifiStaConnected = true;
      localIP = WiFi.localIP();

      UDP_remoteIP = localIP; // Initially broadcast on the subnet we are attached to. patch by Stefan Arbes.
      UDP_remoteIP[3] = 255;  // patch by Stefan Arbes

      log.print("Local IP address: ");
      log.print(localIP);
      log.println();

      int16_t wifi_rssi = WiFi.RSSI();
      log.print("WiFi RSSI:");
      log.print(wifi_rssi);
      log.println(" dBm");

      udp_read_port = UDP_REMOTEPORT;
      udp_send_port = UDP_LOCALPORT; // so we swap read and send ports, local (read) becomes 14550
      WiFiUDP UDP_STA_Object;
      udp_object[0] = new WiFiUDP(UDP_STA_Object);
      log.printf("Begin UDP using STA UDP object  read port:%d  send port:%d\n", udp_read_port, udp_send_port);
      udp_object[0]->begin(udp_read_port); // there are 2 possible udp objects, STA [0]    and    AP [1]
      UDP_remoteIP = localIP;
      UDP_remoteIP[3] = 255; // broadcast until we know which ip to target
      udpremoteip[1] = UDP_remoteIP; // [1] IPs reserved for GCS side
      log.printf("UDP for STA started, local %s   remote %s\n", localIP.toString().c_str(),
                UDP_remoteIP.toString().c_str());

      wifiSuGood = true;
    }  
    return true;
  }
#endif  // WiFi UDP select

void setup() {
  log.begin(115200);
  delay(2000);
#if (MEDIUM_IN  == 1)      // UART select
  crsfSerial.begin(crsf_baud, SERIAL_8N1, crsf_rxPin, crsf_txPin, crsf_invert);
  log.printf("CRFS uart:%u  baud:%u  rxPin:%u  txPin:%u  invert:%u\n", crsf_uart, crsf_baud, crsf_rxPin, crsf_txPin, crsf_invert);
  crsf.initialise(crsfSerial);
#endif  

#if (MEDIUM_IN  == 2)      // WiFi UDP
  #if (WIFI_MODE == 1)  // AP
    startAccessPoint();
    log.println("Waiting for UDP clients to connect ...");
  #endif  
  #if (WIFI_MODE == 2)  // STA
    startStation();
  #endif   
#endif

#if (MEDIUM_IN  == 3)      // BT Classic select
  #if (BT_MODE == 1)       // 1 master mode, connect to slave name
    log.printf("Bluetooth master mode, looking for slave name \"%s\"\n", BT_Slave_Name);               
    inSerial.begin(BT_Slave_Name, true);            
  #else                  // 2 slave mode, advertise slave name
      log.printf("Bluetooth slave mode advertising slave name \"%s\"\n", BT_Slave_Name);              
      inSerialBT.begin(BT_Slave_Name);   
  #endif 
  bt_connected = inSerial.connect(BT_Slave_Name);
  while(!bt_connected) {
    log.print(".");
    delay(1000);
    bt_connected = inSerial.connect(BT_Slave_Name);       
  }  
  if(bt_connected) {
    btSuGood = true;       
    log.println("Bluetooth connected!");
  } else {
    log.println("Bluetooth NOT connected!");
  } 
  crsf.initialise(inSerial);  // initialise pointer to Stream &port
#endif // end of Classic BT

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

#if (MEDIUM_IN  == 1) || (MEDIUM_IN  == 3)     // UART or BT select
  crsf.printLinkStats();    // optional

  if (crsf.readCrsfFrame(crsf.frame_lth))  // exposes discovered frame_lth if needed
  {
    uint8_t len = crsf.frame_lth;
#endif

#if (MEDIUM_IN  == 2) // UDP select
  serviceWiFiRoutines();
  uint8_t len = read_UDP();
  if (len) 
  {
#endif

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
      log.printf("lth:%u %s\n", crsf.flight_mode_lth, crsf.flightModec.c_str());
#endif
    }
#endif // end of Telemetry
  }
}
