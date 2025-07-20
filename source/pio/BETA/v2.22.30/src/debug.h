  //================================================================================================= 
  //============================= D E B U G G I N G   O P T I O N S   ===============================
  //================================================================================================= 

#define DEBUG_MINIMUM    //  Leave this as is unless you need the serial port for something else
#define DEBUG_STATUS

//#define DEBUG_ALL
//#define DEBUG_PROTOCOL
//#define DEBUG_BAUD
//#define DEBUG_LEDS

//#define DEBUG_BOXCOMPASS   
#define ACTIVATE_PROFILING  // loop timing profiling
#define DEBUG_MAV_INPUT
//#define DEBUG_MAV_INPUT_BYTE
//#define DEBUG_MAV_BUFFER  
#define DEBUG_AZEL
#define DEBUG_MOTORS 
//#define DEBUG_MAV_HEARTBEAT 
#define DEBUG_MAV_GPS  
//#define DEBUG_HOME
//#define DEBUG_HEADINGSOURCE
//#define DEBUG_MIN_DIST
//#define DEBUG_LTM
//#define DEBUG_MSP
//#define DEBUG_inGPS               // a GPS on the 'plane

//#define DEBUG_EEPROM
//#define DEBUG_TIME 


//#define DEBUG_BT
//#define DEBUG_BLE
//#define DEBUG_WIFI

//#define DEBUG_CRC
//#define DEBUG_FRSKY
//#define DEBUG_FRSKY_MESSAGES
//#define DEBUG_FRSKY_RSSI            // 0xF101
//#define DEBUG_FRSKY_GPS_STATUS      // 0x410 and 0x5002
//#define DEBUG_FRSKY_GPS             // 0x800
//#define DEBUG_FRSKY_ALT             // 0x820
//#define DEBUG_FRSKY_SPEED           // 0x830
//#define DEBUG_FRSKY_HDG             // 0x840
//#define DEBUG_FRSKY_BATTERY         // 0x5003
//#define DEBUG_FRSKY_HOME            // 0x5004
//#define DEBUG_FRSKY_MESSAGES_UDP
//define DEBUG_FRSKY_MESSAGES_BT
//#define DEBUG_FRSKY_MESSAGES_BLE

//#define DEBUG_FRPORT_STREAM
//#define DEBUG_FPORT_BUFFER
//#define DEBUG_BOXGPS             // the GPS on the tracker box
//#define DEBUG_BOXCOMPASS         // The compass on the tracker box
//#define DEBUG_FRSKYD_Flight_Mode
//#define DEBUG_ESPNOW
//#define DEBUG_CRSF_BUF
//#define DEBUG_CRSF_GPS
//#define DEBUG_CRSF_BAT
//#define DEBUG_CRSF_ATTI
//#define DEBUG_CRSF_FLIGHT_MODE
//#define DEBUG_GOODFLAGS  // includes all the goodFlags
#define DEBUG_SENDUDP
#define SHOW_BAD_PACKETS // resonability test

#define Report_Packetloss   2     // F.Port packet loss every n minutes