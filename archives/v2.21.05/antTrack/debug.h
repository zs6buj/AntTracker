  //================================================================================================= 
  //============================= D E B U G G I N G   O P T I O N S   ===============================
  //================================================================================================= 

#define DEBUG_Minimum    //  Leave this as is unless you need the serial port for something else
#define DEBUG_Status

//#define DEBUG_All
//#define DEBUG_Protocol
//#define DEBUG_BAUD
//#define DEBUG_AzEl
//#define DEBUG_Servos 
//#define DEBUG_LEDs

//#define DEBUG_BOXCOMPASS                           
//#define DEBUG_Input
//#define DEBUG_Mav_Buffer  

//#define DEBUG_Mav_Heartbeat 
 
//#define DEBUG_Mav_GPS   

//#define DEBUG_FrSky
//#define DEBUG_LTM
//#define DEBUG_MSP
//#define DEBUG_inGPS               // a GPS on the 'plane

//#define DEBUG_EEPROM
//#define DEBUG_Time 
//#define DEBUG_Home

//#define DEBUG_BT
//#define DEBUG_WiFi
//#define DEBUG_CRC
//#define DEBUG_FrSky_Messages_UDP
//define DEBUG_FrSky_Messages_BT

//#define DEBUG_FrSky_GPS           // 0x5002
//#define DEBUG_FrSky_Home          // 0x5004

//#define DEBUG_FrSky_Messages
//#define DEBUG_Frsky_GPS_Status

//#define DEBUG_FrPort_Stream
//#define DEBUG_FPort_Buffer
//#define DEBUG_BOXGPS             // the GPS on the tracker box
//#define DEBUG_BOXCOMPASS         // The compass on the tracker box
//#define DEBUG_Our_FC_Heartbeat
//#define DEBUG_FrSkyD_Flight_Mode

#define DEBUG_CRSF_GPS
//#define DEBUG_CRSF_BAT
//#define DEBUG_CRSF_ATTI
#define DEBUG_CRSF_FLIGHT_MODE
//#define DEBUG_GOODFLAGS  // includes all the goodFlags
#define SHOW_BAD_PACKETS // resonability test

#define Report_Packetloss   2     // F.Port packet loss every n minutes