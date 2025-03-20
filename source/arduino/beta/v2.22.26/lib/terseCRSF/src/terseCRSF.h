#include <Arduino.h>
#include <iostream>
#include <string>
#include <HardwareSerial.h>

//#define RC_BUILD    // else TELEMETRY_BUILD
#if defined RC_BUILD
  //#define SUPPORT_SBUS_OUT 
#endif

#define MAJOR_VER          0
#define MINOR_VER          0
#define PATCH_LEV          9 

//#define TELEMETRY_SOURCE  1  // BetaFlight/CF
#define TELEMETRY_SOURCE  2  // EdgeTX/OpenTX

#if not defined TELEMETRY_SOURCE
  #define TELEMETRY_SOURCE  1
#endif

/*
  Changelog
  v0.0.3 2024-05-13 Add SHOW_BYTE_STREAM debug option
  v0.0.4 2024-05-17 Fix flight-mode position and length
  v0.0.5 2024-05-18 Rationalise macros
                    Add Telemetry source selection
  v0.0.6 2024-06-26 Add UART, UDP, BT telemetry example
                    Divide Battery Volts and Amps by 10
  V0.0.7 2024-07-09 Divide by V & A by a further 10
  v0.0.9 2025-01-30 Add support for BLE4.2
*/

//=========  D E M O / D E B U G   M A C R O S  ========

//#define DEMO_PWM_VALUES
//#define DEMO_SBUS
#define DEMO_CRSF_GPS
#define DEMO_CRSF_BATTERY
//#define DEMO_CRSF_LINK
#define DEMO_CRSF_ATTITUDE
#define DEMO_CRSF_FLIGHT_MODE

//#define SHOW_BUFFER
//#define SHOW_BYTE_STREAM
//#define SHOW_LOOP_PERIOD

#define SHOW_CRSF_CF_VARIO 
#define SHOW_CRSF_BARO   
#define SHOW_LINK_STATS
#define SHOW_CRSF_CHANNELS 
#define SHOW_CRSF_LINK_RX 
#define SHOW_CRSF_LINK_TX
#define SHOW_CRSF_DEVIDE_INFO
#define SHOW_CRSF_REQUEST_SETTINGS 
#define SHOW_CRSF_COMMAND 
#define SHOW_CRSF_RADIO 
#define SHOW_OTHER_FRAME_IDs

//==========================================

#define log   Serial

#define RADS2DEGS 180 / PI

typedef enum sbus_mode_state
{
  sbm_normal = 0,
  sbm_fast = 1
} sbmode_t;

// Frame id
#define GPS_ID                         0x02
#define CF_VARIO_ID                    0x07
#define BATTERY_ID                     0x08
#define BARO_ALT_ID                    0x09
#define HEARTBEAT_ID                   0x0B  // added
#define LINK_ID                        0x14  // link statistics
#define CHANNELS_ID                    0x16
#define LINK_RX_ID                     0x1C
#define LINK_TX_ID                     0x1D
#define ATTITUDE_ID                    0x1E
#define FLIGHT_MODE_ID                 0x21
#define PING_DEVICES_ID                0x28
#define DEVICE_INFO_ID                 0x29
#define REQUEST_SETTINGS_ID            0x2A
#define COMMAND_ID                     0x32
#define RADIO_ID                       0x3A

#if (TELEMETRY_SOURCE  == 1)      // BetaFlight
  #define CRSF_TEL_SYNC_BYTE  0xC8 
#elif (TELEMETRY_SOURCE  == 2)    // EdgeTX/OpenTx
  #define CRSF_TEL_SYNC_BYTE             0xEA  
#endif

#define CRSF_RC_SYNC_BYTE              0xEE

class CRSF
{
  // Data members
public:
// quote "416KBaud that CRSF uses", ELRS uses 420000
#if defined RC_BUILD 
const uint32_t crfs_baud = 420000;  // works for both
#else
const uint32_t crfs_baud = 420000;  // works for both
#endif

const uint8_t   crsf_buffer_size  = 64;
const uint8_t   max_rc_bytes      = 22; // just the RC bytes, not the full sbus
const uint8_t   sbus_buffer_size  = 25; // Header(1) + RC_bytes(22) + status(1)(los+fs) + footer(1)
const uint8_t   max_ch            = 8;  // max 18

uint8_t   frame_lth = 0;
uint8_t   crsf_buf[64] {};     // sizes as per above
uint8_t   rc_bytes[22] {};             
uint8_t   sb_bytes[25] {};    
uint16_t  pwm_val[8] {};  

uint8_t   crsf_id = 0;
uint8_t   crsf_lth = 0;

/* GPS ID:0x02 */
int32_t     gps_lat = 0;              // deg * 1e7
int32_t     gps_lon = 0;
float       gpsF_lat = 0;             // deg
float       gpsF_lon = 0.0;
uint16_t    gps_groundspeed = 0;
float       gpsF_groundspeed = 0.0;   // km/hr
uint16_t    gps_heading = 0;  
float       gpsF_heading = 0.0;       // deg
int16_t     gps_altitude = 0;         // metres, 1000m offset
uint8_t     gps_sats = 0;

/* Battery ID:0x08 */
uint16_t    bat_voltage = 0;           // mV * 100
float       batF_voltage = 0.0;        // volts
uint16_t    bat_current = 0;           // mA * 100
float       batF_current = 0.0;        // amps
uint32_t    bat_fuel_drawn = 0;        // uint24_t    mAh drawn
float       batF_fuel_drawn = 0.0;     // Ah drawn
uint8_t     bat_remaining = 0;         // percent

/* Link Statistics ID 0x14*/
uint8_t     link_up_rssi_ant_1 = 0;         // dBm * -1
uint8_t     link_up_rssi_ant_2 = 0;         // dBm * -1
uint8_t     link_up_quality = 0;            // packet_success_rate (%)
int8_t      link_up_snr = 0;                // db
uint8_t     link_diversity_active_ant = 0;  //(enum ant_1 = 0, ant_2)
uint8_t     link_rf_mode = 0;               //(enum 4fps = 0, 50fps, 150hz)
uint8_t     link_up_tx_power = 0;           //(enum 0mW = 0, 10mW, 25 mW, 100 mW, 500 mW, 1000 mW, 2000mW)
uint8_t     link_dn_rssi = 0;               // RSSI(dBm * -1)
uint8_t     link_dn_quality = 0;            // packet_success_rate (%)
int8_t      link_dn_snr = 0;                // db

    /* Attitude ID:0x1E */
    int16_t atti_pitch = 0;            // rad / 10000
float       attiF_pitch = 0.0;         // deg
int16_t     atti_roll = 0;             // rad / 10000
float       attiF_roll = 0.0;          // deg
int16_t     atti_yaw = 0;              // rad / 10000)
float       attiF_yaw = 0.0;           // deg

/* Flight Mode ID:0x21*/
uint8_t     flight_mode_lth = 0;
std::string flightMode;

//= (char *)"ACRO";

// Member function prototypes

private:
  Stream* crsf_port;   // pointer type
  Stream* sbus_port;   // pointer type
// own link stats
  uint32_t frames_read = 0;
  uint32_t good_frames = 0;
  uint32_t crc_errors = 0;
  uint32_t frame_errors = 0;
  uint16_t unknown_ids = 0;

public:
  //CRSF();   // for 
  bool initialise(Stream& port);
  bool sbus_initialise(Stream& port);
  bool readCrsfFrame(uint8_t &lth);
  uint8_t decodeTelemetry(uint8_t *_buf, uint8_t len);
  void decodeRC();
  void printByte(byte b, char delimiter);
  void printBytes(uint8_t *buf, uint8_t len);
  void printPWM(uint16_t *ch, uint8_t num_of_channels);
  void printLinkStats();
private:
  uint8_t crc8_dvb_s2(uint8_t, unsigned char);
  uint8_t crc8_dvb_s2_update(uint8_t, const void *, uint32_t);
  int32_t bytes2int32(uint8_t *byt);
  uint16_t bytes2uint16(uint8_t *byt);;
  int16_t bytes2int16(uint8_t *byt);
  uint16_t wrap360(int16_t);
  bool fixBadRc(uint8_t *);
  void prepSBUS(uint8_t *rc_buf, uint8_t *sb_buf, bool _los, bool _failsafe);
#if defined SUPPORT_SBUS_OUT
  void sendSBUS(uint8_t *sb_buf);
#endif
  bool bytesToPWM(uint8_t *sb_byte, uint16_t *ch_val, uint8_t max_ch);
  void pwmToBytes(uint16_t *in_pwm, uint8_t *rc_byt, uint8_t max_ch);

};  // end of class