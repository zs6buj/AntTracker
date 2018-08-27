  
/*  *****************************************************************************

    ZS6BUJ's Antenna Tracker

    Eric Stockenstrom - June 2017

    This program is free software. You may redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation. See here <http://www.gnu.org/licenses>

    The application was written in the hope that it will be useful, but it comes
    without any warranty or implied warranty of merchantability or fitness 
    for a particular purpose 
    
   *****************************************************************************
   
   The code is written from scratch, but I've taken ideas from Jalves' OpenDIY-AT and others. 
   Information and ideas on other protocols was obtained from GhettoProxy by Guillaume S.

   The target board is an STM32F103 "Blue Pill", chosen for its relative power, small size 
   and second (multi) serial port(s) for debugging. The arduino Teensy 3.x is also suitable,
   but much more expensive. The arduino mini pro or similar can be made to work but is not 
   recommended for performance reasons and lack of second (debugging) serial port.
   
   The application reads serial telemetry sent from a flight controller or GPS. The module 
   calculates where an airbourne craft is relative to the home position. From this it 
   calculates the azimuth and elevation of the craft, and then positions azimuth and 
   elevation PWM controlled servos to point a direction high-gain antenna for telemetry, 
   RC or/and and video.

  After the craft and the tracker have been powered on, and any telemetry is received by 
  the tracker, the tracker's LED flashes slowly. After a good GPS lock is received, the 
  tracker's LED flashes fast.

  Now, in order to establish the home position, push the Set_Home button. The LED goes on 
  solid.(If your craft has no FC and compass, see Heading_Source discussion below);

  For the tracker to position the servos relative to the compass direction of the craft,
  it needs to know the compass direction in degrees where the tracker antenna is pointing 
  at rest. For convenience we call this the Heading_Source. Three possible heading sources 
  are available:

    1) Flight Computer - use this if your craft has a Flight Computer with a compass
 
     Allign the craft's heading (where the nose points) with the direction of the tracker 
    antenna, and press the Set_Home button. The tracker will use the heading of the craft 
    obtained from the craft's Flight Computer via telemetry.
    
    2) GPS - use this if your craft has a GPS, but no FC or compass
 
    Position the tracker box such that the antenna faces the field straight ahead. Power up
    the craft and tracker, and wait for a good GPS lock. The tracker's LED will flash fast. 
    Walk several metres (say 5) straight ahead with the craft, place it on the ground, 
    return and press the Set_Home button. The tracker calculates the compass direction of a 
    vector (line) from where the craft was at the first GPS lock, to where it was when the 
    Set_Home button was pressed, then use as the assumed direction of the tracker antenna. 
    
    3) Tracker's Own Compass - use this if your craft has a GPS, but no FC or compass
 
    Glue a suitable magnetometer anywhere on the tracker box, facing the direction of the 
    antenna at rest.

  Note that tracking (movement of the antenna) will occur only when the craft is more than 
  minDist = 4 metres from home because accuracy increases sharply thereafter.   
    
  Before you build/compile the tracker firmware, be sure to un-comment the appropriate 
  #define macro according to your choice of heading source:

  //#define Heading_Source   1  // GPS 
  //#define Heading_Source   2  // Flight Computer 
  #define Heading_Source   3  // Tracker_Compass   

  If your servo pair is of the 180 degree type, be sure to comment out this line like 
  this:    //#define Az_Servo_360 

  Note that the elevation (180 degree) servo flips over to cover the field-of-view behind 
  you when the craft enters that space.

  If your servo pair comprises a 360 degree azimuth servo and 90 degree elevation servo, please
  un-comment the line like this:    #define Az_Servo_360 - 360 degree code contributed by macfly1202
  
  The small double bi-quad antenna has excellent gain, and works well with a vertically polarised antenna
  on the craft. Other antennas can be added for diversity, and some improvement in link resilience has 
  been observed despite the possibly lower gain of the other links. Of course it is possible to stack 
  double bi-quad antennas on the tracker, but more robust mechanicals will be called for.

  STM32F103C Blue-Pill wiring:
  
    Debug monitor   Serial(0) -->TX1 Pin A9
                              <--RX1 Pin A10  
                              
    Mavlink-in      Serial1   -->TX2 Pin A2   
                              <--RX2 Pin A3 
                              
    Compass         Optional     SCL Pin B6                          
                                 SDA Pin B7 
                                 
    Azimuth Servo                    Pin A7 
    Elevation Servo                  Pin A8 

    SetHome button                   Pin A5

    StatusLed                        Pin A6 - Off=No good GPS yet, flashing=good GPS but home not set yet, solid = ready to track
                                  
    Vcc - 3.3V      IMPORTANT!
    Gnd                              
                                 
v0.14 2017-05-22 Serial input version
v0.15 2017-05-30 Mod word length for 32bit MPUs like STM32
v0.20 2017-10-20 Fix gps timeout check
v0.21 2018-05-25 Enable debugging with #define, 360 azimuth servos with #define (as per macfly)
v0.22 2018-05-26 Include #define option for No_FC_Compass working
v0.23 2018-05-28 Switch to the GCS_Mavlink library.  
v0.24 2018-05-29 Limit close-to-home elevation error due to poor vertical GPS accuracy
v0.25 2018-05-29 Include #define Setup_BT option for HC-06 BlueTooth slave setup on input 
                 telemetry line 
v0.26 2018-05-30 Fix new bug in Servo. uint8_t now changed to init16_t. Oops. 
v0.27 2018-05-30 Fixed nasty typo in No_FC_Compass home calc!  lo1 - hom.lon; should be  lo1 = hom.lon;  
v0.28 2018-05-31 Relax GPS lock requirement from 3D Plus (fixtype=4) to 3D (fixtype=3)
v0.29 2018-07-01 Streamline use of Location structure 
v0.30 2018-07-20 Clarify compile options 
v0.31 2018-08-14 Add support for compass on the Tracker to determine direction the tracker is facing        
v0.32 2018-08-27 Tidy up tracker's-own-compass code. Patch Adafruit_HMC5883_Unified library.
 */
 
#include <GCS_MAVLink.h>
#include <Servo.h>

#define mavSerial            Serial1

//************************************* Please select your options here before compiling **************************
// Un-comment (activate) the options below
//#define Az_Servo_360   // Means the azimuth servo can point in a 360 deg circle, elevation servo 90 deg
                         // Default (comment out #define above) is 180 deg azimuth and 180 deg elevation 
//#define Heading_Source   1  // GPS 
//#define Heading_Source   2  // Flight Computer 
#define Heading_Source   3  // Tracker_Compass                         

//#define Setup_BT       // Sets up a previously unused BT-06 BT slave module
//*****************************************************************************************************************

#define Debug_Minimum    //  Leave this as is unless you need the serial port for something else
//#define Debug_All
//#define Debug_Status
//#define Mav_Debug_Heartbeat      
//#define Mav_Debug_GPS_Raw
//#define Mav_Debug_GPS_Int 
//#define Debug_AzEl
//#define Debug_Servos 
//#define Debug_LEDs
#define Debug_Compass                            

uint8_t azPWM_Pin =  7;    // A7 azimuth servo
uint8_t elPWM_Pin =  8;    // A8 elevation servo

uint8_t SetHomePin = 5;    // A5

uint8_t  StatusLed = 6;     // A6 - Off=No good GPS yet, flashing=good GPS but home not set yet, solid = ready to track
uint8_t  BoardLed = PC13;
uint8_t  ledState = LOW; 
uint32_t led_millis = 0;
uint32_t startup_millis = 0;
//*************
  
bool  mavGood=false;
bool  gpsGood = false;
bool  cpsGood = false;
bool  homGood=false;    
bool  homeInitialised = false;
bool  new_GPS_data = false;

uint32_t hb_millis = 0;
uint16_t  hb_count = 0;

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
};

struct Location hom     = {
  0,0,0,0};   // home location

struct Location cur      = {
  0,0,0,0};   // current location
  
struct Vector {
  float    az;                     
  float    el;                     
  int32_t  dist;
};

// Vector for home-to-current location
struct Vector hc_vector  = {
  90, 0, 0};
  
// Servo class declarations
Servo azServo;            // Azimuth
Servo elServo;            // Elevation

// ******************************************
// Mavlink Messages

// Mavlink Header
uint8_t    system_id;
uint8_t    component_id;
uint8_t    target_component;
uint8_t    mvType;

// Message #0  HEARTHBEAT 
uint8_t    ap_type = 0;
uint8_t    ap_autopilot = 0;
uint8_t    ap_base_mode = 0;
uint32_t   ap_custom_mode = 0;
uint8_t    ap_system_status = 0;
uint8_t    ap_mavlink_version = 0;

// Message #24  GPS_RAW_INT 
uint8_t    ap_fixtype = 3;            //   0= No GPS, 1 = No Fix, 2 = 2D Fix, 3 = 3D Fix
uint8_t    ap_sat_visible = 0;        // numbers of visible satelites
uint8_t    ap_gps_status = 0;         // (ap_sat_visible*10) + ap_fixtype; 
int32_t    ap_latitude = 0;           // 7 assumed decimal places
int32_t    ap_longitude = 0;          // 7 assumed decimal places
int32_t    ap_amsl24 = 0;             // 1000 = 1m
uint16_t   ap_eph;                    // GPS HDOP horizontal dilution of position (unitless)
uint16_t   ap_epv;                    // GPS VDOP vertical dilution of position (unitless)
uint16_t   ap_vel;                    //  GPS ground speed (m/s * 100)
uint16_t   ap_cog;                    // Course over ground in degrees * 100, 0.0..359.99 degrees. 

// Message GLOBAL_POSITION_INT ( #33 ) (Filtered)
int32_t ap_lat;            // Latitude, expressed as degrees * 1E7
int32_t ap_lon;            // Longitude, expressed as degrees * 1E7
int32_t ap_amsl33;         // Altitude above mean sea level (millimeters)
int32_t ap_alt_ag;         // Altitude above ground (millimeters)
int16_t ap_vx;             //  Ground X Speed (Latitude, positive north), expressed as m/s * 100
int16_t ap_vy;             //  Ground Y Speed (Longitude, positive east), expressed as m/s * 100
int16_t ap_vz;             // Ground Z Speed (Altitude, positive down), expressed as m/s * 100
uint16_t ap_hdg;           // Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees

//***************************************************
void setup() {

  #if defined Debug_Minimum || defined Debug_All || defined Debug_Status || defined Debug_LEDs  || defined Mav_Debug_Heartbeat || \
        defined Mav_Debug_GPS_Raw || defined Mav_Debug_GPS_Int || defined Debug_Servos || defined Debug_Compass
    #define Debug               Serial         // USB 
    Debug.begin(115200);                       // Debug monitor output
    delay(2000);
    Debug.println("Starting up......");
  #endif

  #ifdef Setup_BT
  // Using HC-06 bluetooth model front-end
  mavSerial.begin(57600);              // If speed already set to 57600, else it will ignore the next command
  Serial1.print("AT+BAUD4");           // Set the HC-06 speed to default speed 9600 bps for AT command mode
  delay(3000);                         // Wait for HC-06 reboot
  
  MavSerial.begin(9600);               //  HC-06 bluetooth module default speed for AT command mode
  delay(200); 
  Serial1.print("AT+NAMEAntTrack");    //  Optional - Configure your HC-06 bluetooth name
  delay(200); 
  Serial1.print("AT+BAUD7");           // Set the speed to Mavlink default speed 57600 bps
  delay(3000);                         // Wait for HC-06 reboot
  // Now proceed as per normal serial link
  #endif

  mavSerial.begin(57600);        // Telemetry input
  
  startup_millis = millis();
  pinMode(SetHomePin, INPUT_PULLUP); 
  pinMode(StatusLed, OUTPUT );  
  pinMode(BoardLed, OUTPUT);     // Board LED mimics status led
  digitalWrite(BoardLed, HIGH);  // Logic is reversed! Initialse off    

  azServo.attach(azPWM_Pin);
  elServo.attach(elPWM_Pin);
  PositionServos(90, 0, 90);   // Intialise servos to az=90, el=0, hom.hdg = 90;

  DisplayHeadingSource();
  
  #if (Heading_Source == 3)  // Tracker_Compass

    cpsGood = Initialise_Compass();  // Check if we have a compass on the Tracker
    
    if (!(cpsGood)) {
      #if defined Debug_Minimum || defined Debug_All || defined Debug_Compass  
        Debug.println("Heading_Source = Tracker's Own Compass, but no compass found! Aborting."); 
      #endif  
      while (1) delay (1000);  // Wait here forever
    }
   
    #if defined Debug_All || defined Debug_Compass
      Debug.println("Display tracker heading for 20 seconds");
      for (int i=1; i<20;i++) {
        GetMagHeading();
        delay(1000);
      }
    #endif
    
  #endif
  
  //TestServos();   // Uncomment this code to observe how well your servos reach their specified limits
                    // Fine tune MaxPWM and MinPWM in Servos module
}
//***************************************************
//***************************************************
void loop()  {
  
    MavLink_Receive();                      // Get Mavlink Data

    if (mavGood && (millis() - hb_millis >= 8000)){
      mavGood = false;   // If no heartbeat for 8 seconds then link timed out 
      gpsGood = false;
      #ifdef Debug_All 
      Debug.println("No heartbeat for 5 seconds"); 
      #endif
    }
    
    ServiceTheStatusLed();
    
    #if (Heading_Source == 2)  || (Heading_Source == 3) // Flight Computer or Tracker's Own Compass
      if (gpsGood==1 && ft) {
        ft=false;
        if (homeInitialised ==0)
          Serial.println("GPS lock good! Push set-home button anytime to start tracking.");
        else
          Serial.println("GPS lock good again!");
      }
    #endif

    if (mavGood && homeInitialised && new_GPS_data) {  //  every time there is new GPS data from mavlink
      GetAzEl(hom, cur);
      if (hc_vector.dist >= minDist) PositionServos(hc_vector.az, hc_vector.el, hom.hdg);
      new_GPS_data = false;
    }
 
  
  uint8_t SetHomeState = digitalRead(SetHomePin);            // Check if home button is pushed
  if (SetHomeState == 0 && gpsGood && !homeInitialised){     // pin 5 is pulled up - normally high
    SetHomeParameters();
  }
}
//***************************************************
//***************************************************
void SetHomeParameters() {
  
    #if (Heading_Source == 1)  // GPS 
      if (homGood) {            // Use home established when 3D+ lock established, homGood = 1 
        // Calculate heading as vector from home to where craft is now
        float a, la1, lo1, la2, lo2;
        lo1 = hom.lon;
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

        hom.lat = cur.lat;
        hom.lon = cur.lon;
        hom.alt = cur.alt;
        
        homeInitialised = true;
        DisplayHome();
      }
    
    #elif  (Heading_Source == 2)  // Flight Computer  
      hom.hdg = cur.hdg;
      
      hom.lat = cur.lat;
      hom.lon = cur.lon;
      hom.alt = cur.alt;
      
      homeInitialised = true;
      DisplayHome();
    
    #elif (Heading_Source == 3)  // Tracker's Own Compass 
      hom.hdg = GetMagHeading(); 
      
      hom.lat = cur.lat;
      hom.lon = cur.lon;
      hom.alt = cur.alt;
      
      homeInitialised = true;
      DisplayHome(); 
    
    #else  
      #error You must define at least one Heading_Source !
    #endif 
    

}
//***************************************************
void DisplayHome() {
  #if defined Debug_Minimum || defined Debug_All || defined Debug_AzEl
 //   Debug.print("******************************************");
    Debug.print("Home location set to Lat = "); Debug.print(hom.lat,7);
    Debug.print(" Lon = "); Debug.print(hom.lon,7);
    Debug.print(" Alt = "); Debug.print(hom.alt,0); 
    Debug.print(" hom.hdg = "); Debug.print(hom.hdg,0); 
    DisplayHeadingSource();
  #endif 
}
//***************************************************
void DisplayHeadingSource() {
#if defined Debug_Minimum || defined Debug_All || defined Debug_Compass  
  #if (Heading_Source == 1)  
      Debug.println(  "Heading_Source = Craft's GPS"); 
  #elif  (Heading_Source == 2)  
      Debug.println("  Heading_Source = Flight Computer");
  #elif (Heading_Source == 3)  
      Debug.println("  Heading_Source = Tracker's Own Compass"); 
  #endif 
#endif  
}
//***************************************************
uint8_t len;
void MavLink_Receive() { 
  mavlink_message_t msg;
  mavlink_status_t status;

  while(mavSerial.available()) {
    uint8_t c = mavSerial.read();
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
       

      #ifdef Debug_All
      //  Debug.print("Mavlink in: ");
      //  Debug.print("Message ID=");
      //  Debug.println(msg.msgid); 
      #endif
      switch(msg.msgid) {
    
        case MAVLINK_MSG_ID_HEARTBEAT:    // #0   http://mavlink.org/messages/common
          ap_type = mavlink_msg_heartbeat_get_type(&msg);
          ap_autopilot = mavlink_msg_heartbeat_get_autopilot(&msg);
          ap_base_mode = mavlink_msg_heartbeat_get_base_mode(&msg);
          ap_custom_mode = mavlink_msg_heartbeat_get_custom_mode(&msg);
          ap_system_status = mavlink_msg_heartbeat_get_system_status(&msg);
          ap_mavlink_version = mavlink_msg_heartbeat_get_mavlink_version(&msg);
          hb_millis=millis(); 

          #if defined Debug_All || defined Mav_Debug_Heartbeat
            Debug.print("Mavlink in #0 Heartbeat: ");           
            Debug.print("ap_type="); Debug.print(ap_type);   
            Debug.print("  ap_autopilot="); Debug.print(ap_autopilot); 
            Debug.print("  ap_base_mode="); Debug.print(ap_base_mode); 
            Debug.print(" ap_custom_mode="); Debug.print(ap_custom_mode);   
            Debug.print("  ap_system_status="); Debug.print(ap_system_status); 
            Debug.print("  ap_mavlink_version="); Debug.println(ap_mavlink_version);
          #endif

          if(!mavGood) {
            hb_count++; 
            #ifdef Debug_Status
            Debug.print(" hb_count=");
            Debug.print(hb_count);
            Debug.println("");
            #endif
            if((hb_count >= 3) || (homeInitialised)) {  // If 3 heartbeats or 1 hb && previously connected, we are connected
              mavGood=true;                       
              #ifdef Debug_Status
              Debug.println("mavGood=true");  
              #endif
              hb_count=0;
              }
          }
          break;
     
        case MAVLINK_MSG_ID_GPS_RAW_INT:          // #24
          if (!mavGood) break;        
          ap_fixtype = mavlink_msg_gps_raw_int_get_fix_type(&msg);                   // 0 = No GPS, 1 =No Fix, 2 = 2D Fix, 3 = 3D Fix
          ap_sat_visible =  mavlink_msg_gps_raw_int_get_satellites_visible(&msg);    // number of visible satelites
          ap_gps_status = (ap_sat_visible*10) + ap_fixtype; 
          if(ap_fixtype > 2)  {
            ap_latitude = mavlink_msg_gps_raw_int_get_lat(&msg);
            ap_longitude = mavlink_msg_gps_raw_int_get_lon(&msg);
            ap_amsl24 = mavlink_msg_gps_raw_int_get_alt(&msg);             // 1m =1000 
            ap_eph = mavlink_msg_gps_raw_int_get_eph(&msg);                // GPS HDOP 
            ap_epv = mavlink_msg_gps_raw_int_get_epv(&msg);                // GPS VDOP 
            ap_vel = mavlink_msg_gps_raw_int_get_vel(&msg);                // GPS ground speed (m/s * 100)
            ap_cog = mavlink_msg_gps_raw_int_get_cog(&msg);                // Course over ground (NOT heading) in degrees * 100
          }
          #if defined Debug_All || defined Mav_Debug_GPS_Raw 
            Debug.print("Mavlink in #24 GPS_RAW_INT: ");  
            Debug.print("ap_fixtype="); Debug.print(ap_fixtype);
            if (ap_fixtype==1) Debug.print(" No GPS");
              else if (ap_fixtype==2) Debug.print(" No Lock");
              else if (ap_fixtype==3) Debug.print(" 3D Lock");
              else if (ap_fixtype==4) Debug.print(" 3D+ Lock");
              else Debug.print(" Unknown");

            Debug.print("  sats visible="); Debug.print(ap_sat_visible);
            Debug.print("  GPS status="); Debug.print(ap_gps_status);
            Debug.print("  latitude="); Debug.print((float)(ap_latitude)/1E7, 7);
            Debug.print("  longitude="); Debug.print((float)(ap_longitude)/1E7, 7);
            Debug.print("  gps alt amsl"); Debug.print((float)(ap_amsl24)/1E3, 0);
            Debug.print("  eph (hdop)="); Debug.print(ap_eph);               // HDOP
            Debug.print("  epv (vdop)="); Debug.print(ap_epv);
            Debug.print("  vel="); Debug.print((float)ap_vel / 100, 1);         // GPS ground speed (m/s)
            Debug.print("  cog="); Debug.println((float)ap_cog / 100, 1);       // Course over ground in degrees
          #endif     
          break;
 
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:     // #33
          if ((!mavGood) || (ap_fixtype < 3)) break;  
          // We have a 3D Lock - change to 4 if you want 3D plus
          
          ap_lat = mavlink_msg_global_position_int_get_lat(&msg);             // Latitude, expressed as degrees * 1E7
          ap_lon = mavlink_msg_global_position_int_get_lon(&msg);             // Pitch angle (rad, -pi..+pi)
          ap_amsl33 = mavlink_msg_global_position_int_get_alt(&msg);          // x Supposedly altitude above mean sea level (millimeters)
          ap_alt_ag = mavlink_msg_global_position_int_get_relative_alt(&msg); // Altitude above ground (millimeters)
          ap_vx = mavlink_msg_global_position_int_get_vx(&msg);               //  Ground X Speed (Latitude, positive north), expressed as m/s * 100
          ap_vy = mavlink_msg_global_position_int_get_vy(&msg);               //  Ground Y Speed (Longitude, positive east), expressed as m/s * 100
          ap_vz = mavlink_msg_global_position_int_get_vz(&msg);               // Ground Z Speed (Altitude, positive down), expressed as m/s * 100
          ap_hdg = mavlink_msg_global_position_int_get_hdg(&msg);             // Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees          ap_ap_amsl = mavlink_msg_attitude_get_yaw(&msg);                // Yaw angle (rad, -pi..+pi)
          
          if (!gpsGood) {
            gpsGood = true;
            #ifdef Debug_Status
              Debug.println("gpsGood=true");  
            #endif
          }
          new_GPS_data = true;
          
          if (!homGood) {
            homGood = true;
            #ifdef Debug_Status
              Debug.println("homGood=true");  
            #endif
            hom.lat = (float)ap_lat / 1E7;
            hom.lon = (float)ap_lon / 1E7;
            hom.alt = (float)ap_amsl24 / 1E3;
            hom.hdg = (float)ap_hdg / 100;

            #if defined Debug_All || defined Mav_Debug_GPS_Int 
              Debug.print("******************************************Mavlink in #33 GPS Int: Home established: ");       
              Debug.print("hom.lat=");  Debug.print(hom.lat, 7);
              Debug.print(" hom.lon="); Debug.print(hom.lon, 7 );        
              Debug.print(" hom.alt="); Debug.print(hom.alt, 1);
              Debug.print(" hom.hdg="); Debug.println(hom.hdg);                   
            #endif 
          } 

          cur.lat =  (float)ap_lat / 1E7;
          cur.lon = (float)ap_lon / 1E7;
          cur.alt = ap_amsl24 / 1E3;
          cur.hdg = ap_hdg / 100;
          
          #if defined Debug_All || defined Mav_Debug_GPS_Int
            Debug.print("Mavlink in #33 GPS Int: ");
            Debug.print(" ap_lat="); Debug.print((float)ap_lat / 1E7, 6);
            Debug.print(" ap_lon="); Debug.print((float)ap_lon / 1E7, 6);
            Debug.print(" ap_amsl="); Debug.print((float)ap_amsl33 / 1E3, 0);
            Debug.print(" ap_alt_ag="); Debug.print((float)ap_alt_ag / 1E3, 1);           
            Debug.print(" ap_vx="); Debug.print((float)ap_vx / 100, 1);
            Debug.print(" ap_vy="); Debug.print((float)ap_vy / 100, 1);
            Debug.print(" ap_vz="); Debug.print((float)ap_vz / 100, 1);
            Debug.print(" ap_hdg="); Debug.println((float)ap_hdg / 100, 1);
          #endif 
                            
          break;
      }
    }
  }
}
//***************************************************
void TestServos() {
PositionServos(90, 0, 90); 
  for (int i=1; i<=360; i++) {
    delay(60);
    PositionServos(i, 30, 90);   
    }
  for (int i=1; i<=170; i++) {
    delay(60);
    PositionServos(90, i, 90);   
    }
   PositionServos(90, 0, 90);   
   }

//***************************************************

void ServiceTheStatusLed() {
  #ifdef Debug_LEDs
    Debug.print("mavGood = ");
    Debug.print(mavGood);
    Debug.print("   gpsGood = ");
    Debug.print(gpsGood);
    Debug.print("   homeInitialised = ");
    Debug.println(homeInitialised);
 #endif

  if (gpsGood) {
    if (homeInitialised) 
      ledState = HIGH;
    else 
      BlinkLed(100);
    }
  else 
     if (mavGood) 
       BlinkLed(1300);
     else
       ledState = LOW;
       
    digitalWrite(StatusLed, ledState);  
    digitalWrite(BoardLed, !ledState);
}

//***************************************************
void BlinkLed(uint16_t rate) {
  unsigned long cMillis = millis();
     if (cMillis - led_millis >= rate) {    // blink period
        led_millis = cMillis;
        if (ledState == LOW) {
          ledState = HIGH; }   
        else {
          ledState = LOW;  } 
      }
}

//***************************************************
String TimeString (unsigned long epoch){
 uint8_t hh = (epoch  % 86400L) / 3600;   // remove the days (86400 secs per day) and div the remainer to get hrs
 uint8_t mm = (epoch  % 3600) / 60;       // calculate the minutes (3600 secs per minute)
 uint8_t ss = (epoch % 60);               // calculate the seconds

  String S = "";
  if (hh<10) S += "0";
  S += String(hh);
  S +=":";
  if (mm<10) S += "0";
  S += String(mm);
  S +=":";
  if (ss<10) S += "0";
  S += String(ss);
  return S;
}

