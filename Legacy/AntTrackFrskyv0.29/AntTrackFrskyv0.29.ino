
/*

    ZS6BUJ's Antenna Tracker

     Universal Frsky serial telemetry input version - supports X, D and Mavlink Passthrough

     Eric Stockenstrom - First code June 2017
     

This application reads serial telemetry sent from a flight controller or GPS. The module 
calculates where an airbourne craft is relative to the home position. From this it 
calculates the azimuth and elevation of the craft, and then positions azimuth and 
elevation PWM controlled servos to point a directional high-gain antenna for telemetry, 
RC or/and and video.

If your servo pair is of the 180 degree type, be sure to comment out this line like 
this:    //#define Az_Servo_360 

Note that the elevation (180 degree) servo flips over to cover the field-of-view behind 
you when the craft enters that space.

If your servo pair comprises a 360 degree azimuth servo and 90 degree elevation servo, be 
sure to un-comment out this line like this:    #define Az_Servo_360 
360 degree code contributed by macfly1202

The code is written from scratch, but I've taken ideas from Jalves' OpenDIY-AT and others. 
Information and ideas on other protocols was obtained from GhettoProxy by Guillaume S.

The target board is an STM32F103 "Blue Pill", chosen for its relative power, small size 
and second (multi) serial port(s) for debugging. The arduino Teensy 3.x is also suitable,
but much more expensive. The arduino mini pro or similar can be made to work but is not 
recommended for perfomance reasons and lack of second (debugging) serial port.

To use the AntTRacker, position it with the antenna facing the centre of the field in front 
of you. Position the craft a few metres further, also facing the same heading for take-off. 
Tracking (movement of the antenna) will occur only when the craft is more than minDist = 4 
metres from home because accuracy increases sharply thereafter.

When your flight system includes a compass/magnetometer:

0 Be sure to comment out this line like this :  //#define No_Compass  
1 Power up the craft.
2 Power up the ground ground system.
3 Power up the AntTracker.
4 When AntTracker successfully connects to the ground system, the LED on the front flashes slowly.
5 When AntTracker receives its first good GPS location record, the LED flashes fast.
6 Make sure the front of your craft is pointing in the direction of the AntTracker antenna at rest.
  The compass heading of the craft now determines the relative heading of the AntTracker antenna.
7 Push the home button to register the home position and heading.  The LED goes solidly on.
8 Enjoy your flight! The Tracker will track your craft anywhere in the hemisphere around you.

When your flight system does NOT include a compass/magnetometer:

0 Be sure to un-comment this line like this :  #define No_Compass  
1 Power up the craft.
2 Power up the ground system.
3 Power up the AntTracker.
4 When AntTracker successfully connects to the ground system, the LED on the front flashes slowly.
5 When AntTracker receives its first 3D fix GPS location record, the LED flashes fast.
6 Pick up your craft and walk forward several metres (4 to 8) in the direction of the AntTracker antenna at rest.
  This deternines the relative heading for the AntTracker antenna.
7 Return and push the home button to register the home position and heading. The LED goes solidly on.
8 Enjoy your flight! The Tracker will track your craft anywhere in the hemisphere around you.

The small double bi-quad antenna has excellent gain, and works well with a vertically polarised stick on the craft. Other reception sticks 
can be added for diversity, and some improvement in link resilience has been observed despite the lower gain of the other links. 
Of course it would be possible to stack double bi-quad antennas on the AntTracker, but more robust mechanicals will be called for.
NOTE: In Mavlink Passthrough mode only, relative altitude is GPS (not barometer) derived, so altitude is inaccurate until at least 10 satelites are visible. 
      More is better.

  Connections to STM32F103C are:

    1) To Frsky S.Port Converter   NOT ESSENTIAL    -->TX2 Pin A2   Serial1 
    2) From Frsky S.Port Converter                  <--RX2 Pin A3   Serial1  
    3) SetHomePin                                          Pin A5
    4) StatusLed                                           Pin A6
    5) Azimuth Servo                                       Pin A7
    6) Elevation Servo                                     Pin A8 
    7) Vcc 3.3V !
    8) GND


1 Power up the craft.
2 Power up the ground Raspi board.
3 Power up the AntTracker.
4 When AntTracker successfully connects to the Taranis, the LED on the front flashes slowly.
5 When more than 10 GPS satellites are visible, and HDOP is 3 or greater, and AntTracker receives its first good GPS location record, the LED flashes fast.
6 Push the home button to register the home position and heading. The LED goes on solid.
7 Enjoy your flight! The Tracker will track your craft anywhere in the hemisphere around you (but not closer than 3 metres).

The small double bi-quad antenna has excellent gain, and works well with a vertically polarised stick on the craft. Other reception sticks 
can be added for diversity, and some improvement in link resilience has been observed despite the lower gain of the other links. 
Of course it would be possible to stack double bi-quad antennas on the AntTracker, but more robust mechanicals will be called for.

v0.14 2017-05-22 Serial input version
v0.15 2017-05-30 Mod word length for 32bit MPUs like STM32
v0.18 2017-10-09 Frsky S.Port version
v0.19 2017-10-17 Include support for Frsky D hub protocol direct from Pixhawk/APM
v0.20 2017-10-20 Fix gps timeout check
v0.21 2017-11-04 Add Frsky Mavlink Passthrough
v0.22 2017-11-10 Tidy up after flight test
v0.23 2018-06-26 Include support for 360 deg servos, craft with no GPS, limit close-to-home altitude error
v0.24 2018-07-01 Streamline use of Location structure  
v0.25 2018-07-10 Improve debugging with #define Debug_SPort and #define Debug_Telemetry
v0.26 2018-07-14 Force lon and lat not zero to real compare, not integer.
v0.28 2018-07-17 Include decode of iNav temp2 sensor 0x410 GPS status.
v0.29 2018-07-20 Clarify compile options 
 
 */

#include <Servo.h>

//************************************* Please select your options here before compiling **************************
// Un-comment (activate) the options below
//#define Az_Servo_360   // Means the azimuth servo can point in a 360 deg circle, elevation servo 90 deg
                         // Default (comment out #define above) is 180 deg azimuth and 180 deg elevation 
//#define No_Compass     // Use the GPS to determine initial heading of craft, and initial heading of Tracker
//*****************************************************************************************************************

#define Debug_All
//#define DebugStatusFlags
#define Debug_SPort
#define Debug_Telemetry
//#define Debug_AzEl
//#define Debug_Servos 
//#define Debug_LEDs    

boolean FT = true;
int iLth=0;
int pLth;
byte chr = 0x00;
const int packetSize = 70; 
byte packetBuffer[packetSize]; 
short crc=0;  
boolean crc_bad; 

//************* Pin Assignments
// BT Serial1 telemetry pins - RX = A3    TX = A2
// Serial for printout       - RX = A10   TX = A9

uint16_t azPWM_Pin =  7;    // A7 Azimuth
uint16_t elPWM_Pin =  8;    // A8 Elevation

uint16_t SetHomePin = 5;    // A5

uint16_t StatusLed = 6;  // A6 - Off=No good GPS yet, Flashing=Good GPS but Home not set yet, Solid = Ready to Track
uint16_t BoardLed = PC13;
uint16_t ledState = LOW; 
uint32_t ledMillis = 0;
uint32_t startup_millis = 0;

//*************

bool homeInitialised = false;
bool serGood = false;
bool lonGood = false;
bool latGood = false;
bool altGood = false;
bool hdgGood = false;
bool gpsGood = false;
bool gpsGoodMsg = false;
bool hdopGood = false;
bool Passthrough = false;
bool iNav = false;

uint32_t gpsMillis = 0;

//  variables for servos
uint16_t azPWM = 0;
uint16_t elPWM = 0;
uint16_t LastGoodpntAz = 90;
uint16_t LastGoodEl = 0;

float tLon = 0;

float fvx = 0;
float fvy = 0;
float fvz = 0;
float fhdg = 0;

// 0x800 GPS
uint32_t fr_latlong;
uint32_t fr_velyaw;
short ms2bits;

uint32_t fr_heading;
uint32_t fr_altitude;
uint32_t fr_home;
uint16_t fr_home_dist;
float fHomeDist;
short fr_pwr;
uint32_t fr_gps;
uint16_t fr_numsats;
uint8_t fr_gpsStatus;
uint8_t fr_hdop;
uint8_t fr_vdop;
uint8_t neg;
uint8_t gpsAlt;

//    0x410 iNav GPS Status
uint32_t fr_gps_status;
uint8_t fr_gps_fix;
uint8_t fr_gps_homefix;
uint8_t fr_gps_homereset;
uint8_t fr_gps_accuracy;     // 0 thru 9 highest accuracy
uint16_t fr_gps_numsats;

//Working variables for iNav 0x410 decode
uint8_t  d1;
uint16_t dr1;
uint8_t  d2;
uint16_t dr2;
uint8_t  d3;
uint16_t dr3;
uint8_t  d4;
  
uint8_t d14;      // say 7 / 4 = 1
uint8_t dr12;     //     7 % 4 = 3
uint8_t d12;      //     3 / 2 = 1
uint8_t d11;      //     3 % 2 = 1
//.....................  
uint16_t lonDDMM;
uint16_t latDDMM;
uint16_t DD;
uint16_t MM;
uint16_t mmmm;
float MMmmmm;
char NS;   // No kidding!
char EW;

uint8_t minDist = 4;  // dist from home before tracking starts

// 3D Location vectors
struct Location {
  float lat; 
  float lon;
  float alt;
  float hdg;
};

struct Location hom         = {
  0,0,0,0};   // home location

//float homeHdg;

struct Location cur      = {
  0,0,0,0};   // current location

struct Vector {
  float az;                     
  float el;                     
  long  dist;
};

// Vector for home-to-current location
struct Vector hc_vector  = {
  90, 0, 0};

// Servo instances
Servo azServo;            // Azimuth
Servo elServo;            // Elevation

//***************************************************
void setup()
{
#define Frsky               Serial1         // From S.Port conveter

#if defined Debug_All || defined Debug_SPort || defined Debug_Telemetry || defined Debug_AzEl || defined Debug_LEDs || defined Debug_Servos
    #define Debug               Serial         // USB 
    Debug.begin(115200);                       // Debug monitor output
    delay(2000);
    Debug.println("Starting up......");
  #endif

  Frsky.begin(57600);        // Telemetry input

  pinMode(SetHomePin, INPUT_PULLUP); 
  pinMode(StatusLed , OUTPUT ); 
  pinMode(BoardLed, OUTPUT);     // Board LED mimics status led
  digitalWrite(BoardLed, HIGH);  // Logic is reversed! Initialse off

  azServo.attach(azPWM_Pin);
  elServo.attach(elPWM_Pin);
  PositionServos(90, 0, 90);   // Intialise servos to az=90, el=0, homeHdg = 90;
  
// TestServos();   // Uncomment this code to observe how well your servos reach 0 deg and 180 deg
                 // Fine tune MaxPWM and MinPWM in Servos module
  
}
//***************************************************
//***************************************************
void loop()  {
  
  if (FT) {                  // Sync to the first start/stop character
    chr = NextChar();
    while (!(chr==0x7E)) {
      chr = NextChar();
     }
    FT=false;
  }
  // Candidate found 
  
  packetBuffer[0]=chr;            // Start-Stop character 0x7E
  packetBuffer[1]=NextChar();     // Sensor-ID
  
  chr=NextChar();     // Start-Stop or Data-Frame Header
  
  if (chr==0x10) {    // If data frame header
    packetBuffer[2]=chr;
    boolean goodPacket=ParseData();
    if (goodPacket) ProcessData();
    #if defined Debug_All || defined Debug_SPort
      DisplayTheBuffer(10);
    #endif 
    chr=NextChar();   //  Should be the next Start-Stop  
    }
  else {
    #if defined Debug_All || defined Debug_SPort
      DisplayTheBuffer(2);
    #endif  
    }

      
  if (!(chr==0x7E)) FT=true;  //  If next char is not start-stop then the frame sync has been lost. Resync.

//  ++++++++++++++++++++

    #if defined Debug_All || defined DebugStatusFlags
      DisplayStatusFlags();
    #endif

    if ((Passthrough) || (iNav))
      gpsGood = hdopGood & lonGood & latGood & altGood & hdgGood ;
     else
      gpsGood = lonGood & latGood & altGood & hdgGood ;

    if (gpsGood) {
      gpsMillis = millis();                 // Time of last good GPS packet
      if (!gpsGoodMsg) {
        gpsGoodMsg = true;
        #if defined Debug_All || defined Debug_Telemetry
        if (!homeInitialised)
            Debug.println("GPS lock good! Push set-home button anytime to start tracking.");
        else
            Debug.println("GPS lock good again!");
        #endif  
      }
      
      if (homeInitialised) {    
        GetAzEl(hom, cur);
        if (hc_vector.dist >= minDist) PositionServos(hc_vector.az, hc_vector.el, hom.hdg);
      }
  
  uint8_t SetHomeState = digitalRead(SetHomePin);
  
  #ifdef No_Compass  // If no compass use home determined when 3D+ lock established, homGood = 1
    if ((SetHomeState == 0) && (gpsGood) && (!homeInitialised)){   
      homeInitialised = true;
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

      DisplayHome();
      
    }  
  #else    // if have compass, use FC heading
  if (SetHomeState == 0 && gpsGood && !homeInitialised){     // pin 5 is pulled up - normally high
    homeInitialised = true;
    hom.lat = cur.lat;
    hom.lon = cur.lon;
    hom.alt = cur.alt;
    hom.hdg = cur.hdg;
   
    DisplayHome();
    
    }
  #endif 

    delay(10);
  }
}  
//***************************************************
//***************************************************
void DisplayHome() {
    #if defined Debug_All || defined Debug_Telemetry || defined Debug_AzEl
 //   Debug.print("******************************************");
    Debug.print("Home location set to Lat = "); Debug.print(hom.lat,7);
    Debug.print(" Lon = "); Debug.print(hom.lon,7);
    Debug.print(" Alt = "); Debug.print(hom.alt,0); 
    Debug.print(" hom.hdg = "); Debug.println(hom.hdg,0); 
    #endif 
}
//***************************************************
void Add_Crc (uint8_t byte) {
  crc += byte;       //0-1FF
  crc += crc >> 8;   //0-100
  crc &= 0x00ff;
  crc += crc >> 8;   //0-0FF
  crc &= 0x00ff;
  }
//***************************************************
byte NextChar() {
byte x;

  iLth=Serial1.available();     //   wait for more data
  while (iLth==0) {
    CheckForTimeouts();
    iLth=Serial1.available();
  }
  // Data is available
  serGood = true;                     // We have a good serial connection!
  x =Frsky.read();

  return x;
}
//***************************************************
boolean ParseData() {
 crc=0;
  Add_Crc(packetBuffer[2]);           // data frame char into crc
 
  for (int i=3; i<=8; i++) {
    chr = NextChar();
    packetBuffer[i]=chr;
    Add_Crc(chr);
  }
  chr=NextChar(); 
  packetBuffer[9]=chr;  //  crc

  if (chr==(0xFF-crc)){
    crc_bad = false; 
 //  Debug.println("CRC Good");
  }
  else {
    crc_bad=true;
//   Debug.println("CRC Bad");
  }
  return !crc_bad;
} 
//***************************************************
void ProcessData() {
  // Do the sensor packets according to value type
 uint16_t ValueType = Unpack_uint16(3);

      switch(ValueType) {
                   // *****************************************************************
                //   Old D Style Hub Protocol below 
                  case 0x01:                         // GPS Alt BP
                    cur.alt = Unpack_uint16(5);
                    if (!(cur.alt==0.0000)) altGood=true; 
                    #if defined Debug_All || defined Debug_Telemetry              
                      Debug.print(" GPS Altitude 0x01=");
                      Debug.println(cur.hdg,0);
                    #endif
                    break;
                  case 0x12:                        // Lon BP
                    lonDDMM = Unpack_uint32(5);
                    #if defined Debug_All || defined Debug_Telemetry              
                      Debug.print(" lonDDMM 0x12=");
                      Debug.println(lonDDMM);
                    #endif             
                    break;
                  case 0x13:                       // Lat BP
                    latDDMM = Unpack_uint32(5);
                    #if defined Debug_All || defined Debug_Telemetry              
                      Debug.print(" latDDMM 0x13=");
                      Debug.println(latDDMM);
                    #endif           
                    break;
                  case 0x14:        
                    cur.hdg = Unpack_uint16(5);      // Course / Heading BP
                    if (!(cur.hdg==0.000)) hdgGood=true;
                    #if defined Debug_All || defined Debug_Telemetry              
                      Debug.print(" Heading 0x14=");
                      Debug.println(cur.hdg,0);
                    #endif
                    break;               
                  case 0x1A:                      // Lon AP
                    mmmm = Unpack_uint32(5);
                    DD = lonDDMM/100;
                    MM = lonDDMM -(DD*100);
                    MMmmmm = MM + (mmmm/1E4);       
                    tLon = DD + (MMmmmm/60);
                    if (EW==0x57)  tLon = 0-tLon; //  "W", as opposed to "E"
                    // Store tLon and wait for lat to make matched pair    
                     lonGood=true;
                    #if defined Debug_All || defined Debug_Telemetry              
                      Debug.print(" Lon After Point 0x1A=");
                      Debug.println(tLon,0);
                    #endif
                     
                    break;
                  case 0x1B:                      // Lat AP
                    mmmm = Unpack_uint32(5);
                    DD = latDDMM/100;
                    MM = latDDMM -(DD*100);
                    MMmmmm = MM + (mmmm/1E4);
                    cur.lat = DD + (MMmmmm/60);     
                    if (NS==0x53) cur.lat = 0-cur.lat;  //  "S", as opposed to "N" 
                    cur.lon = tLon;  // Complete the pair 
                    latGood=true;
                    #if defined Debug_All || defined Debug_Telemetry              
                      Debug.print(" Lat After Point 0x1B=");
                      Debug.println(cur.lat,0);
                    #endif
                    break;
                  case 0x22:                      // Lon E/W
                    EW = Unpack_uint8(5);
                    #if defined Debug_All || defined Debug_Telemetry              
                      Debug.print(" Lon E/W 0x22=");
                      Debug.println(EW);
                    #endif
                    break;
                  case 0x23:                      // Lat N/S
                    NS = Unpack_uint8(5);  
                    #if defined Debug_All || defined Debug_Telemetry              
                      Debug.print(" Lon Lat N/S 0x23=");
                      Debug.println(NS);
                    #endif
                    break;

                    
                // *****************************************************************
                //   New S.Port Protocol below    
                
                  case 0x100:              // Altitude
                    fr_altitude= Unpack_uint32(5);
                    cur.alt  = fr_altitude / 100;
                    if (!(cur.alt ==0)) altGood=true; 
                    break; 
                  case 0x410:              // Tmp2 - iNav GPS status 
                    iNav=true;
                    fr_gps_status= Unpack_uint32(5);
                    
                    // decode to digits 1 thru 4
                    d1 = (fr_gps_status / 1000);
                    dr1 = d1 * 1000;
                    d2 = (fr_gps_status - dr1) / 100;
                    dr2 = dr1 + (d2 * 100);
                    d3 = (fr_gps_status - dr2) / 10;
                    dr3 = dr2 + (d3 * 10);
                    d4 = fr_gps_status - dr3;
                    
                    // decode to sub-digits of d1
                    d14 = d1 / 4;      // say 7 / 4 = 1
                    dr12 = d1 % 4;     //     7 % 4 = 3
                    d12 = dr12 / 2;    //     3 / 2 = 1
                    d11 = dr12 % 2;    //     3 % 2 = 1

                    fr_gps_fix = d11;
                    fr_gps_homefix = d12;
                    fr_gps_homereset = d14;
                    fr_gps_accuracy = d2;   // 0 thru 9 highest accuracy
                    fr_gps_numsats = (d3*10) + d4;

                    hdopGood = (fr_gps_accuracy > 7);  // 0 thru 9 - 9 best
                    
                    #if defined Debug_All || defined Debug_Telemetry 
                      Debug.print("fr_gp_fix="); Serial.print(fr_gps_fix);     
                      Debug.print(" fr_gps_homefix ="); Debug.print(fr_gps_homefix);
                      Debug.print(" fr_gp_homereset="); Debug.print(fr_gps_homereset);     
                      Debug.print(" fr_gps_accuracy ="); Debug.print(fr_gps_accuracy);
                      Debug.print(" fr_gps_numsats="); Debug.println(fr_gps_numsats); 
                    #endif  
                    break;                    

                 case 0x800:                      // Latitude and Longitude
                   fr_latlong= Unpack_uint32(5);
                   #if defined Debug_All || defined Debug_Telemetry  
                     Debug.print(" latlong=");
                     Debug.println(fr_latlong);
                   #endif   
                   ms2bits = fr_latlong >> 30;
                   fr_latlong = fr_latlong & 0x3fffffff; // remove ms2bits
                   #if defined Debug_All     
                     Debug.print(" ms2bits=");
                     Debug.println(ms2bits);
                   #endif   
                   switch(ms2bits) {
                     case 0:   // Latitude Positive
                       cur.lat = fr_latlong / 6E5;     // Only ever update lon and lat in pairs. Lon always comes first                   
                       cur.lon = tLon;                 // Update lon from temp lon below      
                       #if defined Debug_All || defined Debug_Telemetry                 
                         Debug.print(" 0x800 latitude=");
                         Debug.println(cur.lat,7);
                       #endif
                       latGood=true;
                       break;
                     case 1:   // Latitude Negative       
                       cur.lat = 0-(fr_latlong / 6E5);  
                       #if defined Debug_All || defined Debug_Telemetry            
                         Debug.print(" 0x800 latitude=");
                         Debug.println(cur.lat,7);  
                       #endif   
                       cur.lon = tLon;

                       if (!(cur.lat==0.000000) && !(cur.lon==0.000000)) latGood=true;
                       break;
                     case 2:   // Longitude Positive
                       tLon = fr_latlong / 6E5;                       
                       #if defined Debug_All || defined Debug_Telemetry    
                         Debug.print(" 0x800 longitude=");
                         Debug.println(cur.lon,7); 
                       #endif                       
                       lonGood=true;
                       break;
                     case 3:   // Longitude Negative
                       tLon = 0-(fr_latlong / 6E5);  
                       #if defined Debug_All                        
                         Debug.print(" 0x800 longitude=");
                         Debug.println(tLon,7); 
                       #endif                   
                       lonGood=true;
                       break;
                    }
                    break;
                  case 0x820:              // Altitude
                    fr_altitude= Unpack_uint32(5);
                    cur.alt  = fr_altitude / 100;
                    if (!(cur.alt ==0.0000)) altGood=true; 
                    #if defined Debug_All || defined Debug_Telemetry    
                       Debug.print(" 0x820 altitude=");
                       Debug.println(cur.alt,1); 
                     #endif    
                    
                    break;          
                  case 0x840:              // Heading
                    fr_heading= Unpack_uint32(5);
                    cur.hdg = fr_heading / 100;
                    if (!(cur.hdg==0.0000)) hdgGood=true;
                    #if defined Debug_All || defined Debug_Telemetry    
                       Debug.print(" 0x840 heading=");
                       Debug.println(cur.hdg,1); 
                     #endif               
                    break;

                 // *****************************************************************    
                 //   Mavlink Passthrough Protocol below     
                  case 0x5002:
                  
                  // GPS Status &  gpsAlt
                    Passthrough=true;

                    fr_gps = Unpack_uint32(5);
                    fr_numsats = bit32Extract(fr_gps, 0, 4);
                    fr_gpsStatus = bit32Extract(fr_gps, 4, 2) + bit32Extract(fr_gps, 14, 2);
                    fr_hdop = bit32Extract(fr_gps, 7, 7) * (10^bit32Extract(fr_gps, 6, 1));
                    gpsAlt = bit32Extract(fr_gps, 24, 7) * (10^bit32Extract(fr_gps, 22, 2));
                    cur.alt  = (float)(gpsAlt) / 10;
                    neg = bit32Extract(fr_gps, 31, 1);
                    if (neg==1) cur.alt = 0 - cur.alt;
                    
                    hdopGood=(fr_hdop>=3) && (fr_numsats>10);
                    #if defined Debug_All || defined Debug_Telemetry 
                      Debug.print(" 0x5002 Num sats=");
                      Debug.print(fr_numsats);
                      Debug.print(" gpsStatus=");
                      Debug.print(fr_gpsStatus);                
                      Debug.print(" HDOP=");
                      Debug.print(fr_hdop);
                      Debug.print(" fr_vdop=");
                      Debug.print(fr_vdop);                     
                      Debug.print(" gpsAlt=");
                      Debug.print(cur.alt, 1);
                      Debug.print(" neg=");
                      Debug.println(neg);   
                    #endif

                    break;
                  case 0x5004:                         // Home
                    fr_home = Unpack_uint32(5);
                    fr_home_dist = bit32Extract(fr_home, 2, 10) * (10^bit32Extract(fr_home, 0, 2));
                    fHomeDist = (float)fr_home_dist * 0.1;  // Not used here 
                    cur.alt = bit32Extract(fr_home, 14, 10) * (10^bit32Extract(fr_home, 12, 2)) * 0.01; // metres
                    if (bit32Extract(fr_home,24,1) == 1) 
                      cur.alt = cur.alt * -1;
                    altGood=true; 
                    #if defined Debug_All || defined Debug_Telemetry 
                      Debug.print(" 0x5004 Dist to home=");
                      Debug.print(fHomeDist, 1);             
                      Debug.print(" Rel Alt=");
                      Debug.println(cur.alt,1);
                    #endif
                    break;
                      
                  case 0x5005:                      
                  // Vert and Horiz Velocity and Yaw angle (Heading)
                    fr_velyaw = Unpack_uint32(5);      
                    fr_velyaw = fr_home_dist = bit32Extract(fr_velyaw, 16, 11);
                    cur.hdg = fr_velyaw/10;
      
                    hdgGood=true;
                    #if defined Debug_All || defined Debug_Telemetry 
                      Debug.print(" 0x5005 Heading=");
                      Debug.println(cur.hdg,2);
                    #endif
                    break;   
               
                   
      }
}
#if defined Debug_All || defined DebugStatusFlags
void DisplayStatusFlags() {
  Debug.print("gpsGood="); Debug.println(gpsGood);
  Debug.print("Passthrough flag="); Debug.println(Passthrough);
  Debug.print("iNav flag="); Debug.println(iNav);
  Debug.print("hdopGood="); Debug.println(hdopGood);
  Debug.print("lonGood="); Debug.println(lonGood);
  Debug.print("latGood="); Debug.println(latGood); 
  Debug.print("altGood="); Debug.println(altGood);  
  Debug.print("hdgGood="); Debug.println(hdgGood);        
}
#endif
//***************************************************
  uint32_t bit32Extract(uint32_t dword,uint8_t displ, uint8_t lth) {
  uint32_t r = (dword & createMask(displ,(displ+lth-1))) >> displ;
//  Debug.print(" Result=");
 // Debug.println(r);
  return r;
}
uint32_t createMask(uint8_t lo, uint8_t hi) {
  uint32_t r = 0;
  for (unsigned i=lo; i<=hi; i++)
       r |= 1 << i;
//  Debug.print(" Mask 0x=");
//  Debug.println(r, HEX);      
  return r;
}

//***************************************************
void TestServos() {
PositionServos(90, 0, 90); 
delay(2000);

for (int i=1; i<=360; i++) {
  delay(60);
  PositionServos(i, 30, 90);   
  }
for (int i=1; i<=180; i++) {
  delay(60);
  PositionServos(90, i, 90);   
  }
  
PositionServos(90, 0, 90);   
 }
//***************************************************
void CheckForTimeouts() {
  unsigned long cMillis = millis();
    if ((gpsGood==1) && (cMillis - gpsMillis >= 5000)){
      gpsGood = 0;   // If no GPS packet for 5 seconds then GPS timeout 
      serGood = 0;
      #if defined Debug_All || defined Debug_Telemetry
        Debug.println("No GPS telemetry for 5 seconds"); 
      #endif  
    }
   ServiceTheStatusLed();
}
//***************************************************

void ServiceTheStatusLed() {
#ifdef Debug_LEDs
    Debug.print("gpsGood = ");
    Debug.print(gpsGood);
    Debug.print("   serGood = ");
    Debug.print(serGood);
    Debug.print("   homeInitialised = ");
    Debug.println(homeInitialised);
#endif
  if (gpsGood) {
    if (homeInitialised) 
      ledState = HIGH;
    else 
      BlinkLed(300);
    }
  else 
     if (serGood) 
       BlinkLed(1500);
     else
       ledState = LOW;
    digitalWrite(StatusLed, ledState);  
    digitalWrite(BoardLed, !ledState);
}

//***************************************************
void BlinkLed(int rate) {
  unsigned long cMillis = millis();
     if (cMillis - ledMillis >= rate) {    // blink period
        ledMillis = cMillis;
        if (ledState == LOW) {
          ledState = HIGH; }   
        else {
          ledState = LOW;  } 
      }
}


//***************************************************

uint32_t Unpack_uint32 (int posn){
  
    //  The number starts at byte "posn" of the received packet and is four bytes long.
    //  GPS payload fields are little-endian, i.e they need an end-to-end byte swap
    
   byte b1 = packetBuffer[posn+3];
   byte b2 = packetBuffer[posn+2];
   byte b3 = packetBuffer[posn+1];
   byte b4 = packetBuffer[posn]; 
   
   unsigned long highWord = b1 << 8 | b2;
   unsigned long lowWord  = b3 << 8 | b4;
    
    // Now combine the four bytes into an unsigned 32bit integer

   uint32_t myvar = highWord << 16 | lowWord;
   return myvar;
}
//***************************************************
int32_t Unpack_int32 (int posn){
  
    //  The number starts at byte "posn" of the received packet and is four bytes long.
    //  GPS payload fields are little-endian, i.e they need an end-to-end byte swap
    
   byte b1 = packetBuffer[posn+3];
   byte b2 = packetBuffer[posn+2];
   byte b3 = packetBuffer[posn+1];
   byte b4 = packetBuffer[posn]; 
   
   unsigned long highWord = b1 << 8 | b2;
   unsigned long lowWord  = b3 << 8 | b4;
   
 // Now combine the four bytes into an unsigned 32bit integer
 
   int32_t myvar = highWord << 16 | lowWord;
   return myvar;
}
//***************************************************
uint16_t Unpack_uint16 (int posn){
  
    //  The number starts at byte "posn" of the received packet and is two bytes long
    //  GPS payload fields are little-endian, i.e they need an end-to-end byte swap

   byte b1 = packetBuffer[posn+1];
   byte b2 = packetBuffer[posn];  
    
    // Now convert the 2 bytes into an unsigned 16bit integer
    
    uint16_t myvar = b1 << 8 | b2;
    return myvar;
}
//***************************************************
int16_t Unpack_int16 (int posn){
  
    //  The number starts at byte "posn" of the received packet and is two bytes long
    //  GPS payload fields are little-endian, i.e they need an end-to-end byte swap
   byte b1 = packetBuffer[posn+1];
   byte b2 = packetBuffer[posn];
    
    // Now convert the 2 bytes into a signed 16bit integer
    
    int16_t myvar = b1 << 8 | b2;
    return myvar;
}
//***************************************************
uint8_t Unpack_uint8 (int posn){
  
    //  The number starts at byte "posn" of the received packet and is one byte long

  byte b1 = packetBuffer[posn];
    
    // Now convert the byte into an unsigned 8 bit integer
    
   uint8_t myvar = b1;
   return myvar;
}
//***************************************************
#if defined Debug_All || defined Debug_SPort
void DisplayTheBuffer (int lth){
  for ( int i = 0; i < lth; i++ ) {
    byte b = packetBuffer[i];
    if (b<=0xf) Debug.print("0");
    Debug.print(b,HEX);
    Debug.print(" ");
  }
  Debug.println();
}
#endif
//***************************************************
#if defined Debug_All || defined Debug_SPort
void DisplayField (int pos, int lth){
  for ( int i = pos; i < pos+lth; i++ ) {
    Debug.print(packetBuffer[i],HEX);
    Debug.print(" ");
  }
  Debug.print("// ");
}
#endif
//***************************************************
String TimeString (unsigned long epoch){
 int hh = (epoch  % 86400L) / 3600;   // remove the days (86400 secs per day) and div the remainer to get hrs
 int mm = (epoch  % 3600) / 60;       // calculate the minutes (3600 secs per minute)
 int ss = (epoch % 60);               // calculate the seconds

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
//***************************************************

uint8_t Unpack8 (int posn){
  
    uint8_t myvar = packetBuffer[posn];
    return myvar;
}
//***************************************************
#if defined Debug_All  
void ShowElapsed() {
  Debug.print(" Seconds=");
  unsigned long millnow=millis();
  float fSecs = millnow / 1000;
  Debug.print(fSecs,1);
  Debug.print(" ");
}
#endif
//***************************************************
 
