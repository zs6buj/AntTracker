//=====================================================
void moveServos(uint16_t az, uint16_t el) 
{
   #if (defined DEBUG_Servos)
    log.printf("moveServos() az=%u  el=%u\n", az, el);
  #endif

  // Flip and Mirror if necessary
  #if not defined Az_Servo_360                // If 180 deg az servo
    if ( (az > 180) && (az <= 360) ) 
    {        // Pointing direction is behind us,
      az -= 180;                              // so flip the frame of reference over and mirror it 
      el = 180 - el;                          // and make the el servo reach over and behind
      #if (defined DEBUG_Servos)
        log.printf("Flipped az:%d el:%d \n", az, el);  
      #endif            
    }
  #endif 

  bool az_reversed = false;
  bool el_reversed = false;   
  
  // Reverse servo action if necessary
  #if defined Az_Servo_360   
    #if defined ReverseAzimuth    
      az = 360 - az;
      az_reversed = true;    
    #endif 
    #if defined ReverseElevation    
      el = 360 - el;
      el_reversed = true;   
    #endif      
  #else
    #if defined ReverseAzimuth    
      az = 180 - az;
      az_reversed = true;   
    #endif 
    #if defined ReverseElevation    
      el = 180 - el;
      el_reversed = true;     
    #endif     
  #endif

  // here we move the servos and hence the antenna
  // note: myservo.attach(pin, 1000, 2000)

  #if (defined DEBUG_Servos) 
    if (az_reversed) log.print("az reversed ");
    if (el_reversed) log.print("el reversed ");
    log.printf("Servo write az=%u  el=%u\n", az, el);
  #endif
  
  #if (Servo_Slowdown == 0 )
    azServo.write(az);
    elServo.write(el);
  #else
    static uint16_t az_prev = 0;
    static uint16_t el_prev = 0;
    
    //log.printf(">>   az=%u  el=%u    az_prev=%u  el_prev=%u \n", az, el, az_prev, el_prev);
    if (az < az_prev) 
    {
      for (int deg = az_prev ; deg >= az ; deg--) {
        azServo.write(deg);
        delay(Servo_Slowdown); 
        // ESP will yield to rtos for wifi & bt work 
        //log.printf(">>   az=%u  az_prev=%u    deg=%u \n", az, az_prev, deg);    
      }
    } else 
    {
      for (int deg = az_prev ; deg <= az ; deg++) {
        azServo.write(deg);
        delay(Servo_Slowdown); 
        //log.printf(">>   az=%u  az_prev=%u    deg=%u \n", az, az_prev, deg);               
      }   
    }
     
    if (el < el_prev)  {
      for (int deg = el_prev ; deg >= el ; deg--) {
        elServo.write(deg);
        delay(Servo_Slowdown);
        //log.printf(">>   el=%u  el_prev=%u    deg=%u \n", el, el_prev, deg);                  
      }   
    } else {
      for (int deg = el_prev ; deg <= el ; deg++) {
        elServo.write(deg);
        delay(Servo_Slowdown);  
        //log.printf(">>   el=%u  el_prev=%u    deg=%u \n", el, el_prev, deg);               
      }     
    }
    az_prev = az;
    el_prev = el; 
  #endif    
}
//=======================================================
void pointServos(int16_t worldAz, int16_t ourEl, int16_t boxHdg) {
 
  if (worldAz == 360) worldAz = 0;   // our servo azimuth working range is 0 deg through 359 deg

  #if defined DEBUG_All || defined DEBUG_Servos
    log.print("\nworldAz = " );
    log.print(worldAz);
    log.print("\t ourEl = ");
    log.print(ourEl);
    log.print("\t boxHdg = " );
    log.println(boxHdg);
  #endif 
  
  // Remap tracker-box azimuth to world-compass azimuth
  int16_t ourAz = worldAz - boxHdg + azStart;    // here we compensate for offset of the boxview to worldview
  ourAz = wrap360(ourAz);
 
  #if defined DEBUG_All || defined DEBUG_Servos
    log.print("ourAz = " );
    log.print(ourAz);
    log.print(" ourEl = ");
    log.print(ourEl);
    log.print(" boxHdg = " );
    log.println(boxHdg);
  #endif

  moveServos(ourAz, ourEl);
}
 
//=====================================================
void TestServos() {
  delay(1500);
  moveServos(0, 0);  // az, el
  delay(1500); 
  moveServos(90, 0);
  delay(1500); 
  moveServos(90, 90);
  delay(1500);
  moveServos(90, 180);
  delay(1500);
  moveServos(90, 0);
  delay(1500);      
  moveServos(180, 0);
  delay(1500);  
  moveServos(azStart, elStart);

 /*
  log.println("Now rotating box heading every 45 degrees"); 
  delay(2000);   

  for (int t_hdg = 0; t_hdg < 360 ; t_hdg += 45) {
    int t_el = 0;
    log.printf("box_hdg=%u\n", t_hdg);      
    for (int t_az = 0 ; t_az <= 360 ; t_az++) {
      if (Servo_Slowdown == 0) {
        delay(10);
      }
      pointServos(t_az, t_el, t_hdg);   // az, el, hdg
      //log.printf("test   t_az=%u  t_el=%u    box_hdg=%u\n", T_az, t_el, t_hdg);   
    }
    delay(2000);
  } 
 */
  
  moveServos(azStart, elStart); 

}
