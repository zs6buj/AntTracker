//=====================================================
void moveMotors(uint16_t az, uint16_t el) 
{
   #if (defined DEBUG_MOTORS)
    log.printf("moveMotors() az=%u  el=%u\n", az, el);
  #endif

  // Flip and Mirror if necessary
  #if not defined AZ_SERVO_360                // If 180 deg az servo
    if ( (az > 180) && (az <= 360) ) 
    {        // Pointing direction is behind us,
      az -= 180;                              // so flip the frame of reference over and mirror it 
      el = 180 - el;                          // and make the el servo reach over and behind
      #if (defined DEBUG_MOTORS)
        log.printf("Flipped az:%d el:%d \n", az, el);  
      #endif            
    }
  #endif 

  bool az_reversed = false;
  bool el_reversed = false;   
  
  // Reverse servo action if necessary
  #if defined AZ_SERVO_360   
    #if defined REVERSEAZIMUTH    
      az = 360 - az;
      az_reversed = true;    
    #endif 
    #if defined REVERSEELEVATION    
      el = 360 - el;
      el_reversed = true;   
    #endif      
  #else
    #if defined REVERSEAZIMUTH    
      az = 180 - az;
      az_reversed = true;   
    #endif 
    #if defined REVERSEELEVATION    
      el = 180 - el;
      el_reversed = true;     
    #endif     
  #endif

  // here we move the servos and hence the antenna
  // note: myservo.attach(pin, 1000, 2000)

  #if (defined DEBUG_MOTORS) 
    if (az_reversed) log.print("az reversed ");
    if (el_reversed) log.print("el reversed ");
    log.printf("Servo write az=%u  el=%u\n", az, el);
  #endif

  #if defined SERVOS
    azServo.write(az);       // async non-blocking
    elServo.write(el);       // async non-blocking
  #endif
  #if defined STEPPERS
    azStepper.write(az, 1);  // async non-blocking
    elStepper.write(el, 1);  // async non-blocking
  #endif
}
//=======================================================
void pointMotors(int16_t worldAz, int16_t ourEl, int16_t boxHdg) {
  // remap our inner azimuth to world azimuth, then move servos
  if (worldAz == 360) worldAz = 0;   // our servo azimuth working range is 0 deg through 359 deg

  #if defined DEBUG_ALL || defined DEBUG_MOTORS
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
 
  #if defined DEBUG_ALL || defined DEBUG_MOTORS
    log.print("ourAz = " );
    log.print(ourAz);
    log.print(" ourEl = ");
    log.print(ourEl);
    log.print(" boxHdg = " );
    log.println(boxHdg);
  #endif

  moveMotors(ourAz, ourEl);
} 
//=====================================================
void waitForComplete()
{   
  #if defined SERVOS
    while ((azServo.moving()) || (elServo.moving())) {delay(100);};
  #endif
  #if defined STEPPERS
    while ((azStepper.moving()) || (elStepper.moving())) {delay(100);};
  #endif
}
//=====================================================
void testMotors() {
  delay(1500);
  moveMotors(0, 0);  // az, el asynch no block
  waitForComplete();
  delay(1500);
  moveMotors(90, 0);
  delay(1500);
  waitForComplete();
  delay(1500);
  moveMotors(90, 90);
  waitForComplete();
  delay(1500); 
  moveMotors(90, 180);
  waitForComplete();
  delay(1500);
  moveMotors(90, 0);
  waitForComplete();  
  delay(1500);
  moveMotors(180, 0);
  waitForComplete();  
  delay(1500);
  moveMotors(azStart, elStart);
 /*
  log.println("Now rotating box heading every 45 degrees"); 
  delay(2000);   

  for (int t_hdg = 0; t_hdg < 360 ; t_hdg += 45) {
    int t_el = 0;
    log.printf("box_hdg=%u\n", t_hdg);      
    for (int t_az = 0 ; t_az <= 360 ; t_az++) {
      pointMotors(t_az, t_el, t_hdg);   // az, el, hdg
      //log.printf("test   t_az=%u  t_el=%u    box_hdg=%u\n", T_az, t_el, t_hdg);   
    }
    delay(2000);
  } 
 */
}
