#include <MobaTools.h>
/* Demo: Non blocking time delays ( without delay command )
 *  In principle, the 'MoToTimer' works like a kitchen timer.
 *  You wind it up to a certain time and then it runs back to 0. 
 *  Unlike the kitchen alarm clock, however, it does not ring.
 *  You have to check cyclically to see if the time has expired. But that fits
 *  perfectly with the principle of the 'loop', i.e. a cyclical query in an endless loop.
 *  Function calls:
 *  MoToTimer.setTime( long Runtime ); sets the time in ms
 *  bool = MoToTimer.running(); == true as long as the time is still running, 
 *                                    
 *  In contrast to the procedure with delay(), this allows for several
 *  independent and asynchronous cycle times.
 *  In this demo 2 leds are flashing with different clock rates
 *  You can add additional tasks ad the end of loop without disturbing
 *  the flashing ( as long as this tasks are not blocking too ;-) )
 */
const int led1P =  5; 
const int led2P =  6; 

MoToTimer timer1;
MoToTimer timer2;

void setup() {
    pinMode(led1P, OUTPUT); 
    pinMode(led2P, OUTPUT);
}

void loop() {
    // -------- flashing of the first led ------------------
    // this led flashes with a non-symmetrical clock ratio
	if ( timer1.running()== false ) {
        // flash time expired, toggle output and
        // start timer again
        if ( digitalRead( led1P ) == HIGH ) {
            digitalWrite( led1P, LOW );
            timer1.setTime( 600 );
       } else {
            digitalWrite( led1P, HIGH );
            timer1.setTime( 300 );
        }
	}
   
    // -------- flashing of the second led ------------------
    // this led flashes with a symmetrical clock ratio
    if ( timer2.running() == false ) {
        // flash time expired, toggle output and
        // start timer again
        if ( digitalRead( led2P ) == HIGH ) {
            digitalWrite( led2P, LOW);
        } else {
            digitalWrite( led2P, HIGH);
        }
        timer2.setTime( 633 );
    }
}
