/* MoToSyncStepper
** class to syncronise the movement of up to MAXSTEPPER steppers
** The speed of each stepper is set in such a way, that alle steppers
** reach their target nearly at the same time
** IMPORTANT: rampLen is set to 0 when moving the steppers!!
** During the synced movement no changes to the stepper objects are allowed.
**
** The functionalitiy is derived from the MultiStepper class of AccelStepper
**
*/

#pragma once

class MoToSyncStepper
{
public:
    MoToSyncStepper();

    boolean addStepper(MoToStepper& stepper);  	// Add a stepper to group of synced steppers
	void setMaxSpeedSteps( uintxx_t speed10 );		// fastet speed of the motors ( in steps/10sec )
    void moveTo(long absolute[]);  				// define the target positions of all steppers
												// in group and start the move
												
	void setTargets(long absolute[]); 			// define targets (and set stepper speeds) , 
												// but don't start movement
	void startSyncMove();			// start the move and wait until it is finished
	
	bool moving();				    // true until all steppers reached their targeet.
     
private:
    // Array of pointers to the steppers we are controlling.
    // Fills from 0 onwards
    MoToStepper* _steppers[MAX_STEPPER];

    // Number of steppers we are controlling and the number
    // of steppers in _steppers[]
    uint8_t		_num_steppers;
	uintxx_t 	_maxSpeed;		// speed of the stepper with longest distance
   long *_targets;      // pointer to the targets			
};
