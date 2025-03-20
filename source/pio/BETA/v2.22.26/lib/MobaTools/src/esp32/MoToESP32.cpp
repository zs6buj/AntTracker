// ESP32 HW-spcific Functions
#ifdef ARDUINO_ARCH_ESP32
#include <MobaTools.h>
#define debugTP
//#define debugPrint
#include <utilities/MoToDbg.h>

//#warning "HW specfic - ESP32 ---"

bool spiInitialized = false;
void IRAM_ATTR stepperISR(int32_t cyclesLastIRQ)  __attribute__ ((weak));
nextCycle_t nextCycle;
static nextCycle_t cyclesLastIRQ = 1;  // cycles since last IRQ

void IRAM_ATTR ISR_Stepper(void) {
    static uint64_t lastAlarm, aktAlarm;
    // Timer running up, used for stepper motor. No reload of timer
    SET_TP1;
    nextCycle = ISR_IDLETIME  / CYCLETIME ;// min ist one cycle per IDLETIME
    portENTER_CRITICAL_ISR(&stepperMux);
    cyclesLastIRQ = (aktAlarm - lastAlarm) / TICS_PER_MICROSECOND;
    if ( stepperISR ) stepperISR(cyclesLastIRQ);
	// next alarm ISR must be at least MIN_STEP_CYCLE/2 beyond last alarm value ( time between to ISR's )
    lastAlarm = aktAlarm;
    aktAlarm = lastAlarm+(nextCycle*TICS_PER_MICROSECOND); // minimumtime until next Interrupt
    uint64_t minNextAlarm = lastAlarm + (MIN_STEP_CYCLE*TICS_PER_MICROSECOND/2);
	if ( aktAlarm < minNextAlarm ) {
		// time till next ISR ist too short, set to mintime and adjust nextCycle
        CLR_TP1;
		aktAlarm =  minNextAlarm;
	}
	#if (ESP_ARDUINO_VERSION_MAJOR == 2)
     timerAlarmWrite(stepTimer, aktAlarm , false); // no autorelaod
     timerAlarmEnable(stepTimer);
	#elif (ESP_ARDUINO_VERSION_MAJOR == 3)
    //3.0.3 void timerAlarm(hw_timer_t * timer, uint64_t alarm_value, bool autoreload, uint64_t reload_count);
    timerAlarm(stepTimer, aktAlarm , false, 0); // no autorelaod, 0=unlimited - zs6buj
	#else
		 #error "ESP-core version unsupported"
    #endif
    SET_TP1;
    portEXIT_CRITICAL_ISR(&stepperMux);
    CLR_TP1; // Oszimessung Dauer der ISR-Routine
}
////////////////////////////////////////////////////////////////////////////////////////////
timerConfig_t timerConfig;
portMUX_TYPE stepperMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE servoMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE softledMux = portMUX_INITIALIZER_UNLOCKED;
hw_timer_t * stepTimer = NULL;

void seizeTimerAS() {
static bool timerInitialized = false;
    // Initiieren des Stepper Timers ------------------------
    if ( !timerInitialized ) {
		#if (ESP_ARDUINO_VERSION_MAJOR == 2)
			#pragma message "Info: using esp core 2.x.x"
        stepTimer = timerBegin(STEPPER_TIMER, DIVIDER, true); // true= countup
        timerAttachInterrupt(stepTimer, &ISR_Stepper, true);  // true= edge Interrupt
        timerAlarmWrite(stepTimer, ISR_IDLETIME*TICS_PER_MICROSECOND , false); // false = no autoreload );
        timerAlarmEnable(stepTimer);
		#elif (ESP_ARDUINO_VERSION_MAJOR == 3)
			#pragma message "Info: using esp core 3.x.x"
        // core 3.0.3 hw_timer_t * timerBegin(uint32_t frequency);   // frequency in Hz    
		stepTimer = timerBegin(2000000);  // frequency
        // core 3.0.3void timerAttachInterrupt(hw_timer_t * timer, void (*userFunc)(void));
        timerAttachInterrupt(stepTimer, &ISR_Stepper); // assume edge - zs6buj
        timerAlarm(stepTimer, ISR_IDLETIME*TICS_PER_MICROSECOND , false, 0); // false = no autoreload );
		#else
		 #error "ESP-core version unsupported"
		#endif
			
        timerInitialized = true;  
        MODE_TP1;   // set debug-pins to Output
        MODE_TP2;
        MODE_TP3;
        MODE_TP4;
    }
}


void enableServoIsrAS() {
}


void enableSoftLedIsrAS() {

}


#endif
