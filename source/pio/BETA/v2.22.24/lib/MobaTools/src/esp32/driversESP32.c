// ESP32 HW-spcific Functions
#ifdef ARDUINO_ARCH_ESP32
// Logging
#include <Arduino.h>
#include "drivers.h"

// ------------------ SOFTLED & Servos ------------------------------------------------------
// different versions of ledc programmimng: ( if both are defined, a mixed version is selected - not yet implemented )
#ifdef LEDC_USE_SDK
    #define SDK_ACCESS  // Die SDK-Aufrufe sind NICHT Interruptfest - die SDK-Version darf daher nicht aktiviert werden
#else
    #define REGISTER_ACCESS
#endif

//===================================================================================
#if defined SDK_ACCESS //&& !defined REGISTER_ACCESS  // sdk version
// programming ledc hardware by means of sdk functions
// version with sdk calls
#include <driver/ledc.h>

//#warning "HW specfic drivers.c (using sdk) - ESP32  --"
#error "SDK-version must not be used"

// variant with using the sdk to configure ledc hardware
#define groupUsed(pwmNr)    ((pwmNr)/8)
#define channelUsed(pwmNr)  ((pwmNr)%8)

// There are max 16 pwm ouputs. These are used by servos and softleds
pwmUse_t pwmUse[16];

int8_t _findPwmNbr( ) {
    // find a free pwm Nbr and return it
    // value is negative if no free channel available
    int8_t pwmNbr;
    for( pwmNbr=15; pwmNbr>=0; pwmNbr-- ) {
        if ( !( pwmUse[pwmNbr].inUse ) ) {
            pwmUse[pwmNbr].inUse = 1;
            pwmUse[pwmNbr].group = groupUsed(pwmNbr);
            pwmUse[pwmNbr].channel = channelUsed(pwmNbr);
            break;
        }
    }
    return pwmNbr;
}

int8_t freePwmNbr( uint8_t pwmNbr ) {
    if ( pwmNbr > 15 ) return false;
    setPwmDuty( pwmNbr, 0 );
    ledc_stop(pwmUse[pwmNbr].group, pwmUse[pwmNbr].channel,0); 
    pwmUse[pwmNbr].inUse = 0;
    return true;
}

int8_t initPwmChannel( uint8_t pin, uint8_t timer ) {
    ledc_channel_config_t channelConfig;
    ledc_timer_config_t  timerConfig;
    int8_t pwmNbr = _findPwmNbr();
    
    if ( pwmNbr >= 0 ) {
        // found a free channel
        // set timerconfig values
        timerConfig.speed_mode = pwmUse[pwmNbr].group;
        timerConfig.duty_resolution = LEDC_BITS;
        timerConfig.timer_num = timer;
        timerConfig.freq_hz = (timer==SERVO_TIMER?SERVO_FREQ:SOFTLED_FREQ);
        ledc_timer_config( &timerConfig ) ;
        //set channel configuration values
        channelConfig.gpio_num = pin;
        channelConfig.speed_mode = pwmUse[pwmNbr].group;
        channelConfig.channel = pwmUse[pwmNbr].channel;
        channelConfig.intr_type = 0;  // no fade interrupt
        channelConfig.timer_sel = timer;
        channelConfig.duty = 0;
        channelConfig.hpoint = 0;
        ledc_channel_config( &channelConfig );
        pwmUse[pwmNbr].timer = timer;
        pwmUse[pwmNbr].pin = pin;
    }
    return pwmNbr;
}

void setPwmPin( uint8_t pwmNbr ) {
    // wird im SDK-Aufruf 'ledc_channel_config' gemacht
    //pinMode(pwmUse[pwmNbr].pin, OUTPUT);
    //pinMatrixOutAttach(pwmUse[pwmNbr].pin, ((pwmUse[pwmNbr].group)?LEDC_LS_SIG_OUT0_IDX:LEDC_HS_SIG_OUT0_IDX) + pwmUse[pwmNbr].channel, false, false);
}
    
void IRAM_ATTR setPwmDuty(int8_t pwmNbr, uint32_t duty ){
      ledc_set_duty_with_hpoint(pwmUse[pwmNbr].group, pwmUse[pwmNbr].channel, duty, 0) ;
      ledc_update_duty(pwmUse[pwmNbr].group, pwmUse[pwmNbr].channel) ;
      //ledc_set_duty_and_update(pwmUse[pwmNbr].group, pwmUse[pwmNbr].channel, duty, 0);
}

#endif // end of sdk-version ===========================================================
//=======================================================================================
#if defined REGISTER_ACCESS && !defined SDK_ACCESS  // register version
// programming ledc hardware with direct register access

//#define CONFIG_DISABLE_HAL_LOCKS

#include "esp32-hal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "rom/ets_sys.h"
#include "esp32-hal-matrix.h"
#include "soc/dport_reg.h"
#include "soc/ledc_reg.h"
#include "soc/ledc_struct.h"

//#pragma message "HW specfic drivers.c (direct reg access) - ESP32 --"
// variant with direct acces to ledc PWM register
// APB_CLK must be 80MHz

#define CONFIG_DISABLE_HAL_LOCKS
#ifdef CONFIG_DISABLE_HAL_LOCKS
#define LEDC_MUTEX_LOCK()
#define LEDC_MUTEX_UNLOCK()
#else
#define LEDC_MUTEX_LOCK()    do {} while (xSemaphoreTake(_ledc_sys_lock, portMAX_DELAY) != pdPASS)
#define LEDC_MUTEX_UNLOCK()  xSemaphoreGive(_ledc_sys_lock)
xSemaphoreHandle _ledc_sys_lock;
#endif

#define LEDC_CHAN(g,c) LEDC.channel_group[(g)].channel[(c)]
#define LEDC_TIMER(g,t) LEDC.timer_group[(g)].timer[(t)]

//static uint16_t ledcUsed =0 ; // Bit field to mark used pwm channels
#define groupUsed(pwmNr)    ((pwmNr)/8)
#define channelUsed(pwmNr)  ((pwmNr)%8)


#undef ARDUHAL_LOG_LEVEL
# define ARDUHAL_LOG_LEVEL (5)

// There are max 16 pwm ouputs. These are used by servos and softleds

pwmUse_t pwmUse[16];

static int8_t _findPwmNbr( ) {
    // find a free pwm Nbr and return it
    // value is negative if no free channel available
    int8_t pwmNbr;
    for( pwmNbr=15; pwmNbr>=0; pwmNbr-- ) {
        if ( !( pwmUse[pwmNbr].inUse ) ) {
            pwmUse[pwmNbr].inUse = 1;
            pwmUse[pwmNbr].group = groupUsed(pwmNbr);
            pwmUse[pwmNbr].channel = channelUsed(pwmNbr);
            break;
        }
    }
    return pwmNbr;
}

static void _initLedcHw(int8_t pwmNbr) {
    // look if ledc clock is enabled, and ledc HW is running
    uint8_t group = pwmUse[pwmNbr].group;
    uint8_t timer = pwmUse[pwmNbr].timer;
    uint8_t channel = pwmUse[pwmNbr].channel;
    // timer 3 für Leds (100Hz) timer 2 für Servos (50Hz)
    uint8_t freq  = timer==3?100:50;
    uint64_t clk_freq = getApbFrequency();
    clk_freq <<= 8;//div_num is fixpoint with 8 bit fractional
    uint32_t div_num = (clk_freq >> LEDC_BITS) / freq;

    if(!(DPORT_GET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_LEDC_CLK_EN)) ||
       (DPORT_GET_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_LEDC_RST) ) )  {
        // LEC HW in reset or no clock enabled
        DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_LEDC_CLK_EN);
        DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_LEDC_RST);
        LEDC.conf.apb_clk_sel = 1;//LS use apb clock ( 80MHz )
        #ifndef CONFIG_DISABLE_HAL_LOCKS
            _ledc_sys_lock = xSemaphoreCreateMutex();
        #endif
    }
    // initialize the timer
    LEDC_MUTEX_LOCK();
    LEDC_TIMER(group, timer).conf.clock_divider = div_num;      //18 bit (10.8) This register is used to configure parameter for divider in timer the least significant eight bits represent the decimal part.
    LEDC_TIMER(group, timer).conf.duty_resolution = LEDC_BITS;  //5 bit (0...20) This register controls the range of the counter in timer. the counter range is [0 2**bit_num] the max bit width for counter is 20.
    LEDC_TIMER(group, timer).conf.tick_sel = true;              //apb clock
    if(group) {
        LEDC_TIMER(group, timer).conf.low_speed_update = 1;     //This bit is only useful for low speed timer channels, reserved for high speed timers
    }
    LEDC_TIMER(group, timer).conf.pause = 0;
    LEDC_TIMER(group, timer).conf.rst = 1;//This bit is used to reset timer the counter will be 0 after reset.
    LEDC_TIMER(group, timer).conf.rst = 0;
    // apb_clk must not be changed after initializing pwm hardware
    //uint32_t iarg = chan;
    //addApbChangeCallback((void*)iarg, _on_apb_change);
    
    //initialize the channel
    LEDC_CHAN(group, channel).conf0.timer_sel = timer;//2 bit Selects the timer to attach 0-3
    LEDC_CHAN(group, channel).conf0.idle_lv = 0;//1 bit This bit is used to control the output value when channel is off.
    LEDC_CHAN(group, channel).hpoint.hpoint = 0;//20 bit The output value changes to high when timer selected by channel has reached hpoint
    LEDC_CHAN(group, channel).conf1.duty_inc = 1;//1 bit This register is used to increase the duty of output signal or decrease the duty of output signal for high speed channel
    LEDC_CHAN(group, channel).conf1.duty_num = 1;//10 bit This register is used to control the number of increased or decreased times for channel
    LEDC_CHAN(group, channel).conf1.duty_cycle = 1;//10 bit This register is used to increase or decrease the duty every duty_cycle cycles for channel
    LEDC_CHAN(group, channel).conf1.duty_scale = 0;//10 bit This register controls the increase or decrease step scale for channel.
    LEDC_CHAN(group, channel).duty.duty = 0;
    LEDC_CHAN(group, channel).conf0.sig_out_en = 0;//This is the output enable control bit for channel
    LEDC_CHAN(group, channel).conf1.duty_start = 0;//When duty_num duty_cycle and duty_scale has been configured. these register won't take effect until set duty_start. this bit is automatically cleared by hardware.
    if(group) {
        LEDC_CHAN(group, channel).conf0.low_speed_update = 1;
    } else {
        LEDC_CHAN(group, channel).conf0.clk_en = 0;
    }
    LEDC_MUTEX_UNLOCK();
}
int8_t freePwmNbr( uint8_t pwmNbr ) {
    if ( pwmNbr > 15 ) return false;
    setPwmDuty( pwmNbr, 0 );
    pinMatrixOutDetach(pwmUse[pwmNbr].pin, false, false);
    pwmUse[pwmNbr].inUse = 0;
    return true;
}

int8_t initPwmChannel( uint8_t pin, uint8_t timer ) {
    int8_t pwmNbr = _findPwmNbr();
    if ( pwmNbr >= 0 ) {
        // found a free channel
        pwmUse[pwmNbr].timer = timer;
        pwmUse[pwmNbr].pin = pin;
        _initLedcHw(pwmNbr );
    }
    return pwmNbr;
}

void setPwmPin( uint8_t pwmNbr ) {
    uint8_t group = pwmUse[pwmNbr].group;
    uint8_t pin = pwmUse[pwmNbr].pin;
    uint8_t channel = pwmUse[pwmNbr].channel;
    pinMode(pin, OUTPUT);
    pinMatrixOutAttach(pin, (group?LEDC_LS_SIG_OUT0_IDX:LEDC_HS_SIG_OUT0_IDX) + channel, false, false);
}
    
void IRAM_ATTR setPwmDuty(int8_t pwmNbr, uint32_t duty ){
    uint8_t group = pwmUse[pwmNbr].group;
    uint8_t channel = pwmUse[pwmNbr].channel;
    LEDC_MUTEX_LOCK();
    LEDC_CHAN(group, channel).duty.duty = (duty << 4);//25 bit (21.4)
    LEDC_CHAN(group, channel).hpoint.hpoint = 1000;
    if(duty) {
        LEDC_CHAN(group, channel).conf0.sig_out_en = 1;//This is the output enable control bit for channel
        LEDC_CHAN(group, channel).conf1.duty_start = 1;//When duty_num duty_cycle and duty_scale has been configured. these register won't take effect until set duty_start. this bit is automatically cleared by hardware.
        //if(group) {
           LEDC_CHAN(group, channel).conf0.low_speed_update = 1;
        //} else {
           //LEDC_CHAN(group, channel).conf0.clk_en = 1;
        //}
    } else {
        LEDC_CHAN(group, channel).conf0.sig_out_en = 0;//This is the output enable control bit for channel
        LEDC_CHAN(group, channel).conf1.duty_start = 0;//When duty_num duty_cycle and duty_scale has been configured. these register won't take effect until set duty_start. this bit is automatically cleared by hardware.
        if(group) {
            LEDC_CHAN(group, channel).conf0.low_speed_update = 1;
        } else {
            LEDC_CHAN(group, channel).conf0.clk_en = 0;
        }
    }
    LEDC_MUTEX_UNLOCK();
}
#endif //======== end of register access version ======================================================
//======================================================================================================
#if defined REGISTER_ACCESS && defined SDK_ACCESS   // mixed version
//  mixed version: sdk calls and direkt register access (experimental )
#endif //======== end of mixed version
//=======================================================================================================

#endif // End of file