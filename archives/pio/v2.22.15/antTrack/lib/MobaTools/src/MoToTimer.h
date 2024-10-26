////////////////////////////////////////////////////////////////////////////
// MoToTimer.h - part of MobaTools
//
// Class MoToTimer - Timer management for time delays in the loop
/*
    MoToTimer myTimer()                 // Create Timerobject
    void setTime( unsigned long value ) // Start Timer with new time value
    void restart()                      // Restart Timer with last set time value ( setTime )
    bool running()                      // True while timer is running
    bool expired()                      // Only 'true' at the first call after timer expiry.
    void stop()                         // Stopping the timer prematurely
    unsigned long getRemain()           // Get remaining time if timer is running, 0 otherwise
    unsigned long getElapsed()          // Get elapsed time if timer is running, last value of setTime otherwise
    unsigned long getRuntime()          // Get last value of setTime
*/
// Class MoToTimebase - create regular Events in fixed time distance
/*
    MoToTimebase myTimerbase()          // Create Timerobject
    void setBasetime( long value )       // set intervall time ( timer is not started if negative )
    void tick( )                        // true if intervalltime elapsed ( must be called regularly to start event )
    void stop()                         // stop creating ticks ( tick() will always return false )
    void start()                        // Start creating ticks if a time is set, but it is not running
    bool running()                      // True if timebase is running
    bool inactive()                     // true if no intervall time is set
*/
/*
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#ifndef MOTOTIMER_H
#define MOTOTIMER_H

class MoToTimebase
{   // create regular Events in fixed time distance
  private:
    long _interval;             //time base time im ms ( <=0 menas not running )
    unsigned long _lastTick;     // Time of last tick
    
  public:  
    MoToTimebase() {
        _interval = 0;
    }
    
    void setBasetime( long baseTime ) {
        // if time baseTime is negativ, than time is set, but time base is not started and
        // can be started later with 'start' method
        _interval = baseTime;
        _lastTick = millis();
    }
    
    bool tick() {
        bool flag = false;
        if ( _interval>0 ) {
            // To be on the safe side, if tick is called too rarely...
            while ( millis() - _lastTick >= (unsigned long) _interval ) {
                _lastTick += _interval;
                flag = true;
            }
        }
        return flag;
    }
    
    void stop() {
        // Stop time base, if it is running
        if ( _interval > 0 ) _interval = -_interval;
    }
    
    void start() {
        // Start time base if a time is set, but it is not running
        if ( _interval < 0 ) {
            _interval = -_interval;
            _lastTick = millis();
        }
    }
    
    bool running() {
        // return true if time base is running
        return ( _interval > 0 );
    }
    
    bool inactive() {
        // return true if time base is not active
        return ( _interval == 0 );
    }
};


class MoToTimer
{
  private:
    static constexpr byte RUNNING = 0b1;
    static constexpr byte NOTEXPIRED = 0b10;
    byte active;    // Bit0: Timer is running, Bit 1: not expired Flag
    unsigned long startTime;
    unsigned long runTime;
    
  public:
    MoToTimer() {
        active = 0;
        startTime = 0;
        runTime = 0;
    }

    void setTime(  unsigned long wert ) {
        runTime = wert;
        if ( runTime > 0 ) {
            startTime =  millis();
            active = RUNNING | NOTEXPIRED; // set running and !expired flag
        } else {
            stop();
        }
    }

    unsigned long getRuntime() { return runTime; }
    
    bool running() {
        if ( active & RUNNING ) active &= ~RUNNING | ( millis() - startTime < runTime );
        return active & RUNNING;
    }

    bool expired() { // event 'timer expired'
        // this event is cleared after call of this method
        if ( running() || active == 0 ) return false;
        else active = 0;
        return true;
    }

    void stop() { active = 0; }
    
    void restart() { setTime( runTime ); }
    
    unsigned long getElapsed() {
        // return elapsed time
        if ( running() ) return ( millis() - startTime )+1;
        else return runTime;
    }
    
    unsigned long getRemain() {
        // return remaining time
        if ( running() ) return runTime - ( millis() - startTime );
        else return 0;
    }
    unsigned long getTime() { return getRemain(); }
    
};

class MoToTimerRop // Ram optimized version
{
  private:
    static constexpr byte RUNNING = 0b1;
    static constexpr byte NOTEXPIRED = 0b10;
    byte active;    // Bit0: Timer is running, Bit 1: not expired Flag
    long endtime;
    
  public:
    MoToTimerRop() {
        active = 0;
    }

    void setTime(  long wert ) {
        endtime =  (long) millis() + ( (long)wert>0?wert:1 );
        active = RUNNING | NOTEXPIRED; // set running and !expired flag
    }

    bool running() {
        if ( active & RUNNING ) active &= ~RUNNING | ( endtime - (long)millis() > 0 );
        return active & RUNNING;
    }

    bool expired() { // event 'timer expired'
        // this event is cleared after call of this method
        if ( running() || active == 0 ) return false;
        else active = 0;
        return true;
    }

    void stop() { active = 0; }

    long getTime() {
        // return remaining time
        if ( running() ) return endtime - (long)millis();
        else return 0;
    }
};

#endif
