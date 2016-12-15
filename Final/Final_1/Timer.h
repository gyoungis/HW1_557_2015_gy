//////////////////////////////////////////////////////////////////////////////
// Timer.h
//////////////////////////////////////////////////////////////////////////////

#ifndef TIMER_H
#define TIMER_H

#include <windows.h>

class Timer
{
public:
    Timer();                                    // default constructor
    ~Timer() {}                                 

    void   Start();                             // start timer
    void   Stop();                              // stop the timer
    double GetElapsedTime();                    // get elapsed time in second
    double GetElapsedTimeInSec();               // get elapsed time in second (same as getElapsedTime)
    double GetElapsedTimeInMilliSec();          // get elapsed time in milli-second
    double GetElapsedTimeInMicroSec();          // get elapsed time in micro-second


private:
    double startTimeInMicroSec;                 // starting time in micro-second
    double endTimeInMicroSec;                   // ending time in micro-second
    int    stopped;                             // stop flag 

    LARGE_INTEGER frequency;                    // ticks per second
    LARGE_INTEGER startCount;                   //
    LARGE_INTEGER endCount;                     //
};

#endif // TIMER_H
