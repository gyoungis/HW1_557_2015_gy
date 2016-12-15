#include "Timer.h"
#include <stdlib.h>

Timer::Timer()
{
    QueryPerformanceFrequency(&frequency);
    startCount.QuadPart = 0;
    endCount.QuadPart = 0;

    stopped = 0;
    startTimeInMicroSec = 0;
    endTimeInMicroSec = 0;
}

void Timer::Start()
{
    stopped = 0; 
    QueryPerformanceCounter(&startCount);
}

void Timer::Stop()
{
    stopped = 1; 
    QueryPerformanceCounter(&endCount);
}

double Timer::GetElapsedTimeInMicroSec()
{
    if(!stopped)
        QueryPerformanceCounter(&endCount);

    startTimeInMicroSec = startCount.QuadPart * (1000000.0 / frequency.QuadPart);
    endTimeInMicroSec = endCount.QuadPart * (1000000.0 / frequency.QuadPart);

    return endTimeInMicroSec - startTimeInMicroSec;
}

double Timer::GetElapsedTimeInMilliSec()
{
    return this->GetElapsedTimeInMicroSec() * 0.001;
}

double Timer::GetElapsedTimeInSec()
{
    return this->GetElapsedTimeInMicroSec() * 0.000001;
}

double Timer::GetElapsedTime()
{
    return this->GetElapsedTimeInMilliSec();
}
