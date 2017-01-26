/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <time.h>
static double sleep_duration(double duration){
    timespec req,rem;
    int seconds = duration;
    req.tv_nsec = (duration - (double)seconds) * 1.0e+9;
    req.tv_sec = seconds;
    nanosleep(&req,&rem);
    return 0;//( ( (double) rem.tv_nsec / 1.0e+9 ) + (double) rem.tv_sec);
}

namespace Pacer{
extern void init( );
extern double step(double dt = 0);
}

#include <cstdlib>


int main(int argc, char* argv[])
{
  Pacer::init();

    // Hz Frequency
    int freq = atoi(argv[2]);
    const double seconds_per_message = 1.0 / (double) freq;
  
  float max_time = atof(argv[1]);
  if(max_time < 0)
    max_time = INFINTY;
  float time = 0;
  while (time < max_time) {
    time = Pacer::step(seconds_per_message);
    double remaining = sleep_duration(seconds_per_message);
  }

  return 0;
}
