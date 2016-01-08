/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
/////////////////////////////////////////////////////////////////////
///////////////////////// MOTOR /////////////////////////////////////
/////////////////////////////////////////////////////////////////////

#include <math.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#include <ServoController/ServoDriver.h>

#include <boost/assign/list_of.hpp>
#include <boost/assign/std/vector.hpp>

/*  Server Controller test file
 *  Behavior: Servos with ids \in {1..12} will oscilate within 1/10 of their maximum range
 *            3 full periods of sinusoidal oscilation will occur before exit.
 *  usage:
 *     ./ServoController-Test <arduino serial port>
 */

#ifdef USE_THREADS
#include <pthread.h>
pthread_mutex_t command_mutex_;
pthread_t thread;
#endif

std::vector<double> COMMAND;
std::vector<int> IDS;
std::vector<std::string> NAMES;

std::map<int,double> TARE;
void get_data(){
    int N = IDS.size();
    std::vector<double> pos(N), vel(N), torque(N);
    std::vector<int> recieved_ids(N);
    bool got_data = false;
    got_data = getVal(recieved_ids,pos,vel,torque);
    if(got_data){
      printf(" ID |    POS    |    VEL    |    TOR    | \n");
      for(int i=0;i<N;i++)
        printf(" %2d |  %1.6f  |  %2.5f  |  %2.5f  |\n",recieved_ids[i],pos[i],vel[i],torque[i]);
      printf("\n");
    }
}

double sleep_duration(double duration){
  timespec req,rem;
  int seconds = duration;
  req.tv_nsec = (duration - (double)seconds) * 1.0e+9;
  req.tv_sec = seconds;
  nanosleep(&req,&rem);
  return 0;//( ( (double) rem.tv_nsec / 1.0e+9 ) + (double) rem.tv_sec);
}

std::string DEVICE_NAME;
const int baud = 115200;

void setup(){
  TARE[3] = -1.5709;
  TARE[7] = 0;
  TARE[11] = -1.5709;
  
  std::vector<std::string> dxl_name = boost::assign::list_of
  /*("LF_X_1")("RF_X_1")("LH_X_1")*/("RH_X_1")
  /*("LF_Y_2")("RF_Y_2")("LH_Y_2")*/("RH_Y_2")
  /*("LF_Y_3")("RF_Y_3")("LH_Y_3")*/("RH_Y_3");
  
  std::vector<int> dxl_ids = boost::assign::list_of
  /*(1)(2)(4)*/(3)
  /*(5)(6)(8)*/(7)
  /*(9)(10)(12)*/(11);
  
  IDS = dxl_ids;
  NAMES = dxl_name;
  
  init(DEVICE_NAME.c_str(),baud);
}

double TIME = 0.0;
void loop(){
  
  const int N = IDS.size();
  const int Bps = ( baud / 10 );
  const int Bytes = ( N * 3 ) ;
  const double seconds_per_message = 0.005;//( 1.0 / ((double) Bps) ) * ((double)Bytes);
  for(int i=0;i<N;i++)
    printf("%2.6f(%d)    ",COMMAND[i],IDS[i]);
  printf("\n");
  
  // Use torque controller
  //setVal(IDS,COMMAND);

  double remaining = sleep_duration(seconds_per_message);
  printf("TIME: %f + ( %f - %f )",TIME,seconds_per_message,remaining);
  printf("\n");
  // only add time for time waited: subtract remaining time (rem)
  TIME += seconds_per_message - remaining;
}
