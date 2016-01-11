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
/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/controller.h>
#include <time.h>


using Pacer::Controller;
using Pacer::Robot;

boost::shared_ptr<Controller> robot_ptr;
unsigned NDOFS;

// ============================================================================
// ================================ INIT ======================================
// ============================================================================

void init(std::string model_f,std::string vars_f){
  OUT_LOG(logDEBUG2) << "STARTING PACER" << std::endl;
    
  setup();

  /// Set up quadruped robot, linking data from moby's articulated body
  /// to the quadruped model used by Control-Moby
  OUT_LOG(logDEBUG2) << "STARTING ROBOT" << std::endl;
  
  robot_ptr = boost::shared_ptr<Controller>(new Controller());
  
  robot_ptr->init();
  OUT_LOG(logDEBUG2) << "ROBOT INITED" << std::endl;
  
  robot_ptr->set_generalized_value(Pacer::Robot::position_goal,robot_ptr->get_generalized_value(Pacer::Robot::position));
  robot_ptr->set_generalized_value(Pacer::Robot::velocity_goal,robot_ptr->get_generalized_value(Pacer::Robot::velocity));
  robot_ptr->set_generalized_value(Pacer::Robot::acceleration_goal,robot_ptr->get_generalized_value(Pacer::Robot::acceleration));

  COMMAND.resize(IDS.size());
  
  std::map<std::string,Ravelin::VectorNd> joint_pos_map;
  robot_ptr->get_joint_value(Pacer::Robot::position_goal,joint_pos_map);
  for(int i=0;i<IDS.size();i++){
    COMMAND[i] = joint_pos_map[NAMES[i]][0] + TARE[IDS[i]];
  }
  
#ifdef USE_THREADS
  pthread_mutex_unlock(&command_mutex_);;
#endif
  loop();
  
#ifdef USE_THREADS
  const char *message;
  static int iret = pthread_create( &thread, NULL,&loop,(void*)NULL);
  if(iret)
  {
    fprintf(stderr,"Error - pthread_create() return code: %d\n",iret);
    exit(1);
  }
#endif
  
}

/// Gets the current time (as a floating-point number)
double get_current_time()
{
  const double MICROSEC = 1.0/1000000;
  timeval t;
  gettimeofday(&t, NULL);
  return (double) t.tv_sec + (double) t.tv_usec * MICROSEC;
}

void controller(double t)
{
  OUT_LOG(logDEBUG2) << "controller()" << std::endl;
  static double last_t = -0.001;
  double dt = t-last_t;
  
  
  std::map<std::string,Ravelin::VectorNd> q,qd,qdd,u;
  robot_ptr->get_joint_value(Pacer::Robot::position_goal,q);
  robot_ptr->get_joint_value(Pacer::Robot::velocity_goal,qd);
  robot_ptr->get_joint_value(Pacer::Robot::load,u);


  static std::map<std::string,Ravelin::VectorNd>  qd_last = qd;
  robot_ptr->set_joint_value(Pacer::Robot::position,q);
  robot_ptr->set_joint_value(Pacer::Robot::velocity,qd);
  robot_ptr->set_joint_value(Pacer::Robot::load,u);
  robot_ptr->set_joint_value(Pacer::Robot::acceleration,qdd);
  
  robot_ptr->control(t);
  
#ifdef USE_THREADS
  if(pthread_mutex_lock(&command_mutex_))
#endif
  {
    bool kineamtic_control = true;
    if(kineamtic_control){
      std::map<std::string,Ravelin::VectorNd> joint_pos_map;
      robot_ptr->get_joint_value(Pacer::Robot::position_goal,joint_pos_map);
      for(int i=0;i<NAMES.size();i++){
        //COMMAND[i] = sin(t * 16.0) * M_PI/8.0; //joint_pos_map[NAMES[i]][0] + TARE[IDS[i]];
        COMMAND[i] = joint_pos_map[NAMES[i]][0] + TARE[IDS[i]];
      }
    }
#ifdef USE_THREADS
    pthread_mutex_unlock(&command_mutex_);
#endif
  }
  
#ifndef USE_THREADS
  OUT_LOG(logDEBUG2) << "call control_motor() from controller" << std::endl;
  loop();
#endif
  last_t = t;
  
  OUT_LOG(logDEBUG2) << "end controller()" << std::endl;
}

int main(int argc, char* argv[])
{
  for(int i=0;i<argc;i++){
    OUT_LOG(logDEBUG2) << argv[i] << std::endl;
  }
  
  DEVICE_NAME = argv[1];
  double max_time = INFINITY;
  
  init("model","vars.xml");
  
  //  struct timeval start_t, now_t;
  //  gettimeofday(&start_t, NULL);
  while(TIME<max_time){
   // t += 1.0/FREQ;
    //    gettimeofday(&now_t, NULL);
    //    double t = (now_t.tv_sec - start_t.tv_sec) + (now_t.tv_usec - start_t.tv_usec) * 1E-6;
    controller(TIME);
   // sleep(1.0/FREQ);
  }
  
#ifdef USE_THREADS
  pthread_join( thread, NULL);
#endif
  return 0;
}

