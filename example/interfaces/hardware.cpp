/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/controller.h>
#include <time.h>

#include "arduino.cpp"

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
        COMMAND[i] = sin(t * 16.0) * M_PI/8.0; //joint_pos_map[NAMES[i]][0] + TARE[IDS[i]];
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

