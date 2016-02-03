/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include "../plugin.h"
#include <time.h>

#include <boost/assign/list_of.hpp>
#include <boost/assign/std/vector.hpp>

#ifdef USE_DXL
#include <dxl/Dynamixel.h>
DXL::Dynamixel * dxl_;
#endif
std::vector<std::string> joint_name;

#ifdef USE_THREADS
#include <pthread.h>
pthread_mutex_t joint_data_mutex_;
pthread_t thread;
#endif

static std::vector<double> q_motors,qd_motors,u_motors;
static std::vector<double> q_sensor,qd_sensor,u_sensor;

static double TIME = 0; 
static double FIRST_TIME = 0;
static double sleep_duration(double duration){
  timespec req,rem;
  int seconds = duration;
  req.tv_nsec = (duration - (double)seconds) * 1.0e+9;
  req.tv_sec = seconds;
  nanosleep(&req,&rem);
  return 0;//( ( (double) rem.tv_nsec / 1.0e+9 ) + (double) rem.tv_sec);
}
/// Gets the current time (as a floating-point number)
static double get_current_time()
{
  const double MICROSEC = 1.0/1000000;
  timeval t;
  gettimeofday(&t, NULL);
  return (double) t.tv_sec + (double) t.tv_usec * MICROSEC;
}


static void control_motor(){
    boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  //fprintf(stdout,"refreshing actuators (threads): TIME: %f \n",TIME-FIRST_TIME);
  OUT_LOG(logDEBUG2) << ">> control_motor()" << std::endl;

  OUT_LOG(logDEBUG) << ">> motor controller";

  OUT_LOG(logDEBUG2) << "READ OUT COMMANDS" << std::endl;
  
  {
#ifdef USE_THREADS
    pthread_mutex_lock(&joint_data_mutex_);
#endif
#ifdef USE_DXL
#ifndef USE_THREADS
      OUT_LOG(logDEBUG) << "q_des = " << q_motors;
      OUT_LOG(logDEBUG) << "qd_des = " << qd_motors;
      OUT_LOG(logDEBUG) << "u_des = " << u_motors;
#endif
      dxl_->set_state(q_motors,qd_motors);
      static bool fake_state = ctrl->get_data<bool>(plugin_namespace+".fake-state");
      if(fake_state){
        q_sensor = q_motors;
        qd_sensor = qd_motors;
      } else {
        dxl_->get_state(q_sensor,qd_sensor,u_sensor);
      }
#ifndef USE_THREADS
      OUT_LOG(logDEBUG) << "q = " << q_sensor;
      OUT_LOG(logDEBUG) << "qd = " << qd_sensor;
#endif
#endif
#ifdef USE_THREADS
      pthread_mutex_unlock(&joint_data_mutex_);
#endif
    }
  OUT_LOG(logDEBUG) << "<< motor controller";
  
  const double seconds_per_message = 0.001;
  double remaining = sleep_duration(seconds_per_message);
#ifndef NDEBUG
  printf("TIME: %f + ( %f - %f )",TIME,seconds_per_message,remaining);
  printf("\n");
#endif
  //TIME += seconds_per_message - remaining;
}
  
static void *control_motor(void* data){
  while(1){
    control_motor();
  }
}

  static std::map<std::string,Ravelin::VectorNd> q,qd,qdd,u;
void loop()
{
    boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);

  OUT_LOG(logDEBUG2) << "controller()";
  static double last_t = -0.001;
  double dt = t-last_t;
  
  
  static std::map<std::string,Ravelin::VectorNd>  qd_last = qd;
#ifdef USE_THREADS
  if(pthread_mutex_lock(&joint_data_mutex_))
#endif
  {
    for(int i=0;i<joint_name.size();i++){
      q[joint_name[i]][0] = q_sensor[i];
      qd[joint_name[i]][0] = qd_sensor[i];
      u[joint_name[i]][0] = u_sensor[i];
    }
    
#ifdef USE_THREADS
    pthread_mutex_unlock(&joint_data_mutex_);
#endif
    for(int i=0;i<joint_name.size();i++){
      qdd[joint_name[i]] = qd[joint_name[i]];
      qdd[joint_name[i]] -= qd_last[joint_name[i]];
      qdd[joint_name[i]] /= dt;
    }
  }
  qd_last = qd;
  bool APPLY_SENSORS = false;
  ctrl->get_data<bool>(plugin_namespace+".apply-sensors",APPLY_SENSORS);
  if(APPLY_SENSORS){
    ctrl->set_joint_value(Pacer::Robot::position,q);
    ctrl->set_joint_value(Pacer::Robot::velocity,qd);
    ctrl->set_joint_value(Pacer::Robot::load,u);
    ctrl->set_joint_value(Pacer::Robot::acceleration,qdd);
  }
#ifdef USE_THREADS
  if(pthread_mutex_lock(&joint_data_mutex_))
#endif
  {
    bool kineamtic_control = ctrl->get_data<bool>(plugin_namespace+".kinematic-control");
    if(kineamtic_control){
      std::map<std::string,Ravelin::VectorNd> joint_pos_map;
      ctrl->get_joint_value(Pacer::Robot::position_goal,joint_pos_map);
      //std::map<std::string,Ravelin::VectorNd> joint_vel_map;
      //ctrl->get_joint_value(Pacer::Robot::velocity_goal,joint_vel_map);
      OUT_LOG(logDEBUG) << "kineamtic control = " << joint_pos_map;
      for(int i=0;i<joint_name.size();i++){
        q_motors[i]  = joint_pos_map[joint_name[i]][0];
        qd_motors[i] = 0;//joint_vel_map[joint_name[i]][0];
      }
    } else {
      std::map<std::string,Ravelin::VectorNd> joint_load_map;
      OUT_LOG(logDEBUG) << "torque control = " << joint_load_map;
      ctrl->get_joint_value(Pacer::Robot::load_goal,joint_load_map);
      for(int i=0;i<joint_name.size();i++){
        u_motors[i] = joint_load_map[joint_name[i]][0];
      }
    }
#ifdef USE_THREADS
    pthread_mutex_unlock(&joint_data_mutex_);
#endif
  }
  
#ifdef USE_THREADS
  OUT_LOG(logDEBUG) << "q_des = " << q_motors;
  OUT_LOG(logDEBUG) << "qd_des = " << qd_motors;
  OUT_LOG(logDEBUG) << "u_des = " << u_motors;
  OUT_LOG(logDEBUG) << "q = " << q_sensor;
  OUT_LOG(logDEBUG) << "qd = " << qd_sensor;
#else
  OUT_LOG(logDEBUG2) << "call control_motor() from controller" << std::endl;
  control_motor();
#endif
  
  last_t = t;
  OUT_LOG(logDEBUG2) << "end controller()" << std::endl;
}

void destruct(){
  #ifdef USE_THREADS
    pthread_join( thread, NULL);
  #endif
}


void setup(){
    boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  ctrl->get_joint_value(Pacer::Robot::position,q);
  ctrl->get_joint_value(Pacer::Robot::velocity,qd);
  ctrl->get_joint_value(Pacer::Robot::load,u);

  std::string DEVICE_NAME = ctrl->get_data<std::string>(plugin_namespace+".port");
#ifdef USE_DXL
  // If use robot is active also init dynamixel controllers
  dxl_ = new DXL::Dynamixel(DEVICE_NAME.c_str());
#endif
  
  // Set Dynamixel Names
  joint_name.push_back("LF_X_1");
  joint_name.push_back("RF_X_1");
  joint_name.push_back("RH_X_1");
  joint_name.push_back("LH_X_1");

  joint_name.push_back("LF_Y_2");
  joint_name.push_back("RF_Y_2");
  joint_name.push_back("RH_Y_2");
  joint_name.push_back("LH_Y_2");

  joint_name.push_back("LF_Y_3");
  joint_name.push_back("RF_Y_3");
  joint_name.push_back("RH_Y_3");
  joint_name.push_back("LH_Y_3");
#ifdef USE_DXL
  // LINKS robot
  dxl_->tare.push_back(-M_PI_2 * RX_24F_RAD2UNIT);
  dxl_->tare.push_back( M_PI_2 * RX_24F_RAD2UNIT);
  dxl_->tare.push_back(-M_PI_2 * RX_24F_RAD2UNIT);
  dxl_->tare.push_back( M_PI_2 * RX_24F_RAD2UNIT);
  
  dxl_->tare.push_back(-M_PI_2 * RX_24F_RAD2UNIT);
  dxl_->tare.push_back( M_PI_2 * RX_24F_RAD2UNIT);
  dxl_->tare.push_back(-M_PI_2 * MX_64R_RAD2UNIT - 20 );
  dxl_->tare.push_back( M_PI_2 * MX_64R_RAD2UNIT - 270);

  dxl_->tare.push_back(0);
  dxl_->tare.push_back(0);
  dxl_->tare.push_back(0);
  dxl_->tare.push_back(0);
  
  dxl_->stype.push_back(DXL::Dynamixel::RX_24F);
  dxl_->stype.push_back(DXL::Dynamixel::RX_24F);
  dxl_->stype.push_back(DXL::Dynamixel::RX_24F);
  dxl_->stype.push_back(DXL::Dynamixel::RX_24F);

  dxl_->stype.push_back(DXL::Dynamixel::RX_24F);
  dxl_->stype.push_back(DXL::Dynamixel::RX_24F);
  dxl_->stype.push_back(DXL::Dynamixel::MX_64R);
  dxl_->stype.push_back(DXL::Dynamixel::MX_64R);

  dxl_->stype.push_back(DXL::Dynamixel::RX_24F);
  dxl_->stype.push_back(DXL::Dynamixel::RX_24F);
  dxl_->stype.push_back(DXL::Dynamixel::RX_24F);
  dxl_->stype.push_back(DXL::Dynamixel::RX_24F);

  dxl_->names = joint_name;

  for(int i=1;i<=dxl_->names.size();i++){
    dxl_->ids.push_back(i);
  }
  
  dxl_->relaxed(false);
#endif
  
  int N = joint_name.size();
  q_motors = std::vector<double>(N);
  qd_motors = std::vector<double>(N);
  u_motors = std::vector<double>(N);
  q_sensor = std::vector<double>(N);
  qd_sensor = std::vector<double>(N);
  u_sensor = std::vector<double>(N);
  
  for(int i=0;i<N;i++){
    q_motors[i] = q[joint_name[i]][0];
    qd_motors[i] = 0;
    u_motors[i] = 0;
    q_sensor[i] = q[joint_name[i]][0];
    qd_sensor[i] = 0;
    u_sensor[i] = 0;
  }
  
#ifdef USE_THREADS
  pthread_mutex_unlock(&joint_data_mutex_);;
#endif
  
  double wait_time = 5.0;
  ctrl->get_data<double>(plugin_namespace+".hold-time",wait_time);
#ifdef USE_THREADS
  const char *message;
  static int iret = pthread_create( &thread, NULL,&control_motor,(void*)NULL);
  if(iret)
  {
    fprintf(stderr,"Error - pthread_create() return code: %d\n",iret);
    exit(1);
  }
  const double t0 = get_current_time();
  double t1 = get_current_time();
  while( t1 - t0 < wait_time){
    t1 = get_current_time();
    fprintf(stdout,"refreshing actuators (threads): TIME: %f \n",t1-t0);
    sleep_duration(0.1);
  }
#else
  OUT_LOG(logDEBUG2) << "call control_motor() from controller" << std::endl;
  const double t0 = get_current_time();
  double t1 = get_current_time();
  while( t1 - t0 < wait_time){
    t1 = get_current_time();
    fprintf(stdout,"refreshing actuators (no threads): TIME: %f \n",t1-t0);
    control_motor();
    sleep_duration(0.1);
  }
#endif
}
