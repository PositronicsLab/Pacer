/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/controller.h>
#include <time.h>

#include <boost/assign/list_of.hpp>
#include <boost/assign/std/vector.hpp>

#ifdef USE_DXL
#include <dxl/Dynamixel.h>
DXL::Dynamixel * dxl_;
#endif

#ifdef USE_THREADS
#include <pthread.h>
pthread_mutex_t joint_data_mutex_;
pthread_t thread;
#endif

std::string DEVICE_NAME;

using Pacer::Controller;
using Pacer::Robot;

boost::shared_ptr<Controller> robot_ptr;
unsigned NDOFS;

static Ravelin::VectorNd q_motors,qd_motors,u_motors;
static Ravelin::VectorNd q_sensor,qd_sensor,u_sensor;

static double FREQ = 1000;

void *control_motor(void* data){
  OUT_LOG(logDEBUG2) << ">> control_motor()" << std::endl;
#ifdef USE_THREADS
  while(true)
#endif
  {
    OUT_LOG(logDEBUG2) << ">> motor controller loop" << std::endl;

    OUT_LOG(logDEBUG) << ">> motor controller";
    OUT_LOG(logDEBUG) << "q_des = " << q_motors;
    OUT_LOG(logDEBUG) << "qd_des = " << qd_motors;
    OUT_LOG(logDEBUG) << "u_des = " << u_motors;
    qd_motors.set_zero();

    OUT_LOG(logDEBUG2) << "READ OUT COMMANDS" << std::endl;

    std::vector<double> qm, qdm, um;
#ifdef USE_THREADS
    if(pthread_mutex_trylock(&joint_data_mutex_))
#endif
    {
      qm = std::vector<double>(q_motors.begin(),q_motors.end());
      qdm = std::vector<double>(qd_motors.begin(),qd_motors.end());
      um = std::vector<double>(u_motors.begin(),u_motors.end());
#ifdef USE_THREADS
      pthread_mutex_unlock(&joint_data_mutex_);
#endif
    }
    
    std::vector<double> qs, qds, us;
    
#ifdef USE_DXL
    dxl_->set_state(qm,qdm);
    qs = qm;
    qds = qdm;
//    dxl_->get_state(qs,qds,us);
//    dxl_->set_torque(um);
#endif
    
    OUT_LOG(logDEBUG2) << "READ IN SENSORS" << std::endl;

#ifdef USE_THREADS
    if(pthread_mutex_trylock(&joint_data_mutex_))
#endif
    {
      q_sensor = Ravelin::VectorNd(qs.size(),&qm[0]);
      qd_sensor = Ravelin::VectorNd(qds.size(),&qdm[0]);
//      u_sensor = Ravelin::VectorNd(us.size(),&um[0]);
#ifdef USE_THREADS
      pthread_mutex_unlock(&joint_data_mutex_);
#endif
    }
    
    OUT_LOG(logDEBUG) << "q = " << q_sensor;
    OUT_LOG(logDEBUG) << "qd = " << qd_sensor;
//    OUT_LOG(logDEBUG) << "u = " << u_sensor;
    OUT_LOG(logDEBUG) << "<< motor controller";
    OUT_LOG(logDEBUG2) << "<< motor controller" << std::endl;
    
#ifdef USE_THREADS
    sleep(1.0/FREQ);
#endif
  }
  OUT_LOG(logDEBUG2) << "<< control_motor()" << std::endl;
  return data;
}

// ============================================================================
// ================================ INIT ======================================
// ============================================================================

void init(std::string model_f,std::string vars_f){
  OUT_LOG(logDEBUG2) << "STARTING PACER" << std::endl;
#ifdef USE_DXL
  // If use robot is active also init dynamixel controllers
  dxl_ = new DXL::Dynamixel(DEVICE_NAME.c_str());
#endif
  
  /// Set up quadruped robot, linking data from moby's articulated body
  /// to the quadruped model used by Control-Moby
  OUT_LOG(logDEBUG2) << "STARTING ROBOT" << std::endl;
  
  robot_ptr = boost::shared_ptr<Controller>(new Controller());
  
  robot_ptr->init();
  OUT_LOG(logDEBUG2) << "ROBOT INITED" << std::endl;
  
  robot_ptr->set_generalized_value(Pacer::Robot::position_goal,robot_ptr->get_generalized_value(Pacer::Robot::position));
  robot_ptr->set_generalized_value(Pacer::Robot::velocity_goal,robot_ptr->get_generalized_value(Pacer::Robot::velocity));
  robot_ptr->set_generalized_value(Pacer::Robot::acceleration_goal,robot_ptr->get_generalized_value(Pacer::Robot::acceleration));
#ifdef USE_DXL
  // LINKS robot
  
  // Set Dynamixel Names
  std::vector<std::string> dxl_name = boost::assign::list_of
  ("LF_X_1")("RF_X_1")("LH_X_1")("RH_X_1")
  ("LF_Y_2")("RF_Y_2")("LH_Y_2")("RH_Y_2")
  ("LF_Y_3")("RF_Y_3")("LH_Y_3")("RH_Y_3");
  
  dxl_->names = dxl_name;
  // Set Joint Angles
  //std::vector<int> dxl_tare = boost::assign::list_of
  //    (0)(0)(0)(0)
  //    (M_PI/4 * RX_24F_RAD2UNIT)(-M_PI/4 * RX_24F_RAD2UNIT)(-M_PI/4 * MX_64R_RAD2UNIT+40)(M_PI/4 * MX_64R_RAD2UNIT+250)
  //    (M_PI/2 * RX_24F_RAD2UNIT)(-M_PI/2 * RX_24F_RAD2UNIT)(-M_PI/2 * RX_24F_RAD2UNIT)(M_PI/2 * RX_24F_RAD2UNIT);
  dxl_->tare.push_back( M_PI_2 * RX_24F_RAD2UNIT);
  dxl_->tare.push_back(-M_PI_2 * RX_24F_RAD2UNIT);
  dxl_->tare.push_back(-M_PI_2 * RX_24F_RAD2UNIT);
  dxl_->tare.push_back( M_PI_2 * RX_24F_RAD2UNIT);
  
  dxl_->tare.push_back( M_PI_2 * RX_24F_RAD2UNIT);
  dxl_->tare.push_back(-M_PI_2 * RX_24F_RAD2UNIT);
  dxl_->tare.push_back(-M_PI_2 * MX_64R_RAD2UNIT+250);
  dxl_->tare.push_back( M_PI_2 * MX_64R_RAD2UNIT+40);
  
  dxl_->tare.push_back(0);
  dxl_->tare.push_back(0);
  dxl_->tare.push_back(0);
  dxl_->tare.push_back(0);
  
  //dxl_->tare = dxl_tare;
  
  // Set Dynamixel Type
  std::vector<DXL::Dynamixel::Type> dxl_type = boost::assign::list_of
  (DXL::Dynamixel::RX_24F)(DXL::Dynamixel::RX_24F)(DXL::Dynamixel::RX_24F)(DXL::Dynamixel::RX_24F)
  (DXL::Dynamixel::RX_24F)(DXL::Dynamixel::RX_24F)(DXL::Dynamixel::MX_64R)(DXL::Dynamixel::MX_64R)
  (DXL::Dynamixel::RX_24F)(DXL::Dynamixel::RX_24F)(DXL::Dynamixel::RX_24F)(DXL::Dynamixel::RX_24F);
  
  dxl_->stype = dxl_type;
  dxl_->ids.push_back(1);
  dxl_->ids.push_back(100);
  dxl_->ids.push_back(4);
  dxl_->ids.push_back(3);
  
  dxl_->ids.push_back(5);
  dxl_->ids.push_back(6);
  dxl_->ids.push_back(8);
  dxl_->ids.push_back(7);
  
  dxl_->ids.push_back(9);
  dxl_->ids.push_back(10);
  dxl_->ids.push_back(12);
  dxl_->ids.push_back(11);
  
  q_motors.set_zero(dxl_->ids.size());
  qd_motors.set_zero(dxl_->ids.size());
  u_motors.set_zero(dxl_->ids.size());
  
  dxl_->relaxed(false);
  
  int N = dxl_->ids.size();
  q_motors = Ravelin::VectorNd::zero(N);
  qd_motors = Ravelin::VectorNd::zero(N);
  u_motors = Ravelin::VectorNd::zero(N);
  q_sensor = Ravelin::VectorNd::zero(N);
  qd_sensor = Ravelin::VectorNd::zero(N);
  u_sensor = Ravelin::VectorNd::zero(N);
  
  std::map<std::string,Ravelin::VectorNd> joint_pos_map;
  robot_ptr->get_joint_value(Pacer::Robot::position,joint_pos_map);
  for(int i=0;i<dxl_->ids.size();i++){
    q_motors[i] = joint_pos_map[dxl_->JointName(i)][0];
    qd_motors[i] = 0;
    u_motors[i] = 0;
    q_sensor[i] = joint_pos_map[dxl_->JointName(i)][0];
    qd_sensor[i] = 0;
    u_sensor[i] = 0;

  }

#endif
  
#ifdef USE_THREADS
  pthread_mutex_unlock(&joint_data_mutex_);;
#endif
  
#ifdef USE_THREADS
  const char *message;
  static int iret = pthread_create( &thread, NULL,&control_motor,(void*)NULL);
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
  robot_ptr->get_joint_value(Pacer::Robot::position,q);
  robot_ptr->get_joint_value(Pacer::Robot::velocity,qd);
  robot_ptr->get_joint_value(Pacer::Robot::load,u);

  static std::map<std::string,Ravelin::VectorNd>  qd_last = qd;
#ifdef USE_THREADS
  if(pthread_mutex_lock(&joint_data_mutex_))
#endif
  {
    for(int i=0;i<dxl_->ids.size();i++){
      q[dxl_->JointName(i)][0] = q_sensor[i];
      qd[dxl_->JointName(i)][0] = qd_sensor[i];
      u[dxl_->JointName(i)][0] = u_sensor[i];
    }
    
#ifdef USE_THREADS
    pthread_mutex_unlock(&joint_data_mutex_);
#endif
    for(int i=0;i<dxl_->ids.size();i++){
      qdd[dxl_->JointName(i)] = qd[dxl_->JointName(i)];
      qdd[dxl_->JointName(i)] -= qd_last[dxl_->JointName(i)];
      qdd[dxl_->JointName(i)] /= dt;
    }
  }
  qd_last = qd;
  
  robot_ptr->set_joint_value(Pacer::Robot::position,q);
  robot_ptr->set_joint_value(Pacer::Robot::velocity,qd);
  robot_ptr->set_joint_value(Pacer::Robot::load,u);
  robot_ptr->set_joint_value(Pacer::Robot::acceleration,qdd);
  
  
  /*
   robot_ptr->set_generalized_value(Pacer::Robot::position,robot_ptr->get_generalized_value(Pacer::Robot::position_goal));
   robot_ptr->set_generalized_value(Pacer::Robot::velocity,robot_ptr->get_generalized_value(Pacer::Robot::velocity_goal));
   robot_ptr->set_generalized_value(Pacer::Robot::acceleration,robot_ptr->get_generalized_value(Pacer::Robot::acceleration_goal));
   
   robot_ptr->set_joint_generalized_value(Pacer::Robot::position,robot_ptr->get_joint_generalized_value(Pacer::Robot::position) += (robot_ptr->get_joint_generalized_value(Pacer::Robot::velocity_goal)*=dt));
   */
  
  
  robot_ptr->control(t);
  
#ifdef USE_THREADS
  if(pthread_mutex_lock(&joint_data_mutex_))
#endif
  {
    bool kineamtic_control = true;
    if(kineamtic_control){
      std::map<std::string,Ravelin::VectorNd> joint_pos_map;
      robot_ptr->get_joint_value(Pacer::Robot::position_goal,joint_pos_map);
      std::map<std::string,Ravelin::VectorNd> joint_vel_map;
      robot_ptr->get_joint_value(Pacer::Robot::velocity_goal,joint_vel_map);
      for(int i=0;i<dxl_->ids.size();i++){
        q_motors[i] = joint_pos_map[dxl_->JointName(i)][0];
        qd_motors[i] = joint_vel_map[dxl_->JointName(i)][0];
      }
    } else {
      std::map<std::string,Ravelin::VectorNd> joint_load_map;
      robot_ptr->get_joint_value(Pacer::Robot::load_goal,joint_load_map);
      for(int i=0;i<dxl_->ids.size();i++){
        u_motors[i] = joint_load_map[dxl_->JointName(i)][0];
      }
    }
    
#ifdef USE_THREADS
    pthread_mutex_unlock(&joint_data_mutex_);
#endif
  }
  
#ifndef USE_THREADS
  OUT_LOG(logDEBUG2) << "call control_motor() from controller" << std::endl;
  control_motor((void*)NULL);
#endif
  last_t = t;
#ifdef TIMING
  static double last_time, this_time;
  static bool inited = false;
  if(inited){
    this_time = get_current_time();
    double duration_ms = (this_time - last_time)*1000.0;
    //    std::cerr<< "dt = " << duration_ms<<std::endl;
  }
  inited = true;
  last_time = get_current_time();
#endif
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
  
  double t=0;
  //  struct timeval start_t, now_t;
  //  gettimeofday(&start_t, NULL);
  while(t<max_time){
    t += 1.0/FREQ;
    //    gettimeofday(&now_t, NULL);
    //    double t = (now_t.tv_sec - start_t.tv_sec) + (now_t.tv_usec - start_t.tv_usec) * 1E-6;
    controller(t);
    sleep(1.0/FREQ);
  }
  
#ifdef USE_THREADS
  pthread_join( thread, NULL);
#endif
}

