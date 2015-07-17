/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/controller.h>
#include <thread>

#define USE_DXL

#ifdef USE_DXL
#include <dxl/Dynamixel.h>
  DXL::Dynamixel * dxl_;
  std::string DEVICE_NAME;
#endif

using Pacer::Controller;
using Pacer::Robot;

boost::shared_ptr<Controller> robot_ptr;
unsigned NDOFS;

static Ravelin::VectorNd q_motors_data,qd_motors_data,u_motors_data;

std::mutex joint_data_mutex_;
static double FREQ = 500;

static void control_motor(){
  while(true){
    static Ravelin::VectorNd q_motors,qd_motors,u_motors;
    if(joint_data_mutex_.try_lock()){
      q_motors = q_motors_data;
      
      qd_motors = qd_motors_data;
      u_motors = u_motors_data;
      joint_data_mutex_.unlock();
    }
 
    dxl_->set_state(std::vector<double>(q_motors.begin(),q_motors.end()),std::vector<double>(qd_motors.begin(),qd_motors.end()));
//    dxl_->set_torque(std::vector<double>(q_motors.begin(),q_motors.end()));
    sleep(1.0/FREQ);
  }
}

// ============================================================================
// ================================ INIT ======================================
// ============================================================================

void init(std::string model_f,std::string vars_f){
  std::cout << "STARTING PACER" << std::endl;
#ifdef USE_DXL
  // If use robot is active also init dynamixel controllers
  dxl_ = new DXL::Dynamixel(DEVICE_NAME.c_str());
#endif

  /// Set up quadruped robot, linking data from moby's articulated body
  /// to the quadruped model used by Control-Moby
    std::cout << "STARTING ROBOT" << std::endl;

  robot_ptr = boost::shared_ptr<Controller>(new Controller());
    
    robot_ptr->init();
    std::cout << "ROBOT INITED" << std::endl;

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
  std::vector<int> dxl_tare = boost::assign::list_of
      (0)(0)(0)(0)
      (M_PI/4 * RX_24F_RAD2UNIT)(-M_PI/4 * RX_24F_RAD2UNIT)(-M_PI/4 * MX_64R_RAD2UNIT+40)(M_PI/4 * MX_64R_RAD2UNIT+250)
      (M_PI/2 * RX_24F_RAD2UNIT)(-M_PI/2 * RX_24F_RAD2UNIT)(-M_PI/2 * RX_24F_RAD2UNIT)(M_PI/2 * RX_24F_RAD2UNIT);

  dxl_->tare = dxl_tare;

  // Set Dynamixel Type
  std::vector<DXL::Dynamixel::Type> dxl_type = boost::assign::list_of
    (DXL::Dynamixel::RX_24F)(DXL::Dynamixel::RX_24F)(DXL::Dynamixel::RX_24F)(DXL::Dynamixel::RX_24F)
    (DXL::Dynamixel::RX_24F)(DXL::Dynamixel::RX_24F)(DXL::Dynamixel::MX_64R)(DXL::Dynamixel::MX_64R)
    (DXL::Dynamixel::RX_24F)(DXL::Dynamixel::RX_24F)(DXL::Dynamixel::RX_24F)(DXL::Dynamixel::RX_24F);

  dxl_->stype = dxl_type;

  for(int i=1;i<=dxl_->names.size();i++){
    dxl_->ids.push_back(i);
  }
    q_motors_data.set_zero(dxl_->ids.size());
    qd_motors_data.set_zero(dxl_->ids.size());
    u_motors_data.set_zero(dxl_->ids.size());
    
    dxl_->relaxed(false);

#endif
  
  joint_data_mutex_.unlock();

}

void controller(double t)
{
  static double last_t = -0.001;
  double dt = t-last_t;

//  static Ravelin::VectorNd  generalized_qd_last = generalized_qd;
//  //NOTE: Pre-contact    abrobot->get_generalized_acceleration(Moby::DynamicBody::eSpatial,generalized_qdd);
//  ((generalized_qdd = generalized_qd) -= generalized_qd_last) /= dt;
//  generalized_qd_last = generalized_qd;

  robot_ptr->set_generalized_value(Pacer::Robot::position,robot_ptr->get_generalized_value(Pacer::Robot::position_goal));
  robot_ptr->set_generalized_value(Pacer::Robot::velocity,robot_ptr->get_generalized_value(Pacer::Robot::velocity_goal));
  robot_ptr->set_generalized_value(Pacer::Robot::acceleration,robot_ptr->get_generalized_value(Pacer::Robot::acceleration_goal));

  robot_ptr->control(t);

#ifdef USE_DXL
  if(joint_data_mutex_.try_lock()){
//    for(int i=0;i<DXL::N_JOINTS;i++)
//      qd_motors[i] = robot_ptr->qd_joints[dxl_->JointName(i)];

    std::map<std::string,Ravelin::VectorNd> joint_val_map;
    robot_ptr->get_joint_value(Pacer::Robot::position_goal,joint_val_map);

    for(int i=0;i<dxl_->ids.size();i++)
      q_motors_data[i] = joint_val_map[dxl_->JointName(i)][0];

    //for(int i=0;i<dxl_->ids.size();i++)
    //  u_motors_data[i] = robot_ptr->get_joint_value(Pacer::Robot::load_goal,dxl_->JointName(i),0);
    joint_data_mutex_.unlock();
  }
#endif
  static std::thread motor_thread(control_motor);

    last_t = t;
}

int main(int argc, char* argv[])
{
  for(int i=0;i<argc;i++){
    std::cout << argv[i] << std::endl;
  }

  DEVICE_NAME = argv[1];
  double max_time = INFINITY;
    
  init("model","vars.xml");

  

#ifdef USE_DXL
  double t=0;
//  struct timeval start_t, now_t;
//  gettimeofday(&start_t, NULL);
  while(t<max_time){
    t += 1.0/FREQ;
//    gettimeofday(&now_t, NULL);
//    double t = (now_t.tv_sec - start_t.tv_sec) + (now_t.tv_usec - start_t.tv_usec) * 1E-6;
    controller(t);
  }
#endif
}

