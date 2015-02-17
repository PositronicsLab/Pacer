/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/controller.h>
#include <dxl/Dynamixel.h>

#ifdef USE_DXL
  DXL::Dynamixel * dxl_;
# define DEVICE_NAME "/dev/tty.usbserial-A9YL9ZZV"
#endif

using Pacer::Controller;
using Pacer::Robot;
using Pacer::EndEffector;

boost::shared_ptr<Controller> robot_ptr;
unsigned NDOFS;

#ifdef DRIVE_ROBOT
extern void controller(double time,
                       const Ravelin::VectorNd& q,
                       const Ravelin::VectorNd& qd,
                       Ravelin::VectorNd& command,
                       Ravelin::Pose3d& pose,
                       Ravelin::VectorNd& params);
#endif

// ============================================================================
// ================================ INIT ======================================
// ============================================================================

void init(std::string model_f,std::string vars_f){
  std::cout << "STARTING PACER" << std::endl;
  std::cout << "Robot:" << model_f << std::endl;
  std::cout << "Variables:" << vars_f << std::endl;
#ifdef USE_DXL
  // If use robot is active also init dynamixel controllers
  dxl_ = new DXL::Dynamixel(DEVICE_NAME);
#endif

  /// Set up quadruped robot, linking data from moby's articulated body
  /// to the quadruped model used by Control-Moby

  robot_ptr = boost::shared_ptr<Controller>(new Controller(model_f,vars_f));

//  detect current configuration, then have robot start from there

  // Have robot start from desired initial configuration
  std::vector<Moby::JointPtr> joints = robot_ptr->get_joints();
  Moby::RCArticulatedBodyPtr abrobot = robot_ptr->get_articulated_body();

  std::map<std::string,double> q0 = robot_ptr->get_q0();

  std::vector<double>
      workvd,
      base_start = Utility::get_variable("init.base.x",workvd);

  Ravelin::VectorNd q_start;
  abrobot->get_generalized_coordinates(Moby::DynamicBody::eSpatial,q_start);
  NDOFS = abrobot->num_generalized_coordinates(Moby::DynamicBody::eSpatial) - 6;


  for(unsigned i=NDOFS;i<q_start.rows();i++)
    q_start[i] = base_start[i-NDOFS];

  abrobot->set_generalized_coordinates(Moby::DynamicBody::eSpatial,q_start);
  abrobot->update_link_poses();
  for(int i=0,ii=0;i<joints.size();i++){
    for(int j=0;j<joints[i]->num_dof();j++,ii++){
      joints[i]->q[j] = q0[std::to_string(j)+joints[i]->id];
    }
  }
  abrobot->update_link_poses();

#ifdef USE_DXL
  // LINKS robot
  dxl_->tare.push_back(0);
  dxl_->tare.push_back(0);
  dxl_->tare.push_back(0);
  dxl_->tare.push_back(0);

  dxl_->tare.push_back(M_PI/4 * RX_24F_RAD2UNIT);
  dxl_->tare.push_back(-M_PI/4 * RX_24F_RAD2UNIT);
  dxl_->tare.push_back(-M_PI/4 * MX_64R_RAD2UNIT+40);
  dxl_->tare.push_back(M_PI/4 * MX_64R_RAD2UNIT+250);

  dxl_->tare.push_back(M_PI/2 * RX_24F_RAD2UNIT);
  dxl_->tare.push_back(-M_PI/2 * RX_24F_RAD2UNIT);
  dxl_->tare.push_back(-M_PI/2 * RX_24F_RAD2UNIT);
  dxl_->tare.push_back(M_PI/2 * RX_24F_RAD2UNIT);

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


  dxl_->names.push_back("0LF_X_1");
  dxl_->names.push_back("0RF_X_1");
  dxl_->names.push_back("0LH_X_1");
  dxl_->names.push_back("0RH_X_1");

  dxl_->names.push_back("0LF_Y_2");
  dxl_->names.push_back("0RF_Y_2");
  dxl_->names.push_back("0LH_Y_2");
  dxl_->names.push_back("0RH_Y_2");

  dxl_->names.push_back("0LF_Y_3");
  dxl_->names.push_back("0RF_Y_3");
  dxl_->names.push_back("0LH_Y_3");
  dxl_->names.push_back("0RH_Y_3");

  for(int i=1;i<=dxl_->names.size();i++){
    dxl_->ids.push_back(i);
  }
#endif
}

void controller(double t)
{
  static double last_t = -0.001;
  double dt = t-last_t;
  static Ravelin::VectorNd generalized_qdd,generalized_qd,generalized_q,generalized_fext;
  Moby::ArticulatedBodyPtr abrobot = robot_ptr->get_articulated_body();

  static bool inited = false;
  if(!inited){
    inited= true;
    abrobot->get_generalized_coordinates(Moby::DynamicBody::eEuler,generalized_q);
    abrobot->get_generalized_velocity(Moby::DynamicBody::eSpatial,generalized_qd);
    std::vector<Moby::JointPtr> joints = robot_ptr->get_joints();
    generalized_qdd.set_zero(generalized_qd.size());
    generalized_fext.set_zero(generalized_qd.size());
  }

//  static Ravelin::VectorNd  generalized_qd_last = generalized_qd;
//  //NOTE: Pre-contact    abrobot->get_generalized_acceleration(Moby::DynamicBody::eSpatial,generalized_qdd);
//  ((generalized_qdd = generalized_qd) -= generalized_qd_last) /= dt;
//  generalized_qd_last = generalized_qd;


  static Ravelin::VectorNd q_des,
                          qd_des,
                          qdd_des;
  static Ravelin::VectorNd u;

#ifdef DRIVE_ROBOT
  controller(t,generalized_q,generalized_qd,robot_ptr->movement_command,*robot_ptr->gait_pose.get(),robot_ptr->gait_params);
#endif
  robot_ptr->control(t,generalized_q,generalized_qd,generalized_qdd,generalized_fext,q_des,qd_des,qdd_des,u);

  generalized_q.set_sub_vec(0,q_des);
  generalized_qd.set_sub_vec(0,qd_des);
  generalized_qdd.set_sub_vec(0,qd_des);
#ifdef USE_DXL
    Ravelin::VectorNd q_motors(dxl_->ids.size()),qd_motors(dxl_->ids.size()),u_motors(dxl_->ids.size());
    q_motors.set_zero();
    qd_motors.set_zero();
    u_motors.set_zero();

//    for(int i=0;i<DXL::N_JOINTS;i++)
//      qd_motors[i] = robot_ptr->qd_joints[dxl_->JointName(i)];

    for(int i=0;i<dxl_->ids.size();i++)
      q_motors[i] = robot_ptr->q_joints[dxl_->JointName(i)];
    dxl_->set_state(std::vector<double>(q_motors.begin(),q_motors.end()),std::vector<double>(qd_motors.begin(),qd_motors.end()));

    for(int i=0;i<dxl_->ids.size();i++)
      u_motors[i] = robot_ptr->u_joints[dxl_->JointName(i)];
//    dxl_->set_torque(std::vector<double>(q_motors.begin(),q_motors.end()));
#endif

    last_t = t;
}

int main(int argc, char* argv[])
{
  for(int i=0;i<argc;i++){
    std::cout << argv[i] << std::endl;
  }

  double max_time = INFINITY;
  if(argc == 3){
    init(std::string(argv[1]),std::string(argv[2]));
  } else if(argc == 4) {
    init(std::string(argv[1]),std::string(argv[2]));
    max_time = std::atof(argv[3]);
  } else {
    init("model","vars.xml");
  }

  double t=0;
//  struct timeval start_t, now_t;
//  gettimeofday(&start_t, NULL);
  while(t<max_time){
    t+=0.001;
//    gettimeofday(&now_t, NULL);
//    double t = (now_t.tv_sec - start_t.tv_sec) + (now_t.tv_usec - start_t.tv_usec) * 1E-6;
    controller(t);
  }
}

