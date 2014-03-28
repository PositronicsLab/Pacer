#include <robot.h>
#include <utilities.h>

using namespace Ravelin;
using namespace Moby;

double Robot::calc_energy(Ravelin::VectorNd& v, Ravelin::MatrixNd& M){
  // Potential Energy
  double PE = 0;
  for(int i=0;i<links_.size();i++){
     Moby::RigidBody& link = *links_[i];
     double m = link.get_mass();
     Ravelin::Pose3d link_com = *link.get_inertial_pose();
     link_com.update_relative_pose(Moby::GLOBAL);
     PE += link_com.x[2] * m * grav;
  }
  M.mult(v, workv_);
  double KE = workv_.dot(v)*0.5;
#ifndef NDEBUG
//  std::cout << "KE = " << KE << ", PE = " << PE << std::endl;
//  std::cout << "Total Energy = " << (KE + PE) << std::endl;
#endif
  return (KE + PE);
  // Kinetic Energy
}

void Robot::calc_com(Ravelin::Vector3d& weighted_com,Ravelin::Vector3d& com_acc){
  weighted_com.set_zero();
  const SAcceld& base_acc = links_[0]->get_accel();
  double total_mass=0;
  for(int i=0;i<links_.size();i++){

     RigidBody& link = *links_[i];
     double m = link.get_mass();
     total_mass += m;
     Ravelin::Pose3d link_com = *link.get_inertial_pose();
     link_com.update_relative_pose(Moby::GLOBAL);
     weighted_com += (link_com.x *= m);
  }
  weighted_com /= total_mass;

  boost::shared_ptr<Ravelin::Pose3d> base_com_w(new Ravelin::Pose3d());
  base_com_w->x = Ravelin::Origin3d(weighted_com);
  SAcceld com_xdd = Ravelin::Pose3d::transform(base_com_w, base_acc);
  com_acc = com_xdd.get_linear();

  zero_moment_point =
      Ravelin::Vector3d(center_of_mass[0] - (center_of_mass[2]*com_acc[0])/(com_acc[2]-grav),
                        center_of_mass[1] - (center_of_mass[2]*com_acc[1])/(com_acc[2]-grav),
                        0);
}

void Robot::calculate_dyn_properties(Ravelin::MatrixNd& M, Ravelin::VectorNd& fext){
   M.resize(NDOFS,NDOFS);
   fext.resize(NDOFS);
   abrobot_->get_generalized_inertia(M);
   abrobot_->get_generalized_forces(fext);
}

void Robot::compile(){
  dbrobot_ = boost::dynamic_pointer_cast<DynamicBody>(abrobot_);
  std::vector<JointPtr> joints = abrobot_->get_joints();
  joints_.resize(joints.size());

  NUM_FIXED_JOINTS = 0;
  for(unsigned i=0;i<joints.size();i++){
    if(joints[i]->q.rows() == 0){
      NUM_FIXED_JOINTS ++;
      continue;
    }
    joints_[joints[i]->get_coord_index()] = joints[i];
  }

  for(unsigned i=0;i<joints_.size()-NUM_FIXED_JOINTS;i++){
    if(joints_[i]->q.rows() == 0){
      continue;
    }
    joint_names_.push_back(joints_[i]->id);
    std::cout << joints_[i]->get_coord_index() << " "
              << joints_[i]->id << std::endl;
  }

  // Set up link references
  links_ = abrobot_->get_links();
}

void EndEffector::init(){
  Moby::JointPtr joint_ptr = link->get_inner_joint_explicit();
  Moby::RigidBodyPtr rb_ptr = link;
  std::cout << id << std::endl;
  chain_bool.resize(joint_names_.size());
  while (rb_ptr->id.compare("ABDOMEN") != 0 && rb_ptr->id.compare("THORAX") != 0){
    rb_ptr = joint_ptr->get_inboard_link();
    for(int j=0;j<joint_names_.size();j++){
      if(joint_ptr->id.compare(joint_names_[j]) == 0){
        std::cout << j <<  " "<< joint_ptr->id << std::endl;
        chain.push_back(j);
        chain_bool[j] = true;
      }
    }
    joint_ptr = rb_ptr->get_inner_joint_explicit();
  }

  Ravelin::Pose3d pose = *link->get_pose();
  normal = Ravelin::Vector3d(0,0,1);
  point = pose.x;
  active = false;
}

void Robot::update(){
  // fetch robot state vectors
  dbrobot_->get_generalized_acceleration(acc);
  dbrobot_->get_generalized_velocity(Moby::DynamicBody::eSpatial,vel);
  dbrobot_->get_generalized_coordinates(Moby::DynamicBody::eSpatial,gc);
  calc_contact_jacobians(N,ST,D,R);
  // Cn * M * v = iM * fext
  //      M * v = iM * fext * h
  // Get robot dynamics state
  // SRZ: Very Heavy Computation
  calculate_dyn_properties(M,fext);
  calc_energy(vel,M);
  calc_com(center_of_mass,workv3_);

  // Get base frame
  base_frame = boost::shared_ptr<Ravelin::Pose3d>( new Ravelin::Pose3d(*links_[0]->get_pose()));
  base_frame->update_relative_pose(Moby::GLOBAL);


  base_horizontal_frame = boost::shared_ptr<Ravelin::Pose3d>( new Ravelin::Pose3d(Moby::GLOBAL));
  base_horizontal_frame->update_relative_pose(Moby::GLOBAL);
  base_horizontal_frame->x = base_frame->x;
  Ravelin::Matrix3d Rot(base_frame->q);
  R2rpy(Rot,roll_pitch_yaw);
//  OUTLOG(roll_pitch_yaw,"roll_pitch_yaw");
  // remove roll and pitch -- preserve yaw
  Rz(roll_pitch_yaw[2],Rot);
  base_horizontal_frame->x = base_frame->x;
  base_horizontal_frame->q = base_frame->q;//Quatd(Rot);
  for(int i=0;i<NUM_EEFS;i++)
    eefs_[i].origin.pose = base_horizontal_frame;

  base_frame_global = boost::shared_ptr<Ravelin::Pose3d>( new Ravelin::Pose3d(Moby::GLOBAL));
  base_frame_global->update_relative_pose(Moby::GLOBAL);
  base_frame_global->x = base_frame->x;
  base_frame_global->q.set_identity();

  calc_eef_jacobians(J);
}
