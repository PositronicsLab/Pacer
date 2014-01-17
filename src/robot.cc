#include <robot.h>

using namespace Ravelin;
using namespace Moby;

static Ravelin::VectorNd workv_;
static Ravelin::MatrixNd workM_;

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
  std::cout << "KE = " << KE << ", PE = " << PE << std::endl;
  std::cout << "Total Energy = " << (KE + PE) << std::endl;
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
}

void Robot::calculate_dyn_properties(Ravelin::MatrixNd& M, Ravelin::VectorNd& fext){
   M.resize(NDOFS,NDOFS);
   fext.resize(NDOFS);
   abrobot_->get_generalized_inertia(M);
   abrobot_->get_generalized_forces(fext);
}

void Robot::compile(){
  std::vector<JointPtr> joints = abrobot_->get_joints();
  joints_.resize(joints.size());

  NUM_FIXED_JOINTS = 0;
  for(unsigned i=0;i<joints.size();i++){
    if(joints[i]->q.rows() == 0){
      NUM_FIXED_JOINTS ++;
      continue;
    }
    joints_[joints[i]->get_coord_index()] = joints[i];
     std::cout << joints[i]->get_coord_index() << " "
               << joints_[joints[i]->get_coord_index()]->id << std::endl;
  }

  // Set up link references
  links_ = abrobot_->get_links();
}

void EndEffector::init(){
  Moby::JointPtr joint_ptr = link->get_inner_joint_explicit();
  Moby::RigidBodyPtr rb_ptr = link;
  while (!rb_ptr->is_base() && rb_ptr->id.compare("THORAX") != 0){
    rb_ptr = joint_ptr->get_inboard_link();
    for(int j=0;j<joint_names_.size();j++){
      if(joint_ptr->id.compare(joint_names_[j]) == 0){
        chain.push_back(j);
      }
    }
    joint_ptr = rb_ptr->get_inner_joint_explicit();
  }

  Ravelin::Pose3d pose = *link->get_pose();
  normal = Ravelin::Vector3d(0,0,1);
  point = pose.x;
  active = false;
}

