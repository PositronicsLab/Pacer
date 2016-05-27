/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/controller.h>
#include <Pacer/utilities.h>
#include "plugin.h"

void loop(){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  
  const  std::vector<std::string>
  eef_names_ = ctrl->get_data<std::vector<std::string> >("init.end-effector.id");
  
  int NUM_FEET = eef_names_.size();
  Ravelin::VectorNd local_q,q_goal, qd_goal, qdd_goal;
  ctrl->get_joint_generalized_value(Pacer::Controller::position_goal,q_goal);
  ctrl->get_joint_generalized_value(Pacer::Controller::velocity_goal,qd_goal);
  ctrl->get_joint_generalized_value(Pacer::Controller::acceleration_goal,qdd_goal);
  
  int N = ctrl->num_joint_dof();
  local_q.set_zero(ctrl->num_total_dof_euler());
  local_q.segment(0,N) = q_goal;
  local_q[N+6] = 1;
  
  for(unsigned i=0;i<NUM_FEET;i++){
    Ravelin::Origin3d xd,xdd;
    
    // Calc jacobian for AB at this EEF
    Ravelin::MatrixNd J = ctrl->calc_link_jacobian(local_q,eef_names_[i]);
    
    // Now that model state is set ffrom jacobian calculation
    const boost::shared_ptr<Ravelin::RigidBodyd>  link = ctrl->get_link(eef_names_[i]);
    
    
    Ravelin::Pose3d foot_pose(Ravelin::Matrix3d(link->get_pose()->q)*Ravelin::Matrix3d(0,0,-1, -1,0,0, 0,1,0),link->get_pose()->x,link->get_pose()->rpose);
    foot_pose.update_relative_pose(Pacer::GLOBAL);
    VISUALIZE(POSE(foot_pose,0.8));
    
    //    OUT_LOG(logERROR) << eef_names_[i] << "-orientation: " << t << " " << foot_pose.q;
    
    //    Ravelin::Origin3d x(Ravelin::Pose3d::transform_point(Pacer::GLOBAL,Ravelin::Vector3d(0,0,0,link->get_pose())).data());
    ctrl->set_data<Ravelin::Origin3d>(eef_names_[i]+".goal.x",foot_pose.x);
    ctrl->set_data<Ravelin::Quatd>(eef_names_[i]+".goal.q",foot_pose.q);
    
    J.block(0,3,0,N).mult(qd_goal,xd);
    J.block(0,3,0,N).mult(qdd_goal,xdd);

    ctrl->set_data<Ravelin::Origin3d>(eef_names_[i]+".goal.xd",xd);
    ctrl->set_data<Ravelin::Origin3d>(eef_names_[i]+".goal.xdd",xdd);
    ctrl->set_end_effector_value(eef_names_[i],Pacer::Controller::position_goal,foot_pose.x);
    ctrl->set_end_effector_value(eef_names_[i],Pacer::Controller::velocity_goal,xd);
    ctrl->set_end_effector_value(eef_names_[i],Pacer::Controller::acceleration_goal,xdd);
  }
}
void setup(){
}