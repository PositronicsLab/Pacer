/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/controller.h>
#include <Pacer/utilities.h>
#include "plugin.h"

#define DISPLAY 
#ifndef DISPLAY
#undef VISUALIZE
#define VISUALIZE(x) if(0){}
#endif

void loop(){
  static double last_time = t;
  double dt = t - last_time;
  last_time = t;
  
boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);

  std::vector<std::string>
  foot_names_ = ctrl->get_data<std::vector<std::string> >("init.end-effector.id");
  
  int NUM_FEET = foot_names_.size();
  std::vector<Ravelin::Origin3d>
      foot_pos,
      foot_vel,
      foot_acc;

  boost::shared_ptr<Ravelin::Pose3d> base_frame;
  base_frame = boost::shared_ptr<Ravelin::Pose3d>( new Ravelin::Pose3d(
                                                     ctrl->get_data<Ravelin::Pose3d>("base_link_frame")));
  
  boost::shared_ptr<Ravelin::Pose3d> base_horizontal_frame;
  base_horizontal_frame = boost::shared_ptr<Ravelin::Pose3d>( new Ravelin::Pose3d(
                                                                ctrl->get_data<Ravelin::Pose3d>("base_horizontal_frame")));
  
  base_horizontal_frame->update_relative_pose(Pacer::GLOBAL);
  
  std::vector<std::string> ik_feet;
  for(int i=0;i<NUM_FEET;i++){
    std::string& foot_name = foot_names_[i];
    OUT_LOG(logDEBUG1) << foot_name ;

    Ravelin::Origin3d x,xd,xdd;
    ctrl->get_end_effector_value(foot_name,Pacer::Controller::position_goal,x);
    if(x.norm() < Pacer::NEAR_ZERO)
      continue;
    ctrl->get_end_effector_value(foot_name,Pacer::Controller::velocity_goal,xd);
    ctrl->get_end_effector_value(foot_name,Pacer::Controller::acceleration_goal,xdd);

    Ravelin::Vector3d x_base(x,base_frame);
    Ravelin::Vector3d x_global = Ravelin::Pose3d::transform_point(Pacer::GLOBAL,x_base);

    Ravelin::Vector3d xd_base(xd,base_frame);
    Ravelin::Vector3d xd_global = Ravelin::Pose3d::transform_vector(Pacer::GLOBAL,xd_base);
    xd = Ravelin::Origin3d(xd_global.data());
    VISUALIZE(RAY(  xd_global+x_global,   x_global,   Ravelin::Vector3d(1,0,1),0.005));

    Ravelin::Vector3d xdd_base(xdd,base_frame);
    Ravelin::Vector3d xdd_global = Ravelin::Pose3d::transform_vector(Pacer::GLOBAL,xdd_base);
    xdd = Ravelin::Origin3d(xdd_global.data());
    VISUALIZE(RAY( xdd_global+xd_global+x_global,   xd_global+x_global,   Ravelin::Vector3d(1,1,0),0.005));

    ////// enforce maximum reach on legs ////////////
    
    Ravelin::Origin3d base_joint;
    if(ctrl->get_data<Ravelin::Origin3d>(foot_name+".base",base_joint)){
      double max_reach = ctrl->get_data<double>(foot_name+".reach");
      
      Ravelin::Origin3d goal_from_base_joint = x - base_joint;
      double goal_reach = goal_from_base_joint.norm();
      OUT_LOG(logDEBUG1) << " goal_reach < max_reach : " << goal_reach<<  " < "  << max_reach ;
      
      if(goal_reach > max_reach){
        OUT_LOG(logDEBUG1) << foot_name << " goal reduced from: " << x ;
        x = base_joint + goal_from_base_joint * (max_reach/goal_reach);
        OUT_LOG(logDEBUG1) << " to: " << x ;
      }
    }
    
    foot_pos.push_back(x);
    foot_vel.push_back(xd);
    foot_acc.push_back(xdd);
    ik_feet.push_back(foot_name);
  }

  // if no feet to do IK for return
  if(ik_feet.size() == 0)
    return;
  
  Ravelin::VectorNd q;
  ctrl->get_generalized_value(Pacer::Controller::position,q);
  int N = ctrl->num_joint_dof();
  q[N] = 0;
  q[N+1] = 0;
  q[N+2] = 0;
  q[N+3] = 0;
  q[N+4] = 0;
  q[N+5] = 0;
  q[N+6] = 1;
  
  Ravelin::VectorNd q_goal, qd_goal, qdd_goal;
  ctrl->get_joint_generalized_value(Pacer::Controller::position_goal,q_goal);
  ctrl->get_joint_generalized_value(Pacer::Controller::velocity_goal,qd_goal);
  ctrl->get_joint_generalized_value(Pacer::Controller::acceleration_goal,qdd_goal);

  double TOL = ctrl->get_data<double>(plugin_namespace+".abs-err-tolerance");
  // This is calculated in global frame always (assume base_link is at origin)
  ctrl->end_effector_inverse_kinematics(ik_feet,foot_pos,foot_vel,foot_acc,q,
                                        q_goal,qd_goal,qdd_goal,TOL);

  ctrl->set_joint_generalized_value(Pacer::Controller::position_goal,q_goal);
  ctrl->set_joint_generalized_value(Pacer::Controller::velocity_goal,qd_goal);
  ctrl->set_joint_generalized_value(Pacer::Controller::acceleration_goal,qdd_goal);
}
void setup(){
}