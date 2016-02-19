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

  std::vector<std::string>
  foot_names_ = ctrl->get_data<std::vector<std::string> >("init.end-effector.id");
  
  int NUM_FEET = foot_names_.size();
  std::vector<Ravelin::Origin3d>
      foot_pos,
      foot_vel,
      foot_acc;

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

    ////// enforce maximum reach on legs ////////////
    Ravelin::Origin3d base_joint = ctrl->get_data<Ravelin::Origin3d>(foot_name+".base");
    double max_reach = ctrl->get_data<double>(foot_name+".reach");
    
    Ravelin::Origin3d goal_from_base_joint = x - base_joint;
    double goal_reach = goal_from_base_joint.norm();
    OUT_LOG(logDEBUG1) << " goal_reach < max_reach : " << goal_reach<<  " < "  << max_reach ;
    
    if(goal_reach > max_reach){
      OUT_LOG(logDEBUG1) << foot_name << " goal reduced from: " << x ;
      x = base_joint + goal_from_base_joint * (max_reach/goal_reach);
      OUT_LOG(logDEBUG1) << " to: " << x ;
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
  int N = q.size() - Pacer::NEULER;
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
  // TODO: set q_goal dofs that dont have a foot that needs IK to joint_generalized_value(Pacer::Controller::position_goal
  OUT_LOG(logDEBUG1) << "q_goal = " << q_goal; 
  OUT_LOG(logDEBUG1) << "qd_goal = " << qd_goal; 
  OUT_LOG(logDEBUG1) << "qdd_goal = " << qdd_goal; 

  ctrl->set_joint_generalized_value(Pacer::Controller::position_goal,q_goal);
  ctrl->set_joint_generalized_value(Pacer::Controller::velocity_goal,qd_goal);
  ctrl->set_joint_generalized_value(Pacer::Controller::acceleration_goal,qdd_goal);
}
void setup(){
}