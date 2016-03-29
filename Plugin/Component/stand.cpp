/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/controller.h>
#include <Pacer/utilities.h>

#include "plugin.h"
using namespace Ravelin;

void loop(){
boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  // Alpha calculation
  VectorNd q_goal = ctrl->get_data<Ravelin::VectorNd>("init.q"),
  qd_goal = Ravelin::VectorNd::zero(q_goal.rows()),
  qdd_goal = Ravelin::VectorNd::zero(q_goal.rows());
  
  ctrl->set_joint_generalized_value(Pacer::Controller::position_goal,q_goal);
  ctrl->set_joint_generalized_value(Pacer::Controller::velocity_goal,qd_goal);
  ctrl->set_joint_generalized_value(Pacer::Controller::acceleration_goal,qdd_goal);
  static std::vector<std::string>
      foot_names = ctrl->get_data<std::vector<std::string> >("init.end-effector.id");

  boost::shared_ptr<Ravelin::Pose3d> base_frame = boost::shared_ptr<Ravelin::Pose3d>( new Ravelin::Pose3d(
        ctrl->get_data<Ravelin::Pose3d>("base_link_frame")));
  for(int i=0;i<foot_names.size();i++){
    Ravelin::Origin3d x = ctrl->get_data<Ravelin::Origin3d>(foot_names[i]+".init.x"),
      xd(0,0,0),
      xdd(0,0,0);
    ctrl->set_end_effector_value(foot_names[i],Pacer::Controller::position_goal,x);
    ctrl->set_end_effector_value(foot_names[i],Pacer::Controller::velocity_goal,xd);
    ctrl->set_end_effector_value(foot_names[i],Pacer::Controller::acceleration_goal,xdd);
    ctrl->set_data<bool>(foot_names[i]+".stance",true);
  }
}

void setup(){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  const  std::vector<std::string>
  eef_names_ = ctrl->get_data<std::vector<std::string> >("init.end-effector.id");
  for(unsigned i=0;i<eef_names_.size();i++){
//    variable_names.push_back(eef_names_[i]+".stance");
  }
}