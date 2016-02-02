/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/controller.h>
#include <Pacer/utilities.h>
#include "plugin.h"
using namespace Ravelin;

VectorNd
q_goal, qd_goal, qdd_goal;

std::map<std::string,Origin3d>
x_goal, xd_goal, xdd_goal;

void loop(){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  
  // Joints
  ctrl->set_joint_generalized_value(Pacer::Controller::position_goal,q_goal);
  ctrl->set_joint_generalized_value(Pacer::Controller::velocity_goal,qd_goal);
  ctrl->set_joint_generalized_value(Pacer::Controller::acceleration_goal,qdd_goal);
  
  // End Effectors
  ctrl->set_foot_value(Pacer::Controller::position_goal,x_goal);
}

void setup(){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);

  q_goal = ctrl->get_joint_generalized_value(Pacer::Controller::position);
  qd_goal = Ravelin::VectorNd::zero(q_goal.rows());
  qdd_goal = Ravelin::VectorNd::zero(q_goal.rows());
  
  x_goal = ctrl->get_foot_value(Pacer::Controller::position_goal);
  for (std::map<std::string,Origin3d>::iterator it = x_goal.begin();  it != x_goal.end(); it++) {
    xd_goal[it->first] = Origin3d(0,0,0);
    xdd_goal[it->first] = Origin3d(0,0,0);
  }
}
