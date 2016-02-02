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
  static double start_time = t;
  double duration = ctrl->get_data<double>(plugin_namespace + ".time-to-pose");
  VectorNd q_goal = ctrl->get_data<Ravelin::VectorNd>("init.q"),
  qd_goal = Ravelin::VectorNd::zero(q_goal.rows()),
  qdd_goal = Ravelin::VectorNd::zero(q_goal.rows());
  
  static VectorNd q_start;
  if(t == start_time){
    ctrl->get_joint_generalized_value(Pacer::Controller::position,q_start);
  }
  
  double alpha = ((start_time+duration)-t)/duration;

  if (q_start.rows() == 0)
  
  OUTLOG(q_start, "q_start", logDEBUG);
  OUTLOG(q_goal, "q_goal", logDEBUG);
  OUTLOG(alpha, "alpha", logDEBUG);
  
  if(alpha <= 1 && alpha >= 0){
    VectorNd q_target = q_start;
    q_target *= 1-alpha;
    q_goal *= alpha;
    q_target += q_goal;
    
    OUTLOG(q_target, "q_target", logDEBUG);
    
    ctrl->set_joint_generalized_value(Pacer::Controller::position_goal,q_target);
    ctrl->set_joint_generalized_value(Pacer::Controller::velocity_goal,qd_goal);
    ctrl->set_joint_generalized_value(Pacer::Controller::acceleration_goal,qdd_goal);

    return;
  } else if (alpha > 1.0){
    ctrl->set_joint_generalized_value(Pacer::Controller::position_goal,q_start);
    ctrl->set_joint_generalized_value(Pacer::Controller::velocity_goal,qd_goal);
    ctrl->set_joint_generalized_value(Pacer::Controller::acceleration_goal,qdd_goal);

    return;
  } else { // alpha < 0
    // quit
    ctrl->set_joint_generalized_value(Pacer::Controller::position_goal,q_goal);
    ctrl->set_joint_generalized_value(Pacer::Controller::velocity_goal,qd_goal);
    ctrl->set_joint_generalized_value(Pacer::Controller::acceleration_goal,qdd_goal);

    ctrl->open_plugin("stand");
    ctrl->close_plugin("reset-trajectory");
  }
}

void setup(){
}