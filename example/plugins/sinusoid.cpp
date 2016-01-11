/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/controller.h>
#include <Pacer/utilities.h>

#include "plugin.h"
using namespace Ravelin;

  VectorNd q_init;
  int N;
void loop(){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  VectorNd q_goal   = Ravelin::VectorNd::zero(N);
  VectorNd qd_goal  = Ravelin::VectorNd::zero(N);
  VectorNd qdd_goal = Ravelin::VectorNd::zero(N);


  for(int i=0;i<N;i++){
    q_goal[i]    = q_init[i] +    sin( t * 16.0 ) * M_PI/8.0;
    qd_goal[i]   = 16.0 *         cos( t * 16.0 ) * M_PI/8.0;
    qdd_goal[i]  = 16.0 * 16.0 * -sin( t * 16.0 ) * M_PI/8.0;
  }


  ctrl->set_joint_generalized_value(Pacer::Controller::position_goal,q_goal);
  ctrl->set_joint_generalized_value(Pacer::Controller::velocity_goal,qd_goal);
  ctrl->set_joint_generalized_value(Pacer::Controller::acceleration_goal,qdd_goal);

  OUT_LOG(logINFO) << "New q = " << q_goal;
  OUT_LOG(logINFO) << "New qd = " << qd_goal;
  OUT_LOG(logINFO) << "New qdd = " << qdd_goal;
}

void setup(){
  OUT_LOG(logINFO) << "INITING SINUSOID";
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  q_init = ctrl->get_data<Ravelin::VectorNd>("init.q");
  N = q_init.size();
}
