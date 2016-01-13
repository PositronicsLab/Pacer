/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/controller.h>
#include <Pacer/utilities.h>

#include <time.h>

#include "plugin.h"
using namespace Ravelin;
/// Gets the current time (as a floating-point number)
double get_current_time()
{
  const double MICROSEC = 1.0/1000000;
  timeval t;
  gettimeofday(&t, NULL);
  return (double) t.tv_sec + (double) t.tv_usec * MICROSEC;
}

void loop(){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  static VectorNd q_init = ctrl->get_data<Ravelin::VectorNd>("init.q");
  const int N = q_init.size();
  VectorNd q_goal   = Ravelin::VectorNd::zero(N);
  VectorNd qd_goal  = Ravelin::VectorNd::zero(N);
  VectorNd qdd_goal = Ravelin::VectorNd::zero(N);

  double f = ctrl->get_data<double>(plugin_namespace+".frequency");
  double a = ctrl->get_data<double>(plugin_namespace+".amplitude");

  double time = t;
  

  bool check_wall_time = false;
  ctrl->get_data<bool>(plugin_namespace+".realtime",check_wall_time); 
  if(check_wall_time){
    static double start_time = get_current_time(); 
    double this_time = get_current_time(); 
    time = (this_time - start_time);
  }
  
  ctrl->set_data<double>("trajectory.time",time);

  for(int i=0;i<N;i++){
    q_goal[i]    = q_init[i] +    sin( time * f ) * M_PI * a;
    qd_goal[i]   = f *         cos( time * f ) * M_PI * a;
    qdd_goal[i]  = f * f * -sin( time * f ) * M_PI * a;
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
}
