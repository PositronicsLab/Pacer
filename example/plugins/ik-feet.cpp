/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/controller.h>
#include <Pacer/utilities.h>

std::string plugin_namespace;

void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t){
  Ravelin::VectorNd 
    q_goal = ctrl->get_data<Ravelin::VectorNd>("init.q"),q;
  Ravelin::VectorNd
    qd_goal = Ravelin::VectorNd::zero(q_goal.rows()),
    qdd_goal = Ravelin::VectorNd::zero(q_goal.rows());
  
  std::vector<std::string>
      foot_names = ctrl->get_data<std::vector<std::string> >("init.end-effector.id");
      
  int NUM_FEET = foot_names.size();
  std::vector<Ravelin::Vector3d>
      foot_pos(NUM_FEET),
      foot_vel(NUM_FEET),
      foot_acc(NUM_FEET);
      
  for(int i=0;i<NUM_FEET;i++){     
    //NOTE: Change so some eefs can be non-feet 
    if(!ctrl->get_data<Ravelin::Vector3d>(foot_names[i]+".goal.x",foot_pos[i]))
      return;
    ctrl->get_data<Ravelin::Vector3d>(foot_names[i]+".goal.xd",foot_vel[i]);
    ctrl->get_data<Ravelin::Vector3d>(foot_names[i]+".goal.xdd",foot_acc[i]);
    foot_pos[i].pose = Pacer::GLOBAL;
  }
  
  ctrl->get_generalized_value(Pacer::Controller::position,q);
  int N = q.size() - Pacer::NEULER;
  q[N] = 0;
  q[N+1] = 0;
  q[N+2] = 0;
  q[N+3] = 0;
  q[N+4] = 0;
  q[N+5] = 0;
  q[N+6] = 1;
  
  double TOL = ctrl->get_data<double>(plugin_namespace+".abs-err-tolerance");
  // This is calculated in global frame always (assume base_link is at origin)
  ctrl->end_effector_inverse_kinematics(foot_names,foot_pos,foot_vel,foot_acc,q,
                                        q_goal,qd_goal,qdd_goal,TOL);

  ctrl->set_joint_generalized_value(Pacer::Controller::position_goal,q_goal);
  ctrl->set_joint_generalized_value(Pacer::Controller::velocity_goal,qd_goal);
  ctrl->set_joint_generalized_value(Pacer::Controller::acceleration_goal,qdd_goal);
}

/** This is a quick way to register your plugin function of the form:
  * void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t)
  */
#include "register-plugin"
