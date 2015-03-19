/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/controller.h>
#include <Pacer/utilities.h>

std::string plugin_namespace;

void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t){
  OUT_LOG(logDEBUG) << ">> ik-feet";
  Ravelin::VectorNd 
    q_goal = ctrl->get_data<Ravelin::VectorNd>("init.q"),q;
  Ravelin::VectorNd
    qd_goal = Ravelin::VectorNd::zero(q_goal.rows()),
    qdd_goal = Ravelin::VectorNd::zero(q_goal.rows());
  
  std::vector<std::string>
      &foot_names = Utility::get_variable<std::vector<std::string> >("init.end-effector.id");
      
  int NUM_FEET = foot_names.size();
  std::vector<Ravelin::Vector3d>
      foot_pos(NUM_FEET),
      foot_vel(NUM_FEET),
      foot_acc(NUM_FEET);
      
  for(int i=0;i<NUM_FEET;i++){     
    //NOTE: Change so some eefs can be non-feet 
    foot_pos[i] = ctrl->get_data<Ravelin::Vector3d>(foot_names[i]+".goal.x");
    foot_vel[i] = ctrl->get_data<Ravelin::Vector3d>(foot_names[i]+".goal.xd");
    foot_acc[i] = ctrl->get_data<Ravelin::Vector3d>(foot_names[i]+".goal.xdd");
    OUTLOG(foot_pos[i],foot_names[i]+".goal.x",logDEBUG);
    OUTLOG(foot_vel[i],foot_names[i]+".goal.xd",logDEBUG);
    OUTLOG(foot_acc[i],foot_names[i]+".goal.xdd",logDEBUG);
  }
  
  ctrl->get_joint_generalized_value(Pacer::Controller::position,q);
      OUTLOG(q,"Current_q(ik)",logDEBUG);

  ctrl->end_effector_inverse_kinematics(foot_names,foot_pos,foot_vel,foot_acc,q,
                                        q_goal,qd_goal,qdd_goal);
  
  OUTLOG(q_goal,"goal_q"  ,logDEBUG);
  OUTLOG(qd_goal,"goal_qd"  ,logDEBUG);
  OUTLOG(qdd_goal,"goal_qdd"  ,logDEBUG);

  ctrl->set_joint_generalized_value(Pacer::Controller::position_goal,q_goal);
  ctrl->set_joint_generalized_value(Pacer::Controller::velocity_goal,qd_goal);
  ctrl->set_joint_generalized_value(Pacer::Controller::acceleration_goal,qdd_goal);
  OUT_LOG(logDEBUG) << "<< ik-feet";
}

/** This is a quick way to register your plugin function of the form:
  * void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t)
  */
#include "register_plugin"
