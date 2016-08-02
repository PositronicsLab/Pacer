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
  
  Origin3d x = ctrl->get_data<Ravelin::Origin3d>("HAND.state.x");

  static double last_time = t;
  double dt = t - last_time;
  last_time = t;
  
  const double speed = 1;
  double TOL = speed * dt * 10;
  
  static int STAGE = 0;

  std::string command;
  ctrl->get_data<std::string>("pick.command",command);
  
  double grip_origin = 0;
  ctrl->get_data<double>(plugin_namespace+".resting-angle",grip_origin);

  Ravelin::VectorNd current_robot_q, current_robot_qd;
  ctrl->get_joint_generalized_value(Pacer::Controller::position,current_robot_q);
  ctrl->get_joint_generalized_value(Pacer::Controller::velocity,current_robot_qd);

  const int NDOFS = current_robot_q.rows();

  static double grip_size = current_robot_q[NDOFS-4];

  double grip_offset = 0;
  ctrl->get_data<double>(plugin_namespace+".grip-angle",grip_offset);
 

    double diff = 0;
  if (command.compare("grab") == 0) {
    diff = grip_size - (grip_origin + grip_offset);
    std::cout << "Grasp: Grabbing" << std::endl;
  } else if (command.compare("release") == 0) {
    diff = grip_size - grip_origin;
  } else {
    std::cout << "Grasp: Do nothing" << std::endl;
  }
  
  if( fabs(diff) > TOL ){
    grip_size -= Utility::sign(diff)*speed*dt;
    std::cout << "Grasp: grip_size = " << grip_size << std::endl;
  }
  
  Ravelin::VectorNd desired_robot_q, desired_robot_qd;
  desired_robot_q = current_robot_q;
  desired_robot_qd = current_robot_qd;
  
  OUTLOG(current_robot_q,"current_robot_q",logDEBUG);
  OUTLOG(current_robot_qd,"current_robot_qd",logDEBUG);


  for(int i=0;i<4;i++){
    desired_robot_q[NDOFS-4+i] = grip_size;
    desired_robot_qd[NDOFS-4+i] = 0;
  }

  OUTLOG(desired_robot_q,"desired_robot_q",logDEBUG);
  OUTLOG(desired_robot_qd,"desired_robot_qd",logDEBUG);

  Ravelin::VectorNd error_robot_q;
  Ravelin::VectorNd error_robot_qd;
  (error_robot_q  = current_robot_q)  -= desired_robot_q;
  (error_robot_qd = current_robot_qd) -= desired_robot_qd;
  
  OUTLOG(error_robot_q,"error_robot_q",logDEBUG);
  OUTLOG(error_robot_qd,"error_robot_qd",logDEBUG);
  
  double Kp = 0, Kv = 0;
  ctrl->get_data<double>(plugin_namespace+".kp",Kp);
  ctrl->get_data<double>(plugin_namespace+".kv",Kv);

  Ravelin::VectorNd feedback_force, feedback_force_p, feedback_force_v;
  (feedback_force_p = error_robot_q) *= -Kp;
  (feedback_force_v = error_robot_qd) *= -Kv;
  (feedback_force = feedback_force_p ) += feedback_force_v;

  OUTLOG(feedback_force_p,"feedback_force_p",logDEBUG);
  OUTLOG(feedback_force_v,"feedback_force_v",logDEBUG);
  
  OUTLOG(feedback_force,"feedback_force",logDEBUG);
  
  feedback_force.segment(0,NDOFS-5).set_zero();
  
  Ravelin::VectorNd existing_feedback_force;
  ctrl->get_joint_generalized_value(Pacer::Controller::load_goal,existing_feedback_force);
  for(int i=0;i<NDOFS-4;i++)
    feedback_force[i] = existing_feedback_force[i];
  ctrl->set_joint_generalized_value(Pacer::Controller::load_goal,feedback_force);

}

void setup(){
}
