/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <sstream>
#include <Pacer/controller.h>
#include <Pacer/utilities.h>

#include "plugin.h"

struct Gains
{
  double perr_sum;
  double kp;
  double kv;
  double ki;
};

std::vector<Gains> _gains;

void loop(){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  
  
  Ravelin::VectorNd
    q_des  = ctrl->get_joint_generalized_value(Pacer::Controller::position_goal),
    qd_des = ctrl->get_joint_generalized_value(Pacer::Controller::velocity_goal),
    q  = ctrl->get_joint_generalized_value(Pacer::Controller::position),
    qd = ctrl->get_joint_generalized_value(Pacer::Controller::velocity);
  
  OUTLOG(q,"joint_pid_q",logDEBUG);
  OUTLOG(qd,"joint_pid_qd",logDEBUG);
  OUTLOG(q_des,"joint_pid_q_des",logDEBUG);
  OUTLOG(qd_des,"joint_pid_qd_des",logDEBUG);
  
  qd_des.set_zero();
  
  // calculate errors
  Ravelin::VectorNd perr = q;
  perr -= q_des;
  Ravelin::VectorNd derr = qd;
  derr -= qd_des;
  
  assert(q.rows() == _gains.size());
  
  Ravelin::VectorNd u(q.rows());
  
  // calculate feedback forces
  for (int i=0; i< q.rows(); i++)
  {
    Gains& g = _gains[i];
    g.perr_sum += perr[i];
    double ierr = g.perr_sum;
    u[i] = -(perr[i]*g.kp + derr[i]*g.kv + ierr*g.ki);
  }
  
  // acceleration or force feeback?
  bool acceleration_ff = false;
  ctrl->get_data<bool>(plugin_namespace+".acceleration",acceleration_ff);

  // send to robot
  if(acceleration_ff){
    Ravelin::VectorNd u_robot = ctrl->get_joint_generalized_value(Pacer::Controller::acceleration_goal);
    OUTLOG(u,"joint_pid_U",logDEBUG);
    u_robot += u;
    ctrl->set_joint_generalized_value(Pacer::Controller::acceleration_goal,u_robot);
  } else {
    Ravelin::VectorNd u_robot = ctrl->get_joint_generalized_value(Pacer::Controller::load_goal);
    OUTLOG(u,"joint_pid_U",logDEBUG);
    u_robot += u;
    ctrl->set_joint_generalized_value(Pacer::Controller::load_goal,u_robot);
  }  
}


void setup(){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);

  std::map<std::string,std::vector<Gains> > gains;
  
  static std::vector<std::string>
  joint_names = ctrl->get_data<std::vector<std::string> >("init.joint.id");
  
  OUTLOG(joint_names,"joint_names",logERROR);
  
  static std::vector<double>
  Kp = ctrl->get_data<std::vector<double> >(plugin_namespace+".gains.kp"),
  Kv = ctrl->get_data<std::vector<double> >(plugin_namespace+".gains.kv"),
  Ki = ctrl->get_data<std::vector<double> >(plugin_namespace+".gains.ki"),
  dofs = ctrl->get_data<std::vector<double> >("init.joint.dofs");
  
  OUTLOG(Kp,"Kp",logERROR);
  OUTLOG(Kv,"Kv",logERROR);
  OUTLOG(Ki,"Ki",logERROR);
  OUTLOG(dofs,"dofs",logERROR);
  
  int num_dofs = std::accumulate ( dofs.begin( ) , dofs.end( ) , 0.0 ) ;
  assert(num_dofs == Kp.size());
  assert(num_dofs == Kv.size());
  assert(num_dofs == Ki.size());
  
  for(int i=0,ii=0;i<joint_names.size();i++){
    gains[joint_names[i]] = std::vector<Gains>(dofs[i]);
    for(int j=0;j<dofs[i];j++,ii++){
      gains[joint_names[i]][j].kp = Kp[ii];
      gains[joint_names[i]][j].kv = Kv[ii];
      gains[joint_names[i]][j].ki = Ki[ii];
      gains[joint_names[i]][j].perr_sum = 0;
    }
  }
  
  ctrl->convert_to_generalized<Gains>(gains,_gains);
  
  OUT_LOG(logERROR) << "Controller: " << plugin_namespace << " inited!";
}