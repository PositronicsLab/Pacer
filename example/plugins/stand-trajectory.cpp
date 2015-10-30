/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/controller.h>
#include <Pacer/utilities.h>

std::string plugin_namespace;
using namespace Ravelin;

void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t){
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
    Ravelin::Vector3d x = ctrl->get_data<Ravelin::Vector3d>(foot_names[i]+".init.x"),
      xd(0,0,0,base_frame), 
      xdd(0,0,0,base_frame);
    x.pose = base_frame;
    ctrl->set_data<Ravelin::Vector3d>(foot_names[i]+".goal.x",x);
    ctrl->set_data<Ravelin::Vector3d>(foot_names[i]+".goal.xd",xd);
    ctrl->set_data<Ravelin::Vector3d>(foot_names[i]+".goal.xdd",xdd);
    ctrl->set_data<bool>(foot_names[i]+".stance",true);
  }
}

/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
/** This is a quick way to register your plugin function of the form:
 * void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t)
 * void Deconstruct(const boost::shared_ptr<Pacer::Controller>& ctrl)
 */

void update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t){
  static int ITER = 0;
  int RTF = (int) ctrl->get_data<double>(plugin_namespace+".real-time-factor");
  if(ITER%RTF == 0)
    Update(ctrl,t);
  ITER+=1;
}

void deconstruct(const boost::shared_ptr<Pacer::Controller>& ctrl, double t){
  std::vector<std::string>
  foot_names = ctrl->get_data<std::vector<std::string> >("init.end-effector.id");
  
  int NUM_FEET = foot_names.size();
  
  for(int i=0;i<NUM_FEET;i++){
    ctrl->remove_data(foot_names[i]+".goal.x");
    ctrl->remove_data(foot_names[i]+".goal.xd");
    ctrl->remove_data(foot_names[i]+".goal.xdd");
    ctrl->remove_data(foot_names[i]+".stance");
  }
  
}

extern "C" {
  void init(const boost::shared_ptr<Pacer::Controller> ctrl, const char* name){
    plugin_namespace = std::string(std::string(name));
    
    int priority = ctrl->get_data<double>(plugin_namespace+".priority");
    
    ctrl->add_plugin_update(priority,name,&update);
    ctrl->add_plugin_deconstructor(name,&deconstruct);
  }
}
