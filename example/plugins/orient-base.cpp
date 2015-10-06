#include <Pacer/controller.h>
#include <Pacer/utilities.h>

std::string plugin_namespace;
using namespace Ravelin;

boost::shared_ptr<Pacer::Controller> ctrl;

void Update(const boost::shared_ptr<Pacer::Controller>& ctrl_ptr, double t){
  ctrl = ctrl_ptr;
  
  static std::vector<std::string>
  eef_names_ = ctrl->get_data<std::vector<std::string> >("init.end-effector.id");
  
  static double last_time = t;
  double dt = t - last_time;
  last_time = t;
  
  for(int i=0;i<eef_names_.size();i++){
    std::vector< boost::shared_ptr< const Pacer::Robot::contact_t> > c;
    ctrl->get_link_contacts(eef_names_[i],c);
    if (c.size() != 0) {
      continue;
    }
    // ELSE: move down to contact
    
    Vector3d x_foot,xd_foot,xdd_foot;
    
    ctrl->get_data<Ravelin::Vector3d>(eef_names_[i]+".goal.x",x_foot);
    //    ctrl->get_data<Ravelin::Vector3d>(eef_names_[i]+".goal.xd",xd_foot);
    //    ctrl->get_data<Ravelin::Vector3d>(eef_names_[i]+".goal.xdd",xdd_foot);

    
    
    ctrl->set_data<Ravelin::Vector3d>(eef_names_[i]+".goal.x",x_foot);
    ctrl->set_data<Ravelin::Vector3d>(eef_names_[i]+".goal.xd",xd_foot);
    //    ctrl->set_data<Ravelin::Vector3d>(eef_names_[i]+".goal.xdd",xdd_foot);
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
