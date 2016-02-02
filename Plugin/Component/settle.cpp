#include <Pacer/controller.h>
#include <Pacer/utilities.h>

#include "plugin.h"
using namespace Ravelin;

void loop(){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  
  static std::vector<std::string>
    eef_names_ = ctrl->get_data<std::vector<std::string> >("init.end-effector.id");
  
  static double last_time = t;
  double dt = t - last_time;
  last_time = t;
  
  for(int i=0;i<eef_names_.size();i++){
    std::vector< boost::shared_ptr< const Pacer::Robot::contact_t> > c;
    ctrl->get_link_contacts(eef_names_[i],c);
    
    OUT_LOG(logERROR) << "Foot " << eef_names_[i] << " has " << c.size() << " contacts";

    Origin3d x_foot,xd_foot,xdd_foot;
    xd_foot = Vector3d(0,0,0);

    if (c.size() != 0) {
      ctrl->set_data<Ravelin::Origin3d>(eef_names_[i]+".goal.xd",xd_foot);

      continue;
    }
    
    // ELSE: move down to contact
    
    
    ctrl->get_data<Ravelin::Origin3d>(eef_names_[i]+".goal.x",x_foot);
//    ctrl->get_data<Ravelin::Origin3d>(eef_names_[i]+".goal.xd",xd_foot);
//    ctrl->get_data<Ravelin::Origin3d>(eef_names_[i]+".goal.xdd",xdd_foot);
    
    double speed = 0.2; // m/s
    
    xd_foot = -Origin3d(0,0,speed);
    x_foot += dt*xd_foot;

    ctrl->set_data<Ravelin::Origin3d>(eef_names_[i]+".goal.x",x_foot);
    ctrl->set_data<Ravelin::Origin3d>(eef_names_[i]+".goal.xd",xd_foot);
//    ctrl->set_data<Ravelin::Origin3d>(eef_names_[i]+".goal.xdd",xdd_foot);
  }
}

void setup(){
}