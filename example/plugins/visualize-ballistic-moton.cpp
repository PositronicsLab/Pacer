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
  VectorNd pos,vel,acc;
  
  ctrl->get_base_value(Pacer::Controller::position,pos);
  ctrl->get_base_value(Pacer::Controller::velocity,vel);
  ctrl->get_base_value(Pacer::Controller::acceleration,acc);
  
  Ravelin::Pose3d P = Utility::vec_to_pose(pos);
  Ravelin::Origin3d xd(vel.segment(0,3).data());
  Ravelin::Origin3d xdd(0,0,-9.8);
  
  double min_t = 0;
  double max_t = 1;
  double dt = (max_t-min_t)/40;
  for(double t=min_t ; t<=max_t ; t += dt){
    P.x += xd*dt;
    xd += xdd*dt;
    if(P.x[2] > 0){
      Utility::visualize.push_back( Pacer::VisualizablePtr( new Pacer::Point( Vector3d(P.x.data()),   Vector3d(1,0,1),0.05)));
    } else {
      break;
    }
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

extern "C" {
  void init(const boost::shared_ptr<Pacer::Controller> ctrl, const char* name){
    plugin_namespace = std::string(std::string(name));
    
    int priority = ctrl->get_data<double>(plugin_namespace+".priority");
    
    ctrl->add_plugin_update(priority,name,&update);
  }
}
