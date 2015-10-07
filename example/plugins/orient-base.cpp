#include <Pacer/controller.h>
#include <Pacer/utilities.h>

std::string plugin_namespace;
using namespace Ravelin;

boost::shared_ptr<Pacer::Controller> ctrl;

void Update(const boost::shared_ptr<Pacer::Controller>& ctrl_ptr, double t){
  ctrl = ctrl_ptr;
  
  static double last_time = t;
  double dt = t - last_time;
  last_time = t;
  boost::shared_ptr<Pose3d>
    base_link_frame(new Pose3d(ctrl->get_data<Pose3d>("base_link_frame"))),
    base_horizontal_frame(new Pose3d(ctrl->get_data<Pose3d>("base_horizontal_frame")));
  
  base_horizontal_frame->update_relative_pose(Pacer::GLOBAL);
  base_link_frame->update_relative_pose(Pacer::GLOBAL);
  
  std::vector<double>
    input_gait_pose = ctrl_ptr->get_data<std::vector<double> >(plugin_namespace+".pose");

  static boost::shared_ptr<Pose3d> stability_frame_offset(new Pose3d(Quatd::identity(),
                                                                     Origin3d(),
                                                                     base_link_frame));
  
  Pose3d base_stability_frame(
     Ravelin::Quatd::rpy(
                         input_gait_pose[3],
                         input_gait_pose[4],
                         input_gait_pose[5]
                         ),
     Origin3d(
              input_gait_pose[0] ,
              input_gait_pose[1] ,
              input_gait_pose[2]
              ),
     base_link_frame
  );
  
  static Ravelin::Origin3d rpy_correction(0,0,0);
  // Update base_stability_frame
  Ravelin::Origin3d rpy_base,rpy_goal;
  base_link_frame->q.to_rpy(rpy_base);
//  base_horizontal_frame->q.to_rpy(rpy_goal);
  
  
  rpy_correction = -dt*rpy_base;
  rpy_goal = rpy_base + rpy_correction;
  base_stability_frame.q = Quatd::rpy(rpy_goal[0],rpy_goal[1],0);
  
//  static Ravelin::Origin3d pos_correction(0,0,0);
//  Ravelin::Origin3d pos_base,pos_goal;
//
//  pos_base = base_link_frame.x;

  base_stability_frame.update_relative_pose(Pacer::GLOBAL);
  
  Utility::visualize.push_back( Pacer::VisualizablePtr( new Pacer::Pose(base_stability_frame,1,0.2)));

  ctrl->set_data<Pose3d>("base_stability_frame",base_stability_frame);
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
  ctrl->remove_data("base_stability_frame");
}

extern "C" {
  void init(const boost::shared_ptr<Pacer::Controller> ctrl, const char* name){
    plugin_namespace = std::string(std::string(name));
    
    int priority = ctrl->get_data<double>(plugin_namespace+".priority");
    
    ctrl->add_plugin_update(priority,name,&update);
    ctrl->add_plugin_deconstructor(name,&deconstruct);
  }
}
