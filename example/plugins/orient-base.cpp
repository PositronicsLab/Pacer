#include <Pacer/controller.h>
#include <Pacer/utilities.h>

#include "plugin.h"

using namespace Ravelin;

void loop(){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  
  static double last_time = t;
  double dt = t - last_time;
  last_time = t;
  boost::shared_ptr<Pose3d>
    base_link_frame(new Pose3d(ctrl->get_data<Pose3d>("base_link_frame"))),
    base_horizontal_frame(new Pose3d(ctrl->get_data<Pose3d>("base_horizontal_frame")));
  
  base_horizontal_frame->update_relative_pose(Pacer::GLOBAL);
  base_link_frame->update_relative_pose(Pacer::GLOBAL);
  
  std::vector<double>
    input_gait_pose = ctrl->get_data<std::vector<double> >(plugin_namespace+".pose");

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
void setup(){
}