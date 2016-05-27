#include <Pacer/controller.h>
#include <Pacer/utilities.h>
#include "plugin.h"

std::string target_file;
std::string camera_file;
bool new_file = true;
void loop(){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);

  Ravelin::Vector3d base_state_x(0,0,0,Pacer::GLOBAL);
  Ravelin::Quatd base_state_q;
  // collect data
  ctrl->get_data<Ravelin::Vector3d>("base.state.x",base_state_x);
  ctrl->get_data<Ravelin::Quatd>("base.state.q",base_state_q);
  
  std::vector<double> camera_x, camera_rpy;
  ctrl->get_data< std::vector<double> >(plugin_namespace+".xyz",camera_x);

  ctrl->get_data< std::vector<double> >(plugin_namespace+".rpy",camera_rpy);

  boost::shared_ptr<Ravelin::Pose3d> camera_pose
    (new Ravelin::Pose3d
     (Ravelin::Quatd::rpy(camera_rpy[0],camera_rpy[1],camera_rpy[2]),
      Ravelin::Origin3d(camera_x[0],camera_x[1],camera_x[2])+
      Ravelin::Origin3d(base_state_x.data()),
      Pacer::GLOBAL
     )
    );

  Ravelin::Vector3d camera_position(0,0,0,camera_pose);

  Ravelin::Vector3d camera_position_global = Ravelin::Pose3d::transform_point(Pacer::GLOBAL,camera_position);

//  VISUALIZE(POINT(Ravelin::Vector3d(camera_position_global[0],camera_position_global[1],camera_position_global[2]),
//                Ravelin::Vector3d(base_state_x[0],base_state_x[1],base_state_x[2]),
//                Ravelin::Vector3d(0,0,0),0.2));
  
  // File pointers
  FILE * pFile_target;
  FILE * pFile_camera;
  
  // Open files for write
  if(new_file){
    pFile_target = fopen(target_file.c_str(),"w");
    pFile_camera = fopen(camera_file.c_str(),"w");
    new_file = false;
  } else {
    pFile_target = fopen(target_file.c_str(),"a");
    pFile_camera = fopen(camera_file.c_str(),"a");
  }
  
  /// Write out:
  fprintf(pFile_target, "%f %f %f\n", base_state_x[0],base_state_x[1],base_state_x[2]);
  fprintf(pFile_camera, "%f %f %f\n", camera_position_global[0],camera_position_global[1],camera_position_global[2]);
  
  // Close files after write
  fclose (pFile_target);
  fclose (pFile_camera);
}

void setup(){
  // determine name of output files
  static int pid = getpid();
  char buffer[9];
  sprintf(buffer,"%06d",pid);
  target_file = std::string("target-"+std::string(buffer)+".log");
  camera_file = std::string("camera-"+std::string(buffer)+".log");
}
