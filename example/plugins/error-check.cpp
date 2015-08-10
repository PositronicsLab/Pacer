
#include <Pacer/controller.h>
std::string plugin_namespace;

void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t){
  
  Ravelin::VectorNd generalized_fext = ctrl->get_generalized_value(Pacer::Robot::load);

  // Unstable simulation.
  if(generalized_fext.norm() > 1e8)
    throw std::runtime_error("Robot flipped over!");

  // Fell off edge
  Ravelin::Vector3d x;
  ctrl->get_data<Ravelin::Vector3d>("base.state.x",x);
  if(x[2] < -1.0)
    throw std::runtime_error("Robot fell!");

  boost::shared_ptr<Ravelin::Pose3d> base_link_frame(new Ravelin::Pose3d(ctrl->get_data<Ravelin::Pose3d>("base_link_frame")));

  Ravelin::Vector3d up(0,0,1,base_link_frame);
  up = Ravelin::Pose3d::transform_vector(Moby::GLOBAL,up);
  if(up[2] < 0)
    throw std::runtime_error("Robot flipped over!");

}

/** This is a quick way to register your plugin function of the form:
 * void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t)
 */
#include "register-plugin"
