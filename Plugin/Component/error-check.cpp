
#include <Pacer/controller.h>
#include "plugin.h"

void loop(){
boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  
  Ravelin::VectorNd generalized_fext = ctrl->get_generalized_value(Pacer::Robot::load);

  // Unstable simulation.
  if(generalized_fext.norm() > 1e8)
    throw std::runtime_error("Robot exploded!");

  // Fell off edge
  Ravelin::Vector3d x;
  ctrl->get_data<Ravelin::Vector3d>("base.state.x",x);
  if(x[2] < -1.0)
    throw std::runtime_error("Robot fell!");

  boost::shared_ptr<Ravelin::Pose3d> base_link_frame(new Ravelin::Pose3d(ctrl->get_data<Ravelin::Pose3d>("base_link_frame")));

  Ravelin::Vector3d up(0,0,1,base_link_frame);
  up = Ravelin::Pose3d::transform_vector(Pacer::GLOBAL,up);
  if(up[2] < 0)
    throw std::runtime_error("Robot flipped over!");
    
  std::vector< boost::shared_ptr< Pacer::Robot::contact_t> > c;
  ctrl->get_link_contacts("BODY0",c);
  if(c.size() != 0)
    throw std::runtime_error("Robot body contacted ground!");
    

}
void setup(){
}