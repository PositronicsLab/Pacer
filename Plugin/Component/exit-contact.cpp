
#include <Pacer/controller.h>
#include "plugin.h"

void loop(){
  Ravelin::VectorNd position;
  int num_contacts = 0;
  {
    boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
    
    std::vector< boost::shared_ptr<Pacer::Robot::contact_t> > c;
    num_contacts = ctrl->get_all_contacts(c);
    if(num_contacts != 0)
      ctrl->get_base_value(Pacer::Robot::position,position);
  }
  if(num_contacts != 0){
    Ravelin::Origin3d rpy;
    Ravelin::Quatd(position[3],position[4],position[5],position[6]).to_rpy(rpy);
    OUT_LOG(logERROR) << "Final Time: " << t << " : " << Ravelin::Origin3d(position[0],position[1],position[2]) << " " << rpy << std::endl;
    throw std::runtime_error("Robot contacted ground!");
  }
}
void setup(){
}