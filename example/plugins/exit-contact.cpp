
#include <Pacer/controller.h>
#include "plugin.h"

void loop(){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);

  std::vector< boost::shared_ptr<const Pacer::Robot::contact_t> > c;
  if(ctrl->get_all_contacts(c) != 0){
    Ravelin::VectorNd position;
    ctrl->get_base_value(Pacer::Robot::position,position);
    Ravelin::Origin3d rpy;
    Ravelin::Quatd(position[3],position[4],position[5],position[6]).to_rpy(rpy);
    std::cerr << "Final Time: " << t << " : " << Ravelin::Origin3d(position[0],position[1],position[2]) << " " << rpy << std::endl;
    exit(0);
  }
  
  
}
void setup(){
}