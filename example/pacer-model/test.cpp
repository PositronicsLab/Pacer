#include <Pacer/robot.h>
#include <map>
#include <string>
#include <Ravelin/Pose3d.h>
#include <Ravelin/SVector6d.h>
#include <Ravelin/Quatd.h>
#include <Ravelin/Origin3d.h>

using namespace Pacer;

int main(){
  //
  std::map<std::string,double> q,qd;

  q["0LF_X_1"   ] = 0.3927;
  q["0LF_Y_2"   ] = 0.7854;
  q["0LF_Y_3"   ] = 1.5708;
  q["0LF_ANKLE4"] =  0;
  q["0RF_X_1"   ] = -0.3927;
  q["0RF_Y_2"   ] = -0.7854;
  q["0RF_Y_3"   ] = -1.5708;
  q["0RF_ANKLE4"] =  0;
  q["0LH_X_1"   ] = -0.3927;
  q["0LH_Y_2"   ] = -0.7854;
  q["0LH_Y_3"   ] = -1.5708;
  q["0LH_ANKLE4"] =  0;
  q["0RH_X_1"   ] = 0.3927;
  q["0RH_Y_2"   ] = 0.7854;
  q["0RH_Y_3"   ] = 1.5708;
  q["0RH_ANKLE4"] =  0;

  qd["0LF_X_1"   ] = 0;
  qd["0LF_Y_2"   ] = 0;
  qd["0LF_Y_3"   ] = 0;
  qd["0LF_ANKLE4"] = 0;
  qd["0RF_X_1"   ] = 0;
  qd["0RF_Y_2"   ] = 0;
  qd["0RF_Y_3"   ] = 0;
  qd["0RF_ANKLE4"] = 0;
  qd["0LH_X_1"   ] = 0;
  qd["0LH_Y_2"   ] = 0;
  qd["0LH_Y_3"   ] = 0;
  qd["0LH_ANKLE4"] = 0;
  qd["0RH_X_1"   ] = 0;
  qd["0RH_Y_2"   ] = 0;
  qd["0RH_Y_3"   ] = 0;
  qd["0RH_ANKLE4"] = 0;
  Ravelin::SVector6d base_xd(1,0,0,0,0,0);
  boost::shared_ptr<Ravelin::Pose3d>
      base_pose = boost::shared_ptr<Ravelin::Pose3d>(
                    new Ravelin::Pose3d(
                      Ravelin::Quatd::identity(),
                      Ravelin::Origin3d(5,10,1),
                      Moby::GLOBAL)
                    );

  // Init a robot from the model we want
  std::string sdf_file("model.sdf");
  std::string init_file("init.xml");
  boost::shared_ptr<Robot> robot
      = boost::shared_ptr<Robot>(new Robot(sdf_file,init_file));

  robot
      = boost::shared_ptr<Robot>(new Robot(sdf_file,init_file));

  boost::shared_ptr<const RobotData> data =
            Robot::gen_vars_from_model(
                  q,qd,
                  boost::shared_ptr<const Ravelin::Pose3d>(base_pose),
                  base_xd,
                  robot);

  std::cout << data->center_of_mass_x;

  base_pose->x = Ravelin::Origin3d(0,0,0);
  // Test if state changes
  data = Robot::gen_vars_from_model(
                q,qd,
                boost::shared_ptr<const Ravelin::Pose3d>(base_pose),
                base_xd,
                robot);

  std::cout << data->center_of_mass_x;

  // Test if Jacobians work
  std::cout << data->N;

  std::vector<EndEffector>& eefs = robot->get_end_effectors();

  EndEffector& ef = eefs[0];
  ef.active = true;
  ef.normal.push_back(Ravelin::Vector3d(0,0,1));
  ef.tan1.push_back(Ravelin::Vector3d(1,0,0));
  ef.tan2.push_back(Ravelin::Vector3d(0,1,0));
  ef.point.push_back(Ravelin::Vector3d(ef.link->get_pose()->x.data()));

  data = Robot::gen_vars_from_model(
                q,qd,
                boost::shared_ptr<const Ravelin::Pose3d>(base_pose),
                base_xd,
                robot);

  std::cout << data->N;


}
