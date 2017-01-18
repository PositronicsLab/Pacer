#include "plugin.h"
#include <Pacer/controller.h>
#include <Pacer/utilities.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sys/types.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>


void loop(){
//check to see whether in the actual limit testing loop which is when jac_count=0
//otherwise we are in jacobian matrix generation
//curr_vel is the current velocity when incrementing through all possible velocities from delta_v to max velocity
//in increments of delta_v
//fail_vel is determined by the plugin error_check_builder, and is the velocity a limit was exceeded at
  double currSpeed;
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  
  if(std::stod(getenv("jac_count"))==0)
  {currSpeed =std::stod(getenv("curr_vel"));}
  else
  {currSpeed =std::stod(getenv("fail_vel"));}


  //gets how long each recording session should be
  double max_duration;
  ctrl->get_data<double>(plugin_namespace+".duration",max_duration);
  
  boost::shared_ptr<Ravelin::Pose3d>
  environment_frame(new Ravelin::Pose3d());
  VISUALIZE(POSE(*environment_frame.get(),0.1));
  
  boost::shared_ptr<Ravelin::Pose3d>
  base_horizontal_frame(new Ravelin::Pose3d(ctrl->get_data<Ravelin::Pose3d>("base_horizontal_frame")));
  
  Ravelin::Vector3d goto_point = Ravelin::Vector3d(100,0,0);
  Ravelin::Vector3d com(base_horizontal_frame->x.data());
  
  Ravelin::Vector3d goto_direction =
  Ravelin::Vector3d(goto_point[0],goto_point[1],0,environment_frame)
  - Ravelin::Vector3d(com[0],com[1],0,environment_frame);
  
  goto_direction = Ravelin::Pose3d::transform_vector(base_horizontal_frame,goto_direction);
  goto_direction.normalize();
  
  Ravelin::Origin3d command_SE2(goto_direction[0]*currSpeed,0,0);
  ctrl->set_data<Ravelin::Origin3d>("SE2_command",command_SE2);
  
  
  double duration =std::stod(getenv("duration"));
  duration+=0.001;
  
  std::ostringstream s;
  s << duration;
  std::string line=s.str();
  setenv("duration",line.c_str(),1);
  if(std::stod(getenv("jac_count"))==0)
  {
    //if we've reached the end of the current gait, and there are still velocities to record then go again
    if(duration>=max_duration && std::stod(getenv("curr_vel"))<std::stod(getenv("max_vel")))
    {
      std::string line2=getenv("BUILDER_CAPT_PATH");
      
      line2+="/run.sh";
      
      
      
      
      
      execl(line2.c_str(), line2.c_str(), (char *) 0);
    }
    
    
    
    //if we've finished generating poseSets for all velocities, move on to the limit testing step
    else if(duration>=max_duration && std::stod(getenv("curr_vel"))>=std::stod(getenv("max_vel")))
    {
      
      std::string line3=getenv("BUILDER_SCRIPT_PATH");
      
      line3+="/setup-plugins-play.sh";
      
      
      setenv("curr_vel",getenv("delta_v"),1);
      
      
      
      execl("/bin/sh","sh","-c" , line3.c_str(), (char *) 0);
      
      
      
      
    }
  }
  else
  {
    //this is the case where we are in jacobian matrix generation
    if(duration>=max_duration)
    {
      
      std::string line3=getenv("BUILDER_SCRIPT_PATH");
      
      line3+="/setup-plugins-play.sh";
      //make sure the file we open is the file we failed at
      setenv("curr_vel",getenv("fail_vel"),1);
      execl("/bin/sh","sh","-c" , line3.c_str(), (char *) 0);
    }
    
    
  }
}

void setup(){
  
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  setenv("duration","0",1);
  //if in normal limit testing loop(jac_count=0), increment the velocity by delta_v and then set the environment variable to the new value
  if(std::stod(getenv("jac_count"))==0)
  {
    double currSpeed =std::stod(getenv("curr_vel"));
    currSpeed+=std::stod(getenv("delta_v"));
    
    std::ostringstream s;
    s << currSpeed;
    std::string line=s.str();
    
    setenv("curr_vel",line.c_str(),1);
    
    
    s.clear();
    s.str(std::string());
  }
}
