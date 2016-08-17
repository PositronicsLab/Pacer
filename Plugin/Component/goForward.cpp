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

double currSpeed;
boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
if(std::stod(getenv("jac_count"))==0)
{currSpeed =std::stod(getenv("curr_vel"));}
else
{currSpeed =std::stod(getenv("fail_vel"));}
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
std::cout << "\n" << "\n" << duration << "\n" << "\n";
if(std::stod(getenv("jac_count"))==0)
{
if(duration>=max_duration && std::stod(getenv("curr_vel"))<std::stod(getenv("max_vel")))
{
	std::string line2=getenv("BUILDER_CAPT_PATH");
	
	line2+="/run.sh";

	


       
        execl(line2.c_str(), line2.c_str(), (char *) 0);
}
	

        

else if(duration>=max_duration && std::stod(getenv("curr_vel"))==std::stod(getenv("max_vel")))
	{
                
		std::string line3=getenv("BUILDER_SCRIPT_PATH");
                
	        line3+="/setup-plugins-play.sh";
                
        	
		setenv("curr_vel",getenv("delta_v"),1);


        	
        	execl("/bin/sh","sh","-c" , line3.c_str(), (char *) 0);

		

               
	}
   }
   else
   {
        if(duration>=max_duration)
	{
                
		std::string line3=getenv("BUILDER_SCRIPT_PATH");
                
	        line3+="/setup-plugins-play.sh";
        	
		setenv("curr_vel",getenv("fail_vel"),1);
        	execl("/bin/sh","sh","-c" , line3.c_str(), (char *) 0);
	}


   }
}

void setup(){
 
boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
setenv("duration","0",1);
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
