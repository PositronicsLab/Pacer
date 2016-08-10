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

std::fstream myfile;
   myfile.open ("/home/brad/Desktop/Tests/pacer-tests/BotBuilder/FrontEnd/debug.txt", std::ios::in | std::ios::out | std::ios::ate);
   myfile << "----------------------------goForward.cpp---------------------------------";
   myfile << "\n";
   myfile << "modelNo: " << getenv("modelNo") << "\n";
   myfile << "max_vel: " << getenv("max_vel") << "\n";
   myfile << "delta_v: " << getenv("delta_v") << "\n";
   myfile << "curr_vel: " << getenv("curr_vel") << "\n";
   myfile << "unit_len: " << getenv("unit_len") << "\n";
   myfile << "unit_den: " << getenv("unit_den") << "\n";
   myfile << "unit_rad: " << getenv("unit_rad") << "\n";
   myfile << "test_dur: " << getenv("test_dur") << "\n";
   myfile << "curr_line: " << getenv("curr_line") << "\n";
   myfile << "curr_iter: " << getenv("curr_iter") << "\n";
   myfile << "lenF1: " << getenv("lenF1") << "\n";
   myfile << "lenF2: " << getenv("lenF2") << "\n";
   myfile << "lenH1: " << getenv("lenH1") << "\n";
   myfile << "lenH2: " << getenv("lenH2") << "\n";
   myfile << "base_size_length: " << getenv("base_size_length") << "\n";
   myfile << "base_size_width: " << getenv("base_size_width") << "\n";
   myfile << "base_size_height: " << getenv("base_size_height") << "\n";
   myfile << "density: " << getenv("density") << "\n";
   myfile << "linkRad: " << getenv("linkRad") << "\n";
   myfile << "footRad: " << getenv("footRad") << "\n";
   myfile << "footLen: " << getenv("footLen") << "\n";
   myfile << "KINEMATIC: " << getenv("KINEMATIC") << "\n";
   







boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
double currSpeed =std::stod(getenv("curr_vel"));


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

if(duration==.351 && std::stod(getenv("curr_vel"))<std::stod(getenv("max_vel")))
{
	std::string line2=getenv("BUILDER_CAPT_PATH");
	
	line2+="/run.sh";

	
        myfile << "\n" << "\n" << "\n" <<"Forking to XML-traj " <<currSpeed << "\n" << "\n" << "\n";


       
        execl(line2.c_str(), line2.c_str(), (char *) 0);
}
	

        

else if(duration==.351 && std::stod(getenv("curr_vel"))==std::stod(getenv("max_vel")))
	{
                
		std::string line3=getenv("BUILDER_SCRIPT_PATH");
                
	        line3+="/setup-plugins-play.sh";
                 myfile << "\n" << "\n" << "\n" <<"Forking to Play " <<currSpeed << "\n" << "\n" << "\n";
        	
		setenv("curr_vel",getenv("delta_v"),1);


        	
        	execl("/bin/sh","sh","-c" , line3.c_str(), (char *) 0);

		

               
	}
   
myfile.close();
}

void setup(){
 
boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
setenv("duration","0",1);

double currSpeed =std::stod(getenv("curr_vel"));
  currSpeed+=std::stod(getenv("delta_v"));

    std::ostringstream s;
        s << currSpeed;
        std::string line=s.str();

     setenv("curr_vel",line.c_str(),1);
   
        
    s.clear();
    s.str(std::string());
}
