#include <Pacer/controller.h>
#include "plugin.h"
#include <boost/tokenizer.hpp>
#include <sys/stat.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

void loop(){

  
    boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
     //initializing variables
     std::fstream file;
     std::string filename;
     Ravelin::VectorNd command_xd;

     
  double currVel;
  double modelNo= std::stod(getenv("modelNo"));
std::cout << "\n" << "\n" << "\n" << "\n"<< "\n" << "fail here?" << "\n"<< "\n"<< "\n"<< "\n" << "\n";
  if(std::stod(getenv("jac_count"))>0)
	{
            std::cout << "\n" << "\n" << "\n" << "\n"<< "\n" << "fail here?" << "\n"<< "\n"<< "\n"<< "\n" << "\n";
	    currVel= std::stod(getenv("fail_vel"));	
	}
        else
	{
	    currVel= std::stod(getenv("curr_vel"));
	}
  
  double deltaV= std::stod(getenv("delta_v"));
    
    std::ostringstream s;
    s << getenv("BUILDER_POSE_PATH") << "/" << modelNo << "-" << currVel << "-" << "PoseSet.txt";
    filename = s.str();
     //get the joint names
     std::map<std::string, Ravelin::VectorNd > q, qd,qdd;
     std::vector<std::string> joint_names = ctrl->get_data<std::vector<std::string> >("init.joint.id");
     ctrl->get_joint_value(Pacer::Robot::position, q);
     ctrl->get_joint_value(Pacer::Robot::velocity_goal, qd);
     ctrl->get_joint_value(Pacer::Robot::acceleration_goal, qdd);
     //open the file that is being written to
     file.open(filename, std::fstream::app|std::fstream::out);
     double duration=std::stod(getenv("duration"));
     std::string line;
     //get the desired position, velocity, and acceleration for each joint and put that in the file
     for (int i=0; i<joint_names.size(); i++) 
     {
        
        
            double pos_des = q[joint_names[i]][0];
	    double vel_des = qd[joint_names[i]][0];
	    double accel_des = qdd[joint_names[i]][0];

		if(currVel==deltaV && duration==0.001)
		{
			 //export q
                              s.clear();
                              s.str(std::string());
                              s << joint_names[i] << "_q";
        		      line=s.str();
                              s.clear();
                              s.str(std::string());
                              s << q[joint_names[i]][0];
                              setenv(line.c_str(),s.str().c_str(),1);
		              line=std::string();
			//export qd
                              s.clear();
                              s.str(std::string());
                              s << joint_names[i] << "_qd";
        		      line=s.str();
                              s.clear();
                              s.str(std::string());
                              s << qd[joint_names[i]][0];
                              setenv(line.c_str(),s.str().c_str(),1);
		              line=std::string();
			//export qdd
                              s.clear();
                              s.str(std::string());
                              s << joint_names[i] << "_qdd";
        		      line=s.str();
                              s.clear();
                              s.str(std::string());
                              s << qdd[joint_names[i]][0];
                              setenv(line.c_str(),s.str().c_str(),1);
		              line=std::string();
		}

		
    
            file << pos_des << " " 
                 << vel_des << " " 
                 << accel_des << " "; 
        
     }
     
     
     //grab base position and base command and put those in the file and return to start the next row for the next iteration
     ctrl->get_data<Ravelin::VectorNd>("base-command",command_xd);
     file << command_xd[0] << " " << command_xd[1] << " " << command_xd[2] << " " << command_xd[3] << " " << command_xd[4] << " " << command_xd[5] 
     << "\n"; 
	    

if(currVel==deltaV && duration==0.001)
	{
		//export body x axis velocity
		              s.clear();
                              s.str(std::string());
                              s << "BODYx";
        		      line=s.str();
                              s.clear();
                              s.str(std::string());
                              s << command_xd[0];
                              setenv(line.c_str(),s.str().c_str(),1);
		              line=std::string();

				//export body y axis velocity
    		              s.clear();
                              s.str(std::string());
                              s << "BODYy";
        		      line=s.str();
                              s.clear();
                              s.str(std::string());
                              s << command_xd[1];
                              setenv(line.c_str(),s.str().c_str(),1);
		              line=std::string();
				
				//export body z axis velocity
    		              s.clear();
                              s.str(std::string());
                              s << "BODYz";
        		      line=s.str();
                              s.clear();
                              s.str(std::string());
                              s << command_xd[2];
                              setenv(line.c_str(),s.str().c_str(),1);
		              line=std::string();
		
				//export body roll axis velocity
		              s.clear();
                              s.str(std::string());
                              s << "BODYr";
        		      line=s.str();
                              s.clear();
                              s.str(std::string());
                              s << command_xd[3];
                              setenv(line.c_str(),s.str().c_str(),1);
		              line=std::string();

				//export body pitch axis velocity
    		              s.clear();
                              s.str(std::string());
                              s << "BODYp";
        		      line=s.str();
                              s.clear();
                              s.str(std::string());
                              s << command_xd[4];
                              setenv(line.c_str(),s.str().c_str(),1);
		              line=std::string();
				
				//export body yaw axis velocity
    		              s.clear();
                              s.str(std::string());
                              s << "BODYt";
        		      line=s.str();
                              s.clear();
                              s.str(std::string());
                              s << command_xd[5];
                              setenv(line.c_str(),s.str().c_str(),1);
		              line=std::string();
	}

     //close the file
     file.close();

	
}

void setup(){
	boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);

        

}
