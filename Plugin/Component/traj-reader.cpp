#include <Pacer/controller.h>
#include "plugin.h"
#include <boost/tokenizer.hpp>
#include <sys/stat.h>
#include <string>
#include <deque>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <sys/types.h>
#include <sys/wait.h>

void loop(){
std::cout << "begin" << std::endl;
boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);

  double numIter= std::stod(getenv("curr_iter"));
  double testDur= std::stod(getenv("test_dur"));
  double currVel= 0.3; 

 double currLine;
std::cout << "inits" << std::endl;
   currLine=std::stod(getenv("curr_line"));


std::cout << "currLine: " << currLine << std::endl;
  std::ostringstream s;
 
  std::vector<std::vector<double> > allqVals;
  std::vector<std::vector<double> > allqdVals;
  std::vector<std::vector<double> > allqddVals;
  std::vector<Ravelin::VectorNd > allBodyVals;
  double num_rows;
  std::vector<std::string> joint_names = ctrl->get_data<std::vector<std::string> >("init.joint.id");

  ctrl->get_data("q_vals", allqVals);
  ctrl->get_data("qd_vals", allqdVals);
  ctrl->get_data("qdd_vals", allqddVals);
  ctrl->get_data("body_vals", allBodyVals);
  
  ctrl->get_data("num_pose_rows",num_rows);



      numIter+=1;
      s << numIter;
      setenv("curr_iter",s.str().c_str(),1);
 //same deal here, make sure that the init file gets updated!
    if(currLine>=num_rows)
    {
	currLine=0;
	numIter=0;
        setenv("curr_line","0",1);
        setenv("curr_iter","0",1);
	
    }
std::cout << "curr_line " << getenv("curr_line") <<std::endl;
	std::cout << "curr_iter " << getenv("curr_iter") <<std::endl;
//now, when we start a new process make sure to edit the init q and qd
//that is, the initial values for the next window must be exported and set in the vars.xml file so the model 
//immediately matches the beginning of the next test
      

std::map<std::string, std::vector<double> > q, qd,qdd;
//get the current position/velocity/acceleration for everything for the current line
//and set the ctrl values to those values
for(int joint=0;joint<joint_names.size(); joint++)
{
	        q[joint_names[joint]] = std::vector<double>(1);
		qd[joint_names[joint]] = std::vector<double>(1);
		qdd[joint_names[joint]] = std::vector<double>(1);

                q[joint_names[joint]][0] = allqVals[currLine][joint];
		qd[joint_names[joint]][0] = allqdVals[currLine][joint];
		qdd[joint_names[joint]][0] = allqddVals[currLine][joint];
		
}

	    ctrl->set_joint_value(Pacer::Controller::position_goal,q);
            ctrl->set_joint_value(Pacer::Controller::velocity_goal,qd);
            ctrl->set_joint_value(Pacer::Controller::acceleration_goal,qdd);	
            ctrl->set_base_value(Pacer::Controller::velocity_goal,allBodyVals[currLine]);
        s.clear();
        s.str(std::string());
        currLine+=1;
        s << currLine;
    setenv("curr_line",s.str().c_str(),1);
std::cout << "curr_line2 " << getenv("curr_line") <<std::endl;
    std::cout << "end of loop?" << std::endl;
 }  
void setup(){
std::cout << "setup?" << std::endl;
boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);

std::vector<std::vector<double>> allqVals;
std::vector<std::vector<double>> allqdVals;
std::vector<std::vector<double>> allqddVals;
std::vector<Ravelin::VectorNd > allBodyVals;
std::vector<double> qVals;
std::vector<double> qdVals;
std::vector<double> qddVals;
Ravelin::VectorNd bodyVals;
 //string to hold extracted values, and size_type to keep track of location in the line
  std::string line;

  //get the end effector names
  std::vector<std::string> joint_names = ctrl->get_data<std::vector<std::string> >("init.joint.id");
std::cout << getenv("curr_vel") << std::endl;
  std::string name=std::string(getenv("name"));
  double currVel=0.3;
std::cout << currVel << std::endl;
 double init;
init=std::stod(getenv("curr_line"));

   std::ostringstream s;
    s << getenv("POSE_PATH") << "/" << name << "-" << currVel << "-" << "PoseSet.txt";
	std::cout << s.str() << std::endl;
    std::string filename = s.str();
  std::ifstream myfile(filename);

int line_count=0;
std::cout << "here1" << std::endl;
std::fstream check;
//take all values from the file of trajectories and store them in pacer's data storage(edit to use joints instead of eefs)
while(std::getline(myfile,line))
{
       typedef boost::tokenizer<boost::char_separator<char> > 
            tokenizer;
            boost::char_separator<char> sep(" ");
            tokenizer tokens(line, sep);
            tokenizer::iterator tok_iter = tokens.begin();

        qVals.clear();
	qdVals.clear();
	qddVals.clear();
        std::cout << "here?" << std::endl;

for(int i=0;i<joint_names.size();i++)
{
     
	qVals.push_back(std::stod(*tok_iter));
        qdVals.push_back(std::stod(*(std::next(tok_iter,1))));
        qddVals.push_back(std::stod(*(std::next(tok_iter,2))));
       
          std::advance(tok_iter,3);
}
            bodyVals=bodyVals.construct_variable(6,std::stod(*tok_iter),std::stod(*(std::next(tok_iter,1))),std::stod(*(std::next(tok_iter,2))),std::stod(*(std::next(tok_iter,3))),std::stod(*(std::next(tok_iter,4))),std::stod(*(std::next(tok_iter,5))));
//the all variables contain the whole file, where as the normal variables are just one line
allqVals.push_back(qVals);
allqdVals.push_back(qdVals);
allqddVals.push_back(qddVals);
allBodyVals.push_back(bodyVals);

	line_count++;
}
ctrl->set_data<double>("num_pose_rows",line_count);

check.close();
//when every line has been scanned and insterted into the "all" variables, create and set some variables to that, so that
//these values can be accessed without rescanning the file
ctrl->set_data<std::vector<std::vector<double> > >("q_vals", allqVals);
ctrl->set_data<std::vector<std::vector<double> > >("qd_vals", allqdVals);
ctrl->set_data<std::vector<std::vector<double> > >("qdd_vals", allqddVals);
ctrl->set_data<std::vector<Ravelin::VectorNd > >("body_vals", allBodyVals);


myfile.close();
}
