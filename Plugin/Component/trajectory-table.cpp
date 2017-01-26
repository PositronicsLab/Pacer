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

boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);

  double numIter= std::stod(getenv("curr_iter"));
  double testDur= std::stod(getenv("test_dur"));
  double currVel= std::stod(getenv("curr_vel")); 

 double currLine;
  if(std::stod(getenv("jac_count"))==0)
{
   currLine=std::stod(getenv("curr_line"));
}
else
{
   currLine=std::stod(getenv("fail_line_start"));
   if(currLine<=0){currLine==1;}
}
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




 //same deal here, make sure that the init file gets updated!
    if(currLine>=num_rows && std::stod(getenv("curr_vel"))<std::stod(getenv("max_vel")) && std::stod(getenv("jac_count"))==0)
    {
	std::cout << "\n" << "\n" << "\n" << "\n" << "\n" << "\n" << "here" << "\n" << "\n" << "\n" << "\n";
        std::ofstream errOut;
  errOut.open(getenv("BUILDER_HOME_PATH")+std::string("/matlabData.txt"), std::ios::app);
	  errOut << getenv("lenF1") << " " << getenv("lenF2") << " " << getenv("FfootLen") << " " << getenv("lenH1") << " " << getenv("lenH2") << " "<< getenv("HfootLen") << " " << getenv("base_size_length") << " " << getenv("base_size_width") << " " << getenv("base_size_height") << " " << getenv("FlinkRad") << " " << getenv("HlinkRad") << " " << getenv("FfootRad") << " " << getenv("HfootRad") << " " << getenv("massF1") << " " << getenv("massF2") << " " << getenv("massF3") << " " << getenv("massH1") << " " << getenv("massH2") << " "<< getenv("massH3") << " " << getenv("massBase") << " " << getenv("LF_X_1_vel") << " " << getenv("LF_Y_2_vel") << " " << getenv("LF_Y_3_vel") << " " << getenv("RF_X_1_vel") << " " << getenv("RF_Y_2_vel") << " " << getenv("RF_Y_3_vel") << " " << getenv("LH_X_1_vel") << " " << getenv("LH_Y_2_vel") << " " << getenv("LH_Y_3_vel") << " " << getenv("RH_X_1_vel") << " " << getenv("RH_Y_2_vel") << " " << getenv("RH_Y_3_vel") << " " << getenv("LF_X_1_tor") << " " << getenv("LF_Y_2_tor") << " " << getenv("LF_Y_3_tor") << " " << getenv("RF_X_1_tor") << " " << getenv("RF_Y_2_tor") << " " << getenv("RF_Y_3_tor") << " " << getenv("LH_X_1_tor") << " " << getenv("LH_Y_2_tor") << " " << getenv("LH_Y_3_tor") << " " << getenv("RH_X_1_tor") << " " << getenv("RH_Y_2_tor") << " " << getenv("RH_Y_3_tor") << " " << getenv("curr_vel") << " " << getenv("curr_line") << " " << getenv("modelNo") << "\n";
errOut.close();


        setenv("curr_line","0",1);
        setenv("curr_iter","0",1);
        
        


	double curr_vel=std::stod(getenv("curr_vel"));
        curr_vel+=std::stod(getenv("delta_v"));
        std::ostringstream s;
        s << curr_vel;
        setenv("curr_vel",s.str().c_str(),1);
        setenv("fail_vel",getenv("curr_vel"),1);
        std::string line=s.str();
	double modelNo = std::stod(getenv("modelNo"));
	std::ostringstream file;
        file << getenv("BUILDER_POSE_PATH") << "/" << modelNo << "-" << curr_vel << "-" << "PoseSet.txt";
       std::string filename = file.str();
       std::ifstream nextFile(filename);

	std::string initVals;


	std::getline(nextFile,initVals);

	typedef boost::tokenizer<boost::char_separator<char> > 
            tokenizer;
            boost::char_separator<char> sep(" ");
            tokenizer tokens(initVals, sep);
            tokenizer::iterator tok_iter = tokens.begin();

	    for(int i=0;i<joint_names.size();i++)
            {
     		
		//export q
                              s.clear();
                              s.str(std::string());
                              s << joint_names[i] << "_q";
        		      line=s.str();
                              s.clear();
                              s.str(std::string());
                              s << std::stod(*tok_iter);
                              setenv(line.c_str(),s.str().c_str(),1);
		              line=std::string();
			//export qd
                              s.clear();
                              s.str(std::string());
                              s << joint_names[i] << "_qd";
        		      line=s.str();
                              s.clear();
                              s.str(std::string());
                              s << std::stod(*(std::next(tok_iter,1)));
                              setenv(line.c_str(),s.str().c_str(),1);
		              line=std::string();
			//export qdd
                              s.clear();
                              s.str(std::string());
                              s << joint_names[i] << "_qdd";
        		      line=s.str();
                              s.clear();
                              s.str(std::string());
                              s << std::stod(*(std::next(tok_iter,2)));
                              setenv(line.c_str(),s.str().c_str(),1);
		              line=std::string();
          	std::advance(tok_iter,3);
             }        
				//export body x axis velocity
		              s.clear();
                              s.str(std::string());
                              s << "BODYx";
        		      line=s.str();
                              s.clear();
                              s.str(std::string());
                              s << std::stod(*tok_iter);
                              setenv(line.c_str(),s.str().c_str(),1);
		              line=std::string();

				//export body y axis velocity
    		              s.clear();
                              s.str(std::string());
                              s << "BODYy";
        		      line=s.str();
                              s.clear();
                              s.str(std::string());
                              s << std::stod(*(std::next(tok_iter,1)));
                              setenv(line.c_str(),s.str().c_str(),1);
		              line=std::string();
				
				//export body z axis velocity
    		              s.clear();
                              s.str(std::string());
                              s << "BODYz";
        		      line=s.str();
                              s.clear();
                              s.str(std::string());
                              s << std::stod(*(std::next(tok_iter,2)));
                              setenv(line.c_str(),s.str().c_str(),1);
		              line=std::string();
		
				//export body roll axis velocity
		              s.clear();
                              s.str(std::string());
                              s << "BODYr";
        		      line=s.str();
                              s.clear();
                              s.str(std::string());
                              s << std::stod(*(std::next(tok_iter,3)));
                              setenv(line.c_str(),s.str().c_str(),1);
		              line=std::string();

				//export body pitch axis velocity
    		              s.clear();
                              s.str(std::string());
                              s << "BODYp";
        		      line=s.str();
                              s.clear();
                              s.str(std::string());
                              s << std::stod(*(std::next(tok_iter,4)));
                              setenv(line.c_str(),s.str().c_str(),1);
		              line=std::string();
				
				//export body yaw axis velocity
    		              s.clear();
                              s.str(std::string());
                              s << "BODYt";
        		      line=s.str();
                              s.clear();
                              s.str(std::string());
                              s << std::stod(*(std::next(tok_iter,5)));
                              setenv(line.c_str(),s.str().c_str(),1);
		              line=std::string();
                
                
                
                
                

	

        std::string line3=getenv("BUILDER_SCRIPT_PATH");
	
	line3+="/setup-plugins-play.sh";
        
	
        execl(line3.c_str(), line3.c_str(), (char *) 0);
	
	
	
       
    	
    }
	//if no limit is reached then the model works, then record the data in MatlabData.txt and exit the program
	else if(currLine>=num_rows && std::stod(getenv("curr_vel"))==std::stod(getenv("max_vel")) && std::stod(getenv("jac_count"))==0)
	{
		
                std::ofstream errOut;
  errOut.open(getenv("BUILDER_HOME_PATH")+std::string("/matlabData.txt"), std::ios::app);
	  errOut << getenv("lenF1") << " " << getenv("lenF2") << " " << getenv("FfootLen") << " " << getenv("lenH1") << " " << getenv("lenH2") << " "<< getenv("HfootLen") << " " << getenv("base_size_length") << " " << getenv("base_size_width") << " " << getenv("base_size_height") << " " << getenv("FlinkRad") << " " << getenv("HlinkRad") << " " << getenv("FfootRad") << " " << getenv("HfootRad") << " " << getenv("massF1") << " " << getenv("massF2") << " " << getenv("massF3") << " " << getenv("massH1") << " " << getenv("massH2") << " "<< getenv("massH3") << " " << getenv("massBase") << " " << getenv("LF_X_1_vel") << " " << getenv("LF_Y_2_vel") << " " << getenv("LF_Y_3_vel") << " " << getenv("RF_X_1_vel") << " " << getenv("RF_Y_2_vel") << " " << getenv("RF_Y_3_vel") << " " << getenv("LH_X_1_vel") << " " << getenv("LH_Y_2_vel") << " " << getenv("LH_Y_3_vel") << " " << getenv("RH_X_1_vel") << " " << getenv("RH_Y_2_vel") << " " << getenv("RH_Y_3_vel") << " " << getenv("LF_X_1_tor") << " " << getenv("LF_Y_2_tor") << " " << getenv("LF_Y_3_tor") << " " << getenv("RF_X_1_tor") << " " << getenv("RF_Y_2_tor") << " " << getenv("RF_Y_3_tor") << " " << getenv("LH_X_1_tor") << " " << getenv("LH_Y_2_tor") << " " << getenv("LH_Y_3_tor") << " " << getenv("RH_X_1_tor") << " " << getenv("RH_Y_2_tor") << " " << getenv("RH_Y_3_tor") << " " << getenv("curr_vel") << " " << getenv("curr_line") << " " << getenv("modelNo") << "\n";
errOut.close();
		throw std::runtime_error("The model works! No improvement is required with the current task");
	}



//now, when we start a new process make sure to edit the init q and qd
//that is, the initial values for the next window must be exported and set in the vars.xml file so the model 
//immediately matches the beginning of the next test
  if(numIter>=testDur && std::stod(getenv("jac_count"))==0)
   {
        currLine=currLine-testDur;
	currLine++;
        s << currLine;
        
        bool properWindow;
 ctrl->get_data<bool>(plugin_namespace+".propWindow",properWindow);
        if(properWindow){
        setenv("curr_line",s.str().c_str() ,1);}
        setenv("fail_vel",getenv("curr_vel"),1);
        setenv("curr_iter","0",1);
        std::string line2=getenv("BUILDER_SCRIPT_PATH");
	
	line2+="/setup-plugins-play.sh";
std::string line;
for (int i=0; i<joint_names.size(); i++) 
     {
         		      //export q
                              s.clear();
                              s.str(std::string());
                              s << joint_names[i] << "_q";
        		      line=s.str();
                              s.clear();
                              s.str(std::string());
                              s << allqVals[currLine][i];
                              setenv(line.c_str(),s.str().c_str(),1);
		              line=std::string();
			//export qd
                              s.clear();
                              s.str(std::string());
                              s << joint_names[i] << "_qd";
        		      line=s.str();
                              s.clear();
                              s.str(std::string());
                              s << allqdVals[currLine][i];
                              setenv(line.c_str(),s.str().c_str(),1);
		              line=std::string();
			//export qdd
                              s.clear();
                              s.str(std::string());
                              s << joint_names[i] << "_qdd";
        		      line=s.str();
                              s.clear();
                              s.str(std::string());
                              s << allqddVals[currLine][i];
                              setenv(line.c_str(),s.str().c_str(),1);
		              line=std::string();
        
	}

		//export body x axis velocity
		              s.clear();
                              s.str(std::string());
                              s << "BODYx";
        		      line=s.str();
                              s.clear();
                              s.str(std::string());
                              s << allBodyVals[currLine][0];
                              setenv(line.c_str(),s.str().c_str(),1);
		              line=std::string();

				//export body y axis velocity
    		              s.clear();
                              s.str(std::string());
                              s << "BODYy";
        		      line=s.str();
                              s.clear();
                              s.str(std::string());
                              s << allBodyVals[currLine][1];
                              setenv(line.c_str(),s.str().c_str(),1);
		              line=std::string();
				
				//export body z axis velocity
    		              s.clear();
                              s.str(std::string());
                              s << "BODYz";
        		      line=s.str();
                              s.clear();
                              s.str(std::string());
                              s << allBodyVals[currLine][2];
                              setenv(line.c_str(),s.str().c_str(),1);
		              line=std::string();
		
				//export body roll axis velocity
		              s.clear();
                              s.str(std::string());
                              s << "BODYr";
        		      line=s.str();
                              s.clear();
                              s.str(std::string());
                              s << allBodyVals[currLine][3];
                              setenv(line.c_str(),s.str().c_str(),1);
		              line=std::string();

				//export body pitch axis velocity
    		              s.clear();
                              s.str(std::string());
                              s << "BODYp";
        		      line=s.str();
                              s.clear();
                              s.str(std::string());
                              s << allBodyVals[currLine][4];
                              setenv(line.c_str(),s.str().c_str(),1);
		              line=std::string();
				
				//export body yaw axis velocity
    		              s.clear();
                              s.str(std::string());
                              s << "BODYt";
        		      line=s.str();
                              s.clear();
                              s.str(std::string());
                              s << allBodyVals[currLine][5];
                              setenv(line.c_str(),s.str().c_str(),1);
		              line=std::string();
        execl(line2.c_str(), line2.c_str(), (char *) 0);

	}
	
   else
   {
      numIter+=1;
      s << numIter;
      setenv("curr_iter",s.str().c_str(),1);
   }
if(currLine<0){currLine=0;
	setenv("fail_line","0",1);
}
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
    if(std::stod(getenv("jac_count"))==0)
   { setenv("curr_line",s.str().c_str(),1);}
   else
   {
     if(currLine<num_rows)
     setenv("fail_line_start",s.str().c_str(),1);
   }
    
 }  
void setup(){
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

  double modelNo=std::stod(getenv("modelNo"));
  double currVel=std::stod(getenv("curr_vel"));
 double init;
  if(std::stod(getenv("jac_count"))==0)
  {init=std::stod(getenv("curr_line"));
   setenv("fail_line_start",getenv("curr_line"),1);
   setenv("fail_vel",getenv("curr_vel"),1);
   setenv("fail_line",getenv("curr_line"),1);
}
  else
  {init=std::stod(getenv("fail_line_start"));}
  if(init<0){init=0;}
  
   std::ostringstream s;
    s << getenv("BUILDER_POSE_PATH") << "/" << modelNo << "-" << currVel << "-" << "PoseSet.txt";
    std::string filename = s.str();
  std::ifstream myfile(filename);

int line_count=0;

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
