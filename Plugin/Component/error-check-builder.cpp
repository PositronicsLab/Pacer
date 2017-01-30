#include <stdexcept>
#include <Pacer/controller.h>
#include "plugin.h"
#include <sys/types.h>
#include <sys/wait.h>

void loop(){
boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);


//all of these initial checks just go through and check every links contacts to see if they're in contact with anything they shouldn't be touching
std::vector< boost::shared_ptr< Pacer::Robot::contact_t> > contacts;
//Check body for bad contact
ctrl->get_link_contacts("BODY0",contacts);

for (int i=0; i < contacts.size(); i++) 
{
    if(contacts[i]->id.compare("LF_1") == 0 && contacts[i]->id.compare("RF_1") == 0 && contacts[i]->id.compare("LH_1") == 0 && contacts[i]->id.compare("RH_1") == 0)
    {
	std::cout << "penetration: " << contacts[i]->normal << std::endl;
	throw std::runtime_error("Self Collision");
    }
}

//check _1 links for bad contact
ctrl->get_link_contacts("LF_1",contacts);

for (int i=0; i < contacts.size(); i++) 
{
    if(contacts[i]->id.compare("LF_2") == 0 && contacts[i]->id.compare("BODY0") == 0)
    {
	std::cout << "penetration: " << contacts[i]->normal << std::endl;
	throw std::runtime_error("Self Collision");
    }
}
ctrl->get_link_contacts("RF_1",contacts);

for (int i=0; i < contacts.size(); i++) 
{
    if(contacts[i]->id.compare("RF_2") == 0 && contacts[i]->id.compare("BODY0") == 0)
    {
	std::cout << "penetration: " << contacts[i]->normal << std::endl;
	throw std::runtime_error("Self Collision");
    }
}
ctrl->get_link_contacts("LH_1",contacts);

for (int i=0; i < contacts.size(); i++) 
{
    if(contacts[i]->id.compare("LH_2") == 0 && contacts[i]->id.compare("BODY0") == 0)
    {
	std::cout << "penetration: " << contacts[i]->normal << std::endl;
	throw std::runtime_error("Self Collision");
    }
}
ctrl->get_link_contacts("RH_1",contacts);

for (int i=0; i < contacts.size(); i++) 
{
    if(contacts[i]->id.compare("RH_2") == 0 && contacts[i]->id.compare("BODY0") == 0)
    {
	std::cout << "penetration: " << contacts[i]->normal << std::endl;
	throw std::runtime_error("Self Collision");
    }
}
//check _2 links for bad contact
ctrl->get_link_contacts("LF_2",contacts);

for (int i=0; i < contacts.size(); i++) 
{
    if(contacts[i]->id.compare("LF_1") == 0 && contacts[i]->id.compare("LF_3") == 0)
    {
	std::cout << "penetration: " << contacts[i]->normal << std::endl;
	throw std::runtime_error("Self Collision");
    }
}

ctrl->get_link_contacts("RF_2",contacts);

for (int i=0; i < contacts.size(); i++) 
{
    if(contacts[i]->id.compare("RF_1") == 0 && contacts[i]->id.compare("RH_3") == 0)
    {
	std::cout << "penetration: " << contacts[i]->normal << std::endl;
	throw std::runtime_error("Self Collision");
    }
}

ctrl->get_link_contacts("LH_2",contacts);

for (int i=0; i < contacts.size(); i++) 
{
    if(contacts[i]->id.compare("LH_1") == 0 && contacts[i]->id.compare("LH_3") == 0)
    {
	std::cout << "penetration: " << contacts[i]->normal << std::endl;
	throw std::runtime_error("Self Collision");
    }
}

ctrl->get_link_contacts("RH_2",contacts);

for (int i=0; i < contacts.size(); i++) 
{
    if(contacts[i]->id.compare("RH_1") == 0 && contacts[i]->id.compare("RH_3") == 0)
    {
	std::cout << "penetration: " << contacts[i]->normal << std::endl;
	throw std::runtime_error("Self Collision");
    }
}
//check _3 links for bad contact
ctrl->get_link_contacts("LF_3",contacts);

for (int i=0; i < contacts.size(); i++) 
{
    if(contacts[i]->id.compare("LF_2") == 0 && contacts[i]->id.compare("LF_FOOT") == 0)
    {
	std::cout << "penetration: " << contacts[i]->normal << std::endl;
	throw std::runtime_error("Self Collision");
    }
}

ctrl->get_link_contacts("RF_3",contacts);

for (int i=0; i < contacts.size(); i++) 
{
    if(contacts[i]->id.compare("RF_2") == 0 && contacts[i]->id.compare("RH_FOOT") == 0)
    {
	std::cout << "penetration: " << contacts[i]->normal << std::endl;
	throw std::runtime_error("Self Collision");
    }
}

ctrl->get_link_contacts("LH_3",contacts);

for (int i=0; i < contacts.size(); i++) 
{
    if(contacts[i]->id.compare("LH_2") == 0 && contacts[i]->id.compare("LH_FOOT") == 0)
    {
	std::cout << "penetration: " << contacts[i]->normal << std::endl;
	throw std::runtime_error("Self Collision");
    }
}

ctrl->get_link_contacts("RH_3",contacts);

for (int i=0; i < contacts.size(); i++) 
{
    if(contacts[i]->id.compare("RH_2") == 0 && contacts[i]->id.compare("RH_FOOT") == 0)
    {
	std::cout << "penetration: " << contacts[i]->normal << std::endl;
	throw std::runtime_error("Self Collision");
    }
}
//check _FOOT links for bad contact
ctrl->get_link_contacts("LF_FOOT",contacts);

for (int i=0; i < contacts.size(); i++) 
{
    if(contacts[i]->id.compare("LF_3") == 0)
    {
	std::cout << "penetration: " << contacts[i]->normal << std::endl;
	throw std::runtime_error("Self Collision");
    }
}

ctrl->get_link_contacts("RF_FOOT",contacts);

for (int i=0; i < contacts.size(); i++) 
{
    if(contacts[i]->id.compare("RF_3") == 0)
    {
	std::cout << "penetration: " << contacts[i]->normal << std::endl;
	throw std::runtime_error("Self Collision");
    }
}

ctrl->get_link_contacts("LH_FOOT",contacts);

for (int i=0; i < contacts.size(); i++) 
{
    if(contacts[i]->id.compare("LH_3") == 0)
    {
	std::cout << "penetration: " << contacts[i]->normal << std::endl;
	throw std::runtime_error("Self Collision");
    }
}

ctrl->get_link_contacts("RH_FOOT",contacts);

for (int i=0; i < contacts.size(); i++) 
{
    if(contacts[i]->id.compare("RH_3") == 0)
    {
	std::cout << "penetration: " << contacts[i]->normal << std::endl;
	throw std::runtime_error("Self Collision");
    }
}



double torque_limit_at_velocity;

  
  Ravelin::VectorNd generalized_fext = ctrl->get_generalized_value(Pacer::Robot::load);
  std::ostringstream s;
 std::string line;
  std::vector<std::string> var_names={"lenF2","FfootLen","lenH2","HfootLen","base_size_length","base_size_width"};

 //this is all to get the data we'll be comparing to, all of the positions, velocities, accelerations, and loads
  std::map<std::string, Ravelin::VectorNd > q, qd, u;
  ctrl->get_joint_value(Pacer::Robot::position_goal, q);
  ctrl->get_joint_value(Pacer::Robot::velocity_goal, qd);
  ctrl->get_joint_value(Pacer::Robot::load_goal, u);
  double num_rows;
  ctrl->get_data("num_pose_rows",num_rows);
  int jac_count=std::stod(getenv("jac_count"));

  //get the various joints and the torque/velocity limits
  std::vector<std::string> joint_names = ctrl->get_data<std::vector<std::string> >("init.joint.id");
  std::vector<double> joint_dofs = ctrl->get_data<std::vector<double> >("init.joint.dofs");
  std::vector<double> torque_limit;
  bool apply_torque_limit = ctrl->get_data<std::vector<double> >("init.joint.limits.u",torque_limit);

  std::vector<double> velocity_limit;
  bool apply_velocity_limit = ctrl->get_data<std::vector<double> >("init.joint.limits.qd",velocity_limit);

//check to see if any of the joints are past the angles at which they'd collide to their adjacent links
//this is because the links are technically always in contact, so doing a simple search would always result in failure
//these values need to be measured on the actual robot and then applied here
/*for (int i=0; i<joint_names.size(); i++) 
{
	if(q[joint_names[i]][0]>1.8326 || q[joint_names[i]][0]<-1.8326)
	{
		std::cout << "penetration: " << contacts[i]->normal << std::endl;
	        throw std::runtime_error("Self Collision");
		
	}
}*/




if(jac_count==0)
{

        //for every joint, get the simple difference between it and its curresponding velocity limit, and set it to the corresponding joint/limit enivornment variable
	for (int i=0, ii=0; i<joint_names.size(); i++) 
  	{
   		for (int j=0; j<joint_dofs[i]; j++,ii++) 
    		{
			torque_limit_at_velocity = (-torque_limit[ii]/velocity_limit[ii]) * fabs(qd[joint_names[i]][j]) + torque_limit[ii];
			std::ostringstream tor_main;
        		tor_main << joint_names[i] << "_tor";
        		line=tor_main.str();
        		tor_main.clear();//clear any bits set
        		tor_main.str(std::string());
			tor_main << fabs(torque_limit_at_velocity)-fabs(u[joint_names[i]][j]);
       		 	setenv(line.c_str(),tor_main.str().c_str(),1);
    		}
  	}
        //check every joint to see if they're above/below the torque limit
        //we haven't been using the velocity limits as of august
	for (int i=0, ii=0; i<joint_names.size(); i++) 
  	{
    		for (int j=0; j<joint_dofs[i]; j++,ii++) 
    		{
			torque_limit_at_velocity = (-torque_limit[ii]/velocity_limit[ii]) * fabs(qd[joint_names[i]][j]) + torque_limit[ii];
        		if ((fabs(torque_limit_at_velocity)-fabs(u[joint_names[i]][j]))<0) 
			{
                               //if a limit is exceeded, set the fail line start to the beginning of the current sliding window
			       //and fail_line to the current line
				s.clear();
        			s.str(std::string());
        			double fail=std::stod(getenv("curr_line"))-std::stod(getenv("test_dur"));
        			s << fail;
        			setenv("fail_line",getenv("curr_line"),1);
        			setenv("fail_line_start",s.str().c_str(),1);
        			s.clear();
        			s.str(std::string());
				//send all of the information on the current model, its value-limit enivornment variables created above
			        //and the velocity, line, and model number of the failure
        			std::ofstream errOut;
        			errOut.open(getenv("BUILDER_HOME_PATH")+std::string("/matlabData.txt"), std::ios::app);
				errOut  << getenv("lenF2") << " " << getenv("FfootLen") << " "<< getenv("lenH2") 
					<< " " << getenv("HfootLen") << " " << getenv("base_size_length") 
					<< " "<< getenv("base_size_width") << " " << getenv("base_size_height") 
					<< " " << getenv("FlinkRad") << " "<< getenv("HlinkRad") << " " << getenv("FfootRad") 
					<< " " << getenv("HfootRad") << " "<< getenv("massF2") << " " << getenv("massF3")  
					<< " " << getenv("massH2") << " "<< getenv("massH3") << " " << getenv("massBase") 
					<< " " << getenv("LF_X_1_tor") << " " << getenv("LF_Y_2_tor") << " " << getenv("LF_Y_3_tor") 
					<< " " << getenv("RF_X_1_tor") << " " << getenv("RF_Y_2_tor") << " " << getenv("RF_Y_3_tor") 
					<< " " << getenv("LH_X_1_tor") << " " << getenv("LH_Y_2_tor") << " " << getenv("LH_Y_3_tor") 
					<< " " << getenv("RH_X_1_tor") << " " << getenv("RH_Y_2_tor") << " " << getenv("RH_Y_3_tor") 
					<< " " << getenv("curr_vel") << " " << getenv("curr_line") << " " << getenv("modelNo") << "\n";
				errOut.close();
                                //reset curr_line and cur_iter for the next test
				setenv("curr_line","0",1);
				setenv("curr_iter","0",1);
				//jac_count is no longer 0, so all of the files will execute the jacobian matrix generation loop
        			jac_count++;
        			s << jac_count;
        			line=s.str();
        			setenv("jac_count",s.str().c_str(),1);
        			s.clear();
        			s.str(std::string());
    				//this is the first variable in the list, so it is the first one that will be perturbed
        			double lenF2= std::stod(getenv("lenF2"));
        			double unitLen = 0.001;
	
				//perturb the value by unit size, then set the environment variable to the new value
				lenF2+=unitLen;
        			s << lenF2;
        			line=s.str();
        			setenv("lenF2",line.c_str(),1);
        			s.clear();
        			s.str(std::string());
				//generate this robot and begin testing it to generate its row for the matrix
				std::string generate=getenv("BUILDER_SCRIPT_PATH");
				generate+="/generate.sh";
        
        			execl(generate.c_str(), generate.c_str(), (char *) 0);
			} 
		}
	}
}
else if(jac_count<=var_names.size())
{
	//within the jacobian matrix generation, get the same joint-limit environment variables that we were getting before, except divided by the unit size
	for (int i=0, ii=0; i<joint_names.size(); i++) 
  	{
    		for (int j=0; j<joint_dofs[i]; j++,ii++) 
    		{
			torque_limit_at_velocity = (-torque_limit[ii]/velocity_limit[ii]) * fabs(qd[joint_names[i]][j]) + torque_limit[ii];
			std::cout << "\n" << "\n" << "\n" <<"\n" << "here" <<"\n" << "\n" << "\n" <<"\n" << std::endl;
			std::ostringstream tor,init_tor;
        		tor << jac_count<<"_" << joint_names[i] << "_tor";
        		init_tor << joint_names[i] << "_tor";
        		line=tor.str();
        		tor.clear();//clear any bits set
        		tor.str(std::string());
			
				tor << (fabs(torque_limit_at_velocity)-fabs(u[joint_names[i]][j])-std::stod(getenv(init_tor.str().c_str())))/100;
			
        		setenv(line.c_str(),tor.str().c_str(),1);
    		}
  	}
     
        //once the current sliding window for this variable has ended, increment jac_count, rest the past variable to its original value
	//and then increment the next value by unit size, and then begin testing anew with that variable
        if (std::stod(getenv("fail_line_start"))>=std::stod(getenv("fail_line"))) {

        	if(jac_count<var_names.size())   
        	{
			double pastVar= std::stod(getenv(var_names[jac_count-1].c_str()));
        		double currVar= std::stod(getenv(var_names[jac_count].c_str()));
        
			pastVar-=0.001;
        		s << pastVar;
        		line=s.str();
        		setenv(var_names[jac_count-1].c_str(),line.c_str(),1);
        		s.clear();
        		s.str(std::string());

			currVar+=0.001;
        		s << currVar;
        		line=s.str();
        		setenv(var_names[jac_count].c_str(),line.c_str(),1);
        		s.clear();
        		s.str(std::string());
		}

        	setenv("curr_line","0",1);
		setenv("curr_iter","0",1);
        	s.clear();
          	s.str(std::string());
          	double fail=std::stod(getenv("fail_line"))-std::stod(getenv("test_dur"));
          	s << fail;
          	setenv("fail_line_start",s.str().c_str(),1);
          	s.clear();
          	s.str(std::string());
        	jac_count++;
        	s << jac_count;
        	line=s.str();
        	setenv("jac_count",s.str().c_str(),1);
        	s.clear();
        	s.str(std::string());

		std::string generate=getenv("BUILDER_SCRIPT_PATH");
		generate+="/generate.sh";
        
        	execl(generate.c_str(), generate.c_str(), (char *) 0);



	  
        } 

}
else
{
	setenv("curr_line","0",1);
	setenv("curr_iter","0",1);
        setenv("jac_count","0",1);
        //reset the last variable
        double last= std::stod(getenv(var_names[var_names.size()-1].c_str()));
	
	last-=0.001;
        s << last;
        line=s.str();
        setenv(var_names[var_names.size()-1].c_str(),line.c_str(),1);
        s.clear();
        s.str(std::string());


	
		std::string editor=getenv("BUILDER_GUI_PATH");
		editor+="/editor";
        
        	execl(editor.c_str(), editor.c_str(), (char *) 0);

	 

}




}


void setup(){

}
