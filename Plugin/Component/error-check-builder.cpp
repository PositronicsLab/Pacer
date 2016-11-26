
#include <stdexcept>
#include <Pacer/controller.h>
#include "plugin.h"
#include <sys/types.h>
#include <sys/wait.h>

void loop(){
boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);


std::vector< boost::shared_ptr< Pacer::Robot::contact_t> > contacts;
//Check body for bad contact
ctrl->get_link_contacts("BODY0",contacts);

for (int i=0; i < contacts.size(); i++) 
{
    if(contacts[i]->id.compare("LF_1") == 0 && contacts[i]->id.compare("RF_1") == 0 && contacts[i]->id.compare("LH_1") == 0 && contacts[i]->id.compare("RH_1") == 0)
    {
	std::cout << "penetration: " << contacts[i]->restitution << std::endl;
	throw std::runtime_error("Self-Collision!");
    }
}

//check _1 links for bad contact
ctrl->get_link_contacts("LF_1",contacts);

for (int i=0; i < contacts.size(); i++) 
{
    if(contacts[i]->id.compare("LF_2") == 0 && contacts[i]->id.compare("BODY0") == 0)
    {
	std::cout << "penetration: " << contacts[i]->restitution << std::endl;
	throw std::runtime_error("Self-Collision!");
    }
}
ctrl->get_link_contacts("RF_1",contacts);

for (int i=0; i < contacts.size(); i++) 
{
    if(contacts[i]->id.compare("RF_2") == 0 && contacts[i]->id.compare("BODY0") == 0)
    {
	std::cout << "penetration: " << contacts[i]->restitution << std::endl;
	throw std::runtime_error("Self-Collision!");
    }
}
ctrl->get_link_contacts("LH_1",contacts);

for (int i=0; i < contacts.size(); i++) 
{
    if(contacts[i]->id.compare("LH_2") == 0 && contacts[i]->id.compare("BODY0") == 0)
    {
	std::cout << "penetration: " << contacts[i]->restitution << std::endl;
	throw std::runtime_error("Self-Collision!");
    }
}
ctrl->get_link_contacts("RH_1",contacts);

for (int i=0; i < contacts.size(); i++) 
{
    if(contacts[i]->id.compare("RH_2") == 0 && contacts[i]->id.compare("BODY0") == 0)
    {
	std::cout << "penetration: " << contacts[i]->restitution << std::endl;
	throw std::runtime_error("Self-Collision!");
    }
}
//check _2 links for bad contact
ctrl->get_link_contacts("LF_2",contacts);

for (int i=0; i < contacts.size(); i++) 
{
    if(contacts[i]->id.compare("LF_1") == 0 && contacts[i]->id.compare("LF_3") == 0)
    {
	std::cout << "penetration: " << contacts[i]->restitution << std::endl;
	throw std::runtime_error("Self-Collision!");
    }
}

ctrl->get_link_contacts("RF_2",contacts);

for (int i=0; i < contacts.size(); i++) 
{
    if(contacts[i]->id.compare("RF_1") == 0 && contacts[i]->id.compare("RH_3") == 0)
    {
	std::cout << "penetration: " << contacts[i]->restitution << std::endl;
	throw std::runtime_error("Self-Collision!");
    }
}

ctrl->get_link_contacts("LH_2",contacts);

for (int i=0; i < contacts.size(); i++) 
{
    if(contacts[i]->id.compare("LH_1") == 0 && contacts[i]->id.compare("LH_3") == 0)
    {
	std::cout << "penetration: " << contacts[i]->restitution << std::endl;
	throw std::runtime_error("Self-Collision!");
    }
}

ctrl->get_link_contacts("RH_2",contacts);

for (int i=0; i < contacts.size(); i++) 
{
    if(contacts[i]->id.compare("RH_1") == 0 && contacts[i]->id.compare("RH_3") == 0)
    {
	std::cout << "penetration: " << contacts[i]->restitution << std::endl;
	throw std::runtime_error("Self-Collision!");
    }
}
//check _3 links for bad contact
ctrl->get_link_contacts("LF_3",contacts);

for (int i=0; i < contacts.size(); i++) 
{
    if(contacts[i]->id.compare("LF_2") == 0 && contacts[i]->id.compare("LF_FOOT") == 0)
    {
	std::cout << "penetration: " << contacts[i]->restitution << std::endl;
	throw std::runtime_error("Self-Collision!");
    }
}

ctrl->get_link_contacts("RF_3",contacts);

for (int i=0; i < contacts.size(); i++) 
{
    if(contacts[i]->id.compare("RF_2") == 0 && contacts[i]->id.compare("RH_FOOT") == 0)
    {
	std::cout << "penetration: " << contacts[i]->restitution << std::endl;
	throw std::runtime_error("Self-Collision!");
    }
}

ctrl->get_link_contacts("LH_3",contacts);

for (int i=0; i < contacts.size(); i++) 
{
    if(contacts[i]->id.compare("LH_2") == 0 && contacts[i]->id.compare("LH_FOOT") == 0)
    {
	std::cout << "penetration: " << contacts[i]->restitution << std::endl;
	throw std::runtime_error("Self-Collision!");
    }
}

ctrl->get_link_contacts("RH_3",contacts);

for (int i=0; i < contacts.size(); i++) 
{
    if(contacts[i]->id.compare("RH_2") == 0 && contacts[i]->id.compare("RH_FOOT") == 0)
    {
	std::cout << "penetration: " << contacts[i]->restitution << std::endl;
	throw std::runtime_error("Self-Collision!");
    }
}
//check _FOOT links for bad contact
ctrl->get_link_contacts("LF_FOOT",contacts);

for (int i=0; i < contacts.size(); i++) 
{
    if(contacts[i]->id.compare("LF_3") == 0)
    {
	std::cout << "penetration: " << contacts[i]->restitution << std::endl;
	throw std::runtime_error("Self-Collision!");
    }
}

ctrl->get_link_contacts("RF_FOOT",contacts);

for (int i=0; i < contacts.size(); i++) 
{
    if(contacts[i]->id.compare("RF_3") == 0)
    {
	std::cout << "penetration: " << contacts[i]->restitution << std::endl;
	throw std::runtime_error("Self-Collision!");
    }
}

ctrl->get_link_contacts("LH_FOOT",contacts);

for (int i=0; i < contacts.size(); i++) 
{
    if(contacts[i]->id.compare("LH_3") == 0)
    {
	std::cout << "penetration: " << contacts[i]->restitution << std::endl;
	throw std::runtime_error("Self-Collision!");
    }
}

ctrl->get_link_contacts("RH_FOOT",contacts);

for (int i=0; i < contacts.size(); i++) 
{
    if(contacts[i]->id.compare("RH_3") == 0)
    {
	std::cout << "penetration: " << contacts[i]->restitution << std::endl;
	throw std::runtime_error("Self-Collision!");
    }
}





  
  Ravelin::VectorNd generalized_fext = ctrl->get_generalized_value(Pacer::Robot::load);
  std::ostringstream s;
 std::string line;
  std::vector<std::string> var_names={"lenF1","lenF2","FfootLen","lenH1","lenH2","HfootLen","base_size_length","base_size_width","base_size_height","FfootRad","HfootRad"};

  std::map<std::string, Ravelin::VectorNd > q, qd, u;
  ctrl->get_joint_value(Pacer::Robot::position_goal, q);
  ctrl->get_joint_value(Pacer::Robot::velocity_goal, qd);
  ctrl->get_joint_value(Pacer::Robot::load_goal, u);
  double num_rows;
  ctrl->get_data("num_pose_rows",num_rows);
  int jac_count=std::stod(getenv("jac_count"));

  std::vector<std::string> joint_names = ctrl->get_data<std::vector<std::string> >("init.joint.id");
  std::vector<double> joint_dofs = ctrl->get_data<std::vector<double> >("init.joint.dofs");
  std::vector<double> torque_limit;
  bool apply_torque_limit = ctrl->get_data<std::vector<double> >("init.joint.limits.u",torque_limit);

  std::vector<double> velocity_limit;
  bool apply_velocity_limit = ctrl->get_data<std::vector<double> >("init.joint.limits.qd",velocity_limit);


	if(q[joint_names[0]][0]>1.8326 || q[joint_names[0]][0]<-1.8326)
	{
		std::cout << "Self collision on adjacent link";
		throw std::runtime_error("Self-Collision!");
	}





if(jac_count==0)
{

	for (int i=0, ii=0; i<joint_names.size(); i++) 
  	{
   		for (int j=0; j<joint_dofs[i]; j++,ii++) 
    		{
       			std::ostringstream s;
        		s << joint_names[i] << "_vel";
        		line=s.str();
        		s.clear();//clear any bits set
        		s.str(std::string());
        		if(qd[joint_names[i]][j]>=0)
        		{
				s << qd[joint_names[i]][j]-velocity_limit[ii];
			}
			else
			{
				s << (-1*qd[joint_names[i]][j])-velocity_limit[ii];	
			}
            		setenv(line.c_str(),s.str().c_str(),1);
    		}
  	}

  	for (int i=0, ii=0; i<joint_names.size(); i++) 
  	{
    		for (int j=0; j<joint_dofs[i]; j++,ii++) 
    		{
        		std::ostringstream s;
        		s << joint_names[i] << "_tor";
        		line=s.str();
        		s.clear();//clear any bits set
        		s.str(std::string());
			if(u[joint_names[i]][j]>=0)
       			{
				s << u[joint_names[i]][j]-torque_limit[ii];
			}
			else
			{
				s << (-1*u[joint_names[i]][j])-torque_limit[ii];	
			}
       		 	setenv(line.c_str(),s.str().c_str(),1);
    		}
 	}
	for (int i=0, ii=0; i<joint_names.size(); i++) 
  	{
    		for (int j=0; j<joint_dofs[i]; j++,ii++) 
    		{
        		if (u[joint_names[i]][j] < -torque_limit[ii] || u[joint_names[i]][j] > torque_limit[ii]) 
			{
				s.clear();
        			s.str(std::string());
        			double fail=std::stod(getenv("curr_line"))-std::stod(getenv("test_dur"));
        			s << fail;
        			setenv("fail_line",getenv("curr_line"),1);
        			setenv("fail_line_start",s.str().c_str(),1);
        			s.clear();
        			s.str(std::string());

        			std::ofstream errOut;
        			errOut.open(getenv("BUILDER_HOME_PATH")+std::string("/matlabData.txt"), std::ios::app);
				errOut << getenv("lenF1") << " " << getenv("lenF2") << " " << getenv("FfootLen") << " " << getenv("lenH1") 
					  		  << " "<< getenv("lenH2") << " "<< getenv("HfootLen") << " " << getenv("base_size_length") 
					  		  << " "<< getenv("base_size_width") << " " << getenv("base_size_height") 
					  		  << " " << getenv("FlinkRad") << " "<< getenv("HlinkRad") << " " << getenv("FfootRad") 
					  		  << " " << getenv("HfootRad") << " " << getenv("massF1") << " "<< getenv("massF2") 
					  		  << " " << getenv("massF3") << " " << getenv("massH1") << " " << getenv("massH2") 
					  		  << " "<< getenv("massH3") << " " << getenv("massBase") << " " << getenv("LF_X_1_vel") 
					  		  << " " << getenv("LF_Y_2_vel") << " "<< getenv("LF_Y_3_vel") << " " << getenv("RF_X_1_vel") 
					  	          << " " << getenv("RF_Y_2_vel") << " " << getenv("RF_Y_3_vel") << " "<< getenv("LH_X_1_vel") 
					  << " " << getenv("LH_Y_2_vel") << " " << getenv("LH_Y_3_vel") << " " << getenv("RH_X_1_vel") 
					  << " " << getenv("RH_Y_2_vel") << " " << getenv("RH_Y_3_vel") << " " << getenv("LF_X_1_tor") 
					  << " " << getenv("LF_Y_2_tor") << " " << getenv("LF_Y_3_tor") << " " << getenv("RF_X_1_tor") 
					  << " " << getenv("RF_Y_2_tor") << " " << getenv("RF_Y_3_tor") << " " << getenv("LH_X_1_tor") 
					  << " " << getenv("LH_Y_2_tor") << " " << getenv("LH_Y_3_tor") << " " << getenv("RH_X_1_tor") 
					  << " " << getenv("RH_Y_2_tor") << " " << getenv("RH_Y_3_tor") << " " << getenv("curr_vel") 
					  << " " << getenv("curr_line") << " " << getenv("modelNo") << "\n";
				errOut.close();
				setenv("curr_line","0",1);
				setenv("curr_iter","0",1);
        			jac_count++;
        			s << jac_count;
        			line=s.str();
        			setenv("jac_count",s.str().c_str(),1);
        			s.clear();
        			s.str(std::string());
    
        			double lenF1= std::stod(getenv("lenF1"));
        			double unitLen = std::stod(getenv("unit_len"));
	
				lenF1+=unitLen;
        			s << lenF1;
        			line=s.str();
        			setenv("lenF1",line.c_str(),1);
        			s.clear();
        			s.str(std::string());

				std::string generate=getenv("BUILDER_SCRIPT_PATH");
				generate+="/generate.sh";
        
        			execl(generate.c_str(), generate.c_str(), (char *) 0);
			} 
		}
	}
}
else if(jac_count<=var_names.size())
{
	for (int i=0, ii=0; i<joint_names.size(); i++) 
  	{
    		for (int j=0; j<joint_dofs[i]; j++,ii++) 
    		{
        		std::ostringstream s,s2;
        		s << jac_count<<"_" << joint_names[i] << "_vel";
        		s2 << joint_names[i] << "_vel";
        		line=s.str();
        		s.clear();//clear any bits set
        		s.str(std::string());
        		if(qd[joint_names[i]][j]>=0)
        		{
				s << floorf((qd[joint_names[i]][j]-velocity_limit[ii]-std::stod(getenv(s2.str().c_str()))/std::stod(getenv("unit_len"))) * 100) / 100;
			}
			else
			{
				s << floorf(((-1*qd[joint_names[i]][j])-velocity_limit[ii]-std::stod(getenv(s2.str().c_str()))/std::stod(getenv("unit_len"))) * 100) / 100;		
			}
        
        		setenv(line.c_str(),s.str().c_str(),1);
    		}
  	}
  
  	for (int i=0, ii=0; i<joint_names.size(); i++) 
  	{
    		for (int j=0; j<joint_dofs[i]; j++,ii++) 
    		{
        		std::ostringstream s,s2;
        		s << jac_count<<"_" << joint_names[i] << "_tor";
        		s2 << joint_names[i] << "_tor";
        		line=s.str();
        		s.clear();//clear any bits set
        		s.str(std::string());
			if(u[joint_names[i]][j]>=0)
        		{
				s << floorf((u[joint_names[i]][j]-torque_limit[ii]-std::stod(getenv(s2.str().c_str()))/std::stod(getenv("unit_len"))) * 100) / 100;
			}
			else
			{
				s << floorf(((-1*u[joint_names[i]][j])-torque_limit[ii]-std::stod(getenv(s2.str().c_str()))/std::stod(getenv("unit_len"))) * 100) / 100;	
			}
        		setenv(line.c_str(),s.str().c_str(),1);
    		}
 	}
     
                           
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
    
        double HfootRad= std::stod(getenv(var_names[var_names.size()-1].c_str()));
	
	HfootRad-=0.001;
        s << HfootRad;
        line=s.str();
        setenv("HfootRad",line.c_str(),1);
        s.clear();
        s.str(std::string());

        std::string fileLine;

	 std::ostringstream labmat;
         labmat << getenv("BUILDER_HOME_PATH") << "matlabData.txt";
         std::string mat = labmat.str();
         std::ifstream matlab(mat);

	int lines=0;
        while (std::getline(matlab, fileLine) && lines<3)
	{
           ++lines;
	}
        if(lines<2)
	{
		std::string sample=getenv("BUILDER_BIF_PATH");
		sample+="/sample/run.sh";
        
        	execl(sample.c_str(), sample.c_str(), (char *) 0);
	}
	else
	{
		std::string editor=getenv("BUILDER_GUI_PATH");
		editor+="/editor";
        
        	execl(editor.c_str(), editor.c_str(), (char *) 0);
	}
	 

}




}


void setup(){

}
