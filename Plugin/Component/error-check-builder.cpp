
#include <stdexcept>
#include <Pacer/controller.h>
#include "plugin.h"
#include <sys/types.h>
#include <sys/wait.h>

void loop(){
boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  
  Ravelin::VectorNd generalized_fext = ctrl->get_generalized_value(Pacer::Robot::load);
  std::ostringstream s;
 std::string line;
  std::vector<std::string> var_names={"lenF1","lenF2","FfootLen","lenH1","lenH2","HfootLen","base_size_length","base_size_width","base_size_height","FlinkRad","HlinkRad","density","FfootRad","HfootRad"};

/*
  // Unstable simulation.
  if(generalized_fext.norm() > 1e8)
    throw std::runtime_error("Robot exploded!");

  // Fell off edge
  Ravelin::Vector3d x;
  ctrl->get_data<Ravelin::Vector3d>("base.state.x",x);
  if(x[2] < -1)
    throw std::runtime_error("Robot fell!");

  boost::shared_ptr<Ravelin::Pose3d> base_link_frame(new Ravelin::Pose3d(ctrl->get_data<Ravelin::Pose3d>("base_link_frame")));

  Ravelin::Vector3d up(0,0,1,base_link_frame);
  up = Ravelin::Pose3d::transform_vector(Pacer::GLOBAL,up);
  if(up[2] < 0)
  {
    throw std::runtime_error("Robot flipped over!");
  }

    
  std::vector< boost::shared_ptr< Pacer::Robot::contact_t> > c;
  ctrl->get_link_contacts("BODY0",c);
  if(c.size() != 0)
    throw std::runtime_error("Robot body contacted ground!");
    
 // bounced too high!
  Ravelin::Vector3d z;
  ctrl->get_data<Ravelin::Vector3d>("base.state.com",z);
  if(z.z() >.1)
    throw std::runtime_error("Robot Bounced!");*/

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

if(jac_count==0)
{
  for (int i=0, ii=0; i<joint_names.size(); i++) {
    for (int j=0; j<joint_dofs[i]; j++,ii++) {
      
        if (u[joint_names[i]][j] < -torque_limit[ii] || u[joint_names[i]][j] > torque_limit[ii] || qd[joint_names[i]][j] < -velocity_limit[ii] || qd[joint_names[i]][j] > velocity_limit[ii]) {


          s.clear();
          s.str(std::string());
          double fail=std::stod(getenv("curr_line"))-std::stod(getenv("test_dur"));
          s << fail;
          setenv("fail_line",getenv("curr_line"),1);
          setenv("fail_line_start",s.str().c_str(),1);
          s.clear();
          s.str(std::string());

          std::ofstream errOut;
  errOut.open("/home/brad/Desktop/Tests/pacer-tests/BotBuilder/matlabData.txt", std::ios::app);
	  errOut << getenv("lenF1") << " " << getenv("lenF2") << " " << getenv("lenH1") << " " << getenv("lenH2") << " " << getenv("base_size_length") << " "<< getenv("base_size_width") << " " << getenv("base_size_height") << " " << getenv("density") << " " << getenv("FlinkRad") << " " << getenv("HlinkRad") << " " << getenv("FfootRad") << " " << getenv("HfootRad") << " " << getenv("FfootLen") << " " << getenv("HfootLen") << " "<< getenv("LF_X_1_vel") << " " << getenv("LF_Y_2_vel") << " " << getenv("LF_Y_3_vel") << " " << getenv("RF_X_1_vel") << " " << getenv("RF_Y_2_vel")
       << " " << getenv("RF_Y_3_vel") << " " << getenv("LH_X_1_vel") << " " << getenv("LH_Y_2_vel") << " " << getenv("LH_Y_3_vel") << " " << getenv("RH_X_1_vel")
       << " " << getenv("RH_Y_2_vel") << " " << getenv("RH_Y_3_vel") << " " << getenv("LF_X_1_tor") << " " << getenv("LF_Y_2_tor") << " " << getenv("LF_Y_3_tor") 
       << " " << getenv("RF_X_1_tor") << " " << getenv("RF_Y_2_tor") << " " << getenv("RF_Y_3_tor") << " " << getenv("LH_X_1_tor") << " " << getenv("LH_Y_2_tor") 
       << " " << getenv("LH_Y_3_tor") << " " << getenv("RH_X_1_tor") << " " << getenv("RH_Y_2_tor") << " " << getenv("RH_Y_3_tor") << " " << getenv("curr_vel") << " " << getenv("curr_line") << "\n";
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
        else
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
        {s << qd[joint_names[i]][j]-velocity_limit[ii];}
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
        {s << u[joint_names[i]][j]-torque_limit[ii];}
	else
	{
		s << (-1*u[joint_names[i]][j])-torque_limit[ii];	
	}
        setenv(line.c_str(),s.str().c_str(),1);
    }
 }
        }
}}

}
else if(jac_count<15)
{

        
     
                           
        if (std::stod(getenv("fail_line_start"))==std::stod(getenv("fail_line"))) {

	std::cout << "\n" << "jac_count: " << jac_count << "\n";
        if(jac_count<14)   
        {double pastVar= std::stod(getenv(var_names[jac_count-1].c_str()));
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
        else 
        {
             
        for (int i=0, ii=0; i<joint_names.size(); i++) 
  {
    for (int j=0; j<joint_dofs[i]; j++,ii++) 
    {
        std::ostringstream s,s2;
        s << jac_count<<"_" << joint_names[i] << "_vel";
        std::cout << "\n" << s.str() << "\n";
        s2 << joint_names[i] << "_vel";
        line=s.str();
        s.clear();//clear any bits set
        s.str(std::string());
        if(qd[joint_names[i]][j]>=0)
        {s << floorf((qd[joint_names[i]][j]-velocity_limit[ii]-std::stod(getenv(s2.str().c_str()))/std::stod(getenv("unit_len"))) * 100) / 100;}
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
        {s << floorf((u[joint_names[i]][j]-torque_limit[ii]-std::stod(getenv(s2.str().c_str()))/std::stod(getenv("unit_len"))) * 100) / 100;}
	else
	{
		s << floorf(((-1*u[joint_names[i]][j])-torque_limit[ii]-std::stod(getenv(s2.str().c_str()))/std::stod(getenv("unit_len"))) * 100) / 100;	
	}
        setenv(line.c_str(),s.str().c_str(),1);
    }
 }
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

	  std::string editor=getenv("BUILDER_GUI_PATH");
	  editor+="/editor";
        
        execl(editor.c_str(), editor.c_str(), (char *) 0);
	 

}




}


void setup(){

}
