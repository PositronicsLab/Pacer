
#include <stdexcept>
#include <Pacer/controller.h>
#include "plugin.h"
#include <sys/types.h>
#include <sys/wait.h>

void printQandQd()
{
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  std::map<std::string, Ravelin::VectorNd > q, qd,u;

  std::vector<double> torque_limit;
  std::vector<double> velocity_limit;

  std::ofstream errOut;
  errOut.open("/home/brad/Desktop/Tests/pacer-tests/BotBuilder/matlabData.txt", std::ios::app);

  std::vector<std::string> joint_names = ctrl->get_data<std::vector<std::string> >("init.joint.id");
  std::vector<double> joint_dofs = ctrl->get_data<std::vector<double> >("init.joint.dofs");
  ctrl->get_data<std::vector<double> >("init.joint.limits.u",torque_limit);
  ctrl->get_data<std::vector<double> >("init.joint.limits.qd",velocity_limit);
  ctrl->get_joint_value(Pacer::Robot::position_goal, q);
  ctrl->get_joint_value(Pacer::Robot::velocity_goal, qd);
  ctrl->get_joint_value(Pacer::Robot::load_goal, u);

  for (int i=0, ii=0; i<joint_names.size(); i++) 
  {
    for (int j=0; j<joint_dofs[i]; j++,ii++) 
    {
        std::ostringstream s;
        s << joint_names[i] << "_vel";
        std::string line=s.str();
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
        std::string line=s.str();
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
  
errOut << getenv("lenF1") << " " << getenv("lenF2") << " " << getenv("lenH1") << " " << getenv("lenH2") << " " << getenv("base_size_length") << " "<< getenv("base_size_width")
      << " " << getenv("base_size_height") << " " << getenv("density") << " " << getenv("linkRad") << " " << getenv("footRad") << " " << getenv("footLen")
       << " "<< getenv("LF_X_1_vel") << " " << getenv("LF_Y_2_vel") << " " << getenv("LF_Y_3_vel") << " " << getenv("RF_X_1_vel") << " " << getenv("RF_Y_2_vel")
       << " " << getenv("RF_Y_3_vel") << " " << getenv("LH_X_1_vel") << " " << getenv("LF_Y_2_vel") << " " << getenv("LH_Y_3_vel") << " " << getenv("RH_X_1_vel")
       << " " << getenv("RH_Y_2_vel") << " " << getenv("RH_Y_3_vel") << " " << getenv("LF_X_1_tor") << " " << getenv("LF_Y_2_tor") << " " << getenv("LF_Y_3_tor") 
       << " " << getenv("RF_X_1_tor") << " " << getenv("RF_Y_2_tor") << " " << getenv("RF_Y_3_tor") << " " << getenv("LH_X_1_tor") << " " << getenv("LF_Y_2_tor") 
       << " " << getenv("LH_Y_3_tor") << " " << getenv("RH_X_1_tor") << " " << getenv("RH_Y_2_tor") << " " << getenv("RH_Y_3_tor") << " " << getenv("curr_vel") << " " << getenv("curr_line") << "\n";

	setenv("curr_line","0",1);
	setenv("curr_iter","0",1);
  std::string editor=getenv("BUILDER_GUI_PATH");
	editor+="/editor";
        pid_t pid=fork();
	if(pid==0)
        {execl(editor.c_str(), editor.c_str(), (char *) 0);}
	
}

void loop(){
boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
pid_t pid;
double num_rows;
ctrl->get_data("num_pose_rows",num_rows);


std::fstream myfile;
   myfile.open ("/home/brad/Desktop/Tests/pacer-tests/BotBuilder/FrontEnd/debug.txt", std::ios::in | std::ios::out | std::ios::ate);
   myfile << "----------------------------error-check.cpp---------------------------------";
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
   myfile.close();












  
  Ravelin::VectorNd generalized_fext = ctrl->get_generalized_value(Pacer::Robot::load);
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
  


  std::vector<std::string> joint_names = ctrl->get_data<std::vector<std::string> >("init.joint.id");
  std::vector<double> joint_dofs = ctrl->get_data<std::vector<double> >("init.joint.dofs");
  std::vector<double> torque_limit;
  bool apply_torque_limit = ctrl->get_data<std::vector<double> >("init.joint.limits.u",torque_limit);

  std::vector<double> velocity_limit;
  bool apply_velocity_limit = ctrl->get_data<std::vector<double> >("init.joint.limits.qd",velocity_limit);

  for (int i=0, ii=0; i<joint_names.size(); i++) {
    for (int j=0; j<joint_dofs[i]; j++,ii++) {
      // If motor speed limit met, cancel torque
      // (if applied in direction of limit)
        if (qd[joint_names[i]][j] > velocity_limit[ii]) {
          std::cout << joint_names[i] << ": qd["<<j<<"]= " << qd[joint_names[i]][j] << " exceeds velocity limit: " << velocity_limit[ii] << "\n";
          printQandQd();
	  exit(6);
        } else if  (qd[joint_names[i]][j] < -velocity_limit[ii]) {
          std::cout << joint_names[i] << ": qd["<<j<<"]= " << qd[joint_names[i]][j] << " exceeds negative velocity limit: " << -velocity_limit[ii] << "\n";
          printQandQd();
          exit(6);

        }



        // Limit torque
        if (u[joint_names[i]][j] > torque_limit[ii]) {
          std::cout << joint_names[i] << ": u["<<j<<"]= " << u[joint_names[i]][j] << " exceeds torque limit: " << torque_limit[ii] << ", setting to " << torque_limit[ii] << "\n";
          printQandQd();
          exit(6);
        } else if  (u[joint_names[i]][j] < -torque_limit[ii]) {
          std::cout << joint_names[i] << ": u["<<j<<"]= " << u[joint_names[i]][j] << " exceeds torque limit: " << -torque_limit[ii] << ", setting to " << -torque_limit[ii] << "\n";
          printQandQd();
          exit(6);
        }

    }
  }

  
}


void setup(){

}
