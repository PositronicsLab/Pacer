#include <Pacer/controller.h>
#include <Pacer/utilities.h>

#include "plugin.h"

void loop(){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  static double start_time = t;
  
  const  std::vector<std::string>
  eef_names_ = ctrl->get_data<std::vector<std::string> >("init.end-effector.id");
  
  int NUM_FEET = eef_names_.size();
  
  boost::shared_ptr<Ravelin::Pose3d> base_frame( new Ravelin::Pose3d(ctrl->get_data<Ravelin::Pose3d>("base_link_frame")));
  
  Ravelin::VectorNd qd  = ctrl->get_joint_generalized_value(Pacer::Controller::velocity);
  Ravelin::VectorNd qdd = ctrl->get_joint_generalized_value(Pacer::Controller::acceleration);
  // Initialize end effectors
  Ravelin::VectorNd local_q = ctrl->get_generalized_value(Pacer::Controller::position);
  int NUM_JOINT_DOFS = qd.rows();
  local_q.set_sub_vec(NUM_JOINT_DOFS,Utility::pose_to_vec(Ravelin::Pose3d()));
  
  ctrl->set_model_state(local_q);
  
  for(unsigned i=0;i<NUM_FEET;i++){
    Ravelin::Origin3d xd,xdd;
    //angular
    Ravelin::Origin3d rpy,axd,axdd;
    
    // Calc jacobian for AB at this EEF
    Ravelin::MatrixNd J = ctrl->calc_link_jacobian(local_q,eef_names_[i]);
    
    // Now that model state is set ffrom jacobian calculation
    const boost::shared_ptr<Ravelin::RigidBodyd>  link = ctrl->get_link(eef_names_[i]);
    
    // Pose of foot
    Ravelin::Pose3d foot_pose(Ravelin::Matrix3d(link->get_pose()->q)*Ravelin::Matrix3d(0,0,-1, -1,0,0, 0,1,0),link->get_pose()->x,link->get_pose()->rpose);
    foot_pose.update_relative_pose(Pacer::GLOBAL);
    Utility::visualize.push_back( Pacer::VisualizablePtr( new Pacer::Pose(foot_pose,0.8)));
    
//    OUTLOG(logERROR) << eef_names_[i] << "-orientation: " << t << " " << foot_pose.q;
    OUTLOG(foot_pose.q,eef_names_[i]+".state.q",logERROR);
    OUTLOG(foot_pose.x,eef_names_[i]+".state.x",logERROR);
    
    J.block(0,3,0,NUM_JOINT_DOFS).mult(qd,xd);
    J.block(0,3,0,NUM_JOINT_DOFS).mult(qdd,xdd);
    J.block(3,6,0,NUM_JOINT_DOFS).mult(qd,axd);
    J.block(3,6,0,NUM_JOINT_DOFS).mult(qdd,axdd);
    
    OUTLOG(xd,eef_names_[i]+".state.xd",logERROR);
    OUTLOG(xdd,eef_names_[i]+".state.xdd",logERROR);
    OUTLOG(axd,eef_names_[i]+".state.axd",logERROR);
    OUTLOG(axdd,eef_names_[i]+".state.axdd",logERROR);

  }
}

void setup(){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
}
