#include <Pacer/controller.h>
#include <Pacer/utilities.h>

std::string plugin_namespace;

void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t){
  
  static double start_time = t;
  
  const  std::vector<std::string>
  eef_names_ = ctrl->get_data<std::vector<std::string> >("init.end-effector.id");
  
  boost::shared_ptr<Ravelin::Pose3d> base_frame( new Ravelin::Pose3d(ctrl->get_data<Ravelin::Pose3d>("base_link_frame")));
  
  Ravelin::VectorNd qd  = ctrl->get_joint_generalized_value(Pacer::Controller::velocity);
  Ravelin::VectorNd qdd = ctrl->get_joint_generalized_value(Pacer::Controller::acceleration);
  // Initialize end effectors
  Ravelin::VectorNd local_q = ctrl->get_generalized_value(Pacer::Controller::position);
  int NUM_JOINT_DOFS = qd.rows();
  local_q.set_sub_vec(NUM_JOINT_DOFS,Utility::pose_to_vec(Ravelin::Pose3d()));

  ctrl->set_model_state(local_q);

  for(unsigned i=0;i<eef_names_.size();i++){
    Ravelin::Origin3d xd,xdd;
    
    // Calc jacobian for AB at this EEF
    Ravelin::MatrixNd J = ctrl->calc_link_jacobian(local_q,eef_names_[i]);
    
    // Now that model state is set ffrom jacobian calculation
    const boost::shared_ptr<Ravelin::RigidBodyd>  link = ctrl->get_link(eef_names_[i]);
    Ravelin::Origin3d x(Ravelin::Pose3d::transform_point(Pacer::GLOBAL,Ravelin::Vector3d(0,0,0,link->get_pose())).data());
    bool new_var = ctrl->set_data<Ravelin::Origin3d>(eef_names_[i]+".state.x",x);
    
    J.block(0,3,0,NUM_JOINT_DOFS).mult(qd,xd);
    J.block(0,3,0,NUM_JOINT_DOFS).mult(qdd,xdd);
    
    ctrl->set_data<Ravelin::Origin3d>(eef_names_[i]+".state.xd",xd);
    ctrl->set_data<Ravelin::Origin3d>(eef_names_[i]+".state.xdd",xdd);

    if(new_var){
      ctrl->set_data<Ravelin::Origin3d>(eef_names_[i]+".init.x",x);
      ctrl->set_data<Ravelin::Origin3d>(eef_names_[i]+".init.xd",xd);
    }

    ctrl->set_data<bool>(eef_names_[i]+".stance",false);

    // if no goal, set goal to current position
   if(!ctrl->get_data<Ravelin::Origin3d>(eef_names_[i]+".goal.x",x)){
      ctrl->set_data<Ravelin::Origin3d>(eef_names_[i]+".goal.x",x);
      ctrl->set_data<Ravelin::Origin3d>(eef_names_[i]+".goal.xd",xd);
      ctrl->set_data<Ravelin::Origin3d>(eef_names_[i]+".goal.xdd",xdd);
    }
    
  }
  
  if(start_time == t){
    local_q.segment(0,qd.rows()) = Ravelin::VectorNd::zero(qd.rows());
    ctrl->set_model_state(local_q);
    for(unsigned i=0;i<eef_names_.size();i++){
      Ravelin::Origin3d x,base_x;
      ctrl->get_data<Ravelin::Origin3d>(eef_names_[i]+".init.x",x);
      base_x = x;
      base_x[2] = 0;
      ctrl->set_data<Ravelin::Origin3d>(eef_names_[i]+".base",base_x);
      
      const boost::shared_ptr<Ravelin::RigidBodyd>  link = ctrl->get_link(eef_names_[i]);
      Ravelin::Origin3d x_reaching(Ravelin::Pose3d::transform_point(Pacer::GLOBAL,Ravelin::Vector3d(0,0,0,link->get_pose())).data());
      
      // write in maximum reach from limb base
      ctrl->set_data<double>(eef_names_[i]+".reach",(x_reaching-base_x).norm());
    }
  }
}


/** This is a quick way to register your plugin function of the form:
 * void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t)
 */
#include "register-plugin"