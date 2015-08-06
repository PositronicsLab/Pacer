#include <Pacer/controller.h>
std::string plugin_namespace;

void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t){
  const  std::vector<std::string>
  eef_names_ = ctrl->get_data<std::vector<std::string> >("init.end-effector.id");
  
  boost::shared_ptr<Ravelin::Pose3d> base_frame( new Ravelin::Pose3d(ctrl->get_data<Ravelin::Pose3d>("base_link_frame")));
  
  Ravelin::VectorNd qd  = ctrl->get_joint_generalized_value(Pacer::Controller::velocity);
  Ravelin::VectorNd qdd = ctrl->get_joint_generalized_value(Pacer::Controller::acceleration);
  // Initialize end effectors
  Ravelin::VectorNd local_q = ctrl->get_generalized_value(Pacer::Controller::position);
  int NUM_JOINT_DOFS = qd.rows();
  local_q.set_sub_vec(NUM_JOINT_DOFS,Utility::pose_to_vec(Ravelin::Pose3d()));

  for(unsigned i=0;i<eef_names_.size();i++){
    ctrl->set_model_state(local_q);
    Ravelin::Vector3d xd,xdd;
    
    // Calc jacobian for AB at this EEF
    Ravelin::MatrixNd J = ctrl->calc_link_jacobian(local_q,eef_names_[i]);
    
    // Now that model state is set ffrom jacobian calculation
    const Moby::RigidBodyPtr link = ctrl->get_link(eef_names_[i]);
    Ravelin::Vector3d x = Ravelin::Pose3d::transform_point(Moby::GLOBAL,Ravelin::Vector3d(0,0,0,link->get_pose()));
    x.pose = base_frame;
    bool new_var = ctrl->set_data<Ravelin::Vector3d>(eef_names_[i]+".state.x",x);
    
    J.block(0,3,0,NUM_JOINT_DOFS).mult(qd,xd);
    J.block(0,3,0,NUM_JOINT_DOFS).mult(qdd,xdd);
    
    xd.pose = base_frame;
    xdd.pose = base_frame;

    ctrl->set_data<Ravelin::Vector3d>(eef_names_[i]+".state.xd",xd);
    ctrl->set_data<Ravelin::Vector3d>(eef_names_[i]+".state.xdd",xdd);
    if(new_var){
      ctrl->set_data<Ravelin::Vector3d>(eef_names_[i]+".init.x",x);
      ctrl->set_data<Ravelin::Vector3d>(eef_names_[i]+".init.xd",xd);
    }
  }
}


/** This is a quick way to register your plugin function of the form:
 * void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t)
 */
#include "register-plugin"