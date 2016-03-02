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
  
  int NUM_JOINT_DOFS = qd.rows();
  
  // Initialize end effectors
  Ravelin::VectorNd local_q = ctrl->get_generalized_value(Pacer::Controller::position);

#ifdef USE_OSG_DISPLAY
  // 1 w/ base position
  ctrl->set_model_state(local_q);
  for(unsigned i=0;i<NUM_FEET;i++){
    Ravelin::Pose3d foot_pose(Ravelin::Matrix3d(link->get_pose()->q)*Ravelin::Matrix3d(0,0,-1, -1,0,0, 0,1,0),link->get_pose()->x,link->get_pose()->rpose);
    foot_pose.update_relative_pose(Pacer::GLOBAL);
    VISUALIZE(POSE(foot_pose,0.8));
    OUT_LOG(logERROR) << eef_names_[i] << "-orientation: " << t << " " << foot_pose.q;
  }
#endif

  // q w/o base position
  if(ctrl->floating_base())
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
   
    
    Ravelin::Pose3d foot_pose(Ravelin::Matrix3d(link->get_pose()->q)*Ravelin::Matrix3d(0,0,-1, -1,0,0, 0,1,0),link->get_pose()->x,link->get_pose()->rpose);
    foot_pose.update_relative_pose(Pacer::GLOBAL);
    

//    Ravelin::Origin3d x(Ravelin::Pose3d::transform_point(Pacer::GLOBAL,Ravelin::Vector3d(0,0,0,link->get_pose())).data());
    bool new_var = ctrl->set_data<Ravelin::Origin3d>(eef_names_[i]+".state.x",foot_pose.x);
    ctrl->set_data<Ravelin::Quatd>(eef_names_[i]+".state.q",foot_pose.q);
    
    J.block(0,3,0,NUM_JOINT_DOFS).mult(qd,xd);
    J.block(0,3,0,NUM_JOINT_DOFS).mult(qdd,xdd);
    J.block(3,6,0,NUM_JOINT_DOFS).mult(qd,axd);
    J.block(3,6,0,NUM_JOINT_DOFS).mult(qdd,axdd);
    
    ctrl->set_data<Ravelin::Origin3d>(eef_names_[i]+".state.xd",xd);
    ctrl->set_data<Ravelin::Origin3d>(eef_names_[i]+".state.xdd",xdd);
    ctrl->set_end_effector_value(eef_names_[i],Pacer::Controller::position,foot_pose.x);
    ctrl->set_end_effector_value(eef_names_[i],Pacer::Controller::velocity,xd);
    ctrl->set_end_effector_value(eef_names_[i],Pacer::Controller::acceleration,xdd);
    ctrl->set_end_effector_value(eef_names_[i],Pacer::Controller::load,Ravelin::Origin3d(0,0,0));

    if(new_var){
      ctrl->set_data<Ravelin::Quatd>(eef_names_[i]+".init.q",foot_pose.q);
      ctrl->set_data<Ravelin::Origin3d>(eef_names_[i]+".init.x",foot_pose.x);
      ctrl->set_data<Ravelin::Origin3d>(eef_names_[i]+".init.xd",xd);
    }

    ctrl->set_data<bool>(eef_names_[i]+".stance",false);
  }
  
  if(start_time == t){
    local_q.segment(0,qd.rows()) = Ravelin::VectorNd::zero(qd.rows());
    ctrl->set_model_state(local_q);
    std::vector<Ravelin::Origin3d> x1(NUM_FEET) ,x2(NUM_FEET);

    for(unsigned i=0;i<NUM_FEET;i++){
      Ravelin::Origin3d x,base_x;
      ctrl->get_data<Ravelin::Origin3d>(eef_names_[i]+".init.x",x);
      base_x = x;
      base_x[2] = 0;
      ctrl->set_data<Ravelin::Origin3d>(eef_names_[i]+".base",base_x);
      x2[i] = base_x;
      const boost::shared_ptr<Ravelin::RigidBodyd>  link = ctrl->get_link(eef_names_[i]);
      x1[i] = Ravelin::Origin3d(Ravelin::Pose3d::transform_point(Pacer::GLOBAL,Ravelin::Vector3d(0,0,0,link->get_pose())).data());
    }
    
    std::fill(local_q.segment(0,qd.rows()).begin(),local_q.segment(0,qd.rows()).end(),M_PI);
    ctrl->set_model_state(local_q);

    for(unsigned i=0;i<NUM_FEET;i++){
//      const boost::shared_ptr<Ravelin::RigidBodyd>  link = ctrl->get_link(eef_names_[i]);
//      x2[i] = Ravelin::Origin3d(Ravelin::Pose3d::transform_point(Pacer::GLOBAL,Ravelin::Vector3d(0,0,0,link->get_pose())).data());

      // write in maximum reach from limb base
      double reach = (x1[i]-x2[i]).norm();
      ctrl->set_data<double>(eef_names_[i]+".reach",reach*0.95);
    }
  }
}

void setup(){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);

  const  std::vector<std::string>
  eef_names_ = ctrl->get_data<std::vector<std::string> >("init.end-effector.id");
  int NUM_FEET = eef_names_.size();

  for(unsigned i=0;i<NUM_FEET;i++){
   variable_names.push_back(eef_names_[i]+".stance");
   variable_names.push_back(eef_names_[i]+".state.x");
   variable_names.push_back(eef_names_[i]+".state.xd");
   variable_names.push_back(eef_names_[i]+".state.xdd");
   variable_names.push_back(eef_names_[i]+".init.x");
   variable_names.push_back(eef_names_[i]+".init.xd");
   variable_names.push_back(eef_names_[i]+".base");
   variable_names.push_back(eef_names_[i]+".reach");
  }
}
