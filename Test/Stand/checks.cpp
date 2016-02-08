#ifdef USE_GTEST
#include <gtest/gtest.h>
#endif

void check_final_state(boost::shared_ptr<Ravelin::ArticulatedBodyd>& rb){
  boost::shared_ptr<RCArticulatedBodyd> robot 
    = boost::dynamic_pointer_cast<RCArticulatedBodyd>(rb);

  int n_dofs = q_0.size();
  int joint_dofs = n_dofs-7;
  Ravelin::Pose3d P0 = Utility::vec_to_pose(q_0.segment(joint_dofs,n_dofs));
  Ravelin::Pose3d Pf = Utility::vec_to_pose(q_f.segment(joint_dofs,n_dofs));
  
  const double MAX_ERROR = 1.0e-5;
  // Joints
  Ravelin::VectorNd expected_joints = q_0.segment(0,joint_dofs);
  Ravelin::VectorNd actual_joints = q_f.segment(0,joint_dofs);
  Ravelin::VectorNd joints_error;
  (joints_error = actual_joints) -= expected_joints;
  // check error
#ifdef USE_GTEST
  ASSERT_GT(MAX_ERROR,joints_error.norm_inf()) << "Joints:\n" <<  actual_joints << "\n(actual) != (expected)\n" << expected_joints;
#else
  if(MAX_ERROR <= joints_error.norm_inf()){
    std::cerr << "Joints:\n" <<  actual_joints << "\n(actual) != (expected)\n" << expected_joints;
    exit(EXIT_FAILURE);
  }
#endif

  // base linear
  Ravelin::Origin3d expected_linear = P0.x;
  Ravelin::Origin3d actual_linear = Pf.x;
  // check error
#ifdef USE_GTEST
  ASSERT_GT(MAX_ERROR,(actual_linear-expected_linear).norm_inf()) << "Linear: " << actual_linear << " (actual) != (expected) " << expected_linear;
#else
  if(MAX_ERROR <= (actual_linear-expected_linear).norm_inf()){
    std::cerr << "Linear: " << actual_linear << " (actual) != (expected) " << expected_linear;
    exit(EXIT_FAILURE);
  }
#endif
  
  // base angular
  Ravelin::Origin3d expected_angular;
  P0.q.to_rpy(expected_angular);
  Ravelin::Origin3d actual_angular;
  Pf.q.to_rpy(actual_angular);
  // check error
#ifdef USE_GTEST
  ASSERT_GT(MAX_ERROR,(actual_angular-expected_angular).norm_inf()) << "Angular: " <<  actual_angular << " (actual) != (expected) " << expected_angular;
#else
  if(MAX_ERROR <= (actual_angular-expected_angular).norm_inf()){
    std::cerr << "Angular: " <<  actual_angular << " (actual) != (expected) " << expected_angular;
    exit(EXIT_FAILURE);
  }
#endif

  // Velocity
#ifdef USE_GTEST
  ASSERT_GT(MAX_ERROR,qd_f.norm_inf()) << "Generalized Velocity:\n " <<  qd_f << "\n (actual) != (expected) 0";
#else
  if(MAX_ERROR <= qd_f.norm_inf()){
    std::cerr << "Generalized Velocity:\n " <<  qd_f << "\n (actual) != (expected) 0";
    exit(EXIT_FAILURE);
  }
#endif

}

void check_current_state(boost::shared_ptr<Ravelin::ArticulatedBodyd>& rb){
  boost::shared_ptr<RCArticulatedBodyd> robot 
    = boost::dynamic_pointer_cast<RCArticulatedBodyd>(rb);
  // Do Nothing
}
