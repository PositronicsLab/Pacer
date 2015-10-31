void check_final_state(boost::shared_ptr<Ravelin::ArticulatedBodyd>& rb){
  boost::shared_ptr<RCArticulatedBodyd> robot 
    = boost::dynamic_pointer_cast<RCArticulatedBodyd>(rb);

  Ravelin::VectorNd q;
  robot->get_generalized_coordinates_euler(q);

  Ravelin::Pose3d P = Utility::vec_to_pose(q.segment(q.size()-7,q.size()));

  Ravelin::Origin3d actual_linear = P.x;
  Ravelin::Origin3d expected_linear(0,0,0.18129)
  
  Ravelin::Origin3d actual_angular;
  P.q.to_rpy(actual_angular);
  Ravelin::Origin3d expected_angular(0,0,0)

  // Progress from first version:
  ASSERT_GT(1e-5,(actual_linear-expected_linear).norm_inf());
  ASSERT_GT(1e-5,(actual_angular-expected_angular).norm_inf());
}
