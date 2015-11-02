void check_final_state(boost::shared_ptr<Ravelin::ArticulatedBodyd>& rb){
  boost::shared_ptr<RCArticulatedBodyd> robot 
    = boost::dynamic_pointer_cast<RCArticulatedBodyd>(rb);

  int n_dofs = q_0.size();
  int joint_dofs = n_dofs-7;
  Ravelin::Pose3d P0 = Utility::vec_to_pose(q_0.segment(joint_dofs,n_dofs));
  Ravelin::Pose3d Pf = Utility::vec_to_pose(q_f.segment(joint_dofs,n_dofs));

}

void check_current_state(boost::shared_ptr<Ravelin::ArticulatedBodyd>& rb){
  boost::shared_ptr<RCArticulatedBodyd> robot 
    = boost::dynamic_pointer_cast<RCArticulatedBodyd>(rb);
  // Do Nothing
}
