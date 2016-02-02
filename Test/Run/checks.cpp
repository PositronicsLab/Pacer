#ifdef USE_GTEST
#include <gtest/gtest.h>
#endif
void check_final_state(boost::shared_ptr<Ravelin::ArticulatedBodyd>& rb){
  boost::shared_ptr<RCArticulatedBodyd> robot 
    = boost::dynamic_pointer_cast<RCArticulatedBodyd>(rb);

  Ravelin::VectorNd q;
  robot->get_generalized_coordinates_euler(q);

  Ravelin::Pose3d P = Utility::vec_to_pose(q.segment(q.size()-7,q.size()));

  double x_progress = P.x[0];

  // Progress from first version:
  double last_x_progress = 1.01395;
  
#ifdef USE_GTEST
  ASSERT_LE(last_x_progress,x_progress);
#endif
}

void check_current_state(boost::shared_ptr<Ravelin::ArticulatedBodyd>& rb){
  boost::shared_ptr<RCArticulatedBodyd> robot 
    = boost::dynamic_pointer_cast<RCArticulatedBodyd>(rb);
  // Do Nothing
}
