#ifdef USE_GTEST
#include <gtest/gtest.h>
#endif

void check_final_state(boost::shared_ptr<Ravelin::ArticulatedBodyd>& rb){

}

void check_current_state(boost::shared_ptr<Ravelin::ArticulatedBodyd>& rb){
  boost::shared_ptr<RCArticulatedBodyd> robot 
    = boost::dynamic_pointer_cast<RCArticulatedBodyd>(rb);
  // Do Nothing
}
