#include <Pacer/controller.h>
#include <Pacer/utilities.h>
#include "plugin.h"

#ifdef USE_GSL
#include <Pacer/Random.h>
Random::ParamValueMap initial_value_parameters;
Random::ParamMap jitter_parameter_generator;
#endif

void loop(){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);

#ifdef USE_GSL
  Random::ParamValueMap jitter_parameters;
  Random::generate_parameters(jitter_parameter_generator,ctrl,jitter_parameters);
#endif
  
  std::vector<std::string> parameter_names;
  ctrl->get_data<std::vector<std::string> >(plugin_namespace+".parameters",parameter_names);
  
  for (int i=0; i<parameter_names.size(); i++) {
    double delay_time = 0;
#ifdef USE_GSL
    delay_time = initial_value_parameters["initial.delay.control."+parameter_names[i]][0] + jitter_parameters["jitter.delay.control."+parameter_names[i]][0];
#endif
    if (parameter_names[i].compare("kinematic") == 0) {
      static std::deque< std::pair<double,Ravelin::VectorNd> > q_queue;
      static std::deque< std::pair<double,Ravelin::VectorNd> > qd_queue;

      Ravelin::VectorNd q_des  = ctrl->get_joint_generalized_value(Pacer::Controller::position_goal);
      Ravelin::VectorNd qd_des = ctrl->get_joint_generalized_value(Pacer::Controller::velocity_goal);
      
      // Save Delayed Value into deque
      q_queue.push_back(std::pair<double,Ravelin::VectorNd>(t+delay_time,q_des));
      qd_queue.push_back(std::pair<double,Ravelin::VectorNd>(t+delay_time,qd_des));

      // Apply Delayed Value
      static Ravelin::VectorNd last_q_des = q_des;
      static Ravelin::VectorNd last_qd_des = Ravelin::VectorNd::zero(qd_des.rows());
      q_des = last_q_des;
      qd_des = last_qd_des;

      while (q_queue.front().first <= t){
        q_des = q_queue.front().second;
        qd_des = qd_queue.front().second;
        qd_queue.pop_front();
        q_queue.pop_front();
        if(q_queue.empty()) break;
      }
      last_q_des = q_des;
      last_qd_des = qd_des;

      // send to robot
      ctrl->set_joint_generalized_value(Pacer::Controller::position_goal,q_des);
      ctrl->set_joint_generalized_value(Pacer::Controller::velocity_goal,qd_des);
//      ctrl->set_joint_generalized_value(Pacer::Controller::acceleration_goal,qdd_des);
      
    } else if (parameter_names[i].compare("dynamic") == 0) {
      static std::deque< std::pair<double,Ravelin::VectorNd> > u_queue;
      
      Ravelin::VectorNd u_des  = ctrl->get_joint_generalized_value(Pacer::Controller::load_goal);

      // Save Delayed Value into deque
      u_queue.push_back(std::pair<double,Ravelin::VectorNd>(t+delay_time,u_des));

      // Apply Delayed Value
      static Ravelin::VectorNd last_u_des = Ravelin::VectorNd::zero(u_des.rows());
      u_des = last_u_des;
      while (u_queue.front().first <= t){
        u_des = u_queue.front().second;
        u_queue.pop_front();
        if(u_queue.empty()) break;
      }
      last_u_des = u_des;

      // send to robot
      ctrl->set_joint_generalized_value(Pacer::Controller::load_goal,u_des);
  } else if (parameter_names[i].compare("end-effector") == 0) {
    static std::deque< std::pair< double , std::map<std::string, Ravelin::Origin3d > > > x_queue;
    static std::deque< std::pair< double , std::map<std::string, Ravelin::Origin3d > > > xd_queue;
//    static std::deque< std::pair< double , std::map<std::string, Ravelin::Origin3d > > > xdd_queue;
    
    std::map<std::string, Ravelin::Origin3d > x_des;
    std::map<std::string, Ravelin::Origin3d > xd_des;
//    std::map<std::string, Ravelin::Origin3d > xdd_des;
    
    ctrl->get_end_effector_value(Pacer::Controller::position_goal,x_des);
    ctrl->get_end_effector_value(Pacer::Controller::velocity_goal,xd_des);
//    ctrl->get_end_effector_value(Pacer::Controller::acceleration_goal,xdd_des);
    
    // Save Delayed Value into deque
    x_queue.push_back(std::pair<double,std::map<std::string, Ravelin::Origin3d > >(t+delay_time,x_des));
    xd_queue.push_back(std::pair<double,std::map<std::string, Ravelin::Origin3d > >(t+delay_time,xd_des));
//    xdd_queue.push_back(std::pair<double,std::map<std::string, Ravelin::Origin3d > >(t+delay_time,xdd_des));
    
    // Apply Delayed Value
    static std::map<std::string, Ravelin::Origin3d > last_x_des = x_des;
    static std::map<std::string, Ravelin::Origin3d > last_xd_des = xd_des;
//    static std::map<std::string, Ravelin::Origin3d > last_xdd_des = xdd_des;
    x_des = last_x_des;
    xd_des = last_xd_des;
//    xdd_des = last_xdd_des;
    while (x_queue.front().first <= t){
      x_des = x_queue.front().second;
      xd_des = xd_queue.front().second;
//      xdd_des = xdd_queue.front().second;
      x_queue.pop_front();
      xd_queue.pop_front();
//      xdd_queue.pop_front();
      if(x_queue.empty()) break;
    }
    last_x_des = x_des;
    last_xd_des = xd_des;
//    last_xdd_des = xdd_des;
    
    // send to robot
    ctrl->set_end_effector_value(Pacer::Controller::position_goal,x_des);
    ctrl->set_end_effector_value(Pacer::Controller::velocity_goal,xd_des);
//    ctrl->set_end_effector_value(Pacer::Controller::acceleration_goal,xdd_des);
  }
  }
}

void setup(){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
#ifdef USE_GSL
  // Parse imported values for 'control jitter'
  Random::create_distributions("control-uncertainty-jitter",ctrl,jitter_parameter_generator,true);
  
  // Parse imported values for 'control inital values'
  Random::ParamMap initial_value_parameter_generator;
  Random::create_distributions("initial-control-uncertainty",ctrl,initial_value_parameter_generator,true);
  Random::generate_parameters(initial_value_parameter_generator,ctrl,initial_value_parameters);
#endif

}