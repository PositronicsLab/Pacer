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