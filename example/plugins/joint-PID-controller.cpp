/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <sstream>
#include <Pacer/controller.h>
#include <Pacer/utilities.h>

std::string plugin_namespace;

boost::shared_ptr<Pacer::Controller> ctrl_ptr;

class JointPID {
public:
  JointPID() { init(); }

    Ravelin::VectorNd u;

  struct Gains
  {
    double perr_sum;
    double kp;
    double kv;
    double ki;
  };
    
  std::vector<Gains> _gains;
  
  Ravelin::VectorNd q_des,
                    qd_des,
                    q,
                    qd;

  void init(){
    std::map<std::string,std::vector<Gains> > gains;

    static std::vector<std::string>
      joint_names = ctrl_ptr->get_data<std::vector<std::string> >("init.joint.id");

    OUTLOG(joint_names,"joint_names",logERROR);

    static std::vector<double>
        Kp = ctrl_ptr->get_data<std::vector<double> >(plugin_namespace+"gains.kp"),
        Kv = ctrl_ptr->get_data<std::vector<double> >(plugin_namespace+"gains.kv"),
        Ki = ctrl_ptr->get_data<std::vector<double> >(plugin_namespace+"gains.ki"),
        dofs = ctrl_ptr->get_data<std::vector<double> >("init.joint.dofs");

    OUTLOG(Kp,"Kp",logERROR);
    OUTLOG(Kv,"Kv",logERROR);
    OUTLOG(Ki,"Ki",logERROR);
       OUTLOG(dofs,"dofs",logERROR);
 
    int num_dofs = std::accumulate ( dofs.begin( ) , dofs.end( ) , 0.0 ) ;
    assert(num_dofs == Kp.size());
    assert(num_dofs == Kv.size());
    assert(num_dofs == Ki.size());
    
    for(int i=0,ii=0;i<joint_names.size();i++){
      gains[joint_names[i]] = std::vector<Gains>(dofs[i]);
      for(int j=0;j<dofs[i];j++,ii++){
        gains[joint_names[i]][j].kp = Kp[ii];
        gains[joint_names[i]][j].kv = Kv[ii];
        gains[joint_names[i]][j].ki = Ki[ii];
        gains[joint_names[i]][j].perr_sum = 0;
      }
    }
    
    ctrl_ptr->convert_to_generalized<Gains>(gains,_gains);
    
    OUT_LOG(logERROR) << "Controller: " << plugin_namespace << " inited!";
  }

  void update(){
    qd_des.set_zero();
          
    Ravelin::VectorNd perr = q;
    perr -= q_des;
    Ravelin::VectorNd derr = qd;
    derr -= qd_des;
      
    assert(q.rows() == _gains.size());
    u.set_zero(q.rows());

    for (int i=0; i< q.rows(); i++)
    {
      const double KP = _gains[i].kp;
      const double KV = _gains[i].kv;
      const double KI = _gains[i].ki;

      _gains[i].perr_sum += perr[i];
      double ierr = _gains[i].perr_sum;
      
      u[i] = -(perr[i]*KP + derr[i]*KV + ierr*KI);
    }
  }
};

void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t){
  
    ctrl_ptr = ctrl;
    // --------------------------- JOINT FEEDBACK ------------------------------
      static JointPID pid;
      
      pid.q_des  = ctrl->get_joint_generalized_value(Pacer::Controller::position_goal);
      pid.qd_des = ctrl->get_joint_generalized_value(Pacer::Controller::velocity_goal);
      pid.q  = ctrl->get_joint_generalized_value(Pacer::Controller::position);
      pid.qd = ctrl->get_joint_generalized_value(Pacer::Controller::velocity);
      
      OUTLOG(pid.q,"joint_pid_q",logDEBUG);
      OUTLOG(pid.qd,"joint_pid_qd",logDEBUG);
      OUTLOG(pid.q_des,"joint_pid_q_des",logDEBUG);
      OUTLOG(pid.qd,"joint_pid_qd_des",logDEBUG);
      
      pid.update();
    
      Ravelin::VectorNd u = ctrl->get_joint_generalized_value(Pacer::Controller::load_goal);
      OUTLOG(pid.u,"joint_pid_U",logDEBUG);
      u += pid.u;
      ctrl->set_joint_generalized_value(Pacer::Controller::load_goal,u);
      
}

/** This is a quick way to register your plugin function of the form:
  * void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t)
  */
#include "register-plugin"
