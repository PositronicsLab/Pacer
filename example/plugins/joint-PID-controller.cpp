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

class PIDController {
  public:
    Ravelin::VectorNd value;
  protected:

  struct Gains
  {
    double perr_sum;
    double kp;
    double kv;
    double ki;
  };
    
  std::vector<Gains> _gains;
  
  double get_error_feedback(int n,double perr,double derr){
      const double KP = _gains[n].kp;
      const double KV = _gains[n].kv;
      const double KI = _gains[n].ki;

      _gains[n].perr_sum += perr;
      double ierr = _gains[n].perr_sum;

      return perr*KP + derr*KV + ierr*KI;
  }
};



class JointPID : public PIDController {
public:
  JointPID() { init(); }

  Ravelin::VectorNd q_des,
                    qd_des,
                    q,
                    qd;

  void init(){
    std::map<std::string,std::vector<Gains> > gains;

    std::vector<std::string>
      &joint_names = Utility::get_variable<std::vector<std::string> >("init.joint.id");

    OUTLOG(joint_names,"joint_names",logERROR);

    std::vector<double>
        &Kp = Utility::get_variable<std::vector<double> >(plugin_namespace+".gains.kp"),
        &Kv = Utility::get_variable<std::vector<double> >(plugin_namespace+".gains.kv"),
        &Ki = Utility::get_variable<std::vector<double> >(plugin_namespace+".gains.ki"),
        &dofs = Utility::get_variable<std::vector<double> >("init.joint.dofs");

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
    value.set_zero(q.rows());

    for (unsigned i=0; i< _gains.size(); i++)
    {
      double perr = q_des[i] - q[i];
      double derr = qd_des[i] - qd[i];
      get_error_feedback(i,perr,derr);
    }
  }
};

boost::shared_ptr<JointPID> pid;

void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t){
  
    ctrl_ptr = ctrl;
    // --------------------------- JOINT FEEDBACK ------------------------------
      if(!pid)
        pid = boost::shared_ptr<JointPID>( new JointPID());
      
      pid->q      = ctrl->get_joint_generalized_value(Pacer::Controller::position);
      pid->qd     = ctrl->get_joint_generalized_value(Pacer::Controller::velocity);
      pid->q_des  = ctrl->get_joint_generalized_value(Pacer::Controller::position_goal);
      pid->qd_des = ctrl->get_joint_generalized_value(Pacer::Controller::velocity_goal);

      pid->update();

      ctrl->set_joint_generalized_value(Pacer::Controller::velocity_goal, pid->value);
}

/** This is a quick way to register your plugin function of the form:
  * void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t)
  */
#include "register_plugin"
