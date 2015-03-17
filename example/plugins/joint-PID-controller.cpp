/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <sstream>
#include <Pacer/controller.h>
#include <Pacer/utilities.h>

std::string plugin_namespace;

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
  
  std::map<std::string,Gains> _gains;
  
  double get_error_feedback(std::string& n,double perr,double derr){
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

  std::vector<std::string> joint_names;

  void init(){
    joint_names = Utility::get_variable<std::vector<std::string> >(plugin_namespace+".id");

    OUTLOG(joint_names,"joint_names",logERROR);

    std::vector<double>
        &Kp = Utility::get_variable<std::vector<double> >(plugin_namespace+".gains.kp"),
        &Kv = Utility::get_variable<std::vector<double> >(plugin_namespace+".gains.kv"),
        &Ki = Utility::get_variable<std::vector<double> >(plugin_namespace+".gains.ki");

    for(int i=0;i<joint_names.size();i++){
      _gains[joint_names[i]].kp = Kp[i];
      _gains[joint_names[i]].kv = Kv[i];
      _gains[joint_names[i]].ki = Ki[i];
      _gains[joint_names[i]].perr_sum = 0;
    }

    OUTLOG(Kp,"Kp",logERROR);
    OUTLOG(Kv,"Kv",logERROR);
    OUTLOG(Ki,"Ki",logERROR);
    OUT_LOG(logERROR) << "Controller: " << plugin_namespace << " inited!";
  }

  void update(){
    value.set_zero(q.rows());

    // get the two gains
    unsigned num_joints = joint_names.size();
    for (unsigned i=0; i< num_joints; i++)
    {
      std::string& joint_name = joint_names[i];
      // add feedback torque to joints
      double perr = q_des[i] - q[i];
      double derr = qd_des[i] - qd[i];
      get_error_feedback(joint_name,perr,derr);
    }
  }
};

void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t){
  int &ERROR_FEEDBACK = Utility::get_variable<int>("controller.error-feedback.active");
  if (ERROR_FEEDBACK){
    // --------------------------- JOINT FEEDBACK ------------------------------
    int &JOINT_FEEDBACK = Utility::get_variable<int>(plugin_namespace+".active");
    if(JOINT_FEEDBACK){
      int &FEEDBACK_ACCEL = Utility::get_variable<int>(plugin_namespace+".accel");

      static boost::shared_ptr<JointPID> pid;
      if(!pid)
        pid = boost::shared_ptr<JointPID>( new JointPID());


      unsigned N = pid->joint_names.size();
      
      pid->q.set_zero(N);
      pid->qd.set_zero(N);
      pid->q_des.set_zero(N);
      pid->qd_des.set_zero(N);

      // Get data from pacer
      for(int i=0,ii=0;i<N;i++){
        Ravelin::VectorNd Qi = ctrl->get_joint_param(joint_names[ii],Pacer::Controller::position);
        Ravelin::VectorNd QDi = ctrl->get_joint_param(joint_names[ii],Pacer::Controller::velocity);
        Ravelin::VectorNd Qi_des = ctrl->get_joint_param(joint_names[ii],Pacer::Controller::position_goal);
        Ravelin::VectorNd QDi_des = ctrl->get_joint_param(joint_names[ii],Pacer::Controller::velocity_goal);
        for(int j=0;j<Qi.rows();j++,ii++){
          pid->q[ii] = Qi[j];
          pid->qd[ii] = QDi[j];
          pid->q_des[ii] = Qi_des[j];
          pid->qd_des[ii] = QDi_des[j];
        }
      }

      pid->update();

      // Set new torques in pacer
      for(int i=0,ii=0;i<N;i++){
        Ravelin::VectorNd Ui = Ravelin::VectorNd(ctrl->get_joint_param(&joint_names[ii][1],Pacer::Controller::torque));

        for(int j=0;j<Qi.rows();j++,ii++){
          Ui[j] += pid->value[ii];
        }
        
        ctrl->set_joint_param(&joint_names[ii][1],Pacer::Controller::torque,Ui);
      }
    }

/*
    // --------------------------- WORKSPACE FEEDBACK --------------------------
    static int &WORKSPACE_FEEDBACK = Utility::get_variable<int>("controller.error-feedback.operational-space.active");
    if(WORKSPACE_FEEDBACK){
      // CURRENTLY THIS IS ONLY FORCE
      // BUT IT CAN BE ACCELERATIONS TOO
      static int &FEEDBACK_ACCEL = Utility::get_variable<int>("controller.error-feedback.operational-space.accel");
      std::vector<Ravelin::Matrix3d> W(boost::assign::list_of(Ravelin::Matrix3d::identity())(Ravelin::Matrix3d::identity())(Ravelin::Matrix3d::identity())(Ravelin::Matrix3d::identity()).convert_to_container<std::vector<Ravelin::Matrix3d> >() );
      static std::vector<double>
          &Kp = Utility::get_variable<std::vector<double> >("controller.error-feedback.operational-space.gains.kp"),
          &Kv = Utility::get_variable<std::vector<double> >("controller.error-feedback.operational-space.gains.kv"),
          &Ki = Utility::get_variable<std::vector<double> >("controller.error-feedback.operational-space.gains.ki");

      Ravelin::VectorNd fb = Ravelin::VectorNd::zero(NUM_JOINT_DOFS);
      eef_stiffness_fb(Kp,Kv,Ki,x_des,xd_des,data->q,data->qd,fb);

      if(FEEDBACK_ACCEL)
        qdd_des += fb;
      else
        ufb += fb;
    }
  }
  
  */
}

/** This is a quick way to register your plugin function of the form:
  * void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t)
  */
#include "register_plugin"
