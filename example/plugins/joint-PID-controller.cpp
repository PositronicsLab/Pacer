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
  JointPID(std::string n) { init(); }

  Ravelin::VectorNd q_des,
                    qd_des,
                    q,
                    qd;

  std::vector<std::string> joint_names;

  void init(){
    std::vector<std::string>
        &joint_names = CVarUtils::GetCVarRef<std::vector<std::string> >(plugin_namespace+".id");

    OUTLOG(joint_names,"joint_names",logERROR);

    std::vector<double>
        &Kp = CVarUtils::GetCVarRef<std::vector<double> >(plugin_namespace+".gains.kp"),
        &Kv = CVarUtils::GetCVarRef<std::vector<double> >(plugin_namespace+".gains.kv"),
        &Ki = CVarUtils::GetCVarRef<std::vector<double> >(plugin_namespace+".gains.ki");

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
  int &ERROR_FEEDBACK = CVarUtils::GetCVarRef<int>("controller.error-feedback.active");
  if (ERROR_FEEDBACK){
    // --------------------------- JOINT FEEDBACK ------------------------------
    int &JOINT_FEEDBACK = CVarUtils::GetCVarRef<int>(plugin_namespace+".active");
    if(JOINT_FEEDBACK){
      int &FEEDBACK_ACCEL = CVarUtils::GetCVarRef<int>(plugin_namespace+".accel");

      static boost::shared_ptr<JointPID> pid;
      if(!pid){
         if(FEEDBACK_ACCEL)
           pid = boost::shared_ptr<JointPID>( new JointPID(std::string(plugin_namespace+".accel")));
         else
           pid = boost::shared_ptr<JointPID>( new JointPID(std::string(plugin_namespace+".force")));
      }

      pid->q = Ravelin::VectorNd(q;
      pid->qd = Ravelin::VectorNd(qd;
      pid->q_des = Ravelin::VectorNd(q_des;
      pid->qd_des = Ravelin::VectorNd(qd_des;
      pid->joint_names = ;
      pid->update();

    }

/*
    // --------------------------- WORKSPACE FEEDBACK --------------------------
    static int &WORKSPACE_FEEDBACK = CVarUtils::GetCVarRef<int>("controller.error-feedback.operational-space.active");
    if(WORKSPACE_FEEDBACK){
      // CURRENTLY THIS IS ONLY FORCE
      // BUT IT CAN BE ACCELERATIONS TOO
      static int &FEEDBACK_ACCEL = CVarUtils::GetCVarRef<int>("controller.error-feedback.operational-space.accel");
      std::vector<Ravelin::Matrix3d> W(boost::assign::list_of(Ravelin::Matrix3d::identity())(Ravelin::Matrix3d::identity())(Ravelin::Matrix3d::identity())(Ravelin::Matrix3d::identity()).convert_to_container<std::vector<Ravelin::Matrix3d> >() );
      static std::vector<double>
          &Kp = CVarUtils::GetCVarRef<std::vector<double> >("controller.error-feedback.operational-space.gains.kp"),
          &Kv = CVarUtils::GetCVarRef<std::vector<double> >("controller.error-feedback.operational-space.gains.kv"),
          &Ki = CVarUtils::GetCVarRef<std::vector<double> >("controller.error-feedback.operational-space.gains.ki");

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
