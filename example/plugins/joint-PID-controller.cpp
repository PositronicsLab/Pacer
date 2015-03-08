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
  PIDController();
private:

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
  JointPID(std::string n) : PIDController(n){ init(); }


  Ravelin::VectorNd q_des,
                    qd_des,
                    q,
                    qd;

  std::vector<std::string> joint_names;

  void init(){
    type = CONTROLLER;
    std::vector<std::string>
        &joint_names = CVarUtils::GetCVarRef<std::vector<std::string> >(plugin_namespace+".id");

    OUTLOG(joint_names,"joint_names",logERROR);

    std::vector<double>
        &Kp = CVarUtils::GetCVarRef<std::vector<double> >(plugin_namespace+".gains.kp"),
        &Kv = CVarUtils::GetCVarRef<std::vector<double> >(plugin_namespace+".gains.kv"),
        &Ki = CVarUtils::GetCVarRef<std::vector<double> >(plugin_namespace+".gains.ki");

    for(int i=0;i<joint_names.size();i++){
      gains[joint_names[i]].kp = Kp[i];
      gains[joint_names[i]].kv = Kv[i];
      gains[joint_names[i]].ki = Ki[i];
      gains[joint_names[i]].perr_sum = 0;
    }

    OUTLOG(Kp,"Kp",logERROR);
    OUTLOG(Kv,"Kv",logERROR);
    OUTLOG(Ki,"Ki",logERROR);
    OUT_LOG(logERROR) << "Controller: " << name << " inited!";
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
}

void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t){
  
}

/** This is a quick way to register your plugin function of the form:
  * void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t)
  */
#include "register_plugin"
