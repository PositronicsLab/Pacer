/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#ifndef MODULE_H
#define MODULE_H

#include <Pacer/robot.h>
namespace Pacer{

/**
 * @brief module class is the basic structure used to encapsulate controllers.
 */

enum ModuleType{
  CONTROLLER = 0,
  PLANNER,
  SENSOR
};

template <class val_type>
class Module
{
public:
  Module(){}
  ModuleType type;
  val_type value;

  /**
   *  @brief init function, this is called by the constructor.
   */
   virtual void init() = 0;

  /**
   *  @brief update function, this is called by the controller loop.
   */
   virtual void update() = 0;
};


#include <CVars/CVar.h>
struct Gains
{
  double perr_sum;
  double kp;
  double kv;
  double ki;
};

class ControllerModule : public Module<Ravelin::VectorNd>
                       , public boost::enable_shared_from_this<ControllerModule> {
public:
  boost::shared_ptr<ControllerModule> ptr() {return shared_from_this();}
};

class IMUModule : public Module<Ravelin::SVector6d>
                , public boost::enable_shared_from_this<IMUModule>{
public:
  boost::shared_ptr<IMUModule> ptr() {return shared_from_this();}
};



class JointPID : public ControllerModule {
public:
  JointPID(){}
  JointPID(std::string n) : name(n){ init(); }

  std::string name;

  Ravelin::VectorNd q_des,
                    qd_des,
                    q,
                    qd;

  std::vector<std::string> joint_names;

  void init(){
    type = CONTROLLER;
    std::vector<std::string>
        &joint_names = CVarUtils::GetCVarRef<std::vector<std::string> >("controller.error-feedback.configuration-space.id");

    OUTLOG(joint_names,"joint_names",logERROR);

    std::vector<double>
        &Kp = CVarUtils::GetCVarRef<std::vector<double> >(name+".gains.kp"),
        &Kv = CVarUtils::GetCVarRef<std::vector<double> >(name+".gains.kv"),
        &Ki = CVarUtils::GetCVarRef<std::vector<double> >(name+".gains.ki");

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
      const double KP = gains[joint_name].kp;
      const double KV = gains[joint_name].kv;
      const double KI = gains[joint_name].ki;

      // add feedback torque to joints
      double perr = q_des[i] - q[i];
      gains[joint_name].perr_sum += perr;
      double ierr = gains[joint_name].perr_sum;
      double derr = qd_des[i] - qd[i];

      value[i] = perr*KP + derr*KV + ierr*KI;
    }
    OUTLOG(value,"ufb",logERROR);
    OUTLOG(joint_names,"joint_names",logERROR);

  }
private:
  struct Gains
  {
    double perr_sum;
    double kp;
    double kv;
    double ki;
  };
  std::map<std::string,Gains> gains;

};

class eefPID : public ControllerModule {
public:
  eefPID(){}
  eefPID(std::string n) : name(n){ init(); }

  std::string name;

  std::vector<const EndEffector>
                    eefs;

  std::vector<Ravelin::Vector3d>
                    x_des,
                    xd_des,
                    x,
                    xd;

  std::vector<std::string> eef_names;

  void init(){
    std::vector<std::string>
        &eef_names = CVarUtils::GetCVarRef<std::vector<std::string> >("init.end-effector.id");

    std::vector<double>
        &Kp = CVarUtils::GetCVarRef<std::vector<double> >("controller."+name+".gains.kp"),
        &Kv = CVarUtils::GetCVarRef<std::vector<double> >("controller."+name+".gains.kp"),
        &Ki = CVarUtils::GetCVarRef<std::vector<double> >("controller."+name+".gains.kp");

    for(int i=0;i<eef_names.size();i++){
        gains[eef_names[i]].kp = Kp[i];
        gains[eef_names[i]].kv = Kv[i];
        gains[eef_names[i]].ki = Ki[i];
    }
  }

  void update(){
    // get the two gains
    for (unsigned i=0; i< eefs.size(); i++)
    {
      Ravelin::Vector3d err;
      const std::string& eef_name = eefs[i].id;
      for (unsigned j=0; j< 3; j++)
      {
        const double KP = gains[eef_name+std::to_string(j)].kp;
        const double KV = gains[eef_name+std::to_string(j)].kv;
        const double KI = gains[eef_name+std::to_string(j)].ki;

        // add feedback torque to joints
        double perr = x_des[i][j] - x[i][j];
        gains[eef_name+std::to_string(j)].perr_sum += perr;
        double ierr = gains[eef_name].perr_sum;
        double derr = xd_des[i][j] - xd[i][j];

        err[j] = perr*KP + derr*KV + ierr*KI;
      }

    }
  }
private:
  std::map<std::string,Gains> gains;

};
}


#endif // MODULE_H
