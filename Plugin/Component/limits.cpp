/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/controller.h>
#include <Pacer/utilities.h>

#include "plugin.h"

void loop(){
boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);

  std::map<std::string, Ravelin::VectorNd > q, qd, u;
  ctrl->get_joint_value(Pacer::Robot::position_goal, q);
  ctrl->get_joint_value(Pacer::Robot::velocity_goal, qd);
  ctrl->get_joint_value(Pacer::Robot::load_goal, u);
  
#ifndef NDEBUG
  OUT_LOG(logINFO) << "Before Limits: ";
  std::vector<std::string> keys1 = ctrl->get_map_keys(u);
  for(int i=0;i<keys1.size();i++){
    OUT_LOG(logINFO) << keys1[i] << " : " << u[keys1[i]];
  }
#endif
  
  // Enforce joint and motor limits
  std::vector<std::string> joint_names = ctrl->get_data<std::vector<std::string> >("init.joint.id");
  std::vector<double> joint_dofs = ctrl->get_data<std::vector<double> >("init.joint.dofs");
  std::vector<double> torque_limit;
  bool apply_torque_limit = ctrl->get_data<std::vector<double> >("init.joint.limits.u",torque_limit);
  
  std::vector<double> velocity_limit;
  bool apply_velocity_limit = ctrl->get_data<std::vector<double> >("init.joint.limits.qd",velocity_limit);
  
  for (int i=0, ii=0; i<joint_names.size(); i++) {
    for (int j=0; j<joint_dofs[i]; j++,ii++) {
      // If motor speed limit met, cancel torque
      // (if applied in direction of limit)
      if(apply_velocity_limit){
        if (qd[joint_names[i]][j] > velocity_limit[ii]) {
          OUT_LOG(logDEBUG) << joint_names[i] << ": qd["<<j<<"]= " << qd[joint_names[i]][j] << " exceeds velocity limit: " << velocity_limit[ii];
          if(u[joint_names[i]][j] > 0){
            OUT_LOG(logDEBUG) << joint_names[i] << ": u["<<j<<"]= " << u[joint_names[i]][j] << " is moving towards exceeded velocity limit, setting to 0";
            u[joint_names[i]][j] = 0;
          }
        } else if  (qd[joint_names[i]][j] < -velocity_limit[ii]) {
          OUT_LOG(logDEBUG) << joint_names[i] << ": qd["<<j<<"]= " << qd[joint_names[i]][j] << " exceeds negative velocity limit: " << -velocity_limit[ii];
          if(u[joint_names[i]][j] < 0){
            OUT_LOG(logDEBUG) << joint_names[i] << ": u["<<j<<"]= " << u[joint_names[i]][j] << " is moving towards exceeded velocity limit, setting to 0";
            u[joint_names[i]][j] = 0;
          }
        }
      }
      
      if(apply_torque_limit){
        // Limit torque
        if (u[joint_names[i]][j] > torque_limit[ii]) {
          OUT_LOG(logDEBUG) << joint_names[i] << ": u["<<j<<"]= " << u[joint_names[i]][j] << " exceeds torque limit: " << torque_limit[ii] << ", setting to " << torque_limit[ii];
          u[joint_names[i]][j] = torque_limit[ii];
        } else if  (u[joint_names[i]][j] < -torque_limit[ii]) {
          OUT_LOG(logDEBUG) << joint_names[i] << ": u["<<j<<"]= " << u[joint_names[i]][j] << " exceeds torque limit: " << -torque_limit[ii] << ", setting to " << -torque_limit[ii];
          u[joint_names[i]][j] = -torque_limit[ii];
        }
      }
    }
  }
  
#ifndef NDEBUG
  OUT_LOG(logINFO) << "After Limits: ";
  std::vector<std::string> keys = ctrl->get_map_keys(u);
  for(int i=0;i<keys.size();i++){
    OUT_LOG(logINFO) << keys[i] << " : " << u[keys[i]];
  }
#endif
  ctrl->set_joint_value(Pacer::Robot::load_goal, u);

}
void setup(){
}