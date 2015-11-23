/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/controller.h>
#include <Pacer/utilities.h>

std::string plugin_namespace;

void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t){
  // Enforce limits
  // Note: homogeneous limits
  static std::vector<double> load_max = get_data<std::vector<double> >("init.joint.limits.u");
  
  Ravelin::VectorNd u = ctrl->get_joint_generalized_value(Pacer::Controller::load_goal);
  OUTLOG(u,"U_NO_LIMIT",logINFO);
  
  for (int i=0;i<u.rows(); i++) {
    if (!std::isfinite(u[i])) {
      throw std::runtime_error("Joint command is not finite!");
    }
    
    // Limit clamp values
    if(u[i] > load_max[0]) u[i] = load_max[0];
      else if(u[i] < -load_max[0]) u[i] = -load_max[0];
        }
  OUTLOG(u,"U_WITH_LIMIT",logINFO);
  ctrl->set_joint_generalized_value(Pacer::Controller::load_goal,u);
}

/** This is a quick way to register your plugin function of the form:
 * void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t)
 */
#include "register-plugin"
