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

  double switch_progress = ctrl->get_data<double>(plugin_namespace+".progress");
  std::string switch_gait = ctrl->get_data<std::string>(plugin_namespace+".gait");
  std::string gait_planner_name = ctrl->get_data<std::string>(plugin_namespace+".planner");
  
  double gait_progress = ctrl->get_data<double>("gait-proportion");

  if (gait_progress >= switch_progress) {
    // right now only imports double vector params
    //    std::vector<std::string> gait_features = ctrl->get_data<std::string>(plugin_namespace+"."+switch_gait+".id");
    std::vector<double> gait, duty_factor, speed;
    gait = ctrl->get_data<std::vector<double> >(plugin_namespace+"."+switch_gait+".gait");
    duty_factor = ctrl->get_data<std::vector<double> >(plugin_namespace+"."+switch_gait+".duty-factor");
    speed = ctrl->get_data<std::vector<double> >(plugin_namespace+"."+switch_gait+".speed");
    // set gait parameter in gait-planner
    ctrl->set_data<std::vector<double> >(gait_planner_name+".gait",gait);
    ctrl->set_data<std::vector<double> >(gait_planner_name+".duty-factor",duty_factor);
    ctrl->set_data<double>("waypoints.max-forward-speed",speed[0]);
    ctrl->set_data<double>("waypoints.max-strafe-speed",speed[1]);
    ctrl->set_data<double>("waypoints.max-turn-speed",speed[2]);
  }
}

void setup(){}
