/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/controller.h>
#include <Pacer/utilities.h>
#include "plugin.h"
using namespace Ravelin;

void loop(){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  
  std::vector<double> new_command(3);
  ctrl->get_data<std::vector<double> >(plugin_namespace+".command",new_command);
    
    Origin3d command = Origin3d(new_command[0],new_command[1],new_command[2]);
    ctrl->set_data<Origin3d>("SE2_command",command);
}

void setup(){
  variable_names.push_back("SE2_command");
}

