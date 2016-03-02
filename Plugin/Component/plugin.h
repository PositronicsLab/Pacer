/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
/** This is a quick way to register your plugin function of the form:
 * void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t)
 * void Deconstruct(const boost::shared_ptr<Pacer::Controller>& ctrl)
 */

#ifndef _PLUGIN_H
#define _PLUGIN_H

#include <Pacer/controller.h>

std::string plugin_namespace;
boost::weak_ptr<Pacer::Controller> ctrl_weak_ptr;
double t;


// Implemented by specific plugin
static void loop();

void update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t){
  ::t = t;
  static int ITER = -1;
  int RTF = ctrl->get_data<int>(plugin_namespace+".real-time-factor");
  if(++ITER % RTF == 0){
//#ifndef NDEBUG
//    try {
//#endif
      std::cout << plugin_namespace << " is at iteration (" << ITER
      << ") with RTF (" << RTF << ") at time "<< t << std::endl;
      loop();
//#ifndef NDEBUG
//    }
//    catch (std::exception& e) {
//      throw std::runtime_error(plugin_namespace+" failed with error: " + e.what());
//    }
//    catch (...){
//      throw std::runtime_error(plugin_namespace+" failed with unkown error.");
//    }
//#endif
  }
}

static void setup();

std::vector<std::string> variable_names;
void deconstruct(const boost::shared_ptr<Pacer::Controller>& ctrl, double t){
#ifdef LOG_TO_FILE
  OUT_LOG(logINFO) << "Deconstructing plugin: " << plugin_namespace;
#endif
  for(int i=0;i<variable_names.size();i++){
    ctrl->remove_data(variable_names[i]);
#ifdef LOG_TO_FILE
    OUT_LOG(logINFO) << "\t-removing variable: " << variable_names[i];
#endif
  }
}

extern "C" {
  void init(const boost::shared_ptr<Pacer::Controller> ctrl, const char* name){
    ctrl_weak_ptr = boost::weak_ptr<Pacer::Controller>(ctrl);
    plugin_namespace = std::string(std::string(name));
    setup();
    
    int priority = ctrl->get_data<double>(plugin_namespace+".priority");
    
    ctrl->add_plugin_update(priority,name,&update);
    ctrl->add_plugin_deconstructor(name,&deconstruct);
  }
}

#endif // end _PLUGIN_H
