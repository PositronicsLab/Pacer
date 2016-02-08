#include <Pacer/controller.h>
#include "plugin.h"

void loop(){
boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  std::vector<std::string> events
  = ctrl->get_data<std::vector<std::string> >(plugin_namespace+".events");
  
  static double start_time = -1;
  
  if(start_time<0){
    int relative_start_time = 0;
    ctrl->get_data<int>(plugin_namespace+".relative-start-time",relative_start_time);
    if(relative_start_time)
      start_time = 0;
    else
      start_time = t;
  }
  
  static double last_time = -1;
  
  BOOST_FOREACH(const std::string& event_name, events){
    double event_time = ctrl->get_data<double>(plugin_namespace+"."+event_name+".time");
    
    // Just stepped over the event
    if(!(last_time-start_time < event_time && t-start_time >= event_time))
      continue;
    
    // Start
    std::vector<std::string> plugins_to_open;
    if(ctrl->get_data<std::vector<std::string> >(plugin_namespace+"."+event_name + ".open",plugins_to_open)){
      BOOST_FOREACH(const std::string& plugin_name, plugins_to_open){
        ctrl->open_plugin(plugin_name);
        OUT_LOG(logDEBUG) << "Plugin: " << plugin_name << " opened from scheduler!";
      }
    }
    
    std::vector<std::string> plugins_to_close;
    if(ctrl->get_data<std::vector<std::string> >(plugin_namespace+"."+event_name + ".close",plugins_to_close)){
      BOOST_FOREACH(const std::string& plugin_name, plugins_to_close){
        ctrl->close_plugin(plugin_name);
        OUT_LOG(logDEBUG) << "Plugin: " << plugin_name << " closed from scheduler!";
      }
    }
  }
  last_time = t;
}
void setup(){
}