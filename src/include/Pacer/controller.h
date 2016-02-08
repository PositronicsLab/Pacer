/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#ifndef CONTROL_H
#define CONTROL_H

#include <Pacer/robot.h>
#include <Moby/XMLTree.h>
#include <Moby/XMLReader.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>

namespace Pacer{
  
  class Controller;
  
  typedef void (*update_t)(const boost::shared_ptr<Controller>&, double);
  
  const int
  NON_REALTIME = -1,
  HIGHEST_PRIORITY = 0,
  LOWEST_PRIORITY = 10;
  
  class Controller : public Robot, public boost::enable_shared_from_this<Controller>
  {
  public:
    boost::shared_ptr<Controller> ptr()
    {
      return shared_from_this();
    }
    /**
     * @brief Controller constructor
     * @see Robot()
     */
    Controller();
    virtual ~Controller();
    void init();
    
  private:
    std::string PARAMS_FILE;
    void load_variables(std::string xml_file, std::string root);
    void process_tag(std::string tag,boost::shared_ptr<const Moby::XMLTree> node);
    
    // call Pacer at time t
  public:
    void control(double t);
    
    void add_plugin_update(int priority,const std::string& name,update_t f){
      // Fix priority
      if(priority > LOWEST_PRIORITY || priority < NON_REALTIME){
        OUT_LOG(logERROR) << "Set priorities to range [-1,0.."<<LOWEST_PRIORITY<<"], -1 is for non-realtime processes (will only return data when complete)";
        if(priority < NON_REALTIME)
          priority = NON_REALTIME;
        else if(priority > LOWEST_PRIORITY)
          priority = LOWEST_PRIORITY;
      }
      
      // Check if this function already has an updater
      if(_name_priority_map.find(name) != _name_priority_map.end())
        remove_plugin_update(name);
      
      // add plugin back in at new priority
      _update_priority_map[priority][name] = f;
      _name_priority_map[name] = priority;
    }
    
    void add_plugin_deconstructor(const std::string& name,update_t f){
      _plugin_deconstruct_map[name] = f;
    }
    
    
    void close_plugin(const std::string& name){
      OUT_LOG(logINFO) << "marked plugin '" << name << "' for closure.";
      plugins_to_close.push_back(name);
    }
    void open_plugin(const std::string& name){
      OUT_LOG(logINFO) << "marked plugin '" << name << "' for open.";
      plugins_to_open.push_back(name);
    }
    
  private:
    typedef void (*init_t)(const boost::shared_ptr<Controller>, const char*);
    static std::map<std::string, void*> handles;
    typedef std::map<std::string , update_t> name_update_t;
    
    std::map<int , name_update_t> _update_priority_map;
    name_update_t _plugin_deconstruct_map;
    std::map< std::string , int > _name_priority_map;
    std::vector<std::string> plugins_to_open, plugins_to_close;
    
    bool reload_plugin(const std::string& name){
      remove_plugin(name);
      return init_plugin(name);
    }
    
    bool init_plugin(const std::string& name);
    bool remove_plugin(const std::string& plugin_name);
    void remove_plugin_update(const std::string& name){
      (*_plugin_deconstruct_map[name])(this->ptr(),0);
      
      //delete _update_priority_map.at(_name_priority_map.at(name)).at(name)->second;
      _update_priority_map[_name_priority_map[name]].erase(name);
      _name_priority_map.erase(name);
      _plugin_deconstruct_map.erase(name);
      
    }
    
    bool close_all_plugins();
    bool init_all_plugins();
    
    bool update_plugins(double t);
  };
}
#endif // CONTROL_H
