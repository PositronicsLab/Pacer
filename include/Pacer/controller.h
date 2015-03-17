/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#ifndef CONTROL_H
#define CONTROL_H

#include <Pacer/robot.h>
#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>

namespace Pacer{

class Controller;

typedef void (*update_t)(const boost::shared_ptr<Controller>&, double);

enum niceness{
  HIGHEST_PRIORITY = -20,
  LOWEST_PRIORITY = 19
};

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
    ~Controller();

    // call Pacer at time t
    void control(double t);

    bool reload_plugins(){
      close_plugins();
      return init_plugins();
    }

    void add_plugin_update(int priority,std::string& name,update_t f){
      // Fix priority
      if(priority > 19 || priority < -20){
        OUT_LOG(logERROR) << "Set priorities to \"niceness\" range [-20..19]";
        if(priority < -20)
          priority = -20;
        else if(priority > 19)
          priority = 19;
      }

      // Check if this function already has an updater
      if(_name_priority_map.find(name) != _name_priority_map.end())
        remove_plugin_update(name);

      // add plugin back in at new priority
      _update_priority_map[priority][name] = f;
      _name_priority_map[name] = priority;
    }
    
    void remove_plugin_update(std::string& name){
      //delete _update_priority_map.at(_name_priority_map.at(name)).at(name)->second;
      _update_priority_map[_name_priority_map[name]].erase(name);
      _name_priority_map.erase(name);
    }
      
  private:
    typedef std::map<std::string , update_t> name_update_t;
    std::map<int , name_update_t> _update_priority_map;
    std::map< std::string , int > _name_priority_map;
    
    bool close_plugins();
    bool init_plugins();
    
    bool update_plugins(double t){
      for(int i = HIGHEST_PRIORITY;i<=LOWEST_PRIORITY;i++)
        if(!_update_priority_map[i].empty()) // SRZ: do I need this line?
          BOOST_FOREACH( const name_update_t::value_type& update, _update_priority_map[i])
          {  
            (*(update.second))(this->ptr(),t);
          }
    }
};
}
#endif // CONTROL_H
