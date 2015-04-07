/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#ifndef CONTROL_H
#define CONTROL_H

#include <Pacer/robot.h>
#include <boost/shared_ptr.hpp>
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
    ~Controller();
    void init(){
      PARAMS_FILE = std::string("vars.xml");
      load_variables(PARAMS_FILE,boost::dynamic_pointer_cast<Robot>(ptr()));
      unlock_state();
      init_robot();
      // After Robot loads, load plugins
#ifdef USE_PLUGINS
      if(!init_plugins())
        throw std::runtime_error("One of the plugins failed to load");
#endif
      unlock_state();
    }

    // call Pacer at time t
    void control(double t);

    bool reload_plugins(){
      close_plugins();
      return init_plugins();
    }

    void add_plugin_update(int priority,std::string name,update_t f){
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
      OUT_LOG(logDEBUG1) << ">> update_plugins()";
      //name_update_t& non_realtime_map = _update_priority_map[NON_REALTIME];
      //if(!_update_priority_map[i].empty())
      //BOOST_FOREACH( const name_update_t::value_type& update, non_realtime_map)
      //{  
        //static std::map<std::string,std::thread> threads = std::map<std::string,std::thread>();
        //if(threads.find(update.first) != threads.end());
        //OUT_LOG(logDEBUG1) << ">> " << update.first;
        //threads[update.first] = (*(update.second)),this->ptr(),t);
        //OUT_LOG(logDEBUG1) << "<< " << update.first;
      //}
          
      for(int i = HIGHEST_PRIORITY;i<=LOWEST_PRIORITY;i++)
        if(!_update_priority_map[i].empty()) // SRZ: do I need this line?
          BOOST_FOREACH( const name_update_t::value_type& update, _update_priority_map[i])
          {  
            OUT_LOG(logDEBUG1) << ">> " << update.first;
            (*(update.second))(this->ptr(),t);
            OUT_LOG(logDEBUG1) << "<< " << update.first;
          }
                
      OUT_LOG(logDEBUG1) << "<< update_plugins()";
    }
};
}
#endif // CONTROL_H
