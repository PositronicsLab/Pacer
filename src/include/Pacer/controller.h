/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#ifndef CONTROL_H
#define CONTROL_H

#include <Pacer/robot.h>

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
    void load_variables(std::string xml_file, std::string root);

  private:
    std::string PARAMS_FILE;
    void read_robot_from_file(std::string robot_model_file,boost::shared_ptr<Ravelin::ArticulatedBodyd>& abrobot);

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
    
    //////////////////////////////////////////////////////////////////////////
    //////////////////////  PHASE CHECKING ///////////////////////////////////
  public:
    enum ControllerPhase{
      INITIALIZATION = 0,
      PERCEPTION     = 1,
      PLANNING       = 2,
      CONTROL        = 3,
      WAITING        = 4,
      INCREMENT      = 5
    };

    ControllerPhase get_current_phase(){ return controller_phase; }
    std::string get_current_phase_name(){ return std::string(enum_string(controller_phase)); }
    
  private:
    ControllerPhase controller_phase;
    const char * enum_string(const ControllerPhase& e){
      int i = static_cast<int>(e);
      return (const char *[]) {
        "INITIALIZATION",
        "PERCEPTION",
        "PLANNING",
        "CONTROL",
        "WAITING",
        "INCREMENT"
      }[i];
    }
    
    // Enforce that values are only being assigned during the correct phase
    bool check_phase(const unit_e& u, bool internal_call){
      OUT_LOG(logDEBUG) << "-- SCHEDULER -- " << "check unit: " << unit_enum_string(u) << " against phase: " << enum_string(controller_phase);
      switch (u) {
        case initialization:
          if (controller_phase != INITIALIZATION && controller_phase != WAITING)
          {
            if(internal_call)
            {
              throw std::runtime_error("controller must be in INITIALIZATION phase to set Pacer internal parameters.");
            }
            return false;
          }
          return true;
        case misc_sensor:
        case position:
        case velocity:
        case acceleration:
        case load:
          if (controller_phase != PERCEPTION && controller_phase != INITIALIZATION  && controller_phase != WAITING )
          {
            if(internal_call)
            {
              throw std::runtime_error("controller must be in PERCEPTION, INITIALIZATION or WAITING phase to set state information: {misc_sensor,position,velocity,acceleration,load}");
            }
            return false;
          }
          return true;
        case misc_planner:
        case position_goal:
        case velocity_goal:
        case acceleration_goal:
          if (controller_phase != PLANNING && controller_phase != INITIALIZATION && controller_phase != WAITING)
          {
            // NOTE: if this is the first PLANNING call and we were in PERCEPTION phase, increment to PLANNING phase
            if(internal_call)
            {
              if(controller_phase == PERCEPTION)
              {
                increment_phase(PLANNING);
                return true;
              }
              throw std::runtime_error("controller must be in PLANNING phase to set goal state information: {misc_planner,position_goal,velocity_goal,acceleration_goal}");
            }
            return false;
          }
          return true;
        case misc_controller:
        case load_goal:
          if (controller_phase != CONTROL && controller_phase != INITIALIZATION && controller_phase != WAITING){
            // NOTE: if this is the first CONTROL call and we were in PLANNING phase, increment to CONTROL phase
            if(internal_call)
            {
              if(controller_phase == PLANNING)
              {
                increment_phase(CONTROL);
                return true;
              }
              
              if(controller_phase == PERCEPTION)
              {
                increment_phase(CONTROL);
                return true;
              }
              throw std::runtime_error("controller must be in CONTROL phase to set commands: {misc_controller,load_goal}");
            }
            return false;
          }
          return true;
        case clean_up:
          if (controller_phase != WAITING && controller_phase != INITIALIZATION )
          {
            if(internal_call)
            {
              throw std::runtime_error("controller must be in WAITING or INITIALIZATION phase to perform clean_up duties.");
            }
            return false;
          }
          return true;
        default:
          throw std::runtime_error("unknown unit being set in state data");
          return false;
      }
    }
    
    void reset_phase(){
      controller_phase = PERCEPTION;
      OUT_LOG(logINFO) << "-- SCHEDULER -- " << "Controller Phase reset: ==> PLANNING";
    }
    
    void increment_phase(ControllerPhase phase){
      if (phase == INCREMENT) {
        switch (controller_phase) {
          case INITIALIZATION:
            controller_phase = PERCEPTION;
            OUT_LOG(logINFO) << "-- SCHEDULER -- " << "Controller Phase change: *INITIALIZATION* ==> PERCEPTION";
            break;
          case WAITING:
            throw std::runtime_error("Cannot increment waiting controller. call reset_phase()");
            break;
          case PERCEPTION:
            controller_phase = PLANNING;
            OUT_LOG(logINFO) << "-- SCHEDULER -- " << "Controller Phase change: PERCEPTION ==> PLANNING";
            break;
          case PLANNING:
            controller_phase = CONTROL;
            OUT_LOG(logINFO) << "-- SCHEDULER -- " << "Controller Phase change: PLANNING ==> CONTROL";
            break;
          case CONTROL:
            controller_phase = WAITING;
            OUT_LOG(logINFO) << "-- SCHEDULER -- " << "Controller Phase change: CONTROL ==> *WAITING*";
            break;
          default:
            throw std::runtime_error("controller state dropped off list of valid states");
            break;
        }
      } else {
        OUT_LOG(logINFO) << "-- SCHEDULER -- " << "Controller Phase change: " << enum_string(controller_phase) << " ==> " << enum_string(phase);
        controller_phase =phase;
      }
    }
    
    bool check_phase_internal(const unit_e& u){return check_phase( u, true);}

  public:
    bool check_phase(const unit_e& u){ return check_phase( u, false); }

  };
}
#endif // CONTROL_H
