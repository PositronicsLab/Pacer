/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/controller.h>
#include <Pacer/utilities.h>
#include <sys/time.h>
#include <dlfcn.h>
#include <errno.h>
#include <boost/foreach.hpp>
#include <stdlib.h>     /* getenv */

#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <vector>

using namespace Pacer;

std::map<std::string,void*> Controller::handles;
  
Controller::Controller(): Robot(){
  
}

Controller::~Controller(){
  Utility::visualize.clear();
  close_all_plugins();
}


bool Controller::close_all_plugins(){
  typedef std::pair<std::string,void*> handle_pair;
  // close the loaded plugin libraries
  BOOST_FOREACH( handle_pair handle, handles){
    dlclose(handle.second);
//    delete handle.second; // DLCLOSE deletes handle object
  }

  handles.clear();
  _update_priority_map.clear();
  return true;
}


bool Controller::remove_plugin(const std::string& plugin_name){
  std::map<std::string, void*>::iterator it = handles.find(plugin_name);
  
  if(it == handles.end())
    return false;

  remove_plugin_update(plugin_name);
  void * &handle = (*it).second;
  dlclose(handle);
  //  delete handle; // DLCLOSE deletes handle object
  handles.erase(plugin_name);
  
  return true;
}

bool Controller::init_plugin(const std::string& plugin_name){
  OUT_LOG(logDEBUG) << ">> init_plugin("<< plugin_name << ")";
  std::string filename;
  bool plugin_filename_found = get_data<std::string>(plugin_name+".file",filename);
  if (!plugin_filename_found)
    throw std::runtime_error("Plugin "+plugin_name+" needs a filename!");
  
  if (!getenv("PACER_COMPONENT_PATH"))
    throw std::runtime_error("Environment variable PACER_PLUGIN_PATH not defined");
  
  std::string pPath(getenv("PACER_COMPONENT_PATH"));
  std::string lib_path = pPath+"/"+filename;
  OUT_LOG(logINFO) << "Loading Plugin: " << plugin_name;
  OUT_LOG(logINFO) << "\tLIB: " << filename.c_str();
  OUT_LOG(logINFO) << "\tPATH: " << pPath.c_str();
  // attempt to read the file
  //void* HANDLE = dlopen(lib_path.c_str(), RTLD_LAZY);
  void* HANDLE = dlopen(lib_path.c_str(), RTLD_NOW);
  if (!HANDLE)
  {
    std::cerr << "driver: failed to read plugin from " << filename << std::endl;
    std::cerr << "  " << dlerror() << std::endl;
    throw std::runtime_error("driver: failed to read plugin from " + filename);
    return false;
  }
  
  handles[plugin_name] = HANDLE;
  
  // attempt to load the initializer
  dlerror();
  Controller::init_t INIT = (init_t) dlsym(HANDLE, "init");
  const char* dlsym_error = dlerror();
  if (dlsym_error)
  {
    std::cerr << "driver warning: cannot load symbol 'init' from " << filename << std::endl;
    std::cerr << "        error follows: " << std::endl << dlsym_error << std::endl;
    throw std::runtime_error("driver: cannot load symbol 'init' from " + filename);
    return false;
  } else {
    // Init the plugin
    (*INIT)(this->ptr(),plugin_name.c_str());
  }
  OUT_LOG(logDEBUG) << "<< init_plugin("<< plugin_name << ")";

  return true;
}

bool Controller::init_all_plugins(){
  OUT_LOG(logINFO) << ">> Controller::init_plugins()";
  handles = std::map<std::string,void*>();
  bool RETURN_FLAG = true;
  std::vector<init_t> INIT;

  // call the initializers, if any
  std::vector<std::string> plugin_names = get_data<std::vector<std::string> >("plugins");

  // Load all the plugins
  for(unsigned i=0;i<plugin_names.size();i++){
    if(!init_plugin(plugin_names[i]))
      RETURN_FLAG = false;
  }
    
  OUT_LOG(logINFO) << "<< Controller::init_plugins()";
  return RETURN_FLAG;
}

void Controller::init(){
  // ================= INIT LOGGING ==========================
  FILELog::ReportingLevel() = FILELog::FromString("DEBUG1");
  // ================= LOAD VARS ==========================
  PARAMS_FILE = std::string("vars.xml");
  load_variables(PARAMS_FILE,"");
  // ================= SETUP LOGGING ==========================
  const std::string LOG_TYPE = get_data<std::string>("logging");
  OUT_LOG(logDEBUG1) << "Log Type : " << LOG_TYPE;
  FILELog::ReportingLevel() =
  FILELog::FromString( (!LOG_TYPE.empty() ) ? LOG_TYPE : "INFO");
  // ================= IMPORT CONTROLLED ROBOT =================
  std::string robot_model_file = get_data<std::string>("robot-model");
  
  read_robot_from_file(robot_model_file,get_abrobot());
  // ================= INIT ROBOT ==========================
  controller_phase = INITIALIZATION;

  init_robot();
}

// ============================================================================
// =========================== Begin Robot Controller =========================
// ============================================================================

void Controller::control(double t){
    OUT_LOG(logDEBUG) << ">> Controller::control(.)";
  // Import Robot Data
  static long long unsigned int iter = 0;
  static double last_time = -0.001;
  const double dt = t - last_time;
  
  OUTLOG(t,"virtual_time",logINFO);
  OUTLOG(dt,"virtual_time_step",logINFO);
  increment_phase(INITIALIZATION);
  update();
  increment_phase(PERCEPTION);
#ifdef USE_PLUGINS
  update_plugins(t);
#endif
  increment_phase(WAITING);
  reset_contact();
  last_time = t;

  iter++;
  OUT_LOG(logINFO) << "<< Controller::control(.)";
}

bool Controller::update_plugins(double t){
#ifdef USE_PLUGINS
  if(t == 0){
    if(!init_all_plugins())
      throw std::runtime_error("One of the plugins failed to load");
    return true;
  }
#endif
  OUT_LOG(logINFO) << ">> update_plugins()";
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
  
  // remove the plugins that have been marked for closure
  if(!plugins_to_close.empty()){
    BOOST_FOREACH( const std::string& name, plugins_to_close){
      OUT_LOG(logINFO) << "CLOSING " << name;
      remove_plugin(name);
    }
    plugins_to_close.clear();
  }
  
  // add the plugins that have been marked for addition
  if(!plugins_to_open.empty()){
    BOOST_FOREACH( const std::string& name, plugins_to_open){
      OUT_LOG(logINFO) << "OPENING " << name;
      init_plugin(name);
    }
    plugins_to_open.clear();
  }
  
#ifdef LOG_TO_FILE
  for(int i = HIGHEST_PRIORITY;i<=LOWEST_PRIORITY;i++){
    if(!_update_priority_map[i].empty()){
      OUT_LOG(logINFO) << "PRIORITY: " << i ;
      OUT_LOG(logINFO) << "\t" << get_map_keys(_update_priority_map[i]);
    }
  }
#endif
  
  // Update plugins in priority queue
  for(int i = HIGHEST_PRIORITY;i<=LOWEST_PRIORITY;i++){
    if(!_update_priority_map[i].empty()){ // SRZ: do I need this line?
      BOOST_FOREACH( const name_update_t::value_type& update, _update_priority_map[i]){
        OUT_LOG(logINFO) << ">> " << update.first;
        (*(update.second))(this->ptr(),t);
        OUT_LOG(logINFO) << "<< " << update.first;
      }
    }
  }
  
  OUT_LOG(logINFO) << "<< update_plugins()";
  return true;
}

// ===========================  END CONTROLLER  ===============================
// ============================================================================
