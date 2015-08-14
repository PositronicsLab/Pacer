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
  
Controller::Controller(): Robot(){

}

Controller::~Controller(){
  close_all_plugins();
}


std::map<std::string,void*> handles;
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

typedef void (*init_t)(const boost::shared_ptr<Controller>, const char*);
std::map<std::string, init_t> INIT;

bool Controller::remove_plugin(const std::string& plugin_name){
  std::map<std::string, void*>::iterator it = handles.find(plugin_name);
  
  if(it == handles.end())
    return false;

  INIT.erase(plugin_name);
  remove_plugin_update(plugin_name);
  void * &handle = (*it).second;
  dlclose(handle);
  //  delete handle; // DLCLOSE deletes handle object
  handles.erase(plugin_name);
  
  return true;
}

bool Controller::init_plugin(const std::string& plugin_name){
  std::string filename;
  bool plugin_filename_found = get_data<std::string>(plugin_name+".file",filename);
  if (!plugin_filename_found)
    throw std::runtime_error("Plugin "+plugin_name+" needs a filename!");
  
  if (!getenv("PACER_PLUGIN_PATH"))
    throw std::runtime_error("Environment variable PACER_PLUGIN_PATH not defined");
  
  std::string pPath(getenv("PACER_PLUGIN_PATH"));
  std::string lib_path = pPath+"/"+filename;
  OUT_LOG(logINFO) << "Loading Plugin: " << plugin_name;
  OUT_LOG(logINFO) << "\tLIB: " << filename.c_str();
  OUT_LOG(logINFO) << "\tPATH: " << pPath.c_str();
  // attempt to read the file
  void* HANDLE = dlopen(lib_path.c_str(), RTLD_LAZY);
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
  INIT[plugin_name] = (init_t) dlsym(HANDLE, "init");
  const char* dlsym_error = dlerror();
  if (dlsym_error)
  {
    std::cerr << "driver warning: cannot load symbol 'init' from " << filename << std::endl;
    std::cerr << "        error follows: " << std::endl << dlsym_error << std::endl;
    INIT.erase(plugin_name);
    throw std::runtime_error("driver: cannot load symbol 'init' from " + filename);
    return false;
  } else {
    // Init the plugin
    (*INIT[plugin_name])(this->ptr(),plugin_name.c_str());
  }
  return true;
}

bool Controller::init_all_plugins(){
  OUT_LOG(logDEBUG) << ">> Controller::init_plugins()";

  bool RETURN_FLAG = true;
  close_all_plugins();
  std::vector<init_t> INIT;

  // call the initializers, if any
  std::vector<std::string> plugin_names = get_data<std::vector<std::string> >("plugins");

  // Load all the plugins
  for(unsigned i=0;i<plugin_names.size();i++){
    if(!init_plugin(plugin_names[i]))
      RETURN_FLAG = false;
  }
    
  OUT_LOG(logDEBUG) << "<< Controller::init_plugins()";
  return RETURN_FLAG;
}

// ============================================================================
// =========================== Begin Robot Controller =========================
// ============================================================================

void Controller::control(double t){
    OUT_LOG(logDEBUG) << ">> Controller::control(.)";
  // Import Robot Data
  static double last_time = -0.001;
  const double dt = t - last_time;
  
  OUTLOG(t,"virtual_time",logERROR);
  OUTLOG(dt,"virtual_time_step",logERROR);
  
  update();
  lock_state();
#ifdef USE_PLUGINS
  update_plugins(t);
#endif
  unlock_state();
 
  {
    // Enforce limits
    // Note: homogeneous limits
    static std::vector<double> load_max = get_data<std::vector<double> >("init.joint.limits.u");

    Ravelin::VectorNd u = get_joint_generalized_value(Pacer::Controller::load_goal);
    OUTLOG(u,"U_NO_LIMIT",logERROR);

    for (int i=0;i<u.rows(); i++) {
      if(u[i] > load_max[0]) u[i] = load_max[0];
      else if(u[i] < -load_max[0]) u[i] = -load_max[0];
    }
    OUTLOG(u,"U_WITH_LIMIT",logERROR);
    set_joint_generalized_value(Pacer::Controller::load_goal,u);
  }
  
  reset_contact();
  last_time = t;
  OUT_LOG(logDEBUG) << "<< Controller::control(.)";
}
// ===========================  END CONTROLLER  ===============================
// ============================================================================
