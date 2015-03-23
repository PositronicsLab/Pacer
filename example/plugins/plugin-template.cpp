#include <Pacer/Controller.h>
std::string plugin_namespace;

void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t){
  
}

/** This is a quick way to register your plugin function of the form:
  * void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t)
  */
#include "register-plugin"
