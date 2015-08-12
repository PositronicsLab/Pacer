#include <Pacer/controller.h>
std::string plugin_namespace;

void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t){
  static bool forked = false;
  static int pid;
  if(!forked){
    forked = true;
    
    // Create fork of moby process
    pid = fork();
  }
  
  // Now check which process we're in
  if (pid == 0) {
    std::cout << "Child Process" << std::endl;
    assert(false);
  } else {
    std::cout << "Parent Process will wait for " << std::endl;
  }
}

/** This is a quick way to register your plugin function of the form:
 * void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t)
 */
#include "register-plugin"
