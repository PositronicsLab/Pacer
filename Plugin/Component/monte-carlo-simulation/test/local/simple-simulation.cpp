#include "../../service.h"

#include <cstdlib>
#include <iostream>
#include <ctime>

using namespace Pacer::Service;

int main( int argc, char* argv[] ) {
  std::string port_id = "pacer-planner-" + SSTR(getpid());
  Server server = Server(port_id);
  
  {
    // Wait for message from a client
    std::string request;
    server.serve( request );
    
    // Acknowledge that server has started
    std::string reply = "SimulationService: [" + port_id + "] , has connected to PlanningService!";
    server.respond(reply);
  }
  
  // Wait on job
  {
    std::string request;
    server.serve(request);
    
    std::string reply = "SimulationService: Job received";
    server.respond(reply);
  }
  
  //how long will this take?
  /* initialize random seed: */
  
  {
    std::string request;
    server.serve(request);
    
    sleep(10);
    
    std::string reply = "complete";
    server.respond(reply);
  }
  
  std::cout << "SimulationService: [" + port_id + "] Completed Successfully!" << std::endl;
  
  return 0;
}

