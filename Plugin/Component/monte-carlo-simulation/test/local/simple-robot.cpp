#include "../../service.h"
#include <iostream>

using namespace Pacer::Service;

int main( int argc, char* argv[] ) {
  std::string port_id = "pacer-planner-robot";
  Client client = Client(port_id);

  std::string request;
  
  {
    std::string request_reply = "Robot: connected to planning service!";
    client.request(request_reply);
  }
  
  {
    // Process rquest
    // send response
    std::string request_reply = "Robot: TODO: PLANNING JOB DESCRIPTION";
    client.request(request_reply);
  }
  
  while (true) {
    // Process rquest
    // send response
    std::string request_reply = "Robot: Are you done yet?";
    client.request(request_reply);
    if(request_reply == "complete"){
      break;
    }
  }
  
  std::cout << "Robot: Completed Successfully!" << std::endl;
  return 0;
}
