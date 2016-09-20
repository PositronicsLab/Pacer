#include "../../service.h"

#include <map>
#include <unistd.h>
#include <iostream>

#include <algorithm>
#include <vector>
#include <numeric>
#include <functional>

using namespace Pacer::Service;

#ifdef USE_THREADS
#include <pthread.h>
std::vector<pthread_t> worker_threads;
#endif

std::vector<Client> workers;
std::vector<int> thread_complete;
static void *thread_worker(void *threadid)
{
  long process;
  process = (long)threadid;
  
  std::string request_reply = "PlanningService-->SimulationService: Start this simulation: TODO: POLICY INFO";
  workers[process].request( request_reply );
  
  while (1) {
    std::string request_reply = "PlanningService-->SimulationService: Give me simulation data";
    workers[process].request( request_reply );
    if(request_reply == "complete"){
      thread_complete[process] = 1;
    } else {
      thread_complete[process] = 0;
    }
  }
}

int main( int argc, char* argv[] ) {
  std::string port_id_robot = "pacer-planner-robot";
  Server server = Server(port_id_robot);

  {
    std::string request;
    server.serve( request );
    
    std::string response = "PlanningService-->Robot: started, connected to Robot";
    server.respond( response );

  }
  
  int N_THREADS = 10;
  workers.resize(N_THREADS);
  thread_complete.resize(N_THREADS);
  worker_threads.resize(N_THREADS);
  for (int i=0; i<N_THREADS; i++) {
    // Run each sample as its own process
    pid_t pid = fork();
    if( pid < 0 ) {
      // fork failed
      throw std::runtime_error("Fork failed!");
    }
    
    if( pid == 0 ) {
      /// ---------- START CHILD PROCESS ------------- ///
      // execve
      execl("./Plugin/Component/mc-service/simple-simulation","simple-simulation");
      
      assert(false);
    } else {
      /// ---------- START PARENT PROCESS ------------- ///
      // wait for fork
      tick();
      std::string port_id_simulation = "pacer-planner-" + SSTR(pid);
      Client& client = ( workers[i] = Client(port_id_simulation));

      std::string request_reply = "PlanningService-->SimulationService: sending request to SimulationService!";
      client.request( request_reply );
    } // Start next child
  }
  
  
  {
    // Pause PlannerService to wait for job from Robot
    std::string request;
    server.serve( request );
    
    std::string response = "PlanningService-->Robot: got the Job!";
    server.respond( response );
  }
  
  for (int i=0;i<N_THREADS; i++) {
    int iret = pthread_create( &worker_threads[i], NULL,&thread_worker, (void *)i);
    if(iret)
    {
      fprintf(stderr,"Error - pthread_create() return code: %d\n",iret);
      exit(1);
    }
  }
  
  std::string request;
  server.serve( request );

  int product;
  do {
    
    int sum = std::accumulate(thread_complete.begin(), thread_complete.end(), 0);
      std::string response = "PlanningService-->Robot: waiting for jobs to complete! ("+ SSTR(sum) + "/" + SSTR(N_THREADS) +")";
      server.respond( response );
    
    tick(0,100);
    std::string request;
    server.serve( request );

    product = std::accumulate(thread_complete.begin(), thread_complete.end(), 1, std::multiplies<int>());
  } while(product == 0);
  
  {
    std::string response = "PlanningService-->Robot: TODO: DATA RESPONSE";
    server.respond( response );
  }
  
  {
  std::string request;
  server.serve( request );

  std::string response = "complete";
  server.respond( response );
  }
  
  std::cout << "PlanningService: Completed Successfully!" << std::endl;

  return 0;
}

