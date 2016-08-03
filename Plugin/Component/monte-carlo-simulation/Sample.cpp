#include <Moby/ControlledBody.h>
#include <Moby/TimeSteppingSimulator.h>
#include <Moby/RCArticulatedBody.h>
#include <Moby/ArticulatedBody.h>
#include <Moby/CollisionGeometry.h>

#include <Ravelin/RCArticulatedBodyd.h>
#include <Ravelin/RevoluteJointd.h>

#include <Ravelin/Transform3d.h>
#include <Ravelin/Vector3d.h>

#include <Moby/XMLWriter.h>

#include <stdio.h>
#include <set>
#include <string.h>

#ifdef USE_OSG_DISPLAY
#include "visualize.cpp"
#endif

#include <boost/shared_ptr.hpp>

#include <sstream>

#include <Pacer/utilities.h>

#include <cstdlib>
#include <ctime>

#include "service.h"

#define ARM

using namespace Pacer::Service;

using Moby::Simulator;
using Ravelin::RigidBodyd;
using Ravelin::Jointd;
using Ravelin::RCArticulatedBodyd;
using Ravelin::Pose3d;
using Ravelin::DynamicBodyd;
using boost::shared_ptr;

bool EXPORT_XML = false;
int SAMPLE_NUMBER = 0;
bool UPDATE_MODEL = true;

int CHILD_TO_PARENT_WRITE = 0;
int PARENT_TO_CHILD_READ = 0;
bool USE_PIPES = true;
int pid = 0;
double DURATION = 0;

using namespace Ravelin;

namespace Moby {
  extern void close();
  extern bool init(int argc, char** argv, boost::shared_ptr<Simulator>& s);
  extern bool step(boost::shared_ptr<Simulator> s);
}

#include "perturb.cpp"

/////////////////////////////////////////////////////////////////
/// -------------------- MAIN FUNCTION -----------------------///
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>

int main(int argc, char* argv[]){
  
  int argc_main = argc;
  char** argv_main = argv;
  std::cout << " -- Sample Started -- " << std::endl;
  
  std::cout << "argc_main: " << argc_main << std::endl;
  std::cout << "argv_main: " << std::endl;
  for(int i=0;i<argc_main;i++){
    std::cout << "\t" << argv_main[i];
  }
  
  // Cet this process's PID for debugging
  pid = getpid();
  
  parse_command_line_options(argc_main,argv_main);
  
  // DO NOT CHANGE DIRECTORIES BEFORE SETTING UP ZEROMQ
//  std::cout << "Moving working directory to: " << argv_main[1] << std::endl;
//  chdir( argv_main[1] );
  
  std::string port_id = "pacer-planner-" + SSTR(pid);
  std::cout << "Server creating server port " << port_id << " on PID ("<< pid <<").  Client can not be inited yet!" << std::endl;
  Server server = Server(port_id);
  std::cout << "Server created to server port " << port_id << " on PID ("<< pid <<").  Client can be inited after this!" << std::endl;
  
  // CHANGE DIRECTORIES AFTER SETTING UP ZEROMQ
  std::cout << "Moving working directory to: " << argv_main[1] << std::endl;
  chdir( argv_main[1] );

  if(USE_PIPES){
    // Wait for message from a client
    std::string request;
    server.serve( request );
    
    // Acknowledge that server has started
    std::string reply = "SimulationService: [" + port_id + "] , has connected to PlanningService!";
    server.respond(reply);
  }
  
  int argc_sim;
  char** argv_sim;
  if(USE_PIPES){
    std::cout << "Sample (" << SAMPLE_NUMBER << ") with PID: "<< pid <<  " Started! waiting on simulatior options message at port: " << port_id << std::endl;
    
    std::string message;
    server.serve(message);
    
    std::cout << "Sample (" << SAMPLE_NUMBER << ") with PID: "<< pid <<  " read message string: [" << message << "]" << std::endl;
    
    std::vector<std::string> messagev;
    boost::split(messagev, message, boost::is_any_of(" "));
    std::cout << "Message vector: " << messagev << std::endl;

    char** cstrings = new char*[messagev.size()];
    
    for(size_t i = 0; i < messagev.size(); ++i)
    {
      cstrings[i] = new char[messagev[i].size() + 1];
      std::strcpy(cstrings[i], messagev[i].c_str());
    }
    
    argc_sim = messagev.size();
    argv_sim = cstrings;
    
  } else {
    argc_sim = argc_main;
    argv_sim = argv_main;
  }
  
  std::cout << "argc_sim: " << argc_sim << std::endl;
  std::cout << "argv_sim: " << std::endl;
  for(int i=0;i<argc_sim;i++){
    std::cout << " " << argv_sim[i];
  }
  
  // Setup this instance of moby
  shared_ptr<Simulator> sim;
  
  std::cout << " -> Starting Simulator -- " << std::endl;
  preload_simulation(argc_sim,argv_sim, sim);
  std::cout << " -- Started Simulator -> " << std::endl;
  
  if(!sim)
    throw std::runtime_error("Could not start Moby");
  
  std::cout << " -- Created Simulator -- " << std::endl;
  
  // get event driven simulation and dynamics bodies
  shared_ptr<RCArticulatedBodyd> robot;
  shared_ptr<RigidBodyd> environment;
  
  BOOST_FOREACH(shared_ptr<Moby::ControlledBody> db, sim->get_dynamic_bodies()){
    if(!robot)
    robot = boost::dynamic_pointer_cast<RCArticulatedBodyd>(db);
    if(!environment){
      environment = boost::dynamic_pointer_cast<RigidBodyd>(db);
#ifdef ARM
      std::cerr << " Found object : " << db->id << std::endl;
      if(db->id.compare("BLOCK") != 0){
        environment.reset();
      }
#endif
#ifdef QUAD
      if(db->id.compare("GROUND") != 0){
        environment.reset();
      }
#endif
    }
  }
#ifdef ARM
  if(environment->body_id.compare("BLOCK") != 0)
  throw std::runtime_error("Could not find block");
#endif
#ifdef QUAD
  if(environment->body_id.compare("GROUND") != 0)
  throw std::runtime_error("Could not find ground");
#endif
  
  // Fail if moby was inited wrong
  if(!robot)
  throw std::runtime_error("Could not find robot");
  
  std::cout << " -- Found Robot -- " << std::endl;
  {
    std::string response("started");
    server.respond(response);
  }
#ifdef INIT_SIM
  do {
#endif
  int argc_sample;
  char** argv_sample;
  if(USE_PIPES){
    std::cerr << "Sample (" << SAMPLE_NUMBER << ") with PID: "<< pid <<  " Started Simulation! waiting on parameter message" << std::endl;
    
    std::string message;
    server.serve(message);
    
    std::cerr << "Sample (" << SAMPLE_NUMBER << ") with PID: "<< pid <<  " read message string: [" << message << "]" << std::endl;
    
    message = "sample-options " + message;

    std::vector<std::string> messagev;
    boost::split(messagev, message, boost::is_any_of(" "));
    std::cout << "Message vector: " << messagev << std::endl;
    
    char** cstrings = new char*[messagev.size()];
    
    for(size_t i = 0; i < messagev.size(); ++i)
    {
      cstrings[i] = new char[messagev[i].size() + 1];
      std::strcpy(cstrings[i], messagev[i].c_str());
    }
    
    argc_sample = messagev.size();
    argv_sample = cstrings;
  } else {
    argc_sample = argc_main;
    argv_sample = argv_main;
  }
  
  std::cout << "argc_sample: " << argc_sample << std::endl;
    std::cout << "argv_sample: " << std::endl;
    for(int i=0;i<argc_sim;i++){
      std::cout << "\t" << argv_sample[i];
    }
  // Setup this instance of moby
  std::cout << " -> parse_sample_options -- " << std::endl;
  parse_sample_options(argc_sample,argv_sample);
  std::cout << " -- parse_sample_options -> " << std::endl;
  //  sim->current_time = 0;
  
  //  if(!environment)
  //    throw std::runtime_error("Could not find environment");
  
  //  std::cout << " -- Found Environment -- " << std::endl;
  
  //
  // Apply uncertainty to robot model
  std::cout << " -> apply_manufacturing_uncertainty -- " << std::endl;
  if(UPDATE_MODEL)
  apply_manufacturing_uncertainty(argc_sample,argv_sample,robot);
  std::cout << " -- apply_manufacturing_uncertainty -> " << std::endl;
  
  if(EXPORT_XML){
    // write the file (fails silently)
    std::cout << " -- Exporting robot model file -- " << std::endl;
    
    //     Reset robot state to adjust robot model
    Ravelin::VectorNd q_starting_position,q_current_position;
    robot->get_generalized_coordinates_euler(q_starting_position);
    q_current_position = q_starting_position;
    q_current_position.segment(0,q_current_position.rows()-7) = Ravelin::VectorNd::zero(q_current_position.rows()-7);
    robot->set_generalized_coordinates_euler(q_current_position);
    
    char buffer[128];
    sprintf(buffer, "model-%06u.xml", pid);
    boost::shared_ptr<Moby::RCArticulatedBody> robot_moby = boost::dynamic_pointer_cast<Moby::RCArticulatedBody>(robot);
    Moby::XMLWriter::serialize_to_xml(std::string(buffer), sim );
    robot->set_generalized_coordinates_euler(q_starting_position);
  }
  
  // Apply uncertainty to robot initial conditions
  //  std::cout << " -> Applying State Uncertainty -- " << std::endl;
  //  if(UPDATE_MODEL)
  //    apply_state_uncertainty(argc_sample,argv_sample,robot);
  //  std::cout << " -- Applied State Uncertainty -> " << std::endl;
  
  /*
   *  Running experiment
   */
  //  robot->set_generalized_coordinates_euler(q_starting_position);
  
#ifdef USE_OSG_DISPLAY
  sim->update_visualization();
  osg::Group * MAIN_GROUP;
  //  if (!MAIN_GROUP) {
  MAIN_GROUP = new osg::Group;
  MAIN_GROUP->addChild(sim->get_persistent_vdata());
  //    MAIN_GROUP->addChild(sim->get_transient_vdata());
  //  }
#endif
  
  //#ifndef NDEBUG
  std::cerr << "Sample: "<< SAMPLE_NUMBER << " with PID: "<< pid <<  " -- Starting simulation ("<< SAMPLE_NUMBER << ")"<< std::endl;
  //#endif
  bool stop_sim = false;
  unsigned long long ITER = 0;
  
#ifdef ARM
  /////////// std::cout , get hand
  boost::shared_ptr<Ravelin::RigidBodyd> hand;
  BOOST_FOREACH(boost::shared_ptr<Ravelin::RigidBodyd> rbd, robot->get_links()){
    if(!hand){
      if (rbd->body_id.compare("HAND") == 0) {
        hand = rbd;
      }
    }
  }
#endif
  
  {
    std::string response("simulating");
    server.respond(response);
  }
  std::string data_message;
  
  while (!stop_sim &&  sim->current_time < DURATION) {
#ifdef USE_OSG_DISPLAY
    if (ITER % 100 == 0) {
      sim->update_visualization();
      // write the file (fails silently)
      char buffer[128];
      // If you use C's char arrays:
      sprintf(buffer, "frame-%08llu-%d-%d.osg", ITER,pid,SAMPLE_NUMBER);
      osgDB::writeNodeFile(*MAIN_GROUP, std::string(buffer));
    }
#endif
    //    std::cout << " -- Stepping simulation -- " << std::endl;
    // NOTE: Applied in Pacer -- for now
    // apply_control_uncertainty(argc_sample,argv_sample,robot);
#ifdef NDEBUG
    if (USE_PIPES) {
      try {
        stop_sim = !Moby::step(sim);
      } catch (std::exception& e) {
        std::cerr << "There was an error that forced the simulation to stop: "<< e.what() << std::endl;
        stop_sim = true;
      }
    } else
#endif
    {
      stop_sim = !Moby::step(sim);
    }
    
    
#ifdef ARM
    Vector3d block_point = Pose3d::transform_point(Moby::GLOBAL, Vector3d(0,0,0,environment->get_pose()));
    Vector3d hand_point = Pose3d::transform_point(Moby::GLOBAL, Vector3d(0,0,0,hand->get_pose()));
    // Perturb mass of robot link by scaling by parameter
    data_message = SSTR(SAMPLE_NUMBER) + " " + SSTR(sim->current_time) + " " + SSTR(ITER) + " " + SSTR(block_point[0]) + " " + SSTR(block_point[1]) + " " + SSTR(block_point[2]) + " " + SSTR(hand_point[0]) + " " + SSTR(hand_point[1]) + " " + SSTR(hand_point[2]);
#endif
#ifdef QUAD
    Vector3d robot_point = Pose3d::transform_point(Moby::GLOBAL, Vector3d(0,0,0,robot->get_pose()));
    // Perturb mass of robot link by scaling by parameter
    data_message = SSTR(SAMPLE_NUMBER) + " " + SSTR(sim->current_time) + " " + SSTR(ITER) + " " + SSTR(robot_point[0]) + " " + SSTR(robot_point[1]) + " " + SSTR(robot_point[2]);
#endif
    
    fprintf(stderr,"data_message = [ %s ]\n",data_message.c_str());
    ITER++;
  }
  
#ifdef USE_OSG_DISPLAY
  {
    sim->update_visualization();
    
    char buffer[128];
    sprintf(buffer, "last-%d-%d.osg",pid,SAMPLE_NUMBER);
    osgDB::writeNodeFile(*MAIN_GROUP, std::string(buffer));
  }
#endif
  
  //#ifndef NDEBUG
  std::cerr << "Simulation ("<< SAMPLE_NUMBER << ") at time: t = " << sim->current_time  << ", iteration: " << ITER << " Ended!" << std::endl;
  //#endif
//  
//  {
//    std::string message;
//    server.serve(message);
//    std::cerr << "Sample: "<< SAMPLE_NUMBER << " with PID: "<< pid << " got! message: " << message << std::endl;
//  }
  /*
   *  Collecting final data
   */
  Ravelin::VectorNd q,qd;
  robot->get_generalized_coordinates_euler(q);
  std::cout << "q2 = " << q << std::endl;
  
  for (int i=q.rows()-7; i<q.rows() ;i++) {
    data_message += " " + SSTR(q[i]) ;
  }
  
  std::cerr << "Sample: "<< SAMPLE_NUMBER << " with PID: "<< pid << " Ended! message: " << data_message << std::endl;
  if (USE_PIPES) {
    server.respond(data_message);
#ifdef INIT_SIM
    restart = true;
#else
    std::cerr << "execv: "<< SAMPLE_NUMBER << " with PID: "<< pid << " Ended! message: " << data_message << std::endl;
    std::cerr << "execv: "<< argv[0] << std::endl;
    char cwd[1024];
    if (getcwd(cwd, sizeof(cwd)) != NULL)
    fprintf(stdout, "Current working dir: %s\n", cwd);
    else
    perror("getcwd() error");
    return 0;
    
    execl( argv[0] , cwd );
    ///////////////////////////////////////////////////
    /// ---------- EXIT CHILD PROCESS ------------- ///
    ///////////////////////////////////////////////////
    
    // This code should be unreachable unless exec failed
    perror( "execv" );
    throw std::runtime_error("This code should be unreachable unless execv failed!");
    /// ---------- END CHILD PROCESS ------------- ///
    //////////////////////////////////////////////////
#endif
  }
#ifdef INIT_SIM
  }
  while (restart);
#endif
  // Quit like this because quitting normally leads to weird deconstructor errors.
  throw std::runtime_error("Sample exited!");
  
  return 0;
}
