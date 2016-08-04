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
//#define INIT_SIM

//#define DEBUG_OUTPUT
#define LOGGING_OUTPUT

#ifdef LOGGING_OUTPUT

namespace Logging {
  FILE * pFile;
  
  bool safe = false;
  bool opened = false;
  std::string name;
  
  void open(int pid,int id){
    if(pFile){  // make new file if already exists
      fflush(pFile);
      fclose (pFile);
    }
    char buffer[9];
    sprintf(buffer,"%06d",pid);
    //    name = std::string("sample-"+SSTR(pid)+"-"+SSTR(id)+".log");
    name = std::string("sample-"+SSTR(id)+".log");
    pFile = fopen (name.c_str(),"w");
    // if we intend to close it:
    if (safe) {
      fprintf(pFile, "");
      fflush(pFile);
      fclose (pFile);
      opened = false;
    } else {
      opened = true;
    }
  }
  
  void send(std::string& line){
    if (safe && !opened) {
      pFile = fopen(name.c_str(),"a");
      opened = true;
    }
    fprintf(pFile, "%s\n", line.c_str());
    if (safe && !opened) {
      fflush(pFile);
      fclose (pFile);
    }
  }
  
  void close(){
    fflush(pFile);
    fclose (pFile);
  }
}
#endif

#ifdef DEBUG_OUTPUT

#define logging1 \
if (0) ; \
else std::cout

#define logging2 \
if (0) ; \
else std::cerr

#else

#define logging1 \
if (1) ; \
else std::cout

#define logging2 \
if (1) ; \
else std::cerr

#endif

using namespace Pacer::Service;

using Moby::Simulator;
using Ravelin::RigidBodyd;
using Ravelin::Jointd;
using Ravelin::RCArticulatedBodyd;
using Ravelin::Pose3d;
using Ravelin::DynamicBodyd;
using boost::shared_ptr;

bool EXPORT_XML = false;
int  SAMPLE_NUMBER = 0;
bool USE_PIPES = true;
bool USE_UNCER = true;
bool PACER_ONLY = false;

int pid = 0;
double DURATION = 0;
int VISUAL_MOD = 0;
using namespace Ravelin;

namespace Moby {
  extern void close();
  extern bool init(int argc, char** argv, boost::shared_ptr<Simulator>& s);
  extern bool step(boost::shared_ptr<Simulator> s);
}

namespace Pacer {
  extern void init();
  extern double step();
}

#include "sample-perturb.cpp"

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
  logging1 << " -- Sample Started -- " << std::endl;
  
  logging1 << "argc_main: " << argc_main << std::endl;
  logging1 << "argv_main: " << std::endl;
  for(int i=0;i<argc_main;i++){
    logging1 << "\t" << argv_main[i];
  }
  
  // Cet this process's PID for debugging
  pid = getpid();
  
  parse_command_line_options(argc_main,argv_main);
  
  // DO NOT CHANGE DIRECTORIES BEFORE SETTING UP ZEROMQ
  //  logging1 << "Moving working directory to: " << argv_main[1] << std::endl;
  //  chdir( argv_main[1] );
  
  std::string port_id = "pacer-planner-" + SSTR(pid);
  logging1 << "Server creating server port " << port_id << " on PID ("<< pid <<").  Client can not be inited yet!" << std::endl;
  Server server = Server(port_id);
  logging1 << "Server created to server port " << port_id << " on PID ("<< pid <<").  Client can be inited after this!" << std::endl;
  
  // CHANGE DIRECTORIES AFTER SETTING UP ZEROMQ
  logging1 << "Moving working directory to: " << argv_main[1] << std::endl;
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
    logging1 << "PID: "<< pid <<  " Started! waiting on simulatior options message at port: " << port_id << std::endl;
    
    std::string message;
    server.serve(message);
    
    logging1 << "PID: "<< pid <<  " read message string: [" << message << "]" << std::endl;
    
    std::vector<std::string> messagev;
    boost::split(messagev, message, boost::is_any_of(" "));
    logging1 << "Message vector: " << messagev << std::endl;
    
    argc_sim = messagev.size();
    argv_sim = new char*[messagev.size()];
    
    for(size_t i = 0; i < messagev.size(); ++i)
    {
      argv_sim[i] = new char[messagev[i].size() + 1];
      std::strcpy(argv_sim[i], messagev[i].c_str());
    }
  } else {
    argc_sim = argc_main;
    argv_sim = argv_main;
  }
  
  
#ifdef INIT_SIM
  bool restart = false;
  do {
    if (restart) {
      logging1 << "======== PID: "<< pid <<  " restarting moby for next run ========" << std::endl;
      restart = false;
    }
#endif
    
    logging1 << "argc_sim: " << argc_sim << std::endl;
    logging1 << "argv_sim: " << std::endl;
    for(int i=0;i<argc_sim;i++){
      logging1 << " " << argv_sim[i];
    }
    
    // Setup this instance of moby
    shared_ptr<Simulator> sim;
    
    logging1 << " -> Starting Simulator -- " << std::endl;
    preload_simulation(argc_sim,argv_sim, sim);
    logging1 << " -- Started Simulator -> " << std::endl;
    
    if( !PACER_ONLY )
    if(!sim)
    throw std::runtime_error("Could not start Moby");
    
    logging1 << " -- Created Simulator -- " << std::endl;
    
    // get event driven simulation and dynamics bodies
    shared_ptr<RCArticulatedBodyd> robot;
    shared_ptr<RigidBodyd> environment;
    if( !PACER_ONLY )
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
    if( !PACER_ONLY )
    if(environment->body_id.compare("BLOCK") != 0)
    throw std::runtime_error("Could not find block");
#endif
#ifdef QUAD
    if( !PACER_ONLY )
    if(environment->body_id.compare("GROUND") != 0)
    throw std::runtime_error("Could not find ground");
#endif
    
    // Fail if moby was inited wrong
    if( !PACER_ONLY )
    if(!robot)
    throw std::runtime_error("Could not find robot");
    
    
    logging1 << " -- Found Robot -- " << std::endl;
    
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
      std::string response("started");
      server.respond(response);
    }
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
      logging1 << "Message vector: " << messagev << std::endl;
      
      argc_sample = messagev.size();
      argv_sample = new char*[messagev.size()];
      
      for(size_t i = 0; i < messagev.size(); ++i)
      {
        argv_sample[i] = new char[messagev[i].size() + 1];
        std::strcpy(argv_sample[i], messagev[i].c_str());
      }
    } else {
      argc_sample = argc_main;
      argv_sample = argv_main;
    }
    
    logging1 << "argc_sample: " << argc_sample << std::endl;
    logging1 << "argv_sample: " << std::endl;
    for(int i=0;i<argc_sample;i++){
      logging1 << "\t" << argv_sample[i];
    }
    // Setup this instance of moby
    logging1 << " -> parse_sample_options -- " << std::endl;
    parse_sample_options(argc_sample,argv_sample);
    logging1 << " -- parse_sample_options -> " << std::endl;
    
    
    Logging::open(pid,SAMPLE_NUMBER);
    
    // Apply uncertainty to robot model
    logging1 << " -> apply_manufacturing_uncertainty -- " << std::endl;
    if(USE_UNCER)
    apply_manufacturing_uncertainty(argc_sample,argv_sample,robot);
    logging1 << " -- apply_manufacturing_uncertainty -> " << std::endl;
    
    if( !PACER_ONLY )
    if(EXPORT_XML){
      logging1 << " -- Exporting robot model file -- " << std::endl;
      
      //     Reset robot state when exporting model
      Ravelin::VectorNd q_starting_position,q_current_position;
      robot->get_generalized_coordinates_euler(q_starting_position);
      q_current_position = q_starting_position;
      q_current_position.segment(0,q_current_position.rows()-7) = Ravelin::VectorNd::zero(q_current_position.rows()-7);
      robot->set_generalized_coordinates_euler(q_current_position);
      
      std::string model_filename("model-"+SSTR(SAMPLE_NUMBER)+".xml");
      //      boost::shared_ptr<Moby::RCArticulatedBody> robot_moby = boost::dynamic_pointer_cast<Moby::RCArticulatedBody>(robot);
      Moby::XMLWriter::serialize_to_xml(model_filename, sim );
      robot->set_generalized_coordinates_euler(q_starting_position);
    }
    
    // Apply uncertainty to robot initial conditions
    logging1 << " -> Applying State Uncertainty -- " << std::endl;
    apply_state_uncertainty(argc_sample,argv_sample,robot);
    logging1 << " -- Applied State Uncertainty -> " << std::endl;
    
    /*
     *  Running experiment
     */
#ifdef USE_OSG_DISPLAY
    osg::Group * MAIN_GROUP;
    if( !PACER_ONLY )
    if (VISUAL_MOD != 0){
      sim->update_visualization();
      //  if (!MAIN_GROUP) {
      MAIN_GROUP = new osg::Group;
      MAIN_GROUP->addChild(sim->get_persistent_vdata());
      //    MAIN_GROUP->addChild(sim->get_transient_vdata());
      //  }
    }
#endif
    
    logging1 << "Sample: "<< SAMPLE_NUMBER << " with PID: "<< pid <<  " -- Starting simulation ("<< SAMPLE_NUMBER << ")"<< std::endl;
    
    // tell planner you've started
    {
      std::string response("simulating");
      server.respond(response);
    }
    
    // message set to parent or used fr data recording
    std::string data_message;
    
    bool stop_sim = false;
    unsigned long long ITER = 0;

    if(PACER_ONLY){
      Pacer::init();
      std::cerr << "Pacer-only sample (kinematic)" << " with PID: "<< pid <<  " -- Starting simulation"<< std::endl;
      
      double current_time = 0;
      while (current_time <= DURATION) {
        current_time = Pacer::step();
        std::cerr << "PID ("<< pid << ") at time: t = " << current_time  << ", iteration: " << ITER <<  std::endl;
        ITER++;
      }
    }{
      while (!stop_sim &&  sim->current_time < DURATION) {
#ifdef USE_OSG_DISPLAY
        if( !PACER_ONLY )
        if (VISUAL_MOD > 0){
          if(ITER % VISUAL_MOD == 0) {
            sim->update_visualization();
            //          std::string visual_filename = "frame-" +SSTR(ITER)+ "-"+SSTR(pid)+"-"+SSTR(SAMPLE_NUMBER)+".osg";
            std::string visual_filename = "frame-" +SSTR(ITER)+"-"+SSTR(SAMPLE_NUMBER)+".osg";
            osgDB::writeNodeFile(*MAIN_GROUP, visual_filename);
          }
        }
#endif
        //    logging1 << " -- Stepping simulation -- " << std::endl;
        // NOTE: Applied in Pacer -- for now
        // apply_control_uncertainty(argc_sample,argv_sample,robot);
#ifdef NDEBUG
        if (USE_PIPES) {
          try {
            stop_sim = !Moby::step(sim);
          } catch (std::exception& e) {
            logging2 << "There was an error that forced the simulation to stop: "<< e.what() << std::endl;
            stop_sim = true;
          } catch (...) {
            logging2 << "There was an unknown error that forced the simulation to stop: "<< std::endl;
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
        
        logging2 << "data_message = [ " << data_message << " ]" << std::endl;
        Logging::send(data_message);
        
        ITER++;
      }
      
    }
    
    Logging::close();
    
#ifdef USE_OSG_DISPLAY
    if (VISUAL_MOD != 0){
      sim->update_visualization();
      std::string visual_filename = "last-"+SSTR(SAMPLE_NUMBER)+".osg";
      osgDB::writeNodeFile(*MAIN_GROUP, visual_filename);
    }
#endif
    
    logging1 << "Simulation ("<< SAMPLE_NUMBER << ") at time: t = " << sim->current_time  << ", iteration: " << ITER << " Ended!" << std::endl;
    
    {
      std::string message;
      server.serve(message);
      std::cerr << "Sample: "<< SAMPLE_NUMBER << " with PID: "<< pid << " got! message: " << message << std::endl;
    }
    /*
     *  Collecting final data
     */
    Ravelin::VectorNd q,qd;
    robot->get_generalized_coordinates_euler(q);
    logging1 << "q2 = " << q << std::endl;
    
    for (int i=q.rows()-7; i<q.rows() ;i++) {
      data_message += " " + SSTR(q[i]) ;
    }
    
    logging2 << "Sample: "<< SAMPLE_NUMBER << " with PID: "<< pid << " Ended! message to planner: " << data_message << std::endl;
    
    if (USE_PIPES) {
      server.respond(data_message);
    }
    
    
    for (int i = 0 ; i < argc_sample ; i++)
      delete[] argv_sample[i] ;
    delete[] argv_sample ;
    
    for (int i = 0 ; i < argc_sim ; i++)
      delete[] argv_sim[i] ;
    delete[] argv_sim ;
    
    return 0;
  }
  
#ifdef INIT_SIM
#undef INIT_SIM
#endif
