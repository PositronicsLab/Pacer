#include <Moby/ControlledBody.h>
#include <Moby/TimeSteppingSimulator.h>
#include <Moby/RCArticulatedBody.h>
#include <Moby/ArticulatedBody.h>
#include <stdlib.h>
#include <Ravelin/RCArticulatedBodyd.h>
#include <Ravelin/ArticulatedBodyd.h>
#include <Ravelin/Jointd.h>
#include <Ravelin/Transform3d.h>
#include <Ravelin/Vector3d.h>
#include <boost/shared_ptr.hpp>
#include <set>


#include <Pacer/controller.h>

using Moby::Simulator;
using Ravelin::RigidBodyd;
using Ravelin::Jointd;
using Ravelin::RCArticulatedBodyd;
using Ravelin::Pose3d;
using Ravelin::DynamicBodyd;
using boost::shared_ptr;

using namespace Pacer;

namespace Moby {
  extern void close();
  extern bool init(int argc, char** argv, void* s);
  extern bool step(void* s);
}

static char** param_array_noconst( std::vector< std::string >& params ) {
  
  char** pa = (char**)malloc( sizeof(char*) * (params.size() + 1) );
  for( unsigned i = 0; i < params.size(); i++ ) {
    pa[i] = (char*)params[i].c_str();
  }
  pa[ params.size() ] = NULL;
  
  return (char**) pa;
}

int main(int argc, char** argv){

  shared_ptr<Simulator> sim;
  std::string pacer_model_path(getenv ("PACER_MODEL_PATH"));
  std::string pacer_interface_path(getenv ("PACER_INTERFACE_PATH"));
  // run sample
  std::vector<std::string> argvs;
  for(int i=0;i<argc;i++)
    argvs.push_back(argv[i]);
  // OSG output first and last viewer frame
  //  argvs.push_back("-y=osg");
  //  argvs.push_back("-v=0");
  // XML output first and last state with viz info
  //  argvs.push_back("-w=0");
  argvs.push_back("-p="+pacer_interface_path+"/libPacerMobyPlugin.so");
  //argvs.push_back(pacer_model_path+"/links/model.xml");
  argvs.push_back("model.xml");
  
  char** moby_argv = param_array_noconst(argvs);
  
  // Ask Moby to init the simulator with options
  Moby::init(argvs.size(), moby_argv,(void*) &sim);

  if(!sim)
    throw std::runtime_error("Could not start Moby");
  
  // get event driven simulation and dynamics bodies
  shared_ptr<Ravelin::ArticulatedBodyd> robot;
  
  BOOST_FOREACH(shared_ptr<Moby::ControlledBody> db, sim->get_dynamic_bodies()){
    std::cout << "Checking:" << db->id << std::endl;
    if(!robot){
      shared_ptr<Moby::ArticulatedBody> _robot;
      _robot = boost::dynamic_pointer_cast<Moby::ArticulatedBody>(db);
      robot = boost::dynamic_pointer_cast<Ravelin::ArticulatedBodyd>(_robot);
    }
  }
  
  // Fail if moby was inited wrong
  if(!robot)
    throw std::runtime_error("Could not find robot");
  
  /*
   *  Collecting Initial data
   */
  {
    Ravelin::VectorNd q,qd;
    robot->get_generalized_coordinates_euler(q);
    std::cout << "q1 = " << q << std::endl;
    
    robot->get_generalized_velocity(DynamicBodyd::eSpatial,qd);
    std::cout << "qd1 = " << qd << std::endl;
  }
  
  /*
   *  Running experiment
   */
  bool stop_sim = false;
  while (!stop_sim) {
    // NOTE: Applied in Pacer -- for now
    // apply_control_uncertainty(argc, argv,robot);
    stop_sim = !Moby::step((void*) &sim);
  }
  
  /*
   *  Collecting final data
   */
  {
    Ravelin::VectorNd q,qd;
    robot->get_generalized_coordinates_euler(q);
    std::cout << "q2 = " << q << std::endl;
    
    robot->get_generalized_velocity(DynamicBodyd::eSpatial,qd);
    std::cout << "qd2 = " << qd << std::endl;
  }
  
  // Clean up Moby
  Moby::close();
  
  return 0;
}
