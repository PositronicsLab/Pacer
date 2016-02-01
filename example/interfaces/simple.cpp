#include <Moby/ControlledBody.h>
#include <Moby/TimeSteppingSimulator.h>
#include <Ravelin/RCArticulatedBodyd.h>
#include <Ravelin/Jointd.h>
#include <Ravelin/Transform3d.h>
#include <Ravelin/Vector3d.h>

#include <stdio.h>
#include <set>
#include <string.h>

#include <boost/shared_ptr.hpp>

#include <sstream>

#include <Pacer/utilities.h>

//#ifdef NDEBUG
#define logging \
if (1) ; \
else std::cout
//#else
//#define logging \
//if (0) ; \
//else std::cout
//#endif

#define SSTR( x ) dynamic_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()


using Moby::Simulator;
using Ravelin::RigidBodyd;
using Ravelin::Jointd;
using Ravelin::RCArticulatedBodyd;
using Ravelin::Pose3d;
using Ravelin::DynamicBodyd;
using boost::shared_ptr;

namespace Moby {
  extern void close();
  extern bool init(int argc, char** argv, boost::shared_ptr<Simulator>& s);
  extern bool step(boost::shared_ptr<Simulator> s);
}

static char** param_array_noconst( std::vector< std::string >& params ) {
  
  char** pa = (char**)malloc( sizeof(char*) * (params.size() + 1) );
  for( unsigned i = 0; i < params.size(); i++ ) {
    pa[i] = (char*)params[i].c_str();
  }
  pa[ params.size() ] = NULL;
  
  return (char**) pa;
}

#include <unistd.h>
#include <boost/program_options.hpp>
namespace po = boost::program_options;

void apply_simulator_options(int argc, char* argv[], shared_ptr<Simulator>& sim){
  // Declare the supported options.
  po::options_description desc("Moby initialization options");
  desc.add_options()
  ("help", "produce help message")
  // INPUT BY USER
  ("duration", po::value<std::string>()->default_value("20"), "set duration (virtual time) of each sample")
  ("stepsize,s", po::value<std::string>()->default_value("0.001"), "set step size (virtual time) of each iteration of the simulatior")
  ("display,r","visualize in moby");
  
  logging << "Parsing Variable Map from command line" << std::endl;
  
  po::variables_map vm;
  //  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::parsed_options parsed
  = po::command_line_parser(argc, argv).style(po::command_line_style::unix_style ^ po::command_line_style::allow_short).options(desc).allow_unregistered().run();
  po::store(parsed, vm);
  //  po::notify(vm);
  
  logging << "Parsed Variable Map from command line" << std::endl;
  
  if ( vm.count("help")  )
  {
    logging << "Available options: " << std::endl
    << desc << std::endl;
    exit(0);
  }
  
  std::string pacer_interface_path(getenv ("PACER_INTERFACE_PATH"));
  logging << "PACER_INTERFACE_PATH: " << pacer_interface_path << std::endl;
  
  std::string
  step_size = vm["stepsize"].as<std::string>(),
  duration = vm["duration"].as<std::string>();
  
  /*
   *  Moby Initialization
   */
  logging << " -- Populating command line options -- " << std::endl;
  
  // run sample
  std::vector<std::string> argvs;
  argvs.push_back("moby-sample");
  // Max time is 0.3 seconds
  argvs.push_back("-mt="+duration);
  argvs.push_back("-s="+step_size);
  // XML output last frame
  //  argvs.push_back("-w=0");
  if (vm.count("display")) {
    argvs.push_back("-r");
  } else {
    // Output ot OSG files
        argvs.push_back("-y=osg");
    //    double capture_step = 0.01;
    //    int rate = capture_step / atof(step_size.c_str());
    //    rate = std::max(1,rate);
    //    argvs.push_back("-v="+SSTR(rate));
        argvs.push_back("-v=0");
  }
  argvs.push_back("-p="+pacer_interface_path+"/libPacerMobyPlugin.so");
  argvs.push_back("model.xml");
  //  argvs.push_back("start.xml");
  
  logging << "Converting command line options: " << argvs << std::endl;
  
  char** moby_argv = param_array_noconst(argvs);
  
  logging << "Moby Starting: " << std::endl;
  for ( size_t i = 0 ; i < argvs.size() ; i++ ){
    logging << argvs[i] << " ";
  }
  logging << std::endl;
  
  // Ask Moby to init the simulator with options
  Moby::init(argvs.size(), moby_argv,sim);
  
  logging << "Moby Started: " << std::endl;
  
  return;
  // clean up argv
  for ( size_t i = 0 ; i < argvs.size() ; i++ ){
    delete [] moby_argv[i];
  }
}


/////////////////////////////////////////////////////////////////
/// -------------------- MAIN FUNCTION -----------------------///
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>

int main(int argc, char* argv[]){
  logging << " -- Sample Started -- " << std::endl;
  
  logging << "Command line arguments: " << std::endl;
  for (int i=0; i<argc; i++) {
    logging << argv[i] << " ";
  }
  logging << std::endl;
  
  shared_ptr<Simulator> sim;
  logging << " -- Applying Simulator Options -- " << std::endl;
  apply_simulator_options(argc,argv,sim);
  logging << " -- Applied Simulator Options -- " << std::endl;
  
  /*
   *  Option Parsing
   */
  
  if(!sim)
    throw std::runtime_error("Could not start Moby");
  
  logging << " -- Created Simulator -- " << std::endl;
  
  // get event driven simulation and dynamics bodies
  shared_ptr<RCArticulatedBodyd> robot;
  shared_ptr<RigidBodyd> environment;
  
  BOOST_FOREACH(shared_ptr<Moby::ControlledBody> db, sim->get_dynamic_bodies()){
    if(!robot)
      robot = boost::dynamic_pointer_cast<RCArticulatedBodyd>(db);
    if(!environment)
      environment = boost::dynamic_pointer_cast<RigidBodyd>(db);
  }
  
  // Fail if moby was inited wrong
  if(!robot)
    throw std::runtime_error("Could not find robot");
  
  logging << " -- Found Robot -- " << std::endl;
  
//  if(!environment)
//    throw std::runtime_error("Could not find environment");
  
//  logging << " -- Found Environment -- " << std::endl;
  
  /*
   *  Running experiment
   */
  logging << " -- Starting simulation -- " << std::endl;
  bool stop_sim = false;
  while (!stop_sim) {
    logging << " -- Stepping simulation -- " << std::endl;
    // NOTE: Applied in Pacer -- for now
    // apply_control_uncertainty(argc, argv,robot);
    stop_sim = !Moby::step(sim);
    logging << "Simulation at time: t = " << sim->current_time <<  std::endl;
  }
  
  // Clean up Moby
  Moby::close();
  
  return 0;
}
