
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

#ifdef NO_GTEST
int main(int argc, char** argv){
#else
#include <gtest/gtest.h>
TEST(TEST_TYPE,TEST_NAME){
#endif

  boost::shared_ptr<Simulator> sim;
  std::string pacer_model_path(getenv ("PACER_MODEL_PATH"));
  std::string pacer_interface_path(getenv ("PACER_INTERFACE_PATH"));
  // run sample
  std::vector<std::string> argvs;
  // OSG output first and last viewer frame
#ifdef NO_GTEST
  argvs.push_back("TEST:PacerTest");
  for(int i=1;i<argc;i++)
    argvs.push_back(argv[i]);
#else
  argvs.push_back("GOOGLE-TEST:PacerTest");
  argvs.push_back("-mt=10");
  argvs.push_back("-v=6");
  argvs.push_back("-s=0.0015");
  argvs.push_back("-y=osg");
#endif
  // XML output first and last state with viz info
  //  argvs.push_back("-w=0");
  argvs.push_back("-p="+pacer_interface_path+"/libPacerMobyPlugin.so");
  argvs.push_back("model.xml");
  
  
  char** moby_argv = param_array_noconst(argvs);
 
  for (int i=0;i<argvs.size();i++)
    std::cout << argvs[i] << " ";
  std::cout << std::endl;

  // Ask Moby to init the simulator with options
  Moby::init(argvs.size(), moby_argv,(void*) &sim);

  if(!sim)
    throw std::runtime_error("Could not start Moby");
  
  // get event driven simulation and dynamics bodies
  boost::shared_ptr<Ravelin::ArticulatedBodyd> robot;
  
  BOOST_FOREACH(boost::shared_ptr<Moby::ControlledBody> db, sim->get_dynamic_bodies()){
    std::cout << "Checking:" << db->id << std::endl;
    if(!robot){
      boost::shared_ptr<Moby::ArticulatedBody> _robot;
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
    robot->get_generalized_coordinates_euler(q_0);
    std::cout << "q0 = " << q_0 << std::endl;
    
    robot->get_generalized_velocity(DynamicBodyd::eSpatial,qd_0);
    std::cout << "qd0 = " << qd_0 << std::endl;
  }
  
  /*
   *  Running experiment
   */
  bool stop_sim = false;
  while (!stop_sim) {
    // NOTE: Applied in Pacer -- for now
    check_current_state(robot);
    stop_sim = !Moby::step((void*) &sim);
  }
  
  /*
   *  Collecting final data
   */
  {
    robot->get_generalized_coordinates_euler(q_f);
    std::cout << "qf = " << q_f << std::endl;
    
    robot->get_generalized_velocity(DynamicBodyd::eSpatial,qd_f);
    std::cout << "qdf = " << qd_f << std::endl;
  }
  check_final_state(robot);
  
  // Clean up Moby
  Moby::close();
  
#ifdef NO_GTEST
  return 0;
#endif
}
