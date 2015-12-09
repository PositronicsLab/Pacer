#include <Moby/ControlledBody.h>
#include <Moby/TimeSteppingSimulator.h>
#include <Ravelin/RCArticulatedBodyd.h>
#include <Ravelin/Jointd.h>
#include <Ravelin/Transform3d.h>
#include <Ravelin/Vector3d.h>

#include <stdio.h>
#include <set>
#include <string.h>
#include "common.h"
#include "random.h"

#include <boost/shared_ptr.hpp>

#include <sstream>

#include <Pacer/utilities.h>

//#ifdef NDEBUG
//#define logging \
//if (0) ; \
//else std::cout
//#else
#define logging \
if (0) ; \
else std::cout
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
unsigned SAMPLE_NUMBER = 0;
int fd1 = 0;
int pid = 0;


namespace Moby {
  extern void close();
  extern bool init(int argc, char** argv, boost::shared_ptr<Simulator>& s);
  extern bool step(boost::shared_ptr<Simulator> s);
}

void apply_transform(Ravelin::Transform3d& T,const std::set<shared_ptr<Jointd> >& outer_joints){
  BOOST_FOREACH(shared_ptr<Jointd> jp, outer_joints){
    shared_ptr<RigidBodyd> rb = jp->get_outboard_link();
    rb->set_pose(T.transform(*(rb->get_pose().get())));
    //    jp->set_pose(T.transform(*(jp->get_pose().get())));
    //    jp->set_location(
    //                     Ravelin::Vector3d(0,0,0,jp->get_pose()),
    //                     jp->get_inboard_link(),
    //                     jp->get_outboard_link());
    apply_transform(T,rb->get_outer_joints());
  }
}

#include <unistd.h>
#include <boost/program_options.hpp>
namespace po = boost::program_options;

void apply_state_uncertainty(int argc,char* argv[],shared_ptr<RCArticulatedBodyd>& robot){
  logging << ">> apply_state_uncertainty" << std::endl;

  po::options_description desc("Monte Carlo Method state (applied at simulator start) uncertainty options");
  desc.add_options()
  ("help", "produce help message")
  ("BODY0.x"    ,   po::value<std::vector<double> >()->multitoken(),  "Absolute Position [m OR rad] of Robot base")
  ("BODY0.xd"    ,   po::value<std::vector<double> >()->multitoken(),  "Absolute Velocity [m/s OR rad/s] of Robot base");
  
  BOOST_FOREACH(shared_ptr<Jointd> jp, robot->get_joints()){
    std::string name,help;
    name = std::string(jp->joint_id+".x");
    help = std::string("Absolute Position [m OR rad] of Joint: "+jp->joint_id);
    
    desc.add_options()
    (name.c_str(), po::value<std::vector<double> >()->multitoken() ,help.c_str());
    
    name = std::string(jp->joint_id+".xd");
    help = std::string("Absolute Velocity [m/s OR rad/s] of Joint: "+jp->joint_id);
    
    desc.add_options()
    (name.c_str(), po::value<std::vector<double> >()->multitoken() ,help.c_str());
  }
  
  po::variables_map vm;
  
  po::parsed_options parsed
  = po::command_line_parser(argc, argv).options(desc).style(po::command_line_style::unix_style ^ po::command_line_style::allow_short).allow_unregistered().run();
  po::store(parsed, vm);
  
  if ( vm.count("help")  )
  {
    logging << "Available options: " << std::endl
    << desc << std::endl;
  }
  
  /*
   *  Parameter Application
   */
  if(vm.count("BODY0.x")){
    std::vector<double> position = vm["BODY0.x"].as<std::vector<double> >();
    logging << "applying state uncertainty to base (position): "<< position << std::endl;

    // Get robot velocity
    Ravelin::VectorNd q;
    robot->get_generalized_coordinates_euler(q);
    int N_JOINT_DOFS = q.rows()-7;
    // update base velocity
    for (int i=0;i<3; i++) {
      q[N_JOINT_DOFS+i] = position[i];
    }
    Ravelin::Quatd quat=Ravelin::Quatd::rpy(position[3],position[4],position[5]);
    for (int i=0;i<4; i++) {
      q[N_JOINT_DOFS+3+i] = quat[i];
    }

    // apply changes
    robot->set_generalized_coordinates_euler(q);
    logging << "Set coords to : " << q << std::endl;

  }
   /*
  if(vm.count("BODY0.x")){
    // Get base Pose
    Pose3d P_base(*(robot->get_base_link()->get_pose().get()));
    // update to relative to global frame
    P_base.update_relative_pose(Moby::GLOBAL);

    std::vector<double> position = vm["BODY0.x"].as<std::vector<double> >();
    // update linear
    P_base.x = Ravelin::Origin3d(position[0],position[1],position[2]);
    // set angular
    P_base.q = Ravelin::Quatd::rpy(position[3],position[4],position[5]);
    // apply changes
    robot->get_base_link()->set_pose(P_base);
  }
  // Update robot state
*/
  robot->update_link_poses();
  // Joints
  BOOST_FOREACH(shared_ptr<Jointd> jp, robot->get_joints()){
    if(vm.count(jp->joint_id+".x")){
      std::vector<double> position = vm[jp->joint_id+".x"].as<std::vector<double> >();
      for (int i=0; i<jp->num_dof(); i++) {
        jp->q[i] = position[i];
      }
    }
  }
  
  // Update robot state
  robot->update_link_poses();
  
  if(vm.count("BODY0.xd")){
    logging << "applying state uncertainty to base (velocity)" << std::endl;

    std::vector<double> velocity = vm["BODY0.xd"].as<std::vector<double> >();
    // Get robot velocity
    Ravelin::VectorNd qd;
    robot->get_generalized_velocity(DynamicBodyd::eSpatial,qd);
    int N_JOINT_DOFS = qd.rows()-6;
    // update base velocity
    for (int i=0;i<6; i++) {
      qd[N_JOINT_DOFS+i] = velocity[i];
    }
    // apply changes
    robot->set_generalized_velocity(DynamicBodyd::eSpatial,qd);
  }
  
  // Joints
  BOOST_FOREACH(shared_ptr<Jointd> jp, robot->get_joints()){
    if(vm.count(jp->joint_id+".xd")){
      std::vector<double> velocity = vm[jp->joint_id+".xd"].as<std::vector<double> >();
      for (int i=0; i<jp->num_dof(); i++) {
        jp->qd[i] = velocity[i];
      }
    }
  }
  
  // Update robot state
  robot->update_link_velocities();
}

void apply_manufacturing_uncertainty(int argc,char* argv[],shared_ptr<RCArticulatedBodyd>& robot){
  // Add permissible robot parameters
  po::options_description desc("Monte Carlo Method manufacturing (applied at simulator start) uncertainty options");
  desc.add_options()
  ("help", "produce help message");
  
  BOOST_FOREACH(shared_ptr<RigidBodyd> rb, robot->get_links()){
    std::string name,help;
    name = std::string(rb->body_id+".density");
    help = std::string("Density [kg/m^3] applied of link: "+rb->body_id);
    
    desc.add_options()
    (name.c_str(), po::value<double>()->default_value(0),help.c_str());
    
    if(rb->is_base())
      continue;
    
    name = std::string(rb->body_id+".length");
    help = std::string("Percent LENGTH error [%] applied to link LENGTH dimension: "+rb->body_id);
    desc.add_options()
    (name.c_str(), po::value<double>()->default_value(0),help.c_str());
    
    name = std::string(rb->body_id+".radius");
    help = std::string("Radius [m] of link RADIUS dimension: "+rb->body_id);
    
    desc.add_options()
    (name.c_str(), po::value<double>()->default_value(0),help.c_str());
    
    if(rb->is_end_effector()){
      name = std::string(rb->body_id+".foot.radius");
      help = std::string("Radius [m] of foot RADIUS dimension: "+rb->body_id);
      
      desc.add_options()
      (name.c_str(), po::value<double>()->default_value(0),help.c_str());
    }
  }
  
  po::variables_map vm;
  po::parsed_options parsed
  = po::command_line_parser(argc, argv).options(desc).style(po::command_line_style::unix_style ^ po::command_line_style::allow_short).allow_unregistered().run();
  po::store(parsed, vm);
  
  if ( vm.count("help")  )
  {
    logging << "Available options: " << std::endl
    << desc << std::endl;
  }
  
  /*
   *  Parameter Application
   */
  
  // Perturb mass of robot link by scaling by parameter
  BOOST_FOREACH(shared_ptr<RigidBodyd> rb, robot->get_links()){
    // Link Length Adjustment
    double length;
    
    if( vm.count(rb->body_id+".length") ){
      double scale_length = 1.0 + vm[rb->body_id+".length"].as<double>();
#ifndef NDEBUG
      fprintf(stdout,"Sample %u : Link %s length += %f\n",SAMPLE_NUMBER,rb->body_id.c_str(),scale_length);
#endif
      
      shared_ptr<Jointd> jp = rb->get_inner_joint_explicit();
      Pose3d rb_original_pose(*(rb->get_pose().get()));
      rb_original_pose.update_relative_pose(rb->get_pose()->rpose);
      
#ifndef NDEBUG
      logging << "Before: " << rb_original_pose << std::endl;
#endif
      // Update transform in link forward direction
      rb_original_pose.x *= scale_length;
#ifndef NDEBUG
      logging << "After: " <<  rb_original_pose << std::endl;
#endif
      length = rb_original_pose.x.norm();
      rb->set_pose(rb_original_pose);
      
      // Update transforms
      jp->Ravelin::Jointd::set_location(jp->get_location(),jp->get_inboard_link(),jp->get_outboard_link());
      // Apply new transform to all children (recursive)
      // NOTE: Doesnt seem to be necessary
      //          apply_transform(T,rb->get_outer_joints());
      
      robot->update_link_poses();
    }
    
    // Link Mass Adjustment
    if( vm.count(rb->body_id+".density")  && vm.count(rb->body_id+".radius")){
      double density = vm[rb->body_id+".density"].as<double>();
      double radius = vm[rb->body_id+".radius"].as<double>();
      
      // cylinder V = pi*h*r^2
      double volume = M_PI*length*sqr(radius);
      // m = d*V
      double mass = density * volume;
      // cylinder J =
      // (m*r^2)/2          0                   0
      //    0      (m/12)(3r^2 + h^2)           0
      //    0               0          (m/12)(3r^2 + h^2)
      double J[3] = {(mass*sqr(radius)*0.5),(mass/12.0)*(3.0*sqr(radius) + sqr(length)),(mass/12.0)*(3.0*sqr(radius) + sqr(length))};
      
#ifndef NDEBUG
      fprintf(stdout,"Sample %u : Link %s mass = %f\n",SAMPLE_NUMBER,rb->body_id.c_str(),mass);
#endif
      Ravelin::SpatialRBInertiad Jm(rb->get_inertia());
      Jm.m = mass;
      Jm.J = Ravelin::Matrix3d(J[0],0,0,
                               0,J[1],0,
                               0,0,J[2]);
      rb->set_inertia(Jm);
    }
    
    if(rb->is_end_effector()){
      if( vm.count(rb->body_id+".foot.radius")){
        boost::shared_ptr<Moby::RigidBody> rigid_body = boost::dynamic_pointer_cast<Moby::RigidBody>(rb);
        //        std::list<CollisionGeometryPtr>& geometries = rigid_body->geometries;
        
        
        //        double density = vm[rb->body_id+".foot.density"].as<double>();
        double radius = vm[rb->body_id+".foot.radius"].as<double>();
        /*
         // sphere V = 4/3 pi*r^3
         double volume = (4.0/3.0)*M_PI*(radius*radius*radius);
         
         // m = d*V
         double mass = density * volume;
         
         // cylinder J =
         // (2*r^2)/5     0         0
         //    0      (2*r^2)/5     0
         //    0          0     (2*r^2)/5
         double inertia = 2.0*mass*sqr(radius) / 5.0;
         double J[3] = {inertia,inertia,inertia};
         
         #ifndef NDEBUG
         fprintf(stdout,"Sample %u : Link (foot) %s mass = %f\n",SAMPLE_NUMBER,rb->body_id.c_str(),mass);
         #endif
         Ravelin::SpatialRBInertiad Jm(rb->get_inertia());
         Jm.m = mass;
         Jm.J = Ravelin::Matrix3d(J[0],0,0,
         0,J[1],0,
         0,0,J[2]);
         rb->set_inertia(Jm);
         */
        
      }
    }
  }
}

void apply_simulator_options(int argc, char* argv[], shared_ptr<Simulator>& sim){
  // Declare the supported options.
  po::options_description desc("Moby initialization options");
  desc.add_options()
  ("help", "produce help message")
  // INPUT BY EXPERIMENT
  ("sample", po::value<unsigned>(),"Sample Number")
  ("pipe", po::value<int>()->default_value(0),"Pipe fd (output side only 'fd1')")
  // INPUT BY USER
  ("duration", po::value<std::string>()->default_value("1"), "set duration (virtual time) of each sample")
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

  
  if (vm.count("pipe")) {
    fd1 = vm["pipe"].as<int>();
    logging << "Sample output FD: " << fd1 << std::endl;
    char * buf = "abcdefghij";
    write(fd1, buf, 10);
    logging << "EXITING: " << std::endl;
    exit(0);
    logging << "EXITED: " << std::endl;
  } else {
    fprintf(stderr,"Sample with PID: %d did not get a port fd", pid);
  }
  
  // Get sample number for output
  if (vm.count("sample")) {
    SAMPLE_NUMBER = vm["sample"].as<unsigned>();
  } else {
    fprintf(stdout,"Sample with PID: %d did not get a sample number", pid);
    exit(1);
  }
  
  logging << "Sample with PID: "<< pid << " has sample number "<< SAMPLE_NUMBER << std::endl;
  
  if ( vm.count("help")  )
  {
    logging << "Available options: " << std::endl
    << desc << std::endl;
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
  argvs.push_back("montecarlo-moby-sample");
  // Max time is 0.3 seconds
  argvs.push_back("-mt="+duration);
  argvs.push_back("-s="+step_size);
  // XML output last frame
  //  argvs.push_back("-w=0");
  if (vm.count("display")) {
    argvs.push_back("-r");
  } else {
//    argvs.push_back("-y=osg");
//    double capture_step = 0.01;
//    int rate = capture_step / atof(step_size.c_str());
//    rate = std::max(1,rate);
//    argvs.push_back("-v="+SSTR(rate));
//    argvs.push_back("-v=0");
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
  
  for (int i=0; i<argc; i++) {
    std::cout << argv[i] << " ";
  }
  std::cout << std::endl;
  

  // Cet this process's PID for debugging
  pid = getpid();
  
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

  if(!environment)
    throw std::runtime_error("Could not find environment");
  
  logging << " -- Found Environment -- " << std::endl;
  
  // Apply uncertainty to robot model
//  apply_manufacturing_uncertainty(argc, argv,robot);
  
  // Apply uncertainty to robot initial conditions
  logging << " -- Applying State Uncertainty -- " << std::endl;
  apply_state_uncertainty(argc, argv,robot);
  logging << " -- Applied State Uncertainty -- " << std::endl;

  /*
   *  Collecting Initial data
   */
//  {
//    Ravelin::VectorNd q,qd;
//    robot->get_generalized_coordinates_euler(q);
//    logging << "q1 = " << q << std::endl;
//    
//    robot->get_generalized_velocity(DynamicBodyd::eSpatial,qd);
//    logging << "qd1 = " << qd << std::endl;
//  }
  
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
  
  /*
   *  Collecting final data
   */
//  {
//    Ravelin::VectorNd q,qd;
//    robot->get_generalized_coordinates_euler(q);
//    logging << "q2 = " << q << std::endl;
//    
//    robot->get_generalized_velocity(DynamicBodyd::eSpatial,qd);
//    logging << "qd2 = " << qd << std::endl;
//  }
  
  // Clean up Moby
  Moby::close();
  
  int sim_elapsed_time = sim->current_time * 1.0e6;
  
  close(fd1);

  _exit(sim_elapsed_time);
  return sim_elapsed_time;
}
