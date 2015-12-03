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
  po::options_description desc("Monte Carlo Method state (applied at simulator start) uncertainty options");
  desc.add_options()
  ("help", "produce help message")
  ("x"    ,   po::value<std::vector<double> >()->multitoken(),  "Absolute Position error [m] applied to Robot base")
  ("xd"    ,   po::value<std::vector<double> >()->multitoken(),  "Relative Velocity error [%] applied to Robot base");
  
  BOOST_FOREACH(shared_ptr<Jointd> jp, robot->get_joints()){
    std::string name,help;
    name = std::string(jp->joint_id+".x");
    help = std::string("Absolute Position error of Joint: "+jp->joint_id);
    
    desc.add_options()
    (name.c_str(), po::value<std::vector<double> >()->multitoken() ,help.c_str());
    
    name = std::string(jp->joint_id+".xd");
    help = std::string("Relative Velocity error of Joint: "+jp->joint_id);
    
    desc.add_options()
    (name.c_str(), po::value<std::vector<double> >()->multitoken() ,help.c_str());
  }
  
  po::variables_map vm;
  po::parsed_options parsed
  = po::command_line_parser(argc, argv).options(desc).allow_unregistered().run();
  po::store(parsed, vm);
  
  if ( vm.count("help")  )
  {
    std::cout << "Available options: " << std::endl
    << desc << std::endl;
  }
  
  /*
   *  Parameter Application
   */
  
  if(vm.count("x")){
    // Get base Pose
    Pose3d P_base(*(robot->get_base_link()->get_pose().get()));
    // update to relative to global frame
    P_base.update_relative_pose(Moby::GLOBAL);
    // update linear
    std::vector<double> position = vm["x"].as<std::vector<double> >();
    P_base.x = Ravelin::Origin3d(position[0],position[1],position[2]);
    // get angular
    double roll,pitch,yaw;
    P_base.q.to_rpy(roll,pitch,yaw);
    // update angular
    roll  += position[3];
    pitch += position[4];
    yaw   += position[5];
    // set angular
    P_base.q = Ravelin::Quatd::rpy(roll,pitch,yaw);
    // apply changes
    robot->get_base_link()->set_pose(P_base);
  }
  
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
  
  if(vm.count("xd")){
    std::vector<double> velocity = vm["xd"].as<std::vector<double> >();
    // Get robot velocity
    Ravelin::VectorNd qd;
    robot->get_generalized_velocity(DynamicBodyd::eSpatial,qd);
    int N_JOINT_DOFS = qd.rows()-6;
    // update base velocity
    for (int i=0;i<6; i++) {
      qd[N_JOINT_DOFS+i] += qd[N_JOINT_DOFS+i]*velocity[i];
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
  = po::command_line_parser(argc, argv).options(desc).allow_unregistered().run();
  po::store(parsed, vm);
  
  if ( vm.count("help")  )
  {
    std::cout << "Available options: " << std::endl
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
      double scale_length = 1.0 + vm[rb->body_id+"length"].as<double>();
#ifndef NDEBUG
      printf("Sample %u : Link %s length += %f\n",SAMPLE_NUMBER,rb->body_id.c_str(),scale_length);
#endif
      
      shared_ptr<Jointd> jp = rb->get_inner_joint_explicit();
      Pose3d rb_original_pose(*(rb->get_pose().get()));
      rb_original_pose.update_relative_pose(rb->get_pose()->rpose);
      
#ifndef NDEBUG
      std::cout << "Before: " << rb_original_pose << std::endl;
#endif
      // Update transform in link forward direction
      rb_original_pose.x *= scale_length;
#ifndef NDEBUG
      std::cout << "After: " <<  rb_original_pose << std::endl;
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
      printf("Sample %u : Link %s mass = %f\n",SAMPLE_NUMBER,rb->body_id.c_str(),mass);
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
         printf("Sample %u : Link (foot) %s mass = %f\n",SAMPLE_NUMBER,rb->body_id.c_str(),mass);
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
  // INPUT BY USER
  ("duration", po::value<std::string>()->default_value("1"), "set duration (virtual time) of each sample")
  ("stepsize,s", po::value<std::string>()->default_value("0.001"), "set step size (virtual time) of each iteration of the simulatior")
  ("display,r","visualize in moby")
  //  ("options", po::value<std::string>()->default_value(""), "other, space-delimited options")
  ;
  
  po::variables_map vm;
  //  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::parsed_options parsed
  = po::command_line_parser(argc, argv).options(desc).allow_unregistered().run();
  po::store(parsed, vm);
  
  po::notify(vm);
  
  // Get sample number for output
  if (vm.count("sample")) {
    SAMPLE_NUMBER = vm["sample"].as<unsigned>();
  } else {
    printf("Sample with PID: %d did not get a sample number", pid);
    exit(1);
  }
  printf("Sample with PID: %d has sample number %u", pid,SAMPLE_NUMBER);
  
  
  if ( vm.count("help")  )
  {
    std::cout << "Available options: " << std::endl
    << desc << std::endl;
  }
  
  std::string pacer_interface_path(getenv ("PACER_INTERFACE_PATH"));
  
  std::string
  step_size = vm["stepsize"].as<std::string>(),
  duration = vm["duration"].as<std::string>();
  
  //  exit(0);
  /*
   *  Moby Initialization
   */
  
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
    argvs.push_back("-y=osg");
    int rate = 0.01 / atof(step_size.c_str());
    rate = std::max(1,rate);
    argvs.push_back("-v="+SSTR(rate));
  }
  argvs.push_back("-p="+pacer_interface_path+"/libPacerMobyPlugin.so");
  argvs.push_back("model.xml");
  //  argvs.push_back("start.xml");
  
  char** moby_argv = param_array_noconst(argvs);
  
  // Ask Moby to init the simulator with options
  Moby::init(argvs.size(), moby_argv,sim);
  
  // clean up argv
  for ( size_t i = 0 ; i < argvs.size() ; i++ ){
    //    delete [] moby_argv[i];
    std::cout << argvs[i] << " ";
  }
  std::cout << std::endl;
}

int main(int argc, char* argv[]){
  // Cet this process's PID for debugging
  pid = getpid();
  
  shared_ptr<Simulator> sim;
  apply_simulator_options(argc,argv,sim);
  /*
   *  Option Parsing
   */
  
  if(!sim)
    throw std::runtime_error("Could not start Moby");
  
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
  if(!environment)
    throw std::runtime_error("Could not find environment");
  
  // Apply uncertainty to robot model
  apply_manufacturing_uncertainty(argc, argv,robot);
  
  // Apply uncertainty to robot initial conditions
  apply_state_uncertainty(argc, argv,robot);
  
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
    stop_sim = !Moby::step(sim);
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
