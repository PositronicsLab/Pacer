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
#include "common.h"

#ifdef USE_OSG_DISPLAY
#include "visualize.cpp"
#endif

#include <boost/shared_ptr.hpp>

#include <sstream>

#include <Pacer/utilities.h>

#define DEBUG_OUTPUT
#ifndef DEBUG_OUTPUT
#define logging \
if (0) ; \
else std::cout
#else
#define logging \
if (0) ; \
else std::cout
#endif

#define SSTR( x ) (( std::ostringstream() << std::dec << x ) ).str()

using Moby::Simulator;
using Ravelin::RigidBodyd;
using Ravelin::Jointd;
using Ravelin::RCArticulatedBodyd;
using Ravelin::Pose3d;
using Ravelin::DynamicBodyd;
using boost::shared_ptr;


bool EXPORT_XML = false;
int SAMPLE_NUMBER = 0;

int CHILD_TO_PARENT_WRITE = 0;
int PARENT_TO_CHILD_READ = 0;
bool USE_PIPES = false;
int pid = 0;
double DURATION = 0;

using namespace Ravelin;

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
  ("stand", "put robot on ground")
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
    
    // Get robot positon
    Ravelin::VectorNd q;
    robot->get_generalized_coordinates_euler(q);
    int N_JOINT_DOFS = q.rows()-7;
    // update base position
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
   //    P_base.q = Ravelin::Quatd::rpy(position[3],position[4],position[5]);
   P_base.q = Ravelin::Quatd(position[3],position[4],position[5],position[6]);
   // apply changes
   robot->get_base_link()->set_pose(P_base);
   }
   // Update robot state
   robot->update_link_poses();
   */
  
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

  double lowest_point_z = 0;
  
  ///////// Project robot up to standing on plane //////////////
  if(vm.count("stand")){
  BOOST_FOREACH(shared_ptr<Ravelin::RigidBodyd> rbd, robot->get_links()){
    shared_ptr<Moby::RigidBody> rb = boost::dynamic_pointer_cast<Moby::RigidBody>(rbd);
    if(rb->is_end_effector()){ // radius of spherical link
      boost::shared_ptr<Ravelin::Pose3d> foot_pose(new Ravelin::Pose3d(*(rb->get_pose().get())));
      foot_pose->update_relative_pose(robot->get_base_link()->get_pose());
      
      // get foot geometry
      Moby::CollisionGeometryPtr foot_geometry;
      shared_ptr<Moby::SpherePrimitive> foot_primitive;

      BOOST_FOREACH(Moby::CollisionGeometryPtr cg, rb->geometries){
        Moby::PrimitivePtr primitive = cg->get_geometry();
        foot_primitive = boost::dynamic_pointer_cast<Moby::SpherePrimitive>(primitive);
        if(foot_primitive){
          foot_geometry = cg;
          break;
        }
      }
      
      Vector3d foot_radius(0,0,-foot_primitive->get_radius(),Moby::GLOBAL);
      Vector3d lowest_point = Ravelin::Pose3d::transform_vector(robot->get_base_link()->get_pose(),foot_radius);
      lowest_point += Vector3d(foot_pose->x,robot->get_base_link()->get_pose());
      lowest_point_z = std::min(lowest_point_z,lowest_point[2]);
    }
  }
  
  {
    logging << "moving robot to just above the ground: "<< lowest_point_z << std::endl;
    
    // Get robot velocity
    Ravelin::VectorNd q;
    robot->get_generalized_coordinates_euler(q);
    int N_JOINT_DOFS = q.rows()-7;
    // update base position
      q[N_JOINT_DOFS+2] = -lowest_point_z;
    
    // apply changes
    robot->set_generalized_coordinates_euler(q);
    logging << "Set coords to : " << q << std::endl;
  }
}
//  else {
//	  {
//    logging << "moving robot to not touch the ground: "<< lowest_point_z << std::endl;
//    
//    // Get robot velocity
//    Ravelin::VectorNd q;
//    robot->get_generalized_coordinates_euler(q);
//    int N_JOINT_DOFS = q.rows()-7;
//    // update base position
//      q[N_JOINT_DOFS+2] = 1;
//    
//    // apply changes
//    robot->set_generalized_coordinates_euler(q);
//    logging << "Set coords to : " << q << std::endl;
//  }
//}
  
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
  
  // Joints
  BOOST_FOREACH(shared_ptr<Jointd> jp, robot->get_joints()){
    std::string name,help;

    if (jp->num_dof() == 1) {
      name = std::string(jp->joint_id+".axis");
      help = std::string("Axis vector in model (base link) frame of REVOLUTE Joint: "+jp->joint_id);
      
      desc.add_options()
      (name.c_str(), po::value<std::vector<double> >()->multitoken() ,help.c_str());
    }
  }

  
  BOOST_FOREACH(shared_ptr<RigidBodyd> rb, robot->get_links()){
    std::string name,help;
    { // Density
      name = std::string(rb->body_id+".density");
      help = std::string("Density [kg/m^3] applied of link (default is solid polycarbonate): "+rb->body_id);
      
      desc.add_options()
      (name.c_str(), po::value<double>()->default_value(1200),help.c_str());
      
      name = std::string(rb->body_id+".mass");
      help = std::string("Mass [kg] applied to link: "+rb->body_id);
      
      desc.add_options()
      (name.c_str(), po::value<double>(),help.c_str());

    }
    
    if(rb->is_base()){
      name = std::string(rb->body_id+".size");
      help = std::string("Body box geometry dimensions (length, width, height) [m] of base link: " +rb->body_id);
      
      desc.add_options()
      (name.c_str(), po::value<std::vector<double> >()->multitoken() ,help.c_str());
    }
    
    {  // Length & radius of cylindrical link
      name = std::string(rb->body_id+".length");
      help = std::string("LENGTH [m] of link LENGTH dimension (along limb axis): "+rb->body_id);
      desc.add_options()
      (name.c_str(), po::value<double>()->default_value(0.1),help.c_str());
      
      name = std::string(rb->body_id+".radius");
      help = std::string("Radius [m] of link RADIUS dimension: "+rb->body_id);
      
      desc.add_options()
      (name.c_str(), po::value<double>()->default_value(0.005),help.c_str());
    }
    
    if(rb->is_end_effector()){ // radius of spherical link
      name = std::string(rb->body_id+".foot.radius");
      help = std::string("Radius [m] of foot RADIUS dimension: "+rb->body_id);
      
      desc.add_options()
      (name.c_str(), po::value<double>()->default_value(0.01),help.c_str());
      
      name = std::string(rb->body_id+".foot.density");
      help = std::string("Density [kg/m^3] applied to link's spherical end effector (default is solid polycarbonate): "+rb->body_id);
      
      desc.add_options()
      (name.c_str(), po::value<double>()->default_value(1200),help.c_str());
      
      name = std::string(rb->body_id+".foot.mass");
      help = std::string("Mass [kg] applied to link's spherical end effector: "+rb->body_id);
      
      desc.add_options()
      (name.c_str(), po::value<double>(),help.c_str());
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
  
//  shared_ptr<RigidBodyd> base_link_ptr = robot->get_base_link();
//  shared_ptr<const Pose3d> model_pose = base_link_ptr->get_pose();
  BOOST_FOREACH(shared_ptr<Jointd> jp, robot->get_joints()){
    shared_ptr<RevoluteJointd> rjp = boost::dynamic_pointer_cast<RevoluteJointd>(jp);
    if(vm.count(jp->joint_id+".axis")){
      std::vector<double> axis = vm[jp->joint_id+".axis"].as<std::vector<double> >();
      
      const Vector3d joint_axis = rjp->get_axis();
      logging << "joint_axis" << joint_axis << std::endl;
      Vector3d new_joint_axis(&axis[0],joint_axis.pose);
      new_joint_axis.normalize();
      logging << "new_joint_axis" << new_joint_axis << std::endl;
//      Vector3d new_joint_axis_inner_fame = Pose3d::transform_vector(joint_axis.pose,new_joint_axis);
//      logging << "new_joint_axis_inner_fame" << new_joint_axis_inner_fame << std::endl;
      rjp->set_axis(new_joint_axis);
      rjp->update_spatial_axes();
      robot->update_link_poses();
    }
  }
  
  
  // Perturb mass of robot link by scaling by parameter
  BOOST_FOREACH(shared_ptr<Ravelin::RigidBodyd> rbd, robot->get_links()){
    shared_ptr<Moby::RigidBody> rb = boost::dynamic_pointer_cast<Moby::RigidBody>(rbd);
    // Link Length Adjustment
//#ifdef USE_OSG_DISPLAY
//    osg::Group* no_viz_data = new osg::Group();
//    rb->set_visualization_data(no_viz_data);
//#endif
    
    bool LINK_IS_END_EFFECTOR = rb->is_end_effector();
    bool LINK_IS_BASE = rb->is_base();
    bool LINK_IS_LIMB = !LINK_IS_BASE;
    //////////////// LINK DATA /////////////////
    double mass     = -1;
    if (vm.count(rb->body_id+".mass")) mass = vm[rb->body_id+".mass"].as<double>();
    
    double density  =                 vm[rb->body_id+".density"].as<double>();
    double length   = (LINK_IS_LIMB)? vm[rb->body_id+".length"].as<double>() : 0;
    double radius   = (LINK_IS_LIMB)? vm[rb->body_id+".radius"].as<double>() : 0;
    
    //////////////// FOOT DATA /////////////////
    double foot_mass    = -1;
    if (vm.count(rb->body_id+".foot.mass")) foot_mass = vm[rb->body_id+".foot.mass"].as<double>();
    
    double foot_density = (LINK_IS_END_EFFECTOR)? vm[rb->body_id+".foot.density"].as<double>() : 0;
    double foot_radius  = (LINK_IS_END_EFFECTOR)? vm[rb->body_id+".foot.radius"].as<double>() : 0;
    Moby::CollisionGeometryPtr foot_geometry;
    shared_ptr<Moby::SpherePrimitive> foot_primitive;
    
    //    Moby::CollisionGeometryPtr limb_geometry;
    //    shared_ptr<Moby::CylinderPrimitive> limb_primitive;
    
    Moby::CollisionGeometryPtr base_geometry;
    shared_ptr<Moby::BoxPrimitive> base_primitive;
    
    if (LINK_IS_END_EFFECTOR) {
      BOOST_FOREACH(Moby::CollisionGeometryPtr cg, rb->geometries){
        Moby::PrimitivePtr primitive = cg->get_geometry();
        foot_primitive = boost::dynamic_pointer_cast<Moby::SpherePrimitive>(primitive);
        if(foot_primitive){
          foot_geometry = cg;
          break;
        }
      }
    }
    
    //    if(LINK_IS_LIMB){
    //      BOOST_FOREACH(Moby::CollisionGeometryPtr cg, rb->geometries){
    //        Moby::PrimitivePtr primitive = cg->get_geometry();
    //        limb_primitive = boost::dynamic_pointer_cast<Moby::CylinderPrimitive>(primitive);
    //        if(foot_primitive){
    //          limb_geometry = cg;
    //          break;
    //        }
    //      }
    //    }
    
    if (LINK_IS_BASE) {
      BOOST_FOREACH(Moby::CollisionGeometryPtr cg, rb->geometries){
        Moby::PrimitivePtr primitive = cg->get_geometry();
        base_primitive = boost::dynamic_pointer_cast<Moby::BoxPrimitive>(primitive);
        if(base_primitive){
          base_geometry = cg;
          break;
        }
      }
    }
    
    
    //////////////////////////  APPLY LINK LENGTH  /////////////////////////////
    boost::shared_ptr<const Ravelin::Pose3d> outer_pose;
    if (LINK_IS_LIMB){
      boost::shared_ptr<const Ravelin::Pose3d> inner_joint_pose = rb->get_inner_joint_explicit()->get_pose();
      if (LINK_IS_END_EFFECTOR) {
#ifndef NDEBUG
        logging << "End Effector link: " << rb->body_id << std::endl;
#endif
        // assumes no outer joint
        // pose of the outer joint
        outer_pose = foot_geometry->get_pose();
      } else {  // stretch the distance between 2 joints
#ifndef NDEBUG
        logging << "Interior link: " << rb->body_id << std::endl;
#endif
        
        // assumes one outer joint
        // pose of the outer joint
        const std::set<shared_ptr<Jointd> >& outers = rb->get_outer_joints();
        //sets inner and outer joints for the current link
        
        outer_pose = (*(outers.begin()))->get_pose();
      }
      
      boost::shared_ptr<Ravelin::Pose3d> outer_joint_wrt_inner(new Ravelin::Pose3d(outer_pose->q,outer_pose->x,outer_pose->rpose));
      
      // pose of the outer joint defined wrt inner joint
#ifndef NDEBUG
      logging << "Sample " << SAMPLE_NUMBER << " : Link "<< rb->body_id.c_str() <<" length = " << length << std::endl;
#endif
      
      outer_joint_wrt_inner->update_relative_pose(inner_joint_pose);
      logging << "Before: " << *outer_joint_wrt_inner << std::endl;
      // Update transform in link forward direction
      outer_joint_wrt_inner->x.normalize();
      outer_joint_wrt_inner->x *= length;
      logging << "After: " <<  *outer_joint_wrt_inner << std::endl;
      length = outer_joint_wrt_inner->x.norm();
      
      
      if (LINK_IS_END_EFFECTOR) {
        outer_joint_wrt_inner->update_relative_pose(outer_pose->rpose);
        foot_geometry->set_relative_pose(*outer_joint_wrt_inner);
      } else {
        const std::set<shared_ptr<Jointd> >& outers = rb->get_outer_joints();
        
        // Update transforms
        try {
          (*(outers.begin()))->Ravelin::Jointd::set_location(Ravelin::Vector3d(0,0,0,outer_joint_wrt_inner),(*(outers.begin()))->get_inboard_link(),(*(outers.begin()))->get_outboard_link());
        } catch (...) {}
        // Apply new transform to all children (recursive)
        // NOTE: Doesnt seem to be necessary
        //          apply_transform(T,rb->get_outer_joints());
        robot->update_link_poses();
      }
    }
    
    
    
    // Link Mass Adjustment
    {
      Ravelin::SpatialRBInertiad J_link(rb->get_inertia());
      if (LINK_IS_LIMB){ // Cylinder limb
                         // cylinder V = pi*h*r^2
        double volume = M_PI*length*sqr(radius);
        // m = d*V
        if (mass <= 0) {
          mass = density * volume;
        }
        // cylinder J =
        // (m*r^2)/2          0                   0
        //    0      (m/12)(3r^2 + h^2)           0
        //    0               0          (m/12)(3r^2 + h^2)
        double J[3] = {
          (mass*sqr(radius)*0.5),
          (mass/12.0)*(3.0*sqr(radius) + sqr(length)),
          (mass/12.0)*(3.0*sqr(radius) + sqr(length))};
        
        J_link.m = mass;
        J_link.J = Ravelin::Matrix3d(J[0],0,0,
                                     0,J[1],0,
                                     0,0,J[2]);
        
        if(LINK_IS_END_EFFECTOR){ // sphere foot
          foot_primitive->set_radius(foot_radius);
          
          // sphere V = 4/3 pi*r^3
          double volume = (4.0/3.0)*M_PI*(foot_radius*foot_radius*foot_radius);
          
          double mass = foot_mass;
          // m = d*V
          if (mass <= 0) {
            mass = density * volume;
          }
          
          // sphere J =
          // (2*r^2)/5     0         0
          //    0      (2*r^2)/5     0
          //    0          0     (2*r^2)/5
          double inertia = 2.0*mass*sqr(foot_radius) / 5.0;
          double J[3] = {inertia,inertia,inertia};
          
          logging << "Sample " << SAMPLE_NUMBER << " : Link (foot) "<< rb->body_id.c_str() << " mass = "<< mass ;

          Ravelin::SpatialRBInertiad J_foot;
          J_foot.m = mass;
          J_foot.J = Ravelin::Matrix3d(J[0],0,0,
                                       0,J[1],0,
                                       0,0,J[2]);
          
          //-- transform foot inertia to link inertial frame --//
          // foot pose
          shared_ptr<const Ravelin::Pose3d> foot_pose = foot_geometry->get_pose();
          Ravelin::Pose3d foot_relative_to_link(foot_pose->q,foot_pose->x,foot_pose->rpose);
          // foot pose relative to link
          foot_relative_to_link.update_relative_pose(J_link.pose);
          // transform foot to link
          Ravelin::Transform3d foot_link_transform(foot_relative_to_link.q,foot_relative_to_link.x);
          // transform inertia and add to link inertia
          Ravelin::SpatialRBInertiad J_foot_new = foot_link_transform.transform(J_foot);
          J_foot_new.pose = J_link.pose;
          J_link += J_foot_new;
        }

      } else if (LINK_IS_BASE) {
        if (vm.count(rb->body_id+".size")) {
          std::vector<double> size = vm[rb->body_id+".size"].as<std::vector<double> >();
          double x = size[0],
                 y = size[1],
                 z = size[2];
          
          // Adjust collision and weight affecting values
          base_primitive->set_size(x,y,z);
        }

        
        // Box V = x * y * z
        double x = base_primitive->get_x_len(),
               y = base_primitive->get_y_len(),
               z = base_primitive->get_z_len();
        
        const std::set<shared_ptr<Jointd> >& outers = rb->get_outer_joints();
        // Adjust kinematics
        BOOST_FOREACH( shared_ptr<Jointd> outer_joint, outers){
          boost::shared_ptr<const Ravelin::Pose3d> inner_joint_pose = rb->get_pose();
#ifndef NDEBUG
          logging << "Interior link: " << rb->body_id << std::endl;
#endif
          
          // assumes one outer joint
          // pose of the outer joint
          outer_pose = outer_joint->get_pose();
          
          boost::shared_ptr<Ravelin::Pose3d> outer_joint_wrt_inner(new Ravelin::Pose3d(outer_pose->q,outer_pose->x,outer_pose->rpose));
          
          // pose of the outer joint defined wrt inner joint
#ifndef NDEBUG
          logging << "Sample " << SAMPLE_NUMBER << " : Link "<< rb->body_id.c_str() <<" length = " << length << std::endl;
#endif
          
          outer_joint_wrt_inner->update_relative_pose(inner_joint_pose);
          logging << "Before: " << *outer_joint_wrt_inner << std::endl;
          // Update transform in link's quadrant
          Origin3d dir = outer_joint_wrt_inner->x;
          dir.normalize();
          outer_joint_wrt_inner->x = Origin3d(Utility::sign(dir[0])*x*0.5,Utility::sign(dir[1])*y*0.5,-z*0.5);
          logging << "After: " <<  *outer_joint_wrt_inner << std::endl;
          
          // Update transforms
          try{
            outer_joint->Ravelin::Jointd::set_location(Ravelin::Vector3d(0,0,0,outer_joint_wrt_inner),outer_joint->get_inboard_link(),outer_joint->get_outboard_link());
          } catch (...) {}
          // Apply new transform to all children (recursive)
          // NOTE: Doesnt seem to be necessary
          //          apply_transform(T,rb->get_outer_joints());
          robot->update_link_poses();
        }
        
        double volume = x*y*z;
        // m = d*V
        if (mass <= 0) {
          mass = density * volume;
        }
        // cylinder J =
        // (m*r^2)/2          0                   0
        //    0      (m/12)(3r^2 + h^2)           0
        //    0               0          (m/12)(3r^2 + h^2)
        double J[3] = {
          (mass/12.0)*(      y*y + z*z),
          (mass/12.0)*(x*x       + z*z),
          (mass/12.0)*(x*x + y*y      )};
        
        J_link.m = mass;
        J_link.J = Ravelin::Matrix3d(J[0],0,0,
                                     0,J[1],0,
                                     0,0,J[2]);
      }
      logging << "Sample " << SAMPLE_NUMBER << " : Link "<< rb->body_id.c_str() <<" mass = " << J_link.m << std::endl
        <<" inertia = " << J_link.J << std::endl;
      rb->set_inertia(J_link);
    }
  }
}

void parse_sample_options(int argc, char* argv[]){
  // Declare the supported options.
  po::options_description desc("Moby initialization options");
  desc.add_options()
  ("help", "produce help message")
  // INPUT BY EXPERIMENT
  ("xml,y","Output XML model file for robot");
  // INPUT BY USER
//  ("sample", po::value<int>(),"Sample Number");
 
  logging << "Parsing Variable Map from command line" << std::endl;
  
  po::variables_map vm;
  //  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::parsed_options parsed
  = po::command_line_parser(argc, argv).style(po::command_line_style::unix_style ^ po::command_line_style::allow_short).options(desc).allow_unregistered().run();
  po::store(parsed, vm);
  //  po::notify(vm);
  
  logging << "Parsed Variable Map from command line" << std::endl;
  
  
  // Get sample number for output
//  if (vm.count("sample")) {
//    SAMPLE_NUMBER = vm["sample"].as<int>();
//  } else {
//    fprintf(stdout,"Sample with PID: %d did not get a sample number", pid);
//    throw std::runtime_error("Sample did not get a sample number, likely parent quit or pipe was closed.  Exiting this simulation.");
//  }
  
  logging << "Sample with PID: "<< pid << " has sample number "<< SAMPLE_NUMBER << std::endl;
  
  if (vm.count("xml")) {
    EXPORT_XML = true;
  }
  
  if ( vm.count("help")  )
  {
    logging << "Available options: " << std::endl
    << desc << std::endl;
  }
  
  return;
}

void preload_simulation(int argc, char* argv[], shared_ptr<Simulator>& sim){
  // Declare the supported options.
  po::options_description desc("Moby initialization options (preload)");
  desc.add_options()
  ("help", "produce help message")
  ("stepsize,s", po::value<std::string>()->default_value("0.001"), "set step size (virtual time) of each iteration of the simulatior")
  ("display,r","visualize in moby")
  ("duration", po::value<std::string>()->default_value("0"),"Duration of simulation.");
//  ("sample", po::value<unsigned>(),"Sample Number");
  
  logging << "Parsing Variable Map from command line" << std::endl;
  
  po::variables_map vm;
  //  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::parsed_options parsed
  = po::command_line_parser(argc, argv).style(po::command_line_style::unix_style ^ po::command_line_style::allow_short).options(desc).allow_unregistered().run();
  po::store(parsed, vm);
  //  po::notify(vm);
  
  logging << "Parsed Variable Map from command line" << std::endl;
  
  logging << "Worker has PID: "<< pid << std::endl;
  
  if (vm.count("xml")) {
    EXPORT_XML = true;
  }
  
  if ( vm.count("help")  )
  {
    logging << "Available options: " << std::endl
    << desc << std::endl;
  }
  
  std::string pacer_interface_path(getenv ("PACER_SIMULATOR_PATH"));
  logging << "PACER_INTERFACE_PATH: " << pacer_interface_path << std::endl;
  
  std::string
  step_size = vm["stepsize"].as<std::string>(),
  duration = vm["duration"].as<std::string>();
  
  DURATION = atof(duration.c_str());
  
  /*
   *  Moby Initialization
   */
  logging << " -- Populating command line options -- " << std::endl;
  
  // run sample
  std::vector<std::string> argvs;
  argvs.push_back("montecarlo-moby-sample");
  // Max time is 0.3 seconds
  argvs.push_back("-s="+step_size);
  // XML output last frame
  //  argvs.push_back("-w=0");
  if (vm.count("display")) {
    argvs.push_back("-r");
  } else {
    //argvs.push_back("-y=osg");
    //    double capture_step = 0.01;
    //    int rate = capture_step / atof(step_size.c_str());
    //    rate = std::max(1,rate);
    //    argvs.push_back("-v="+SSTR(rate));
    //argvs.push_back("-v=0");
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
  
  // clean up argv
  //  for ( size_t i = 0 ; i < argvs.size() ; i++ ){
  //    delete[] moby_argv[i];
  //  }
  //  delete[] moby_argv;
  
  return;
}


void parse_command_line_options(int argc, char* argv[]){
  // Declare the supported options.
  po::options_description desc("Moby initialization options");
  desc.add_options()
  ("help", "produce help message")
  ("readpipe", po::value<int>()->default_value(0),"PARENT_TO_CHILD_READ")
  ("writepipe", po::value<int>()->default_value(0),"CHILD_TO_PARENT_WRITE")
  ("no-pipe","expect piped parameters?");
  
  logging << "Parsing Variable Map from command line" << std::endl;
  
  po::variables_map vm;
  //  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::parsed_options parsed
  = po::command_line_parser(argc, argv).style(po::command_line_style::unix_style ^ po::command_line_style::allow_short).options(desc).allow_unregistered().run();
  po::store(parsed, vm);
  //  po::notify(vm);
  
  logging << "Parsed Variable Map from command line" << std::endl;
  
  if (vm.count("no-pipe")) {
    USE_PIPES = false;
  } else {
    USE_PIPES = true;
    if (vm.count("readpipe")) {
      PARENT_TO_CHILD_READ = vm["readpipe"].as<int>();
      logging << "PID: "<<pid<<" input readpipe (PARENT_TO_CHILD_READ): " << PARENT_TO_CHILD_READ << std::endl;
    } else {
      fprintf(stderr,"Sample with PID: %d did not get a read pipe : PARENT_TO_CHILD_READ", pid);
    }
    
    if (vm.count("writepipe")) {
      CHILD_TO_PARENT_WRITE = vm["writepipe"].as<int>();
      logging << "PID: "<<pid<<" output writepipe (CHILD_TO_PARENT_WRITE): " << CHILD_TO_PARENT_WRITE << std::endl;
    } else {
      fprintf(stderr,"Sample with PID: %d did not get a write pipe : CHILD_TO_PARENT_WRITE", pid);
    }
  }
}



/////////////////////////////////////////////////////////////////
/// -------------------- MAIN FUNCTION -----------------------///
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>

int main(int argc_main, char* argv_main[]){
  logging << " -- Sample Started -- " << std::endl;
  
  // Cet this process's PID for debugging
  pid = getpid();
  
  parse_command_line_options(argc_main,argv_main);

  
  // Setup this instance of moby
  shared_ptr<Simulator> sim;
  
  logging << " -- Starting Simulator -- " << std::endl;
  preload_simulation(argc_main,argv_main, sim);
  logging << " -- Started Simulator -- " << std::endl;
  
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

simulation_start:

  std::string message_str;
  
  int argc_sample;
  char** argv_sample;
  if(USE_PIPES){
    std::cerr << " Sample with PID: "<< pid <<  " Started! waiting on piped parameters: "<< std::endl;
    
    char message[MESSAGE_SIZE];
    // Block on read until next simulation is scheduled
    if( read(PARENT_TO_CHILD_READ, message, sizeof(message)) == -1 ) {
      throw std::runtime_error("Could not read anything from pipe!");
    }
    
    std::cerr << "Sample with PID: "<< pid <<  " read message string: " << message << std::endl;
    
    std::vector<char*> messagev;
    messagev.push_back("filler");
    char* chars_array = strtok(message, " ");
    while(chars_array)
    {
      messagev.push_back(chars_array);
      chars_array = strtok(NULL, " ");
    }
    argc_sample = messagev.size();
    argv_sample = &messagev[0];
    
    logging << "Message vector: " << messagev << std::endl;
  } else {
    argc_sample = argc_main;
    argv_sample = argv_main;
  }
  // Setup this instance of moby
  logging << " -- parse_sample_options -- " << std::endl;
  parse_sample_options(argc_sample,argv_sample);
  logging << " -- parse_sample_options -- " << std::endl;
  sim->current_time = 0;
  
  //  if(!environment)
  //    throw std::runtime_error("Could not find environment");
  
  //  logging << " -- Found Environment -- " << std::endl;
  
//
  // Apply uncertainty to robot model
  apply_manufacturing_uncertainty(argc_sample,argv_sample,robot);
  
  if(EXPORT_XML){
    // write the file (fails silently)
    logging << " -- Exporting robot model file -- " << std::endl;
    
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
  logging << " -- Applying State Uncertainty -- " << std::endl;
  apply_state_uncertainty(argc_sample,argv_sample,robot);
  logging << " -- Applied State Uncertainty -- " << std::endl;

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
    
  while (!stop_sim &&  sim->current_time < DURATION) {
#ifdef USE_OSG_DISPLAY
    if (ITER % 10 == 0) {
      sim->update_visualization();
      // write the file (fails silently)
      char buffer[128];
      sprintf(buffer, "frame-%08llu-%d-%d.osg", ITER ,pid,SAMPLE_NUMBER);
      osgDB::writeNodeFile(*MAIN_GROUP, std::string(buffer));
    }
#endif
    //    logging << " -- Stepping simulation -- " << std::endl;
    // NOTE: Applied in Pacer -- for now
    // apply_control_uncertainty(argc_sample,argv_sample,robot);
    try {
      stop_sim = !Moby::step(sim);
    } catch (std::exception& e) {
      std::cerr << "There was an error that forced the simulation to stop: "<< e.what() << std::endl;
      stop_sim = true;
    }
//#ifndef NDEBUG
    std::cerr << "Simulation ("<< SAMPLE_NUMBER << ") at time: t = " << sim->current_time  << ", iteration: " << ITER <<  std::endl;
//#endif
    ITER++;
  }
  
  {
    sim->update_visualization();

    char buffer[128];
    sprintf(buffer, "last-%d-%d.osg",pid,SAMPLE_NUMBER);
    osgDB::writeNodeFile(*MAIN_GROUP, std::string(buffer));
  }
  
//#ifndef NDEBUG
  std::cerr << "Simulation ("<< SAMPLE_NUMBER << ") at time: t = " << sim->current_time  << ", iteration: " << ITER << " Ended!" << std::endl;
//#endif
  /*
   *  Collecting final data
   */
  Ravelin::VectorNd q,qd;
  robot->get_generalized_coordinates_euler(q);
  logging << "q2 = " << q << std::endl;
  
  std::ostringstream oss;
  oss << std::dec << sim->current_time ;
  for (int i=q.rows()-7; i<q.rows() ;i++) {
    oss << " " << q[i] ;
  }
    message_str = oss.str();
  
  std::cerr << "Sample: "<< SAMPLE_NUMBER << " with PID: "<< pid << " Ended! message: " << message_str << std::endl;
  if (USE_PIPES) {
    write(CHILD_TO_PARENT_WRITE, message_str.c_str(), message_str.length());
//    execv( argv_main[0] , argv_main );
    SAMPLE_NUMBER++;
    goto simulation_start;
  }
  
  // Quit like this because quitting normally leads to weird deconstructor errors.
  throw std::runtime_error("Sample exited!");
  
  return 0;
}
