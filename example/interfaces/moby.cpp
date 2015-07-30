/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Moby/ConstraintSimulator.h>
#include <Pacer/controller.h>
#include <random>
#include "Random.h"

using Pacer::Controller;

// pointer to the simulator
 boost::shared_ptr<Moby::ConstraintSimulator> sim;
// pointer to the articulated body in Moby
Moby::RCArticulatedBodyPtr abrobot;

// pointer to the Pacer controller
 boost::shared_ptr<Controller> robot_ptr;


//#define USE_DXL
#ifdef USE_DXL
#include <dxl/Dynamixel.h>
  DXL::Dynamixel * dxl_;
# define DEVICE_NAME "/dev/tty.usbserial-A9YL9ZZV"

#include <thread>
static Ravelin::VectorNd q_motors_data,qd_motors_data,u_motors_data;

std::mutex joint_data_mutex_;
static double FREQ = 500;

static void control_motor(){
  while(true){
    static Ravelin::VectorNd q_motors,qd_motors,u_motors;
    if(joint_data_mutex_.try_lock()){
      q_motors = q_motors_data;
      
      qd_motors = qd_motors_data;
      u_motors = u_motors_data;
      joint_data_mutex_.unlock();
    }
 
    std::cout << q_motors << std::endl;
    dxl_->set_state(std::vector<double>(q_motors.begin(),q_motors.end()),std::vector<double>(qd_motors.begin(),qd_motors.end()));
//    dxl_->set_torque(std::vector<double>(q_motors.begin(),q_motors.end()));
    sleep(1.0/FREQ);
  }
}
#endif

#ifdef USE_OSG_DISPLAY
void visualize_ray(   const Ravelin::Vector3d& point, const Ravelin::Vector3d& vec, const Ravelin::Vector3d& color, boost::shared_ptr<Moby::ConstraintSimulator> sim ) ;
void visualize_ray(   const Ravelin::Vector3d& point, const Ravelin::Vector3d& vec, const Ravelin::Vector3d& color,double point_radius, boost::shared_ptr<Moby::ConstraintSimulator> sim ) ;
void draw_pose(const Ravelin::Pose3d& pose, boost::shared_ptr<Moby::ConstraintSimulator> sim,double lightness = 1, double size=0.1);

void render( std::vector<Pacer::VisualizablePtr>& viz_vect){
   for (std::vector<boost::shared_ptr<Pacer::Visualizable> >::iterator it = viz_vect.begin() ; it != viz_vect.end(); ++it)
   {
    switch((*it)->eType){
    case Pacer::Visualizable::eRay:{
      Pacer::Ray * v = static_cast<Pacer::Ray*>((it)->get());
      visualize_ray(v->point1,v->point2,v->color,v->size,sim);
      break;
    }
    case Pacer::Visualizable::ePoint:{
      Pacer::Point * v = static_cast<Pacer::Point*>((it)->get());
      visualize_ray(v->point,v->point,v->color,v->size,sim);
      break;
    }
    case Pacer::Visualizable::ePose:{
      Pacer::Pose * v = static_cast<Pacer::Pose*>((it)->get());
      draw_pose(v->pose,sim,v->shade,v->size);
      break;
    }
    default:   std::cout << "UNKNOWN VISUAL: " << (*it)->eType << std::endl; break;
    }
   }
   viz_vect.clear();
}
#endif

// ============================================================================
 // ================================ CUSTOM FNS ================================
 // ============================================================================

 // adds perturbations to the robot for testing stability
 /*
 void apply_sim_perturbations(){
   static std::vector<Moby::JointPtr>& joints_ = robot_ptr->get_joints();
   int num_joints = joints_.size();

   static std::vector<double> workv_,
       &unknown_base_perturbation = robot_ptr->get_data("sim.unknown-base-perturbation",workv_),
       &known_base_perturbation = robot_ptr->get_data("sim.known-base-perturbation",workv_),
       &known_leading_force = robot_ptr->get_data("sim.known-leading-force",workv_);

   Ravelin::Vector3d lead(known_leading_force[3],
                          known_leading_force[4],
                          known_leading_force[5],
                          Moby::GLOBAL);
   OUTLOG(lead,"LEAD_g",logDEBUG);

   Ravelin::Vector3d point_on_robot(known_leading_force[0],
                                    known_leading_force[1],
                                    known_leading_force[2],
                                    robot_ptr->get_base_link_frame());
                                    
   boost::shared_ptr<Ravelin::Pose3d> lead_transform =
       boost::shared_ptr<Ravelin::Pose3d>(new Ravelin::Pose3d(
         Ravelin::Quatd::identity(),
         Ravelin::Origin3d(
           Ravelin::Pose3d::transform_point(Moby::GLOBAL,point_on_robot)
         )
       ));
       
   Ravelin::SForced lead_force = Ravelin::SForced(0,0,0,0,0,0,Moby::GLOBAL);
   if(lead.norm() > Moby::NEAR_ZERO)
     lead_force = Ravelin::Pose3d::transform(
                                              Moby::GLOBAL,
                                              Ravelin::SForced(lead,Ravelin::Vector3d(0,0,0),lead_transform)
                                            );
   

   Ravelin::SForced known_force(&known_leading_force[0],Moby::GLOBAL);

   OUTLOG(lead,"LEAD_bt",logDEBUG);

   Ravelin::VectorNd perturbation(num_joints+6);
   perturbation.set_zero();

   for(int i=0;i<6;i++){
     perturbation[num_joints+i] += ( unknown_base_perturbation[i]
                                    + known_base_perturbation[i]
                                    + lead_force[i]);
   }

   abrobot->add_generalized_force(perturbation);
 }

 Ravelin::VectorNd& remap_values(const std::map<int,int>& value_map,const Ravelin::VectorNd v1,Ravelin::VectorNd& v2,bool reverse = false){
   v2 = v1;
   for(int i=0;i<value_map.size();i++){
     if(reverse)
       v2[i] = v1[value_map.at(i)];
     else
       v2[value_map.at(i)] = v1[i];
   }
    return v2;
 }
*/

 // ============================================================================
 // ================================ CONTROLLER ================================
 // ============================================================================

// implements a controller callback for Moby
void controller_callback(Moby::DynamicBodyPtr dbp, double t, void*)
{

  int num_joint_dof = 0;
  static std::vector<Moby::JointPtr> joints = abrobot->get_joints();
  static std::map<std::string, Moby::JointPtr> joints_map;
  if (joints_map.empty()) {
    for (std::vector<Moby::JointPtr>::iterator it = joints.begin(); it != joints.end(); it++)
      joints_map[(*it)->id] = (*it);
  }

  static double last_time = -0.001;
  double dt = t - last_time;
 
  if(dt > 1e-4){
  robot_ptr->reset_state();
  
  last_time = t;

  Ravelin::VectorNd generalized_q,generalized_qd,generalized_qdd, generalized_fext;

  {
    abrobot->get_generalized_coordinates(Moby::DynamicBody::eEuler,generalized_q);
    abrobot->get_generalized_velocity(Moby::DynamicBody::eSpatial,generalized_qd);
    abrobot->get_generalized_forces(generalized_fext);
  }
  
  {
    // Re-map state simulation->robot joints
    std::map<std::string, Ravelin::VectorNd> q,qd,fext;
    Ravelin::VectorNd q_joints,qd_joints,fext_joints;
    for (int i=0;i<joints.size();i++){
      q[joints[i]->id] = joints[i]->q;
      qd[joints[i]->id] = joints[i]->qd;
      Ravelin::VectorNd& dofs
        = (fext[joints[i]->id] = Ravelin::VectorNd(joints[i]->num_dof()));
      for (int j=0;j<joints[i]->num_dof();j++)
        dofs[j] = generalized_fext[joints[i]->get_coord_index()+j];
    }
    
    robot_ptr->convert_to_generalized(q,q_joints);
    robot_ptr->convert_to_generalized(qd,qd_joints);
    robot_ptr->convert_to_generalized(fext,fext_joints);
    
    generalized_q.set_sub_vec(0,q_joints);
    generalized_qd.set_sub_vec(0,qd_joints);
    generalized_fext.set_sub_vec(0,fext_joints);

    num_joint_dof = q_joints.size();
  }

  static std::vector<std::string> noise_variables,joint_names;
  bool apply_noise = robot_ptr->get_data< std::vector<std::string> >("noise.variables",noise_variables);
  
  if(joint_names.empty())
    robot_ptr->get_data< std::vector<std::string> >("init.joint.id",joint_names);
  
  static
  std::map< std::string , std::vector< boost::shared_ptr<Generator> > >
    noise_generator;
  
  if (apply_noise) {
    if(noise_generator.empty()){
      for (std::vector<std::string>::iterator it = noise_variables.begin(); it != noise_variables.end(); it++) {
        std::string& name = (*it);

        std::vector<double> mu,sigma,xmin, xmax;

        bool use_mu_sigma = true;
        if(!robot_ptr->get_data<std::vector<double> >("noise."+name+".mu",mu))
          use_mu_sigma = false;
        if(!robot_ptr->get_data<std::vector<double> >("noise."+name+".sigma",sigma))
          use_mu_sigma = false;

        robot_ptr->get_data<std::vector<double> >("noise."+name+".min",xmin);
        robot_ptr->get_data<std::vector<double> >("noise."+name+".max",xmax);
        
        std::vector< boost::shared_ptr<Generator> > noise_vec;
        for (int i=0; i<xmin.size(); i++) {
          if(use_mu_sigma)
            noise_vec.push_back(boost::shared_ptr<Generator>(new Generator(mu[i],sigma[i],xmin[i],xmax[i])));
          else
            noise_vec.push_back(boost::shared_ptr<Generator>(new Generator(xmin[i],xmax[i])));
        }
        noise_generator[name] = noise_vec;
      }
    }
   
    for (std::vector<std::string>::iterator it = noise_variables.begin(); it != noise_variables.end(); it++) {
      std::string& name = (*it);
      std::vector< boost::shared_ptr<Generator> >& generator = noise_generator[name];
      Ravelin::VectorNd noise_vector = Ravelin::VectorNd(generator.size());
      for (int i=0; i<generator.size(); i++) {
        noise_vector[i] = generator[i]->generate();
      }
      
      OUTLOG(noise_vector,name+"_perturbation",logERROR);
      
      if (name.compare("q") == 0) {
        assert(generator.size() == num_joint_dof);
        generalized_q.segment(0,num_joint_dof) += noise_vector;
      } else if (name.compare("qd") == 0) {
        assert(generator.size() == num_joint_dof);
        generalized_qd.segment(0,num_joint_dof) += noise_vector;
      } else if (name.compare("position") == 0) {
        generalized_q.segment(num_joint_dof,num_joint_dof+6) += noise_vector;
      } else if (name.compare("velocity") == 0) {
        generalized_q.segment(num_joint_dof,num_joint_dof+6) += noise_vector;
      } else if ("u") {
        for (int i = 0; i<noise_vector.rows(); i++) {
          Ravelin::VectorNd U(1);
          U[0] = noise_vector[i];
          joints_map[joint_names[i]]->add_force(U);
        }
      }
    }
  }
  
  static Ravelin::VectorNd  generalized_qd_last = generalized_qd;
  //NOTE: Pre-contact accel abrobot->get_generalized_acceleration(Moby::DynamicBody::eSpatial,generalized_qdd);
  ((generalized_qdd = generalized_qd) -= generalized_qd_last) /= dt;
  generalized_qd_last = generalized_qd;
  
  robot_ptr->set_generalized_value(Pacer::Robot::position,generalized_q);
  robot_ptr->set_generalized_value(Pacer::Robot::velocity,generalized_qd);
  robot_ptr->set_generalized_value(Pacer::Robot::acceleration,generalized_qdd);
  robot_ptr->set_generalized_value(Pacer::Robot::load,generalized_fext);

  robot_ptr->control(t);

  }
#ifdef USE_OSG_DISPLAY
  render(Utility::visualize);
#endif

  { 
    std::map<std::string, Ravelin::VectorNd > q, qd, u; 
    robot_ptr->get_joint_value(Pacer::Robot::position_goal, q);
    robot_ptr->get_joint_value(Pacer::Robot::velocity_goal, qd);
    robot_ptr->get_joint_value(Pacer::Robot::load_goal, u);
    
    if(!abrobot->get_kinematic()){
      for(int i=0;i<joints.size();i++){
        //std::cout << joints[i]->id << " = "  << u[joints[i]->id] << std::endl;
        joints[i]->add_force(u[joints[i]->id]);
      }
    } else {
      for(int i=0;i<joints.size();i++){
        joints[i]->q = q[joints[i]->id];
        joints[i]->qd = qd[joints[i]->id];
      }
      abrobot->update_link_poses();
    }
  }
#ifdef USE_DXL
  if(joint_data_mutex_.try_lock()){
    for(int i=0;i<dxl_->ids.size();i++)
      qd_motors_data[i] = 0;//robot_ptr->qd_joints[dxl_->JointName(i)];

    std::map<std::string,Ravelin::VectorNd> joint_val_map;
    robot_ptr->get_joint_value(Pacer::Robot::position_goal,joint_val_map);

    for(int i=0;i<dxl_->ids.size();i++)
      q_motors_data[i] = joint_val_map[dxl_->JointName(i)][0];

    //for(int i=0;i<dxl_->ids.size();i++)
    //  u_motors_data[i] = robot_ptr->get_joint_value(Pacer::Robot::load_goal,dxl_->JointName(i),0);
    joint_data_mutex_.unlock();
  }
  static std::thread motor_thread(control_motor);
#endif
  
}

// ============================================================================
// ================================ CALLBACKS =================================
// ============================================================================

// examines contact events (after they have been handled in Moby)
void post_event_callback_fn(const std::vector<Moby::UnilateralConstraint>& e,
                            boost::shared_ptr<void> empty)
{
  // PROCESS CONTACTS
  for(unsigned i=0;i<e.size();i++){
    if (e[i].constraint_type == Moby::UnilateralConstraint::eContact)
    {
      Moby::SingleBodyPtr sb1 = e[i].contact_geom1->get_single_body();
      Moby::SingleBodyPtr sb2 = e[i].contact_geom2->get_single_body();
      
      Ravelin::Vector3d 
        normal = e[i].contact_normal,         
        impulse = e[i].contact_impulse.get_linear();
      impulse.pose = e[i].contact_point.pose;
     
      bool compliant = 
        (e[i].compliance == Moby::UnilateralConstraint::eCompliant)? true : false;

      
      OUT_LOG(logERROR) << "compliant: " << compliant;
      if(robot_ptr->is_end_effector(sb1->id)){
        robot_ptr->add_contact(sb1->id,e[i].contact_point,normal,impulse,e[i].contact_mu_coulomb,e[i].contact_mu_viscous,0,compliant);
      } else if(robot_ptr->is_end_effector(sb2->id)){
        robot_ptr->add_contact(sb2->id,e[i].contact_point,-normal,-impulse,e[i].contact_mu_coulomb,e[i].contact_mu_viscous,0,compliant);
      } else {
        continue;  // Contact doesn't include an end-effector  
      }
      
#ifdef USE_OSG_DISPLAY
      visualize_ray(  e[i].contact_point,
                      e[i].contact_point + impulse*10.0,
                      Ravelin::Vector3d(1,0.5,0),
                      0.1,
                      sim
                    );
      visualize_ray(  e[i].contact_point,
                      e[i].contact_point + normal*0.1,
                      Ravelin::Vector3d(1,1,0),
                      0.1,
                      sim
                    );
  #endif
    }
  }
}

//#define SET_CONTACT_PARAMS

#ifdef SET_CONTACT_PARAMS
//# define RANDOM_FRICTION
# define LOW_FRICTION
#endif
// sets friction parameters for the feet randomly (when used)
boost::shared_ptr<Moby::ContactParameters> get_contact_parameters(Moby::CollisionGeometryPtr geom1, Moby::CollisionGeometryPtr geom2){

  boost::shared_ptr<Moby::ContactParameters> e = boost::shared_ptr<Moby::ContactParameters>(new Moby::ContactParameters());

#ifdef RANDOM_FRICTION
  static std::default_random_engine generator;
  static std::uniform_real_distribution<double> distribution(0.1,1.4);

  e->mu_coulomb = distribution(generator);
#endif
  
#ifdef LOW_FRICTION
  Moby::SingleBodyPtr sb1 = geom1->get_single_body();
  Moby::SingleBodyPtr sb2 = geom2->get_single_body();
  
  Ravelin::Vector3d point(0,0,0);
  if(robot_ptr->is_end_effector(sb1->id)){
    point = Ravelin::Pose3d::transform_point(Moby::GLOBAL,Ravelin::Vector3d(0,0,0,sb1->get_pose()));
  } else if(robot_ptr->is_end_effector(sb2->id)){
    point = Ravelin::Pose3d::transform_point(Moby::GLOBAL,Ravelin::Vector3d(0,0,0,sb2->get_pose()));
  }
  
  if (point[0] > 1.0)
    e->mu_coulomb = 0.2;
  else
    e->mu_coulomb = 1000;

#endif
  return e;
}

// hooks into Moby's post integration step callback function
void post_step_callback_fn(Moby::Simulator* s){}

/// Event callback function for setting friction vars pre-event
void pre_event_callback_fn(std::vector<Moby::UnilateralConstraint>& e, boost::shared_ptr<void> empty){
  for(int i=0;i< e.size();i++){
    OUT_LOG(logDEBUG1) << e[i] << std::endl;
  }
}

// ============================================================================
// ================================ INIT ======================================
// ============================================================================


//boost::shared_ptr<Moby::ContactParameters> get_contact_parameters(Moby::CollisionGeometryPtr geom1, Moby::CollisionGeometryPtr geom2);
//void post_event_callback_fn(const std::vector<Moby::UnilateralConstraint>& e, boost::shared_ptr<void> empty);
//void pre_event_callback_fn(std::vector<Moby::UnilateralConstraint>& e, boost::shared_ptr<void> empty);

/// plugin must be "extern C"

// this is called by Moby for a plugin
void init_cpp(const std::map<std::string, Moby::BasePtr>& read_map, double time){
 boost::shared_ptr<Moby::ConstraintSimulator> esim;
  std::cout << "STARTING MOBY PLUGIN" << std::endl;
#ifdef USE_DXL
  // If use robot is active also init dynamixel controllers
  dxl_ = new DXL::Dynamixel(DEVICE_NAME);
  // LINKS robot

  // Set Dynamixel Names
  std::vector<std::string> dxl_name = boost::assign::list_of
     ("LF_X_1")("RF_X_1")("LH_X_1")("RH_X_1")
     ("LF_Y_2")("RF_Y_2")("LH_Y_2")("RH_Y_2")
     ("LF_Y_3")("RF_Y_3")("LH_Y_3")("RH_Y_3");

  dxl_->names = dxl_name;
  // Set Joint Angles
  std::vector<int> dxl_tare = boost::assign::list_of
      (0)(0)(0)(0)
      (M_PI/4 * RX_24F_RAD2UNIT)(-M_PI/4 * RX_24F_RAD2UNIT)(-M_PI/4 * MX_64R_RAD2UNIT+40)(M_PI/4 * MX_64R_RAD2UNIT+250)
      (M_PI/2 * RX_24F_RAD2UNIT)(-M_PI/2 * RX_24F_RAD2UNIT)(-M_PI/2 * RX_24F_RAD2UNIT)(M_PI/2 * RX_24F_RAD2UNIT);

  dxl_->tare = dxl_tare;

  // Set Dynamixel Type
  std::vector<DXL::Dynamixel::Type> dxl_type = boost::assign::list_of
    (DXL::Dynamixel::RX_24F)(DXL::Dynamixel::RX_24F)(DXL::Dynamixel::RX_24F)(DXL::Dynamixel::RX_24F)
    (DXL::Dynamixel::RX_24F)(DXL::Dynamixel::RX_24F)(DXL::Dynamixel::MX_64R)(DXL::Dynamixel::MX_64R)
    (DXL::Dynamixel::RX_24F)(DXL::Dynamixel::RX_24F)(DXL::Dynamixel::RX_24F)(DXL::Dynamixel::RX_24F);

  dxl_->stype = dxl_type;

  for(int i=1;i<=dxl_->names.size();i++){
    dxl_->ids.push_back(i);
  }
    q_motors_data.set_zero(dxl_->ids.size());
    qd_motors_data.set_zero(dxl_->ids.size());
    u_motors_data.set_zero(dxl_->ids.size());
    
    dxl_->relaxed(false);

  joint_data_mutex_.unlock();
#endif
  
  // If use robot is active also init dynamixel controllers
  // get a reference to the ConstraintSimulator instance
  for (std::map<std::string, Moby::BasePtr>::const_iterator i = read_map.begin();
       i !=read_map.end(); i++)
  {
    // Find the simulator reference
    
    if (!sim)
      sim = boost::dynamic_pointer_cast<Moby::ConstraintSimulator>(i->second);

    // find the robot reference
    if (!abrobot)
    {
      abrobot = boost::dynamic_pointer_cast<Moby::RCArticulatedBody>(i->second);
    }
  }
    
//  esim = boost::dynamic_pointer_cast<Moby::EventDrivenSimulator>(sim);

  /// Set up quadruped robot, linking data from moby's articulated body
  /// to the quadruped model used by Control-Moby

  std::cout << "STARTING ROBOT" << std::endl;

  robot_ptr = boost::shared_ptr<Controller>(new Controller());
  robot_ptr->init();
  std::cout << "ROBOT INITED" << std::endl;

  // CONTACT PARAMETER CALLBACK (MUST BE SET)
#ifdef SET_CONTACT_PARAMS
 sim->get_contact_parameters_callback_fn = &get_contact_parameters;
#endif
  // CONTACT CALLBACK
//  if (esim){
//  //  sim->constraint_callback_fn             = &pre_event_callback_fn;
//    esim->constraint_post_callback_fn        = &post_event_callback_fn;
//  }
  // CONTROLLER CALLBACK
  abrobot->controller                     = &controller_callback;

  // ================= INIT ROBOT STATE ==========================
  int is_kinematic = robot_ptr->get_data<int>("init.kinematic");

  if(is_kinematic)
    abrobot->set_kinematic(true);

  std::map<std::string,Ravelin::VectorNd > q_start, qd_start;
  robot_ptr->get_joint_value(Pacer::Robot::position,q_start);
  robot_ptr->get_joint_value(Pacer::Robot::velocity,qd_start);

  Ravelin::VectorNd base_x, base_xd;
  robot_ptr->get_generalized_value(Pacer::Robot::position,base_x);
  robot_ptr->get_generalized_value(Pacer::Robot::velocity,base_xd);
  
  abrobot->set_generalized_coordinates(Moby::DynamicBody::eEuler,base_x);
  abrobot->set_generalized_velocity(Moby::DynamicBody::eSpatial,base_xd);
  abrobot->update_link_poses();

  std::vector<Moby::JointPtr> joints = abrobot->get_joints();

  for(int i=0;i<joints.size();i++){
    joints[i]->q = q_start[joints[i]->id];
    joints[i]->qd = qd_start[joints[i]->id];
  }
  
  abrobot->update_link_poses();
}

// plugins must be declared 'extern "C"'
extern "C" {

void init(void* separator, const std::map<std::string, Moby::BasePtr>& read_map, double time)
{
  init_cpp(read_map,time);
}
} // end extern C


#ifdef USE_OSG_DISPLAY
///////////////////////////////////////////////////////////////////////////////
/////////////////////////////// Visualization /////////////////////////////////

#include <boost/shared_ptr.hpp>
#include <Moby/ConstraintSimulator.h>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>
#include <osg/Plane>
#include <osg/LineSegment>
#include <osg/LineWidth>

using namespace Moby;
using namespace Ravelin;
const double VIBRANCY = 1;

/// Draws a ray directed from a contact point along the contact normal
void visualize_ray( const Ravelin::Vector3d& point, const Ravelin::Vector3d& vec, const Ravelin::Vector3d& c,double point_radius, boost::shared_ptr<ConstraintSimulator> sim ) {

  // random color for this contact visualization
  double r = c[0] * VIBRANCY;
  double g = c[1] * VIBRANCY;
  double b = c[2] * VIBRANCY;
  osg::Vec4 color = osg::Vec4( r, g, b, 1.0 );

  const double point_scale = point_radius;

  // the osg node this event visualization will attach to
  osg::Group* group_root = new osg::Group();

  // turn off lighting for this node
  osg::StateSet *point_state = group_root->getOrCreateStateSet();
  point_state->setMode( GL_LIGHTING, osg::StateAttribute::PROTECTED | osg::StateAttribute::OFF );

  // a geode for the visualization geometry
  osg::Geode* point_geode = new osg::Geode();

  // add some hints to reduce the polygonal complexity of the visualization
  osg::TessellationHints *hints = new osg::TessellationHints();
  hints->setTessellationMode( osg::TessellationHints::USE_TARGET_NUM_FACES );
  hints->setCreateNormals( true );
  hints->setDetailRatio( 0.001 );

  osg::Sphere* point_geometry = new osg::Sphere( osg::Vec3( 0,0,0 ), point_radius );
  osg::ShapeDrawable* point_shape = new osg::ShapeDrawable( point_geometry, hints );
  point_shape->setColor( color );
  point_geode->addDrawable( point_shape );

  osg::PositionAttitudeTransform* point_transform;
  point_transform = new osg::PositionAttitudeTransform();
  point_transform->setPosition( osg::Vec3( point[0], point[1], point[2] ) );
  point_transform->setScale( osg::Vec3( point_scale, point_scale, point_scale ) );

  // add the geode to the transform
  point_transform->addChild( point_geode );

  // add the transform to the root
  group_root->addChild( point_transform );

  // add the root to the transient data scene graph
  sim->add_transient_vdata( group_root );

  // ----- LINE -------

//  osg::Group* vec_root = new osg::Group();
  osg::Geode* vec_geode = new osg::Geode();
  osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
  osg::ref_ptr<osg::DrawArrays> drawArrayLines =
      new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP);

  osg::ref_ptr<osg::Vec3Array> vertexData = new osg::Vec3Array;

  geom->addPrimitiveSet(drawArrayLines);
  geom->setVertexArray(vertexData);

  //loop through points
  vertexData->push_back(osg::Vec3d(point[0],point[1],point[2]));
  vertexData->push_back(osg::Vec3d(vec[0],vec[1],vec[2]));

  drawArrayLines->setFirst(0);
  drawArrayLines->setCount(vertexData->size());

  // Add the Geometry (Drawable) to a Geode and return the Geode.
  vec_geode->addDrawable( geom.get() );
  // the osg node this event visualization will attach to

  // add the root to the transient data scene graph
  // create the visualization transform
  osg::PositionAttitudeTransform* vec_transform;
  vec_transform = new osg::PositionAttitudeTransform();

  // add the geode to the transform
  vec_transform->addChild( vec_geode );

  // add the transform to the root
  group_root->addChild( vec_transform );

  // add the root to the transient data scene graph
  sim->add_transient_vdata( group_root );
}

void visualize_ray( const Ravelin::Vector3d& point, const Ravelin::Vector3d& vec, const Ravelin::Vector3d& c, boost::shared_ptr<ConstraintSimulator> sim ) {
  visualize_ray(point,vec,c,0.1,sim);
}

void draw_pose(const Ravelin::Pose3d& p, boost::shared_ptr<ConstraintSimulator> sim ,double lightness, double size){
  Ravelin::Pose3d pose(p);
  assert(lightness >= 0.0 && lightness <= 2.0);
  pose.update_relative_pose(Moby::GLOBAL);
  Ravelin::Matrix3d Rot(pose.q);
  Rot*= 0.3;
  double alpha = (lightness > 1.0)? 1.0 : lightness,
         beta = (lightness > 1.0)? lightness-1.0 : 0.0;

  visualize_ray(pose.x+Ravelin::Vector3d(Rot(0,0),Rot(1,0),Rot(2,0),Moby::GLOBAL)*size,Ravelin::Vector3d(0,0,0)+pose.x,Ravelin::Vector3d(alpha,beta,beta),size,sim);
  visualize_ray(pose.x+Ravelin::Vector3d(Rot(0,1),Rot(1,1),Rot(2,1),Moby::GLOBAL)*size,Ravelin::Vector3d(0,0,0)+pose.x,Ravelin::Vector3d(beta,alpha,beta),size,sim);
  visualize_ray(pose.x+Ravelin::Vector3d(Rot(0,2),Rot(1,2),Rot(2,2),Moby::GLOBAL)*size,Ravelin::Vector3d(0,0,0)+pose.x,Ravelin::Vector3d(beta,beta,alpha),size,sim);
}
#endif // USE_OSG_DISPLAY
