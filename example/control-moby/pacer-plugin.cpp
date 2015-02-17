/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/controller.h>

using Pacer::Controller;
using Pacer::Robot;
using Pacer::EndEffector;

 boost::shared_ptr<Moby::EventDrivenSimulator> sim;
 boost::shared_ptr<Controller> robot_ptr;
 Moby::RCArticulatedBodyPtr abrobot;

#ifdef DRIVE_ROBOT
extern void controller(double time,
                       const Ravelin::VectorNd& q,
                       const Ravelin::VectorNd& qd,
                       Ravelin::VectorNd& command,
                       Ravelin::Pose3d& pose,
                       Ravelin::VectorNd& params);
#endif

#ifdef USE_OSG_DISPLAY
extern std::vector<boost::shared_ptr<Pacer::Visualizable> > visualize;
void visualize_ray(   const Ravelin::Vector3d& point, const Ravelin::Vector3d& vec, const Ravelin::Vector3d& color, boost::shared_ptr<Moby::EventDrivenSimulator> sim ) ;
void visualize_ray(   const Ravelin::Vector3d& point, const Ravelin::Vector3d& vec, const Ravelin::Vector3d& color,double point_radius, boost::shared_ptr<Moby::EventDrivenSimulator> sim ) ;
void draw_pose(const Ravelin::Pose3d& pose, boost::shared_ptr<Moby::EventDrivenSimulator> sim,double lightness = 1);



void render( std::vector<Pacer::VisualizablePtr> viz_vect){
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
      draw_pose(v->pose,sim,v->shade);
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

 void apply_sim_perturbations(){
   static std::vector<Moby::JointPtr>& joints_ = robot_ptr->get_joints();
   int num_joints = joints_.size();

   static std::vector<double> workv_,
       &unknown_base_perturbation = Utility::get_variable("sim.unknown-base-perturbation",workv_),
       &known_base_perturbation = Utility::get_variable("sim.known-base-perturbation",workv_),
       &known_leading_force = Utility::get_variable("sim.known-leading-force",workv_);

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
   robot_ptr->set_leading_force(lead_force);

   Ravelin::SForced known_force(&known_leading_force[0],Moby::GLOBAL);
   robot_ptr->set_known_force(known_force);

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

 // ============================================================================
 // ================================ CONTROLLER ================================
 // ============================================================================

void controller_callback(Moby::DynamicBodyPtr dbp, double t, void*)
{
  std::vector<Moby::JointPtr> joints_quad = robot_ptr->get_joints();
  std::vector<Moby::JointPtr> joints = abrobot->get_joints();

  std::map<int,int> joint_map;
  if(joint_map.empty())
    for(int i=0;i<joints.size();i++){
      if(joints[i]->num_dof() == 0) continue;
      for(int j=0;j<joints_quad.size();j++){
        if(!joints_quad[j]) continue;
        if(joints_quad[j]->num_dof() == 0) continue;
        if(joints_quad[j]->id.compare(joints[i]->id) == 0){
          for(int d = 0;d<joints_quad[j]->num_dof();d++)
            joint_map[joints[i]->get_coord_index()+d] = joints_quad[j]->get_coord_index()+d;
        }
      }
    }

  Ravelin::VectorNd generalized_q,generalized_qd,generalized_qdd, generalized_fext;
  int num_joints = joints_quad.size();
  static double last_time = -0.001;
  double dt = t - last_time;
  last_time = t;

  // Re-map state simulation->robot joints
  {
    abrobot->get_generalized_coordinates(Moby::DynamicBody::eEuler,generalized_q);
    generalized_q = remap_values(joint_map,generalized_q,workv_);
    abrobot->get_generalized_velocity(Moby::DynamicBody::eSpatial,generalized_qd);
    generalized_qd = remap_values(joint_map,generalized_qd,workv_);
    abrobot->get_generalized_forces(generalized_fext);
    generalized_fext = remap_values(joint_map,generalized_fext,workv_);
  }

  static Ravelin::VectorNd  generalized_qd_last = generalized_qd;
  //NOTE: Pre-contact    abrobot->get_generalized_acceleration(Moby::DynamicBody::eSpatial,generalized_qdd);
  ((generalized_qdd = generalized_qd) -= generalized_qd_last) /= dt;
  generalized_qd_last = generalized_qd;


  Ravelin::VectorNd q_des(num_joints),
                    qd_des(num_joints),
                    qdd_des(num_joints);
  Ravelin::VectorNd u(num_joints);

#ifdef USE_OSG_DISPLAY
  //visualize.clear();
#endif


#ifdef DRIVE_ROBOT
  controller(t,generalized_q,generalized_qd,robot_ptr->movement_command,*robot_ptr->gait_pose.get(),robot_ptr->gait_params);
#endif

#ifdef USE_OSG_DISPLAY
  render(visualize);
#endif

  robot_ptr->control(t,generalized_q,generalized_qd,generalized_qdd,generalized_fext,q_des,qd_des,qdd_des,u);
  // Re-map goals robot->simulation joints
  {
    q_des = remap_values(joint_map,q_des,workv_,true);
    qd_des = remap_values(joint_map,qd_des,workv_,true);
    qdd_des = remap_values(joint_map,qdd_des,workv_,true);
    u = remap_values(joint_map,u,workv_,true);
  }

  if(!abrobot->get_kinematic()){
    for(int i=0;i<joints.size();i++){
      Ravelin::VectorNd U(joints[i]->num_dof());
      for(int j=0;j<joints[i]->num_dof();j++)
        U[j] = u[joints[i]->get_coord_index()+j];
      joints[i]->add_force(U);
    }
  } else {
    for(int i=0;i<joints.size();i++){
      Ravelin::VectorNd Q(joints[i]->num_dof());
      Ravelin::VectorNd QD(joints[i]->num_dof());
      for(int j=0;j<joints[i]->num_dof();j++){
        Q[j] = q_des[joints[i]->get_coord_index()+j];
        QD[j] = qd_des[joints[i]->get_coord_index()+j];
      }
      joints[i]->q = Q;
      joints[i]->qd = QD;
    }
    abrobot->update_link_poses();
  }
}

// ============================================================================
// ================================ CALLBACKS =================================
// ============================================================================

void post_event_callback_fn(const std::vector<Moby::UnilateralConstraint>& e,
                            boost::shared_ptr<void> empty)
{

  std::vector<std::string>& eef_names_ = robot_ptr->get_end_effector_names();
  std::vector<EndEffector>& eefs_ = robot_ptr->get_end_effectors();
  robot_ptr->reset_contact();
  // PROCESS CONTACTS
  for(unsigned i=0;i<e.size();i++){
    if (e[i].constraint_type == Moby::UnilateralConstraint::eContact)
    {
      bool MIRROR_FLAG = false;

      Moby::SingleBodyPtr sb1 = e[i].contact_geom1->get_single_body();
      Moby::SingleBodyPtr sb2 = e[i].contact_geom2->get_single_body();

      std::vector<std::string>::iterator iter =
          std::find(eef_names_.begin(), eef_names_.end(), sb1->id);
      //if end effector doesnt exist, check other SB
      if(iter  == eef_names_.end()){
        iter = std::find(eef_names_.begin(), eef_names_.end(), sb2->id);
        if(iter  == eef_names_.end())
          continue;  // Contact doesn't include an end-effector
        else{
          // contact is upside-down, mark for negation
          MIRROR_FLAG = true;
          std::swap(sb1,sb2);
        }
      }

      size_t index = std::distance(eef_names_.begin(), iter);

      if(MIRROR_FLAG){
        eefs_[index].impulse.push_back(-e[i].contact_impulse.get_linear());
      } else {
        eefs_[index].impulse.push_back( e[i].contact_impulse.get_linear());
      }

      // Push Active contact info to EEF
      eefs_[index].active = true;

      eefs_[index].point.push_back(e[i].contact_point);
      if(MIRROR_FLAG){
        eefs_[index].impulse.push_back(-e[i].contact_impulse.get_linear());
        eefs_[index].normal.push_back(-e[i].contact_normal);
        eefs_[index].tan1.push_back(-e[i].contact_tan1);
        eefs_[index].tan2.push_back(-e[i].contact_tan2);

      } else {
        eefs_[index].impulse.push_back(e[i].contact_impulse.get_linear());
        eefs_[index].normal.push_back(e[i].contact_normal);
        eefs_[index].tan1.push_back(e[i].contact_tan1);
        eefs_[index].tan2.push_back(e[i].contact_tan2);
      }
      eefs_[index].mu_coulomb.push_back(e[i].contact_mu_coulomb);
      eefs_[index].mu_viscous.push_back(e[i].contact_mu_viscous);
      eefs_[index].impulse[eefs_[index].impulse.size()-1].pose = eefs_[index].point[eefs_[index].point.size()-1].pose;
    }
  }

  for(int i=0, ii = 0;i<eefs_.size();i++){
    if(eefs_[i].active){
      for(int j=0;j<eefs_[i].point.size();j++){
#ifdef USE_OSG_DISPLAY
      visualize_ray(  eefs_[i].point[j],
                      eefs_[i].point[j] + eefs_[i].impulse[j]*10.0,
                      Ravelin::Vector3d(1,0.5,0),
                      0.1,
                      sim
                    );
#endif
      ii++;
      }
    }
  }

//#define OVERSAMPLE_FEET
#ifdef OVERSAMPLE_FEET
  static unsigned cpf = 1;
  static unsigned counter = 0;
  const unsigned obs = 10;
  counter += 1;
  if(counter%obs == 0)
    cpf += 1;

  for(int i=0;i<eefs_.size();i++){
    double sample_step = 0;
    if(eefs_[i].active){
      while(cpf > eefs_[i].point.size()){
        sample_step = -sample_step*1.01;
        Moby::Point3d new_point(eefs_[i].point[0][0],eefs_[i].point[0][1]+sample_step,eefs_[i].point[0][2],eefs_[i].point[0].pose);
        eefs_[i].point.push_back(new_point);
        eefs_[i].impulse.push_back(eefs_[i].impulse[0]);
        eefs_[i].normal.push_back(eefs_[i].normal[0]);
        eefs_[i].tan1.push_back(eefs_[i].tan1[0]);
        eefs_[i].tan2.push_back(eefs_[i].tan2[0]);
        eefs_[i].mu_coulomb.push_back(eefs_[i].mu_coulomb[0]);
        eefs_[i].mu_viscous.push_back(eefs_[i].mu_viscous[0]);
        eefs_[i].impulse[eefs_[i].impulse.size()-1].pose = eefs_[i].point[eefs_[i].point.size()-1].pose;
      }
    }
  }
#endif


}

#include <random>
boost::shared_ptr<Moby::ContactParameters> get_contact_parameters(Moby::CollisionGeometryPtr geom1, Moby::CollisionGeometryPtr geom2){

  boost::shared_ptr<Moby::ContactParameters> e = boost::shared_ptr<Moby::ContactParameters>(new Moby::ContactParameters());

  static std::default_random_engine generator;
  static std::uniform_real_distribution<double> distribution(0.1,1.4);

  e->mu_coulomb = distribution(generator);

  return e;
}

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

void init_cpp(const std::map<std::string, Moby::BasePtr>& read_map, double time){
  std::cout << "STARTING MOBY PLUGIN" << std::endl;
  // If use robot is active also init dynamixel controllers
  // get a reference to the EventDrivenSimulator instance
  for (std::map<std::string, Moby::BasePtr>::const_iterator i = read_map.begin();
       i !=read_map.end(); i++)
  {
    // Find the simulator reference
    if (!sim)
      sim = boost::dynamic_pointer_cast<Moby::EventDrivenSimulator>(i->second);

    // find the robot reference
    if (!abrobot)
    {
      abrobot = boost::dynamic_pointer_cast<Moby::RCArticulatedBody>(i->second);
    }
  }

  /// Set up quadruped robot, linking data from moby's articulated body
  /// to the quadruped model used by Control-Moby

  robot_ptr = boost::shared_ptr<Controller>(new Controller("model","vars.xml"));

  // CONTACT PARAMETER CALLBACK (MUST BE SET)
#ifdef RANDOM_FRICTION
  sim->get_contact_parameters_callback_fn = &get_contact_parameters;
#endif
  // CONTACT CALLBACK
//  sim->constraint_callback_fn             = &pre_event_callback_fn;
  sim->constraint_post_callback_fn        = &post_event_callback_fn;
  // CONTROLLER CALLBACK
  abrobot->controller                     = &controller_callback;

  // ================= INIT ROBOT STATE ==========================
  std::vector<Moby::JointPtr> joints_quad = robot_ptr->get_joints();
  std::vector<Moby::JointPtr> joints = abrobot->get_joints();

  std::map<std::string,double> q0 = robot_ptr->get_q0();

  std::vector<double> base_start;
  Utility::get_variable("init.base.x",base_start);

  int is_kinematic = 0;
  Utility::get_variable("init.kinematic",is_kinematic);

  if(is_kinematic)
    abrobot->set_kinematic(true);

  Ravelin::VectorNd q_start;
  abrobot->get_generalized_coordinates(Moby::DynamicBody::eSpatial,q_start);
  unsigned NDOFS = abrobot->num_generalized_coordinates(Moby::DynamicBody::eSpatial) - 6;


  for(unsigned i=NDOFS;i<q_start.rows();i++)
    q_start[i] = base_start[i-NDOFS];

  abrobot->set_generalized_coordinates(Moby::DynamicBody::eSpatial,q_start);
  abrobot->update_link_poses();
  for(int i=0,ii=0;i<joints.size();i++){
    for(int j=0;j<joints[i]->num_dof();j++,ii++){
      joints[i]->q[j] = q0[std::to_string(j)+joints[i]->id];
    }
  }
  abrobot->update_link_poses();
}

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
#include <Moby/EventDrivenSimulator.h>

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
void visualize_ray( const Ravelin::Vector3d& point, const Ravelin::Vector3d& vec, const Ravelin::Vector3d& c,double point_radius, boost::shared_ptr<EventDrivenSimulator> sim ) {

  // random color for this contact visualization
  double r = c[0] * VIBRANCY;
  double g = c[1] * VIBRANCY;
  double b = c[2] * VIBRANCY;
  osg::Vec4 color = osg::Vec4( r, g, b, 1.0/point_radius );

  const double point_scale = 0.01;

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

void visualize_ray( const Ravelin::Vector3d& point, const Ravelin::Vector3d& vec, const Ravelin::Vector3d& c, boost::shared_ptr<EventDrivenSimulator> sim ) {
  visualize_ray(point,vec,c,0.1,sim);
}

void draw_pose(const Ravelin::Pose3d& p, boost::shared_ptr<EventDrivenSimulator> sim ,double lightness){
  Ravelin::Pose3d pose(p);
  assert(lightness >= 0.0 && lightness <= 2.0);
  pose.update_relative_pose(Moby::GLOBAL);
  Ravelin::Matrix3d Rot(pose.q);
  Rot*= 0.3;
  double alpha = (lightness > 1.0)? 1.0 : lightness,
         beta = (lightness > 1.0)? lightness-1.0 : 0.0;

  visualize_ray(pose.x+Ravelin::Vector3d(Rot(0,0),Rot(1,0),Rot(2,0),Moby::GLOBAL)/10,Ravelin::Vector3d(0,0,0)+pose.x,Ravelin::Vector3d(alpha,beta,beta),sim);
  visualize_ray(pose.x+Ravelin::Vector3d(Rot(0,1),Rot(1,1),Rot(2,1),Moby::GLOBAL)/10,Ravelin::Vector3d(0,0,0)+pose.x,Ravelin::Vector3d(beta,alpha,beta),sim);
  visualize_ray(pose.x+Ravelin::Vector3d(Rot(0,2),Rot(1,2),Rot(2,2),Moby::GLOBAL)/10,Ravelin::Vector3d(0,0,0)+pose.x,Ravelin::Vector3d(beta,beta,alpha),sim);
}
#endif // USE_OSG_DISPLAY
