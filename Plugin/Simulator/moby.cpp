/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Moby/ArticulatedBody.h>
#include <Moby/RCArticulatedBody.h>
#include <Moby/ConstraintSimulator.h>
#include <Pacer/controller.h>
#include <Pacer/utilities.h>

#ifdef USE_OSG_DISPLAY
#warning "USE_OSG_DISPLAY turned on in moby.cpp"
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osg/Geode>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgGA/TrackballManipulator>
#include <osgGA/StateSetManipulator>
#endif

#include <Pacer/Random.h>

#include <time.h>
bool control_kinematics = false;

static double sleep_duration(double duration){
  timespec req,rem;
  int seconds = duration;
  req.tv_nsec = (duration - (double)seconds) * 1.0e+9;
  req.tv_sec = seconds;
  nanosleep(&req,&rem);
  return 0;//( ( (double) rem.tv_nsec / 1.0e+9 ) + (double) rem.tv_sec);
}

using Pacer::Controller;
typedef boost::shared_ptr<Ravelin::Jointd> JointPtr;

// pointer to the Pacer controller
//boost::shared_ptr<Ravelin::Pose3d> GLOBAL;
 boost::shared_ptr<Controller> robot_ptr;

// Weak pointer to moby objects
// pointer to the simulator
boost::weak_ptr<Moby::Simulator> sim_weak_ptr;
// pointer to the articulated body in Moby
boost::weak_ptr<Moby::ControlledBody> controlled_weak_ptr;


#ifdef USE_OSG_DISPLAY
void render( std::vector<Pacer::VisualizablePtr>& viz_vect);
void visualize_primitive(Moby::PrimitivePtr& primitive, boost::shared_ptr<const Ravelin::Pose3d>& pose_ptr, boost::shared_ptr<Moby::Simulator>& sim);
void visualize_ray( const Ravelin::Vector3d& point, const Ravelin::Vector3d& vec, const Ravelin::Vector3d& c, boost::shared_ptr<Moby::Simulator> sim );
#endif

 // ============================================================================
 // ================================ CONTROLLER ================================
 // ============================================================================

//     void (*controller)(boost::shared_ptr<ControlledBody>, double, void*);

// implements a controller callback for Moby
Ravelin::VectorNd& controller_callback(boost::shared_ptr<Moby::ControlledBody> cbp,Ravelin::VectorNd& cf, double t, void*)
{
  static Ravelin::VectorNd control_force = cf;
  boost::shared_ptr<Moby::RCArticulatedBody>
    abrobot = boost::dynamic_pointer_cast<Moby::RCArticulatedBody>(cbp);

  int num_joint_dof = 0;
  static std::vector<JointPtr> joints = abrobot->get_joints();
  static std::map<std::string, JointPtr> joints_map;
  if (joints_map.empty()) {
    for (std::vector<JointPtr>::iterator it = joints.begin(); it != joints.end(); it++)
      joints_map[(*it)->joint_id] = (*it);
  }

  static unsigned long long ITER = -1;
  static double last_time = -0.001;

  std::cout << "controller: time=" << t << " (dt="<< t - last_time << ")" << std::endl;

  if ((t - last_time) < 0.001-Moby::NEAR_ZERO) {
    cf = control_force;
    cf.set_zero();
    return cf;
  }
  robot_ptr->reset_state();

  
  ITER++;
  std::cout << "controller: iteration=" << ITER << " , time=" << t << " (dt="<< t - last_time << ")" << std::endl;

  double dt = t - last_time;
  last_time = t;

  /////////////////////////////////////////////////////////////////////////////
  ////////////////////////////// Get State: ///////////////////////////////////
  
  Ravelin::VectorNd generalized_q,generalized_qd,generalized_qdd, generalized_fext;

  {
    abrobot->get_generalized_coordinates_euler(generalized_q);
    abrobot->get_generalized_velocity(Ravelin::DynamicBodyd::eSpatial,generalized_qd);
    abrobot->get_generalized_forces(generalized_fext);
  }
  
  {
    // Re-map state simulation->robot joints
    std::map<std::string, Ravelin::VectorNd> q,qd,fext;
    Ravelin::VectorNd q_joints,qd_joints,fext_joints;
    for (int i=0;i<joints.size();i++){
      q[joints[i]->joint_id] = joints[i]->q;
      qd[joints[i]->joint_id] = joints[i]->qd;
      Ravelin::VectorNd& dofs
        = (fext[joints[i]->joint_id] = Ravelin::VectorNd(joints[i]->num_dof()));
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
  
  /////////////////////////////////////////////////////////////////////////////
  ////////////////////////////// Apply State: /////////////////////////////////
  static Ravelin::VectorNd  generalized_qd_last = generalized_qd;
  //NOTE: Pre-contact accel abrobot->get_generalized_acceleration(Ravelin::DynamicBodyd::eSpatial,generalized_qdd);
  ((generalized_qdd = generalized_qd) -= generalized_qd_last) /= dt;
  generalized_qd_last = generalized_qd;
  
  robot_ptr->set_generalized_value(Pacer::Robot::position,generalized_q);
  robot_ptr->set_generalized_value(Pacer::Robot::velocity,generalized_qd);
  robot_ptr->set_generalized_value(Pacer::Robot::acceleration,generalized_qdd);
  robot_ptr->set_generalized_value(Pacer::Robot::load,generalized_fext);
  
  // pointer to the simulator
  boost::shared_ptr<Moby::Simulator> sim = sim_weak_ptr.lock();

  boost::shared_ptr<Moby::ConstraintSimulator> csim;
  csim = boost::dynamic_pointer_cast<Moby::ConstraintSimulator>(sim);

  std::vector<Moby::Constraint>& e = csim->get_rigid_constraints();
  for(unsigned i=0;i<e.size();i++){
    if (e[i].constraint_type == Moby::Constraint::eContact)
    {
      boost::shared_ptr<Ravelin::SingleBodyd> sb1 = e[i].contact_geom1->get_single_body();
      boost::shared_ptr<Ravelin::SingleBodyd> sb2 = e[i].contact_geom2->get_single_body();
      
      Ravelin::Vector3d
      normal = e[i].contact_normal,
      tangent = e[i].contact_tan1,
      impulse = Ravelin::Vector3d(0,0,0);//e[i].contact_impulse.get_linear();
      impulse.pose = e[i].contact_point.pose;
      tangent.normalize();
      normal.normalize();
      
      OUT_LOG(logDEBUG) << "MOBY: contact-ids: " << sb1->body_id << " <-- " << sb2->body_id ;
      OUT_LOG(logDEBUG) << "MOBY: normal: " << normal;
      OUT_LOG(logDEBUG) << "MOBY: tangent: " << tangent;
      OUT_LOG(logDEBUG) << "MOBY: point: " << e[i].contact_point;
      
      boost::shared_ptr<Pacer::Robot::contact_t> new_contact;
      if(robot_ptr->is_end_effector(sb1->body_id)){
        new_contact = robot_ptr->create_contact(sb1->body_id,e[i].contact_point,normal,tangent,impulse,e[i].contact_mu_coulomb,e[i].contact_mu_viscous,0);
      } else if(robot_ptr->is_end_effector(sb2->body_id)){
        new_contact = robot_ptr->create_contact(sb2->body_id,e[i].contact_point,-normal,tangent,-impulse,e[i].contact_mu_coulomb,e[i].contact_mu_viscous,0);
      } else {
        continue;  // Contact doesn't include an end-effector
      }
      
      robot_ptr->add_contact(new_contact);
      
#ifdef USE_OSG_DISPLAY
      Utility::visualize.push_back(  Pacer::VisualizablePtr( new Pacer::Ray(e[i].contact_point,
                    e[i].contact_point + impulse*10.0,
                    Ravelin::Vector3d(1,0.5,0),0.1)));
      Utility::visualize.push_back(  Pacer::VisualizablePtr( new Pacer::Ray(e[i].contact_point,
                    e[i].contact_point + normal*0.1,
                    Ravelin::Vector3d(1,1,0),
                    0.1)));
#endif
    }
  }
  /////////////////////////////////////////////////////////////////////////////
  ////////////////////////////// Control Robot: ///////////////////////////////
  robot_ptr->control(t);
//#ifdef USE_OSG_DISPLAY
//  static osg::Group * MAIN_GROUP;
//  if (!MAIN_GROUP) {
//    MAIN_GROUP = new osg::Group;
//    MAIN_GROUP->addChild(sim->get_persistent_vdata());
//    MAIN_GROUP->addChild(sim->get_transient_vdata());
//  }
//  
//  if (ITER % 10 == 0) {
//    // write the file (fails silently)
//    char buffer[128];
//    sprintf(buffer, "driver.out-%08llu.osg", ITER, t);
//    osgDB::writeNodeFile(*MAIN_GROUP, std::string(buffer));
//  }
//  
//  std::cout << "Pacer called: iteration: " << ITER << " , time :" << t << " (dt="<< dt << ")" << std::endl;
//
//
//#endif

  /////////////////////////////////////////////////////////////////////////////
  /////////////////////// Send forces to Simulation: //////////////////////////
  {
    std::map<std::string, Ravelin::VectorNd > q, qd, u; 
    robot_ptr->get_joint_value(Pacer::Robot::position_goal, q);
    robot_ptr->get_joint_value(Pacer::Robot::velocity_goal, qd);
    robot_ptr->get_joint_value(Pacer::Robot::load_goal, u);
    
    if (robot_ptr->floating_base()) {
      control_force.set_zero(num_joint_dof+6);
    } else {
      control_force.set_zero(num_joint_dof);
    }
    
    if(!control_kinematics){
    for(int i=0;i<joints.size();i++){
//      joints[i]->add_force(u[joints[i]->joint_id]);
      int joint_index = joints[i]->get_coord_index();
      int num_joint_dofs = joints[i]->num_dof();
      std::string& joint_id = joints[i]->joint_id;
      for(int joint_dof=0;joint_dof<num_joint_dofs;joint_dof++){
        control_force[joint_index+joint_dof] = u[joint_id][joint_dof];
        OUT_LOG(logDEBUG) << joint_index << " " << joint_id << " " << joint_dof<< "/"<< num_joint_dofs<< "=" << u[joint_id][joint_dof];
      }
    }
      OUT_LOG(logINFO) << "MOBY: control: " << control_force;
    } else {
      control_force.set_zero();
      OUT_LOG(logINFO) << "MOBY is controlled kinematically ";
//      assert(control_force.norm_inf() == 0);
    }
  }

  std::vector<std::string> eef_names = robot_ptr->get_data<std::vector<std::string> >("init.end-effector.id");
  for(int i=0;i<eef_names.size();i++)
    robot_ptr->remove_data(eef_names[i]+".contact-force");
  cf = control_force;
  return cf;
}

// ============================================================================
// ================================ CALLBACKS =================================
// ============================================================================

/////////////////////////////////////////////////////////////////////////////////
//////// This is how we control in the kinematic only simulator /////////////////

//void (*constraint_callback_fn)(std::vector<Constraint>&, boost::shared_ptr<void>);
void constraint_callback_fn(std::vector<Moby::Constraint>& constraints, boost::shared_ptr<void> data){
    if(control_kinematics){
      boost::shared_ptr<Moby::ControlledBody> cbp = controlled_weak_ptr.lock();
      boost::shared_ptr<Moby::RCArticulatedBody>
        abrobot = boost::dynamic_pointer_cast<Moby::RCArticulatedBody>(cbp);
    
      static std::vector<JointPtr> joints = abrobot->get_joints();
      static std::map<std::string, boost::shared_ptr<Moby::Joint> > joints_map;
      if (joints_map.empty()) {
          for (std::vector<JointPtr>::iterator it = joints.begin(); it != joints.end(); it++){
              boost::shared_ptr<Ravelin::Jointd> jp = boost::const_pointer_cast<Ravelin::Jointd>(*it);
              boost::shared_ptr<Moby::Joint> mjp = boost::dynamic_pointer_cast<Moby::Joint>(jp);
              joints_map[(*it)->joint_id] = mjp;
            }
        }
      std::map<std::string, Ravelin::VectorNd > q, qd;
      robot_ptr->get_joint_value(Pacer::Robot::position_goal, q);
      robot_ptr->get_joint_value(Pacer::Robot::velocity_goal, qd);
      for(std::map<std::string, Ravelin::VectorNd >::iterator it = qd.begin() ; it!=qd.end() ; it++){
          Moby::Constraint c;
          c.constraint_type = Moby::Constraint::eInverseDynamics;
          c.qdot_des = (*it).second;
          c.inv_dyn_joint = joints_map[(*it).first];
          constraints.push_back(c);
        }
      }
  }

// examines contact events (after they have been handled in Moby)
void post_event_callback_fn(const std::vector<Moby::Constraint>& e,
                            boost::shared_ptr<void> empty)
{
  
  // pointer to the simulator
  boost::shared_ptr<Moby::Simulator> sim = sim_weak_ptr.lock();
  // pointer to the articulated body in Moby
  boost::shared_ptr<Moby::ControlledBody> abrobot = controlled_weak_ptr.lock();

  double t = sim->current_time;
  static double last_time = -0.001;
  double dt = t - last_time;
  last_time = t;
  
  std::vector<boost::shared_ptr<Pacer::Robot::contact_t> > contacts;
  // PROCESS CONTACTS
  OUT_LOG(logDEBUG) << "Events (post_event_callback_fn): " << e.size();
  double normal_sum = 0;

  for(unsigned i=0;i<e.size();i++){
    if (e[i].constraint_type == Moby::Constraint::eContact)
    {
      boost::shared_ptr<Ravelin::SingleBodyd> sb1 = e[i].contact_geom1->get_single_body();
      boost::shared_ptr<Ravelin::SingleBodyd> sb2 = e[i].contact_geom2->get_single_body();
      
      Ravelin::Vector3d 
        normal = e[i].contact_normal,
        tangent = e[i].contact_tan1,
        impulse = e[i].contact_impulse.get_linear();
      impulse.pose = e[i].contact_point.pose;
      tangent.normalize();
      normal.normalize();
    
      OUTLOG(impulse,"MOBY_force_"+sb1->body_id,logDEBUG);
      
      OUT_LOG(logDEBUG) << "SIMULATOR CONTACT FORCE : ";
      OUT_LOG(logDEBUG) << "t: " << t << " " << sb1->body_id << " <-- " << sb2->body_id;
        impulse/=dt;
        OUT_LOG(logDEBUG) << "force: " << impulse;

      if(robot_ptr->is_end_effector(sb1->body_id)){
        normal_sum += impulse.dot(normal);
        contacts.push_back(robot_ptr->create_contact(sb1->body_id,e[i].contact_point,normal,tangent,impulse,e[i].contact_mu_coulomb,e[i].contact_mu_viscous,0));
        robot_ptr->set_data<Ravelin::Vector3d>(sb1->body_id+".contact-force",
        Ravelin::Vector3d(impulse.dot(normal),impulse.dot(e[i].contact_tan1),impulse.dot(e[i].contact_tan2))
                                               );

      } else if(robot_ptr->is_end_effector(sb2->body_id)){
        normal_sum -= impulse.dot(normal);
        contacts.push_back(robot_ptr->create_contact(sb2->body_id,e[i].contact_point,-normal,tangent,-impulse,e[i].contact_mu_coulomb,e[i].contact_mu_viscous,0));
        robot_ptr->set_data<Ravelin::Vector3d>(sb2->body_id+".contact-force",Ravelin::Vector3d(impulse.dot(-normal),impulse.dot(e[i].contact_tan1),impulse.dot(-e[i].contact_tan2)));
      } else {
        continue;  // Contact doesn't include an end-effector  
      }
      
#ifdef USE_OSG_DISPLAY
      Utility::visualize.push_back(  Pacer::VisualizablePtr( new Pacer::Ray(e[i].contact_point,
                      e[i].contact_point + impulse*0.005,
                      Ravelin::Vector3d(1,0.5,0),
                      0.1)));
      Utility::visualize.push_back(  Pacer::VisualizablePtr( new Pacer::Ray(e[i].contact_point,
                      e[i].contact_point + normal*0.1,
                      Ravelin::Vector3d(1,1,0),
                      0.1)));
  #endif
    }
  }
  OUT_LOG(logDEBUG) << "0, Sum normal force: " << normal_sum ;
}

boost::weak_ptr<Moby::ArticulatedBody> abrobot_weak_ptr;
// hooks into Moby's post integration step callback function
void post_step_callback_fn(Moby::Simulator* s){
  boost::shared_ptr<Moby::Simulator> sim = sim_weak_ptr.lock();

  bool   display_moby_skeleton = false;
  bool   display_pacer_skeleton = false;

  robot_ptr->get_data<bool>("moby.display-moby-skeleton",display_moby_skeleton);
  robot_ptr->get_data<bool>("moby.display-pacer-skeleton",display_pacer_skeleton);
#ifndef NDEBUG
  display_moby_skeleton = true;
  display_pacer_skeleton = true;
#endif

#ifdef USE_OSG_DISPLAY
  if(display_moby_skeleton){
    // display collision
    static std::vector<std::string> _foot_ids = robot_ptr->get_data< std::vector<std::string> >("init.end-effector.id");

    boost::shared_ptr<Moby::ArticulatedBody> abrobot(abrobot_weak_ptr);
    BOOST_FOREACH(boost::shared_ptr<Ravelin::RigidBodyd> rbd, abrobot->get_links()){
      boost::shared_ptr<Moby::RigidBody> rb = boost::dynamic_pointer_cast<Moby::RigidBody>(rbd);
      
      boost::shared_ptr<Ravelin::Pose3d> foot_pose;
      BOOST_FOREACH(Moby::CollisionGeometryPtr cg, rb->geometries){
        Moby::PrimitivePtr primitive = cg->get_geometry();
        boost::shared_ptr<const Ravelin::Pose3d> cg_pose = cg->get_pose();
        boost::shared_ptr<Moby::SpherePrimitive> foot_geometry;
        foot_geometry = boost::dynamic_pointer_cast<Moby::SpherePrimitive>(primitive);
        if(foot_geometry){
          foot_pose = boost::shared_ptr<Ravelin::Pose3d>(new Ravelin::Pose3d(cg_pose->q,cg_pose->x,cg_pose->rpose));
        }
        visualize_primitive(primitive,cg_pose,sim);
      }
      
      for (int i=0;i<_foot_ids.size(); i++) {
        if(rb->body_id.compare(_foot_ids[i]) == 0){
          boost::shared_ptr<Ravelin::RigidBodyd> rb_ptr = rb;
          bool is_foot = true;
          
          // Iterate through joints
          do {
            OUT_LOG(logDEBUG) << "  " << rb_ptr->body_id;
            
            boost::shared_ptr<Ravelin::Jointd> joint_ptr = rb_ptr->get_inner_joint_explicit();
            OUT_LOG(logDEBUG) << "  " << joint_ptr->joint_id;
            
            {
              Ravelin::Pose3d joint_pose = *(joint_ptr->get_pose());
              joint_pose.update_relative_pose(Pacer::GLOBAL);
              
              {
                Ravelin::Pose3d link_pose = (is_foot)? *foot_pose : *(rb_ptr->get_pose());
                link_pose.update_relative_pose(Pacer::GLOBAL);
                visualize_ray(Ravelin::Vector3d(joint_pose.x.data()),Ravelin::Vector3d(link_pose.x.data()),Ravelin::Vector3d(1,0,0),sim);
              }
              
              rb_ptr = joint_ptr->get_inboard_link();
              
              {
                Ravelin::Pose3d link_pose = *(rb_ptr->get_pose());
                link_pose.update_relative_pose(Pacer::GLOBAL);
                visualize_ray(Ravelin::Vector3d(joint_pose.x.data()),Ravelin::Vector3d(link_pose.x.data()),Ravelin::Vector3d(1,0,0),sim);
              }
            }
            is_foot = false;
          } while (rb_ptr != abrobot->get_base_link());
        }
      }
    }
}
#endif
  
#ifdef USE_OSG_DISPLAY
  if(display_pacer_skeleton){
  // display collision
  static std::vector<std::string> _foot_ids = robot_ptr->get_data< std::vector<std::string> >("init.end-effector.id");
  robot_ptr->set_model_state(robot_ptr->get_generalized_value(Pacer::Robot::position));

  boost::shared_ptr<Moby::ArticulatedBody> abrobot = boost::dynamic_pointer_cast<Moby::ArticulatedBody>(robot_ptr->get_abrobot());
  BOOST_FOREACH(boost::shared_ptr<Ravelin::RigidBodyd> rbd, abrobot->get_links()){
    boost::shared_ptr<Moby::RigidBody> rb = boost::dynamic_pointer_cast<Moby::RigidBody>(rbd);
    
    boost::shared_ptr<Ravelin::Pose3d> foot_pose;
    BOOST_FOREACH(Moby::CollisionGeometryPtr cg, rb->geometries){
      Moby::PrimitivePtr primitive = cg->get_geometry();
      boost::shared_ptr<const Ravelin::Pose3d> cg_pose = cg->get_pose();
      boost::shared_ptr<Moby::SpherePrimitive> foot_geometry;
      foot_geometry = boost::dynamic_pointer_cast<Moby::SpherePrimitive>(primitive);
      if(foot_geometry){
        foot_pose = boost::shared_ptr<Ravelin::Pose3d>(new Ravelin::Pose3d(cg_pose->q,cg_pose->x,cg_pose->rpose));
      }
      visualize_primitive(primitive,cg_pose,sim);
    }
    
    for (int i=0;i<_foot_ids.size(); i++) {
      if(rb->body_id.compare(_foot_ids[i]) == 0){
        boost::shared_ptr<Ravelin::RigidBodyd> rb_ptr = rb;
        bool is_foot = true;
        do {
          OUT_LOG(logDEBUG) << "  " << rb_ptr->body_id;
          
          boost::shared_ptr<Ravelin::Jointd> joint_ptr = rb_ptr->get_inner_joint_explicit();
          OUT_LOG(logDEBUG) << "  " << joint_ptr->joint_id;
          
          {
            Ravelin::Pose3d joint_pose = *(joint_ptr->get_pose());
            joint_pose.update_relative_pose(Pacer::GLOBAL);
            
            {
              Ravelin::Pose3d link_pose = (is_foot)? *foot_pose : *(rb_ptr->get_pose());
              link_pose.update_relative_pose(Pacer::GLOBAL);
              VISUALIZE(RAY(  joint_pose.x.data(),   link_pose.x.data(),   Ravelin::Vector3d(1,1,1),0.1));
            }
            
            rb_ptr = joint_ptr->get_inboard_link();
//
//            {
//              Ravelin::Pose3d link_pose = *(rb_ptr->get_pose());
//              link_pose.update_relative_pose(Pacer::GLOBAL);
//              visualize_ray(Ravelin::Vector3d(joint_pose.x.data()),Ravelin::Vector3d(link_pose.x.data()),Ravelin::Vector3d(1,0,0),sim);
//            }
          }
          is_foot = false;
        }
        while (rb_ptr != abrobot->get_base_link());
      }
    }
    
  }
  }
#endif
//  bool WAIT_TIMESTEP = false;
//  robot_ptr->get_data<bool>("moby.wait-timestep",WAIT_TIMESTEP);
//  if (WAIT_TIMESTEP) {
//    sleep_duration(0.001);
//  }
/////////////////////////////////////////////////////////////////////////////
////////////////////////////// Visualize: ///////////////////////////////////
  #ifdef USE_OSG_DISPLAY
  render(Utility::visualize);
  #endif
}

// ============================================================================
// ================================ INIT ======================================
// ============================================================================

/// plugin must be "extern C"
extern "C" {

void init(void* separator, const std::map<std::string, Moby::BasePtr>& read_map, double time)
{
  // pointer to the simulator
  boost::shared_ptr<Moby::Simulator> sim;
  // pointer to the articulated body in Moby
  boost::shared_ptr<Moby::ControlledBody> controlled_body;

  OUT_LOG(logINFO) << "STARTING MOBY PLUGIN" ;
  
  // If use robot is active also init dynamixel controllers
  // get a reference to the Simulator instance
  for (std::map<std::string, Moby::BasePtr>::const_iterator i = read_map.begin();
       i !=read_map.end(); i++)
  {
    // Find the simulator reference
    
    if (!sim){
      sim = boost::dynamic_pointer_cast<Moby::Simulator>(i->second);
    }
    
    // find the robot reference
    if (!controlled_body)
    {
      boost::shared_ptr<Moby::ArticulatedBody> abrobot = boost::dynamic_pointer_cast<Moby::ArticulatedBody>(i->second);
      if(abrobot){
        abrobot_weak_ptr = boost::weak_ptr<Moby::ArticulatedBody>(abrobot);
        controlled_body = boost::dynamic_pointer_cast<Moby::ControlledBody>(i->second);
        controlled_weak_ptr = boost::weak_ptr<Moby::ControlledBody>(controlled_body);
      }
    }
  }
  if(!sim)
    throw std::runtime_error("Could not find simulator!");
  
  sim_weak_ptr = boost::weak_ptr<Moby::Simulator>(sim);

  if(!controlled_body)
    throw std::runtime_error("Could not find robot in simulator!");
  
  /// Set up quadruped robot, linking data from moby's articulated body
  /// to the quadruped model used by Control-Moby
  
  OUT_LOG(logINFO) << "Building Controller" ;
  
  robot_ptr = boost::shared_ptr<Controller>(new Controller());
  OUT_LOG(logINFO) << "Controller built" ;

  robot_ptr->init();
  
  OUT_LOG(logINFO) << "Controller inited" ;
  
  // CONTACT CALLBACK
  boost::shared_ptr<Moby::ConstraintSimulator>
  csim = boost::dynamic_pointer_cast<Moby::ConstraintSimulator>(sim);
  if (csim){
    csim->constraint_post_callback_fn        = &post_event_callback_fn;
    robot_ptr->get_data<bool>("init.control-kinematics",control_kinematics);
    if(control_kinematics){
      csim->constraint_callback_fn        = &constraint_callback_fn;
    }
  }
  sim->post_step_callback_fn = &post_step_callback_fn;
  // CONTROLLER CALLBACK
  controlled_body->controller                     = &controller_callback;
  
  // ================= INIT ROBOT STATE ==========================
  boost::shared_ptr<Moby::ArticulatedBody>
  abrobot = boost::dynamic_pointer_cast<Moby::ArticulatedBody>(controlled_body);
  boost::shared_ptr<Moby::RCArticulatedBody>
  rcabrobot = boost::dynamic_pointer_cast<Moby::RCArticulatedBody>(abrobot);

  std::vector<boost::shared_ptr<Ravelin::Jointd> > joints = abrobot->get_joints();
  
  // get generalized data from Moby
  Ravelin::VectorNd gq, gqd;
  abrobot->get_generalized_coordinates_euler(gq);
  abrobot->get_generalized_velocity(Ravelin::DynamicBodyd::eSpatial,gqd);
  
  // set values into vectors
  gq.set_zero();
  gqd.set_zero();

  // Get joint values
  std::map<std::string,Ravelin::VectorNd > q_start, qd_start;
  robot_ptr->get_joint_value(Pacer::Robot::position,q_start);
  robot_ptr->get_joint_value(Pacer::Robot::velocity,qd_start);
  
  OUTLOG(q_start,"Initial joint Coords",logERROR);
  OUTLOG(qd_start,"Initial joint Vel",logERROR);
  // Push robot data to Moby
  
  for(unsigned i=0;i<joints.size();i++){
//    if(joints[i]->num_dof() != 0){
//      assert(joints[i]->num_dof() == q_start[joints[i]->joint_id].rows());
//      joints[i]->q = q_start[joints[i]->joint_id];
//    }
    gq.segment(joints[i]->get_coord_index(),joints[i]->get_coord_index()+joints[i]->num_dof()) = q_start[joints[i]->joint_id];
//    gqd.segment(joints[i]->get_coord_index(),joints[i]->get_coord_index()+joints[i]->num_dof()) = qd_start[joints[i]->joint_id];
  }
  // update moby
//  rcabrobot->update_link_poses();
  
  if (robot_ptr->floating_base()) {
    // get pacer data
    Ravelin::VectorNd base_x, base_xd;
    robot_ptr->get_base_value(Pacer::Robot::position,base_x);
    robot_ptr->get_base_value(Pacer::Robot::velocity,base_xd);
    OUTLOG(base_x,"Initial base Coords",logERROR);
    OUTLOG(base_xd,"Initial base Vel",logERROR);
    
    //base
    gq.set_sub_vec(gq.size()-7,base_x);
    gqd.set_sub_vec(gqd.size()-6,base_xd);
  }
  
  OUTLOG(gq,"Initial Coords",logERROR);
  OUTLOG(gqd,"Initial Vels",logERROR);

  // send to moby
  abrobot->set_generalized_coordinates_euler(gq);
  abrobot->set_generalized_velocity(Ravelin::DynamicBodyd::eSpatial,gqd);
}
} // end extern C

#ifdef USE_OSG_DISPLAY
///////////////////////////////////////////////////////////////////////////////
/////////////////////////////// Visualization /////////////////////////////////

void visualize_ray(   const Ravelin::Vector3d& point, const Ravelin::Vector3d& vec, const Ravelin::Vector3d& color, boost::shared_ptr<Moby::Simulator> sim ) ;
void visualize_ray(   const Ravelin::Vector3d& point, const Ravelin::Vector3d& vec, const Ravelin::Vector3d& color,double point_radius, boost::shared_ptr<Moby::Simulator> sim ) ;
void draw_pose(const Ravelin::Pose3d& pose, boost::shared_ptr<Moby::Simulator> sim,double lightness = 1, double size=0.1);
void draw_text(std::string& text_str, const Ravelin::Vector3d& point, const Ravelin::Quatd& quat, const Ravelin::Vector3d& c, boost::shared_ptr<Moby::Simulator> sim, double size);

void render( std::vector<Pacer::VisualizablePtr>& viz_vect){
  for (std::vector<boost::shared_ptr<Pacer::Visualizable> >::iterator it = viz_vect.begin() ; it != viz_vect.end(); ++it)
  {
    // pointer to the simulator
    boost::shared_ptr<Moby::Simulator> sim = sim_weak_ptr.lock();
    
    switch((*it)->eType){
      case Pacer::Visualizable::eRay:{
        Pacer::Ray * v = static_cast<Pacer::Ray*>((it)->get());
        if (!std::isfinite(v->point1.norm())) break;
        if (!std::isfinite(v->point2.norm())) break;

        visualize_ray(v->point1,v->point2,v->color,v->size,sim);
        break;
      }
      case Pacer::Visualizable::ePoint:{
        Pacer::Point * v = static_cast<Pacer::Point*>((it)->get());
        if (!std::isfinite(v->point.norm())) break;
        visualize_ray(v->point,v->point,v->color,v->size,sim);
        break;
      }
      case Pacer::Visualizable::ePose:{
        Pacer::Pose * v = static_cast<Pacer::Pose*>((it)->get());
        if (!std::isfinite(v->pose.x.norm())) break;
        //        if (!std::isfinite(v->pose.q.norm())) break;
        draw_pose(v->pose,sim,v->shade,v->size);
        break;
      }
      case Pacer::Visualizable::eText:{
        Pacer::Text * v = static_cast<Pacer::Text*>((it)->get());
        if (!std::isfinite(v->point.norm())) break;
        //        if (!std::isfinite(v->pose.q.norm())) break;
        draw_text(v->text_string,v->point,v->quat,v->color,sim,v->size);
        break;
      }
      default:   OUT_LOG(logINFO) << "UNKNOWN VISUAL: " << (*it)->eType ; break;
    }
  }
  viz_vect.clear();
}

#include "visualize.cpp"

#endif // USE_OSG_DISPLAY
