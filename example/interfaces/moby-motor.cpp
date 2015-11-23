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

#ifdef SIMULATE_UNCERTAINTY
#include "Random.h"
#endif

using Pacer::Controller;
typedef boost::shared_ptr<Ravelin::Jointd> JointPtr;

// pointer to the Pacer controller
boost::shared_ptr<Controller> robot_ptr;

// Weak pointer to moby objects
// pointer to the simulator
boost::weak_ptr<Moby::Simulator> sim_weak_ptr;
// pointer to the articulated body in Moby
boost::weak_ptr<Moby::ControlledBody> controlled_weak_ptr;


#ifdef USE_OSG_DISPLAY
void render( std::vector<Pacer::VisualizablePtr>& viz_vect);
#endif


/////////////////////////////////////////////////////////////////////
///////////////////////// MOTOR /////////////////////////////////////
/////////////////////////////////////////////////////////////////////


#include <time.h>

#include <boost/assign/list_of.hpp>
#include <boost/assign/std/vector.hpp>

#ifdef USE_DXL
#include <dxl/Dynamixel.h>
DXL::Dynamixel * dxl_;
#endif

#ifdef USE_THREADS
#include <pthread.h>
pthread_mutex_t joint_data_mutex_;
pthread_t thread;
#endif

static Ravelin::VectorNd q_motors,qd_motors,u_motors;
static Ravelin::VectorNd q_sensor,qd_sensor,u_sensor;

static double FREQ = 1000;

void *control_motors(void* data){
  OUT_LOG(logDEBUG2) << ">> control_motor()" << std::endl;
#ifdef USE_THREADS
  while(true)
#endif
  {
    OUT_LOG(logDEBUG2) << ">> motor controller loop" << std::endl;
    
    OUT_LOG(logDEBUG) << ">> motor controller";
    std::cerr << q_motors << std::endl;
    OUT_LOG(logDEBUG) << "qd_des = " << qd_motors;
//    OUT_LOG(logDEBUG) << "u_des = " << u_motors;
    qd_motors.set_zero();
    
    OUT_LOG(logDEBUG2) << "READ OUT COMMANDS" << std::endl;
    
    std::vector<double> qm, qdm, um;
#ifdef USE_THREADS
    if(pthread_mutex_trylock(&joint_data_mutex_))
#endif
    {
      qm = std::vector<double>(q_motors.begin(),q_motors.end());
      qdm = std::vector<double>(qd_motors.begin(),qd_motors.end());
      um = std::vector<double>(u_motors.begin(),u_motors.end());
#ifdef USE_THREADS
      pthread_mutex_unlock(&joint_data_mutex_);
#endif
    }
    
    std::vector<double> qs, qds, us;
    
#ifdef USE_DXL
    dxl_->set_state(qm,qdm);
    qs = qm;
    qds = qdm;
    //    dxl_->get_state(qs,qds,us);
    //    dxl_->set_torque(um);
#endif
    
    OUT_LOG(logDEBUG2) << "READ IN SENSORS" << std::endl;
    
#ifdef USE_THREADS
    if(pthread_mutex_trylock(&joint_data_mutex_))
#endif
    {
      q_sensor = Ravelin::VectorNd(qs.size(),&qm[0]);
      qd_sensor = Ravelin::VectorNd(qds.size(),&qdm[0]);
      //      u_sensor = Ravelin::VectorNd(us.size(),&um[0]);
#ifdef USE_THREADS
      pthread_mutex_unlock(&joint_data_mutex_);
#endif
    }
    
    OUT_LOG(logDEBUG) << "q = " << q_sensor;
    OUT_LOG(logDEBUG) << "qd = " << qd_sensor;
    //    OUT_LOG(logDEBUG) << "u = " << u_sensor;
    OUT_LOG(logDEBUG) << "<< motor controller";
    OUT_LOG(logDEBUG2) << "<< motor controller" << std::endl;
    
#ifdef USE_THREADS
    sleep(1.0/FREQ);
#endif
  }
  OUT_LOG(logDEBUG2) << "<< control_motor()" << std::endl;
  
  return data;
}

void init_motors(){
  OUT_LOG(logINFO) << "STARTING MOTORS" << std::endl;
#ifdef USE_DXL
  // If use robot is active also init dynamixel controllers
  dxl_ = new DXL::Dynamixel("/dev/tty.usbserial-A9YL9ZZV");
  // LINKS robot
  
  // Set Dynamixel Names
  std::vector<std::string> dxl_name = boost::assign::list_of
  ("LF_X_1")("RF_X_1")("LH_X_1")("RH_X_1")
  ("LF_Y_2")("RF_Y_2")("LH_Y_2")("RH_Y_2")
  ("LF_Y_3")("RF_Y_3")("LH_Y_3")("RH_Y_3");
  
  dxl_->names = dxl_name;
  
  // Set Joint Angles
  dxl_->tare.push_back( M_PI_2 * RX_24F_RAD2UNIT);
  dxl_->tare.push_back(-M_PI_2 * RX_24F_RAD2UNIT);
  dxl_->tare.push_back(-M_PI_2 * RX_24F_RAD2UNIT);
  dxl_->tare.push_back( M_PI_2 * RX_24F_RAD2UNIT);
  
  dxl_->tare.push_back( M_PI_2 * RX_24F_RAD2UNIT);
  dxl_->tare.push_back(-M_PI_2 * RX_24F_RAD2UNIT);
  dxl_->tare.push_back(-M_PI_2 * MX_64R_RAD2UNIT+250);
  dxl_->tare.push_back( M_PI_2 * MX_64R_RAD2UNIT+40);
  
  dxl_->tare.push_back(0);
  dxl_->tare.push_back(0);
  dxl_->tare.push_back(0);
  dxl_->tare.push_back(0);
  
  
  // Set Dynamixel Type
  std::vector<DXL::Dynamixel::Type> dxl_type = boost::assign::list_of
  (DXL::Dynamixel::RX_24F)(DXL::Dynamixel::RX_24F)(DXL::Dynamixel::RX_24F)(DXL::Dynamixel::RX_24F)
  (DXL::Dynamixel::RX_24F)(DXL::Dynamixel::RX_24F)(DXL::Dynamixel::MX_64R)(DXL::Dynamixel::MX_64R)
  (DXL::Dynamixel::RX_24F)(DXL::Dynamixel::RX_24F)(DXL::Dynamixel::RX_24F)(DXL::Dynamixel::RX_24F);
  
  dxl_->stype = dxl_type;
  dxl_->ids.push_back(1);
  dxl_->ids.push_back(100);
  dxl_->ids.push_back(4);
  dxl_->ids.push_back(3);
  
  dxl_->ids.push_back(5);
  dxl_->ids.push_back(6);
  dxl_->ids.push_back(8);
  dxl_->ids.push_back(7);
  
  dxl_->ids.push_back(9);
  dxl_->ids.push_back(10);
  dxl_->ids.push_back(12);
  dxl_->ids.push_back(11);
  
  q_motors.set_zero(dxl_->ids.size());
  qd_motors.set_zero(dxl_->ids.size());
  u_motors.set_zero(dxl_->ids.size());
  
  dxl_->relaxed(false);
  
  int N = dxl_->ids.size();
  
#ifndef NDEBUG
  for (int i=0; i<N; i++) {
    std::cout << dxl_->names[i] << " ";
  }
  std::cout << std::endl;
  for (int i=0; i<N; i++) {
    std::cout << dxl_->ids[i] << " ";
  }
  std::cout << std::endl;

#endif
  
  q_motors = Ravelin::VectorNd::zero(N);
  qd_motors = Ravelin::VectorNd::zero(N);
  u_motors = Ravelin::VectorNd::zero(N);
  q_sensor = Ravelin::VectorNd::zero(N);
  qd_sensor = Ravelin::VectorNd::zero(N);
  u_sensor = Ravelin::VectorNd::zero(N);
  
  std::map<std::string,Ravelin::VectorNd> joint_pos_map;
  robot_ptr->get_joint_value(Pacer::Robot::position,joint_pos_map);
  for(int i=0;i<dxl_->ids.size();i++){
    q_motors[i] = joint_pos_map[dxl_->JointName(i)][0];
    qd_motors[i] = 0;
    u_motors[i] = 0;
    q_sensor[i] = joint_pos_map[dxl_->JointName(i)][0];
    qd_sensor[i] = 0;
    u_sensor[i] = 0;
    
  }
  
#endif
  
#ifdef USE_THREADS
  pthread_mutex_unlock(&joint_data_mutex_);;
#endif
  
#ifdef USE_THREADS
  const char *message;
  static int iret = pthread_create( &thread, NULL,&control_motors,(void*)NULL);
  if(iret)
  {
    fprintf(stderr,"Error - pthread_create() return code: %d\n",iret);
    exit(1);
  }
#endif
  
}

void update_motors(){
#ifdef USE_THREADS
  if(pthread_mutex_lock(&joint_data_mutex_))
#endif
  {
    bool kineamtic_control = true;
    if(kineamtic_control){
      std::map<std::string,Ravelin::VectorNd> joint_pos_map;
      robot_ptr->get_joint_value(Pacer::Robot::position_goal,joint_pos_map);
      std::map<std::string,Ravelin::VectorNd> joint_vel_map;
      robot_ptr->get_joint_value(Pacer::Robot::velocity_goal,joint_vel_map);
      for(int i=0;i<dxl_->ids.size();i++){
        q_motors[i] = joint_pos_map[dxl_->JointName(i)][0];
        qd_motors[i] = joint_vel_map[dxl_->JointName(i)][0];
      }
    } else {
      std::map<std::string,Ravelin::VectorNd> joint_load_map;
      robot_ptr->get_joint_value(Pacer::Robot::load_goal,joint_load_map);
      for(int i=0;i<dxl_->ids.size();i++){
        u_motors[i] = joint_load_map[dxl_->JointName(i)][0];
      }
    }
    
#ifdef USE_THREADS
    pthread_mutex_unlock(&joint_data_mutex_);
#endif
  }
  
#ifndef USE_THREADS
  OUT_LOG(logDEBUG2) << "call control_motor() from controller" << std::endl;
  control_motors((void*)NULL);
#endif

  OUT_LOG(logDEBUG2) << "end controller()" << std::endl;
}



// ============================================================================
// ================================ CONTROLLER ================================
// ============================================================================

//     void (*controller)(boost::shared_ptr<ControlledBody>, double, void*);

// implements a controller callback for Moby
Ravelin::VectorNd& controller_callback(boost::shared_ptr<Moby::ControlledBody> cbp,Ravelin::VectorNd& control_force, double t, void*)
{
  
  boost::shared_ptr<Moby::RCArticulatedBody>
  abrobot = boost::dynamic_pointer_cast<Moby::RCArticulatedBody>(cbp);
  
  int num_joint_dof = 0;
  static std::vector<JointPtr> joints = abrobot->get_joints();
  static std::map<std::string, JointPtr> joints_map;
  if (joints_map.empty()) {
    for (std::vector<JointPtr>::iterator it = joints.begin(); it != joints.end(); it++)
      joints_map[(*it)->joint_id] = (*it);
  }
  
  static unsigned long long ITER = 0;
  static double last_time = -0.001;
  double dt = t - last_time;
  last_time = t;
  
  /////////////////////////////////////////////////////////////////////////////
  ////////////////////////////// Get State: ///////////////////////////////////
  
  if (ITER%1 == 0) {
    
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
    ////////////////////////////// Simulate Uncertainty /////////////////////////
#ifdef SIMULATE_UNCERTAINTY
    /////////////////////////////////////////////////////////////////////////////
    ////////////////////////////// Apply Noise: /////////////////////////////////
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
#endif
    
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
    
    /////////////////////////////////////////////////////////////////////////////
    /////////////////////// Collision Detection: ////////////////////////////////
    // pointer to the simulator
    boost::shared_ptr<Moby::Simulator> sim = sim_weak_ptr.lock();
    
    boost::shared_ptr<Moby::ConstraintSimulator> csim;
    csim = boost::dynamic_pointer_cast<Moby::ConstraintSimulator>(sim);
    
    std::vector<Moby::UnilateralConstraint>& rigid_constraints = csim->get_rigid_constraints();
    std::vector<Moby::UnilateralConstraint>& compliant_constraints = csim->get_compliant_constraints();
    std::vector<Moby::UnilateralConstraint> e;
    e.insert(e.end(), rigid_constraints.begin(), rigid_constraints.end());
    e.insert(e.end(), compliant_constraints.begin(), compliant_constraints.end());
    for(unsigned i=0;i<e.size();i++){
      if (e[i].constraint_type == Moby::UnilateralConstraint::eContact)
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
        
        bool compliant =
        (e[i].compliance == Moby::UnilateralConstraint::eCompliant)? true : false;
        
        
        //      OUT_LOG(logERROR) << "compliant: " << compliant;
        //      OUT_LOG(logERROR) << "normal: " << normal;
        //      OUT_LOG(logERROR) << "tangent: " << tangent;
        //      OUT_LOG(logERROR) << "point: " << e[i].contact_point;
        if(robot_ptr->is_end_effector(sb1->body_id)){
          robot_ptr->add_contact(sb1->body_id,e[i].contact_point,normal,tangent,impulse,e[i].contact_mu_coulomb,e[i].contact_mu_viscous,0,compliant);
          robot_ptr->set_data<Ravelin::Vector3d>
          (sb1->body_id+".contact-force",
           Ravelin::Vector3d
           (impulse.dot(normal),impulse.dot(e[i].contact_tan1),impulse.dot(e[i].contact_tan2)));
        } else if(robot_ptr->is_end_effector(sb2->body_id)){
          robot_ptr->add_contact(sb2->body_id,e[i].contact_point,-normal,tangent,-impulse,e[i].contact_mu_coulomb,e[i].contact_mu_viscous,0,compliant);
          robot_ptr->set_data<Ravelin::Vector3d>
          (sb2->body_id+".contact-force",
           Ravelin::Vector3d
           (impulse.dot(-normal),impulse.dot(e[i].contact_tan1),impulse.dot(-e[i].contact_tan2)));
        } else {
          continue;  // Contact doesn't include an end-effector
        }
        
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
#ifdef USE_OSG_DISPLAY
    /////////////////////////////////////////////////////////////////////////////
    ////////////////////////////// Visualize: ///////////////////////////////////
    render(Utility::visualize);
#endif
    /////////////////////////////////////////////////////////////////////////////
    /////////////////////// Send forces to Simulation: //////////////////////////
    {
      std::map<std::string, Ravelin::VectorNd > q, qd, u;
      robot_ptr->get_joint_value(Pacer::Robot::position_goal, q);
      robot_ptr->get_joint_value(Pacer::Robot::velocity_goal, qd);
      robot_ptr->get_joint_value(Pacer::Robot::load_goal, u);
      
      // Enforce joint and motor limits
      std::vector<std::string> joint_names = robot_ptr->get_data<std::vector<std::string> >("init.joint.id");
      std::vector<double> joint_dofs = robot_ptr->get_data<std::vector<double> >("init.joint.dofs");
      std::vector<double> torque_limit;
      bool apply_torque_limit = robot_ptr->get_data<std::vector<double> >("init.joint.limits.u",torque_limit);
      
      std::vector<double> velocity_limit;
      bool apply_velocity_limit = robot_ptr->get_data<std::vector<double> >("init.joint.limits.qd",velocity_limit);
      
      for (int i=0, ii=0; i<joint_names.size(); i++) {
        for (int j=0; j<joint_dofs[i]; j++,ii++) {
          // If motor speed limit met, cancel torque
          // (if applied in direction of limit)
          if(apply_velocity_limit){
            if (qd[joint_names[i]][j] > velocity_limit[ii]) {
              if(u[joint_names[i]][j] > 0)
                u[joint_names[i]][j] = 0;
            } else if  (qd[joint_names[i]][j] < -velocity_limit[ii]) {
              if(u[joint_names[i]][j] < 0)
                u[joint_names[i]][j] = 0;
            }
          }
          
          if(apply_torque_limit){
            // Limit torque
            if (u[joint_names[i]][j] > torque_limit[ii]) {
              u[joint_names[i]][j] = torque_limit[ii];
            } else if  (u[joint_names[i]][j] < -torque_limit[ii]) {
              u[joint_names[i]][j] = -torque_limit[ii];
            }
          }
        }
      }
      
      // Control simulation
      control_force.set_zero(num_joint_dof+6);
      for(int i=0;i<joints.size();i++){
        //      joints[i]->add_force(u[joints[i]->joint_id]);
        int joint_index = joints[i]->get_coord_index();
        int num_joint_dofs = joints[i]->num_dof();
        std::string& joint_id = joints[i]->joint_id;
        for(int joint_dof=0;joint_dof<num_joint_dofs;joint_dof++){
          control_force[joint_index+joint_dof] = u[joint_id][joint_dof];
        }
      }

      update_motors();
    }
  }
  
  std::vector<std::string> eef_names = robot_ptr->get_data<std::vector<std::string> >("init.end-effector.id");
  for(int i=0;i<eef_names.size();i++)
    robot_ptr->remove_data(eef_names[i]+".contact-force");
  return control_force;
}

// ============================================================================
// ================================ CALLBACKS =================================
// ============================================================================

// examines contact events (after they have been handled in Moby)
void post_event_callback_fn(const std::vector<Moby::UnilateralConstraint>& e,
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
  OUT_LOG(logERROR) << "Events (post_event_callback_fn): " << e.size();
  double normal_sum = 0;
  
  for(unsigned i=0;i<e.size();i++){
    if (e[i].constraint_type == Moby::UnilateralConstraint::eContact)
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
      
      bool compliant =
      (e[i].compliance == Moby::UnilateralConstraint::eCompliant)? true : false;
      
      //      OUT_LOG(logERROR) << "compliant: " << compliant;
      OUT_LOG(logERROR) << "SIMULATOR CONTACT FORCE : ";
      OUT_LOG(logERROR) << "t: " << t << " " << sb1->body_id << " <-- " << sb2->body_id;
      if(compliant)
        OUT_LOG(logERROR) << "force: " << impulse;
      else{
        impulse/=dt;
        OUT_LOG(logERROR) << "force: " << impulse;
      }
      
      if(robot_ptr->is_end_effector(sb1->body_id)){
        normal_sum += impulse.dot(normal);
        contacts.push_back(robot_ptr->create_contact(sb1->body_id,e[i].contact_point,normal,tangent,impulse,e[i].contact_mu_coulomb,e[i].contact_mu_viscous,0,compliant));
        robot_ptr->set_data<Ravelin::Vector3d>(sb1->body_id+".contact-force",
                                               Ravelin::Vector3d(impulse.dot(normal),impulse.dot(e[i].contact_tan1),impulse.dot(e[i].contact_tan2))
                                               );
        
      } else if(robot_ptr->is_end_effector(sb2->body_id)){
        normal_sum -= impulse.dot(normal);
        contacts.push_back(robot_ptr->create_contact(sb2->body_id,e[i].contact_point,-normal,tangent,-impulse,e[i].contact_mu_coulomb,e[i].contact_mu_viscous,0,compliant));
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
  OUT_LOG(logERROR) << "0, Sum normal force: " << normal_sum ;
  
  /////////////////////////////////////////////////////////////////////////////
  ////////////////////////////// Get State: ///////////////////////////////////
  
#ifdef COMPARE_SIM_STATE
  int num_joint_dof;
  
  static std::vector<JointPtr> joints = abrobot->get_joints();
  static std::map<std::string, JointPtr> joints_map;
  if (joints_map.empty()) {
    for (std::vector<JointPtr>::iterator it = joints.begin(); it != joints.end(); it++)
      joints_map[(*it)->joint_id] = (*it);
  }
  
  Ravelin::VectorNd generalized_q,generalized_qd,generalized_qdd, generalized_fext;
  
  {
    abrobot->get_generalized_coordinates_euler(generalized_q);
    abrobot->get_generalized_velocity(Ravelin::DynamicBodyd::eSpatial,generalized_qd);
    //    abrobot->get_generalized_forces(generalized_fext);
  }
  
  {
    // Re-map state simulation->robot joints
    std::map<std::string, Ravelin::VectorNd> q,qd,fext;
    Ravelin::VectorNd q_joints,qd_joints,fext_joints;
    for (int i=0;i<joints.size();i++){
      q[joints[i]->joint_id] = joints[i]->q;
      qd[joints[i]->joint_id] = joints[i]->qd;
      //      Ravelin::VectorNd& dofs
      //      = (fext[joints[i]->joint_id] = Ravelin::VectorNd(joints[i]->num_dof()));
      //      for (int j=0;j<joints[i]->num_dof();j++)
      //        dofs[j] = generalized_fext[joints[i]->get_coord_index()+j];
    }
    
    robot_ptr->convert_to_generalized(q,q_joints);
    robot_ptr->convert_to_generalized(qd,qd_joints);
    //    robot_ptr->convert_to_generalized(fext,fext_joints);
    
    generalized_q.set_sub_vec(0,q_joints);
    generalized_qd.set_sub_vec(0,qd_joints);
    //    generalized_fext.set_sub_vec(0,fext_joints);
    
    num_joint_dof = q_joints.size();
  }
  
  OUT_LOG(logERROR) << "generalized_q (post-contact) " << generalized_q << normal_sum ;
  OUT_LOG(logERROR) << "generalized_qd (post-contact) " << generalized_qd << normal_sum ;
#endif
}

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
    point = Ravelin::Pose3d::transform_point(Pacer::GLOBAL,Ravelin::Vector3d(0,0,0,sb1->get_pose()));
  } else if(robot_ptr->is_end_effector(sb2->id)){
    point = Ravelin::Pose3d::transform_point(Pacer::GLOBAL,Ravelin::Vector3d(0,0,0,sb2->get_pose()));
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

/// plugin must be "extern C"
extern "C" {
  
  void init(void* separator, const std::map<std::string, Moby::BasePtr>& read_map, double time)
  {
    // pointer to the simulator
    boost::shared_ptr<Moby::Simulator> sim;
    // pointer to the articulated body in Moby
    boost::shared_ptr<Moby::ControlledBody> controlled_body;
    
    std::cout << "STARTING MOBY PLUGIN" << std::endl;
    
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
    
    std::cout << "Building Controller" << std::endl;
    
    robot_ptr = boost::shared_ptr<Controller>(new Controller());
    std::cout << "Controller built" << std::endl;
    
    robot_ptr->init();
    
    std::cout << "Controller inited" << std::endl;
    
    // CONTACT CALLBACK
    //  boost::shared_ptr<Moby::ConstraintSimulator>
    //  csim = boost::dynamic_pointer_cast<Moby::ConstraintSimulator>(sim);
    //  if (csim){
    //    csim->constraint_post_callback_fn        = &post_event_callback_fn;
    //  }
    
    // CONTROLLER CALLBACK
    controlled_body->controller                     = &controller_callback;
    
    init_motors();
    
    // ================= INIT ROBOT STATE ==========================
    
    std::map<std::string,Ravelin::VectorNd > q_start, qd_start;
    robot_ptr->get_joint_value(Pacer::Robot::position,q_start);
    robot_ptr->get_joint_value(Pacer::Robot::velocity,qd_start);
    
    boost::shared_ptr<Moby::ArticulatedBody>
    abrobot = boost::dynamic_pointer_cast<Moby::ArticulatedBody>(controlled_body);
    std::vector<boost::shared_ptr<Ravelin::Jointd> > joints = abrobot->get_joints();
    
    for(unsigned i=0;i<joints.size();i++){
      if(joints[i]->num_dof() != 0){
        assert(joints[i]->num_dof() == q_start[joints[i]->joint_id].rows());
        joints[i]->q = q_start[joints[i]->joint_id];
        joints[i]->qd = qd_start[joints[i]->joint_id];
      }
    }
    
    boost::shared_ptr<Moby::RCArticulatedBody>
    rcabrobot = boost::dynamic_pointer_cast<Moby::RCArticulatedBody>(abrobot);
    rcabrobot->update_link_poses();
    
    Ravelin::VectorNd base_x, base_xd;
    robot_ptr->get_base_value(Pacer::Robot::position,base_x);
    robot_ptr->get_base_value(Pacer::Robot::velocity,base_xd);
    
    OUTLOG(base_x,"Initial base Coords",logERROR);
    OUTLOG(base_xd,"Initial base Vel",logERROR);
    
    Ravelin::VectorNd gq, gqd;
    abrobot->get_generalized_coordinates_euler(gq);
    gq.set_sub_vec(gq.size()-7,base_x);
    abrobot->set_generalized_coordinates_euler(gq);
    
    abrobot->get_generalized_velocity(Ravelin::DynamicBodyd::eSpatial,gqd);
    gqd.set_sub_vec(gqd.size()-6,base_xd);
    abrobot->set_generalized_velocity(Ravelin::DynamicBodyd::eSpatial,gqd);
  }
} // end extern C



#ifdef USE_OSG_DISPLAY
///////////////////////////////////////////////////////////////////////////////
/////////////////////////////// Visualization /////////////////////////////////

void visualize_ray(   const Ravelin::Vector3d& point, const Ravelin::Vector3d& vec, const Ravelin::Vector3d& color, boost::shared_ptr<Moby::Simulator> sim ) ;
void visualize_ray(   const Ravelin::Vector3d& point, const Ravelin::Vector3d& vec, const Ravelin::Vector3d& color,double point_radius, boost::shared_ptr<Moby::Simulator> sim ) ;
void draw_pose(const Ravelin::Pose3d& pose, boost::shared_ptr<Moby::Simulator> sim,double lightness = 1, double size=0.1);

void render( std::vector<Pacer::VisualizablePtr>& viz_vect){
  for (std::vector<boost::shared_ptr<Pacer::Visualizable> >::iterator it = viz_vect.begin() ; it != viz_vect.end(); ++it)
  {
    // pointer to the simulator
    boost::shared_ptr<Moby::Simulator> sim = sim_weak_ptr.lock();
    
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


#include <boost/shared_ptr.hpp>
#include <Moby/Simulator.h>

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
void visualize_ray( const Ravelin::Vector3d& point, const Ravelin::Vector3d& vec, const Ravelin::Vector3d& c,double point_radius, boost::shared_ptr<Simulator> sim ) {
  
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
  hints->setDetailRatio( 0.1 );
  
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

void visualize_ray( const Ravelin::Vector3d& point, const Ravelin::Vector3d& vec, const Ravelin::Vector3d& c, boost::shared_ptr<Simulator> sim ) {
  visualize_ray(point,vec,c,0.1,sim);
}

void draw_pose(const Ravelin::Pose3d& p, boost::shared_ptr<Simulator> sim ,double lightness, double size){
  Ravelin::Pose3d pose(p);
  assert(lightness >= 0.0 && lightness <= 2.0);
  pose.update_relative_pose(Pacer::GLOBAL);
  Ravelin::Matrix3d Rot(pose.q);
  Rot*= 0.3;
  double alpha = (lightness > 1.0)? 1.0 : lightness,
  beta = (lightness > 1.0)? lightness-1.0 : 0.0;
  
  visualize_ray(pose.x+Ravelin::Vector3d(Rot(0,0),Rot(1,0),Rot(2,0),Pacer::GLOBAL)*size,Ravelin::Vector3d(0,0,0)+pose.x,Ravelin::Vector3d(alpha,beta,beta),size,sim);
  visualize_ray(pose.x+Ravelin::Vector3d(Rot(0,1),Rot(1,1),Rot(2,1),Pacer::GLOBAL)*size,Ravelin::Vector3d(0,0,0)+pose.x,Ravelin::Vector3d(beta,alpha,beta),size,sim);
  visualize_ray(pose.x+Ravelin::Vector3d(Rot(0,2),Rot(1,2),Rot(2,2),Pacer::GLOBAL)*size,Ravelin::Vector3d(0,0,0)+pose.x,Ravelin::Vector3d(beta,beta,alpha),size,sim);
}
#endif // USE_OSG_DISPLAY
