/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#define LOG_TO_FILE
#include <Pacer/output.h>

//#undef OUT_LOG
//#define OUT_LOG(level) \
//if(0) std::cout << std::endl << Logger::ToString(level) << " -- "
//#undef OUTLOG
//#define OUTLOG(x,name,level) \
//if(0) std::cout << std::endl << Logger::ToString(level) << " -- " << name << " = \n" << x;

#include "inverse-dynamics.cpp"

#include <Moby/ArticulatedBody.h>
#include <Moby/RCArticulatedBody.h>
#include <Moby/ConstraintSimulator.h>


typedef boost::shared_ptr<Ravelin::Jointd> JointPtr;

// Weak pointer to moby objects
// pointer to the simulator
boost::weak_ptr<Moby::Simulator> sim_weak_ptr;
// pointer to the articulated body in Moby
boost::weak_ptr<Moby::ControlledBody> controlled_weak_ptr;
boost::weak_ptr<Moby::RCArticulatedBody> rcabrobot_weak_ptr;

boost::weak_ptr<Moby::RigidBody> object_weak_ptr;

int NDOFS = 0;

///////////////////////////////////////////////////////////////////////////////
////////////////////////// JACOBIANS //////////////////////////////////////////
void calc_contact_jacobians(const std::vector<Moby::UnilateralConstraint>& c ,Ravelin::MatrixNd& N,Ravelin::MatrixNd& S,Ravelin::MatrixNd& T){
  static Ravelin::VectorNd workv_;
  static Ravelin::MatrixNd workM_;
  
  boost::shared_ptr<Moby::RCArticulatedBody> abrobot(rcabrobot_weak_ptr);
  boost::shared_ptr<Moby::RigidBody> object(object_weak_ptr);

  int NC = c.size();
  N.set_zero(NDOFS+6,NC);
  S.set_zero(NDOFS+6,NC);
  T.set_zero(NDOFS+6,NC);
  if(NC==0) return;
  
  // Contact Jacobian [GLOBAL frame]
  Ravelin::MatrixNd J(3,NDOFS);
  for(int i=0;i<NC;i++){
    
    boost::shared_ptr<Ravelin::SingleBodyd> sb1 = c[i].contact_geom1->get_single_body();
    boost::shared_ptr<Ravelin::SingleBodyd> sb2 = c[i].contact_geom2->get_single_body();

    // Arm
    boost::shared_ptr<const Ravelin::Pose3d>
    impulse_frame(new Ravelin::Pose3d(Ravelin::Quatd::identity(),c[i].contact_point.data(),Moby::GLOBAL));
    
    Ravelin::Vector3d normal = c[i].contact_normal;
    Ravelin::Vector3d tan1 = c[i].contact_tan1;
    Ravelin::Vector3d tan2 = c[i].contact_tan2;

    abrobot->calc_jacobian(impulse_frame,sb2,workM_);

    OUTLOG(workM_,"J_arm",logERROR);

    workM_.get_sub_mat(0,3,0,NDOFS,J);
    
    // Normal direction
    J.transpose_mult(normal,workv_);
    N.column(i).segment(0,NDOFS) = workv_;
    
    // 1st tangent
    J.transpose_mult(tan1,workv_);
    S.column(i).segment(0,NDOFS) = workv_;
    
    // 2nd tangent
    J.transpose_mult(tan2,workv_);
    T.column(i).segment(0,NDOFS) = workv_;
    
    // Rigid Body
    object->calc_jacobian(impulse_frame,Moby::GLOBAL,sb1,workM_);
    OUTLOG(workM_,"J_object",logERROR);

    workM_.get_sub_mat(0,3,0,6,J);

    // Normal direction
    J.transpose_mult(normal,workv_);
    N.column(i).segment(NDOFS,NDOFS+6) = workv_;
    
    // 1st tangent
    J.transpose_mult(tan1,workv_);
    S.column(i).segment(NDOFS,NDOFS+6) = workv_;
    
    // 2nd tangent
    J.transpose_mult(tan2,workv_);
    T.column(i).segment(NDOFS,NDOFS+6) = workv_;
  }
}

Ravelin::MatrixNd jacobian_ST_to_D(Ravelin::MatrixNd S,Ravelin::MatrixNd T){
  int NC = S.columns();
  int N = S.rows();
  Ravelin::MatrixNd D(N,NC*4);
  D.set_sub_mat(0,0,S);
  D.set_sub_mat(0,NC,T);
  S.negate();
  T.negate();
  D.set_sub_mat(0,NC*2,S);
  D.set_sub_mat(0,NC*3,T);
  return D;
}


// ============================================================================
// ================================ CONTROLLER ================================
// ============================================================================

//     void (*controller)(boost::shared_ptr<ControlledBody>, double, void*);

// implements a controller callback for Moby
Ravelin::VectorNd& controller_callback(boost::shared_ptr<Moby::ControlledBody> cbp,Ravelin::VectorNd& control_force, double t, void*)
{
  
  Ravelin::VectorNd WORKV;
  Ravelin::MatrixNd WORKM;
  boost::shared_ptr<Moby::RCArticulatedBody>
    abrobot = boost::dynamic_pointer_cast<Moby::RCArticulatedBody>(cbp);
  
  boost::shared_ptr<Moby::RigidBody> object(object_weak_ptr);
  

  NDOFS = abrobot->num_joint_dof();

  
  static unsigned long long ITER = 0;
  static double last_time = -0.001;
  double dt = t - last_time;
  last_time = t;
  
  /////////////////////////////////////////////////////////////////////////////
  ////////////////////////////// Get State: ///////////////////////////////////
  
//  Ravelin::VectorNd generalized_q(NDOFS+6);
  Ravelin::VectorNd generalized_qd(NDOFS+6), generalized_fext(NDOFS+6);
  
    {
//      // Arm
//      abrobot->get_generalized_coordinates_euler(WORKV);
//      generalized_q.segment(0,NDOFS) = WORKV;
      
      abrobot->get_generalized_velocity(Ravelin::DynamicBodyd::eSpatial,WORKV);
      generalized_qd.segment(0,NDOFS) = WORKV;
      
      abrobot->get_generalized_forces(WORKV);
      generalized_fext.segment(0,NDOFS) = WORKV;
      
      // Object
//      object->get_generalized_coordinates_euler(WORKV);
//      generalized_q.segment(NDOFS,NDOFS+7) = WORKV;
      
      object->get_generalized_velocity(Ravelin::DynamicBodyd::eSpatial,WORKV);
      generalized_qd.segment(NDOFS,NDOFS+6) = WORKV;
      
      object->get_generalized_forces(WORKV);
      generalized_fext.segment(NDOFS,NDOFS+6) = WORKV;
    }
  
  OUTLOG(generalized_fext,"generalized_fext",logDEBUG);
  OUTLOG(generalized_qd,"generalized_qd",logDEBUG);

  /////////////////////////////////////////////////////////////////////////////
  ////////////////////////////// Get contacts: ///////////////////////////////////

    // pointer to the simulator
    boost::shared_ptr<Moby::Simulator> sim = sim_weak_ptr.lock();
    
    boost::shared_ptr<Moby::ConstraintSimulator> csim;
    csim = boost::dynamic_pointer_cast<Moby::ConstraintSimulator>(sim);
    
    std::vector<Moby::UnilateralConstraint>& rigid_constraints = csim->get_rigid_constraints();
    std::vector<Moby::UnilateralConstraint>& compliant_constraints = csim->get_compliant_constraints();
    std::vector<Moby::UnilateralConstraint> e;
    std::map<std::string, Moby::UnilateralConstraint> contacts;

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
        
        
        OUT_LOG(logDEBUG) << "MOBY: contact-ids: " << sb1->body_id << " <-- " << sb2->body_id ;
        OUT_LOG(logDEBUG) << "MOBY: compliant: " << compliant;
        OUT_LOG(logDEBUG) << "MOBY: normal: " << normal;
        OUT_LOG(logDEBUG) << "MOBY: tangent: " << tangent;
        OUT_LOG(logDEBUG) << "MOBY: point: " << e[i].contact_point;
        
        if(sb2->body_id.compare("BLOCK") == 0){
          continue;
//          throw std::runtime_error("sb2 nees to be a FINGER_, not the BLOCK");
        }

        contacts[sb2->body_id] = e[i];
        }
    }
  
  std::map<std::string, unsigned> finger_index;
  finger_index["FINGER_0"] = 0;
  finger_index["FINGER_1"] = 1;
  finger_index["FINGER_2"] = 2;
  finger_index["FINGER_3"] = 3;

  // Create active contacts vector
  std::vector<Moby::UnilateralConstraint> c;
  std::vector<unsigned> indices;
  for( std::map<std::string, Moby::UnilateralConstraint>::iterator it = contacts.begin(); it != contacts.end(); ++it ) {
    
    boost::shared_ptr<Ravelin::SingleBodyd> sb1 = it->second.contact_geom1->get_single_body();
    boost::shared_ptr<Ravelin::SingleBodyd> sb2 = it->second.contact_geom2->get_single_body();
    
    if(sb2->body_id.compare("BLOCK") == 0){
//      continue;
      throw std::runtime_error("sb2 nees to be a FINGER_, not the BLOCK");
    }
    
    c.push_back( it->second );
    indices.push_back(finger_index[sb2->body_id]);
  }
  
  /////////////////////////////////////////////////////////////////////////////
  ////////////////////////////// Get JACOBIANS: ///////////////////////////////////

  int NC = c.size();

  OUTLOG(NC,"NC",logDEBUG);

  // Make Jacobians
  Ravelin::MatrixNd N,S,T,D,MU;
  calc_contact_jacobians(c,N,S,T);
  D = jacobian_ST_to_D(S,T);
  OUTLOG(N,"N",logDEBUG);

  MU.set_zero(NC,2);
  for( int i = 0 ; i < c.size() ; i++ ) {
    std::fill(MU.row(i).begin(),MU.row(i).end(),c[i].contact_mu_coulomb);
  }
  OUTLOG(MU,"MU",logDEBUG);

  Ravelin::VectorNd qdd_des = Ravelin::VectorNd::zero(NDOFS);

  Ravelin::MatrixNd M = Ravelin::MatrixNd::zero(NDOFS+6,NDOFS+6);
  
  abrobot->get_generalized_inertia(WORKM);
  M.block(0,NDOFS,0,NDOFS) = WORKM;
  
  object->get_generalized_inertia(WORKM);
  OUTLOG(WORKM,"M_block",logDEBUG);

  M.block(NDOFS,NDOFS+6,NDOFS,NDOFS+6) = WORKM;
  OUTLOG(M,"M",logDEBUG);

  assert(M.rows() == N.rows());
  assert(generalized_qd.rows() == N.rows());
  
  // reset control force.
  control_force.set_zero(generalized_qd.rows());
  
  
  ////////////////////////// simulator DT IDYN //////////////////////////////

  // IDYN MAXIMAL DISSIPATION MODEL
  std::vector<std::string> controller_name;
//  controller_name.push_back("CFQP1");
//  controller_name.push_back("SCFQP");
  controller_name.push_back("CFLCP");
  controller_name.push_back("NSQP");
//  controller_name.push_back("SNSQP");
  controller_name.push_back("NSLCP");
  controller_name.push_back("CFQP");
  controller_name.push_back("NOCP");

  std::map<std::string,Ravelin::VectorNd> cf_map,uff_map;
  
  OUTLOG(NC,"idyn_NC",logDEBUG);
  
  // Index of foot check (continuity of contacts)
  static std::vector<unsigned> last_indices = indices;
  static int last_NC = NC;
  bool SAME_INDICES = false;
  if(NC == last_NC){
    SAME_INDICES = true;
    for (int i=0; i<indices.size(); i++) {
      if(last_indices[i] != indices[i]){
        SAME_INDICES = false;
        break;
      }
    }
  }
  last_NC = NC;
  last_indices = indices;

  int NUM_JOINT_DOFS = NDOFS;
  bool USE_LAST_CFS = false;
  
  for (std::vector<std::string>::iterator it=controller_name.begin();
       it!=controller_name.end(); it++) {
    
    bool solve_flag = false;
    
    Ravelin::VectorNd id = Ravelin::VectorNd::zero(NUM_JOINT_DOFS);
    Ravelin::VectorNd cf;// = Ravelin::VectorNd::zero(NC*5);
    
    std::string& name = (*it);
    
    OUT_LOG(logDEBUG) << "CONTROLLER: " << name;
    OUT_LOG(logDEBUG) << "USE_LAST_CFS: " << USE_LAST_CFS;
    //
    if (NC == 0) {
      solve_flag = inverse_dynamics_no_contact(generalized_qd,qdd_des,M,generalized_fext,dt,id);
      cf /= dt;
      cf_map[name] = cf;
      uff_map[name] = id;
      control_force = id;
      return control_force;
    }
//    else if(USE_LAST_CFS){
//      cf = cf_init;
//      cf *= dt;
//      OUT_LOG(logDEBUG) << "USING LAST CFS: " << cf;
//      //      solve_flag = inverse_dynamics(qdd_des,M,N,D,generalized_fext,dt,id,cf);
//      solve_flag = inverse_dynamics_one_stage(generalized_qd,qdd_des,M,N,D,generalized_fext,dt,MU,id,cf,indices,NC,SAME_INDICES);
//      OUT_LOG(logDEBUG) << "SAME: " << cf;
//    }
    else {
#ifdef TIMING
      struct timeval start_t;
      struct timeval end_t;
      gettimeofday(&start_t, NULL);
#endif
      try{
        if(name.compare("NSQP") == 0){
          solve_flag = inverse_dynamics_no_slip(generalized_qd,qdd_des,M,N,D,generalized_fext,dt,id,cf,indices,NC,SAME_INDICES);
        } else if(name.compare("NSLCP") == 0){
          solve_flag = inverse_dynamics_no_slip_fast(generalized_qd,qdd_des,M,N,D,generalized_fext,dt,id,cf,false,indices,NC,SAME_INDICES);
        } else if(name.compare("CFQP") == 0){    // IDYN QP
          solve_flag = inverse_dynamics_two_stage(generalized_qd,qdd_des,M,N,D,generalized_fext,dt,MU,id,cf,indices,NC,SAME_INDICES);
        } else if(name.compare("CFQP1") == 0){    // IDYN QP
          solve_flag = inverse_dynamics_one_stage(generalized_qd,qdd_des,M,N,D,generalized_fext,dt,MU,id,cf,indices,NC,SAME_INDICES);
        } else if(name.compare("CFLCP") == 0){
          solve_flag = inverse_dynamics_ap(generalized_qd,qdd_des,M,N,D,generalized_fext,dt,MU,id,cf);
        }  else if(name.compare("SCFQP") == 0){
          double damping = 0;
          solve_flag = inverse_dynamics_two_stage_simple(generalized_qd,qdd_des,M,N,D,generalized_fext,dt,MU,id,cf, damping);
        } else if(name.compare("SNSQP") == 0){
          double damping = 0;
          solve_flag = inverse_dynamics_two_stage_simple_no_slip(generalized_qd,qdd_des,M,N,D,generalized_fext,dt,id,cf, damping);
        } else {
          solve_flag = inverse_dynamics_no_contact(generalized_qd,qdd_des,M,generalized_fext,dt,id);
          control_force = id;
        }
      } catch (std::exception e){
        solve_flag = false;
      }
      
      /*
      if(!std::isfinite(cf.norm()) || !std::isfinite(id.norm())){
        OUTLOG(dt,"dt",logDEBUG);
        OUTLOG(cf,"IDYN_CF",logDEBUG);
        OUTLOG(id,"IDYN_U",logDEBUG);
        
#ifndef TIMING
        throw std::runtime_error("IDYN forces are NaN or INF");
#else //NOT TIMING
        continue;
#endif
      }
#ifdef TIMING
      gettimeofday(&end_t, NULL);
      double duration = (end_t.tv_sec - start_t.tv_sec) + (end_t.tv_usec - start_t.tv_usec) * 1E-6;
      std::cout << ("timing_"+name+" ") << duration*1000.0 << " " << NC << " " << t << std::endl;
#endif
       */

    }
    
    /*

    //// Check for Normal direction torque chatter
    if (NC > 0 && solve_flag){
      static std::map<std::string,Ravelin::VectorNd> last_cf;
      std::map<std::string,Ravelin::VectorNd>::iterator it = last_cf.find(name);
      if(it != last_cf.end()){
        if(last_cf[name].rows() == NC){
          Ravelin::VectorNd diff_cf  = (*it).second;
          diff_cf -= cf.segment(0,NC);
          if(diff_cf.norm() > 0.01)
            OUT_LOG(logDEBUG) << "-- Torque chatter detected " << t;
        }
      } else {
        last_cf[name] = cf.segment(0,NC);
      }
    }
    
    double normal_sum = 0;
    */
    
    cf /= dt;
    cf_map[name] = cf;
    uff_map[name] = id;
  }
  
  OUTLOG(controller_name,"controller_name",logDEBUG);
  for (std::map<std::string,Ravelin::VectorNd>::iterator it=uff_map.begin();it!=uff_map.end();it++){
    std::string name = it->first;
    Ravelin::VectorNd& cf = cf_map[name];
    OUTLOG(uff_map[name],"uff_"+name,logDEBUG);
    OUTLOG(cf,"cf_"+name,logDEBUG);
//    control_force = uff_map[name];
  }

  OUTLOG(control_force,"final_control_force",logDEBUG);

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
    }
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
    boost::shared_ptr<Moby::RigidBody> object;
    
    OUT_LOG(logINFO) << "STARTING MOBY PLUGIN" ;
    
    // If use robot is active also init dynamixel controllers
    // get a reference to the Simulator instance
    for (std::map<std::string, Moby::BasePtr>::const_iterator i = read_map.begin();
         i !=read_map.end(); i++)
    {
      OUT_LOG(logINFO) << i->second->id ;

      // Find the simulator reference
      
      if (!sim){
        sim = boost::dynamic_pointer_cast<Moby::Simulator>(i->second);
      }
      
      // find the robot reference
      if (!controlled_body)
      {
        boost::shared_ptr<Moby::RCArticulatedBody> abrobot = boost::dynamic_pointer_cast<Moby::RCArticulatedBody>(i->second);
        if(abrobot){
          rcabrobot_weak_ptr = boost::weak_ptr<Moby::RCArticulatedBody>(abrobot);
          controlled_body = boost::dynamic_pointer_cast<Moby::ControlledBody>(i->second);
          controlled_weak_ptr = boost::weak_ptr<Moby::ControlledBody>(controlled_body);
        }
      }
      
      // find the robot reference
      if (i->second->id.compare("BLOCK") == 0)
      {
        object = boost::dynamic_pointer_cast<Moby::RigidBody>(i->second);
        if(!object)
          throw std::runtime_error("Could not find block!");
        object_weak_ptr = boost::weak_ptr<Moby::RigidBody>(object);

      }
    }
    if(!sim)
      throw std::runtime_error("Could not find simulator!");
    
    sim_weak_ptr = boost::weak_ptr<Moby::Simulator>(sim);
    
    if(!controlled_body)
      throw std::runtime_error("Could not find robot in simulator!");
    
    // CONTACT CALLBACK
    boost::shared_ptr<Moby::ConstraintSimulator>
    csim = boost::dynamic_pointer_cast<Moby::ConstraintSimulator>(sim);
    if (csim){
      csim->constraint_post_callback_fn        = &post_event_callback_fn;
    }
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
//    gq.set_zero();
    std::fill(gq.begin(),gq.end(),1.5708);
    for(int i=0;i<4;i++)
      gq[gq.rows()-4+i] = 1.5708 * 0.5;
    
    gqd.set_zero();
    
    
    
    abrobot->set_generalized_coordinates_euler(gq);
    abrobot->set_generalized_velocity(Ravelin::DynamicBodyd::eSpatial,gqd);
  }
} // end extern C
