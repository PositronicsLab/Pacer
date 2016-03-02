/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 *
 *  Dynamics with contact
 *  v = generalized_qd
 *  --- Dynamics model ---
 *  v- = v + h inv(M)(P tau + fext)
 *  --- Contact model ---
 *  v+ = v- + inv(M)([ N, ST+, ST- ] z)
 *  --- Contact model + Dynamics model ---
 *  v+ = v + inv(M)([ N, ST+, ST- ] z + h P tau + h fext)
 ****************************************************************************/
#include "inverse-dynamics.cpp"

#include <Pacer/controller.h>
#include "../plugin.h"

#undef OUT_LOG
#define OUT_LOG(level) \
std::cout << std::endl << Logger::ToString(level) << " -- "
#undef OUTLOG
#define OUTLOG(x,name,level) \
std::cout << std::endl << Logger::ToString(level) << " -- " << name << " = \n" << x;

//#define TIMING

using namespace Pacer;

void loop(){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  static double last_time = -0.001;
  double dt = t - last_time;
  last_time = t;
  
  OUT_LOG(logDEBUG) << "simulator_time = " << t;
  
  
  double dt_idyn = ctrl->get_data<double>(plugin_namespace+".dt");
  double alpha = ctrl->get_data<double>(plugin_namespace+".alpha");
  bool USE_DES_CONTACT = ctrl->get_data<bool>(plugin_namespace+".des-contact");
  bool USE_LAST_CFS = ctrl->get_data<bool>(plugin_namespace+".last-cfs");
  
  double DT = (dt_idyn == 0)? dt : dt_idyn;
  if(DT == 0) DT = 0.001;
  
  static std::vector<std::string>
  foot_names = ctrl->get_data<std::vector<std::string> >("init.end-effector.id");
  
  std::vector<std::string> active_feet;
  
  int num_feet = foot_names.size();
  
  if(USE_DES_CONTACT){
    for(int i=0;i<num_feet;i++){
      bool is_stance = false;
      if(ctrl->get_data<bool>(foot_names[i]+".stance",is_stance))
        if(is_stance)
          active_feet.push_back(foot_names[i]);
    }
  } else {
    active_feet = foot_names;
  }
  
  double mcpf = 1e2;
  ctrl->get_data<double>(plugin_namespace+".max-contacts-per-foot",mcpf);
  int MAX_CONTACTS_PER_FOOT = mcpf;
  
  // TODO: REMOVE
  //  int MULTIPLIER = std::ceil(t*5);
  
  //  static std::vector<int> num_added(num_feet);
  // TODO: REMOVE
  //  num_added[(( (int) (t*5.0*4.0) ) % 4 )] = MULTIPLIER;
  
  std::vector<unsigned> indices;
  std::vector< boost::shared_ptr<Pacer::Robot::contact_t> > contacts;
  boost::shared_ptr<Pose3d> base_link_frame = boost::shared_ptr<Pose3d>( new Ravelin::Pose3d(ctrl->get_data<Ravelin::Pose3d>("base_link_frame")));
  
  for(int i=0;i<active_feet.size();i++){
    std::vector< boost::shared_ptr< const Pacer::Robot::contact_t> > c;
    ctrl->get_link_contacts(active_feet[i],c);
    Vector3d pos;
    if(!c.empty()){
      for(int j=0;j<c.size() && j<MAX_CONTACTS_PER_FOOT;j++){
        OUT_LOG(logDEBUG) << "compliant: " << c[j]->compliant;
        OUT_LOG(logDEBUG) << "normal: " << c[j]->normal;
        OUT_LOG(logDEBUG) << "tangent: " << c[j]->tangent;
        OUT_LOG(logDEBUG) << "point: " << c[j]->point;
        std::string id(c[j]->id);
        pos = c[j]->point;
        Ravelin::Vector3d cf(0,0,0);
        ctrl->get_data<Ravelin::Vector3d>(id+".contact-force",cf);
        double mass = ctrl->get_data<double>("mass");
        OUT_LOG(logDEBUG) << "contact force: " << cf;
        OUT_LOG(logDEBUG) << "|contact force|: " << cf.norm();
        OUT_LOG(logDEBUG) << "Robot mass (kg): " << mass;
        OUT_LOG(logDEBUG) << "Robot weight (kg.m / s.s): " << (mass*grav);
        // Normal force > tolerance
        if (cf[0] > 0.1*(mass*grav)) {
          OUT_LOG(logDEBUG) << "Is Active Contact";
          
          //        for (int k=0; k<num_added[i]; k++) {
          contacts.push_back(ctrl->create_contact(id,pos,c[j]->normal,c[j]->tangent,c[j]->impulse,c[j]->mu_coulomb,c[j]->mu_viscous,0,c[j]->compliant));
          indices.push_back((unsigned) i);
          //        }
        }
      }
    }
  }
  
  
  
  int NC = contacts.size();
  OUT_LOG(logDEBUG) << "Number Active Contacts: " << NC;
  
  // State
  
  Ravelin::VectorNd q,generalized_qd,qdd_des,generalized_fext;
  Ravelin::VectorNd outputv;
  
  ctrl->get_generalized_value(Pacer::Robot::position,q);
  OUTLOG(q,"generalized_q",logDEBUG);
  
  ctrl->get_generalized_value(Pacer::Robot::velocity,generalized_qd);
  OUTLOG(generalized_qd,"generalized_qd",logDEBUG);
  
  ctrl->get_generalized_value(Pacer::Robot::load,generalized_fext);
  OUTLOG(generalized_fext,"generalized_fext",logDEBUG);
  
  ctrl->get_generalized_value(Pacer::Robot::acceleration,outputv);
  OUTLOG(outputv,"generalized_qdd",logDEBUG);
  
  // OUTPUT
  ctrl->get_joint_generalized_value(Pacer::Robot::position_goal,outputv);
  OUTLOG(outputv,"q_des",logDEBUG);
  
  ctrl->get_joint_generalized_value(Pacer::Robot::velocity_goal,outputv);
  OUTLOG(outputv,"qd_des",logDEBUG);
  
  ctrl->get_joint_generalized_value(Pacer::Robot::acceleration_goal,qdd_des);
  OUTLOG(qdd_des,"qdd_des",logDEBUG);
  
  int NUM_JOINT_DOFS = ctrl->num_joint_dof();
  int NDOFS = ctrl->num_total_dof();
  
  // Inertia
  static Ravelin::MatrixNd M;
  
  static Ravelin::VectorNd q_last = Ravelin::VectorNd::zero(q.size());
  // Consider increasing tolerance
  //    if((q_last-=q).norm() > ::NEAR_ZERO)
  ctrl->calc_generalized_inertia(q, M);
  q_last = q;
  // Jacobian Calculation
  Ravelin::MatrixNd N,S,T,D;
  
  ctrl->calc_contact_jacobians(q,contacts,N,S,T);
  
  D.set_zero(NDOFS,NC*4);
  D.set_sub_mat(0,0,S);
  D.set_sub_mat(0,NC,T);
  S.negate();
  T.negate();
  D.set_sub_mat(0,NC*2,S);
  D.set_sub_mat(0,NC*3,T);
  
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
  
  bool inf_friction = false;
  Ravelin::MatrixNd MU;
  MU.set_zero(N.columns(),2);
  for(int i=0;i<MU.rows();i++){
    std::fill(MU.row(i).begin(),MU.row(i).end(),contacts[i]->mu_coulomb);
  }
  
  Ravelin::VectorNd cf_init = Ravelin::VectorNd::zero(NC*5);
  {
    for(unsigned i=0;i< contacts.size();i++){
      
      Ravelin::Vector3d tan1,tan2;
      Ravelin::Vector3d::determine_orthonormal_basis(contacts[i]->normal,tan1,tan2);
      Ravelin::Matrix3d R_foot( contacts[i]->normal[0], contacts[i]->normal[1], contacts[i]->normal[2],
                               tan1[0],                tan1[1],                tan1[2],
                               tan2[0],                tan2[1],                tan2[2]);
      
      Ravelin::Origin3d contact_impulse = Ravelin::Origin3d(R_foot.mult(contacts[i]->impulse,workv3_));
      OUT_LOG(logDEBUG) << "Contact " << i << " mu coulomb: " << contacts[i]->mu_coulomb;
      
      OUT_LOG(logDEBUG) << "compliant: " << contacts[i]->compliant;
      OUT_LOG(logDEBUG) << "point: " << contacts[i]->point;
      if( !contacts[i]->compliant ){
        contact_impulse/=dt;
        OUT_LOG(logDEBUG) << "Contact " << i << " is rigid: " << contact_impulse ;
      } else {
        OUT_LOG(logDEBUG) << "Contact " << i << " is compliant: " << contact_impulse ;
      }
      cf_init[i] = contact_impulse[0];
      if(contact_impulse[1] >= 0)
        cf_init[NC+i] = contact_impulse[1];
      else
        cf_init[NC+i+NC*2] = -contact_impulse[1];
      if(contact_impulse[2] >= 0)
        cf_init[NC+i+NC] = contact_impulse[2];
      else
        cf_init[NC+i+NC*3] = -contact_impulse[2];
    }
    Utility::check_finite(cf_init);
    
    //    OUTLOG(cf_init,"cf_0",logDEBUG);
    
    Ravelin::SharedConstVectorNd cf_normal = cf_init.segment(0,NC);
    double sum = std::accumulate(cf_normal.begin(),cf_normal.end(),0.0);
    //    OUT_LOG(logDEBUG) << "0, Sum normal force: " << sum ;
    
    if(USE_LAST_CFS){
      static std::queue<Ravelin::VectorNd>
      cf_delay_queue;
      static std::queue<Ravelin::MatrixNd>
      N_delay_queue,
      D_delay_queue,
      MU_delay_queue;
      static bool FILTER_CFS = ctrl->get_data<bool>(plugin_namespace+".last-cfs-filter");
      
      if(FILTER_CFS && NC>0){
        
        cf_delay_queue.push(cf_init);
        N_delay_queue.push(N);
        D_delay_queue.push(D);
        MU_delay_queue.push(MU);
        cf_init = cf_delay_queue.front();
        N = N_delay_queue.front();
        D = D_delay_queue.front();
        MU = MU_delay_queue.front();
        if(cf_delay_queue.size() >= 2){
          cf_delay_queue.pop();
          N_delay_queue.pop();
          D_delay_queue.pop();
          MU_delay_queue.pop();
        }
      } else {
        cf_delay_queue = std::queue<Ravelin::VectorNd>();
        N_delay_queue = std::queue<Ravelin::MatrixNd>();
        D_delay_queue = std::queue<Ravelin::MatrixNd>();
        MU_delay_queue = std::queue<Ravelin::MatrixNd>();
      }
    }
  }
  
  OUTLOG(MU,"MU",logDEBUG);
  
  // IDYN MAXIMAL DISSIPATION MODEL
  std::vector<std::string> controller_name = ctrl->get_data<std::vector<std::string> >(plugin_namespace+".type");
  
  std::map<std::string,Ravelin::VectorNd> cf_map,uff_map;
  
  OUTLOG(NC,"idyn_NC",logDEBUG);
  
  ////////////////////////// simulator DT IDYN //////////////////////////////
  for (std::vector<std::string>::iterator it=controller_name.begin();
       it!=controller_name.end(); it++) {
    
    const double h_dt = DT/dt;
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
    }
    else if(USE_LAST_CFS){
      cf = cf_init;
      cf *= dt;
      OUT_LOG(logDEBUG) << "USING LAST CFS: " << cf;
      //      solve_flag = inverse_dynamics(qdd_des,M,N,D,generalized_fext,DT,id,cf);
      solve_flag = inverse_dynamics_one_stage(generalized_qd,qdd_des,M,N,D,generalized_fext,DT,MU,id,cf,indices,active_feet.size(),SAME_INDICES);
      OUT_LOG(logDEBUG) << "SAME: " << cf;
    } else {
#ifdef TIMING
      struct timeval start_t;
      struct timeval end_t;
      gettimeofday(&start_t, NULL);
#endif
      try{
        if(name.compare("NSQP") == 0){
          solve_flag = inverse_dynamics_no_slip(generalized_qd,qdd_des,M,N,D,generalized_fext,DT,id,cf,indices,active_feet.size(),SAME_INDICES);
        } else if(name.compare("NSLCP") == 0){
          solve_flag = inverse_dynamics_no_slip_fast(generalized_qd,qdd_des,M,N,D,generalized_fext,DT,id,cf,false,indices,active_feet.size(),SAME_INDICES);
        } else if(name.compare("CFQP") == 0){    // IDYN QP
          solve_flag = inverse_dynamics_two_stage(generalized_qd,qdd_des,M,N,D,generalized_fext,DT,MU,id,cf,indices,active_feet.size(),SAME_INDICES);
        } else if(name.compare("CFQP1") == 0){    // IDYN QP
          solve_flag = inverse_dynamics_one_stage(generalized_qd,qdd_des,M,N,D,generalized_fext,DT,MU,id,cf,indices,active_feet.size(),SAME_INDICES);
        } else if(name.compare("CFLCP") == 0){
          solve_flag = inverse_dynamics_ap(generalized_qd,qdd_des,M,N,D,generalized_fext,DT,MU,id,cf);
        }  else if(name.compare("SCFQP") == 0){
          double damping = 0;
          ctrl->get_data<double>(plugin_namespace+".damping",damping);
          solve_flag = inverse_dynamics_two_stage_simple(generalized_qd,qdd_des,M,N,D,generalized_fext,DT,MU,id,cf, damping);
        } else if(name.compare("SNSQP") == 0){
          double damping = 0;
          ctrl->get_data<double>(plugin_namespace+".damping",damping);
          solve_flag = inverse_dynamics_two_stage_simple_no_slip(generalized_qd,qdd_des,M,N,D,generalized_fext,DT,id,cf, damping);
        } else {
          solve_flag = inverse_dynamics_no_contact(generalized_qd,qdd_des,M,generalized_fext,dt,id);
        }
      } catch (std::exception e){
        solve_flag = false;
      }
      
      
      if(!std::isfinite(cf.norm()) || !std::isfinite(id.norm())){
        OUTLOG(DT,"DT",logDEBUG);
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
    }
    
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
    
    cf /= DT;
    cf_map[name] = cf;
    uff_map[name] = id;
  }
  
  OUTLOG(controller_name,"controller_name",logDEBUG);
  for (int i=0;i<controller_name.size();i++){
    std::string& name = controller_name[i];//(*it);
    Ravelin::VectorNd& cf = cf_map[name];
    OUTLOG(uff_map[name],"uff_"+boost::icl::to_string<double>::apply(i+1),logDEBUG);
    OUTLOG(cf,"cf_"+boost::icl::to_string<double>::apply(i+1),logDEBUG);
  }
  
  Ravelin::VectorNd uff = uff_map[controller_name.front()];
  uff*=alpha;
  
  Ravelin::VectorNd u = ctrl->get_joint_generalized_value(Pacer::Controller::load_goal);
  OUTLOG(uff,"idyn_U",logDEBUG);
  if(u.rows() == uff.rows()){
    u += uff;
    ctrl->set_joint_generalized_value(Pacer::Controller::load_goal,u);
    OUTLOG(u,"FINAL_U",logDEBUG);
  }
}


void setup(){
  
}