/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/controller.h>
#include <Pacer/utilities.h>
#include <sys/time.h>
#include <dlfcn.h>
#include <errno.h>
#include <boost/foreach.hpp>
#include <stdlib.h>     /* getenv */

#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <vector>

using namespace Pacer;
  
Controller::Controller(): Robot(){}

Controller::~Controller(){
  close_plugins();
}


std::vector<void*> handles;
bool Controller::close_plugins(){
  // close the loaded plugin libraries
  for(size_t i = 0; i < handles.size(); ++i){
    dlclose(handles[i]);
  }
  handles.clear();
  _update_priority_map.clear();
  return true;
}

typedef void (*init_t)(const boost::shared_ptr<Controller>, const char*);

bool Controller::init_plugins(){
  OUT_LOG(logDEBUG) << ">> Controller::init_plugins()";

  bool RETURN_FLAG = true;
  close_plugins();
  std::vector<init_t> INIT;

  // call the initializers, if any
  std::vector<std::string> &plugin_names = Utility::get_variable<std::vector<std::string> >("plugin.id");
  std::vector<std::string> &plugin_files = Utility::get_variable<std::vector<std::string> >("plugin.file");
  std::vector<int> &plugin_active = Utility::get_variable<std::vector<int> >("plugin.active");

  // Load all the plugins
  for(unsigned i=0;i<plugin_names.size();i++){
    if(plugin_active[i] == 0) continue;
    std::string filename = plugin_files[i];
    std::string pPath(getenv("PACER_PLUGIN_PATH"));
    std::string lib_path = pPath+"/"+filename;
    OUT_LOG(logINFO) << "Loading Plugin: " << plugin_names[i];
    OUT_LOG(logINFO) << "\tLIB: " << filename.c_str();
    OUT_LOG(logINFO) << "\tPATH: " << pPath.c_str();
    // attempt to read the file
    void* HANDLE = dlopen(lib_path.c_str(), RTLD_LAZY);
    if (!HANDLE)
    {
      std::cerr << "driver: failed to read plugin from " << filename << std::endl;
      std::cerr << "  " << dlerror() << std::endl;
      RETURN_FLAG = false;
    }
 
    handles.push_back(HANDLE);
 
    // attempt to load the initializer
    dlerror();
    INIT.push_back((init_t) dlsym(HANDLE, "init"));
    const char* dlsym_error = dlerror();
    if (dlsym_error)
    {
      std::cerr << "driver warning: cannot load symbol 'init' from " << filename << std::endl;
      std::cerr << "        error follows: " << std::endl << dlsym_error << std::endl;
      INIT.pop_back();
      RETURN_FLAG = false;
    } else {
      // Init the plugin
      (*INIT.back())(this->ptr(),plugin_names[i].c_str());
    }
  }
    
  OUT_LOG(logDEBUG) << "<< Controller::init_plugins()";
  return RETURN_FLAG;
}

// ============================================================================
// =========================== Begin Robot Controller =========================
// ============================================================================

void Controller::control(double t){
    OUT_LOG(logDEBUG) << ">> Controller::control(.)";
  // Import Robot Data
  static double last_time = -0.001;
  const double dt = t - last_time;
  update();
  lock_state();
#ifdef USE_PLUGINS
  update_plugins(t);
#endif
  unlock_state();
  /*
  // Select end effectors that are feet and set them as active contacts
  static std::vector<int>
      &is_foot = Utility::get_variable<std::vector<int> >("init.end-effector.foot");

  for(int i=0;i<NUM_EEFS;i++)
    eefs_[i].active = is_foot[i];

  // =================== BEGIN UPDATE & INIT ROBOT MODEL & CONTROL VARS =====================
  update(generalized_q_in,generalized_qd_in,generalized_qdd_in,generalized_fext_in);

  static Ravelin::VectorNd last_qdd_des = data->qdd,
                          last_qd_des = data->qd;

  // Set Robot Data in robot
  static bool inited = false;
  if(!inited){
    for(unsigned i=0;i< NUM_EEFS;i++){
      eefs_[i].origin = Ravelin::Pose3d::transform_point(base_frame,Ravelin::Vector3d(0,0,0,eefs_[i].link->get_pose()));
    }
    inited = true;
  }

  Ravelin::VectorNd uff, ufb;
  uff.set_zero(NUM_JOINT_DOFS);
  ufb.set_zero(NUM_JOINT_DOFS);
  u.set_zero(NUM_JOINT_DOFS);

  qdd_des.set_zero(NUM_JOINT_DOFS);
  qd_des.set_zero(NUM_JOINT_DOFS);
  q_des.set_zero(NUM_JOINT_DOFS);

  std::vector<Ravelin::Vector3d>
      x_des(NUM_EEFS),
      xd_des(NUM_EEFS),
      xdd_des(NUM_EEFS);


  for(int i=0,ii=0;i<NUM_JOINTS;i++){
    if(joints_[i])
    for(int j=0;j<joints_[i]->num_dof();j++,ii++){
      q_des[ii] = get_q0()[std::to_string(j)+joints_[i]->id];
      qd_des[ii] = 0;
      qdd_des[ii] = 0;
    }
  }

  set_model_state(data->q,data->qd);

  for(unsigned i=0;i< NUM_EEFS;i++){
    x_des[i] = eefs_[i].origin;//Ravelin::Pose3d::transform_point(base_frame,Ravelin::Vector3d(0,0,0,eefs_[i].link->get_pose()));
    xd_des[i].set_zero();
    xdd_des[i].set_zero();
    x_des[i].pose = xd_des[i].pose = xdd_des[i].pose = base_frame;
    Utility::visualize.push_back( Pacer::VisualizablePtr( new Point(Ravelin::Pose3d::transform_point(Moby::GLOBAL,Ravelin::Vector3d(0,0,0,eefs_[i].link->get_pose())),Ravelin::Vector3d(1,1,0),0.2)));
    Utility::visualize.push_back( Pacer::VisualizablePtr( new Point(Ravelin::Pose3d::transform_point(Moby::GLOBAL,eefs_[i].origin),Ravelin::Vector3d(1,0,0),0.2)));
  }

  std::vector<EndEffector*> feet;
  for(unsigned i=0,ii=0;i< NUM_EEFS;i++){
    if(is_foot[i] == 0) continue;
    feet.push_back(&eefs_[i]);
    feet[ii]->origin.pose = base_frame;

    Utility::visualize.push_back( Pacer::VisualizablePtr( new Point(Ravelin::Pose3d::transform_point(Moby::GLOBAL,feet[ii]->origin),
                    Ravelin::Vector3d(1,0,0),
                    0.1)));

    ii++;
  }

  // =================== END UPDATE & INIT ROBOT MODEL & CONTROL VARS =====================

  // =================== BEGIN PLANNING FUNCTIONS =====================

  static Ravelin::Vector3d sum_base_velocity;
  static std::queue<Ravelin::Vector3d> base_vel_queue;
  base_vel_queue.push(workv3_ = Ravelin::Vector3d(data->generalized_qd[NUM_JOINT_DOFS],data->generalized_qd[NUM_JOINT_DOFS+1],data->generalized_qd[NUM_JOINT_DOFS+2]));
  sum_base_velocity += workv3_;
  if(base_vel_queue.size() > 100){
     sum_base_velocity -= base_vel_queue.front();
     base_vel_queue.pop();
  }
  OUTLOG(workv3_,"base_velocity (now)",logDEBUG);
  OUTLOG(sum_base_velocity/(double)base_vel_queue.size(),"base_velocity (avg 1 sec)",logDEBUG);

  Ravelin::VectorNd go_to(6);
  static int &USE_LOCOMOTION = Utility::get_variable<int>("locomotion.active");
  if(USE_LOCOMOTION){
    static std::vector<double>
        &duty_factor = Utility::get_variable<std::vector<double> >("locomotion.duty-factor"),
        &this_gait = Utility::get_variable<std::vector<double> >("locomotion.gait");
    static double &gait_time = Utility::get_variable<double>("locomotion.gait-duration");
    static double &step_height = Utility::get_variable<double>("locomotion.step-height");
    static std::vector<Ravelin::Vector3d> footholds(0);

    // Robot attempts to align base with force and then walk along force axis
//    Ravelin::SForced lead_base_force = Ravelin::Pose3d::transform(base_link_frame,lead_force_);

//    go_to[0] += lead_base_force[0];
//    go_to[1] += lead_base_force[1];
//    go_to[5] += lead_base_force[5]*100.0;

//    OUTLOG(go_to,"go_to",logINFO);

    int NUM_FEET = feet.size();
    std::vector<Ravelin::Vector3d>
        foot_vel(NUM_FEET),
        foot_pos(NUM_FEET),
        foot_acc(NUM_FEET);

    OUTLOG(this_gait,"this_gait",logINFO);
    OUTLOG(duty_factor,"duty_factor",logINFO);

    for(int i=0,ii=0;i<NUM_EEFS;i++){
      if(is_foot[i] == 0) continue;
      foot_pos[ii] = x_des[i];
      foot_vel[ii] = xd_des[i];
      foot_acc[ii] = xdd_des[i];
      double gait_progress = t/gait_time;
      gait_progress = gait_progress - (double) ((int) gait_progress);
      ii++;
    }
      Ravelin::SVector6d goto_6d = (movement_command.rows() == 6 )? movement_command : go_to;
    goto_6d.pose = base_frame;

    int STANCE_ON_CONTACT = Utility::get_variable<int>("locomotion.stance-on-contact");
    walk_toward(goto_6d,this_gait,footholds,duty_factor,gait_time,step_height,STANCE_ON_CONTACT,feet,sum_base_velocity/ (double)base_vel_queue.size(),data->center_of_mass_x,t,foot_pos,foot_vel, foot_acc);
//    cpg_trot(go_to,this_gait,duty_factor,gait_time,step_height,foot_origin,t,foot_pos,foot_vel,foot_acc);
    for(int i=0,ii=0;i<NUM_EEFS;i++){
      if(is_foot[i] == 0) continue;
      x_des[i] = foot_pos[ii];
      xd_des[i] = foot_vel[ii];
      xdd_des[i] = foot_acc[ii];
      ii++;
    }

    boost::shared_ptr<Ravelin::Pose3d> new_gait_pose(new Ravelin::Pose3d(gait_pose->q,gait_pose->x,base_horizontal_frame));
    Utility::visualize.push_back( Pacer::VisualizablePtr( new Pose(*new_gait_pose.get(),0.25,0.2)));

//    for(int i=0,ii=0;i<NUM_EEFS;i++){
//      if(is_foot[i] == 0) continue;
//      x_des[i] = Ravelin::Pose3d::transform_point(base_frame,Ravelin::Vector3d(foot_pos[ii].data(),new_gait_pose));
//      xd_des[i] = Ravelin::Pose3d::transform_vector(base_frame,Ravelin::Vector3d(foot_vel[ii].data(),new_gait_pose));
//      xdd_des[i] = Ravelin::Pose3d::transform_vector(base_frame,Ravelin::Vector3d(foot_acc[ii].data(),new_gait_pose));
//   }
  }
  trajectory_ik(x_des,xd_des, xdd_des,data->q,q_des,qd_des,qdd_des);

  // =================== END PLANNING FUNCTIONS =====================
  // ----------------------------- STABILIZATION -------------------------------
  static int &USE_STABILIZATION = Utility::get_variable<int>("controller.stabilization.active");
  if(USE_STABILIZATION){
    static int &USE_VIIP = Utility::get_variable<int>("controller.stabilization.viip.active");
    if(USE_VIIP){
      static std::vector<double> &x_des = Utility::get_variable<std::vector<double> >("controller.stabilization.viip.desired.x");
      static std::vector<double> &xd_des = Utility::get_variable<std::vector<double> >("controller.stabilization.viip.desired.xd");
      static int &USE_DES_CONTACT = Utility::get_variable<int>("controller.stabilization.viip.des-contact");


      static std::vector<double>
          &Kp = Utility::get_variable<std::vector<double> >("controller.stabilization.viip.gains.kp"),
          &Kv = Utility::get_variable<std::vector<double> >("controller.stabilization.viip.gains.kv"),
          &Ki = Utility::get_variable<std::vector<double> >("controller.stabilization.viip.gains.ki");


        Ravelin::MatrixNd N = data->N,
                          D = data->D,
                          R = data->R;
        // Recalculate contact jacobians based on desired lift-off feet
        if(USE_DES_CONTACT){
          for(int i=0;i<NUM_EEFS;i++)
            if(eefs_[i].active && !eefs_[i].stance)
              eefs_[i].active = false;
          calc_contact_jacobians(N,D,R);
        }

        // Reset active feet
        for(int i=0;i<NUM_EEFS;i++)
          if(eefs_[i].point.size() > 0)
            eefs_[i].active = true;

        int NC = N.columns();

        if(NC > 0){
        std::vector<double> vel_base(6), pos_base(6);
        for(int i=0;i<3;i++){
          vel_base[i] = data->generalized_qd[NUM_JOINT_DOFS+i];
          vel_base[i+3] = data->generalized_qd[NUM_JOINT_DOFS+3+i];
          pos_base[i] = data->center_of_mass_x[i];
          pos_base[i+3] = data->roll_pitch_yaw[i];
        }

        Ravelin::SharedConstMatrixNd Jb = R.block(NUM_JOINT_DOFS,NDOFS,0,NC*3);
        Ravelin::SharedConstMatrixNd Jq = R.block(0,NUM_JOINT_DOFS,0,NC*3);

        Ravelin::VectorNd fb = Ravelin::VectorNd::zero(NUM_JOINT_DOFS);
        contact_jacobian_stabilizer(Jb,Jq,Kp,Kv,Ki,pos_base,x_des,vel_base,xd_des,fb);

        static int &FEEDBACK_ACCEL = Utility::get_variable<int>("controller.stabilization.viip.accel");
        if(FEEDBACK_ACCEL)
          qdd_des += fb;
        else
          ufb += fb;
        OUTLOG(fb,"viip_fb",logDEBUG);
      }
    }
  }


  // ------------------------ INVERSE DYNAMICS ---------------------------------

  static int &CONTROL_IDYN = Utility::get_variable<int>("controller.inverse-dynamics.active");
  if(CONTROL_IDYN){
    static double &dt_idyn = Utility::get_variable<double>("controller.inverse-dynamics.dt");
    static double &alpha = Utility::get_variable<double>("controller.inverse-dynamics.alpha");
    static int &USE_DES_CONTACT = Utility::get_variable<int>("controller.inverse-dynamics.des-contact");
    static int &USE_LAST_CFS = Utility::get_variable<int>("controller.inverse-dynamics.last-cfs");
    double DT = (dt_idyn == 0)? dt : dt_idyn;


    Ravelin::VectorNd cf;
    Ravelin::VectorNd id = Ravelin::VectorNd::zero(NUM_JOINT_DOFS);

    Ravelin::MatrixNd N = data->N,
                      D = data->D,
                      R = data->R;
    if(USE_DES_CONTACT){
      for(int i=0;i<NUM_EEFS;i++)
        if(eefs_[i].active && !eefs_[i].stance)
          eefs_[i].active = false;
      calc_contact_jacobians(N,D,R);
    }
    int NC = N.columns();

    bool inf_friction = true;
    Ravelin::MatrixNd MU;
    MU.set_zero(N.columns(),2);
    for(int i=0,ii=0;i<NUM_EEFS;i++)
      if(eefs_[i].active)
        for(int j=0;j<eefs_[i].point.size();j++,ii++){
          std::fill(MU.row(ii).begin(),MU.row(ii).end(),eefs_[i].mu_coulomb[j]);
          if(std::accumulate(MU.row(ii).begin(),MU.row(ii).end(),0)
             /MU.row(ii).rows() < 100.0)
            inf_friction = false;
        }

    Ravelin::VectorNd cf_init;
      cf.set_zero(NC*5);
      for(unsigned i=0,ii=0;i< eefs_.size();i++){
        if(!eefs_[i].active) continue;
        for(int j=0;j<eefs_[i].point.size();j++,ii++){
          Ravelin::Matrix3d R_foot( eefs_[i].normal[j][0], eefs_[i].normal[j][1], eefs_[i].normal[j][2],
                                      eefs_[i].tan1[j][0],   eefs_[i].tan1[j][1],   eefs_[i].tan1[j][2],
                                      eefs_[i].tan2[j][0],   eefs_[i].tan2[j][1],   eefs_[i].tan2[j][2]);
          Ravelin::Origin3d contact_impulse = Ravelin::Origin3d(R_foot.mult(eefs_[i].impulse[j],workv3_)*(dt/DT));
          cf[ii] = contact_impulse[0];
          if(contact_impulse[1] >= 0)
            cf[NC+ii] = contact_impulse[1];
          else
            cf[NC+ii+NC*2] = -contact_impulse[1];
          if(contact_impulse[2] >= 0)
            cf[NC+ii+NC] = contact_impulse[2];
          else
            cf[NC+ii+NC*3] = -contact_impulse[2];
        }
      }
      Utility::check_finite(cf);
    OUTLOG(cf,"cf_moby",logERROR);

    {
      double sum = std::accumulate(cf.segment(0,NC).begin(),cf.segment(0,NC).end(),0.0);

      OUT_LOG(logERROR) << t-dt <<",M, Sum normal force: " << sum ;
    }

    if(USE_LAST_CFS){

      cf_init = cf;
      static std::queue<Ravelin::VectorNd>
          cf_delay_queue;
      static std::queue<Ravelin::MatrixNd>
          N_delay_queue,
          D_delay_queue,
          MU_delay_queue;
      static int &FILTER_CFS = Utility::get_variable<int>("controller.inverse-dynamics.last-cfs-filter");

      if(FILTER_CFS && NC>0){

        cf_delay_queue.push(cf);
        N_delay_queue.push(N);
        D_delay_queue.push(D);
        MU_delay_queue.push(MU);
        cf = cf_delay_queue.front();
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
    } else {
      cf_init.set_zero(0);
    }


    const double h_dt = DT/dt;
    bool solve_flag = false;
    Ravelin::VectorNd fext_scaled;

    // IDYN MAXIMAL DISSIPATION MODEL
    unsigned ctl_num = 0;
#define USE_CLAWAR_MODEL
//#define USE_AP_MODEL
//#define USE_NO_SLIP_MODEL
#define USE_NO_SLIP_LCP_MODEL

    std::vector<Ravelin::VectorNd> compare_cf_vec;

#define TIMING

    OUTLOG(NC,"idyn_NC",logERROR);

    ////////////////////////// simulator DT IDYN //////////////////////////////

if(inf_friction){
#ifdef USE_NO_SLIP_MODEL
    // NO-SLIP MODEL
    {
#ifdef TIMING
    struct timeval start_t;
    struct timeval end_t;
    gettimeofday(&start_t, NULL);
#endif
    cf = cf_init;
    id.set_zero(NUM_JOINT_DOFS);
    if(inverse_dynamics_no_slip(data->generalized_qd,qdd_des,data->M,N,D,data->generalized_fext,dt,id,cf)){
      compare_cf_vec.push_back(cf.segment(0,NC));
    } else {
      compare_cf_vec.push_back(cf);
    }
    OUTLOG(id,"uff_"+std::to_string(ctl_num),logERROR);
    OUTLOG(cf,"cf_"+std::to_string(ctl_num),logERROR);
#ifdef TIMING
    gettimeofday(&end_t, NULL);
    double duration = (end_t.tv_sec - start_t.tv_sec) + (end_t.tv_usec - start_t.tv_usec) * 1E-6;
    OUTLOG(duration*1000.0,"timing_"+std::to_string(ctl_num),logERROR);
#endif
    ctl_num++;
    }
#endif
#ifdef USE_NO_SLIP_LCP_MODEL
  // NO-SLIP Fast MODEL
    {
#ifdef TIMING
    struct timeval start_t;
    struct timeval end_t;
    gettimeofday(&start_t, NULL);
#endif
    cf.set_zero(NC*5);
    id.set_zero(NUM_JOINT_DOFS);
    {
      solve_flag = inverse_dynamics_no_slip_fast(data->generalized_qd,qdd_des,data->M,N,D,data->generalized_fext,dt,id,cf,false);
      OUTLOG(id,"uff_"+std::to_string(ctl_num),logERROR);
      OUTLOG(cf,"cf_"+std::to_string(ctl_num),logERROR);
      static Ravelin::VectorNd last_cf = cf.segment(0,NC);
      if(last_cf.rows() == NC){
        Ravelin::VectorNd diff_cf  = last_cf;
        diff_cf -= cf.segment(0,NC);
        if(diff_cf.norm() > 0.01)
          OUT_LOG(logERROR) << "-- Torque chatter detected " << t;
      }
      last_cf = cf.segment(0,NC);
    }
    compare_cf_vec.push_back(cf.segment(0,NC));
#ifdef TIMING
    gettimeofday(&end_t, NULL);
    double duration = (end_t.tv_sec - start_t.tv_sec) + (end_t.tv_usec - start_t.tv_usec) * 1E-6;
    OUTLOG(duration*1000.0,"timing_"+std::to_string(ctl_num),logERROR);
#endif
    ctl_num++;
    }
#endif


} else {

#ifdef USE_CLAWAR_MODEL
    // IDYN QP
  {
#ifdef TIMING
    struct timeval start_t;
    struct timeval end_t;
    gettimeofday(&start_t, NULL);
#endif
    cf = cf_init;
    id.set_zero(NUM_JOINT_DOFS);
    if(inverse_dynamics(data->generalized_qd,qdd_des,data->M,N,D,data->generalized_fext,dt,MU,id,cf)){
      compare_cf_vec.push_back(cf.segment(0,NC));
    } else {
      compare_cf_vec.push_back(cf);
    }
    OUTLOG(id,"uff_"+std::to_string(ctl_num),logERROR);
    OUTLOG(cf,"cf_"+std::to_string(ctl_num),logERROR);
#ifdef TIMING
    gettimeofday(&end_t, NULL);
    double duration = (end_t.tv_sec - start_t.tv_sec) + (end_t.tv_usec - start_t.tv_usec) * 1E-6;
    OUTLOG(duration*1000.0,"timing_"+std::to_string(ctl_num),logERROR);
#endif
    ctl_num++;
  }
#endif


#ifdef USE_AP_MODEL
    // A-P Fast MODEL
    {
#ifdef TIMING
    struct timeval start_t;
    struct timeval end_t;
    gettimeofday(&start_t, NULL);
#endif
    cf.set_zero(NC*5);
    id.set_zero(NUM_JOINT_DOFS);
    if(NC>0){
      solve_flag = inverse_dynamics_ap(data->generalized_qd,qdd_des,data->M,N,D,data->generalized_fext,dt,MU,id,cf);
      OUTLOG(id,"uff_"+std::to_string(ctl_num),logERROR);
      OUTLOG(cf,"cf_"+std::to_string(ctl_num),logERROR);
      static Ravelin::VectorNd last_cf = cf.segment(0,NC);
      if(last_cf.rows() == NC){
        Ravelin::VectorNd diff_cf  = last_cf;
        diff_cf -= cf.segment(0,NC);
        if(diff_cf.norm() > 0.01)
          OUT_LOG(logERROR) << "-- Torque chatter detected " << t;
      }
      last_cf = cf.segment(0,NC);
    } else {
      solve_flag = inverse_dynamics(data->generalized_qd,qdd_des,data->M,N,D,data->generalized_fext,dt,MU,id,cf);
    }
    compare_cf_vec.push_back(cf.segment(0,NC));
#ifdef TIMING
    gettimeofday(&end_t, NULL);
    double duration = (end_t.tv_sec - start_t.tv_sec) + (end_t.tv_usec - start_t.tv_usec) * 1E-6;
    OUTLOG(duration*1000.0,"timing_"+std::to_string(ctl_num),logERROR);
#endif
    ctl_num++;
    }
#endif
}

{
      unsigned ii = 0;
#ifdef USE_NO_SLIP_MODEL
      OUT_LOG(logERROR) << ii << " -- USE_NO_SLIP_MODEL";
      ii++;
#endif
#ifdef USE_CLAWAR_MODEL
      OUT_LOG(logERROR) << ii << " -- USE_CLAWAR_MODEL";
      ii++;
#endif
#ifdef USE_NO_SLIP_LCP_MODEL
      OUT_LOG(logERROR) << ii << " -- USE_NO_SLIP_LCP_MODEL";
      ii++;
#endif
#ifdef USE_AP_MODEL
      OUT_LOG(logERROR) << ii << " -- USE_AP_MODEL";
      ii++;
#endif
}


    for(int i=0;i<compare_cf_vec.size();i++){
      double sum = std::accumulate(compare_cf_vec[i].begin(),compare_cf_vec[i].end(),0.0);
      OUT_LOG(logERROR) << i << ", Sum normal force: " << sum ;
    }

    uff += (id*=alpha);

    // Reset active feet
    for(int i=0;i<NUM_EEFS;i++)
      if(eefs_[i].point.size() > 0)
        eefs_[i].active = true;
  }


  // ------------------------- PROCESS FB AND FF FORCES ------------------------

  Utility::check_finite(ufb);
  Utility::check_finite(uff);
  // combine ufb and uff
  u  = ufb;
  u += uff;

  // -------------------------- LIMIT TORQUES ----------------------------

  // Enforce torque limits
  for(unsigned i=0;i< NUM_JOINT_DOFS;i++){
    // Crash if the robot is attempting to apply excessive force
//    assert(u[i] <= 2.0*torque_limits_u[i] && u[i] >= 2.0*torque_limits_l[i]);

    if(u[i] > torque_limits_u[i])
      u[i] = torque_limits_u[i];
    else if(u[i] < torque_limits_l[i])
      u[i] = torque_limits_l[i];
  }

  OUT_LOG(logINFO)<< "time = "<< t ;

  set_model_state(data->q,data->qd);

   OUT_LOG(logINFO) <<"JOINT:A\t: U\t| Q\t: des\t: err\t|Qd\t: des\t: err\t|Qdd\t: des\t: err";
   for(unsigned i=0,ii=0;i< NUM_JOINTS;i++){
     if(joints_[i])
     for(int j=0;j<joints_[i]->num_dof();j++,ii++){
     OUT_LOG(logINFO)<< std::to_string(j)+joints_[i]->id << ":"<< active_joints_[std::to_string(j)+joints_[i]->id]
               << "\t " <<  std::setprecision(3) << u[ii]
               << "\t| " << joints_[i]->q[j]
               << "\t " << q_des[ii]
               << "\t " << data->q[ii] - q_des[ii]
               << "\t| " << joints_[i]->qd[j]
               << "\t " << qd_des[ii]
               << "\t " <<  data->qd[ii] - qd_des[ii]
               << "\t| " << data->qdd[ii]
               << "\t " << qdd_des[ii]
               << "\t " <<  (data->qdd[ii] - qdd_des[ii]);
     }
   }
   OUTLOG(data->roll_pitch_yaw,"roll_pitch_yaw",logINFO);
   OUTLOG(data->zero_moment_point,"ZmP",logINFO);
   OUTLOG(data->center_of_mass_x,"CoM_x",logINFO);
   OUTLOG(data->center_of_mass_xd,"CoM_xd",logINFO);
   OUTLOG(data->center_of_mass_xdd,"CoM_xdd",logINFO);
   OUTLOG(data->q,"q",logINFO);
   OUTLOG(data->qd,"qd",logINFO);
   OUTLOG(last_qd_des,"qd_des{t-1}",logINFO);
   OUTLOG(data->qdd,"qdd",logINFO);
   OUTLOG(last_qdd_des,"qdd_des{t-1}",logINFO);
   OUTLOG(q_des,"q_des",logINFO);
   OUTLOG(qd_des,"qd_des",logINFO);
   OUTLOG(qdd_des,"qdd_des",logINFO);
   OUTLOG(data->generalized_fext,"fext",logINFO);
   OUTLOG(data->generalized_q,"generalized_q",logDEBUG);
   OUTLOG(data->generalized_qd,"generalized_qd",logDEBUG);

   last_qdd_des = qdd_des;
   last_qd_des = qd_des;
   OUTLOG(uff,"uff",logINFO);
   OUTLOG(ufb,"ufb",logINFO);
   OUTLOG(u,"u",logINFO);

   Ravelin::MatrixNd Jf;
   for(int i=0;i<NUM_EEFS;i++){
     boost::shared_ptr<Ravelin::Pose3d> event_frame(new Ravelin::Pose3d(x_des[i].pose));
     EndEffector& foot = eefs_[i];

     // Positional Correction
     Ravelin::Vector3d x_now_g = Ravelin::Pose3d::transform_point(Moby::GLOBAL,Ravelin::Vector3d(0,0,0,eefs_[i].link->get_pose()));
     Ravelin::Vector3d x_des_g = Ravelin::Pose3d::transform_point(Moby::GLOBAL,x_des[i]);
     Ravelin::Vector3d x_err  = x_des_g - x_now_g;
     OUTLOG( x_now_g,foot.id + "_x",logINFO);
     OUTLOG( x_des_g,foot.id + "_x_des",logINFO);
     OUTLOG( x_err,foot.id + "_x_err",logINFO);


     // Remove portion of foot velocity that can't be affected by corrective forces
     event_frame->x = Ravelin::Pose3d::transform_point(x_des[i].pose,Ravelin::Vector3d(0,0,0,eefs_[i].link->get_pose()));
     dbrobot_->calc_jacobian(event_frame,eefs_[i].link,Jf);
     Ravelin::SharedConstMatrixNd Jb = Jf.block(0,3,NUM_JOINTS,NDOFS);
     Ravelin::SharedConstVectorNd vb = data->generalized_qd.segment(NUM_JOINTS,NDOFS);
     Jb.mult(vb,workv3_);
     workv3_.pose = x_des[i].pose;

     // Velocity Correction
     // Need to account for base being a moving frame (but only converting as a static frame)
     Ravelin::Vector3d xd_now = (Ravelin::Pose3d::transform_vector(x_des[i].pose,eefs_[i].link->get_velocity().get_linear()) - workv3_);
     Ravelin::Vector3d xd_err = xd_des[i] - xd_now;
     OUTLOG( xd_now,foot.id + "_xd",logINFO);
     OUTLOG( xd_des[i],foot.id + "_xd_des",logINFO);
     OUTLOG( xd_err,foot.id + "_xd_err",logINFO);
   }

   int ii = 0;
   for(int i=0;i<NUM_EEFS;i++){
     OUT_LOG(logINFO) << eefs_[i].id << " contacts: " << eefs_[i].point.size();
     for(int j=0;j<eefs_[i].point.size();j++,ii++){
       OUTLOG(eefs_[i].point[j],eefs_[i].id + "_point[" + std::to_string(i) + "]",logINFO);
       OUTLOG(eefs_[i].normal[j],eefs_[i].id + "_normal[" + std::to_string(i) + "]",logINFO);
       OUTLOG(eefs_[i].impulse[j],eefs_[i].id + "_impulse[" + std::to_string(i) + "]",logINFO);
       OUTLOG(eefs_[i].mu_coulomb[j],eefs_[i].id + "_mu[" + std::to_string(i) + "]",logINFO);
     }
   }
   OUT_LOG(logINFO) << "num_contacts = " << ii;
   OUT_LOG(logINFO) << "==============================================" << std::endl;
   // -----------------------------------------------------------------------------

   assert(data->generalized_fext.norm() < 1e+6);

   q_joints.clear();
   qd_joints.clear();
   u_joints.clear();
   for(unsigned i=0,ii=0;i< NUM_JOINTS;i++){
     if(joints_[i])
     for(int j=0;j<joints_[i]->num_dof();j++,ii++){
       q_joints[std::to_string(j)+joints_[i]->id] = q_des[ii];
       qd_joints[std::to_string(j)+joints_[i]->id] = qd_des[ii];
       u_joints[std::to_string(j)+joints_[i]->id] = u[ii];
     }
   }
*/
  reset_contact();
  last_time = t;
  OUT_LOG(logDEBUG) << "<< Controller::control(.)";
}
// ===========================  END CONTROLLER  ===============================
// ============================================================================
