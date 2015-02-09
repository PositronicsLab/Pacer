/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/controller.h>
#include <Pacer/utilities.h>
#include <sys/time.h>

using namespace Pacer;


// ============================================================================
// =========================== Begin Robot Controller =========================
// ============================================================================

void Controller::control(double t,
                                      const Ravelin::VectorNd& generalized_q_in,
                                      const Ravelin::VectorNd& generalized_qd_in,
                                      const Ravelin::VectorNd& generalized_qdd_in,
                                      const Ravelin::VectorNd& generalized_fext_in,
                                      Ravelin::VectorNd& q_des,
                                      Ravelin::VectorNd& qd_des,
                                      Ravelin::VectorNd& qdd_des,
                                      Ravelin::VectorNd& u){
  // Import Robot Data
  static double last_time = -0.001;
  const double dt = t - last_time;

  // Select end effectors that are feet and set them as active contacts
  static std::vector<int>
      &is_foot = CVarUtils::GetCVarRef<std::vector<int> >("init.end-effector.foot");

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
    visualize.push_back(Point(Ravelin::Pose3d::transform_point(Moby::GLOBAL,Ravelin::Vector3d(0,0,0,eefs_[i].link->get_pose())),Ravelin::Vector3d(1,1,0),0.2));
    visualize.push_back(Point(Ravelin::Pose3d::transform_point(Moby::GLOBAL,eefs_[i].origin),Ravelin::Vector3d(1,0,0),0.2));
  }

  std::vector<EndEffector*> feet;
  for(unsigned i=0,ii=0;i< NUM_EEFS;i++){
    if(is_foot[i] == 0) continue;
    feet.push_back(&eefs_[i]);
    feet[ii]->origin.pose = base_horizontal_frame;

  // =================== END UPDATE & INIT ROBOT MODEL & CONTROL VARS =====================

  // =================== BEGIN PLANNING FUNCTIONS =====================


    visualize.push_back(Point(Ravelin::Pose3d::transform_point(Moby::GLOBAL,feet[ii]->origin),
                    Ravelin::Vector3d(1,0,0),
                    0.1));

    ii++;
  }

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

  static std::vector<std::string>
     &joint_names = CVarUtils::GetCVarRef<std::vector<std::string> >("init.joint.id");

  Ravelin::VectorNd go_to(6);
  static int &USE_LOCOMOTION = CVarUtils::GetCVarRef<int>("locomotion.active");
  if(USE_LOCOMOTION){
    static std::vector<double>
        &duty_factor = CVarUtils::GetCVarRef<std::vector<double> >("locomotion.duty-factor"),
        &this_gait = CVarUtils::GetCVarRef<std::vector<double> >("locomotion.gait");
    static double &gait_time = CVarUtils::GetCVarRef<double>("locomotion.gait-duration");
    static double &step_height = CVarUtils::GetCVarRef<double>("locomotion.step-height");
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

    int STANCE_ON_CONTACT = CVarUtils::GetCVarRef<int>("locomotion.stance-on-contact");
    walk_toward(goto_6d,this_gait,footholds,duty_factor,gait_time,step_height,STANCE_ON_CONTACT,feet,sum_base_velocity/ (double)base_vel_queue.size(),data->center_of_mass_x,t,foot_pos,foot_vel, foot_acc);
//    cpg_trot(go_to,this_gait,duty_factor,gait_time,step_height,foot_origin,t,foot_pos,foot_vel,foot_acc);
    for(int i=0,ii=0;i<NUM_EEFS;i++){
      if(is_foot[i] == 0) continue;
      x_des[i] = foot_pos[ii];
      xd_des[i] = foot_vel[ii];
      xdd_des[i] = foot_acc[ii];
      ii++;
    }
  }
  trajectory_ik(x_des,xd_des, xdd_des,data->q,q_des,qd_des,qdd_des);

  // =================== END PLANNING FUNCTIONS =====================

  // Find center of stance feet
  {
    static Ravelin::Vector3d sum_center_of_feet(0,0,0,environment_frame);
    static std::queue<Ravelin::Vector3d> center_of_feet_queue;

    Ravelin::Vector3d CoF_x;
    CoF_x.set_zero();
    CoF_x.pose = environment_frame;

    int ii = 0;
    for(int i=0;i<NUM_EEFS;i++){
      if(is_foot[i] == 0 || !eefs_[i].stance) continue;
      workv3_ = Ravelin::Pose3d::transform_point(environment_frame,Ravelin::Vector3d(0,0,0,eefs_[i].link->get_pose()));
      visualize.push_back(Point(workv3_,
                      Ravelin::Vector3d(1,0,1),
                      0.5));
      CoF_x += workv3_;
      ii++;
    }
    CoF_x /= (double)ii;

  visualize.push_back(Point(CoF_x,
                  Ravelin::Vector3d(1,0.5,0)));

    center_of_feet_queue.push(CoF_x);
    sum_center_of_feet += CoF_x;
    if(center_of_feet_queue.size() > 100){
       sum_center_of_feet -= center_of_feet_queue.front();
       center_of_feet_queue.pop();
    }

    workv3_ = center_of_feet_x;
    center_of_feet_x = sum_center_of_feet /  (double) center_of_feet_queue.size();
    workv3_.pose = center_of_feet_x.pose;
    center_of_feet_xd = (center_of_feet_x - workv3_)*dt;
    if(center_of_feet_queue.size() == 0 || ii == 0)
      center_of_feet_x = data->center_of_mass_x;

    visualize.push_back(Point(center_of_feet_x,
                  Ravelin::Vector3d(1,0,0)));

    OUTLOG(CoF_x,"CoF_x (now)",logDEBUG);

    OUTLOG(center_of_feet_x,"center_of_feet_x (avg 1 sec)",logDEBUG);
  }


  // ----------------------------- STABILIZATION -------------------------------
  static int &USE_STABILIZATION = CVarUtils::GetCVarRef<int>("controller.stabilization.active");
  if(USE_STABILIZATION){
    static int &USE_VIIP = CVarUtils::GetCVarRef<int>("controller.stabilization.viip.active");
    if(USE_VIIP){
      static std::vector<double> &x_des = CVarUtils::GetCVarRef<std::vector<double> >("controller.stabilization.viip.desired.x");
      static std::vector<double> &xd_des = CVarUtils::GetCVarRef<std::vector<double> >("controller.stabilization.viip.desired.xd");
      static int &USE_DES_CONTACT = CVarUtils::GetCVarRef<int>("controller.stabilization.viip.des-contact");


      static std::vector<double>
          &Kp = CVarUtils::GetCVarRef<std::vector<double> >("controller.stabilization.viip.gains.kp"),
          &Kv = CVarUtils::GetCVarRef<std::vector<double> >("controller.stabilization.viip.gains.kv"),
          &Ki = CVarUtils::GetCVarRef<std::vector<double> >("controller.stabilization.viip.gains.ki");


        for(int i=0;i<3;i++){
          x_des[i] = center_of_feet_x[i];
  //        xd_des[i] = center_of_feet_xd[i];
        }

  //      xd_des[0] = 0;

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

        static int &FEEDBACK_ACCEL = CVarUtils::GetCVarRef<int>("controller.stabilization.viip.accel");
        if(FEEDBACK_ACCEL)
          qdd_des += fb;
        else
          ufb += fb;
        OUTLOG(fb,"viip_fb",logDEBUG);
      }
    }
  }

  // --------------------------- ERROR FEEDBACK --------------------------------

  for(int i=0,ii=0;i<NUM_JOINTS;i++){
    if(joints_[i])
    for(int j=0;j<joints_[i]->num_dof();j++,ii++){
      if(!get_active_joints()[std::to_string(j)+joints_[i]->id]){
        q_des[ii] = get_q0()[std::to_string(j)+joints_[i]->id];
        qd_des[ii] = 0;
        qdd_des[ii] = 0;
      }
    }
  }

  static int &ERROR_FEEDBACK = CVarUtils::GetCVarRef<int>("controller.error-feedback.active");
  if (ERROR_FEEDBACK){
    // --------------------------- JOINT FEEDBACK ------------------------------
    static int &JOINT_FEEDBACK = CVarUtils::GetCVarRef<int>("controller.error-feedback.configuration-space.active");
    if(JOINT_FEEDBACK){
      static int &FEEDBACK_ACCEL = CVarUtils::GetCVarRef<int>("controller.error-feedback.configuration-space.accel");

      static boost::shared_ptr<JointPID> pid;
      if(!pid){
         if(FEEDBACK_ACCEL)
           pid = boost::shared_ptr<JointPID>( new JointPID(std::string("controller.error-feedback.configuration-space.accel")));
         else
           pid = boost::shared_ptr<JointPID>( new JointPID(std::string("controller.error-feedback.configuration-space.force")));

         controllers.push_back(pid->ptr());
      }
      pid->q = data->q;
      pid->qd = data->qd;
      pid->q_des = q_des;
      pid->qd_des = qd_des;
      pid->joint_names = get_joint_names();
      pid->update();
      if(FEEDBACK_ACCEL)
        qdd_des += pid->value;
      else
        ufb += pid->value;
      OUTLOG(ufb,"controller-ufb",logERROR);

      }

//    BOOST_FOREACH(boost::shared_ptr<ControllerModule> m,controllers){
//      if(m->type == CONTROLLER){
////        boost::shared_ptr<ControllerModule> c = std::make_shared<ControllerModule>();
////        c = boost::dynamic_pointer_cast<ControllerModule>(m);
//        if(false)
//          qdd_des += m->value;
//        else
//          ufb += m->value;
//        OUTLOG(ufb,"controller-ufb",logERROR);
//      }
//    }


    // --------------------------- WORKSPACE FEEDBACK --------------------------
    static int &WORKSPACE_FEEDBACK = CVarUtils::GetCVarRef<int>("controller.error-feedback.operational-space.active");
    if(WORKSPACE_FEEDBACK){
      // CURRENTLY THIS IS ONLY FORCE
      // BUT IT CAN BE ACCELERATIONS TOO
      static int &FEEDBACK_ACCEL = CVarUtils::GetCVarRef<int>("controller.error-feedback.operational-space.accel");
      std::vector<Ravelin::Matrix3d> W(boost::assign::list_of(Ravelin::Matrix3d::identity())(Ravelin::Matrix3d::identity())(Ravelin::Matrix3d::identity())(Ravelin::Matrix3d::identity()).convert_to_container<std::vector<Ravelin::Matrix3d> >() );
      static std::vector<double>
          &Kp = CVarUtils::GetCVarRef<std::vector<double> >("controller.error-feedback.operational-space.gains.kp"),
          &Kv = CVarUtils::GetCVarRef<std::vector<double> >("controller.error-feedback.operational-space.gains.kv"),
          &Ki = CVarUtils::GetCVarRef<std::vector<double> >("controller.error-feedback.operational-space.gains.ki");

      Ravelin::VectorNd fb = Ravelin::VectorNd::zero(NUM_JOINT_DOFS);
      eef_stiffness_fb(Kp,Kv,Ki,x_des,xd_des,data->q,data->qd,fb);

      if(FEEDBACK_ACCEL)
        qdd_des += fb;
      else
        ufb += fb;
    }
  }

  // ------------------------ INVERSE DYNAMICS ---------------------------------

  static int &CONTROL_IDYN = CVarUtils::GetCVarRef<int>("controller.inverse-dynamics.active");
  if(CONTROL_IDYN){
    static double &dt_idyn = CVarUtils::GetCVarRef<double>("controller.inverse-dynamics.dt");
    static double &alpha = CVarUtils::GetCVarRef<double>("controller.inverse-dynamics.alpha");
    static int &USE_DES_CONTACT = CVarUtils::GetCVarRef<int>("controller.inverse-dynamics.des-contact");
    static int &USE_LAST_CFS = CVarUtils::GetCVarRef<int>("controller.inverse-dynamics.last-cfs");
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
      static int &FILTER_CFS = CVarUtils::GetCVarRef<int>("controller.inverse-dynamics.last-cfs-filter");

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
   OUTLOG(center_of_feet_x,"center_of_feet_x",logINFO);
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

   reset_contact();
   last_time = t;
}
// ===========================  END CONTROLLER  ===============================
// ============================================================================
