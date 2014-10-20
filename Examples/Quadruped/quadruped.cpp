#include <quadruped.h>
#include <utilities.h>

#ifdef VISUALIZE_MOBY
#ifdef APPLE
# include <OpenGL/gl.h>
# include <OpenGL/glu.h>
# include <GLUT/glut.h>
#else
# include <GL/gl.h>
# include <GL/glu.h>
# include <GL/glut.h>
#endif
#endif

// ============================================================================
// =========================== Begin Robot Controller =========================
// ============================================================================

Ravelin::VectorNd& Quadruped::control(double t,
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
  new_data.generalized_q = generalized_q_in;
  new_data.generalized_qd = generalized_qd_in;
  new_data.generalized_qdd = generalized_qdd_in;
  new_data.generalized_fext = generalized_fext_in;

  // Set Robot Data in robot
  update();

  data = &new_data;

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

  Ravelin::VectorNd
      os_velocity(NUM_EEFS*3 + 6);

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
#  ifdef VISUALIZE_MOBY
    workv3_ = Ravelin::Pose3d::transform_point(Moby::GLOBAL,Ravelin::Vector3d(0,0,0,eefs_[i].link->get_pose()));
    visualize_ray(workv3_,workv3_,Ravelin::Vector3d(1,1,0),0.2,sim);
    workv3_ = Ravelin::Pose3d::transform_point(Moby::GLOBAL,eefs_[i].origin);
    visualize_ray(workv3_,workv3_,Ravelin::Vector3d(1,0,0),0.2,sim);
#  endif
  }

  static std::vector<int>
      &is_foot = CVarUtils::GetCVarRef<std::vector<int> >("quadruped.init.end-effector.foot");

  std::vector<EndEffector*> feet;
  for(unsigned i=0,ii=0;i< NUM_EEFS;i++){
    if(is_foot[i] == 0) continue;
    feet.push_back(&eefs_[i]);
    feet[ii]->origin.pose = base_horizontal_frame;

#ifdef VISUALIZE_MOBY
    visualize_ray(  Ravelin::Pose3d::transform_point(Moby::GLOBAL,feet[ii]->origin),
                    Ravelin::Pose3d::transform_point(Moby::GLOBAL,feet[ii]->origin),
                    Ravelin::Vector3d(1,0,0),
                    0.1,
                    sim
                  );
#endif
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
     &joint_names = CVarUtils::GetCVarRef<std::vector<std::string> >("quadruped.init.joint.id");

  Ravelin::VectorNd go_to(6);
  static int &USE_LOCOMOTION = CVarUtils::GetCVarRef<int>("quadruped.locomotion.active");
  if(USE_LOCOMOTION){
    static std::vector<double>
        &patrol_points = CVarUtils::GetCVarRef<std::vector<double> >("quadruped.locomotion.patrol"),
        &goto_command = CVarUtils::GetCVarRef<std::vector<double> >("quadruped.locomotion.command"),
        &goto_point = CVarUtils::GetCVarRef<std::vector<double> >("quadruped.locomotion.point"),
        &duty_factor = CVarUtils::GetCVarRef<std::vector<double> >("quadruped.locomotion.duty-factor"),
        &this_gait = CVarUtils::GetCVarRef<std::vector<double> >("quadruped.locomotion.gait");
    static int &HOLONOMIC = CVarUtils::GetCVarRef<int>("quadruped.locomotion.holonomic");
    static double &gait_time = CVarUtils::GetCVarRef<double>("quadruped.locomotion.gait-duration");
    static double &step_height = CVarUtils::GetCVarRef<double>("quadruped.locomotion.step-height");
    static std::vector<Ravelin::Vector3d> footholds(0);
    OUTLOG(goto_command ,"goto_command",logINFO);

    go_to = Ravelin::VectorNd(goto_command.size(),&goto_command[0]);

    // FOOTHOLDS

    /// HANDLE WAYPOINTS
    if(patrol_points.size() >= 4){
      int num_waypoints = patrol_points.size()/2;
      static int patrol_index = 0;
      static Ravelin::Vector3d
          next_waypoint(patrol_points[patrol_index*2],patrol_points[patrol_index*2+1],data->center_of_mass_x[2],environment_frame);
      next_waypoint[2] = data->center_of_mass_x[2];

      double distance_to_wp = (next_waypoint - data->center_of_mass_x).norm();

      if( distance_to_wp < 0.025){
        OUT_LOG(logDEBUG1) << "waypoint reached, incrementing waypoint.";
        OUTLOG(next_waypoint,"this_wp",logDEBUG1);
        OUTLOG(next_waypoint,"center_of_mass_x",logDEBUG1);

        patrol_index = (patrol_index+1) % num_waypoints;

        next_waypoint = Ravelin::Vector3d(patrol_points[patrol_index*2],patrol_points[patrol_index*2+1],data->center_of_mass_x[2],environment_frame);
      }
# ifdef VISUALIZE_MOBY
      OUT_LOG(logDEBUG1) << "num_wps = " << num_waypoints;
      OUT_LOG(logDEBUG1) << "distance_to_wp = " << distance_to_wp;
      OUT_LOG(logDEBUG1) << "patrol_index = " << patrol_index;
    visualize_ray(  next_waypoint,
                    center_of_mass_x,
                    Ravelin::Vector3d(1,0.5,0),
                    sim
                  );
    OUTLOG(next_waypoint,"next_wp",logDEBUG1);

    for(int i=0;i<num_waypoints;i++){
      Ravelin::Vector3d wp(patrol_points[i*2],patrol_points[i*2+1],next_waypoint[2],environment_frame);
      OUTLOG(wp,"wp",logDEBUG1);
      visualize_ray(  wp,
                      wp,
                      Ravelin::Vector3d(1,0.5,0),
                      1.0,
                      sim
                    );
    }
# endif
      goto_point.resize(2);
      goto_point[0] = next_waypoint[0];
      goto_point[1] = next_waypoint[1];
    }

    if(goto_point.size() == 2){
      Ravelin::Vector3d goto_direction =
          Ravelin::Vector3d(goto_point[0],goto_point[1],0,environment_frame)
          - Ravelin::Vector3d(data->center_of_mass_x[0],data->center_of_mass_x[1],0,environment_frame);
      goto_direction = Ravelin::Pose3d::transform_vector(base_horizontal_frame,goto_direction);
      goto_direction.normalize();

      double angle_to_goal = atan2(goto_direction[1],goto_direction[0]);
      if(fabs(angle_to_goal) < M_PI_8){
        if(HOLONOMIC){
          go_to[1] = goto_direction[1]*goto_command[0];
          // goal-centric coords
          go_to[0] =-goto_direction[1]*goto_command[1];
          go_to[2] = goto_direction[0]*goto_command[1];
        }
        go_to[0] = goto_direction[0]*goto_command[0];
        go_to[5] = angle_to_goal/gait_time;
      } else {
        go_to[5] = Utility::sign(angle_to_goal)*1.5;
        if(!HOLONOMIC){
          go_to[0] = 0;
          go_to[1] = 0;
        } else {
          go_to[0] = goto_direction[0]*goto_command[0];
          go_to[1] = goto_direction[1]*goto_command[0];
          // goal-centric coords
          go_to[0] =-goto_direction[1]*goto_command[1];
          go_to[2] = goto_direction[0]*goto_command[1];
        }
      }
    }

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
//      foot_pos[ii] = Ravelin::Pose3d::transform_point(base_horizontal_frame,x_des[i]);
//      foot_vel[ii] = Ravelin::Pose3d::transform_vector(base_horizontal_frame,xd_des[i]);
//      foot_acc[ii] = Ravelin::Pose3d::transform_vector(base_horizontal_frame,xdd_des[i]);
      double gait_progress = t/gait_time;
      gait_progress = gait_progress - (double) ((int) gait_progress);
      ii++;
    }
    Ravelin::SVector6d goto_6d = go_to;
    goto_6d.pose = base_frame;

    int STANCE_ON_CONTACT = CVarUtils::GetCVarRef<int>("quadruped.locomotion.stance-on-contact");
    walk_toward(goto_6d,this_gait,footholds,duty_factor,gait_time,step_height,STANCE_ON_CONTACT,feet,sum_base_velocity/ (double)base_vel_queue.size(),data->center_of_mass_x,t,foot_pos,foot_vel, foot_acc);
//    cpg_trot(go_to,this_gait,duty_factor,gait_time,step_height,foot_origin,t,foot_pos,foot_vel,foot_acc);
    for(int i=0,ii=0;i<NUM_EEFS;i++){
      if(is_foot[i] == 0) continue;
      x_des[i] = foot_pos[ii];
      xd_des[i] = foot_vel[ii];
      xdd_des[i] = foot_acc[ii];
//      x_des[i] = Ravelin::Pose3d::transform_point(base_frame,foot_pos[ii]);
//      xd_des[i] = Ravelin::Pose3d::transform_vector(base_frame,foot_vel[ii]);
//      xdd_des[i] = Ravelin::Pose3d::transform_vector(base_frame,foot_acc[ii]);
      ii++;
    }
  }
  trajectory_ik(x_des,xd_des, xdd_des,data->q,q_des,qd_des,qdd_des);

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
#ifdef VISUALIZE_MOBY
      visualize_ray(  workv3_,
                      workv3_,
                      Ravelin::Vector3d(1,0,1),
                      2,
                      sim
                    );
#endif
      CoF_x += workv3_;
      ii++;
    }
    CoF_x /= (double)ii;

#ifdef VISUALIZE_MOBY
  visualize_ray(  CoF_x,
                  CoF_x,
                  Ravelin::Vector3d(1,0.5,0),
                  1,
                  sim
                );
#endif

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
#ifdef VISUALIZE_MOBY
  visualize_ray(  center_of_feet_x,
                  center_of_feet_x,
                  Ravelin::Vector3d(1,0,0),
                  1,
                  sim
                );
#endif
  OUTLOG(CoF_x,"CoF_x (now)",logDEBUG);
  OUTLOG(center_of_feet_x,"center_of_feet_x (avg 1 sec)",logDEBUG);
  }

  // ----------------------------- STABILIZATION -------------------------------
  static int &USE_STABILIZATION = CVarUtils::GetCVarRef<int>("quadruped.stabilization.active");
  if(USE_STABILIZATION){
    static int &USE_VIIP = CVarUtils::GetCVarRef<int>("quadruped.stabilization.viip.active");
    if(USE_VIIP){
      static std::vector<double> &x_des = CVarUtils::GetCVarRef<std::vector<double> >("quadruped.stabilization.viip.desired.x");
      static std::vector<double> &xd_des = CVarUtils::GetCVarRef<std::vector<double> >("quadruped.stabilization.viip.desired.xd");
      static int &USE_DES_CONTACT = CVarUtils::GetCVarRef<int>("quadruped.stabilization.viip.des-contact");


      static std::vector<double>
          &Kp = CVarUtils::GetCVarRef<std::vector<double> >("quadruped.stabilization.viip.gains.kp"),
          &Kv = CVarUtils::GetCVarRef<std::vector<double> >("quadruped.stabilization.viip.gains.kv"),
          &Ki = CVarUtils::GetCVarRef<std::vector<double> >("quadruped.stabilization.viip.gains.ki");

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

      Ravelin::VectorNd fb = Ravelin::VectorNd::zero(NUM_JOINT_DOFS);
      contact_jacobian_stabilizer(R,Kp,Kv,Ki,x_des,xd_des,fb);

      static int &FEEDBACK_ACCEL = CVarUtils::GetCVarRef<int>("quadruped.stabilization.viip.accel");
      if(FEEDBACK_ACCEL)
        qdd_des += fb;
      else
        ufb += fb;

      OUTLOG(fb,"viip_fb",logDEBUG);
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

  static int &ERROR_FEEDBACK = CVarUtils::GetCVarRef<int>("quadruped.error-feedback.active");
  if (ERROR_FEEDBACK){
    // --------------------------- JOINT FEEDBACK ------------------------------
    static int &JOINT_FEEDBACK = CVarUtils::GetCVarRef<int>("quadruped.error-feedback.configuration-space.active");
    if(JOINT_FEEDBACK){
      static int &FEEDBACK_ACCEL = CVarUtils::GetCVarRef<int>("quadruped.error-feedback.configuration-space.accel");

      static std::vector<double>
          &Kp = CVarUtils::GetCVarRef<std::vector<double> >("quadruped.error-feedback.configuration-space.gains.kp"),
          &Kv = CVarUtils::GetCVarRef<std::vector<double> >("quadruped.error-feedback.configuration-space.gains.kv"),
          &Ki = CVarUtils::GetCVarRef<std::vector<double> >("quadruped.error-feedback.configuration-space.gains.ki");

      static std::map<std::string, Gains> gains;
      for(int i=0,ii=0;i<NUM_JOINTS;i++){
        if(joints_[i])
        for(int j=0;j<joints_[i]->num_dof();j++,ii++){
          // NOTE: Ordered by input joint_names, NOT class joint_names_
          gains[joint_names[ii]].kp = Kp[ii];
          gains[joint_names[ii]].kv = Kv[ii];
          gains[joint_names[ii]].ki = Ki[ii];
        }
      }

      Ravelin::VectorNd fb = Ravelin::VectorNd::zero(NUM_JOINT_DOFS);
      PID::control(q_des, qd_des,data->q,data->qd,joint_names_, gains,fb);

      if(FEEDBACK_ACCEL)
        qdd_des += fb;
      else
        ufb += fb;
    }

    // --------------------------- WORKSPACE FEEDBACK --------------------------
    static int &WORKSPACE_FEEDBACK = CVarUtils::GetCVarRef<int>("quadruped.error-feedback.operational-space.active");
    if(WORKSPACE_FEEDBACK){
      // CURRENTLY THIS IS ONLY FORCE
      // BUT IT CAN BE ACCELERATIONS TOO
      static int &FEEDBACK_ACCEL = CVarUtils::GetCVarRef<int>("quadruped.error-feedback.operational-space.accel");
      std::vector<Ravelin::Matrix3d> W(boost::assign::list_of(Ravelin::Matrix3d::identity())(Ravelin::Matrix3d::identity())(Ravelin::Matrix3d::identity())(Ravelin::Matrix3d::identity()).convert_to_container<std::vector<Ravelin::Matrix3d> >() );
      static std::vector<double>
          &Kp = CVarUtils::GetCVarRef<std::vector<double> >("quadruped.error-feedback.operational-space.gains.kp"),
          &Kv = CVarUtils::GetCVarRef<std::vector<double> >("quadruped.error-feedback.operational-space.gains.kv"),
          &Ki = CVarUtils::GetCVarRef<std::vector<double> >("quadruped.error-feedback.operational-space.gains.ki");

      Ravelin::VectorNd fb = Ravelin::VectorNd::zero(NUM_JOINT_DOFS);
      eef_stiffness_fb(Kp,Kv,Ki,x_des,xd_des,data->q,data->qd,fb);

      if(FEEDBACK_ACCEL)
        qdd_des += fb;
      else
        ufb += fb;
    }
  }

  // ------------------------ INVERSE DYNAMICS ---------------------------------

  static int &CONTROL_IDYN = CVarUtils::GetCVarRef<int>("quadruped.inverse-dynamics.active");
  if(CONTROL_IDYN){
    static double &dt_idyn = CVarUtils::GetCVarRef<double>("quadruped.inverse-dynamics.dt");
    static double &alpha = CVarUtils::GetCVarRef<double>("quadruped.inverse-dynamics.alpha");
    static int &USE_DES_CONTACT = CVarUtils::GetCVarRef<int>("quadruped.inverse-dynamics.des-contact");
    static int &USE_LAST_CFS = CVarUtils::GetCVarRef<int>("quadruped.inverse-dynamics.last-cfs");
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

    Ravelin::MatrixNd MU;
    MU.set_zero(N.columns(),2);
    for(int i=0,ii=0;i<NUM_EEFS;i++)
      if(eefs_[i].active)
        for(int j=0;j<eefs_[i].point.size();j++,ii++)
          std::fill(MU.row(ii).begin(),MU.row(ii).end(),eefs_[i].mu_coulomb[j]);

    if(USE_LAST_CFS){
      int NC = N.columns();
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
    } else {
      cf.set_zero(0);
    }

    // Reset active feet
    for(int i=0;i<NUM_EEFS;i++)
      if(eefs_[i].point.size() > 0)
        eefs_[i].active = true;

#ifdef TIMING
    std::clock_t start = std::clock();
#endif
    Ravelin::VectorNd fext_scaled;
    if(inverse_dynamics(data->generalized_qd,qdd_des,data->M,N,D,(fext_scaled = data->generalized_fext)*=(dt/DT),DT,MU,id,cf))
      uff += (id*=alpha);
#ifdef TIMING
    // Milliseconds
    OUTLOG(((std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000)),"idyn_timing",logINFO);
#endif
  }

  // ------------------------- PROCESS FB AND FF FORCES ------------------------

  Utility::check_finite(ufb);
  Utility::check_finite(uff);
  // combine ufb and uff
  u = ufb;
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

#ifndef NDEBUG
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
   OUTLOG(data->qdd,"qdd",logINFO);
   OUTLOG(q_des,"q_des",logINFO);
   OUTLOG(qd_des,"qd_des",logINFO);
   OUTLOG(qdd_des,"qdd_des",logINFO);
   OUTLOG(data->generalized_fext,"fext",logINFO);
   OUTLOG(data->generalized_q,"generalized_q",logDEBUG);
   OUTLOG(data->generalized_qd,"generalized_qd",logDEBUG);

   OUTLOG(uff,"uff",logINFO);
   OUTLOG(ufb,"ufb",logINFO);
   OUTLOG(u,"u",logINFO);

   Ravelin::MatrixNd Jf;
   for(int i=0;i<NUM_EEFS;i++){
     boost::shared_ptr<Ravelin::Pose3d> event_frame(new Ravelin::Pose3d(x_des[i].pose));
     EndEffector& foot = eefs_[i];

     // Positional Correction
     Ravelin::Vector3d x_des_now = Ravelin::Pose3d::transform_point(x_des[i].pose,Ravelin::Vector3d(0,0,0,eefs_[i].link->get_pose()));
     Ravelin::Vector3d x_err  = x_des[i] - x_des_now;
     OUTLOG( x_des_now,foot.id + "_x",logINFO);
     OUTLOG( x_des[i],foot.id + "_x_des",logINFO);
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
     Ravelin::Vector3d xd_des_now = (Ravelin::Pose3d::transform_vector(x_des[i].pose,eefs_[i].link->get_velocity().get_linear()) - workv3_);
     Ravelin::Vector3d xd_err = xd_des[i] - xd_des_now;
     OUTLOG( xd_des_now,foot.id + "_xd",logINFO);
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
     }
   }
   OUT_LOG(logINFO) << "num_contacts = " << ii;
   OUT_LOG(logINFO) << "==============================================" << std::endl;
   // -----------------------------------------------------------------------------
#endif

   assert(data->generalized_fext.norm() < 1e+6);

   reset_contact();
   last_time = t;

   return u;
}
// ===========================  END CONTROLLER  ===============================
// ============================================================================





// ============================================================================
// ===========================  BEGIN ROBOT INIT  =============================
#if defined(VISUALIZE_MOBY) && defined(USE_GLCONSOLE)
# include <thread>
# include <GLConsole/GLConsole.h>
  GLConsole theConsole;
  extern void init_glconsole();
  std::thread * tglc;
#endif

#include <Moby/XMLReader.h>

void Quadruped::init(){

#if defined(VISUALIZE_MOBY) && defined(USE_GLCONSOLE)
   tglc = new std::thread(init_glconsole);
#endif
  // ================= LOAD SCRIPT DATA ==========================
  load_variables("INIT/startup.xml");
  std::string robot_start_file = CVarUtils::GetCVarRef<std::string>("robot");
  std::cerr << "Using Robot: " << robot_start_file << std::endl;
  load_variables("INIT/startup-"+robot_start_file+".xml");

  // ================= SETUP LOGGING ==========================

  std::string LOG_TYPE = CVarUtils::GetCVarRef<std::string>("logging");

  std::cerr << LOG_TYPE << std::endl;
  FILELog::ReportingLevel() =
      FILELog::FromString( (!LOG_TYPE.empty()) ? LOG_TYPE : "INFO");

  OUT_LOG(logDEBUG1) << "Log Type : " << LOG_TYPE;
  OUT_LOG(logDEBUG1) << "logDEBUG1";
  OUT_LOG(logINFO) << "logINFO";
  OUT_LOG(logDEBUG) << "logDEBUG";
  OUT_LOG(logDEBUG1) << "logDEBUG1";

  // ================= BUILD ROBOT ==========================
  /// The map of objects read from the simulation XML file
  std::map<std::string, Moby::BasePtr> READ_MAP;
  READ_MAP = Moby::XMLReader::read(std::string("MODELS/"+robot_start_file+".xml"));
  for (std::map<std::string, Moby::BasePtr>::const_iterator i = READ_MAP.begin();
       i !=READ_MAP.end(); i++)
  {
    // find the robot reference
    if (!abrobot_)
    {
      abrobot_ = boost::dynamic_pointer_cast<Moby::RCArticulatedBody>(i->second);
    }
  }

  compile();

  // ================= SET UP END EFFECTORS ==========================

  eef_names_
      = CVarUtils::GetCVarRef<std::vector<std::string> >("quadruped.init.end-effector.id");

  std::vector<double> &eefs_start
      = CVarUtils::GetCVarRef<std::vector<double> >("quadruped.init.end-effector.x");

  static std::vector<std::string>
     &joint_names = CVarUtils::GetCVarRef<std::vector<std::string> >("quadruped.init.joint.id");

 static std::vector<double>
    &joints_start = CVarUtils::GetCVarRef<std::vector<double> >("quadruped.init.joint.q"),
    &torque_limits = CVarUtils::GetCVarRef<std::vector<double> >("quadruped.init.joint.max-torque"),
    &base_start = CVarUtils::GetCVarRef<std::vector<double> >("quadruped.init.base.x");

 static std::vector<int>
    &active_joints = CVarUtils::GetCVarRef<std::vector<int> >("quadruped.init.joint.active");

 const double* data = &base_start.front();
 displace_base_link = Ravelin::SVector6d(data);

 OUTLOG(joint_names,"joint_names",logDEBUG1);
 OUTLOG(joints_start,"joints_start",logDEBUG1);

 // MAKE SURE DATA PARSED PROPERLY

// assert(joint_names.size() == joints_.size());
 assert(joint_names.size() == joints_start.size());
 assert(joint_names.size() == torque_limits.size());

  std::map<std::string, double> torque_limits_;
  for(int i=0,ii=0;i<NUM_JOINTS;i++){
    if(joints_[i])
    for(int j=0;j<joints_[i]->num_dof();j++,ii++){
      OUT_LOG(logDEBUG) << joint_names[ii] << " " << ((active_joints[ii] == 0)? "false":"true") << std::endl;
      active_joints_[joint_names[ii]] = (active_joints[ii] == 0)? false:true;
      q0_[joint_names[ii]] = joints_start[ii];
      torque_limits_[joint_names[ii]] = torque_limits[ii];
    }
  }

  // push into robot
  torque_limits_l.resize(NUM_JOINT_DOFS);
  torque_limits_u.resize(NUM_JOINT_DOFS);
  for(int i=0,ii=0;i<NUM_JOINTS;i++){
    if(joints_[i])
    for(int j=0;j<joints_[i]->num_dof();j++,ii++){
//      assert(joint_names[ii].substr(0,2).compare(joints_[ii-j]->id.substr(0,2)) &&
//             joint_names[ii].substr(joint_names[ii].size()-2,1).compare(joints_[ii-j]->id.substr(joints_[ii-j]->id.size()-2,1)));
      OUT_LOG(logINFO)<< "torque_limit: " << joints_[ii-j]->id << " = " <<  torque_limits_[joint_names[ii]];
      torque_limits_l[ii] = -torque_limits_[std::to_string(j)+joints_[i]->id];
      torque_limits_u[ii] =  torque_limits_[std::to_string(j)+joints_[i]->id];
    }
  }
  OUTLOG(torque_limits_l,"torque_limits_l",logDEBUG1);
  OUTLOG(torque_limits_u,"torque_limits_u",logDEBUG1);

  // Initialize Foot Structures
  OUT_LOG(logINFO)<< eef_names_.size() << " end effectors LISTED:" ;
  for(unsigned j=0;j<eef_names_.size();j++){
    for(unsigned i=0;i<links_.size();i++){
      if(eef_names_[j].compare(links_[i]->id) == 0){
        OUT_LOG(logINFO)<< eef_names_[j] << " FOUND!";
        workv3_ = Ravelin::Vector3d(eefs_start[j*3],eefs_start[j*3+1],eefs_start[j*3+2],base_link_frame);
        eefs_.push_back(EndEffector(links_[i],workv3_,joint_names_,(Robot*)this));
        break;
      }
    }
  }
  NUM_EEFS = eefs_.size();

  OUT_LOG(logINFO)<< "NUM_EEFS: " << NUM_EEFS ;
  OUT_LOG(logINFO)<< "N_FIXED_JOINTS: " << NUM_FIXED_JOINTS ;
  OUT_LOG(logINFO)<< "NUM_JOINTS: " << NUM_JOINTS ;
  OUT_LOG(logINFO)<< "NDOFS: " << NDOFS ;
  OUT_LOG(logINFO)<< "NSPATIAL: " << NSPATIAL ;
  OUT_LOG(logINFO)<< "NEULER: " << NEULER ;
}
