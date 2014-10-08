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
  double dt = t - last_time;
  this->generalized_q = generalized_q_in;
  this->generalized_qd = generalized_qd_in;
  this->generalized_qdd = generalized_qdd_in;
  this->generalized_fext = generalized_fext_in;
  OUTLOG(generalized_q,"generalized_q",logDEBUG);
  OUTLOG(generalized_qd,"generalized_qd",logDEBUG);

  // Set Robot Data in robot
  update();

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

  set_model_state(q,qd);

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

  {
    center_of_feet_x.set_zero();
    center_of_feet_x.pose = environment_frame;

    int ii = 0;
    for(int i=0;i<NUM_EEFS;i++){
      if(is_foot[i] == 0) continue;
      center_of_feet_x += Ravelin::Pose3d::transform_point(environment_frame,Ravelin::Vector3d(0,0,0,eefs_[i].link->get_pose()));
      ii++;
    }
    center_of_feet_x /= (double)ii;

#ifdef VISUALIZE_MOBY
  visualize_ray(  center_of_feet_x,
                  center_of_feet_x,
                  Ravelin::Vector3d(1,0.5,0),
                  0.5,
                  sim
                );
#endif
  }

  // NOTE: Some minor balancing code
//  for(unsigned i=0;i< NUM_EEFS;i++){
//    if(!is_foot[i]){
//      eefs_[i].origin[0] += (center_of_feet_x[0]-center_of_mass_x[0])*0.01;
//      eefs_[i].origin[1] += (center_of_feet_x[1]-center_of_mass_x[1])*0.01;
//    } else {
//      eefs_[i].origin[0] -= (center_of_feet_x[0]-center_of_mass_x[0])*0.01;
//      eefs_[i].origin[1] -= (center_of_feet_x[1]-center_of_mass_x[1])*0.01;
//    }
//  }

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
          next_waypoint(patrol_points[patrol_index*2],patrol_points[patrol_index*2+1],center_of_mass_x[2],environment_frame);
      next_waypoint[2] = center_of_mass_x[2];

      double distance_to_wp = (next_waypoint - center_of_mass_x).norm();

      if( distance_to_wp < 0.025){
        OUT_LOG(logERROR) << "waypoint reached, incrementing waypoint.";
        OUTLOG(next_waypoint,"this_wp",logERROR);
        OUTLOG(next_waypoint,"center_of_mass_x",logERROR);

        patrol_index = (patrol_index+1) % num_waypoints;

        next_waypoint = Ravelin::Vector3d(patrol_points[patrol_index*2],patrol_points[patrol_index*2+1],center_of_mass_x[2],environment_frame);
      }
# ifdef VISUALIZE_MOBY
      OUT_LOG(logERROR) << "num_wps = " << num_waypoints;
      OUT_LOG(logERROR) << "distance_to_wp = " << distance_to_wp;
      OUT_LOG(logERROR) << "patrol_index = " << patrol_index;
    visualize_ray(  next_waypoint,
                    center_of_mass_x,
                    Ravelin::Vector3d(1,0.5,0),
                    sim
                  );
    OUTLOG(next_waypoint,"next_wp",logERROR);

    for(int i=0;i<num_waypoints;i++){
      Ravelin::Vector3d wp(patrol_points[i*2],patrol_points[i*2+1],next_waypoint[2],environment_frame);
      OUTLOG(wp,"wp",logERROR);
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
          - Ravelin::Vector3d(center_of_mass_x[0],center_of_mass_x[1],0,environment_frame);
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

    // Edit foot origins to Lean into turns
    std::vector<EndEffector*> feet;
    for(unsigned i=0,ii=0;i< NUM_EEFS;i++){
      if(is_foot[i] == 0) continue;
      feet.push_back(&eefs_[i]);
      feet[ii]->origin.pose = base_horizontal_frame;

      // Robot leans into movement
//      foot_origin[i][0] += go_to[0]*-0.1;
//      foot_origin[i][1] += go_to[1]*-0.1;
//      // lean forward on front feet if moving forward
//      if(foot_origin[i][0] > 0 && go_to[0] > 0)
//        foot_origin[i][2] += go_to[0]*0.1;
//      foot_origin[i][1] += go_to[5]*-0.01;
//      foot_origin[i].pose = base_frame;

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

    if(roll_pitch_yaw[1] > 0.01){
      goto_6d[0] += 0.05;
//      displace_base_link[4] += roll_pitch_yaw[1]*0.1;
    } else if(roll_pitch_yaw[1] < 0.01){
      goto_6d[0] -= 0.05;
//      displace_base_link[4] += roll_pitch_yaw[1]*0.1;
    }

    int STANCE_ON_CONTACT = CVarUtils::GetCVarRef<int>("quadruped.locomotion.stance-on-contact");
    walk_toward(goto_6d,this_gait,footholds,duty_factor,gait_time,step_height,STANCE_ON_CONTACT,feet,generalized_qd.segment(NUM_JOINTS,NDOFS),center_of_mass_x,t,q,qd,qdd,foot_pos,foot_vel, foot_acc);
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
  trajectory_ik(x_des,xd_des, xdd_des,q,q_des,qd_des,qdd_des);

  static Ravelin::MatrixNd MU;
  MU.set_zero(NC,2);

#ifdef EXPERIMENTAL_CODE
  // --------------------------- FRICTION ESTIMATION ---------------------------
  {
    Ravelin::VectorNd cf;
    double err = friction_estimation(vel,fext,dt,N,D,M,MU,cf);
    OUT_LOG(logINFO)<< "err (friction estimation): " << err << std::endl;
    OUTLOG(MU,"MU",logDEBUG);
    OUTLOG(cf,"contact_forces",logDEBUG);
  }
#else
  for(int i=0,ii=0;i<NUM_EEFS;i++){
    if(eefs_[i].active){
      for(int j=0;j<eefs_[i].point.size();j++,ii++){
        for(int k=0;k<2;k++)
          MU(ii,k) = eefs_[i].mu_coulomb[j];
      }
    }
  }
#endif

  // ----------------------------- STABILIZATION -------------------------------
  static int &USE_STABILIZATION = CVarUtils::GetCVarRef<int>("quadruped.stabilization.active");
  if(USE_STABILIZATION){
    static int &USE_VIIP = CVarUtils::GetCVarRef<int>("quadruped.stabilization.viip.active");
    if(USE_VIIP){
      static std::vector<double> &x_des = CVarUtils::GetCVarRef<std::vector<double> >("quadruped.stabilization.viip.desired.x");
      static std::vector<double> &xd_des = CVarUtils::GetCVarRef<std::vector<double> >("quadruped.stabilization.viip.desired.xd");

      static std::vector<double>
          &Kp = CVarUtils::GetCVarRef<std::vector<double> >("quadruped.stabilization.viip.gains.kp"),
          &Kv = CVarUtils::GetCVarRef<std::vector<double> >("quadruped.stabilization.viip.gains.kv"),
          &Ki = CVarUtils::GetCVarRef<std::vector<double> >("quadruped.stabilization.viip.gains.ki");

      Ravelin::VectorNd fb = Ravelin::VectorNd::zero(NUM_JOINT_DOFS);
      contact_jacobian_stabilizer(R,Kp,Kv,Ki,x_des,xd_des,fb);

      static int &FEEDBACK_ACCEL = CVarUtils::GetCVarRef<int>("quadruped.stabilization.viip.accel");
      if(FEEDBACK_ACCEL)
        qdd_des += fb;
      else
        ufb += fb;
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
      PID::control(q_des, qd_des,q,qd,joint_names_, gains,fb);

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
      eef_stiffness_fb(Kp,Kv,Ki,x_des,xd_des,q,qd,fb);

      if(FEEDBACK_ACCEL)
        qdd_des += fb;
      else
        ufb += fb;
    }
  }

  // ------------------------ INVERSE DYNAMICS ---------------------------------

  static int &CONTROL_IDYN = CVarUtils::GetCVarRef<int>("quadruped.inverse-dynamics.active");
  if(CONTROL_IDYN){
    static double &dt = CVarUtils::GetCVarRef<double>("quadruped.inverse-dynamics.dt");
    static double &alpha = CVarUtils::GetCVarRef<double>("quadruped.inverse-dynamics.alpha");
    static int &USE_DES_CONTACT = CVarUtils::GetCVarRef<int>("quadruped.inverse-dynamics.des-contact");
    static int &USE_LAST_CFS = CVarUtils::GetCVarRef<int>("quadruped.inverse-dynamics.last-cfs");

    Ravelin::VectorNd cf;
    Ravelin::VectorNd id = Ravelin::VectorNd::zero(NUM_JOINT_DOFS);
    // Recalculate contact jacobians based on desired lift-off feet
    if(USE_DES_CONTACT){
      NC = 0;
      for(int i=0;i<NUM_EEFS;i++)
        if(eefs_[i].active)
          NC += eefs_[i].point.size();
      calc_contact_jacobians(N,D,R);
    }

    if(USE_LAST_CFS){
      cf.set_zero(NC*5);
      for(unsigned i=0,ii=0;i< eefs_.size();i++){
        if(!eefs_[i].active) continue;
        for(int j=0;j<eefs_[i].point.size();j++,ii++){
          Ravelin::Matrix3d R_foot( eefs_[i].normal[j][0], eefs_[i].normal[j][1], eefs_[i].normal[j][2],
                                      eefs_[i].tan1[j][0],   eefs_[i].tan1[j][1],   eefs_[i].tan1[j][2],
                                      eefs_[i].tan2[j][0],   eefs_[i].tan2[j][1],   eefs_[i].tan2[j][2]);
          Ravelin::Origin3d contact_impulse = Ravelin::Origin3d(R_foot.mult(eefs_[i].impulse[j],workv3_)*((t-last_time)/dt));
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
      OUTLOG(cf,"cf z",logDEBUG);
    }

    // ------------------------ WORKSPACE INVERSE DYNAMICS ---------------------
    static int &WORKSPACE_IDYN = CVarUtils::GetCVarRef<int>("quadruped.inverse-dynamics.operational-space");
    static double &beta = CVarUtils::GetCVarRef<double>("quadruped.inverse-dynamics.beta");
    if(WORKSPACE_IDYN){
      os_velocity.set_zero(Rw.rows());
      Ravelin::SVector6d go_to_global;
      go_to_global = Ravelin::Pose3d::transform(Moby::GLOBAL,Ravelin::SVelocityd(go_to.data(),base_horizontal_frame));
      workspace_trajectory_goal(go_to_global,x_des,xd_des,xdd_des,beta,dt,os_velocity);

      if(USE_LAST_CFS){
        if(workspace_inverse_dynamics(generalized_qd,os_velocity,M,R.mult(cf,workv_) += generalized_fext,dt,MU,id))
          uff += (id*=alpha);
      }else{
        if(workspace_inverse_dynamics(generalized_qd,os_velocity,M,generalized_fext,dt,MU,id,cf))
          uff += (id*=alpha);
      }
    } else {
      if(!inverse_dynamics(generalized_qd,qdd_des,M,N,D,generalized_fext,dt,MU,id,cf))
        cf.set_zero(NC*5);
      else
        uff += (id*=alpha);
    }
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
    if(u[i] > torque_limits_u[i])
      u[i] = torque_limits_u[i];
    else if(u[i] < torque_limits_l[i])
      u[i] = torque_limits_l[i];
  }

#ifndef NDEBUG
  OUT_LOG(logINFO)<< "time = "<< t ;
  OUT_LOG(logINFO)<< "num_contacts = " << NC ;

  set_model_state(q,qd);

  if(Log::ToString(Log::ReportingLevel()).compare("DEBUG") == 0){
    Ravelin::MatrixNd Jf;
    for(int i=0;i<NUM_EEFS;i++){
      boost::shared_ptr<Ravelin::Pose3d> event_frame(new Ravelin::Pose3d(x_des[i].pose));
      EndEffector& foot = eefs_[i];

      // Positional Correction
      Ravelin::Vector3d x_des_now = Ravelin::Pose3d::transform_point(x_des[i].pose,Ravelin::Vector3d(0,0,0,eefs_[i].link->get_pose()));
      Ravelin::Vector3d x_err  = x_des[i] - x_des_now;
      OUTLOG( x_des_now,foot.id + "_x",logDEBUG);
      OUTLOG( x_des[i],foot.id + "_x_des",logDEBUG);
      OUTLOG( x_err,foot.id + "_x_err",logDEBUG);

      // Remove portion of foot velocity that can't be affected by corrective forces
      event_frame->x = Ravelin::Pose3d::transform_point(x_des[i].pose,Ravelin::Vector3d(0,0,0,eefs_[i].link->get_pose()));
      dbrobot_->calc_jacobian(event_frame,eefs_[i].link,Jf);
      Ravelin::SharedConstMatrixNd Jb = Jf.block(0,3,NUM_JOINTS,NDOFS);
      Ravelin::SharedConstVectorNd vb = generalized_qd.segment(NUM_JOINTS,NDOFS);
      Jb.mult(vb,workv3_);
      workv3_.pose = x_des[i].pose;

      // Velocity Correction
      Ravelin::Vector3d xd_des_now = (Ravelin::Pose3d::transform_vector(x_des[i].pose,eefs_[i].link->get_velocity().get_linear()) - workv3_);
      Ravelin::Vector3d xd_err = xd_des[i] - xd_des_now;
      OUTLOG( xd_des_now,foot.id + "_xd",logDEBUG);
      OUTLOG( xd_des[i],foot.id + "_xd_des",logDEBUG);
      OUTLOG( xd_err,foot.id + "_xd_err",logDEBUG);
    }
  }

   OUT_LOG(logINFO) <<"JOINT:A\t: U\t| Q\t: des\t: err\t|Qd\t: des\t: err\t|Qdd\t: des\t: err"<<std::endl;
   for(unsigned i=0,ii=0;i< NUM_JOINTS;i++){
     if(joints_[i])
     for(int j=0;j<joints_[i]->num_dof();j++,ii++){
     OUT_LOG(logINFO)<< std::to_string(j)+joints_[i]->id << ":"<< active_joints_[std::to_string(j)+joints_[i]->id]
               << "\t " <<  std::setprecision(4) << u[ii]
               << "\t| " << joints_[i]->q[j]
               << "\t " << q_des[ii]
               << "\t " << q[ii] - q_des[ii]
               << "\t| " << joints_[i]->qd[j]
               << "\t " << qd_des[ii]
               << "\t " <<  qd[ii] - qd_des[ii]
               << "\t| " << qdd[ii]
               << "\t " << qdd_des[ii]
               << "\t " <<  (qdd[ii] - qdd_des[ii]);
     }
   }
   OUTLOG(roll_pitch_yaw,"roll_pitch_yaw",logINFO);
   OUTLOG(zero_moment_point,"ZmP",logINFO);
   OUTLOG(center_of_mass_x,"CoM_x",logINFO);
   OUTLOG(center_of_feet_x,"center_of_feet_x",logINFO);
   OUTLOG(center_of_mass_xd,"CoM_xd",logINFO);
   OUTLOG(center_of_mass_xdd,"CoM_xdd",logINFO);
   OUTLOG(q,"q",logDEBUG);
   OUTLOG(qd,"qd",logDEBUG);
   OUTLOG(qdd,"qdd",logDEBUG);
   OUTLOG(q_des,"q_des",logDEBUG);
   OUTLOG(qd_des,"qd_des",logDEBUG);
   OUTLOG(qdd_des,"qdd_des",logDEBUG);
   OUTLOG(generalized_fext,"fext",logDEBUG);

   OUTLOG(uff,"uff",logINFO);
   OUTLOG(ufb,"ufb",logINFO);
   // -----------------------------------------------------------------------------
#endif

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

  OUT_LOG(logERROR) << "Log Type : " << LOG_TYPE;
  OUT_LOG(logERROR) << "logERROR";
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

 const double* data = &base_start.front();
 displace_base_link = Ravelin::SVector6d(data);

 OUTLOG(joint_names,"joint_names",logERROR);
 OUTLOG(joints_start,"joints_start",logERROR);

 // MAKE SURE DATA PARSED PROPERLY

// assert(joint_names.size() == joints_.size());
 assert(joint_names.size() == joints_start.size());
 assert(joint_names.size() == torque_limits.size());

  std::map<std::string, double> torque_limits_;
  for(int i=0,ii=0;i<NUM_JOINTS;i++){
    if(joints_[i])
    for(int j=0;j<joints_[i]->num_dof();j++,ii++){
//      OUT_LOG(logDEBUG) << joint_names[ii] << active_joints[ii] << std::endl;
      active_joints_[joint_names[ii]] = (joint_names[ii].substr(joint_names[ii].size()-1,1).compare("4") == 0)? false : true;
//          (joint_names[ii].substr(4,2).compare("XY") == 0)? false : true;
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
  OUTLOG(torque_limits_l,"torque_limits_l",logERROR);
  OUTLOG(torque_limits_u,"torque_limits_u",logERROR);

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
