#include<quadruped.h>
#include<utilities.h>

void Quadruped::sinusoidal_trot(Ravelin::VectorNd& q_des,Ravelin::VectorNd& qd_des,Ravelin::VectorNd& qdd,double dt){
  static double t = 0;
  if(dt==0)
    t = 0;
  static Ravelin::VectorNd workv_,last_qd_des = Ravelin::VectorNd::zero(NUM_JOINTS);
  static Ravelin::MatrixNd workM_;
  t += dt;

  std::vector<Ravelin::Vector3d> foot_vel(NUM_EEFS),foot_poses(NUM_EEFS);
  Ravelin::VectorNd Hs(NUM_EEFS);

  // Additional Parameters for CPG
  double Ls = 0.02, Vf = 10;
  for(int f=0;f<NUM_EEFS;f++){
    // set height of gait (each foot)
    Hs[f] = 0.02;

    // SET EEF ORIGINS TO the ground below that EEF SHOULDER
    if(dt != 0){
      Ravelin::Pose3d joint_pose = *joints_[eefs_[f].chain[2]]->get_pose();
      joint_pose.update_relative_pose(Moby::GLOBAL);
      joint_pose.x[2] += -0.13;
      joint_pose.update_relative_pose(base_frame);
      eefs_[f].origin.pose = base_frame;
      eefs_[f].origin = joint_pose.x;
      if(f%2 == 0)
        eefs_[f].origin[1] += 0.02;
      else
        eefs_[f].origin[1] -= 0.02;
    }
  }


  // Foot goal position
  for(int f=0;f<NUM_EEFS;f++){
    if(f==0 || f==3) {
      foot_poses[f] = eefs_[f].origin+Ravelin::Vector3d(Ls*sin(M_PI_2+t*Vf),0,Hs[f]*cos(M_PI_2+t*Vf),base_frame);
      foot_vel[f] = Ravelin::Vector3d(Vf*Ls*cos(M_PI_2+t*Vf),0,-Vf*Hs[f]*sin(M_PI_2+t*Vf));
    } else {
      foot_poses[f] = eefs_[f].origin+Ravelin::Vector3d(-Ls*sin(M_PI_2+t*Vf),0,Hs[f]*cos(M_PI_2+t*-Vf),base_frame);
      foot_vel[f] = Ravelin::Vector3d(-Vf*Ls*cos(M_PI_2+t*Vf),0,Vf*Hs[f]*sin(M_PI_2+t*-Vf));
    }
  }

  // EEF POSITION
  for(int i=0;i<NUM_EEFS;i++){
    OUT_LOG(logDEBUG) << "\t" << eefs_[i].id << "_x =" << foot_poses[i];
    RMRC(eefs_[i],q_des,foot_poses[i],q_des);
    OUT_LOG(logDEBUG) << "\t" << eefs_[i].id << "_q =" << foot_poses[i];
  }
  // Foot goal Velocity
  if(dt!= 0)
  for(int f=0;f<NUM_EEFS;f++){
    // Calc jacobian for AB at this EEF
    boost::shared_ptr<Ravelin::Pose3d> foot_frame(new Ravelin::Pose3d(*eefs_[f].link->get_pose()));
    foot_frame->update_relative_pose(Moby::GLOBAL);
    boost::shared_ptr<Ravelin::Pose3d> event_frame(new Ravelin::Pose3d(Moby::GLOBAL));
    event_frame->x = foot_frame->x;
    event_frame->update_relative_pose(base_frame);

    dbrobot_->calc_jacobian(event_frame,eefs_[f].link,workM_);// J: qd -> xd
    Ravelin::MatrixNd iJ(3,3);
    for(int j=0;j<3;j++)                                      // x,y,z
      for(int i=0;i<eefs_[f].chain.size();i++)                // actuated joints
        iJ(j,i) = workM_(j,eefs_[f].chain[i]);                // Ji <- J
    LA_.solve_fast(iJ,foot_vel[f]);                           // qd <- Ji\xd
  }

  for(int i=0;i<NUM_EEFS;i++){
    for(int j=0;j<eefs_[i].chain.size();j++){
      qd_des[eefs_[i].chain[j]] = foot_vel[i][j];
    }
  }
  for(int i=0;i<NUM_JOINTS;i++)
    qdd[i] = (qd_des[i] - last_qd_des[i])/dt;

  last_qd_des = qd_des;
}

/// Generate a bunch of random points at z = 0 around the COM
void Quadruped::find_footholds(std::vector<Ravelin::Vector3d>& footholds,int num_footholds){
  footholds.clear();
  double rad = 0.2;
  if(NC>0) center_of_contact.active = true;
  if(!center_of_contact.active) return;
  for(int i=0;i<num_footholds;i++){
    Ravelin::Vector3d fh(
      center_of_mass_x[0] + (double)rand()/RAND_MAX * 2.0 * rad - rad,center_of_mass_x[0] + (double)rand()/RAND_MAX * 2.0 * rad - rad,0
      ,Moby::GLOBAL
    );
    if(center_of_contact.normal.norm() > Moby::NEAR_ZERO)
      fh[2] = Utility::get_z_plane(fh[0],fh[1],center_of_contact.normal,center_of_contact.point);

    footholds.push_back(Ravelin::Pose3d::transform_point(base_frame,fh));
  }
}

void Quadruped::select_foothold(const std::vector<Ravelin::Vector3d>& footholds,const Ravelin::Origin3d &x, Ravelin::Origin3d& x_fh){
  double min_dist = INFINITY;
  int    min_vec = 0;
  for(int i=0;i<footholds.size();i++){
    double norm = (footholds[i]-x).norm();
    if(norm < min_dist){
      min_dist = norm;
      min_vec = i;
    }
  }
  x_fh = Ravelin::Origin3d(footholds[min_vec]);
}

void Quadruped::workspace_trajectory_goal(const Ravelin::SVector6d& v_base, const std::vector<Ravelin::Vector3d>& foot_pos,const std::vector<Ravelin::Vector3d>& foot_vel,const std::vector<Ravelin::Vector3d>& foot_acc,
                                          double beta, double dt, Ravelin::VectorNd& v_bar){
//  v_bar.set_zero();
//  v_bar.set_sub_vec(NUM_EEFS*3,vel.get_sub_vec(NUM_JOINTS,NUM_JOINTS+6,workv_));
  v_bar.set_sub_vec(NUM_EEFS*3,v_base);
#ifdef VISUALIZE_MOBY
//  visualize_ray(center_of_mass_x + center_of_mass_xd,
    visualize_ray(center_of_mass_x + Ravelin::Pose3d::transform_vector(environment_frame,links_[0]->get_velocity().get_linear()),
                center_of_mass_x,
                Ravelin::Vector3d(1,0,0),
                sim);
    // tippyness
    Ravelin::Vector3d ang = links_[0]->get_velocity().get_angular();
    ang[2] = ang[1];
    ang[1] = ang[0];
    ang[0] = ang[2];
    ang[2] = 0;
    visualize_ray(center_of_mass_x + Ravelin::Pose3d::transform_vector(environment_frame,ang),
                center_of_mass_x,
                Ravelin::Vector3d(0,0,1),
                sim);

  visualize_ray(center_of_mass_x + Ravelin::Pose3d::transform_vector(environment_frame, v_base.get_upper()),
                center_of_mass_x,
                Ravelin::Vector3d(0,1,0),
                sim);
#endif
//  Ravelin::Vector3d base_correct = beta/dt * ( goal_base_pose - center_of_mass_x);
//  v_bar[NUM_EEFS*3]   += base_correct[0];
//  v_bar[NUM_EEFS*3+1] += base_correct[1];
//  v_bar[NUM_EEFS*3+2] += base_correct[2];
//  v_bar[NUM_EEFS*3+3] += -beta/dt * roll_pitch_yaw[0];
//  v_bar[NUM_EEFS*3+4] += -beta/dt * roll_pitch_yaw[1];
  // base position should be 0.13m above centroid of feet
  for(int i=0;i<NUM_EEFS;i++){
//    boost::shared_ptr<Ravelin::Pose3d> base_orient_at_foot = boost::shared_ptr<Ravelin::Pose3d>(new Ravelin::Pose3d(Ravelin::Quatd::identity(),Ravelin::Origin3d(foot_pos[i]),base_frame));
//    Ravelin::SVelocityd base_velocity_at_foot = Ravelin::Pose3d::transform(base_orient_at_foot,links_[0]->get_velocity());
//    base_velocity_at_foot.pose = base_frame;
    Ravelin::Vector3d pos = Ravelin::Pose3d::transform_point(base_frame,Ravelin::Vector3d(0,0,0,eefs_[i].link->get_pose())),
                      pos_correct = beta * (foot_pos[i] - pos)/dt,
                      foot_vel_wrt_base = foot_vel[i] + foot_acc[i]*dt;
    v_bar.set_sub_vec(i*3,foot_vel_wrt_base + pos_correct);
#ifdef VISUALIZE_MOBY
    if(false){
    OUTLOG(foot_vel_wrt_base,eefs_[i].id + "_des_vel",logERROR);
    OUTLOG(foot_vel[i],eefs_[i].id + "_vel",logERROR);

    OUTLOG(foot_pos[i],eefs_[i].id + "_des_pos",logERROR);
    OUTLOG(pos,eefs_[i].id + "_pos",logERROR);
    OUTLOG(pos_correct,eefs_[i].id + "_pos_err",logERROR);

    visualize_ray(
          Ravelin::Pose3d::transform_point(Moby::GLOBAL,pos)
            +Ravelin::Pose3d::transform_vector(Moby::GLOBAL,foot_vel_wrt_base),
          Ravelin::Pose3d::transform_point(Moby::GLOBAL,pos),
          Ravelin::Vector3d(0,1,0), sim);
    visualize_ray(
          Ravelin::Pose3d::transform_point(Moby::GLOBAL,pos)
            +Ravelin::Pose3d::transform_vector(Moby::GLOBAL,pos_correct),
          Ravelin::Pose3d::transform_point(Moby::GLOBAL,pos),
          Ravelin::Vector3d(0,0,1), sim);
    visualize_ray(
          Ravelin::Pose3d::transform_point(Moby::GLOBAL,pos)
           + Ravelin::Pose3d::transform_vector(Moby::GLOBAL,foot_vel_wrt_base + pos_correct),
          Ravelin::Pose3d::transform_point(Moby::GLOBAL,pos),
          Ravelin::Vector3d(1,0,1), sim);
    }
#endif
  }
}


void Quadruped::trajectory_ik(const std::vector<Ravelin::Vector3d>& foot_pos,const std::vector<Ravelin::Vector3d>& foot_vel,const std::vector<Ravelin::Vector3d>& foot_acc,
                              Ravelin::VectorNd& q_des,Ravelin::VectorNd& qd_des,Ravelin::VectorNd& qdd_des){
  ////////////////////// IK CONTROL ////////////////////////
  Ravelin::VectorNd q = q_des,
                    qd = qd_des,
                    qdd = qdd_des;

  for(int i=0;i<NUM_EEFS;i++){
    EndEffector& foot = eefs_[i];


    // POSITION
    OUTLOG( foot_pos[i],foot.id + "_x",logDEBUG1);
    RMRC(foot,q,foot_pos[i],q_des);
    OUTLOG(q_des.select(foot.chain_bool,workv_),foot.id + "_q",logDEBUG1);

    // Calc jacobian for AB at this EEF
    Ravelin::MatrixNd J;
    Ravelin::Origin3d x;
    for(int k=0;k<foot.chain.size();k++)                // actuated joints
      x[k] = q[foot.chain[k]];
    foot_jacobian(x,foot,base_frame,J);

    Ravelin::Vector3d qd_foot,qdd_foot;
    // VELOCITY & ACCELERATION
    OUTLOG(foot_vel[i],foot.id + "_xd", logDEBUG1);
    LA_.solve_fast((workM_ = J),(qd_foot = foot_vel[i]));
    OUTLOG(qd_foot,foot.id + "_qd", logDEBUG1);

    OUTLOG(foot_acc[i],foot.id + "_xdd", logDEBUG1);
    LA_.solve_fast((workM_ = J),(qdd_foot = foot_acc[i]));
    OUTLOG(qdd_foot,foot.id + "_qdd", logDEBUG1);

    for(int j=0;j<foot.chain.size();j++){
      qd_des[foot.chain[j]] = qd_foot[j];
      qdd_des[foot.chain[j]] = qdd_foot[j];
    }
  }
}

bool gait_phase(double touchdown,double duty_factor,double gait_progress){
  double liftoff = touchdown + duty_factor,left_in_phase = 0;
  liftoff = liftoff - (double) ((int) liftoff);

  OUT_LOG(logDEBUG) << "gait_progress " << gait_progress;
  OUT_LOG(logDEBUG) << "touchdown " << touchdown;
  OUT_LOG(logDEBUG) << "duty_factor " << duty_factor;
  OUT_LOG(logDEBUG) << "liftoff " << liftoff;

  // ----- STANCE PHASE ------
  if(// during STANCE (no wrap around)
     ( gait_progress >= touchdown  &&  gait_progress  < liftoff)
     // during STANCE (with wrap around) -- same as !SWING
    || (!(gait_progress  < touchdown  &&  gait_progress >= liftoff) && liftoff < touchdown)
     ){
    return true;
  }
  // ----- SWING PHASE -----
  return false;

}

double gait_phase(double touchdown,double duty_factor,double gait_progress,double stance_phase){
  double liftoff = touchdown + duty_factor,left_in_phase = 0;
  liftoff = liftoff - (double) ((int) liftoff);

  OUT_LOG(logDEBUG) << "gait_progress " << gait_progress;
  OUT_LOG(logDEBUG) << "touchdown " << touchdown;
  OUT_LOG(logDEBUG) << "duty_factor " << duty_factor;
  OUT_LOG(logDEBUG) << "liftoff " << liftoff;

  // ----- STANCE PHASE ------
  if(stance_phase){
    // left in stance phase = negaive time left in phase
    left_in_phase = (liftoff - gait_progress);
    if(left_in_phase < 0)
      left_in_phase = (1.0 - gait_progress) + liftoff;
//    left_in_phase = -duty_factor;
  }
  // ----- SWING PHASE -----
  else {
    left_in_phase = (touchdown - gait_progress);
    if(left_in_phase < 0)
      left_in_phase = (1.0 - gait_progress) + touchdown;
//    left_in_phase = 1-duty_factor;
  }
  return left_in_phase;
}

/**
 * @brief Quadruped::walk_toward : OSRF Locomotion System Implementation
 * @param command : 6x1 vector of goal base velocity differential
 * @param touchdown : specify the moment in the gait where a foot touches down stance phase. touchdown_i \in [0..1)
 * @param footholds : vector of 3x1 points indicating locations for valid foot placement
 * @param duty_factor : portion of gait a foot is in stance phase after touchdown duty_factor_{i} \in [0..1)
 * @param gait_duration : duration of time in seconds of one gait cycle [0..inf)
 * @param step_height : maximum height of foot during a step
 * @param foot_origins : vector of NUM_EEFS 3x1 points indicating intermediate foot placement
 * @param t : current time
 * @param q : current generalized coordinates
 * @param qd : current generalized velocity
 * @param qdd : current generalized acceleration
 * @param foot_pos : NUM_FEET length vector of 3x1 cartesian coordinates for feet (populated with current values)
 * @param foot_vel : NUM_FEET length vector of 3x1 cartesian velocities for feet (populated with current values)
 * @param foot_acc : NUM_FEET length vector of 3x1 cartesian acceleration for feet (populated with current values)
 */
void Quadruped::walk_toward(
    // PARAMETERS
    const Ravelin::SVector6d& command,
    const std::vector<double>& touchdown,
    const std::vector<Ravelin::Vector3d>& footholds,
    const std::vector<double> duty_factor,
    double gait_duration,
    double step_height,
    // MODEL
    const std::vector<Ravelin::Vector3d>& foot_origin,
    double t,
    const Ravelin::VectorNd& q,
    const Ravelin::VectorNd& qd,
    const Ravelin::VectorNd& qdd,
    // OUTPUT
    std::vector<Ravelin::Vector3d>& foot_pos,
    std::vector<Ravelin::Vector3d>& foot_vel,
    std::vector<Ravelin::Vector3d>& foot_acc)
{
  OUT_LOG(logDEBUG) << " -- Quadruped::walk_toward() entered";
  const boost::shared_ptr<const Ravelin::Pose3d>& base_frame = command.pose;
  static bool inited = false;
  static int spline_plan_length = 2,
             NUM_EEFS = foot_pos.size();
  static double last_time = t;

  if(t-last_time > gait_duration* (*std::min_element(duty_factor.begin(),duty_factor.end())) )
    inited = false;

  // persistent Vector storing spline coefs
  // [foot][dimension][interval]
  static std::vector< std::vector< std::vector<Ravelin::VectorNd> > > spline_coef(NUM_EEFS);
  // persistent Vector storing time delimitations to each spline
  // [foot][interval]
  static std::vector< std::vector<Ravelin::VectorNd> > spline_t(NUM_EEFS);
  static std::vector< bool > stance_phase(NUM_EEFS);

  double gait_progress = t/gait_duration;
  gait_progress = gait_progress - (double) ((int) gait_progress);
  if(gait_progress >= 1) gait_progress = Moby::NEAR_ZERO;
  OUT_LOG(logDEBUG) << "\tprog : " << gait_progress;
  OUT_LOG(logDEBUG) << "\ttime : " << t;

  // (Re)Populate spline vectors
  if(!inited){
    for(int i=0;i<NUM_EEFS;i++){
      stance_phase[i] = gait_phase(touchdown[i],duty_factor[i],gait_progress);
      spline_coef[i].resize(3);
      for(int d=0; d<3;d++){
        spline_coef[i][d].resize(spline_plan_length);
        for(int j = 0; j<  spline_plan_length;j++)
          spline_coef[i][d][j] = Ravelin::VectorNd();
      }

      spline_t[i].resize(spline_plan_length);
      for(int j = 0; j<  spline_plan_length;j++)
        spline_t[i][j] = Ravelin::VectorNd::zero(2);
    }
  }

  ////////////////// PHASE PLANNING ///////////////////////
  for(int i=0;i<NUM_EEFS;i++){

    Ravelin::Vector3d &x   = foot_pos[i],
                      &xd  = foot_vel[i],
                      &xdd = foot_acc[i];

    // Check if Spline must be reevaluated
    double left_in_phase;
    bool replan_path = false;
    if(inited){
      // Set liftoff feet
      if(eefs_[i].active && stance_phase[i])
        eefs_[i].active = false;
      for(int d=0; d<3;d++){
        OUT_LOG(logDEBUG) << "Evaluate existing spline at " << t;
        replan_path = !Utility::eval_cubic_spline(spline_coef[i][d],spline_t[i],t,x[d],xd[d],xdd[d]);
        if(replan_path)
          break;
      }
    }

    // Plan a new spline for this foot
    if(replan_path || !inited ){
      // What phase of the gait is the controller in?
      // NOTE: Don't ever use modf (this does modf(t/gait_duration,&intpart))
      // truncates t/gait_duration at decimal; gait_progress \in [0,1)

      OUT_LOG(logDEBUG) << "foot: " << i;
      OUT_LOG(logDEBUG) << "\t PHASE PROGRESS: { " << touchdown[i] << " .. " << gait_progress << " .. " << (touchdown[i] + duty_factor[i]) <<" }";
      // How much time is left in the current phase stance (-), swing (+)
      left_in_phase = gait_phase(touchdown[i],duty_factor[i],gait_progress,stance_phase[i]);

      OUT_LOG(logDEBUG) << "\tleft in phase (sec) : " << left_in_phase
                        << " (" << left_in_phase*gait_duration << ") ";

      OUT_LOG(logDEBUG) << "\tPlanning next Spline";
      // creat new spline at top of history
      // take time at end of spline_t history
      double t0 = 0;

      if(!inited){ // first iteration
        // Get Current Foot pos, Velocities & Accelerations
//        foot_jacobian(x,eefs_[i],environment_frame,workM_);
//        workM_.transpose_mult(qd.select(eefs_[i].chain_bool,workv_),xd);
//        workM_.transpose_mult(qdd.select(eefs_[i].chain_bool,workv_),xdd);
        xd.set_zero();
        xdd.set_zero();
      } else {
        // continue off of the end of the last spline
        t0 = *(spline_t[i].rbegin()->end()-1) - Moby::NEAR_ZERO;

        for(int d=0; d<3;d++){
          bool pass = Utility::eval_cubic_spline(spline_coef[i][d],spline_t[i],t0,x[d],xd[d],xdd[d]);
          assert(pass);
        }
      }

      Ravelin::Vector3d com_robot = Ravelin::Pose3d::transform_point(base_frame,center_of_mass_x);
      com_robot[2] = 0;//foot_origin[i][2];
      Ravelin::Origin3d x0 = Ravelin::Origin3d(foot_origin[i] - com_robot);

      // Calculate foot-step info
      // Determine linear portion of step
      Ravelin::Origin3d from_com(x0[0],x0[1],0),
                        yaw_step,
                        foot_goal = Ravelin::Origin3d(command[0],command[1],command[2]);

      // Determine shape of step needed to yaw robot
      Ravelin::Origin3d rotation_axis = Ravelin::Origin3d(0,0,1);
//      if(command[0] != 0 && command[1] != 0 && command[2] != 0){
//        rotation_axis = Ravelin::Origin3d::cross(Ravelin::Origin3d(command[0],0,command[2]),Ravelin::Origin3d(0,command[1],command[2]));
//        rotation_axis /= rotation_axis.norm();
//      }
      Ravelin::AAngled aa_gamma(Ravelin::Vector3d(rotation_axis,base_frame),command[5] *  left_in_phase*gait_duration);
      OUTLOG(aa_gamma,"aa_gamma",logDEBUG);
      Ravelin::Matrix3d R_gamma;
      R_gamma = aa_gamma;
      OUTLOG(R_gamma,"R_gamma",logDEBUG);
      // yaw_step = R(gamma)*r_com - r_com
      R_gamma.mult(from_com, yaw_step) -= from_com;
      OUTLOG( yaw_step," yaw_step",logDEBUG);

      // scale step length by interval duration
      foot_goal *=  left_in_phase*gait_duration;

      // Combine x, y, and yaw to for full step
      foot_goal +=  yaw_step;

      // All steps wrt to x0
      foot_goal /= 2;

      OUT_LOG(logDEBUG) << "\tstep = " << foot_goal;

      // Set control point for either stance or swing phase
      std::vector<Ravelin::Origin3d> control_points;


      if(!stance_phase[i]){
        // SWING
        OUT_LOG(logDEBUG) << "\tPlanning Swing phase (phase > 0)...";

        foot_goal /= (1-duty_factor[i]);
        OUT_LOG(logDEBUG) << "\tstep = " << foot_goal;

        // velocity correction
        // FEEDBACK CAPTURE POINT
//        Ravelin::Vector3d hip_pos = Ravelin::Pose3d::transform_point(environment_frame,Ravelin::Vector3d(0,0,0,links_[3+i]->get_pose()));
//        OUT_LOG(logERROR) << "Getting " << links_[3+i]->id << " pose";
//        double eta = 1.2,
//               height = hip_pos[2];
//        Ravelin::Origin3d rfb = eta*(Ravelin::Origin3d(command.get_upper())
//                                     -  Ravelin::Origin3d(
//                                          Ravelin::Pose3d::transform_vector(base_horizontal_frame, vel.get_sub_vec(NUM_JOINTS,NUM_JOINTS+3,workv3_))
//                                        )
//                                     ) * sqrt(height/grav);
//        rfb[2] = 0;
//        foot_goal -= Ravelin::Origin3d(rfb);

          control_points.push_back(Ravelin::Origin3d(x));
          control_points.push_back(Ravelin::Origin3d(x) + Ravelin::Origin3d(0,0,step_height));
        if(footholds.empty()){
          control_points.push_back(x0 + foot_goal + Ravelin::Origin3d(0,0,step_height));
          control_points.push_back(x0 + foot_goal);
        } else {
          Ravelin::Origin3d x_fh;
          select_foothold(footholds,Ravelin::Pose3d::transform_point(environment_frame, Ravelin::Vector3d((x0+foot_goal).data(),base_frame)).data(),x_fh);
          // Reach new foothold, account for movement of robot during step
          control_points.push_back(Ravelin::Origin3d(Ravelin::Pose3d::transform_point(base_frame,Ravelin::Vector3d(x_fh.data(),environment_frame)).data()) - foot_goal + Ravelin::Origin3d(0,0,step_height));
          control_points.push_back(x_fh - foot_goal);
        }
      }
      else if(stance_phase[i]){
        OUT_LOG(logDEBUG) << "\tPlanning Stance phase (phase < 0)...";

        // STANCE
        foot_goal /= -duty_factor[i];
        OUTLOG( foot_goal," foot_goal_old",logDEBUG);

        // project foot goal 'a' onto walking plane (defined by normal 'b')
        // x = v{||} - w{_|_}
        // w = b * (a.a / b.b)
//        if(center_of_contact.active)
//          foot_goal = foot_goal - center_of_contact.normal * (Ravelin::Vector3d(foot_goal.data(),Moby::GLOBAL).dot(center_of_contact.normal) / center_of_contact.normal.dot(center_of_contact.normal));
//        OUTLOG( foot_goal," foot_goal_new",logDEBUG);

        // all stance phase cases
//        double num_points = floor(foot_goal.norm()*100.0);
//        if(num_points < 1.0) num_points = 1.0;
        control_points.resize(3);
        control_points[0] = Ravelin::Origin3d(x);
        control_points[2] = Ravelin::Origin3d(x) + foot_goal;
        control_points[1] = (control_points[2] + control_points[0])/2.0;
        control_points[1].normalize();
        control_points[1] *= (control_points[2].norm() + control_points[0].norm())/2.0;
        if( (control_points[2] - control_points[1]).norm() < Moby::NEAR_ZERO)
          control_points.pop_back();
        for(int i=0;i<3;i++)
          OUTLOG(control_points[i],"cp",logERROR);

//        for(double pt=1.0;pt<=num_points;pt+=1.0){
//          Ravelin::Origin3d segment = Ravelin::Origin3d(x) + foot_goal*pt/num_points;
//          segment.normalize();
//          control_points.push_back(segment * x0.norm()) ;
//        }
      } else {
        OUT_LOG(logDEBUG) << "\tthrowing assertion (phase == 0) ...";
        assert(false);
      }

      // Plane walking Terrain abstraction (3 lowest feet form the plane)
      //  plane X + Y + c = z

//      if(NC > 0){
//        Ravelin::Vector3d hip_pos = Ravelin::Pose3d::transform_point(environment_frame,Ravelin::Vector3d(0,0,0,links_[3+i]->get_pose()));
//        center_of_contact.point[2] = 0.13;
//        double ground_z = Utility::get_z_plane(hip_pos[0],hip_pos[1],center_of_contact.normal,center_of_contact.point);
//        Ravelin::Vector3d ground_offset(0,0,ground_z - center_of_contact.point[2],environment_frame);
//        OUTLOG(ground_offset,"ground_offset",logERROR);
//        (*control_points.rbegin())[2] -= Ravelin::Pose3d::transform_vector(base_horizontal_frame,ground_offset)[2];
//      }

      // create new spline!!
      // copy last spline to history erasing oldest spline
      for(int j=0;j<spline_plan_length-1;j++){
        for(int d=0; d<3;d++)
          spline_coef[i][d][j] = spline_coef[i][d][j+1];
        spline_t[i][j] = spline_t[i][j+1];
      }

      Ravelin::Vector3d foot_velocity = Ravelin::Vector3d(foot_goal +  yaw_step,base_frame);
      // create spline using set of control points, place at back of history
      int n = control_points.size();

      spline_t[i].rbegin()->set_zero(n);
      Ravelin::VectorNd           &T = *(spline_t[i].rbegin());
      for(int j=0;j<n;j++)
        T[j] = t0 + fabs( left_in_phase*gait_duration / (double)(n-1)) * (double)j ;

      for(int d=0;d<3;d++){
        Ravelin::VectorNd           X(n);
        Ravelin::VectorNd          &coefs = *(spline_coef[i][d].rbegin());

        for(int j=0;j<n;j++)
          X[j] = control_points[j][d];
        OUTLOG(T,"T",logDEBUG1);
        OUTLOG(X,"X",logDEBUG1);

        Utility::calc_cubic_spline_coefs(T,X,Ravelin::Vector2d(xd[d],foot_velocity[d]),coefs);

        // then re-evaluate spline
        // NOTE: this will only work if we replanned for a t_0  <  t  <  t_0 + t_I
        OUT_LOG(logDEBUG) << "Eval first step in spline";
        Utility::eval_cubic_spline(spline_coef[i][d],spline_t[i],t,x[d],xd[d],xdd[d]);
      }
      stance_phase[i] = !stance_phase[i];
    }
    x.pose =  xd.pose = xdd.pose = base_frame;
  }

#ifdef VISUALIZE_MOBY
  if(false){
  for(int i=0;i<footholds.size();i++){
    Ravelin::Vector3d p = Ravelin::Pose3d::transform_point(Moby::GLOBAL,footholds[i]);
    visualize_ray(    p, p,   Ravelin::Vector3d(1,1,0), sim);
  }

  for(int i=0;i<NUM_EEFS;i++){

    for(int i=0;i<eefs_[i].chain.size();i++)
      joints_[eefs_[i].chain[i]]->q[0] = q[eefs_[i].chain[i]];
    abrobot_->update_link_poses();

    Ravelin::Vector3d pos = Ravelin::Pose3d::transform_point(Moby::GLOBAL,Ravelin::Vector3d(0,0,0,eefs_[i].link->get_pose()));

    Ravelin::VectorNd &T1 = *(spline_t[i].begin()),
                      &T2 = *(spline_t[i].rbegin());

    OUT_LOG(logDEBUG) << T1[0] <<" : " << t << " : "<< T2[T2.rows()-1] << " -- " << spline_t[i].size();

    for(double t=T1[0]+Moby::NEAR_ZERO ; t<=T2[T2.rows()-1] ; t += 0.01){
      Ravelin::Vector3d x(base_frame),xd(base_frame),xdd(base_frame);
      for(int d=0;d<3;d++){
        Utility::eval_cubic_spline(spline_coef[i][d],spline_t[i],t,x[d],xd[d],xdd[d]);
      }
      Ravelin::Vector3d p = Ravelin::Pose3d::transform_point(Moby::GLOBAL,x);
      Ravelin::Vector3d v = Ravelin::Pose3d::transform_vector(Moby::GLOBAL,xd)/10;
//      Ravelin::Vector3d a = Ravelin::Pose3d::transform_vector(Moby::GLOBAL,xdd)/100;
      visualize_ray(    p, p,   Ravelin::Vector3d(0,1,0), sim);
      visualize_ray(  v+p,   p,   Ravelin::Vector3d(1,0,0), sim);
//      visualize_ray(a+v+p, v+p, Ravelin::Vector3d(1,0.5,0), sim);
    }

//    Ravelin::Vector3d x(base_frame),xd(base_frame),xdd(base_frame);
//    for(int d=0;d<3;d++){
//      Utility::eval_cubic_spline(spline_coef[i][d],spline_t[i],t,x[d],xd[d],xdd[d]);
//    }
//    Ravelin::Vector3d p = Ravelin::Pose3d::transform_point(Moby::GLOBAL,x);
//    Ravelin::Vector3d v = Ravelin::Pose3d::transform_vector(Moby::GLOBAL,xd)/10;
//    Ravelin::Vector3d a = Ravelin::Pose3d::transform_vector(Moby::GLOBAL,xdd)/100;
//    visualize_ray(    p, p,   Ravelin::Vector3d(0,1,0), sim);
//    visualize_ray(  v+p,   p,   Ravelin::Vector3d(1,0,0), sim);
//    visualize_ray(a+v+p, v+p, Ravelin::Vector3d(1,0.5,0), sim);
  }
  }
#endif

  last_time = t;
  inited = true;
}
