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

  // Additional Aprameters for CPG
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
    RRMC(eefs_[i],q_des,foot_poses[i],q_des);
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

// Decreases the latency of a gait by reducing t_I and keeping the same phase duration
std::vector<std::vector<int> >& Quadruped::expand_gait(const std::vector<std::vector<int> >& gait1,int m, std::vector<std::vector<int> >& gait2){
  int n = gait1.size();
  gait2.resize(n*m);
  for(int i=0;i<n;i++){
    for(int j=0;j<m;j++){
      gait2[i*m+j].resize(gait1[i].size());
      for(int f=0;f<gait1[i].size();f++){
        gait2[i*m+j][f] = gait1[i][f]*m + ((gait1[i][f]>0)? -1.0 : 1.0)*j;
      }
    }
  }
  return gait2;
}

/// Generate a bunch of random points at z = 0 around the COM
void Quadruped::find_footholds(std::vector<Ravelin::Vector3d>& footholds,int num_footholds){
  double rad = 0.2;
  for(int i=0;i<num_footholds;i++)
    footholds.push_back(
          Ravelin::Pose3d::transform_point(
            base_frame,
            Ravelin::Vector3d(
              center_of_mass_x[0] + (double)rand()/RAND_MAX * 2.0 * rad - rad,center_of_mass_x[0] + (double)rand()/RAND_MAX * 2.0 * rad - rad,0.001
              ,Moby::GLOBAL
            )
          )
        );
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

  v_bar.set_sub_vec(NUM_EEFS*3,v_base);
  for(int i=0;i<NUM_EEFS;i++){
//    for(int i=0;i<eefs_[i].chain.size();i++)
//      joints_[eefs_[i].chain[i]]->q[0] = q[eefs_[i].chain[i]];
//    abrobot_->update_link_poses();

    Ravelin::Vector3d pos = Ravelin::Pose3d::transform_point(base_frame,Ravelin::Vector3d(0,0,0,eefs_[i].link->get_pose()));

    v_bar.set_sub_vec(i*3,foot_vel[i] + beta/dt * (foot_pos[i] - pos));
//    v_bar.set_sub_vec(i*3,foot_vel[i]);
  }
  OUTLOG(v_bar,"v_bar",logERROR);
}


void Quadruped::trajectory_ik(const std::vector<Ravelin::Vector3d>& foot_pos,const std::vector<Ravelin::Vector3d>& foot_vel,const std::vector<Ravelin::Vector3d>& foot_acc,
                              Ravelin::VectorNd& q_des,Ravelin::VectorNd& qd_des,Ravelin::VectorNd& qdd_des){
  ////////////////////// IK CONTROL ////////////////////////
  OUT_LOG(logDEBUG) << "Resulting Controls:";

  Ravelin::VectorNd q = q_des,
                    qd = qd_des,
                    qdd = qdd_des;

  for(int i=0;i<NUM_EEFS;i++){
    EndEffector& foot = eefs_[i];

    // POSITION
    OUT_LOG(logDEBUG) << "\t" << foot.id << "_x =" << foot_pos[i];
    RRMC(foot,q,foot_pos[i],q_des);
    OUT_LOG(logDEBUG) << "\t" << foot.id << "_q =" << q_des.select(foot.chain_bool,workv_);

    // Calc jacobian for AB at this EEF
    Ravelin::MatrixNd J;
    Ravelin::Origin3d x;
    for(int k=0;k<foot.chain.size();k++)                // actuated joints
      x[k] = q[foot.chain[k]];
    foot_jacobian(x,foot,base_frame,J);

    Ravelin::Vector3d qd_foot,qdd_foot;
    // VELOCITY & ACCELERATION
    OUT_LOG(logDEBUG) << "\t" << foot.id << "_xd =" << foot_vel[i];
    LA_.solve_fast((workM_ = J),(qd_foot = foot_vel[i]));
    OUT_LOG(logDEBUG) << "\t" << foot.id << "_qd =" << foot_vel[i];

    OUT_LOG(logDEBUG) << "\t" << foot.id << "_xdd =" << foot_acc[i];
    LA_.solve_fast((workM_ = J),(qdd_foot = foot_acc[i]));
    OUT_LOG(logDEBUG) << "\t" << foot.id << "_qdd =" << foot_acc[i];

    for(int j=0;j<foot.chain.size();j++){
      qdd_des[foot.chain[j]] = foot_acc[i][j];
      qd_des[foot.chain[j]] = foot_vel[i][j];
    }
  }
}

/// Walks while trying to match COM velocity "command" in base_frame
void Quadruped::walk_toward(const Ravelin::SVector6d& command,const std::vector<std::vector<int> >& gait,const std::vector<Ravelin::Vector3d>& footholds,double  interval_time,double step_height,double t,
                           const Ravelin::VectorNd& q,const Ravelin::VectorNd& qd,const Ravelin::VectorNd& qdd,
                           std::vector<Ravelin::Vector3d>& foot_pos, std::vector<Ravelin::Vector3d>& foot_vel, std::vector<Ravelin::Vector3d>& foot_acc){
  OUT_LOG(logDEBUG) << " -- Quadruped::walk_toward() entered";
  static bool inited = false;
  static int spline_plan_length = 2, last_time = t;

  if(last_time - t >  interval_time)
    inited = false;

  // persistent Vector storing spline coefs
  // [foot][dimension][interval]
  static std::vector< std::vector< std::vector<alglib::spline1dinterpolant> > > spline_coef(NUM_EEFS);

  // persistent Vector storing time delimitations to each spline
  // [foot][interval]
  static std::vector< std::vector<Ravelin::VectorNd> > spline_t(NUM_EEFS);
  assert(gait[0].size() == NUM_EEFS);

  static std::vector<double> duty_factor(NUM_EEFS);

  static int PHASE = 0, N_PHASES = gait.size();

  foot_pos.resize(NUM_EEFS);
  foot_vel.resize(NUM_EEFS);
  foot_acc.resize(NUM_EEFS);

  // Populate spline vectors
  if(!inited){
    for(int i=0;i<NUM_EEFS;i++){
      //calculate step to swing ratio throughout the gait
      for(int j = 0; j<  N_PHASES;j++)
        duty_factor[i] += (gait[j][i] > 0)? 1.0 : 0.0;
      duty_factor[i] /= (double)N_PHASES;

      spline_coef[i].resize(3);
      for(int d=0; d<3;d++){
        spline_coef[i][d].resize(spline_plan_length);
        for(int j = 0; j<  spline_plan_length;j++)
          spline_coef[i][d][j] = alglib::spline1dinterpolant();
      }
      spline_t[i].resize(spline_plan_length);
      for(int j = 0; j<  spline_plan_length;j++)
        spline_t[i][j] = Ravelin::VectorNd::zero(2);
    }
  }

  // What phase of the gait is the controller in?
  // Evaluate G at interval I(t)
  PHASE = (int)(t/ interval_time + Moby::NEAR_ZERO) % N_PHASES;

  OUT_LOG(logDEBUG) << " Phase: " << PHASE << "/" << N_PHASES;

  ////////////////// PHASE PLANNING ///////////////////////
  for(int i=0;i<NUM_EEFS;i++){
    foot_pos[i].set_zero();
    foot_vel[i].set_zero();
    foot_acc[i].set_zero();

    OUT_LOG(logDEBUG) << eefs_[i].id;
    OUT_LOG(logDEBUG) << "\tgait : " << gait[PHASE][i];
    double fractpart, intpart;
    fractpart = modf((t/ interval_time) , &intpart);
    OUT_LOG(logDEBUG) << "\tprog : " << fractpart;

    std::vector< Ravelin::VectorNd >& t_limits = spline_t[i];

    // Check if Spline must be reevaluated
    bool replan_path = false;
    if(inited)
    for(int d=0; d<3;d++){
      replan_path = !Utility::eval_cubic_spline(spline_coef[i][d],spline_t[i],t,foot_pos[i][d],foot_vel[i][d],foot_acc[i][d]);
      if(replan_path) break;
    }

    // Plan a new spline for this foot
    if(replan_path || !inited ){
      OUT_LOG(logDEBUG) << "\tPlanning next Spline";
      // creat new spline at top of history
      // take time at end of spline_t history
      double t0 = Moby::NEAR_ZERO;

      Ravelin::Origin3d x,xd,xdd;

      if(!inited){ // first iteration
        // Get Current Foot pos, Velocities & Accelerations
        x = Ravelin::Pose3d::transform_point(base_frame,Ravelin::Vector3d(0,0,0,eefs_[i].link->get_pose()));
        foot_jacobian(x,eefs_[i],base_frame,workM_);
        workM_.transpose_mult(qd.select(eefs_[i].chain_bool,workv_),xd);
        workM_.transpose_mult(qdd.select(eefs_[i].chain_bool,workv_),xdd);
      } else {
        // continue off of the end of the last spline
        t0 = *(spline_t[i].back().end()-1) - Moby::NEAR_ZERO;

        for(int d=0; d<3;d++){
          bool pass = Utility::eval_cubic_spline(spline_coef[i][d],spline_t[i],t0,x[d],xd[d],xdd[d]);
          assert(pass);
        }
      }

      // Calculate foot-step info
      // Determine linear portion of step
      Ravelin::Origin3d from_com(x[0],x[1],0),
                         yaw_step,
                        foot_goal = Ravelin::Origin3d(command[0],command[1],command[2]);

      // Determine shape of step needed to yaw robot
      Ravelin::AAngled aa_gamma(center_of_contact.normal,command[5] *  interval_time * (double)gait[PHASE][i]);
      OUTLOG(aa_gamma,"aa_gamma",logDEBUG);
      Ravelin::Matrix3d R_gamma;
      R_gamma =aa_gamma;
      OUTLOG(R_gamma,"R_gamma",logDEBUG);
      // yaw_step = R(gamma)*r_com - r_com
      R_gamma.mult(from_com, yaw_step) -= from_com;
      OUTLOG( yaw_step," yaw_step",logDEBUG);

      // scale step length by interval duration
      foot_goal *=  interval_time* (double)gait[PHASE][i];

      // Combine x, y, and yaw to for full step
      foot_goal +=  yaw_step;

      if(!inited) foot_goal /= 2;  // only works for 2 interval gait

      OUT_LOG(logDEBUG) << "\tstep = " << foot_goal;

      // Set control point for either stance or swing phase
      std::vector<Ravelin::Origin3d> control_points;

      if(gait[PHASE][i] > 0){
        foot_goal /= duty_factor[i];
        if(footholds.empty()){
          control_points.push_back(x);
          control_points.push_back(x + Ravelin::Origin3d(0,0,step_height));
          control_points.push_back(x + foot_goal + Ravelin::Origin3d(0,0,step_height));
          control_points.push_back(x + foot_goal);
        } else {
        Ravelin::Origin3d x_fh;
        // SWING PHASE
        // Select which foothold to use
        select_foothold(footholds,x+foot_goal,x_fh);

        // Reach new foothold, account for movement of robot during step
        control_points.push_back(x);
        control_points.push_back(x - foot_goal*0.33 + Ravelin::Origin3d(0,0,step_height));
        control_points.push_back(x_fh - foot_goal*0.66 + Ravelin::Origin3d(0,0,step_height));
        control_points.push_back(x_fh - foot_goal);
        }
      }
      else if(gait[PHASE][i] < 0){
        foot_goal /= (1-duty_factor[i]);

        // all stance phase cases
        control_points.push_back(x);
        control_points.push_back(x + foot_goal) ;
      }

      // create new spline!!
      // copy last spline to history erasing oldest spline
      for(int j=0;j<spline_plan_length-1;j++){
        for(int d=0; d<3;d++)
          spline_coef[i][d][j] = spline_coef[i][d][j+1];
        spline_t[i][j] = spline_t[i][j+1];
      }

      Ravelin::Vector3d foot_velocity = Ravelin::Vector3d(foot_goal +  yaw_step,base_frame);
      // create spline using set of control points, place at back of history
      for(int d=0;d<3;d++){
        int n = control_points.size();
        Ravelin::VectorNd X(n);
        Ravelin::VectorNd           &T     = *(spline_t[i].rbegin());
        alglib::spline1dinterpolant &coefs = *(spline_coef[i][d].rbegin());

        T.set_zero(n);
        for(int j=0;j<n;j++){
          T[j] = t0 + ( interval_time * fabs(gait[PHASE][i]) / (n-1)) * (double)j ;
          X[j] = control_points[j][d];
        }
        OUTLOG(T,"T",logERROR);
        OUTLOG(X,"X",logERROR);

        Utility::calc_cubic_spline_coefs(T,X,Ravelin::Vector2d(xd[d],foot_velocity[d]),coefs);

        // then re-evaluate spline
        // NOTE: this will only work if we replanned for a t_0  <  t  <  t_0 + t_I
        Utility::eval_cubic_spline(spline_coef[i][d],spline_t[i],t,foot_pos[i][d],foot_vel[i][d],foot_acc[i][d]);
      }
    }
    foot_acc[i].pose = base_frame;
    foot_pos[i].pose = base_frame;
    foot_vel[i].pose = base_frame;
    // Define output in environment frame
//    foot_acc[i] = Ravelin::Pose3d::transform_vector(environment_frame,foot_acc[i]);
//    foot_vel[i] = Ravelin::Pose3d::transform_vector(environment_frame,foot_vel[i]);
//    foot_pos[i] = Ravelin::Pose3d::transform_point(environment_frame,foot_pos[i]);

  }



#ifdef VISUALIZE_MOBY
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

    for(double t=T1[0]+Moby::NEAR_ZERO ; t<=T2[T2.rows()-1] ; t += ( interval_time/10.0) / (double)spline_t[i].size()){
      Ravelin::Vector3d x(base_frame),xd(base_frame),xdd(base_frame);
      for(int d=0;d<3;d++){
        Utility::eval_cubic_spline(spline_coef[i][d],spline_t[i],t,x[d],xd[d],xdd[d]);
      }
      Ravelin::Vector3d p = Ravelin::Pose3d::transform_point(Moby::GLOBAL,x);
      Ravelin::Vector3d v = Ravelin::Pose3d::transform_vector(Moby::GLOBAL,xd)/10;
      Ravelin::Vector3d a = Ravelin::Pose3d::transform_vector(Moby::GLOBAL,xdd)/100;
      visualize_ray(    p, p,   Ravelin::Vector3d(0,0,0), sim);
//      visualize_ray(  v+p,   p,   Ravelin::Vector3d(1,0,0), sim);
//      visualize_ray(a+v+p, v+p, Ravelin::Vector3d(1,0.5,0), sim);
    }

    Ravelin::Vector3d x(base_frame),xd(base_frame),xdd(base_frame);
    for(int d=0;d<3;d++){
      Utility::eval_cubic_spline(spline_coef[i][d],spline_t[i],t,x[d],xd[d],xdd[d]);
    }
    Ravelin::Vector3d p = Ravelin::Pose3d::transform_point(Moby::GLOBAL,x);
    Ravelin::Vector3d v = Ravelin::Pose3d::transform_vector(Moby::GLOBAL,xd)/10;
    Ravelin::Vector3d a = Ravelin::Pose3d::transform_vector(Moby::GLOBAL,xdd)/100;
//    visualize_ray(    p, pos,   Ravelin::Vector3d(0,0,0), sim);
//    visualize_ray(  v+p,   p,   Ravelin::Vector3d(1,0,0), sim);
//    visualize_ray(a+v+p, v+p, Ravelin::Vector3d(1,0.5,0), sim);
  }
#endif

  last_time = t;
  inited = true;
}
