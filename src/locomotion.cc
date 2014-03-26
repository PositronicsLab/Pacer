#include<quadruped.h>
#include<utilities.h>
using namespace Ravelin;
#define VISUALIZE_MOBY

void Quadruped::sinusoidal_trot(Ravelin::VectorNd& q_des,Ravelin::VectorNd& qd_des,Ravelin::VectorNd& qdd,double dt){
  static double t = 0;
  if(dt==0)
    t = 0;
  static Ravelin::VectorNd workv_,last_q_des = VectorNd::zero(NUM_JOINTS);
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

  std::vector<Ravelin::Vector3d> joint_positions(NUM_EEFS);
  feetIK(foot_poses,joint_positions);

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
//       q_des[eefs_[i].chain[j]] = joints_[eefs_[i].chain[j]]->q[0] + foot_vel[i][j]*dt;
       q_des[eefs_[i].chain[j]] = joint_positions[i][j];
    }
  }
  for(int i=0;i<NUM_JOINTS;i++)
    qdd[i] = (qd_des[i] - last_q_des[i])/dt;

  last_q_des = qd_des;
}



/// Walks while trying to match COM velocity "command" in base_frame
void Quadruped::walk_toward(const Ravelin::SVector6d& command,const std::vector<std::vector<int> >& gait,double t,Ravelin::VectorNd& q_des,Ravelin::VectorNd& qd_des,Ravelin::VectorNd& qdd_des){
  static bool inited = false, phase_change = true;
  static int spline_plan_length = 2;
  // Vector holding spline coefs
  static std::vector< std::vector< std::vector<Ravelin::VectorNd> > > spline_coef(NUM_EEFS);
  // Vector holding time delimitations to each spline
  static std::vector< std::vector<Ravelin::VectorNd> > spline_t(NUM_EEFS);
  assert(gait[0].size() == NUM_EEFS);

  static int PHASE = 0, N_PHASES = gait.size();

  static Ravelin::VectorNd duty_factor = Ravelin::VectorNd::zero(NUM_EEFS);

  if(!inited){
    for(int i=0;i<NUM_EEFS;i++){
      for(int j = 0; j<  N_PHASES;j++)
        duty_factor[i] += (gait[j][i] > 0)? 1.0 : 0.0;
      duty_factor[i] /= (double)N_PHASES;


      spline_coef[i].resize(3);
      for(int d=0; d<3;d++){
        spline_coef[i][d].resize(spline_plan_length);
        for(int j = 0; j<  spline_plan_length;j++)
          spline_coef[i][d][j] = Ravelin::VectorNd::zero(2);
      }
      spline_t[i].resize(spline_plan_length);
      for(int j = 0; j<  spline_plan_length;j++)
        spline_t[i][j] = Ravelin::VectorNd::zero(2);
    }
  }

  static std::vector<Ravelin::Vector3d> foot_pos(NUM_EEFS),
                                        foot_vel(NUM_EEFS),
                                        foot_acc(NUM_EEFS);

  Ravelin::VectorNd q = q_des,
                    qd = qd_des,
                    qdd = qdd_des;
  ///////////////////////////////////////////
  /////////////// PARAMETERS ////////////////
  const double phase_time =  0.15,
               step_height = 0.01,
      // ratio between gait time (phase_time*N_PHASES) and base differential time (1s)
               gait_ratio =  1.0/(phase_time * (double)N_PHASES);
  ///////////////////////////////////////////
  PHASE = (int)(t/phase_time) % N_PHASES;

    for(int i=0;i<NUM_EEFS;i++){
      std::vector< Ravelin::VectorNd >& t_limits = spline_t[i];

      // Check if Spline must be reevaluated
      bool replan_path = false;
      if(inited)
      for(int d=0; d<3;d++){
        replan_path = !eval_cubic_spline(spline_coef[i][d],spline_t[i],t,foot_pos[i][d],foot_vel[i][d],foot_acc[i][d]);
        if(replan_path) break;
      }
      if(replan_path || !inited ){
        inited = true;

        // creat new spline at top of history
        // take time at end of spline_t history
        double t0 = 0;

        Ravelin::Origin3d x,xd,xdd;
        if(t == 0){ // first iteration
          // Get Current Foot pos, Velocities & Accelerations
          x = Ravelin::Pose3d::transform_point(base_frame,Ravelin::Vector3d(0,0,0,eefs_[i].link->get_pose()));
          foot_jacobian(x,eefs_[i],workM_);
          workM_.transpose_mult(qd.select(eefs_[i].chain_bool,workv_),xd);
          workM_.transpose_mult(qdd.select(eefs_[i].chain_bool,workv_),xdd);
        } else {
          t0 = *(spline_t[i].back().end()-1) - Moby::NEAR_ZERO;

          for(int d=0; d<3;d++){
            bool pass = eval_cubic_spline(spline_coef[i][d],spline_t[i],t0,x[d],xd[d],xdd[d]);
            assert(pass);
          }
        }

        Ravelin::Origin3d to_com(x[0],x[1],0),
                          arc_step = Ravelin::Origin3d::cross(to_com,Ravelin::Origin3d(0,0,1)),
                          foot_goal = Ravelin::Origin3d(command[0],command[1],command[2]) + arc_step*command[5];

        std::vector<Ravelin::Origin3d> control_points;
        // TODO: Legs fall behind -- gate timing needs to be adjusted slightly
        if(gait[PHASE][i] > 0){        // swing phase
          // for x:
          control_points.push_back(x);
          control_points.push_back(x + Ravelin::Origin3d(0,0,step_height));
          control_points.push_back(x + Ravelin::Origin3d(foot_goal)*phase_time * (double)gait[PHASE][i] * gait_ratio * (1.0-duty_factor[i]) + Ravelin::Origin3d(0,0,step_height));
          control_points.push_back(x + Ravelin::Origin3d(foot_goal)*phase_time * (double)gait[PHASE][i] * gait_ratio * (1.0-duty_factor[i]));
        } else if(gait[PHASE][i] < 0){ // swing phase
          control_points.push_back(x);
          control_points.push_back(x - Ravelin::Origin3d(foot_goal)*phase_time * -(double)gait[PHASE][i] * gait_ratio * duty_factor[i]);
        }

        // create new spline!!
        // copy last spline to history erasing oldest spline
        for(int j=0;j<spline_plan_length-1;j++){
          for(int d=0; d<3;d++)
            spline_coef[i][d][j] = spline_coef[i][d][j+1];
          spline_t[i][j] = spline_t[i][j+1];
        }

        // create spline using set of control points, place at back of history
        for(int d=0;d<3;d++){
          int n = control_points.size();
          Ravelin::VectorNd X(n);
          Ravelin::VectorNd &T = *(spline_t[i].rbegin()),
                            &coefs = *(spline_coef[i][d].rbegin());

          T.set_zero(n);
          for(int j=0;j<n;j++){
            T[j] = t0 + (phase_time * fabs(gait[PHASE][i]) / (n-1)) * (double)j ;
            X[j] = control_points[j][d];
          }

          calc_cubic_spline_coefs(T,X,Ravelin::Vector2d(-foot_goal[d],-foot_goal[d]),Ravelin::Vector2d(0,0),coefs);

          eval_cubic_spline(spline_coef[i][d],spline_t[i],t,foot_pos[i][d],foot_vel[i][d],foot_acc[i][d]);
        }
      }
    }

  // EEF POSITION
  for(int i=0;i<NUM_EEFS;i++){
    RRMC(eefs_[i],q,foot_pos[i],q_des);
  }

  // EEF VELOCITY
  for(int i=0;i<NUM_EEFS;i++){
    EndEffector& foot = eefs_[i];
    // Calc jacobian for AB at this EEF
    Ravelin::MatrixNd J;
    Ravelin::Origin3d x;
    for(int k=0;k<foot.chain.size();k++)                // actuated joints
      x[k] = q[foot.chain[k]];
    foot_jacobian(x,foot,J);

    foot_vel[i].pose = base_frame;
    foot_vel[i] = Ravelin::Pose3d::transform_vector(foot.link->get_pose(),foot_vel[i]);
    LA_.solve_fast(J,foot_vel[i]);
    foot_acc[i].pose = base_frame;
    foot_acc[i] = Ravelin::Pose3d::transform_vector(foot.link->get_pose(),foot_acc[i]);
    LA_.solve_fast(J,foot_acc[i]);

    for(int j=0;j<eefs_[i].chain.size();j++){
      qdd_des[eefs_[i].chain[j]] = 0;//foot_acc[i][j];
      qd_des[eefs_[i].chain[j]] = 0;//foot_vel[i][j];
    }
  }

#ifdef VISUALIZE_MOBY
  for(int i=0;i<NUM_EEFS;i++){
    Ravelin::Vector3d pos = Ravelin::Pose3d::transform_point(Moby::GLOBAL,Ravelin::Vector3d(0,0,0,eefs_[i].link->get_pose()));

    Ravelin::VectorNd &T = *(spline_t[i].rbegin());

    for(double t=T[0];t<=T[T.rows()-1];t += (phase_time/10.0)){
      Ravelin::Vector3d x(base_frame),xd(base_frame),xdd(base_frame);
      for(int d=0;d<3;d++){
        eval_cubic_spline(spline_coef[i][d],spline_t[i],t,x[d],xd[d],xdd[d]);
      }
      Ravelin::Vector3d p = Ravelin::Pose3d::transform_point(Moby::GLOBAL,x);
      Ravelin::Vector3d v = Ravelin::Pose3d::transform_vector(Moby::GLOBAL,xd)/10;
      Ravelin::Vector3d a = Ravelin::Pose3d::transform_vector(Moby::GLOBAL,xdd)/10/10;
      visualize_ray( p, pos, Ravelin::Vector3d(0,0,0), sim);
      visualize_ray(v+p, p, Ravelin::Vector3d(1,0,0), sim);
      visualize_ray(a+v+p, v+p, Ravelin::Vector3d(1,0.5,0), sim);
    }

    Ravelin::Vector3d x(base_frame),xd(base_frame),xdd(base_frame);
    for(int d=0;d<3;d++){
      eval_cubic_spline(spline_coef[i][d],spline_t[i],t,x[d],xd[d],xdd[d]);
    }
    Ravelin::Vector3d p = Ravelin::Pose3d::transform_point(Moby::GLOBAL,x);
    visualize_ray( p, pos, Ravelin::Vector3d(0,1,0), sim);

  }

#endif
}
