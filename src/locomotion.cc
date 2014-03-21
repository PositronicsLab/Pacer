#include<quadruped.h>
#include<utilities.h>
using namespace Ravelin;
//#define VISUALIZE_MOBY

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
void Quadruped::walk_toward(const Ravelin::SVector6d& command,const Ravelin::VectorNd& q,Ravelin::VectorNd& q_des,Ravelin::VectorNd& qd_des,Ravelin::VectorNd& qdd,double dt){
  static double t = 0;
  static int trajectory_segment = 0;
  static std::vector<Ravelin::Vector3d> foot_vel(NUM_EEFS),foot_acc(NUM_EEFS);
  static Ravelin::VectorNd last_q_des = q_des,
                          last_qd_des = Ravelin::VectorNd::zero(NUM_JOINTS);

  static enum Mode{
    SWING = 0,
    STANCE = 1,
    N_MODES = 2
  } MODE = STANCE;
  static int SWING_LEG = 0;
  t += dt;

  ///////////////////////////////////////////
  /////////////// PARAMETERS ////////////////
  double swing_time = 0.1,
         stance_time = 0.0,
         step_height = 0.02;
  double phase_time = (swing_time + stance_time)*4;
  int num_segments = (swing_time/dt) + 1;
  ///////////////////////////////////////////
  // Progress state
  if((MODE == SWING && trajectory_segment>=num_segments-1) ||
     (MODE == STANCE && t>=stance_time) ){
    // SWITCH MODE
    MODE = Mode( (MODE+STANCE) % N_MODES);
    if(MODE == SWING)
      SWING_LEG = (SWING_LEG+1)%NUM_EEFS;
    t=0;
    trajectory_segment = 0;
  }

  std::cout << "Stepping with : " << eefs_[SWING_LEG].id << std::endl;
  std::cout << "Controller Mode : " << MODE << std::endl;

  for(int i=0;i<NUM_EEFS;i++){
    if(MODE == SWING || true){ // TODO add stance phase code (move into support)
      if(i == SWING_LEG && MODE == SWING)
        continue;
      Ravelin::Vector3d foot_goal,
                        to_com = Ravelin::Vector3d(eefs_[i].origin[0],eefs_[i].origin[1],0),
                        arc_step = Ravelin::Vector3d::cross(to_com,Ravelin::Vector3d(0,0,1));
      arc_step.pose = base_frame;

      foot_goal = Ravelin::Vector3d(command[0],command[1],command[2],base_frame) + arc_step*command[5];

      // move feet back at COM goal vel in DT increment
      // scaled up by ammount of time spent on swing phase
      foot_vel[i] = -foot_goal/(1 - swing_time/phase_time);
      foot_acc[i].set_zero();
      eefs_[i].origin += dt*foot_vel[i];

#ifdef VISUALIZE_MOBY
      Ravelin::Vector3d workv3_ = base_frame->transform_vector(-foot_goal);
      Ravelin::Vector3d foot_origin = base_frame->transform_point(eefs_[i].origin);
      visualize_ray(workv3_+foot_origin,foot_origin,Ravelin::Vector3d(0,1,0),sim);
#endif
    }
  }

  static std::vector<Ravelin::Vector3d> swing_trajectory,d_swing_trajectory,dd_swing_trajectory;

  if(MODE == SWING){
    if(trajectory_segment==0){
      Ravelin::Vector3d foot_goal, to_com, arc_step;
      to_com = Ravelin::Vector3d(eefs_[SWING_LEG].origin[0],eefs_[SWING_LEG].origin[1],0);
      arc_step = Ravelin::Vector3d::cross(to_com,Ravelin::Vector3d(0,0,1));
      arc_step.pose = base_frame;

      foot_goal = Ravelin::Vector3d(command[0],command[1],command[2],base_frame) + arc_step*command[5];

      std::vector<Ravelin::Origin3d> control_points;
      // for x:
      control_points.push_back(Ravelin::Origin3d(eefs_[SWING_LEG].origin));
      control_points.push_back(Ravelin::Origin3d(eefs_[SWING_LEG].origin) + Ravelin::Origin3d(0,0,step_height));
      control_points.push_back(Ravelin::Origin3d(eefs_[SWING_LEG].origin) + Ravelin::Origin3d(foot_goal)*phase_time + Ravelin::Origin3d(0,0,step_height));
      control_points.push_back(Ravelin::Origin3d(eefs_[SWING_LEG].origin) + Ravelin::Origin3d(foot_goal)*phase_time);

      // Handle Swing phase leg
//#define BEZIER_CURVE
#ifdef BEZIER_CURVE
      bezierCurve(control_points,num_segments,swing_trajectory,d_swing_trajectory,dd_swing_trajectory);
#else
      for(int i=0;i<3;i++){
        int n = control_points.size();
        Ravelin::VectorNd T(n),B,X(n),x,xd,xdd;
        for(int j=0;j<n;j++){
          T[j] = (swing_time/(n-1)) * (double)(j) ;
          X[j] = control_points[j][i];
        }
        calc_cubic_spline_coefs(T,X,Ravelin::Vector2d(-foot_goal[i],-foot_goal[i]),Ravelin::Vector2d(0,0),B);
        eval_cubic_spline(B,T,num_segments,x,xd,xdd);

        swing_trajectory.resize(num_segments);
        d_swing_trajectory.resize(num_segments);
        dd_swing_trajectory.resize(num_segments);
        for(int t=0;t<num_segments;t++){
          swing_trajectory[t][i] = x[t];
          d_swing_trajectory[t][i] = xd[t];
          dd_swing_trajectory[t][i] = xdd[t];
        }
      }
#endif
    }

    if(!swing_trajectory.empty()){
#ifdef VISUALIZE_MOBY
      for(int i=0;i<swing_trajectory.size()-2;i+=num_segments/10){
        swing_trajectory[i].pose = base_frame;
        Ravelin::Vector3d p = base_frame->transform_point(swing_trajectory[i]);
        visualize_ray(p,p,Ravelin::Vector3d(0,1,0),sim);
//        Ravelin::Vector3d v = base_frame->transform_point(d_swing_trajectory[i]);
//        visualize_ray(p+v*0.1,p,Ravelin::Vector3d(0,0,1),sim);
//        Ravelin::Vector3d a = base_frame->transform_point(dd_swing_trajectory[i]);
//        visualize_ray(p+v*0.1+a*0.1*0.1,p+v*0.1,Ravelin::Vector3d(1,0,1),sim);
      }
#endif

      eefs_[SWING_LEG].origin = swing_trajectory[trajectory_segment];
      foot_vel[SWING_LEG] = d_swing_trajectory[trajectory_segment];
      foot_acc[SWING_LEG] = dd_swing_trajectory[trajectory_segment];
      trajectory_segment += 1;
    }
  }

  // EEF POSITION
  std::vector<Ravelin::Vector3d> joint_positions(NUM_EEFS);
  for(int i=0;i<NUM_EEFS;i++){
//    footIK(i,eefs_[i].origin,joint_positions[i]);
    RRMC(eefs_[i],q,eefs_[i].origin,q_des);
  }

  // EEF VELOCITY
  for(int i=0;i<NUM_EEFS;i++){
    EndEffector& foot = eefs_[i];
    // Calc jacobian for AB at this EEF
    Ravelin::MatrixNd J;
    Ravelin::VectorNd x(foot.chain.size());
    for(int k=0;k<foot.chain.size();k++)                // actuated joints
      x[k] = q[foot.chain[k]];
    foot_kinematics(x,foot,workv3_,workv3_,J);

    foot_vel[i].pose = base_frame;
    foot_vel[i] = Ravelin::Pose3d::transform_point(foot.link->get_pose(),foot_vel[i]);
    LA_.solve_fast(J,foot_vel[i]);
    foot_acc[i].pose = base_frame;
    foot_acc[i] = Ravelin::Pose3d::transform_point(foot.link->get_pose(),foot_acc[i]);
    LA_.solve_fast(J,foot_acc[i]);

    for(int j=0;j<eefs_[i].chain.size();j++){
      qdd[eefs_[i].chain[j]] = foot_acc[i][j];
      qd_des[eefs_[i].chain[j]] = foot_vel[i][j];
    }
  }
}
