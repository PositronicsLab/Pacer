#include<quadruped.h>
using namespace Ravelin;

void Quadruped::sinusoidal_trot(Ravelin::VectorNd& q_des,Ravelin::VectorNd& qd_des,Ravelin::VectorNd& qdd,double dt){
  static double t = 0;
  static Ravelin::VectorNd workv_,last_q_des = VectorNd::zero(NUM_JOINTS);
  static Ravelin::MatrixNd workM_;
  t += dt;

  std::vector<Ravelin::Vector3d> foot_vel(NUM_EEFS),foot_poses(NUM_EEFS);
  Ravelin::VectorNd Hs(NUM_EEFS);

  // Additional Aprameters for CPG
  double Ls = 0.02, Vf = 10;
  for(int f=0;f<NUM_EEFS;f++)
    // set height of gait (each foot)
    Hs[f] = 0.015;

  // Foot goal position
  for(int f=0;f<NUM_EEFS;f++){
    if(f==0 || f==3) {
      foot_poses[f] = eefs_[f].origin+Ravelin::Vector3d(Ls*sin(M_PI_2+t*Vf),0,Hs[f]*cos(M_PI_2+t*Vf));
      foot_vel[f] = Ravelin::Vector3d(Vf*Ls*cos(M_PI_2+t*Vf),0,-Vf*Hs[f]*sin(M_PI_2+t*Vf));
    } else {
      foot_poses[f] = eefs_[f].origin+Ravelin::Vector3d(-Ls*sin(M_PI_2+t*Vf),0,Hs[f]*cos(M_PI_2+t*-Vf));
      foot_vel[f] = Ravelin::Vector3d(-Vf*Ls*cos(M_PI_2+t*Vf),0,Vf*Hs[f]*sin(M_PI_2+t*-Vf));
    }
  }

  std::vector<Ravelin::Vector3d> joint_positions(NUM_EEFS);
  feetIK(foot_poses,joint_positions);

  // Foot goal Velocity
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

  // Use Positional (numerical velocity) Control
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

void Quadruped::cpg_trot(Ravelin::VectorNd& q_des,Ravelin::VectorNd& qd_des,Ravelin::VectorNd& qdd,double dt){
  static Ravelin::VectorNd workv_;
  static Ravelin::MatrixNd workM_;

  std::vector<Ravelin::Vector3d> foot_vel(NUM_EEFS),foot_origins(NUM_EEFS),foot_poses(NUM_EEFS);
  Ravelin::VectorNd Hs(NUM_EEFS);
  Ravelin::MatrixNd C(NUM_EEFS,NUM_EEFS);

  // SRZ: this is the coupling matrix C
  // see: Pattern generators with sensory feedback for the control of quadruped locomotion
  C.set_zero();
  unsigned gait_pattern = 0;
  switch(gait_pattern){
    case 0: // Trotting gait
                   C(0,1) = -1; C(0,2) = -1; C(0,3) =  1;
      C(1,0) = -1;              C(1,2) =  1; C(1,3) = -1;
      C(2,0) = -1; C(2,1) =  1;              C(2,3) = -1;
      C(3,0) =  1; C(3,1) = -1; C(3,2) = -1;
      break;
    case 1: // Pacing gait
                   C(0,1) = -1; C(0,2) =  1; C(0,3) = -1;
      C(1,0) = -1;              C(1,2) = -1; C(1,3) =  1;
      C(2,0) =  1; C(2,1) = -1;              C(2,3) = -1;
      C(3,0) = -1; C(3,1) =  1; C(3,2) = -1;
    break;
    case 2: // Bounding gait
                   C(0,1) =  1; C(0,2) = -1; C(0,3) = -1;
      C(1,0) =  1;              C(1,2) = -1; C(1,3) = -1;
      C(2,0) = -1; C(2,1) = -1;              C(2,3) =  1;
      C(3,0) = -1; C(3,1) = -1; C(3,2) =  1;
    break;
    case 3: // Walking gait
                   C(0,1) = -1; C(0,2) =  1; C(0,3) = -1;
      C(1,0) = -1;              C(1,2) = -1; C(1,3) =  1;
      C(2,0) = -1; C(2,1) =  1;              C(2,3) = -1;
      C(3,0) =  1; C(3,1) = -1; C(3,2) = -1;
    break;
    case 4: // squatting
                   C(0,1) =  1; C(0,2) = -1; C(0,3) =  1;
      C(1,0) =  1;              C(1,2) =  1; C(1,3) = -1;
      C(2,0) = -1; C(2,1) =  1;              C(2,3) =  1;
      C(3,0) =  1; C(3,1) = -1; C(3,2) =  1;
    break;
  }

  // Additional Aprameters for CPG
  double Ls = 0.02,
         Df = 0.55, // where 50% –_–_–_ 100% ======  *Duty factor
                    //           –_–_–_      ======
         Vf = 0.22,
         bp = 10000;

  for(int f=0;f<NUM_EEFS;f++){
    // set height of gait (each foot)
    Hs[f] = 0.005;

    // set gait centers
    Ravelin::Pose3d link_pose = *eefs_[f].link->get_pose();
    link_pose.update_relative_pose(boost::shared_ptr<const Ravelin::Pose3d>(new Ravelin::Pose3d(base_horizonal_frame)));
    foot_origins[f] = eefs_[f].origin;
    foot_poses[f] = link_pose.x;
    std::cout << eefs_[f].id << " " << foot_poses[f] << std::endl;
    foot_vel[f].set_zero();
  }

  // retrieve oscilator value
  foot_oscilator(foot_origins,foot_poses,C,Ls,Hs,Df,Vf,bp,foot_vel);

  // Foot goal position
  for(int f=0;f<NUM_EEFS;f++){
    foot_poses[f] += (foot_vel[f]*dt);
  }
  std::vector<Ravelin::Vector3d> joint_positions(NUM_EEFS);
  feetIK(foot_poses,joint_positions);

  for(int f=0;f<NUM_EEFS;f++)
    std::cout << foot_poses[f] << " -> " << joint_positions[f] << std::endl;

  // Foot goal Velocity
  for(int f=0;f<NUM_EEFS;f++){
    // Calc jacobian for AB at this EEF
    boost::shared_ptr<Ravelin::Pose3d> event_frame(new Ravelin::Pose3d(Moby::GLOBAL));
    event_frame->q = Ravelin::Quatd::identity();
    event_frame->x = foot_poses[f];
    dbrobot_->calc_jacobian(event_frame,eefs_[f].link,workM_);

    assert(eefs_[f].chain.size() == 3);
    Ravelin::MatrixNd iJ(3,3);
    for(int i=0;i<eefs_[f].chain.size();i++)
      for(int j=0;j<3;j++)
        iJ(j,i) = workM_(j,eefs_[f].chain[i]);
    LA_.solve_fast(iJ,foot_vel[f]);
  }

  // Use Positional (numerical velocity) Control
  for(int i=0;i<NUM_EEFS;i++){
    for(int j=0;j<eefs_[i].chain.size();j++){
      qd_des[eefs_[i].chain[j]] = foot_vel[i][j];
       q_des[eefs_[i].chain[j]] = joints_[eefs_[i].chain[j]]->q[0] + foot_vel[i][j]*dt;
        q_des[eefs_[i].chain[j]] = joint_positions[i][j];
    }
  }
}
