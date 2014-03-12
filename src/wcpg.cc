#include <quadruped.h>
#include <utilities.h>

std::vector<Ravelin::Vector3d>& Quadruped::foot_oscilator(
  const std::vector<Ravelin::Vector3d>& x0,  const std::vector<Ravelin::Vector3d>& x, const Ravelin::MatrixNd& C,
  double Ls,const Ravelin::VectorNd& Hs,double Df,double Vf,double bp,std::vector<Ravelin::Vector3d>& xd){

  /* Tunable parameters
   * Ls    : length of step
   * Hs    : height of step
   * Df    : step duty factor
   * Vf    : forward velocity
   * bp    : the transition rate between phases
   * bf    : the transition rate between phases
   */
  // a/b/c : affect the convergence rate of the limit cycle
  double a = 10,
         b = -1,
         c = 10;

  // Stepping Filter params
  // depth of step phase where touchdown occurs (fraction of Hs)
  double ztd = 0.0;
  // speed at which behavior changes (arbitrary scale)
  double bf = 1000;

  std::vector<Ravelin::Vector3d> xb(NUM_EEFS);
  for(int i=0;i<NUM_EEFS;i++)
    xb[i] = x[i] - x0[i];

  for(int i=0;i<NUM_EEFS;i++){
    double Cp = 0;
    // Eqn: 3
    for(int j=0;j<NUM_EEFS;j++)
      Cp += C(i,j)*Hs[i]*xb[j][2]/Hs[j];

    Ravelin::Vector3d& xbar = xb[i];

    // Eqn 4, 5, 6
    double Sp1 = 1.0/(exp(-bp*xbar[2]) + 1.0);
    double Sp2 = 1.0/(exp( bp*xbar[2]) + 1.0);
    double ws  = M_PI * (Vf/Ls) * ((Df*Sp1)/(1.0-Df) + Sp2);

    // realtively thin gait [shoulder width]
    double dyc = 0;

    // \dot{x}
    // Eqn: 1, 2, 3
    double oscil = 1.0 - (4*xbar[0]*xbar[0])/(Ls*Ls) - (xbar[2]*xbar[2])/(Hs[i]*Hs[i]) ;
    xd[i][0] = a*oscil*xbar[0] + (ws*Ls*xbar[2])/(2*Hs[i]);
    xd[i][1] = b*(xbar[1] + dyc);
    xd[i][2] = c*oscil*xbar[2] - (ws*2*Hs[i]*xbar[0])/Ls + Cp;

    // Stepping Terrain Filter
    // Eqns: 7, 8, 9
    double dist_plane = distance_from_plane(center_of_contact.normal,
                                            center_of_contact.point,x[i]);
    std::cout << eefs_[i].id << " " << dist_plane << std::endl;
    double Sf1 = 1.0/(exp(-bf*(dist_plane)) + 1.0);
    double Sf2 = 1.0/(exp( bf*(dist_plane)) + 1.0);
//    double Sf1 = 1.0/(exp(-bf*(xbar[2] - ground_plane(x[i]))) + 1.0);
//    double Sf2 = 1.0/(exp( bf*(xbar[2] - ground_plane(x[i]))) + 1.0);
    xd[i] = (xd[i])*Sf1 - Ravelin::Vector3d(Vf,0,0)*Sf2;
  }

  return xd;
}

void Quadruped::cpg_trot(Ravelin::VectorNd& q_des,Ravelin::VectorNd& qd_des,Ravelin::VectorNd& qdd,double dt){
  static double t = 0;
  if(dt==0)
    t = 0;
  static Ravelin::VectorNd workv_,last_q_des = Ravelin::VectorNd::zero(NUM_JOINTS);
  static Ravelin::MatrixNd workM_;
  t += dt;

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
  double Ls = 0.03,
         Df = 0.45, // where 50% –_–_–_ 100% ======  *Duty factor
                    //           –_–_–_      ======
         Vf = 0.15,
         bp = 5000;

  for(int f=0;f<NUM_EEFS;f++){
    // set height of gait (each foot)
    Hs[f] = 0.015;

    // SET EEF ORIGINS TO the ground below that EEF SHOULDER
    Ravelin::Pose3d joint_pose = *joints_[eefs_[f].chain[2]]->get_pose();
    joint_pose.update_relative_pose(Moby::GLOBAL);
    joint_pose.x[2] = Hs[f]*0.75;
    joint_pose.update_relative_pose(base_horizontal_frame);
    eefs_[f].origin = joint_pose.x;

    // set gait centers
    Ravelin::Pose3d link_pose = *eefs_[f].link->get_pose();
    link_pose.update_relative_pose(base_horizontal_frame);
    foot_origins[f] = eefs_[f].origin;
    foot_origins[f].pose = base_horizontal_frame;
    foot_poses[f] = link_pose.x;
    foot_poses[f].pose = base_horizontal_frame;
    if(link_pose.x.norm() < 1e-3) return;
    foot_vel[f].set_zero();
  }

  // retrieve oscilator value
  foot_oscilator(foot_origins,foot_poses,C,Ls,Hs,Df,Vf,bp,foot_vel);

  // Foot goal position
//  for(int f=0;f<NUM_EEFS;f++)
//    foot_poses[f] += (foot_vel[f]*dt);

//  std::vector<Ravelin::Vector3d> joint_positions(NUM_EEFS);
//  feetIK(foot_poses,joint_positions);

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
    try{
    LA_.solve_fast(iJ,foot_vel[f]);                           // qd <- Ji\xd
    } catch(Ravelin::SingularException e){
      foot_vel[f].set_zero();
    }
  }


  for(int i=0;i<NUM_EEFS;i++){
    for(int j=0;j<eefs_[i].chain.size();j++){
      qd_des[eefs_[i].chain[j]] = foot_vel[i][j];
       q_des[eefs_[i].chain[j]] = joints_[eefs_[i].chain[j]]->q[0] + foot_vel[i][j]*dt;
//      q_des[eefs_[i].chain[j]] = joint_positions[i][j];
    }
  }
  if(t>0.1)
  for(int i=0;i<NUM_JOINTS;i++)
    qdd[i] = (qd_des[i] - last_q_des[i])/dt;

  OUTLOG(qdd,"qdd");

  last_q_des = qd_des;
}
