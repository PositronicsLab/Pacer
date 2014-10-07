#include <quadruped.h>
#include <utilities.h>

std::vector<Ravelin::Vector3d>& Quadruped::foot_oscilator(
    const std::vector<Ravelin::Vector3d>& x0,
    const std::vector<Ravelin::Vector3d>& x,
    const Ravelin::MatrixNd& C,
    double Ls,
    const Ravelin::VectorNd& Hs,
    const std::vector<double>& Df,
    double Vf,
    double bp,
    std::vector<Ravelin::Vector3d>& xd){

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
    double ws  = M_PI * (Vf/Ls) * ((Df[i]*Sp1)/(1.0-Df[i]) + Sp2);

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
    if(center_of_contact.active){
      OUTLOG(center_of_contact.normal[0],"normal",logERROR);
      OUTLOG(center_of_contact.point[0],"point",logERROR);
      OUTLOG(Ravelin::Pose3d::transform_point(environment_frame,x[i]),"foot_pos",logERROR);
      double dist_plane = Utility::distance_from_plane(center_of_contact.normal[0],
                                              center_of_contact.point[0],Ravelin::Pose3d::transform_point(environment_frame,x[i]));
      std::cout << eefs_[i].id << " " << dist_plane << std::endl;
      double Sf1 = 1.0/(exp(-bf*(dist_plane)) + 1.0);
      double Sf2 = 1.0/(exp( bf*(dist_plane)) + 1.0);
  //    double Sf1 = 1.0/(exp(-bf*(xbar[2] - ground_plane(x[i]))) + 1.0);
  //    double Sf2 = 1.0/(exp( bf*(xbar[2] - ground_plane(x[i]))) + 1.0);
      xd[i] = (xd[i])*Sf1 - Ravelin::Vector3d(Vf,0,0,base_frame)*Sf2;
    }
  }

  return xd;
}

void Quadruped::cpg_trot(
    const Ravelin::SVector6d& command,
    const std::vector<double>& touchdown,
    const std::vector<double>& duty_factor,
    double gait_duration,
    double step_height,
    // MODEL
    const std::vector<Ravelin::Vector3d>& foot_origin,
    double t,
    // OUT
    std::vector<Ravelin::Vector3d>& foot_pos,
    std::vector<Ravelin::Vector3d>& foot_vel,
    std::vector<Ravelin::Vector3d>& foot_acc){
  static double last_t = 0;
  static std::vector<Ravelin::Vector3d> last_foot_vel(NUM_EEFS);
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

  // Additional parameters for CPG
  double bp = 1000;

  for(int i=0;i<NUM_EEFS;i++){
    // set height of gait (each foot)
    Hs[i] = step_height;

    foot_pos[i] = Ravelin::Pose3d::transform_point(base_frame,Ravelin::Vector3d(0,0,0,eefs_[i].link->get_pose())) ;
    // SET EEF ORIGINS TO the ground below that EEF SHOULDER
    last_foot_vel[i].pose = foot_vel[i].pose = base_frame;
  }

  // retrieve oscilator value
  foot_oscilator(foot_origin,foot_pos,C,command[0]/gait_duration,Hs,duty_factor,command[0],bp,foot_vel);

  double dt = last_t - t;

  for(int i=0;i<NUM_EEFS;i++){
    if(foot_acc[i][2] > 0.0)
      eefs_[i].active = false;
    foot_pos[i] = Ravelin::Pose3d::transform_point(base_frame,Ravelin::Vector3d(0,0,0,eefs_[i].link->get_pose())) + foot_vel[i]*dt;
    foot_acc[i] = (foot_vel[i] - last_foot_vel[i])/dt;
  }


  last_foot_vel = foot_vel;
  last_t = t;
}
