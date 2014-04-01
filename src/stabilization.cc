#include<robot.h>
#include <utilities.h>

using namespace Ravelin;

void Robot::contact_jacobian_null_stabilizer(const Ravelin::MatrixNd& R, Ravelin::VectorNd& uff){
  static Ravelin::VectorNd workv_;
  static Ravelin::MatrixNd workM_;

  if(NC == 0) return;

  std::vector<int> active_dofs;
  for(int i=0;i<NUM_EEFS;i++)
    if(eefs_[i].active)
      for(int j=0;j<eefs_[i].chain.size();j++)
        active_dofs.push_back(eefs_[i].chain[j]);

  if(active_dofs.size() == 0) return;
  OUTLOG(R,"R",logDEBUG1);
  // Select active rows in Jacobian Matrix
  // R -> Jh
  static Ravelin::MatrixNd Jh;
  for(int i=NUM_JOINTS;i<NDOFS;i++)
    active_dofs.push_back(i);

  std::sort(active_dofs.begin(),active_dofs.end());
  R.select_rows(active_dofs.begin(),active_dofs.end(),Jh);

//  Jh.transpose();
  OUTLOG(Jh,"Jh",logDEBUG1);

  // Generate Jh Nullspace
  Ravelin::MatrixNd NULL_Jh = MatrixNd::identity(Jh.columns()),
      Jh_plus;
//  Jh.mult_transpose(Jh,Jh_plus);
//  OUTLOG(Jh_plus,"(Jh Jh')");
  LA_.pseudo_invert(Jh);
  Jh_plus = Jh;
  OUTLOG(Jh,"Jh' (Jh Jh')^-1 == Jh+",logDEBUG1);

  LA_.nullspace(Jh,NULL_Jh);

  OUTLOG(NULL_Jh,"null(Jh)",logDEBUG1);
  // Use NULL(Jh) to stabilize trunk w/o altering gait

  // Trunk Stability gains
  Ravelin::Vector3d& rpy = roll_pitch_yaw;

  OUTLOG(rpy,"Roll, Pitch, Yaw",logDEBUG1);
  double Xp = 0, Xv = 0,
         Yp = 0, Yv = 0,
         Zp = 0, Zv = 0,
         Rp = 0, Rv = 1e-1,
         Pp = 0, Pv = 1e-1;

  static Ravelin::VectorNd Y,tY;
  Y.set_zero(NC*3+6);
  tY.set_zero(NC*3);

  Y[NC*3+2] = -(0)*Zp + -(vel[NUM_JOINTS+2])*Zv;
  Y[NC*3+3] = -rpy[0]*Rp + -vel[NUM_JOINTS+3]*Rv;
  Y[NC*3+4] = -rpy[1]*Pp + -vel[NUM_JOINTS+4]*Pv;
  Y.negate();
  OUTLOG(Y,"g_stabilization",logDEBUG1);
//  NULL_Jh.transpose_mult(Y,tY);
  Jh_plus.mult(Y,tY);
  OUTLOG(tY,"stabilization",logDEBUG1);
  // apply base stabilization forces
  for(int i=0;i<NC*3;i++)
    uff[active_dofs[i]] += tY[i];


}

void Robot::calc_com(){
  center_of_mass_x.set_zero();
  double total_mass=0;
  for(int i=0;i<links_.size();i++){
    double m = links_[i]->get_mass();
    total_mass += m;
    center_of_mass_x += (Ravelin::Pose3d::transform_point(Moby::GLOBAL,Ravelin::Vector3d(0,0,0,links_[i]->get_inertial_pose())) *= m);
  }
  center_of_mass_x /= total_mass;

  boost::shared_ptr<Ravelin::Pose3d> base_com_w(new Ravelin::Pose3d(Moby::GLOBAL));
  base_com_w->x = Ravelin::Origin3d(center_of_mass_x);
  Ravelin::SVector6d com_vel = Ravelin::Pose3d::transform(base_com_w, links_[0]->get_velocity());
  center_of_mass_xd = com_vel.get_upper();

  Ravelin::SAcceld com_acc = Ravelin::Pose3d::transform(base_com_w, links_[0]->get_accel());
  center_of_mass_xdd = com_acc.get_linear();

  // ZMP
  Ravelin::Vector3d C(1,0,-center_of_mass_x[2]/grav,Moby::GLOBAL);
  zero_moment_point =
      Ravelin::Vector3d(C.dot(Ravelin::Vector3d(center_of_mass_x[0],center_of_mass_xd[0],center_of_mass_xdd[0],Moby::GLOBAL)),
                        C.dot(Ravelin::Vector3d(center_of_mass_x[1],center_of_mass_xd[1],center_of_mass_xdd[1],Moby::GLOBAL)),
                        0);


#ifdef VISUALIZE_MOBY
  // ZMP and COM
  Ravelin::Vector3d CoM_2D(center_of_mass_x[0],center_of_mass_x[1],0,Moby::GLOBAL);
  visualize_ray(CoM_2D,center_of_mass_x,Ravelin::Vector3d(0,0,1),sim);
  visualize_ray(zero_moment_point,CoM_2D,Ravelin::Vector3d(0,0,1),sim);
#endif

}


