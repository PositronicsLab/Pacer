#include<robot.h>
#include <utilities.h>

using namespace Ravelin;

void Robot::contact_jacobian_null_stabilizer(const Ravelin::MatrixNd& R, const Ravelin::SVector6d& pos_des, const Ravelin::SVector6d& vel_des, Ravelin::VectorNd& uff){
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
  static Ravelin::MatrixNd J;
  for(int i=NUM_JOINTS;i<NDOFS;i++)
    active_dofs.push_back(i);

  std::sort(active_dofs.begin(),active_dofs.end());
  R.select_rows(active_dofs.begin(),active_dofs.end(),J);
  OUTLOG(J,"J",logDEBUG1);

  // Generate Jh homogenous portion of jacobian system
  Ravelin::MatrixNd Jh = MatrixNd::identity(J.rows()),
      J_plus = J;
  LA_.pseudo_invert(J_plus);
  OUTLOG(J_plus,"J+",logDEBUG1);

  Jh -= J.mult(J_plus,workM_);

  OUTLOG(Jh,"Jh",logDEBUG1);

  // Trunk Stability gains

  OUTLOG(roll_pitch_yaw,"Roll, Pitch, Yaw",logDEBUG1);
  Ravelin::VectorNd Kp(6), Kv(6);

Kp[0] = 0;
Kp[1] = 0;
Kp[2] = 1e-1;
Kp[3] = 1e-1;
Kp[4] = 1e-1;
Kp[5] = 0;

Kv[0] = 1e-1;
Kv[1] = 1e-1;
Kv[2] = 1e-1;
Kv[3] = 1e-1;
Kv[4] = 1e-1;
Kv[5] = 1e-1;

  Ravelin::VectorNd pos_base(6), vel_base(6);
  vel.get_sub_vec(NUM_JOINTS,NDOFS, vel_base);
  pos_base.set_sub_vec(3,roll_pitch_yaw);
  pos_base.set_sub_vec(0,gc.get_sub_vec(NUM_JOINTS,NUM_JOINTS+3,workv_));

  vel_base -= vel_des;
  pos_base -= pos_des;

  static Ravelin::VectorNd Y,tY;
  Y.set_zero(active_dofs.size());
  tY.set_zero(active_dofs.size()-6);

  // Y = Kp*perr + Kv*ver
  Ravelin::MatrixNd::diag_mult(Kp,pos_base,Y) +=  Ravelin::MatrixNd::diag_mult(Kv,vel_base,workv_);


  OUTLOG(Y,"g_stabilization",logDEBUG);
  Jh.mult(Y,tY);
  OUTLOG(tY,"stabilization",logDEBUG);
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

  center_of_mass_x.pose = center_of_mass_xd.pose = center_of_mass_xdd.pose = Moby::GLOBAL;

#ifdef VISUALIZE_MOBY
  // ZMP and COM
  Ravelin::Vector3d CoM_2D(center_of_mass_x[0],center_of_mass_x[1],0,Moby::GLOBAL);
  visualize_ray(CoM_2D,center_of_mass_x,Ravelin::Vector3d(0,0,1),sim);
//  visualize_ray(center_of_mass_x + center_of_mass_xd,center_of_mass_x,Ravelin::Vector3d(0.5,0,1),sim);
//  visualize_ray(center_of_mass_x + center_of_mass_xd + center_of_mass_xdd,center_of_mass_x + center_of_mass_xd,Ravelin::Vector3d(1,0,1),sim);
  visualize_ray(zero_moment_point,CoM_2D,Ravelin::Vector3d(0,0,1),sim);
#endif

}


