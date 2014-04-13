#include<robot.h>
#include <utilities.h>
extern bool solve_qp(const Ravelin::MatrixNd& Q, const Ravelin::VectorNd& c, const Ravelin::MatrixNd& A, const Ravelin::VectorNd& b, Ravelin::VectorNd& x);

using namespace Ravelin;

void Robot::contact_jacobian_null_stabilizer(const Ravelin::MatrixNd& R,const Ravelin::SVelocityd& vel_des, Ravelin::VectorNd& uff){
  static Ravelin::VectorNd workv_;
  static Ravelin::MatrixNd workM_;

  if(NC == 0) return;

  std::vector<int> active_dofs;
  for(int i=0;i<NUM_EEFS;i++)
    if(eefs_[i].active)
      for(int j=0;j<eefs_[i].chain.size();j++)
        active_dofs.push_back(eefs_[i].chain[j]);

  if(active_dofs.size() == 0) return;

  // Select active rows in Jacobian Matrix
  // R -> Jh
  static Ravelin::MatrixNd J;

  for(int i=0;i<NUM_EEFS;i++){
    static Ravelin::MatrixNd Jf;


  }
  std::sort(active_dofs.begin(),active_dofs.end());
  R.select_rows(active_dofs.begin(),active_dofs.end(),J);
  J.transpose();
  OUTLOG(J,"J",logDEBUG1);

  // Generate Jh homogenous portion of jacobian system
  Ravelin::MatrixNd J_star;
//  Utility::kernal(J,J_star);
  LA_.nullspace(J,J_star);
  OUTLOG(J_star,"J*",logDEBUG1);

  // Trunk Stability gains

  Ravelin::VectorNd Kv(6);

  Kv[0] = 0;
  Kv[1] = 0;
  Kv[2] = 0;
  Kv[3] = 1e4;
  Kv[4] = 1e4;
  Kv[5] = 0;

  Ravelin::SVelocityd vel_base;
  vel.get_sub_vec(NUM_JOINTS,NDOFS, vel_base);

  vel_base -= vel_des;
//  vel_base = (Ravelin::VectorNd::diag_mult(Kv,))

  static Ravelin::VectorNd tY;

  {
    // Minimize ( Kv (qd - qd_des) + J* w )^2
    // min(w) y = w' [J*' J*] w + [qd_err Kv' J*] w
    // apply J* w to joints

    Ravelin::MatrixNd Q,//(J_star.columns(),J_star.columns()),
                      A(0,J_star.columns()),
                      K(active_dofs.size(),active_dofs.size());
    Ravelin::VectorNd c,//(J_star.columns()),
                      b(0),
                      qd_err,
                      w(J_star.columns());
    qd_err.set_zero(active_dofs.size());
    qd_err.set_sub_vec(active_dofs.size()-6,vel_base);
    OUTLOG(qd_err,"qd_err",logDEBUG);

    J_star.transpose_mult(J_star,Q);
    K.transpose_mult(J_star,workM_).transpose_mult(qd_err,c);

    solve_qp(Q,c,A,b,w);

    // tY = J* w
    J_star.mult(w,tY);
    OUTLOG(tY,"qd_correction",logDEBUG);

  }

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


