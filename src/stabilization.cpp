#include<robot.h>
#include <utilities.h>
extern bool solve_qp(const Ravelin::MatrixNd& Q, const Ravelin::VectorNd& c, const Ravelin::MatrixNd& A, const Ravelin::VectorNd& b, Ravelin::VectorNd& x);

using namespace Ravelin;

void Robot::zmp_stabilizer(const Ravelin::MatrixNd& J,const Ravelin::Vector2d& zmp_goal, Ravelin::VectorNd& ufb){
  Ravelin::Matrix2d Kz(1,0,
                       0,1);
  Kz *= 1e-1;

  Ravelin::Vector2d zfb;
  Kz.mult(zmp_goal - zero_moment_point,zfb);

  Ravelin::SVector6d zfb6(zfb[0],zfb[1],0,0,0,0);

#ifdef VISUALIZE_MOBY
  workv3_ = zfb6.get_upper();
  if(workv3_.norm() > 1.0)
    workv3_.normalize();
  visualize_ray(Ravelin::Vector3d(center_of_mass_x[0],center_of_mass_x[1],0)+workv3_,Ravelin::Vector3d(center_of_mass_x[0],center_of_mass_x[1],0),Ravelin::Vector3d(1,0,0.5),sim);
#endif
  J.transpose_mult(zfb6,ufb);
}

void Robot::contact_jacobian_null_stabilizer(const Ravelin::MatrixNd& R,const Ravelin::SVector6d& vel_des, Ravelin::VectorNd& ufb){
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

  Ravelin::VectorNd vel_base(6);
  generalized_qd.get_sub_vec(NUM_JOINTS,NDOFS, vel_base);

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
    qd_err.set_zero(active_dofs.size()+6);
    qd_err.set_sub_vec(active_dofs.size()-6,vel_base);
    OUTLOG(qd_err,"qd_err",logDEBUG);

    J_star.transpose_mult(J_star,Q);
//    K.transpose_mult(J_star,workM_).transpose_mult(qd_err,c);
    J_star.transpose_mult(qd_err,c);
    solve_qp(Q,c,A,b,w);

    // tY = J* w
    J_star.mult(w,tY);
    OUTLOG(tY,"qd_correction",logDEBUG);

  }

  // apply base stabilization forces
  for(int i=0;i<NC*3;i++)
    ufb[active_dofs[i]] += tY[i];
}

void Robot::contact_jacobian_stabilizer(const Ravelin::MatrixNd& R,const std::vector<double>& Kp,const std::vector<double>& Kv,const std::vector<double>& Ki,const std::vector<double>& pos_des,const std::vector<double>& vel_des, Ravelin::VectorNd& js_correct){
  static Ravelin::VectorNd workv_;
  static Ravelin::MatrixNd workM_;

  if(NC == 0) return;
//  workM_ = R;
//  workM_.remove_row(0);
  Ravelin::SharedConstMatrixNd Jb = R.block(NUM_JOINT_DOFS,NDOFS,0,N.columns()*3);
  Ravelin::SharedConstMatrixNd Jq = R.block(0,NUM_JOINT_DOFS,0,N.columns()*3);

  OUTLOG(Jb,"Jb",logDEBUG1);
  OUTLOG(Jq,"Jq",logDEBUG1);

  Ravelin::VectorNd vel_base(6), pos_base(6), base_correct(6);
  generalized_qd.get_sub_vec(NUM_JOINTS,NDOFS, vel_base);
//  pos_base.set_sub_vec(0,center_of_mass_x);
  pos_base.set_sub_vec(3,roll_pitch_yaw);

  OUTLOG(vel_base,"vel_base",logDEBUG1);
  OUTLOG(vel_des,"vel_des",logDEBUG1);
  static Ravelin::VectorNd sum_p_err = Ravelin::VectorNd::zero(6);
  for(int i=0;i<6;i++){
    sum_p_err[i] += (pos_des[i] - pos_base[i]);
    base_correct[i] = (vel_des[i] - vel_base[i])*Kv[i]
                      + (pos_des[i] - pos_base[i])*Kp[i]
                      + sum_p_err[i]*Ki[i];
  }

  OUTLOG(base_correct,"base_correct",logDEBUG);

  Ravelin::VectorNd ws_correct;
  Jb.transpose_mult(base_correct,ws_correct);
  OUTLOG(ws_correct,"ws_correct",logDEBUG);

  // Remove non-compressive elements (cN < 0)
  for(int i=0;i<N.columns();i++)
    if(ws_correct[i] < 0.0)
      ws_correct[i] = 0.0;

//   Remove Tangential Elements (for now)
//  for(int i=N.columns();i<ws_correct.rows();i++)
//      ws_correct[i] = 0.0;

  Jq.mult(ws_correct,js_correct,-1.0,0);
  OUTLOG(js_correct,"js_correct",logDEBUG);
}

void Robot::calc_com(){
  center_of_mass_x.set_zero();
  center_of_mass_x.pose = environment_frame;
  double total_mass=0;
  for(int i=0;i<links_.size();i++){
    double m = links_[i]->get_mass();
    total_mass += m;
    center_of_mass_x += (Ravelin::Pose3d::transform_point(environment_frame,Ravelin::Vector3d(0,0,0,links_[i]->get_inertial_pose())) *= m);
  }
  center_of_mass_x /= total_mass;

  boost::shared_ptr<Ravelin::Pose3d> base_com_w(new Ravelin::Pose3d(environment_frame));
  base_com_w->x = Ravelin::Origin3d(center_of_mass_x);
  Ravelin::SVector6d com_vel = Ravelin::Pose3d::transform(base_com_w, links_[0]->get_velocity());
  center_of_mass_xd = com_vel.get_upper();

  Ravelin::SAcceld com_acc = Ravelin::Pose3d::transform(base_com_w, links_[0]->get_accel());
  center_of_mass_xdd = com_acc.get_linear();
//  center_of_mass_wd = com_acc.get_angular();
//  center_of_mass_w = com_vel.get_angular();

  // ZMP
  // x(k+1) = A x(k) + B u(k)
  // p(k)   = C x(k)

  // From Kajita et al. 2003
  // x(k) = [x(kT),xd(kT),xdd(kT)]'
  // u(k) = u_x(kT)
  // p(k) = p_x(kT)
  // A = [1  T  (T^2)/2;
  //      0  1  T      ;
  //      0  0  1      ]
  // B = [(T^3)/2 ;
  //      (T^2)/2 ;
  //          T   ]
  // C = [1 0 -z/g]

  // e = p - p_ref
  //
  Ravelin::Vector3d C(1,0,-center_of_mass_x[2]/grav,environment_frame);
  zero_moment_point =
      Ravelin::Vector2d(C.dot(Ravelin::Vector3d(center_of_mass_x[0],center_of_mass_xd[0],center_of_mass_xdd[0],environment_frame)),
                        C.dot(Ravelin::Vector3d(center_of_mass_x[1],center_of_mass_xd[1],center_of_mass_xdd[1],environment_frame)));

  center_of_mass_x.pose = center_of_mass_xd.pose = center_of_mass_xdd.pose = environment_frame;

#ifdef VISUALIZE_MOBY
  // ZMP and COM
  Ravelin::Vector3d CoM_2D(center_of_mass_x[0],center_of_mass_x[1],center_of_mass_x[2]-0.10,environment_frame);
  visualize_ray(CoM_2D,center_of_mass_x,Ravelin::Vector3d(0,0,1),sim);
//  visualize_ray(CoM_2D + center_of_mass_xd*0.1,CoM_2D,Ravelin::Vector3d(0.5,0,1),sim);
//  visualize_ray(CoM_2D + center_of_mass_xd*0.1 + center_of_mass_xdd*0.01,CoM_2D + center_of_mass_xd*0.1,Ravelin::Vector3d(1,0,0),sim);
  visualize_ray(CoM_2D+Ravelin::Vector3d(zero_moment_point[0],zero_moment_point[1],0,environment_frame)*0.1,CoM_2D,Ravelin::Vector3d(0,1,0),sim);
#endif

}

