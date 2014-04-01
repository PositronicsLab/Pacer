#include <robot.h>

using namespace Ravelin;
using namespace Moby;

Ravelin::Vector3d& Robot::foot_kinematics(const Ravelin::VectorNd& x,const EndEffector& foot, Ravelin::Vector3d& fk, Ravelin::MatrixNd& gk){
  for(int i=0;i<foot.chain.size();i++)
    joints_[foot.chain[i]]->q[0] = x[i];
  abrobot_->update_link_poses();

  gk.set_zero(3,foot.chain.size());
  fk = Ravelin::Pose3d::transform_point(base_frame,Ravelin::Vector3d(0,0,0,foot.link->get_pose()));
  boost::shared_ptr<Ravelin::Pose3d> jacobian_frame = boost::shared_ptr<Ravelin::Pose3d>(new Ravelin::Pose3d(*base_frame));
  jacobian_frame->x = fk;

  abrobot_->calc_jacobian(jacobian_frame,foot.link,workM_);
  for(int j=0;j<3;j++)                                      // x,y,z
    for(int k=0;k<foot.chain.size();k++)                // actuated joints
      gk(j,k) = workM_(j,foot.chain[k]);
  return fk;
}


/// Working kinematics function [y] = f(x,foot,pt,y,J)
/// evaluated in foot link frame
Ravelin::Vector3d& Robot::foot_kinematics(const Ravelin::VectorNd& x,const EndEffector& foot, const Ravelin::Vector3d& goal, Ravelin::Vector3d& fk, Ravelin::MatrixNd& gk){
  for(int i=0;i<foot.chain.size();i++)
    joints_[foot.chain[i]]->q[0] = x[i];

  abrobot_->update_link_poses();

  gk.resize(3,foot.chain.size());
  abrobot_->calc_jacobian(foot.link->get_pose(),foot.link,workM_);
  for(int j=0;j<3;j++)                                      // x,y,z
    for(int k=0;k<foot.chain.size();k++)                // actuated joints
      gk(j,k) = workM_(j,foot.chain[k]);

  fk = Ravelin::Pose3d::transform_point(foot.link->get_pose(),goal);
  return fk;
}

Ravelin::MatrixNd& Robot::foot_jacobian(const Ravelin::Origin3d& x,const EndEffector& foot, Ravelin::MatrixNd& gk){
  for(int i=0;i<foot.chain.size();i++)
    joints_[foot.chain[i]]->q[0] = x[i];

  abrobot_->update_link_poses();

  gk.resize(3,foot.chain.size());
  boost::shared_ptr<Ravelin::Pose3d> jacobian_frame = boost::shared_ptr<Ravelin::Pose3d>(new Ravelin::Pose3d(*base_frame));
  jacobian_frame->x = Ravelin::Pose3d::transform_point(base_frame,Ravelin::Vector3d(0,0,0,foot.link->get_pose()));

  abrobot_->calc_jacobian(jacobian_frame,foot.link,workM_);
  for(int j=0;j<3;j++)                                      // x,y,z
    for(int k=0;k<foot.chain.size();k++)                // actuated joints
      gk(j,k) = workM_(j,foot.chain[k]);

  return gk;
}

/// Resolved Rate Motion Control
void Robot::RRMC(const EndEffector& foot,const Ravelin::VectorNd& q,Ravelin::Vector3d& goal,Ravelin::VectorNd& q_des){
  Ravelin::MatrixNd J;
  Ravelin::VectorNd x(foot.chain.size());
  Ravelin::Vector3d step;

  double alpha = 1, err = 1, last_err = 2;
  goal.pose = base_frame;
  for(int k=0;k<foot.chain.size();k++)                // actuated joints
    x[k] = q[foot.chain[k]];

  foot_kinematics(x,foot,goal,step,J);

  err = step.norm();

  while(err > 1e-3  && err < last_err){
    // update error
    last_err = err;

    LA_.solve_fast(J,step);

    // Line Search
    alpha = 1;
    {
      /// TODO: never activates (could be fine)
      double beta = 0.5, rho = 0.5; // (0,1)
      Ravelin::Vector3d fk1;
      Ravelin::VectorNd xx = x;
      // distance to goal is greater with alpha*step than beta*alpha*step?
      // alpha = beta*alpha;
      while (foot_kinematics((x = xx) += alpha*step,foot,goal,fk1,workM_).norm() >
//             foot_kinematics((x = xx) += alpha*beta*step,foot,goal,fk1,workM_).norm())
             (Ravelin::Origin3d(foot_kinematics(xx,foot,goal,fk1,workM_)) - Ravelin::Origin3d(rho*alpha*J.transpose_mult(step,workv3_))).norm())
        alpha = alpha*beta;
      x = xx;
    }

    x += alpha*step;

    // get foot pos
    foot_kinematics(x,foot,goal,step,J);

    err = step.norm();

    // if error increases, backstep then return
    if(err > last_err)
      x -= alpha*step;
  }
  for(int k=0;k<foot.chain.size();k++)
    q_des[foot.chain[k]] = x[k];
}

/* Use this to convert from MATHEMATICA
:%s/List(/   /g
:%s/)))),/)));/g
:%s/Power(z,2))))/Power(z,2));/g
:%s/Rule(th1,/th[0] = /g
:%s/Rule(th2,/th[1] = /g
:%s/Rule(th3,/th[2] = /g
:%s/Power/pow/g
:%s/Sqrt/sqrt/g
:%s/ArcCos/acos/g
:%s/x/X[0]/gI
:%s/y/X[1]/g
:%s/z/X[2]/g
*/
void Robot::calc_contact_jacobians(Ravelin::MatrixNd& N,Ravelin::MatrixNd& ST,Ravelin::MatrixNd& D,Ravelin::MatrixNd& R){
  static Ravelin::VectorNd workv_;
  static Ravelin::MatrixNd workM_;
  unsigned NC = 0;
  for (unsigned i=0; i< NUM_EEFS;i++)
    if(eefs_[i].active)
      NC++;

  N.set_zero(NDOFS,NC);
  ST.set_zero(NDOFS,NC*2);
  D.set_zero(NDOFS,NC*NK);
  R.set_zero(NDOFS,NC*5);
  // Contact Jacobian [GLOBAL frame]
  Ravelin::MatrixNd J(3,NDOFS);
  boost::shared_ptr<Ravelin::Pose3d> event_frame(new Ravelin::Pose3d(Moby::GLOBAL));

  for(int i=0,ii=0;i<NUM_EEFS;i++){
    EndEffector& foot = eefs_[i];
    if(!foot.active)
      continue;

    event_frame->x = foot.point;
    dbrobot_->calc_jacobian(event_frame,foot.link,workM_);
    workM_.get_sub_mat(0,3,0,NDOFS,J);

    Vector3d
      normal  = foot.normal,
      tan1    = foot.tan1,
      tan2    = foot.tan2;

    // Normal direction
    J.transpose_mult(normal,workv_);
    N.set_column(ii,workv_);

    // 1st tangent
    J.transpose_mult(tan1,workv_);
    ST.set_column(ii,workv_);

    D.set_column(ii,workv_);
    workv_.negate();
    D.set_column(NC*2+ii,workv_);

    // 2nd tangent
    J.transpose_mult(tan2,workv_);
    ST.set_column(NC+ii,workv_);
    D.set_column(NC+ii,workv_);
    workv_.negate();
    D.set_column(NC*3+ii,workv_);

    ii++;
  }

  R.set_sub_mat(0,0,N);
  R.set_sub_mat(0,NC,D);
}

void Robot::calc_eef_jacobians(Ravelin::MatrixNd& R){
  static Ravelin::VectorNd workv_;
  static Ravelin::MatrixNd workM_;

  R.set_zero(NDOFS,NUM_EEFS*3);
  // Contact Jacobian [GLOBAL frame]
  Ravelin::MatrixNd J(3,NDOFS);
  boost::shared_ptr<Ravelin::Pose3d> event_frame(new Ravelin::Pose3d(*base_frame));
  for(int i=0;i<NUM_EEFS;i++){
    event_frame->x = Ravelin::Pose3d::transform_point(Moby::GLOBAL,Ravelin::Vector3d(0,0,0,eefs_[i].link->get_pose()));

    dbrobot_->calc_jacobian(event_frame,eefs_[i].link,workM_);

    workM_.get_sub_mat(0,3,0,NDOFS,J);

    Ravelin::Vector3d normal = Ravelin::Vector3d(0,0,1),
                      tan1 = Ravelin::Vector3d(1,0,0),
                      tan2 = Ravelin::Vector3d(0,1,0);

    // Normal direction
    J.transpose_mult(normal,workv_);
    this->J.set_column(i,workv_);
    // 1st tangent
    J.transpose_mult(tan1,workv_);
    this->J.set_column(NUM_EEFS+i,workv_);
    // 2nd tangent
    J.transpose_mult(tan2,workv_);
    this->J.set_column(NUM_EEFS*2+i,workv_);
  }
}
/*
:%s/Sin\[/sin(/g
:%s/Cos\[/cos(/g
:%s/\]/)/g
:%s/th1/th\[0\]/g
:%s/th2/th\[1\]/g
:%s/th3/th\[2\]/g
:%s//th\[2\]/g
 */
