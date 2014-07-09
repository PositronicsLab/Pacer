#include <robot.h>

using namespace Ravelin;

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
Ravelin::Vector3d& Robot::foot_kinematics(const Ravelin::VectorNd& x,const EndEffector& foot,const boost::shared_ptr<Ravelin::Pose3d> frame, const Ravelin::Vector3d& goal, Ravelin::Vector3d& fk, Ravelin::MatrixNd& gk){
  foot_jacobian(x.data(),foot,frame,gk);
  fk = Ravelin::Pose3d::transform_vector(frame,Ravelin::Pose3d::transform_point(
         foot.link->get_pose(),goal));
  fk.pose = frame;
  return fk;
}

Ravelin::MatrixNd& Robot::foot_jacobian(const Ravelin::Origin3d& x,const EndEffector& foot,const boost::shared_ptr<Ravelin::Pose3d> frame, Ravelin::MatrixNd& gk){
  for(int i=0;i<foot.chain.size();i++)
    joints_[foot.chain[i]]->q[0] = x[i];
  abrobot_->update_link_poses();

  gk.resize(3,foot.chain.size());
  boost::shared_ptr<Ravelin::Pose3d> jacobian_frame(
        new Ravelin::Pose3d(Ravelin::Quatd::identity(),
                            Ravelin::Pose3d::transform_point(
                              frame,
                              Ravelin::Vector3d(0,0,0,foot.link->get_pose())).data(),
                            frame));
  abrobot_->calc_jacobian(jacobian_frame,foot.link,workM_);
  for(int j=0;j<3;j++)                                      // x,y,z
    for(int k=0;k<foot.chain.size();k++)                // actuated joints
      gk(j,k) = workM_(j,foot.chain[k]);

  return gk;
}

/// Resolved Rate Motion Control
void Robot::RRMC(const EndEffector& foot,const Ravelin::VectorNd& q,const Ravelin::Vector3d& goal,Ravelin::VectorNd& q_des){
  Ravelin::MatrixNd J;
  Ravelin::VectorNd x(foot.chain.size());
  Ravelin::Vector3d step;

  double alpha = 1, err = 1, last_err = 2;
  for(int k=0;k<foot.chain.size();k++)                // actuated joints
    x[k] = q[foot.chain[k]];

  foot_kinematics(x,foot,base_frame,goal,step,J);
//  OUTLOG(x,"q",logDEBUG1);
  OUTLOG(goal,"goal",logDEBUG1);

  err = step.norm();

  while(err > 1e-3  && err < last_err){
    // update error
    last_err = err;
//    OUTLOG(x,"q",logDEBUG1);
    OUTLOG(step,"xstep",logDEBUG1);
    LA_.solve_fast(J,step);
    Ravelin::VectorNd qstep = step;
    OUTLOG(qstep,"qstep",logDEBUG1);

    // Line Search
    alpha = 1;
    {
      double beta = 0.5;
      Ravelin::Vector3d fk1;
      Ravelin::VectorNd xx = x;
      // distance to goal is greater alpha*step than beta*alpha*step?
      // reduce alpha to alpha*beta
      while (foot_kinematics((x = xx) += (workv_ = qstep)*= alpha      ,foot,base_frame,goal,fk1,workM_).norm() >
             foot_kinematics((x = xx) += (workv_ = qstep)*= alpha*beta ,foot,base_frame,goal,fk1,workM_).norm()){
        alpha = alpha*beta;
      }
      x = xx;
    }

    OUT_LOG(logDEBUG1) << "alpha: " << alpha;

    x += ( (workv_ = qstep)*= alpha );

    // get foot pos
    foot_kinematics(x,foot,base_frame,goal,step,J);

    err = step.norm();

    // if error increases, backstep then return
    if(err > last_err)
      x -= ( (workv_ = qstep)*= alpha );
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

/// Calculate contact-frame jacobians,
/* oriented to the contact normal and 2 orthogonal directions
 * N = [n1 n2 .. nN]
 * D = [S T -S -T] : S = [s1 s2 .. sN]
 * R = [N D];
 */
void Robot::calc_contact_jacobians(Ravelin::MatrixNd& N,Ravelin::MatrixNd& D,Ravelin::MatrixNd& R){
  static Ravelin::VectorNd workv_;
  static Ravelin::MatrixNd workM_;

  N.set_zero(NDOFS,NC);
  D.set_zero(NDOFS,NC*NK);
  R.set_zero(NDOFS,NC*5);
  if(NC==0) return;
  // Contact Jacobian [GLOBAL frame]
  Ravelin::MatrixNd J(3,NDOFS);
  boost::shared_ptr<Ravelin::Pose3d> event_frame(new Ravelin::Pose3d(environment_frame));

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

    // TODO: TEST FOR BAD TANGENTS BEFORE DOIGN THIS
    Ravelin::Vector3d::determine_orthonormal_basis(normal,tan1,tan2);

    // Normal direction
    J.transpose_mult(normal,workv_);
    N.set_column(ii,workv_);

    // 1st tangent
    J.transpose_mult(tan1,workv_);
    D.set_column(ii,workv_);
    workv_.negate();
    D.set_column(NC*2+ii,workv_);

    // 2nd tangent
    J.transpose_mult(tan2,workv_);
    D.set_column(NC+ii,workv_);
    workv_.negate();
    D.set_column(NC*3+ii,workv_);

    ii++;
  }

  R.set_sub_mat(0,0,N);
  R.set_sub_mat(0,NC,D);
}

void Robot::calc_base_jacobian(Ravelin::MatrixNd& R){
  int ndofs = NUM_EEFS*3;

  R.set_zero(6,ndofs);
  Ravelin::MatrixNd J;

  boost::shared_ptr<Ravelin::Pose3d> event_frame(new Ravelin::Pose3d(base_frame));
  for(int ii=0,i=0;ii<NUM_EEFS;i++,ii++){
    while(!eefs_[ii].active) ii++;

    // J: Jacobian, _point@link ^frame
    // calculate J_f^base : [vb,qd] -> [vf]
    event_frame->x = Ravelin::Pose3d::transform_point(base_frame,Ravelin::Vector3d(0,0,0,eefs_[i].link->get_pose()));
    dbrobot_->calc_jacobian(event_frame,eefs_[i].link,J);

    for(int c=0;c<eefs_[i].chain.size();c++)
      for(int r=0;r<6;r++)
        R(r,eefs_[i].chain[c]) = -J(r,eefs_[i].chain[c]);
  }
}

/// Claculate Jacobians oriented to operational space at each foot
/* Goes from Minimal coords [v,q]' -> workspace coords [x1,x2,..,xN]'
 *
 */
void Robot::calc_workspace_jacobian(Ravelin::MatrixNd& Rw, const boost::shared_ptr<Ravelin::Pose3d> workspace){
  Rw.set_zero(NUM_EEFS*3 + 6, NUM_JOINTS + 6);
//  Rw.set_zero(NUM_EEFS*3, NUM_JOINTS + 6);
  Ravelin::MatrixNd J(3,NDOFS);
  boost::shared_ptr<Ravelin::Pose3d> event_frame(new Ravelin::Pose3d(workspace));

  // [x y z alpha beta gamma]_ environment_frame
//  Ravelin::Pose3d::spatial_transform_to_matrix2(base_link_frame,environment_frame,base_stability_offset);
//  Rw.set_sub_mat(NUM_EEFS*3,NUM_JOINTS,base_stability_offset);
  Rw.set_sub_mat(NUM_EEFS*3,NUM_JOINTS,Ravelin::MatrixNd::identity(6));
  for(int i=0;i<NUM_EEFS;i++){
    EndEffector& foot = eefs_[i];

//    [j;b] -> [f;b]
//    [Jj Jb;
//     0  I ]
    // swing foot jacobian
    // [x y z]_ base_frame
    // at center of foot
    event_frame->x = Ravelin::Pose3d::transform_point(workspace,Ravelin::Vector3d(0,0,0,foot.link->get_pose()));

    dbrobot_->calc_jacobian(event_frame,foot.link,workM_);
    workM_.get_sub_mat(0,3,0,NDOFS,J);
    Rw.set_row(3*i,J.row(0));
    Rw.set_row(3*i+1,J.row(1));
    Rw.set_row(3*i+2,J.row(2));

  }

//  Rw.set_sub_mat(0,NUM_JOINTS,Rw.get_sub_mat(0,NUM_EEFS*3,NUM_JOINTS,NUM_JOINTS+6,workM_).negate());
  Rw.set_sub_mat(0,NUM_JOINTS,Ravelin::MatrixNd::zero(NUM_EEFS*3,6));

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
