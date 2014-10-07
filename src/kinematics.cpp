#include <robot.h>

using namespace Ravelin;

Ravelin::VectorNd& Robot::foot_kinematics(const Ravelin::VectorNd& x,const EndEffector& foot, Ravelin::VectorNd& fk, Ravelin::MatrixNd& gk){
  for(int i=0;i<foot.chain.size();i++)
    joints_[foot.chain[i]]->q[0] = x[i];
  abrobot_->update_link_poses();

  gk.set_zero(3,foot.chain.size());
  fk = Ravelin::Pose3d::transform_point(base_frame,Ravelin::Vector3d(0,0,0,foot.link->get_pose()));
  boost::shared_ptr<Ravelin::Pose3d> jacobian_frame = boost::shared_ptr<Ravelin::Pose3d>(new Ravelin::Pose3d(*base_frame));
  jacobian_frame->x = Ravelin::Origin3d(fk[0],fk[1],fk[2]);

  abrobot_->calc_jacobian(jacobian_frame,foot.link,workM_);
  for(int j=0;j<3;j++)                                      // x,y,z
    for(int k=0;k<foot.chain.size();k++)                // actuated joints
      gk(j,k) = workM_(j,foot.chain[k]);
  return fk;
}


/// Working kinematics function [y] = f(x,foot,pt,y,J)
/// evaluated in foot link frame
Ravelin::VectorNd& Robot::foot_kinematics(const Ravelin::VectorNd& x,const EndEffector& foot,const boost::shared_ptr<const Ravelin::Pose3d> frame, const Ravelin::Vector3d& goal, Ravelin::VectorNd& fk, Ravelin::MatrixNd& gk){
  foot_jacobian(x,foot,frame,gk);
  fk = Ravelin::Pose3d::transform_vector(frame,Ravelin::Pose3d::transform_point(
         foot.link->get_pose(),goal));
  return fk;
}

Ravelin::MatrixNd& Robot::foot_jacobian(const Ravelin::VectorNd& x,const EndEffector& foot,const boost::shared_ptr<const Ravelin::Pose3d> frame, Ravelin::MatrixNd& gk){
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
extern void solve(Ravelin::MatrixNd& M,Ravelin::VectorNd& bx);

/// Resolved Motion Rate Control
void Robot::RMRC(const EndEffector& foot,const Ravelin::VectorNd& q,const Ravelin::Vector3d& goal,Ravelin::VectorNd& q_des){
  Ravelin::MatrixNd J;
  Ravelin::VectorNd x(foot.chain.size());
  Ravelin::VectorNd step(foot.chain.size());

  double alpha = 1, err = 1, last_err = 2;
  for(int k=0;k<foot.chain.size();k++)                // actuated joints
    x[k] = q[foot.chain[k]];

  foot_kinematics(x,foot,base_frame,goal,step,J);

  err = step.norm();

  while(err > 1e-3  && err < last_err){
    // update error
    last_err = err;
//    OUTLOG(x,"q",logDEBUG1);
    OUTLOG(step,"xstep",logDEBUG1);
      solve(workM_ = J,step);

    Ravelin::VectorNd qstep = step;
    OUTLOG(qstep,"qstep",logDEBUG1);

    // Line Search
    alpha = 1;
    {
      double beta = 0.5;
      Ravelin::VectorNd fk1;
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
 * N = [n1 n2 .. nNC]
 * D = [S T -S -T] : S = [s1 s2 .. sNC]
 * R = [N D];
 */

#ifdef MULTITHREADED
void calc_contact_jacobians(Ravelin::MatrixNd& N,Ravelin::MatrixNd& D,int i,int nc,const EndEffector& foot,boost::shared_ptr<const Ravelin::Pose3d> environment_frame,const boost::shared_ptr<Moby::DynamicBody> dbrobot){
  // Contact Jacobian [GLOBAL frame]
  Ravelin::MatrixNd workM_;

  boost::shared_ptr<Ravelin::Pose3d> event_frame(new Ravelin::Pose3d(environment_frame));
  event_frame->x = foot.point;
  dbrobot->calc_jacobian(event_frame,foot.link,workM_);
  Ravelin::SharedMatrixNd J_linear = workM_.block(0,3,0,workM_.columns());
  N.column(i)      = J_linear.transpose_mult(foot.normal,workv_);
  D.column(i)      = J_linear.transpose_mult(foot.tan1,workv_);
  D.column(nc*2+i) = workv_.negate();
  D.column(nc+i)   = J_linear.transpose_mult(foot.tan2,workv_);
  D.column(nc*3+i) = workv_.negate();
//  copy(S.begin(),S.end(),nS.begin());
//  copy(T.begin(),T.end(),nT.begin());
//  nS.negate();
//  nT.negate();
}

# include <thread>
void Robot::calc_contact_jacobians(Ravelin::MatrixNd& N,Ravelin::MatrixNd& D,Ravelin::MatrixNd& R){
  static Ravelin::VectorNd workv_;
  static Ravelin::MatrixNd workM_;

  N.set_zero(NDOFS,NC);
  D.set_zero(NDOFS,NC*4);
  R.set_zero(NDOFS,NC*5);
  if(NC==0) return;


  std::vector<std::thread * > t;
  for(int i=0,ii=0;i<NUM_EEFS;i++){
    if(!eefs_[i].active)
      continue;
    t.push_back(new std::thread(::calc_contact_jacobians,N,D,ii,NC,eefs_[i],environment_frame,dbrobot_));
    ii++;
  }
  OUTLOG(N,"N",logERROR);
  OUTLOG(D,"D",logERROR);

  R.block(0,N.rows(),0,N.columns()) = N;
  R.block(0,D.rows(),N.columns(),N.columns()+D.columns()) = D;
}
#else
void Robot::calc_contact_jacobians(Ravelin::MatrixNd& N,Ravelin::MatrixNd& D,Ravelin::MatrixNd& R){
  static Ravelin::VectorNd workv_;
  static Ravelin::MatrixNd workM_;

  N.set_zero(NDOFS,NC);
  D.set_zero(NDOFS,NC*4);
  if(NC==0) return;
  // Contact Jacobian [GLOBAL frame]
  Ravelin::MatrixNd J(3,NDOFS);
  for(int i=0,ii=0;i<NUM_EEFS;i++){
    EndEffector& foot = eefs_[i];
    if(!foot.active)
      continue;

    for(int j=0;j<foot.point.size();j++){
      Ravelin::Matrix3d R_foot( eefs_[i].normal[j][0], eefs_[i].normal[j][1], eefs_[i].normal[j][2],
                                  eefs_[i].tan1[j][0],   eefs_[i].tan1[j][1],   eefs_[i].tan1[j][2],
                                  eefs_[i].tan2[j][0],   eefs_[i].tan2[j][1],   eefs_[i].tan2[j][2]);

      boost::shared_ptr<const Ravelin::Pose3d> impulse_frame(new Ravelin::Pose3d(Ravelin::Quatd(R_foot),foot.point[j].data(),Moby::GLOBAL));

      dbrobot_->calc_jacobian(impulse_frame,foot.link,workM_);
      workM_.get_sub_mat(0,3,0,NDOFS,J);

      Vector3d
        normal  = foot.normal[j],
        tan1    = foot.tan1[j],
        tan2    = foot.tan2[j];

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

  }

  R.set_zero(N.rows(),N.columns()+D.columns());

  R.set_sub_mat(0,0,N);
  R.set_sub_mat(0,N.columns(),D);
}
#endif
void Robot::calc_base_jacobian(Ravelin::MatrixNd& R){
  int ndofs = NUM_EEFS*3;

  R.set_zero(6,ndofs);
  Ravelin::MatrixNd J;

  for(int ii=0,i=0;ii<NUM_EEFS;i++,ii++){
    while(!eefs_[ii].active) ii++;

    // J: Jacobian, _point@link ^frame
    // calculate J_f^base : [vb,qd] -> [vf]
    dbrobot_->calc_jacobian(eefs_[i].frame_robot_base,eefs_[i].link,J);

    for(int c=0;c<eefs_[i].chain.size();c++)
      for(int r=0;r<6;r++)
        R(r,eefs_[i].chain[c]) = -J(r,eefs_[i].chain[c]);
  }
}

/// Claculate Jacobians oriented to operational space at each foot
/* Goes from Minimal coords [v,q]' -> workspace coords [x1,x2,..,xN]'
 *
 */
void Robot::calc_workspace_jacobian(Ravelin::MatrixNd& Rw){
  Rw.set_zero(NUM_EEFS*3 + 6, NUM_JOINTS + 6);
  Ravelin::MatrixNd J(3,NDOFS);

  // [x y z alpha beta gamma]_ environment_frame
//  Ravelin::Pose3d::spatial_transform_to_matrix2(base_link_frame,environment_frame,base_stability_offset);
  Rw.set_sub_mat(NUM_EEFS*3,NUM_JOINTS,Ravelin::MatrixNd::identity(6));
  for(int i=0;i<NUM_EEFS;i++){
    EndEffector& foot = eefs_[i];

//    [j;b] -> [f;b]
//    [Jj Jb;
//     0  I ]
    // swing foot jacobian
    // [x y z]_ base_frame
    // at center of foot

    dbrobot_->calc_jacobian(foot.frame_robot_base,foot.link,workM_);
    workM_.get_sub_mat(0,3,0,NDOFS,J);
    Rw.set_row(3*i,J.row(0));
    Rw.set_row(3*i+1,J.row(1));
    Rw.set_row(3*i+2,J.row(2));

  }

  // Remove effects of base mevement on foot velocity from jacobian (foot vel is set realitive to base)
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
