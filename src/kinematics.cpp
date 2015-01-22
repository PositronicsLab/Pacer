/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/robot.h>

using namespace Ravelin;
using namespace Pacer;

Ravelin::VectorNd& Robot::foot_kinematics(const Ravelin::VectorNd& x,const EndEffector& foot, Ravelin::VectorNd& fk, Ravelin::MatrixNd& gk){
  for(int i=0;i<foot.chain.size();i++)
    joints_[foot.chain[i]]->q[0] = x[i];
  abrobot_->update_link_poses();

  gk.set_zero(3,foot.chain.size());
  fk = Ravelin::Pose3d::transform_point(base_frame,Ravelin::Vector3d(0,0,0,foot.link->get_pose()));
  boost::shared_ptr<Ravelin::Pose3d>
      jacobian_frame = boost::shared_ptr<Ravelin::Pose3d>(new Ravelin::Pose3d(*base_frame));
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
  gk = gk.get_sub_mat(0,3,0,gk.columns(),workM_);
  fk = Ravelin::Pose3d::transform_vector(frame,Ravelin::Pose3d::transform_point(
         foot.link->get_pose(),goal));
  return fk;
}

/// Working kinematics function [y] = f(x,foot,pt,y,J)
/// evaluated in foot link frame
Ravelin::VectorNd& Robot::foot_kinematics(const Ravelin::VectorNd& x,const EndEffector& foot,const boost::shared_ptr<const Ravelin::Pose3d> frame, const Ravelin::SVector6d& goal, Ravelin::VectorNd& fk, Ravelin::MatrixNd& gk){
  foot_jacobian(x,foot,frame,gk);
  fk = Ravelin::VectorNd(6,Ravelin::Pose3d::transform(
                           frame,
                           Ravelin::SVelocityd(
                             Ravelin::Pose3d::transform_point(
                               foot.link->get_pose(),
                               goal.get_upper()
                               ),
                             goal.get_lower(),
                             base_frame)).data()
                         );
  return fk;
}

Ravelin::MatrixNd& Robot::foot_jacobian(const Ravelin::VectorNd& x,const EndEffector& foot,const boost::shared_ptr<const Ravelin::Pose3d> frame, Ravelin::MatrixNd& gk){
  for(int i=0;i<foot.chain.size();i++)
    joints_[foot.chain[i]]->q[0] = x[i];
  abrobot_->update_link_poses();

  gk.resize(6,foot.chain.size());
  boost::shared_ptr<Ravelin::Pose3d> jacobian_frame(
        new Ravelin::Pose3d(Ravelin::Quatd::identity(),
                            Ravelin::Pose3d::transform_point(
                              frame,
                              Ravelin::Vector3d(0,0,0,foot.link->get_pose())).data(),
                            frame));
  abrobot_->calc_jacobian(jacobian_frame,foot.link,workM_);
//  OUTLOG(workM_,foot.id + "__J_full", logDEBUG1);

  for(int j=0;j<6;j++)                                      // x,y,z
    for(int k=0;k<foot.chain.size();k++)                // actuated joints
      gk(j,k) = workM_(j,joints_[foot.chain[k]]->get_coord_index());

  return gk;
}

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
  OUTLOG(goal,"goal",logDEBUG1);

  while(err > 1e-3  && err < last_err){
    // update error
    last_err = err;
//    OUTLOG(x,"q",logDEBUG1);
    OUTLOG(step,"xstep",logDEBUG1);
      Utility::solve(workM_ = J,step);

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

void Robot::RMRC(const EndEffector& foot,const Ravelin::VectorNd& q,const Ravelin::SVector6d& goal,Ravelin::VectorNd& q_des){
  Ravelin::MatrixNd J;
  Ravelin::VectorNd x(foot.chain.size());
  Ravelin::VectorNd step(foot.chain.size());

  double alpha = 1, err = 1, last_err = 2;
  for(int k=0;k<foot.chain.size();k++)                // actuated joints
    x[k] = q[foot.chain[k]];

  foot_kinematics(x,foot,base_frame,goal,step,J);

  err = step.norm();
  OUTLOG(goal,"goal",logDEBUG1);

  while(err > 1e-3  && err < last_err){
    // update error
    last_err = err;
//    OUTLOG(x,"q",logDEBUG1);
    OUTLOG(step,"xstep",logDEBUG1);
      Utility::solve(workM_ = J,step);

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
    OUTLOG(err,"err",logDEBUG1);

    // if error increases, backstep then return
    if(err > last_err)
      x -= ( (workv_ = qstep)*= alpha );
  }

  OUTLOG(err,"final_err",logDEBUG1);

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
  OUTLOG(N,"N",logDEBUG1);
  OUTLOG(D,"D",logDEBUG1);

  R.block(0,N.rows(),0,N.columns()) = N;
  R.block(0,D.rows(),N.columns(),N.columns()+D.columns()) = D;
}
#else
void Robot::calc_contact_jacobians(Ravelin::MatrixNd& N,Ravelin::MatrixNd& D,Ravelin::MatrixNd& R){

  static Ravelin::VectorNd workv_;
  static Ravelin::MatrixNd workM_;

  int NC = 0;
  for(int i=0;i<NUM_EEFS;i++)
    if(eefs_[i].active)
      NC += eefs_[i].point.size();

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
      boost::shared_ptr<const Ravelin::Pose3d>
          impulse_frame(new Ravelin::Pose3d(Ravelin::Quatd::identity(),foot.point[j].data(),Moby::GLOBAL));

      dbrobot_->calc_jacobian(impulse_frame,foot.link,workM_);
      workM_.get_sub_mat(0,3,0,NDOFS,J);

      Vector3d
        normal  = foot.normal[j],
        tan1    = foot.tan1[j],
        tan2    = foot.tan2[j];

      // TODO: TEST FOR BAD TANGENTS BEFORE DOIGN THIS
      if(tan1.norm() < 1.0 - Moby::NEAR_ZERO)
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
