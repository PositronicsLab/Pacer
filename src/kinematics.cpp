/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/robot.h>

using namespace Ravelin;
using namespace Pacer;

Ravelin::VectorNd& Robot::foot_kinematics(const Ravelin::VectorNd& x,const end_effector_s& foot, Ravelin::VectorNd& fk, Ravelin::MatrixNd& gk){
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
Ravelin::VectorNd& Robot::foot_kinematics(const Ravelin::VectorNd& x,const end_effector_s& foot,const boost::shared_ptr<const Ravelin::Pose3d> frame, const Ravelin::Vector3d& goal, Ravelin::VectorNd& fk, Ravelin::MatrixNd& gk){
  foot_jacobian(x,foot,frame,gk);
  gk = gk.get_sub_mat(0,3,0,gk.columns(),workM_);
  fk = Ravelin::Pose3d::transform_vector(frame,Ravelin::Pose3d::transform_point(
         foot.link->get_pose(),goal));
  return fk;
}

/// Working kinematics function [y] = f(x,foot,pt,y,J)
/// evaluated in foot link frame
Ravelin::VectorNd& Robot::foot_kinematics(const Ravelin::VectorNd& x,const end_effector_s& foot,const boost::shared_ptr<const Ravelin::Pose3d> frame, const Ravelin::SVector6d& goal, Ravelin::VectorNd& fk, Ravelin::MatrixNd& gk){
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

Ravelin::MatrixNd& Robot::foot_jacobian(const Ravelin::VectorNd& x,const end_effector_s& foot,const boost::shared_ptr<const Ravelin::Pose3d> frame, Ravelin::MatrixNd& gk){
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
void Robot::RMRC(const end_effector_s& foot,const Ravelin::VectorNd& q,const Ravelin::Vector3d& goal,Ravelin::VectorNd& q_des){
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

void Robot::RMRC(const end_effector_s& foot,const Ravelin::VectorNd& q,const Ravelin::SVector6d& goal,Ravelin::VectorNd& q_des){
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

void Robot::calc_contact_jacobians(std::vector<boost::shared_ptr<const contact_s> > c ,Ravelin::MatrixNd& N,Ravelin::MatrixNd& S,Ravelin::MatrixNd& T){

  static Ravelin::VectorNd workv_;
  static Ravelin::MatrixNd workM_;

  int NC = c.size();
  N.set_zero(NDOFS,NC);
  S.set_zero(NDOFS,NC);
  T.set_zero(NDOFS,NC);
  if(NC==0) return;

  // Contact Jacobian [GLOBAL frame]
  Ravelin::MatrixNd J(3,NDOFS);
  for(int i=0;i<NC;i++){
    boost::shared_ptr<const Ravelin::Pose3d>
          impulse_frame(new Ravelin::Pose3d(Ravelin::Quatd::identity(),c[i]->point.data(),Moby::GLOBAL));

      dbrobot_->calc_jacobian(impulse_frame,_link_id_map[c[i]->link_id],workM_);
      workM_.get_sub_mat(0,3,0,NDOFS,J);

      Vector3d
        normal  = c[i]->normal,
        tan1,
        tan2;

      Ravelin::Vector3d::determine_orthonormal_basis(normal,tan1,tan2);

      // Normal direction
      J.transpose_mult(normal,workv_);
      N.set_column(i,workv_);

      // 1st tangent
      J.transpose_mult(tan1,workv_);
      S.set_column(i,workv_);

      // 2nd tangent
      J.transpose_mult(tan2,workv_);
      T.set_column(i,workv_);
    }
}
