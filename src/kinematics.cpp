/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/robot.h>

using namespace Ravelin;
using namespace Pacer;

static Ravelin::VectorNd workv_;
static Ravelin::Vector3d workv3_;
static Ravelin::MatrixNd workM_;

Ravelin::MatrixNd& Robot::link_jacobian(const Ravelin::VectorNd& x,const end_effector_t& foot,const boost::shared_ptr<const Ravelin::Pose3d> frame, Ravelin::MatrixNd& gk){
  for(int i=0;i<foot.chain.size();i++){
    std::pair<std::string, int> id_dof = _coord_id_map[foot.chain[i]];
    _id_joint_map[id_dof.first]->q[id_dof.second] = x[i];
  }
  _abrobot->update_link_poses();
  gk.resize(6,foot.chain.size());
  boost::shared_ptr<Ravelin::Pose3d> jacobian_frame(
        new Ravelin::Pose3d(Ravelin::Quatd::identity(),
                            Ravelin::Pose3d::transform_point(
                              frame,
                              Ravelin::Vector3d(0,0,0,foot.link->get_pose())).data(),
                            frame));
  _abrobot->calc_jacobian(jacobian_frame,foot.link,workM_);

  for(int j=0;j<6;j++) {                                     // x,y,z
    for(int k=0;k<foot.chain.size();k++){
      gk(j,k) = workM_(j,foot.chain[k]);
    }
  }

  return gk;
}
    
Ravelin::MatrixNd Robot::calc_jacobian(const Ravelin::VectorNd& q,const std::string& link, Ravelin::Vector3d point){
   Ravelin::MatrixNd J;
   set_model_state(q);
  
   boost::shared_ptr<Ravelin::Pose3d> jacobian_frame(
        new Ravelin::Pose3d(Ravelin::Quatd::identity(),
                            Ravelin::Pose3d::transform_point(
                              Moby::GLOBAL,
                              point).data(),Moby::GLOBAL));
   
  _abrobot->calc_jacobian(jacobian_frame,_id_link_map[link],J);

  return J;
}
/// Working kinematics function [y] = f(x,foot,pt,y,J)
/// evaluated in foot link frame
Ravelin::VectorNd& Robot::link_kinematics(const Ravelin::VectorNd& x,const end_effector_t& foot,const boost::shared_ptr<const Ravelin::Pose3d> frame, const Ravelin::Vector3d& goal, Ravelin::VectorNd& fk, Ravelin::MatrixNd& gk){
  link_jacobian(x,foot,frame,gk);
  gk = gk.get_sub_mat(0,3,0,gk.columns(),workM_);
  fk = Ravelin::Pose3d::transform_vector(frame,Ravelin::Pose3d::transform_point(
         foot.link->get_pose(),goal));
  return fk;
}
/// Resolved Motion Rate Control
void Robot::RMRC(const end_effector_t& foot,const Ravelin::VectorNd& q,const Ravelin::Vector3d& goal,Ravelin::VectorNd& q_des, double TOL){
  Ravelin::MatrixNd J;
  Ravelin::VectorNd x(foot.chain.size());
  Ravelin::VectorNd step(foot.chain.size());

  boost::shared_ptr<const Ravelin::Pose3d> 
    //base_frame(new Ravelin::Pose3d(get_data<Ravelin::Pose3d>("base_link_frame")));
    base_frame(new Ravelin::Pose3d(goal.pose));

  double alpha = 1, err = 1, last_err = 2;
  for(int k=0;k<foot.chain.size();k++)                // actuated joints
    x[k] = q[foot.chain[k]];

  link_kinematics(x,foot,base_frame,goal,step,J);
  
  err = step.norm();
  OUTLOG(goal,"goal",logDEBUG1);

  while(err > TOL){
    // update error
    last_err = err;
//    OUTLOG(x,"q",logDEBUG1);
    OUTLOG(step,"xstep",logDEBUG1);
    OUTLOG(J,"J",logDEBUG1);
    Utility::solve(workM_ = J,step);

    Ravelin::VectorNd qstep = step;
    OUTLOG(qstep,"qstep",logDEBUG1);

    // Line Search
    alpha = 1;
    {
      double beta = 0.5;
      Ravelin::VectorNd fk1;
      // save x
      Ravelin::VectorNd xx = x;
      // distance to goal is greater alpha*step than beta*alpha*step?
      // reduce alpha to alpha*beta
      while (link_kinematics((x = xx) += ((workv_ = qstep)*= alpha     ) ,foot,base_frame,goal,fk1,workM_).norm() >
             link_kinematics((x = xx) += ((workv_ = qstep)*= alpha*beta) ,foot,base_frame,goal,fk1,workM_).norm()){
        alpha = alpha*beta;
      }
      x = xx;
    }

    OUT_LOG(logDEBUG1) << "alpha: " << alpha;

    x += ( (workv_ = qstep)*= alpha );

    // get foot pos
    link_kinematics(x,foot,base_frame,goal,step,J);

    err = step.norm();
    OUT_LOG(logDEBUG1) << "err: " << err;

    // if error increases, backstep then return
    if(err > last_err){
      x -= ( (workv_ = qstep)*= alpha );
      break;
    }
  }
  
  // NOTE: WARNING: Only revolute joints
  if(err < 1e-3)
    for(int k=0;k<foot.chain.size();k++)
      q_des[foot.chain[k]] = x[k];
  else
    for(int k=0;k<foot.chain.size();k++)
      q_des[foot.chain[k]] = q[foot.chain[k]];
}

/// Working kinematics function [y] = f(x,foot,pt,y,J)
/// evaluated in foot link frame
Ravelin::VectorNd& Robot::link_kinematics(const Ravelin::VectorNd& x,const end_effector_t& foot,const boost::shared_ptr<const Ravelin::Pose3d> frame, const Ravelin::VectorNd& goal, Ravelin::VectorNd& fk, Ravelin::MatrixNd& gk){
  /*
  link_jacobian(x,foot,frame,gk);
  const Ravelin::Vector3d upper(goal[0],goal[1],goal[2],frame);
  const Ravelin::Vector3d lower(goal[3],goal[4],goal[5],frame);
  fk = Ravelin::VectorNd(6,Ravelin::Pose3d::transform(
                           frame,
                           Ravelin::SVelocityd(
                             Ravelin::Pose3d::transform_point(
                               foot.link->get_pose(),
                               upper
                               ),
                             lower,
                             frame)).data()
                         );
  return fk;
  */
}


/// 6d IK
void Robot::RMRC(const end_effector_t& foot,const Ravelin::VectorNd& q,const Ravelin::VectorNd& goal,Ravelin::VectorNd& q_des){
  /*
  Ravelin::MatrixNd J;
  Ravelin::VectorNd x(foot.chain.size());
  Ravelin::VectorNd step(foot.chain.size());

  boost::shared_ptr<const Ravelin::Pose3d> 
    base_frame(new Ravelin::Pose3d(get_data<Ravelin::Pose3d>("base_link_frame")));

  double alpha = 1, err = 1, last_err = 2;
  for(int k=0;k<foot.chain.size();k++)                // actuated joints
    x[k] = q[foot.chain[k]];

  link_kinematics(x,foot,base_frame,goal,step,J);

  err = step.norm();
  OUTLOG(goal,"goal",logDEBUG1);

  while(err > 1e-3  && err < last_err){
    // update error
    last_err = err;
    OUTLOG(step,"xstep",logDEBUG1);
    Utility::solve(workM_ = J,step);

    Ravelin::VectorNd qstep = step;
    OUTLOG(qstep,"qstep",logDEBUG1);

    // Line Search
    alpha = 1;
    {
      double beta = 0.5;
      Ravelin::VectorNd fk1;
      // Save cuurent X
      Ravelin::VectorNd xx = x;
      // distance to goal is greater alpha*step than beta*alpha*step?
      // reduce alpha to alpha*beta
      while (link_kinematics((x = xx) += ((workv_ = qstep)*= alpha)      ,foot,base_frame,goal,fk1,workM_).norm() >
             link_kinematics((x = xx) += ((workv_ = qstep)*= alpha*beta) ,foot,base_frame,goal,fk1,workM_).norm()){
        alpha = alpha*beta;
        if(alpha < 1e-4)
          break;
      }
      x = xx;
    }

    OUT_LOG(logDEBUG1) << "alpha: " << alpha;

    x += ( (workv_ = qstep)*= alpha );

    // get foot pos
    link_kinematics(x,foot,base_frame,goal,step,J);

    err = step.norm();
    OUTLOG(err,"err",logDEBUG1);

    // if error increases, backstep then return
    if(err > last_err)
      x -= ( (workv_ = qstep)*= alpha );
  }

  OUTLOG(err,"final_err",logDEBUG1);

  for(int k=0;k<foot.chain.size();k++)
    q_des[foot.chain[k]] = x[k];
    */
}



void Robot::calc_contact_jacobians(const Ravelin::VectorNd& q, std::vector<boost::shared_ptr<const contact_t> > c ,Ravelin::MatrixNd& N,Ravelin::MatrixNd& S,Ravelin::MatrixNd& T){
  static Ravelin::VectorNd workv_;
  static Ravelin::MatrixNd workM_;

  set_model_state(q);

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

      _dbrobot->calc_jacobian(impulse_frame,_id_link_map[c[i]->id],workM_);
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

void Robot::end_effector_inverse_kinematics(
    const std::vector<std::string>& foot_id,
    const std::vector<Ravelin::Vector3d>& foot_pos,
    const std::vector<Ravelin::Vector3d>& foot_vel,
    const std::vector<Ravelin::Vector3d>& foot_acc,
    const Ravelin::VectorNd& q,
    Ravelin::VectorNd& q_des,
    Ravelin::VectorNd& qd_des,
    Ravelin::VectorNd& qdd_des, double TOL){
  
  q_des = q.segment(0,NUM_JOINT_DOFS);
  qd_des.set_zero(NUM_JOINT_DOFS);
  qdd_des.set_zero(NUM_JOINT_DOFS);

  set_model_state(q);

  int NUM_EEFS = foot_id.size();

  for(int i=0;i<NUM_EEFS;i++){
    end_effector_t& foot = *(_id_end_effector_map[foot_id[i]].get());

    // POSITION
    OUTLOG(Ravelin::Pose3d::transform_point(Moby::GLOBAL,Ravelin::Vector3d(foot.link->get_pose())),foot.id + "_x",logDEBUG1);
    OUTLOG(Ravelin::Pose3d::transform_point(Moby::GLOBAL,foot_pos[i]),foot.id + "_x_des",logDEBUG1);
    RMRC(foot,q,foot_pos[i],q_des,TOL);
    //RMRC(foot,q,Ravelin::VectorNd(6,Ravelin::SVector6d(foot_pos[i],Ravelin::Vector3d::zero()).data()),q_des);
    OUTLOG(q_des.select(foot.chain_bool,workv_),foot.id + "_q",logDEBUG1);

    // Calc jacobian for AB at this EEF
    Ravelin::MatrixNd J;
    Ravelin::VectorNd x(foot.chain.size());
    for(int k=0;k<foot.chain.size();k++)                // actuated joints
      x[k] = q[foot.chain[k]];
    link_jacobian(x,foot,Moby::GLOBAL,J);
    J = J.get_sub_mat(0,6,0,J.columns(),workM_);

    Ravelin::VectorNd qd_foot,qdd_foot;
    // VELOCITY & ACCELERATION
    OUTLOG(J,foot.id + "__J", logDEBUG1);
    OUTLOG(foot_vel[i],foot.id + "_xd", logDEBUG1);
    Utility::solve((workM_ = J),(qd_foot = Ravelin::VectorNd(6,Ravelin::SVector6d(foot_vel[i],Ravelin::Vector3d::zero()).data())));
    OUTLOG(qd_foot,foot.id + "_qd", logDEBUG1);

    OUTLOG(foot_acc[i],foot.id + "_xdd", logDEBUG1);
    Utility::solve((workM_ = J),(qdd_foot = Ravelin::VectorNd(6,Ravelin::SVector6d(foot_acc[i],Ravelin::Vector3d::zero()).data())));
    OUTLOG(qdd_foot,foot.id + "_qdd", logDEBUG1);

    for(int j=0;j<foot.chain.size();j++){
      qd_des[foot.chain[j]] = qd_foot[j];
      qdd_des[foot.chain[j]] = qdd_foot[j];
    }
  }
}
