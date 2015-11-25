/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/robot.h>
#include <Pacer/utilities.h>

using namespace Ravelin;
using namespace Pacer;

static Ravelin::VectorNd workv_;
static Ravelin::Vector3d workv3_;
static Ravelin::MatrixNd workM_;



Ravelin::VectorNd& joint_state_to_vec(const std::vector<boost::shared_ptr<Ravelin::Jointd> >& joints, Ravelin::VectorNd& x ){
  for (int i=0, ii=0; i<joints.size(); i++) {
    for (int j=0; j<joints[i]->num_dof(); j++,ii++) {
      x[ii] = joints[i]->q[j];
    }
  }
  return x;
}
std::vector<boost::shared_ptr<Ravelin::Jointd> >& vec_to_joint_state(const Ravelin::VectorNd& x, std::vector<boost::shared_ptr<Ravelin::Jointd> >& joints){
  for (int i=0, ii=0; i<joints.size(); i++) {
    for (int j=0; j<joints[i]->num_dof(); j++,ii++) {
      joints[i]->q[j] = x[ii];
    }
  }
  boost::shared_ptr<Ravelin::RCArticulatedBodyd> rcab = boost::dynamic_pointer_cast<Ravelin::RCArticulatedBodyd>(joints[0]->get_articulated_body());
  rcab->update_link_poses();
  return joints;
}


Ravelin::MatrixNd& Robot::link_jacobian(const Ravelin::VectorNd& x,const end_effector_t& foot,const boost::shared_ptr<const Ravelin::Pose3d> frame, Ravelin::MatrixNd& gk){
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

Ravelin::MatrixNd Robot::calc_jacobian(const Ravelin::VectorNd& q,const std::string& link, Ravelin::Origin3d point){
  Ravelin::MatrixNd J;
  
  _abrobot->set_generalized_coordinates_euler(q);
  
  boost::shared_ptr<Ravelin::Pose3d>
  jacobian_frame(
                 new Ravelin::Pose3d(Ravelin::Quatd::identity(),
                                     Ravelin::Origin3d(Ravelin::Pose3d::transform_point(GLOBAL,Ravelin::Vector3d(point.data(),_id_link_map[link]->get_pose())).data())
                                     ,GLOBAL));
  
  _abrobot->calc_jacobian(_root_link->get_mixed_pose(),jacobian_frame,_id_link_map[link],J);
  
//  OUTLOG(_root_link->get_mixed_pose(),"base_frame",logERROR);
//  OUTLOG(jacobian_frame,"jacobian_frame",logERROR);
  
  return J;
}

/// Working kinematics function [y] = f(x,foot,pt,y,J)
/// evaluated in foot link frame
Ravelin::VectorNd& Robot::dist_to_goal(const Ravelin::VectorNd& x,const end_effector_t& foot_const,const boost::shared_ptr<const Ravelin::Pose3d> frame, const Ravelin::Origin3d& goal, Ravelin::VectorNd& dist){
  end_effector_t& foot = const_cast<end_effector_t&>(foot_const);
  vec_to_joint_state(x,foot.chain_joints);

  dist = Ravelin::Pose3d::transform_vector(frame,Ravelin::Pose3d::transform_point(
                                                                                  foot.link->get_pose(),Vector3d(goal,Pacer::GLOBAL)));
  return dist;
}

Ravelin::VectorNd& Robot::link_kinematics(const Ravelin::VectorNd& x,const end_effector_t& foot_const,const boost::shared_ptr<const Ravelin::Pose3d> frame, const Ravelin::Origin3d& goal, Ravelin::VectorNd& dist, Ravelin::MatrixNd& jacobian){
  dist_to_goal(x,foot_const,frame,goal,dist);
  link_jacobian(x,foot_const,frame,jacobian);
  jacobian = jacobian.get_sub_mat(0,3,0,jacobian.columns(),workM_);
  return dist;
}

/// Resolved Motion Rate Control
void Robot::RMRC(const end_effector_t& foot,const Ravelin::VectorNd& q,const Ravelin::Origin3d& input_goal,Ravelin::VectorNd& q_des, double TOL){
  // NOTE: Current robot configuration must be set
  
  OUT_LOG(logDEBUG1) << "Robot::RMRC() -- 3D";

  Ravelin::Origin3d goal = input_goal;
  
  Ravelin::MatrixNd J;
  
  boost::shared_ptr<Ravelin::Pose3d>
  //base_frame(new Ravelin::Pose3d(get_data<Ravelin::Pose3d>("base_link_frame")));
  base_frame(new Ravelin::Pose3d);
  base_frame->rpose = Pacer::GLOBAL;
  
  int N = 0;
  for (int i=0; i<foot.chain_joints.size(); i++) {
    N += foot.chain_joints[i]->num_dof();
  }
  Ravelin::VectorNd x(N),step;

  joint_state_to_vec(foot.chain_joints,x);

  double alpha = 1, err = 1, last_err = 2;
  
  link_kinematics(x,foot,base_frame,goal,step,J);
  
  err = step.norm();
  OUT_LOG(logDEBUG1) << "err: " << err;
  OUT_LOG(logDEBUG1) << "q: " << x;
  OUTLOG(goal,"goal",logDEBUG1);
  
  while(err > TOL){
    // update error
    last_err = err;
    //    OUTLOG(x,"q",logDEBUG1);
//    OUTLOG(step,"xstep",logDEBUG1);
//    OUTLOG(J,"J",logDEBUG1);
    Utility::solve(workM_ = J,step);
    
    Ravelin::VectorNd qstep = step;
//    OUTLOG(qstep,"qstep",logDEBUG1);
    
    // Line Search
    {
      Ravelin::VectorNd workv1_,workv2_;

      double alpha = 1, beta = 0.75;
      Ravelin::VectorNd dist1, dist2;
      // save x
      Ravelin::VectorNd qstep1,qstep2;
      // distance to goal is greater alpha*step than beta*alpha*step?
      // reduce alpha to alpha*beta
      
      double err1,err2;
      do {
        (qstep1 = qstep) *= alpha;
        (qstep2 = qstep) *= alpha*beta;
        
        err1 = dist_to_goal((workv1_ = x) += qstep1 ,foot,base_frame,goal,dist1).norm();
        
        err2 = dist_to_goal((workv2_ = x) += qstep2 ,foot,base_frame,goal,dist2).norm();

        OUT_LOG(logDEBUG2) << "alpha -> beta : " << alpha << " -> " << beta;
        OUT_LOG(logDEBUG2) << "dist1 -> dist2 : " << dist1 << " -> " << dist2;
        OUT_LOG(logDEBUG2) << "err1 -> err2 : " << err1 << " -> " << err2;
//        OUT_LOG(logDEBUG2) << "x1 -> x2 : " << workv1_ << " -> " << workv2_;
        
        alpha *= beta;
      } while (err1 > err2);
      x = workv1_;
    }
    
    
    x += ( (workv_ = qstep)*= alpha );
    OUT_LOG(logDEBUG1) << "q: " << x;
    
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
  for(int k=0;k<foot.chain.size();k++)
    q_des[foot.chain[k]] = x[k];

}

/// Working kinematics function [y] = f(x,foot,pt,y,J)
/// evaluated in foot link frame
Ravelin::VectorNd& Robot::link_kinematics(const Ravelin::VectorNd& x,const end_effector_t& foot,const boost::shared_ptr<const Ravelin::Pose3d> frame, const Ravelin::VectorNd& goal, Ravelin::VectorNd& fk, Ravelin::MatrixNd& gk){
  // This is bad code, you should not be here
  assert(false);
  
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
}


/// 6d IK
void Robot::RMRC(const end_effector_t& foot,const Ravelin::VectorNd& q,const Ravelin::VectorNd& goal,Ravelin::VectorNd& q_des, double TOL){
  OUT_LOG(logDEBUG1) << "Robot::RMRC() -- 6D";

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
  
  while(err > TOL){
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
    if(err > last_err){
      x -= ( (workv_ = qstep)*= alpha );
      break;
    }
  }
  
  OUTLOG(err,"final_err",logDEBUG1);
  
  for(int k=0;k<foot.chain.size();k++)
    q_des[foot.chain[k]] = x[k];
}

void Robot::calc_contact_jacobians(const Ravelin::VectorNd& q, std::vector<boost::shared_ptr<contact_t> > c ,Ravelin::MatrixNd& N,Ravelin::MatrixNd& S,Ravelin::MatrixNd& T){
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
    impulse_frame(new Ravelin::Pose3d(Ravelin::Quatd::identity(),c[i]->point.data(),GLOBAL));
    
    _abrobot->calc_jacobian(impulse_frame,_id_link_map[c[i]->id],workM_);
    workM_.get_sub_mat(0,3,0,NDOFS,J);
    
    Vector3d
    normal  = c[i]->normal,
    tan1 = c[i]->tangent,
    tan2 = Ravelin::Vector3d::cross(c[i]->normal,c[i]->tangent);
    tan2.normalize();
    //      Ravelin::Vector3d::determine_orthonormal_basis(normal,tan1,tan2);
    
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
    impulse_frame(new Ravelin::Pose3d(Ravelin::Quatd::identity(),c[i]->point.data(),GLOBAL));
    
    _abrobot->calc_jacobian(impulse_frame,_id_link_map[c[i]->id],workM_);
    workM_.get_sub_mat(0,3,0,NDOFS,J);
    
    Vector3d
    normal  = c[i]->normal,
    tan1 = c[i]->tangent,
    tan2 = Ravelin::Vector3d::cross(c[i]->normal,c[i]->tangent);
    tan2.normalize();
    //      Ravelin::Vector3d::determine_orthonormal_basis(normal,tan1,tan2);
    
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
                                            const std::vector<Ravelin::Origin3d>& foot_pos,
                                            const std::vector<Ravelin::Origin3d>& foot_vel,
                                            const std::vector<Ravelin::Origin3d>& foot_acc,
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
    OUTLOG(Ravelin::Pose3d::transform_point(GLOBAL,Ravelin::Vector3d(foot.link->get_pose())),foot.id + "_x",logDEBUG1);
    OUTLOG(foot_pos[i],foot.id + "_x_des",logDEBUG1);
    RMRC(foot,q,foot_pos[i],q_des,TOL);
    
    //    RMRC(foot,q,Ravelin::VectorNd(6,Ravelin::SVector6d(foot_pos[i],Ravelin::Vector3d::zero()).data()),q_des,TOL);
    OUTLOG(q_des.select(foot.chain_bool,workv_),foot.id + "_q",logDEBUG1);
    
    // Calc jacobian for AB at this EEF
    Ravelin::MatrixNd J;
    Ravelin::VectorNd x(foot.chain.size());
    for(int k=0;k<foot.chain.size();k++)                // actuated joints
      x[k] = q[foot.chain[k]];
    link_jacobian(x,foot,GLOBAL,J);
    J = J.get_sub_mat(0,6,0,J.columns(),workM_);
    
    Ravelin::VectorNd qd_foot,qdd_foot;
    // VELOCITY & ACCELERATION
    OUTLOG(J,foot.id + "__J", logDEBUG1);
    OUTLOG(foot_vel[i],foot.id + "_xd", logDEBUG1);
    qd_foot = Ravelin::VectorNd::zero(6);
    qd_foot.segment(0,3) = foot_vel[i];
    Utility::solve((workM_ = J),qd_foot);
    OUTLOG(qd_foot,foot.id + "_qd", logDEBUG1);
    
    OUTLOG(foot_acc[i],foot.id + "_xdd", logDEBUG1);
    qdd_foot = Ravelin::VectorNd::zero(6);
    qdd_foot.segment(0,3) = foot_acc[i];

    Utility::solve((workM_ = J),qdd_foot);
    OUTLOG(qdd_foot,foot.id + "_qdd", logDEBUG1);
    
    for(int j=0;j<foot.chain.size();j++){
      qd_des[foot.chain[j]] = qd_foot[j];
      qdd_des[foot.chain[j]] = qdd_foot[j];
    }
  }
}
