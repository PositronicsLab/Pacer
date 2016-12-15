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


Ravelin::MatrixNd Pacer::Robot::calc_link_jacobian(const Ravelin::VectorNd& q, const std::string& link){
  return calc_jacobian(q,link,Ravelin::Origin3d(0,0,0));
}


Ravelin::VectorNd& joint_state_to_vec(const std::vector<boost::shared_ptr<Ravelin::Jointd> >& joints, Ravelin::VectorNd& x ){
  for (int i=0, ii=0; i<joints.size(); i++) {
    for (int j=0; j<joints[i]->num_dof(); j++,ii++) {
      x[ii] = joints[i]->q[j];
    }
  }
  return x;
}

std::vector< boost::shared_ptr<Ravelin::Jointd> >& vec_to_joint_state(const Ravelin::VectorNd& x, std::vector<boost::shared_ptr<Ravelin::Jointd> >& joints){
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
  Ravelin::Vector3d foot_origin_vec = Ravelin::Pose3d::transform_point(frame,Ravelin::Vector3d(0,0,0,foot.link->get_pose()));
  double * foot_origin = foot_origin_vec.data();
//  std::cerr << foot.id << " origin: " << foot_origin_vec << std::endl;
  boost::shared_ptr<Ravelin::Pose3d> jacobian_frame
  (new Ravelin::Pose3d
   (Ravelin::Quatd::identity(),
    foot_origin,
    frame
   )
  );
  
  _abrobot->calc_jacobian(_abrobot->get_gc_pose(),jacobian_frame,foot.link,workM_);
  
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
  
  _abrobot->calc_jacobian(_abrobot->get_gc_pose(),jacobian_frame,_id_link_map[link],J);
  
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
        OUT_LOG(logDEBUG2) << "x1 -> x2 : " << workv1_ << " -> " << workv2_;
        
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

//namespace Pacer{
//  class UntestedException : public std::exception
//  {
//  public:
//    virtual char const * what() const _NOEXCEPT { return "This is untested code, if you still want to use it, catch 'Pacer::UntestedException'."; }
//  };
//}

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
    
    _abrobot->calc_jacobian(_abrobot->get_gc_pose(),impulse_frame,_id_link_map[c[i]->id],workM_);
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
  
  int NUM_EEFS = foot_id.size();
  
  set_model_state(q);
  for(int i=0;i<NUM_EEFS;i++){

    end_effector_t& foot = *(_id_end_effector_map[foot_id[i]].get());
    
    // POSITION
    OUTLOG(Ravelin::Pose3d::transform_point(GLOBAL,Ravelin::Vector3d(foot.link->get_pose())),foot.id + "_x",logDEBUG1);
    OUTLOG(foot_pos[i],foot.id + "_x_des",logDEBUG1);
    RMRC(foot,q,foot_pos[i],q_des,TOL);
    
    //    RMRC(foot,q,Ravelin::VectorNd(6,Ravelin::SVector6d(foot_pos[i],Ravelin::Vector3d::zero()).data()),q_des,TOL);
    OUTLOG(q_des.select(foot.chain_bool,workv_),foot.id + "_q",logDEBUG1);
  }
  
  set_model_state(q);
  for(int i=0;i<NUM_EEFS;i++){
    end_effector_t& foot = *(_id_end_effector_map[foot_id[i]].get());

    // Calc jacobian for AB at this EEF
    Ravelin::MatrixNd J;
    Ravelin::VectorNd x(foot.chain.size());
    for(int k=0;k<foot.chain.size();k++)                // actuated joints
      x[k] = q[foot.chain[k]];

    link_jacobian(x,foot,GLOBAL,J);

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


///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// GET/SET STATE DATA ////////////////////////

//////////////////////////////////////////////////////
/// ------------ GET/SET JOINT value  ------------ ///

double Pacer::Robot::get_joint_value(const std::string& id, unit_e u, int dof)
{
  OUT_LOG(logDEBUG) << "Get: "<< id << "_" << unit_enum_string(u) << "["<< dof << "] --> " << _state[u][id][dof];
  return _state[u][id][dof];
}

Ravelin::VectorNd Pacer::Robot::get_joint_value(const std::string& id, unit_e u)
{
  OUT_LOG(logDEBUG) << "Get: "<< id << "_" << unit_enum_string(u) << " --> " << _state[u][id];
  return _state[u][id];
}

void Pacer::Robot::get_joint_value(const std::string& id, unit_e u, Ravelin::VectorNd& dof_val)
{
  dof_val = _state[u][id];
#ifdef LOG_TO_FILE
  OUT_LOG(logDEBUG) << "Get: "<< id << "_" << unit_enum_string(u) << " --> " << dof_val;
#endif
}

void Pacer::Robot::get_joint_value(const std::string& id, unit_e u,std::vector<double>& dof_val)
{
  Ravelin::VectorNd& dof = _state[u][id];
  dof_val.resize(dof.rows());
  for(int i=0;i<dof.rows();i++)
    dof_val[i] = dof[i];
#ifdef LOG_TO_FILE
  OUT_LOG(logDEBUG) << "Get: "<< id << "_" << unit_enum_string(u) << " --> " << dof_val;
#endif
}

void Pacer::Robot::set_joint_value(const std::string& id, unit_e u, int dof, double val)
{
  OUT_LOG(logDEBUG) << "Set: "<< id << "_" << unit_enum_string(u) << "["<< dof <<"] <-- " << val;
  
  check_phase_internal(u);
#ifdef USE_THREADS
  pthread_mutex_lock(&_state_mutex);
#endif
  _state[u][id][dof] = val;
#ifdef USE_THREADS
  pthread_mutex_unlock(&_state_mutex);
#endif
}

void Pacer::Robot::set_joint_value(const std::string& id, unit_e u, const Ravelin::VectorNd& dof_val)
{
#ifdef LOG_TO_FILE
  OUT_LOG(logDEBUG) << "Set: "<< id << "_" << unit_enum_string(u) << " <-- " << dof_val;
#endif
  check_phase_internal(u);
#ifdef USE_THREADS
  pthread_mutex_lock(&_state_mutex);
#endif
  Ravelin::VectorNd& dof = _state[u][id];
  if(dof.rows() != dof_val.rows())
    throw std::runtime_error("Missized dofs in joint "+id+": internal="+boost::icl::to_string<double>::apply(dof.rows())+" , provided="+boost::icl::to_string<double>::apply(dof_val.rows()));
  dof = dof_val;
#ifdef USE_THREADS
  pthread_mutex_unlock(&_state_mutex);
#endif
}

void Pacer::Robot::set_joint_value(const std::string& id, unit_e u, const std::vector<double>& dof_val)
{
#ifdef LOG_TO_FILE
  OUT_LOG(logDEBUG) << "Set: "<< id << "_" << unit_enum_string(u) << " <-- " << dof_val;
#endif
  
  check_phase_internal(u);
#ifdef USE_THREADS
  pthread_mutex_lock(&_state_mutex);
#endif
  Ravelin::VectorNd& dof = _state[u][id];
  if(dof.rows() != dof_val.size())
    throw std::runtime_error("Missized dofs in joint "+id+": internal="+boost::icl::to_string<double>::apply(dof.rows())+" , provided="+boost::icl::to_string<double>::apply(dof_val.size()));
  for(int i=0;i<dof.rows();i++)
    dof[i] = dof_val[i];
#ifdef USE_THREADS
  pthread_mutex_unlock(&_state_mutex);
#endif
}

void Pacer::Robot::get_joint_value(Pacer::Robot::unit_e u, std::map<std::string,std::vector<double> >& id_dof_val_map){
  std::map<std::string,Ravelin::VectorNd>::iterator it;
  for(it=_state[u].begin();it!=_state[u].end();it++){
    const Ravelin::VectorNd& dof_val_internal = (*it).second;
    std::vector<double>& dof_val = id_dof_val_map[(*it).first];
    dof_val.resize(dof_val_internal.size());
    for(int j=0;j<(*it).second.rows();j++){
      dof_val[j] = dof_val_internal[j];
    }
#ifdef LOG_TO_FILE
    OUT_LOG(logDEBUG) << "Get: "<< (*it).first << "_" << unit_enum_string(u) << " --> " << dof_val;
#endif
  }
}

void Pacer::Robot::get_joint_value(Pacer::Robot::unit_e u, std::map<std::string,Ravelin::VectorNd >& id_dof_val_map){
  const std::vector<std::string>& keys = _joint_ids;
  for(int i=0;i<keys.size();i++){
    const Ravelin::VectorNd& dof_val_internal = _state[u][keys[i]];
    id_dof_val_map[keys[i]] = dof_val_internal;
#ifdef LOG_TO_FILE
    OUT_LOG(logDEBUG) << "Get: "<< keys[i] << "_" << unit_enum_string(u) << " --> " << dof_val_internal;
#endif
  }
}

void Pacer::Robot::set_joint_value(Pacer::Robot::unit_e u,const std::map<std::string,std::vector<double> >& id_dof_val_map){
#ifdef USE_THREADS
  pthread_mutex_lock(&_state_mutex);
#endif
  std::map<std::string,std::vector<double> >::const_iterator it;
  
  for(it=id_dof_val_map.begin();it!=id_dof_val_map.end();it++){
    Ravelin::VectorNd& dof_val_internal = _state[u][(*it).first];
    const std::vector<double>& dof_val = (*it).second;
#ifdef LOG_TO_FILE
    OUT_LOG(logDEBUG) << "Set: "<< (*it).first << "_" << unit_enum_string(u) << " <-- " << dof_val;
#endif
    if(dof_val_internal.rows() != dof_val.size()){
      std::cerr << "Missized dofs in joint "+(*it).first+": internal="+boost::icl::to_string<double>::apply(dof_val_internal.rows())+" , provided="+boost::icl::to_string<double>::apply(dof_val.size()) << std::endl;
      throw std::runtime_error("Missized dofs in joint "+(*it).first+": internal="+boost::icl::to_string<double>::apply(dof_val_internal.rows())+" , provided="+boost::icl::to_string<double>::apply(dof_val.size()));
    }
    for(int j=0;j<(*it).second.size();j++){
      dof_val_internal[j] = dof_val[j];
    }
  }
#ifdef USE_THREADS
  pthread_mutex_unlock(&_state_mutex);
#endif
}

void Pacer::Robot::set_joint_value(Pacer::Robot::unit_e u,const std::map<std::string,Ravelin::VectorNd >& id_dof_val_map){
#ifdef USE_THREADS
  pthread_mutex_lock(&_state_mutex);
#endif
  std::map<std::string,Ravelin::VectorNd >::const_iterator it;
  for(it=id_dof_val_map.begin();it!=id_dof_val_map.end();it++){
    Ravelin::VectorNd& dof_val_internal = _state[u][(*it).first];
    OUT_LOG(logDEBUG) << "Set: "<< (*it).first << "_" << unit_enum_string(u) << " <-- " << (*it).second;
    if(dof_val_internal.rows() != (*it).second.rows())
      throw std::runtime_error("Missized dofs in joint "+(*it).first+": internal="+boost::icl::to_string<double>::apply(dof_val_internal.rows())+" , provided="+boost::icl::to_string<double>::apply((*it).second.rows()));
    dof_val_internal = (*it).second;
  }
#ifdef USE_THREADS
  pthread_mutex_unlock(&_state_mutex);
#endif
}

/// ------------ GET/SET JOINT GENERALIZED value  ------------ ///

void Pacer::Robot::set_joint_generalized_value(Pacer::Robot::unit_e u, const Ravelin::VectorNd& generalized_vec)
{
  check_phase_internal(u);
  if(generalized_vec.rows() != NUM_JOINT_DOFS)
    throw std::runtime_error("Missized generalized vector: internal="+boost::icl::to_string<double>::apply(NUM_JOINT_DOFS)+" , provided="+boost::icl::to_string<double>::apply(generalized_vec.rows()));
  
#ifdef USE_THREADS
  pthread_mutex_lock(&_state_mutex);
#endif
  
  // TODO: make this more efficient ITERATORS dont work
  //      std::map<std::string,Ravelin::VectorNd>::iterator it;
  const std::vector<std::string>& keys = _joint_ids;
  
  for(int i=0;i<keys.size();i++){
    const std::vector<int>& dof = _id_dof_coord_map[keys[i]];
    Ravelin::VectorNd& dof_val = _state[u][keys[i]];
    if(dof.size() != dof_val.rows())
      throw std::runtime_error("Missized dofs in joint "+keys[i]+": internal="+boost::icl::to_string<double>::apply(dof.size())+" , provided="+boost::icl::to_string<double>::apply(dof_val.rows()));
    for(int j=0;j<dof.size();j++){
      dof_val[j] = generalized_vec[dof[j]];
    }
  }
#ifdef USE_THREADS
  pthread_mutex_unlock(&_state_mutex);
#endif
  OUT_LOG(logDEBUG) << "Set: joint_generalized_" << unit_enum_string(u) << " <-- " << generalized_vec;
}

void Pacer::Robot::get_joint_generalized_value(Pacer::Robot::unit_e u, Ravelin::VectorNd& generalized_vec){
  generalized_vec.set_zero(NUM_JOINT_DOFS);
  std::map<std::string,Ravelin::VectorNd>::iterator it;
  for(it=_state[u].begin();it!=_state[u].end();it++){
    const std::vector<int>& dof = _id_dof_coord_map[(*it).first];
    const Ravelin::VectorNd& dof_val = (*it).second;
    for(int j=0;j<(*it).second.size();j++){
      generalized_vec[dof[j]] = dof_val[j];
    }
  }
  OUT_LOG(logDEBUG) << "Get: joint_generalized_" << unit_enum_string(u) << " --> " << generalized_vec;
}

Ravelin::VectorNd Pacer::Robot::get_joint_generalized_value(Pacer::Robot::unit_e u){
  Ravelin::VectorNd generalized_vec;
  get_joint_generalized_value(u,generalized_vec);
  return generalized_vec;
}

/// With Base
void Pacer::Robot::set_generalized_value(Pacer::Robot::unit_e u,const Ravelin::VectorNd& generalized_vec){
  check_phase_internal(u);
  if (floating_base())
  switch(u){
    case(position_goal):
    case(position):
      set_base_value(u,generalized_vec.segment(NUM_JOINT_DOFS,NUM_JOINT_DOFS+NEULER));
      break;
    default:
      set_base_value(u,generalized_vec.segment(NUM_JOINT_DOFS,NUM_JOINT_DOFS+NSPATIAL));
      break;
  }
  set_joint_generalized_value(u,generalized_vec.segment(0,NUM_JOINT_DOFS));
  OUT_LOG(logDEBUG) << "Set: generalized_" << unit_enum_string(u) << " <-- " << generalized_vec;
}

/// With Base
void Pacer::Robot::get_generalized_value(Pacer::Robot::unit_e u, Ravelin::VectorNd& generalized_vec){
  switch(u){
    case(position_goal):
    case(position):
      generalized_vec.set_zero(NUM_JOINT_DOFS+NEULER);
      break;
    default:
      generalized_vec.set_zero(NUM_JOINT_DOFS+NSPATIAL);
      break;
  }
  generalized_vec.set_sub_vec(0,get_joint_generalized_value(u));
  generalized_vec.set_sub_vec(NUM_JOINT_DOFS,get_base_value(u));
  OUT_LOG(logDEBUG) << "Get: generalized_" << unit_enum_string(u) << " --> " << generalized_vec;
}

/// With Base
Ravelin::VectorNd Pacer::Robot::get_generalized_value(Pacer::Robot::unit_e u){
  Ravelin::VectorNd generalized_vec;
  get_generalized_value(u,generalized_vec);
  return generalized_vec;
}

void Pacer::Robot::set_base_value(Pacer::Robot::unit_e u,const Ravelin::VectorNd& vec){
  OUT_LOG(logDEBUG) << "Set: base_" << unit_enum_string(u) << " <-- " << vec;
  if (fixed_base()){
    throw std::runtime_error("This robot has a fixed (0 dof) base.");
  }
  
  check_phase_internal(u);
  switch(u){
    case(position_goal):
    case(position):
      if(vec.rows() != NEULER)
        throw std::runtime_error("position vector should have 7 rows [lin(x y z), quat(x y z w)]");
      break;
    default:
      if(vec.rows() != NSPATIAL)
        throw std::runtime_error("spatial vector should have 6 rows [lin(x y z), ang(x y z)]");
      break;
  }
  
#ifdef USE_THREADS
  pthread_mutex_lock(&_state_mutex);
#endif
  _base_state[u] = vec;
#ifdef USE_THREADS
  pthread_mutex_unlock(&_state_mutex);
#endif
  
}

void Pacer::Robot::get_base_value(Pacer::Robot::unit_e u, Ravelin::VectorNd& vec){
  if (fixed_base()){
    vec.set_zero(0);
    return;
  }

  vec = _base_state[u];
  OUT_LOG(logDEBUG) << "Get: base_" << unit_enum_string(u) << " --> " << vec;
}

Ravelin::VectorNd Pacer::Robot::get_base_value(Pacer::Robot::unit_e u){
  Ravelin::VectorNd vec;
  get_base_value(u,vec);
  return vec;
}

void Pacer::Robot::set_end_effector_value(const std::string& id, unit_e u, const Ravelin::Origin3d& val)
{
  OUT_LOG(logDEBUG) << "Set: foot "<< id <<"_" << unit_enum_string(u) << " <-- " << val;
  check_phase_internal(u);
#ifdef USE_THREADS
  pthread_mutex_lock(&_state_mutex);
#endif
  _end_effector_state[u][id] = val;
  _end_effector_is_set[id] = true;
  
#ifdef USE_THREADS
  pthread_mutex_unlock(&_state_mutex);
#endif
}

Ravelin::Origin3d& Pacer::Robot::get_end_effector_value(const std::string& id, unit_e u, Ravelin::Origin3d& val)
{
#ifdef USE_THREADS
  pthread_mutex_lock(&_state_mutex);
#endif
  val = _end_effector_state[u][id];
#ifdef USE_THREADS
  pthread_mutex_unlock(&_state_mutex);
#endif
  OUT_LOG(logDEBUG) << "Get: foot "<< id <<"_" << unit_enum_string(u) << " --> " << val;
  return val;
}

Ravelin::Origin3d Pacer::Robot::get_end_effector_value(const std::string& id, unit_e u)
{
  
  Ravelin::Origin3d val;
  get_end_effector_value(id,u,val);
  return val;
}

void Pacer::Robot::set_end_effector_value(Pacer::Robot::unit_e u, const std::map<std::string,Ravelin::Origin3d>& val)
{
  check_phase_internal(u);
#ifdef USE_THREADS
  pthread_mutex_lock(&_state_mutex);
#endif
  std::map<std::string, Ravelin::Origin3d >::const_iterator it;
  for (it=val.begin(); it != val.end(); it++) {
    std::map<std::string, Ravelin::Origin3d >::iterator jt = _end_effector_state[u].find((*it).first);
    if(jt != _end_effector_state[u].end()){
      OUT_LOG(logDEBUG) << "Set: foot "<< (*it).first << "_" << unit_enum_string(u) << " <-- " << (*it).second;
      (*jt).second = (*it).second;
      _end_effector_is_set[(*it).first] = true;
    }
  }
#ifdef USE_THREADS
  pthread_mutex_unlock(&_state_mutex);
#endif
}

std::map<std::string,Ravelin::Origin3d>& Pacer::Robot::get_end_effector_value(Pacer::Robot::unit_e u, std::map<std::string,Ravelin::Origin3d>& val)
{
#ifdef USE_THREADS
  pthread_mutex_lock(&_state_mutex);
#endif
  std::map<std::string, Ravelin::Origin3d >::iterator it;
  for (it=_end_effector_state[u].begin(); it != _end_effector_state[u].end(); it++) {
    OUT_LOG(logDEBUG) << "Get: foot "<< (*it).first << "_" << unit_enum_string(u) << " --> " << (*it).second;
    val[(*it).first] = (*it).second;
  }
#ifdef USE_THREADS
  pthread_mutex_unlock(&_state_mutex);
#endif
  return val;
}

std::map<std::string,Ravelin::Origin3d> Pacer::Robot::get_end_effector_value(Pacer::Robot::unit_e u)
{
  std::map<std::string,Ravelin::Origin3d> val;
  get_end_effector_value(u,val);
  return val;
}

void Pacer::Robot::init_state(){
  check_phase_internal(initialization);
  reset_contact();
  // TODO: make this more efficient ITERATORS dont work
  //      std::map<std::string,Ravelin::VectorNd>::iterator it;
  
  const int num_state_units = 8;
  unit_e state_units[num_state_units] = {position,position_goal,velocity,velocity_goal,acceleration,acceleration_goal,load, load_goal};
  for(int j=0; j<num_state_units;j++){
    unit_e& u = state_units[j];
    _end_effector_state[u] = std::map<std::string, Ravelin::Origin3d >();
    const std::vector<std::string>& foot_keys = _end_effector_ids;
    for(int i=0;i<foot_keys.size();i++){
      _end_effector_state[u][foot_keys[i]].set_zero();
      _end_effector_is_set[foot_keys[i]] = false;
    }
    
    _state[u] = std::map<std::string, Ravelin::VectorNd >();
    const std::vector<std::string>& keys = _joint_ids;
    for(int i=0;i<keys.size();i++){
      const int N = get_joint_dofs(keys[i]);
      _state[u][keys[i]].set_zero(N);
    }
  }
}

void Pacer::Robot::reset_state(){
  check_phase_internal(clean_up);
  reset_contact();
  // TODO: make this more efficient ITERATORS dont work
  //      std::map<std::string,Ravelin::VectorNd>::iterator it;
  
  const int num_state_units = 8;
  unit_e state_units[num_state_units] = {position,position_goal,velocity,velocity_goal,acceleration,acceleration_goal,load, load_goal};
  for(int j=0; j<num_state_units;j++){
    unit_e& u = state_units[j];
    const std::vector<std::string>& foot_keys = _end_effector_ids;
    for(int i=0;i<foot_keys.size();i++){
      _end_effector_state[u][foot_keys[i]].set_zero();
      _end_effector_is_set[foot_keys[i]] = false;
    }
    
    const std::vector<std::string>& keys = _joint_ids;
    for(int i=0;i<keys.size();i++){
      const int N = get_joint_dofs(keys[i]);
      _state[u][keys[i]].set_zero(N);
    }
  }
}
