#include <quadruped.h>

void Quadruped::fk_stance_adjustment(double dt){
  static Ravelin::VectorNd workv_;
  static Ravelin::Vector3d workv3_;
  static Ravelin::MatrixNd workM_;

  if(NC < 3) return;

  Ravelin::VectorNd xd_foot(NSPATIAL), qd_base(NDOFS);
  qd_base.set_zero();

  Ravelin::Vector3d CoM_CoC_goal(0,0,0.13,environment_frame);
  (workv3_ = center_of_mass_x) -= center_of_contact.point[0];
  CoM_CoC_goal -= workv3_;
  qd_base.set_sub_vec(NUM_JOINTS,CoM_CoC_goal);
  qd_base.set_sub_vec(NUM_JOINTS+3,Ravelin::Vector3d(-roll_pitch_yaw[0],-roll_pitch_yaw[1],0));

  OUTLOG(qd_base,"BASE_GOAL",logDEBUG);
  OUTLOG(CoM_CoC_goal,"GOAL_COM",logDEBUG);
  OUTLOG(center_of_mass_x,"center_of_mass",logDEBUG);
  OUTLOG(center_of_contact.point[0],"center_of_contact",logDEBUG);

//  for(int f=0;f<NUM_EEFS;f++){
//    Rw.get_sub_mat(NUM)
//    if(workv3_.norm() > 0.01){
//      workv3_.normalize();
//      workv3_ *= 1.0*dt;
//      for(int i=0;i<3;i++)
//        eefs_[f].origin[i] -= workv3_[i];
//    }
//    }

//    OUTLOG(workv3_,eefs_[f].id,logDEBUG1);

//  }
}

// Parallel Stiffness Controller
void Quadruped::eef_stiffness_fb(const std::vector<double>& Kp, const std::vector<double>& Kv, const std::vector<double>& Ki, const std::vector<Ravelin::Vector3d>& x_des,const std::vector<Ravelin::Vector3d>& xd_des,const Ravelin::VectorNd& q,const Ravelin::VectorNd& qd,Ravelin::VectorNd& fb){

  for(int i=0,ii=0;i<NUM_JOINTS;i++){
    if(joints_[i])
    for(int j=0;j<joints_[i]->num_dof();j++,ii++){
      joints_[i]->q[j]  = q[ii];
      joints_[i]->qd[j]  = qd[ii];
    }
  }
  abrobot_->update_link_poses();
  abrobot_->update_link_velocities();

  static std::vector<Ravelin::Vector3d> p_err_sum;
  if(p_err_sum.empty()){
    p_err_sum.resize(NUM_EEFS);
    for(int i=0;i<NUM_EEFS;i++)
      p_err_sum[i] = Ravelin::Origin3d(0,0,0);
  }

  for(int i=0;i<NUM_EEFS;i++){
    boost::shared_ptr<Ravelin::Pose3d> event_frame(new Ravelin::Pose3d(x_des[i].pose));
    EndEffector& foot = eefs_[i];

    // Calc jacobian for AB at this EEF
    Ravelin::MatrixNd J,Jf;
    Ravelin::VectorNd x(foot.chain.size());

    Ravelin::VectorNd u;

    for(int k=0;k<foot.chain.size();k++)                // actuated joints
      x[k] = q[foot.chain[k]];
    foot_jacobian(x,foot,boost::shared_ptr<Ravelin::Pose3d>(new Ravelin::Pose3d(*x_des[i].pose)),J);

    // Positional Correction
    Ravelin::Vector3d x_err  = x_des[i] - Ravelin::Pose3d::transform_point(x_des[i].pose,Ravelin::Vector3d(0,0,0,eefs_[i].link->get_pose()));
    x_err = Ravelin::Vector3d(x_err[0]*Kp[i*3],x_err[1]*Kp[i*3+1],x_err[2]*Kp[i*3+2]);
    J.transpose_mult(x_err,u);

    // Integrative Correction
    p_err_sum[i] += Ravelin::Origin3d(x_err.data());
    J.transpose_mult(Ravelin::Vector3d(p_err_sum[i][0]*Ki[i*3],p_err_sum[i][1]*Ki[i*3+1],p_err_sum[i][2]*Ki[i*3+2]),u,1,1);

    // Remove portion of foot velocity that can't be affected by corrective forces
    event_frame->x = Ravelin::Pose3d::transform_point(x_des[i].pose,Ravelin::Vector3d(0,0,0,eefs_[i].link->get_pose()));
    dbrobot_->calc_jacobian(event_frame,eefs_[i].link,Jf);
    Ravelin::SharedConstMatrixNd Jb = Jf.block(0,3,NUM_JOINTS,NDOFS);
    Ravelin::SharedConstVectorNd vb = generalized_qd.segment(NUM_JOINTS,NDOFS);
    Jb.mult(vb,workv3_);
    workv3_.pose = x_des[i].pose;

    // Velocity Correction
    Ravelin::Vector3d xd_err = xd_des[i] - (Ravelin::Pose3d::transform_vector(x_des[i].pose,eefs_[i].link->get_velocity().get_linear()) - workv3_);
    xd_err = Ravelin::Vector3d(xd_err[0]*Kv[i*3],xd_err[1]*Kv[i*3+1],xd_err[2]*Kv[i*3+2]);
    J.transpose_mult(xd_err,u,1,1);

    for(int k=0;k<foot.chain.size();k++)                // actuated joints
      fb[foot.chain[k]] += u[k];
  }
}


