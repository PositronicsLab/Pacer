#include <quadruped.h>

void Quadruped::fk_stance_adjustment(double dt){
  static Ravelin::VectorNd workv_;
  static Ravelin::Vector3d workv3_;
  static Ravelin::MatrixNd workM_;

  if(NC < 3) return;

  Ravelin::VectorNd xd_foot(NSPATIAL), qd_base(NDOFS);
  qd_base.set_zero();

  Ravelin::Vector3d CoM_CoC_goal(0,0,0.13);
  (workv3_ = center_of_mass_x) -= center_of_contact.point;
  CoM_CoC_goal -= workv3_;
  qd_base.set_sub_vec(NUM_JOINTS,CoM_CoC_goal);
  qd_base.set_sub_vec(NUM_JOINTS+3,Ravelin::Vector3d(-roll_pitch_yaw[0],-roll_pitch_yaw[1],0));

  OUTLOG(qd_base,"BASE_GOAL",logDEBUG);
  OUTLOG(CoM_CoC_goal,"GOAL_COM",logDEBUG);
  OUTLOG(center_of_mass_x,"center_of_mass",logDEBUG);
  OUTLOG(center_of_contact.point,"center_of_contact",logDEBUG);

  for(int f=0;f<NUM_EEFS;f++){
    if(!eefs_[f].active)
      eef_origins_[eefs_[f].id] += Ravelin::Vector3d(0,0,-dt);
    else {
    // Calc jacobian for AB at this EEF
    boost::shared_ptr<Ravelin::Pose3d> foot_frame(new Ravelin::Pose3d(*eefs_[f].link->get_pose()));
    foot_frame->update_relative_pose(Moby::GLOBAL);
    boost::shared_ptr<Ravelin::Pose3d> event_frame(new Ravelin::Pose3d(Moby::GLOBAL));
    event_frame->x = foot_frame->x;
    event_frame->update_relative_pose(base_frame);

    dbrobot_->calc_jacobian(event_frame,eefs_[f].link,workM_);
//    OUTLOG(workM_,"J");

    workM_.mult(qd_base,xd_foot);
    xd_foot.get_sub_vec(0,3,workv3_);
    if(workv3_.norm() > 0.01){
      workv3_.normalize();
      workv3_ *= 1*dt;
      eef_origins_[eefs_[f].id] -= workv3_;
    }
    }

    OUTLOG(workv3_,eefs_[f].id,logDEBUG1);

  }
}

// Parallel Stiffness Controller
void Quadruped::eef_stiffness_fb(const std::vector<Ravelin::Matrix3d>& W, const std::vector<Ravelin::Vector3d>& x_des,const std::vector<Ravelin::Vector3d>& xd_des,const Ravelin::VectorNd& q,const Ravelin::VectorNd& qd,Ravelin::VectorNd& ufb){

  for(unsigned i=0;i< NUM_JOINTS;i++){
    joints_[i]->q[0]  = q[i];
    joints_[i]->qd[0]  = qd[i];
  }
  abrobot_->update_link_poses();
  abrobot_->update_link_velocities();

  double  Kp = 2e3,
          Kv = 2e1,
          Ki = 0;

  for(int i=0;i<NUM_EEFS;i++){
    EndEffector& foot = eefs_[i];

    // Calc jacobian for AB at this EEF
    Ravelin::MatrixNd J;
    Ravelin::Origin3d x;

    Ravelin::Vector3d u;

    for(int k=0;k<foot.chain.size();k++)                // actuated joints
      x[k] = q[foot.chain[k]];
    foot_jacobian(x,foot,boost::shared_ptr<Ravelin::Pose3d>(new Ravelin::Pose3d(*x_des[i].pose)),J);

    Ravelin::Vector3d x_err  = x_des[i] - Ravelin::Pose3d::transform_point(x_des[i].pose,Ravelin::Vector3d(0,0,0,eefs_[i].link->get_pose()));
    J.transpose_mult(W[i].mult(x_err,workv3_,Kp,0),u);

    Ravelin::Vector3d xd_err = xd_des[i] - Ravelin::Pose3d::transform_vector(x_des[i].pose,eefs_[i].link->get_velocity().get_linear());
    J.transpose_mult(W[i].mult(xd_err,workv3_,Kv,0),u,1,1);

    for(int k=0;k<foot.chain.size();k++)                // actuated joints
      ufb[foot.chain[k]] = u[k];
  }
}


