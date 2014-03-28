#include<quadruped.h>
#include <utilities.h>

using namespace Ravelin;

void Quadruped::contact_jacobian_null_stabilizer(const Ravelin::MatrixNd& R, Ravelin::VectorNd& uff){
  static Ravelin::VectorNd workv_;
  static Ravelin::MatrixNd workM_;

  if(NC == 0) return;

  std::vector<int> active_dofs;
  for(int i=0;i<NUM_EEFS;i++)
    if(eefs_[i].active)
      for(int j=0;j<eefs_[i].chain.size();j++)
        active_dofs.push_back(eefs_[i].chain[j]);

  if(active_dofs.size() == 0) return;
  OUTLOG(R,"R",logDEBUG1);
  // Select active rows in Jacobian Matrix
  // R -> Jh
  static Ravelin::MatrixNd Jh;
  for(int i=NUM_JOINTS;i<NDOFS;i++)
    active_dofs.push_back(i);

  std::sort(active_dofs.begin(),active_dofs.end());
  R.select_rows(active_dofs.begin(),active_dofs.end(),Jh);

//  Jh.transpose();
  OUTLOG(Jh,"Jh",logDEBUG1);

  // Generate Jh Nullspace
  Ravelin::MatrixNd NULL_Jh = MatrixNd::identity(Jh.columns()),
      Jh_plus;
//  Jh.mult_transpose(Jh,Jh_plus);
//  OUTLOG(Jh_plus,"(Jh Jh')");
  LA_.pseudo_invert(Jh);
  Jh_plus = Jh;
  OUTLOG(Jh,"Jh' (Jh Jh')^-1 == Jh+",logDEBUG1);

  LA_.nullspace(Jh,NULL_Jh);

  OUTLOG(NULL_Jh,"null(Jh)",logDEBUG1);
  // Use NULL(Jh) to stabilize trunk w/o altering gait

  // Trunk Stability gains
  Ravelin::Vector3d& rpy = roll_pitch_yaw;

  OUTLOG(rpy,"Roll, Pitch, Yaw",logDEBUG1);
  double Xp = 0, Xv = 0,
         Yp = 0, Yv = 0,
         Zp = 0, Zv = 0,
         Rp = 0, Rv = 1e-1,
         Pp = 0, Pv = 1e-1;

  static Ravelin::VectorNd Y,tY;
  Y.set_zero(NC*3+6);
  tY.set_zero(NC*3);

  Y[NC*3+2] = -(0)*Zp + -(vel[NUM_JOINTS+2])*Zv;
  Y[NC*3+3] = -rpy[0]*Rp + -vel[NUM_JOINTS+3]*Rv;
  Y[NC*3+4] = -rpy[1]*Pp + -vel[NUM_JOINTS+4]*Pv;
  Y.negate();
  OUTLOG(Y,"g_stabilization",logDEBUG1);
//  NULL_Jh.transpose_mult(Y,tY);
  Jh_plus.mult(Y,tY);
  OUTLOG(tY,"stabilization",logDEBUG1);
  // apply base stabilization forces
  for(int i=0;i<NC*3;i++)
    uff[active_dofs[i]] += tY[i];


}

void Quadruped::fk_stance_adjustment(double dt){
  static Ravelin::VectorNd workv_;
  static Ravelin::Vector3d workv3_;
  static Ravelin::MatrixNd workM_;

  if(NC < 3) return;

  Ravelin::VectorNd xd_foot(NSPATIAL), qd_base(NDOFS);
  qd_base.set_zero();

  Ravelin::Vector3d CoM_CoC_goal(0,0,0.13);
  (workv3_ = center_of_mass) -= center_of_contact.point;
  CoM_CoC_goal -= workv3_;
  qd_base.set_sub_vec(NUM_JOINTS,CoM_CoC_goal);
  qd_base.set_sub_vec(NUM_JOINTS+3,Ravelin::Vector3d(-roll_pitch_yaw[0],-roll_pitch_yaw[1],0));

  OUTLOG(qd_base,"BASE_GOAL",logDEBUG);
  OUTLOG(CoM_CoC_goal,"GOAL_COM",logDEBUG);
  OUTLOG(center_of_mass,"center_of_mass",logDEBUG);
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
void Robot::eef_stiffness_fb(const Ravelin::VectorNd& q_des, const Ravelin::VectorNd& qd_des,
                             const Ravelin::VectorNd& q    , const Ravelin::VectorNd& qd    ,
                             Ravelin::VectorNd& ufb){
  static Ravelin::MatrixNd workM2_;
  static std::vector<double> Kp(NUM_EEFS), Kv(NUM_EEFS),Ka(NUM_EEFS), Kw(NUM_EEFS);
//  boost::shared_ptr<Ravelin::Pose3d> stiffness_frame = boost::shared_ptr<Ravelin::Pose3d>(new Ravelin::Pose3d(*base_frame));
  boost::shared_ptr<Ravelin::Pose3d> stiffness_frame = boost::shared_ptr<Ravelin::Pose3d>(new Ravelin::Pose3d(Moby::GLOBAL));

  boost::shared_ptr<Ravelin::Pose3d> event_frame = boost::shared_ptr<Ravelin::Pose3d>(new Ravelin::Pose3d(*stiffness_frame));

  for(int i=0;i<NUM_EEFS;i++){
    Kp[i] = 2e3;
    Kv[i] = 2e1;
    Ka[i] = 0;//1e-1;
    Kw[i] = 0;//1e-1;

    // Update robot model to desired position
    for(int k=0;k<eefs_[i].chain.size();k++)
      joints_[eefs_[i].chain[k]]->q[0] = q_des[eefs_[i].chain[k]];
    abrobot_->update_link_poses();

    // desired eef pos & vel
    Ravelin::Pose3d foot_pose = *eefs_[i].link->get_pose();
    foot_pose.update_relative_pose(stiffness_frame);
    Ravelin::SVector6d x_des = Ravelin::SVector6d(Ravelin::Vector3d(foot_pose.x,stiffness_frame),
                                                  Ravelin::Vector3d(R2rpy(Matrix3d(foot_pose.q),workv3_)));
//                                                  Ravelin::Vector3d(quat2rpy(foot_pose.q,workv3_)));
//    Ravelin::Vector3d x_des = Ravelin::Pose3d::transform_point(base_frame,Ravelin::Vector3d(0,0,0,eefs_[i].link->get_pose()));
    // Get jacobian at this position
//    OUTLOG(R2rpy(Matrix3d(foot_pose.q),workv3_),"aQ_des");
//    OUTLOG(quat2rpy(foot_pose.q,workv3_),"aR_des");

    event_frame->x = x_des.get_upper();
    abrobot_->calc_jacobian(event_frame,eefs_[i].link,workM_);
    Ravelin::SVector6d xd_des = (workM_.get_sub_mat(0,6,0,NUM_JOINTS,workM2_)).mult(qd_des,workv_);

    // Update robot model to desired position
    for(int k=0;k<eefs_[i].chain.size();k++)
      joints_[eefs_[i].chain[k]]->q[0] = q[eefs_[i].chain[k]];
    abrobot_->update_link_poses();

    // eef pos & vel
    foot_pose = *eefs_[i].link->get_pose();
    foot_pose.update_relative_pose(stiffness_frame);
    Ravelin::SVector6d x = Ravelin::SVector6d(Ravelin::Vector3d(foot_pose.x,stiffness_frame),
                                              Ravelin::Vector3d(R2rpy(Matrix3d(foot_pose.q),workv3_)));
//                                              Ravelin::Vector3d(quat2rpy(foot_pose.q,workv3_)));
//    OUTLOG(R2rpy(Matrix3d(foot_pose.q),workv3_),"aQ");
//    OUTLOG(quat2rpy(foot_pose.q,workv3_),"aR");
//    Ravelin::Vector3d x = Ravelin::Pose3d::transform_point(base_frame,Ravelin::Vector3d(0,0,0,eefs_[i].link->get_pose()));
    // Get jacobian at this position
    event_frame->x = x.get_upper();
    abrobot_->calc_jacobian(event_frame,eefs_[i].link,workM_);
    Ravelin::SVector6d xd = (workM_.get_sub_mat(0,6,0,NUM_JOINTS,workM2_)).mult(qd,workv_);

    // worksapce Feedback calculation
    xd_des.pose = x_des.pose = xd.pose = x.pose = stiffness_frame;
    Ravelin::SVector6d x_err(x_des.get_upper() - x.get_upper(),
                             x_des.get_lower() - x.get_lower()),
                      xd_err(xd_des.get_upper() - xd.get_upper(),
                             xd_des.get_lower() - xd.get_lower());
    OUTLOG(x_err,"x_err",logDEBUG1);
    OUTLOG(xd_err,"xd_err",logDEBUG1);

    Ravelin::SVector6d ffb(Kp[i]*x_err.get_upper() + Kv[i]*xd_err.get_upper(),
                           Ka[i]*x_err.get_lower() + Kw[i]*xd_err.get_lower());

    OUTLOG(ffb,"ffb",logDEBUG1);

    // workspace -> configuration space
    ufb += workM_.get_sub_mat(0,6,0,NUM_JOINTS,workM2_).transpose_mult(ffb,workv_);
  }

  OUTLOG(ufb,"ufb",logDEBUG1);
  // Return robot to initial config
  for(int i=0;i<NUM_JOINTS;i++)
    joints_[i]->q[0] = q[i];
  abrobot_->update_link_poses();
}
