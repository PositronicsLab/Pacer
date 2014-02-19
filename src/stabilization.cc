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
  OUTLOG(R,"R");
  // Select active rows in Jacobian Matrix
  // R -> Jh
  static Ravelin::MatrixNd Jh;
  for(int i=NUM_JOINTS;i<NDOFS;i++)
    active_dofs.push_back(i);

  std::sort(active_dofs.begin(),active_dofs.end());
  R.select_rows(active_dofs.begin(),active_dofs.end(),Jh);

//  Jh.transpose();
  OUTLOG(Jh,"Jh");

  // Generate Jh Nullspace
  Ravelin::MatrixNd NULL_Jh = MatrixNd::identity(Jh.columns()),
      Jh_plus;
//  Jh.mult_transpose(Jh,Jh_plus);
//  OUTLOG(Jh_plus,"(Jh Jh')");
  LA_.pseudo_invert(Jh);
  Jh_plus = Jh;
  OUTLOG(Jh,"Jh' (Jh Jh')^-1 == Jh+");

  LA_.nullspace(Jh,NULL_Jh);

  OUTLOG(NULL_Jh,"null(Jh)");
  // Use NULL(Jh) to stabilize trunk w/o altering gait

  // Trunk Stability gains
  Ravelin::Vector3d& rpy = roll_pitch_yaw;

  OUTLOG(rpy,"Roll, Pitch, Yaw");
  double Xp = 0, Xv = 0,
         Yp = 0, Yv = 0,
         Zp = 0, Zv = 1e-1,
         Rp = 1e1, Rv = 1e-1,
         Pp = 1e1, Pv = 1e-1;

  static Ravelin::VectorNd Y,tY;
  Y.set_zero(NC*3+6);
  tY.set_zero(NC*3);

  Y[NC*3+2] = -(0)*Zp + -(vel[NUM_JOINTS+2])*Zv;
  Y[NC*3+3] = -rpy[0]*Rp + -vel[NUM_JOINTS+3]*Rv;
  Y[NC*3+4] = -rpy[1]*Pp + -vel[NUM_JOINTS+4]*Pv;
  OUTLOG(Y,"g_stabilization");
//  NULL_Jh.transpose_mult(Y,tY);
  Jh_plus.mult(Y,tY);
  OUTLOG(tY,"stabilization");
  // apply base stabilization forces
  for(int i=0;i<NC*3;i++)
    uff[active_dofs[i]] += tY[i];


}

void Quadruped::fk_stance_adjustment(double dt){
  static Ravelin::VectorNd workv_;
  static Ravelin::Vector3d workv3_;
  static Ravelin::MatrixNd workM_;

  if(NC == 1)
    for(int f=0;f<NUM_EEFS;f++)
      if(!eefs_[f].active){
        eef_origins_[eefs_[f].id] += Ravelin::Vector3d(0,0,dt);
        return;
      }

  if(NC < 3) return;

  Ravelin::VectorNd xd_foot(NSPATIAL), qd_base(NDOFS);
  qd_base.set_zero();

  Ravelin::Vector3d CoM_CoC_goal(0,0,0.13);
  (workv3_ = center_of_mass) -= center_of_contact.point;
  CoM_CoC_goal -= workv3_;
  qd_base.set_sub_vec(NUM_JOINTS,CoM_CoC_goal);
  qd_base.set_sub_vec(NUM_JOINTS+3,Ravelin::Vector3d(-roll_pitch_yaw[0],-roll_pitch_yaw[1],0));

  OUTLOG(qd_base,"BASE_GOAL");
  OUTLOG(CoM_CoC_goal,"GOAL_COM");
  OUTLOG(center_of_mass,"center_of_mass");
  OUTLOG(center_of_contact.point,"center_of_contact");

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

    OUTLOG(workv3_,eefs_[f].id);

  }
}
