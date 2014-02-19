#include <quadruped.h>

using namespace Ravelin;
using namespace Moby;

void lf(const double X[3],double th[3]);
void rf(const double X[3],double th[3]);
void lh(const double X[3],double th[3]);
void rh(const double X[3],double th[3]);

// inverse kinematics solver conversion

std::vector<std::vector<Ravelin::Vector3d> >& trajectoryIK(
        const std::vector<std::vector<Ravelin::Vector3d> >& feet_positions,
        std::vector<std::vector<Ravelin::Vector3d> >& joint_positions){

  int num_steps = joint_positions.size();
  int num_feet = feet_positions.size();
    for(int i=0;i<num_steps;i++){
        joint_positions[i].resize(num_feet);
        lf(feet_positions[0][i].data(),joint_positions[i][0].data());
        rf(feet_positions[1][i].data(),joint_positions[i][1].data());
        lh(feet_positions[2][i].data(),joint_positions[i][2].data());
        rh(feet_positions[3][i].data(),joint_positions[i][3].data());
    }

}

std::vector<Ravelin::Vector3d>& Quadruped::feetIK(
        const std::vector<Ravelin::Vector3d>& feet_positions,
        std::vector<Ravelin::Vector3d>& joint_positions){
  int num_feet = feet_positions.size();
      joint_positions.resize(num_feet);
      lf(feet_positions[0].data(),joint_positions[0].data());
      rf(feet_positions[1].data(),joint_positions[1].data());
      lh(feet_positions[2].data(),joint_positions[2].data());
      rh(feet_positions[3].data(),joint_positions[3].data());

      return joint_positions;
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
void Quadruped::calc_contact_jacobians(Ravelin::MatrixNd& N,Ravelin::MatrixNd& ST,Ravelin::MatrixNd& D,Ravelin::MatrixNd& R){
  static Ravelin::VectorNd workv_;
  static Ravelin::MatrixNd workM_;
  unsigned NC = 0;
  for (unsigned i=0; i< NUM_EEFS;i++)
    if(eefs_[i].active)
      NC++;

  N.set_zero(NDOFS,NC);
  ST.set_zero(NDOFS,NC*2);
  D.set_zero(NDOFS,NC*NK);
  R.set_zero(NDOFS,NC*3);
  // Contact Jacobian [GLOBAL frame]
  Ravelin::MatrixNd J(3,NDOFS);
  for(int i=0,ii=0;i<NUM_EEFS;i++){
    if(!eefs_[i].active)
      continue;

    boost::shared_ptr<Ravelin::Pose3d> event_frame(new Ravelin::Pose3d(Moby::GLOBAL));
    event_frame->q = Ravelin::Quatd::identity();
    event_frame->x = eefs_[i].point;

    dbrobot_->calc_jacobian(event_frame,eefs_[i].link,workM_);

    workM_.get_sub_mat(0,3,0,NDOFS,J);

    Vector3d tan1, tan2;
    Vector3d::determine_orthonormal_basis(eefs_[i].normal, tan1, tan2);

    // Normal direction
    J.transpose_mult(eefs_[i].normal,workv_);
    N.set_column(ii,workv_);

    // 1st tangent
    J.transpose_mult(tan1,workv_);
    ST.set_column(ii,workv_);

    D.set_column(ii,workv_);
    workv_.negate();
    D.set_column(NC*2+ii,workv_);

    // 2nd tangent
    J.transpose_mult(tan2,workv_);
    ST.set_column(NC+ii,workv_);
    D.set_column(NC+ii,workv_);
    workv_.negate();
    D.set_column(NC*3+ii,workv_);

    ii++;
  }

  R.set_sub_mat(0,0,N);
  R.set_sub_mat(0,NC,ST);
}

void Quadruped::calc_eef_jacobians(Ravelin::MatrixNd& R){
  static Ravelin::VectorNd workv_;
  static Ravelin::MatrixNd workM_;

  R.set_zero(NDOFS,NUM_EEFS*3);
  // Contact Jacobian [GLOBAL frame]
  Ravelin::MatrixNd J(3,NDOFS);
  for(int i=0;i<NUM_EEFS;i++){
    boost::shared_ptr<Ravelin::Pose3d> event_frame(new Ravelin::Pose3d(Moby::GLOBAL));

    boost::shared_ptr<Ravelin::Pose3d> foot_frame = boost::shared_ptr<Ravelin::Pose3d>(new Ravelin::Pose3d(*eefs_[i].link->get_pose())) ;
    foot_frame->update_relative_pose(Moby::GLOBAL);

    event_frame->q = Ravelin::Quatd::identity();
    event_frame->x = foot_frame->x;

    dbrobot_->calc_jacobian(event_frame,eefs_[i].link,workM_);

    workM_.get_sub_mat(0,3,0,NDOFS,J);

    Ravelin::Vector3d normal = Ravelin::Vector3d(0,0,1),
                      tan1 = Ravelin::Vector3d(1,0,0),
                      tan2 = Ravelin::Vector3d(0,1,0);

    // Normal direction
    J.transpose_mult(normal,workv_);
    R.set_column(i,workv_);
    // 1st tangent
    J.transpose_mult(tan1,workv_);
    R.set_column(NUM_EEFS+i,workv_);
    // 2nd tangent
    J.transpose_mult(tan2,workv_);
    R.set_column(NUM_EEFS*2+i,workv_);
  }
}
