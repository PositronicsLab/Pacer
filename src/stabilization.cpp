#include <utilities.h>
#include <controller.h>

extern bool solve_qp(const Ravelin::MatrixNd& Q, const Ravelin::VectorNd& c, const Ravelin::MatrixNd& A, const Ravelin::VectorNd& b, Ravelin::VectorNd& x);

using namespace Ravelin;

void Controller::contact_jacobian_stabilizer(const Ravelin::MatrixNd& R,const std::vector<double>& Kp,const std::vector<double>& Kv,const std::vector<double>& Ki,const std::vector<double>& pos_des,const std::vector<double>& vel_des, Ravelin::VectorNd& js_correct){
  static Ravelin::VectorNd workv_;
  static Ravelin::MatrixNd workM_;
  int NC = R.columns()/5;
  if(NC == 0) return;
//  workM_ = R;
//  workM_.remove_row(0);
  Ravelin::SharedConstMatrixNd Jb = R.block(NUM_JOINT_DOFS,NDOFS,0,NC*3);
  Ravelin::SharedConstMatrixNd Jq = R.block(0,NUM_JOINT_DOFS,0,NC*3);

  OUTLOG(Jb,"Jb",logDEBUG1);
  OUTLOG(Jq,"Jq",logDEBUG1);

  Ravelin::VectorNd vel_base(6), pos_base(6), base_correct(6);
  data->generalized_qd.get_sub_vec(NUM_JOINT_DOFS,data->generalized_qd.rows(), vel_base);
  pos_base.set_sub_vec(0,data->center_of_mass_x);
  pos_base.set_sub_vec(3,data->roll_pitch_yaw);

  OUTLOG(vel_base,"vel_base",logDEBUG1);
  OUTLOG(vel_des,"vel_des",logDEBUG1);
  OUTLOG(pos_base,"pos_base",logDEBUG1);
  OUTLOG(pos_des,"pos_des",logDEBUG1);

  static Ravelin::VectorNd sum_p_err = Ravelin::VectorNd::zero(6);
  for(int i=0;i<6;i++){
    sum_p_err[i]   += (pos_des[i] - pos_base[i]);
    base_correct[i] =   (vel_des[i] - vel_base[i])*Kv[i]
                      + (pos_des[i] - pos_base[i])*Kp[i]
                      + sum_p_err[i]*Ki[i];
  }

  OUTLOG(base_correct,"base_correct",logDEBUG);

  Ravelin::VectorNd ws_correct;
  Jb.transpose_mult(base_correct,ws_correct);
  OUTLOG(ws_correct,"ws_correct",logDEBUG);

  // Remove non-compressive elements (cN < 0)
  for(int i=0;i<NC;i++)
    if(ws_correct[i] < 0.0)
      ws_correct[i] = 0.0;
  OUTLOG(ws_correct,"ws_correct (compressive)",logDEBUG);

//   Remove Tangential Elements (for now)
  for(int i=NC;i<ws_correct.rows();i++)
      ws_correct[i] = 0.0;
  OUTLOG(ws_correct,"ws_correct (normal)",logDEBUG);

  Jq.mult(ws_correct,js_correct,-1.0,0);
  OUTLOG(js_correct,"js_correct",logDEBUG);
}


// Parallel Stiffness Controller
void Controller::eef_stiffness_fb(const std::vector<double>& Kp, const std::vector<double>& Kv, const std::vector<double>& Ki, const std::vector<Ravelin::Vector3d>& x_des,const std::vector<Ravelin::Vector3d>& xd_des,const Ravelin::VectorNd& q,const Ravelin::VectorNd& qd,Ravelin::VectorNd& fb){

    set_model_state(q,qd);

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
        J = J.get_sub_mat(0,3,0,J.columns(),workM_);

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
        Ravelin::SharedConstVectorNd vb = data->generalized_qd.segment(NUM_JOINTS,NDOFS);
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



