#ifndef CONTROL_H
#define CONTROL_H

#include <robot.h>

class Quadruped : public Robot{
  public:
    Quadruped();
    Quadruped(Moby::RCArticulatedBodyPtr abrobot){
      abrobot_ = abrobot;
      init();
    }
    Ravelin::VectorNd& control(double dt,
                               const Ravelin::VectorNd& q,
                               const Ravelin::VectorNd& qd,
                               Ravelin::VectorNd& q_des,
                               Ravelin::VectorNd& qd_des,
                               Ravelin::VectorNd& u);

    void sinusoidal_trot(Ravelin::VectorNd& q_des,Ravelin::VectorNd& qd_des,Ravelin::VectorNd& qdd,double dt);
    void cpg_trot(Ravelin::VectorNd& q_des,Ravelin::VectorNd& qd_des,Ravelin::VectorNd& qdd,double dt);

    void contact_jacobian_null_stabilizer(const Ravelin::MatrixNd& R, Ravelin::VectorNd& uff);
    std::vector<Ravelin::Vector3d>& foot_oscilator(
      const std::vector<Ravelin::Vector3d>& x0,  const std::vector<Ravelin::Vector3d>& x, const Ravelin::MatrixNd& C,
      double Ls,const Ravelin::VectorNd& Hs,double Df,double Vf,double bp,std::vector<Ravelin::Vector3d>& xd);

    /// 4 foot (body-fixed) state-space traj to joint-space traj
    std::vector<std::vector<Ravelin::Vector3d> > &trajectoryIK(
            const std::vector<std::vector<Ravelin::Vector3d> >& feet_positions,
            std::vector<std::vector<Ravelin::Vector3d> >& joint_positions);

    void feetIK(
            const std::vector<Ravelin::Vector3d>& feet_positions,
            std::vector<Ravelin::Vector3d>& joint_positions);
    void footIK(int foot,
            Ravelin::Vector3d& feet_positions,
            Ravelin::Vector3d& joint_positions);
    void init();

    void fk_stance_adjustment(double dt);

    void walk_toward(const Ravelin::SVector6d& command,const Ravelin::VectorNd& q,Ravelin::VectorNd& q_des,Ravelin::VectorNd& qd_des,Ravelin::VectorNd& qdd,double dt);
};

#endif // CONTROL_H
