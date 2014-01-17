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
  Ravelin::VectorNd& control(const Ravelin::VectorNd& q,
                                         const Ravelin::VectorNd& qd,
                                         Ravelin::VectorNd& q_des,
                                         Ravelin::VectorNd& qd_des,
                                         Ravelin::VectorNd& u);

double friction_estimation(const Ravelin::VectorNd& v, const Ravelin::VectorNd& fext,
                           double dt, const Ravelin::MatrixNd& N,
                           const Ravelin::MatrixNd& ST, const Ravelin::MatrixNd& M,
                           Ravelin::MatrixNd& MU, Ravelin::VectorNd& cf);


void inverse_dynamics(const Ravelin::VectorNd& v, const Ravelin::VectorNd& qdd, const Ravelin::MatrixNd& M,
                      const  Ravelin::MatrixNd& N, const Ravelin::MatrixNd& ST, const Ravelin::VectorNd& fext,
                      double h, const Ravelin::MatrixNd& MU, Ravelin::VectorNd& uff);

std::vector<Ravelin::Vector3d>& foot_oscilator(
  const std::vector<Ravelin::Vector3d>& x0,  const std::vector<Ravelin::Vector3d>& x, const Ravelin::MatrixNd& C,
  double Ls,const Ravelin::VectorNd& Hs,double Df,double Vf,double bp,std::vector<Ravelin::Vector3d>& xd);

// 4 Control Point Bezier curve EEF calculation
std::vector<Ravelin::Vector3d>& stepTrajectory(
    const std::vector<Ravelin::Vector3d>& control_points,
    int num_segments, std::vector<Ravelin::Vector3d>& trajectory);

/// 4 foot (body-fixed) state-space traj to joint-space traj
std::vector<std::vector<Ravelin::Vector3d> > &trajectoryIK(
        const std::vector<std::vector<Ravelin::Vector3d> >& feet_positions,
        std::vector<std::vector<Ravelin::Vector3d> >& joint_positions);

std::vector<Ravelin::Vector3d>& feetIK(
        const std::vector<Ravelin::Vector3d>& feet_positions,
        std::vector<Ravelin::Vector3d>& joint_positions);
void init();
};

#endif // CONTROL_H
