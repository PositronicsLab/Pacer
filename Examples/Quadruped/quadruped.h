#ifndef CONTROL_H
#define CONTROL_H

#include <robot.h>

class Quadruped : public Robot{
  public:
  Quadruped(){}
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

    std::vector<Ravelin::Vector3d>& foot_oscilator(
      const std::vector<Ravelin::Vector3d>& x0,  const std::vector<Ravelin::Vector3d>& x, const Ravelin::MatrixNd& C,
      double Ls,const Ravelin::VectorNd& Hs,double Df,double Vf,double bp,std::vector<Ravelin::Vector3d>& xd);

    void init();

    void fk_stance_adjustment(double dt);

    void walk_toward(const Ravelin::SVector6d& command,const std::vector<std::vector<int> >& gait,double phase_time,double step_height,double t,Ravelin::VectorNd& q_des,Ravelin::VectorNd& qd_des,Ravelin::VectorNd& qdd);
};

#endif // CONTROL_H
