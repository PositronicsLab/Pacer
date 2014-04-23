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

    static std::vector<std::vector<int> >& expand_gait(const std::vector<std::vector<int> >& gait1,int m, std::vector<std::vector<int> >& gait2);

    void init();

    void fk_stance_adjustment(double dt);

    void find_footholds(std::vector<Ravelin::Vector3d>& footholds, int num_footholds);
    static void select_foothold(const std::vector<Ravelin::Vector3d>& footholds,const Ravelin::Origin3d &x, Ravelin::Origin3d& x_fh);

    void workspace_trajectory_goal(const Ravelin::SVector6d& v_base, const std::vector<Ravelin::Vector3d>& foot_pos,const std::vector<Ravelin::Vector3d>& foot_vel,const std::vector<Ravelin::Vector3d>& foot_acc,
                                              double beta, double dt, Ravelin::VectorNd& v_bar);

    void trajectory_ik(const std::vector<Ravelin::Vector3d>& foot_pos,const std::vector<Ravelin::Vector3d>& foot_vel,const std::vector<Ravelin::Vector3d>& foot_acc,
                                  Ravelin::VectorNd& q_des,Ravelin::VectorNd& qd_des,Ravelin::VectorNd& qdd_des);

    /// Walks while trying to match COM velocity "command" in base_frame
    void walk_toward(const Ravelin::SVector6d& command,const std::vector<std::vector<int> >& gait,const std::vector<Ravelin::Vector3d>& footholds,double  interval_time,double step_height,double t,
                               const Ravelin::VectorNd& q,const Ravelin::VectorNd& qd,const Ravelin::VectorNd& qdd,
                               std::vector<Ravelin::Vector3d>& foot_pos, std::vector<Ravelin::Vector3d>& foot_vel, std::vector<Ravelin::Vector3d>& foot_acc);

};

#endif // CONTROL_H
