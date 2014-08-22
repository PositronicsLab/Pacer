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

    void find_footholds(std::vector<Ravelin::Vector3d>& footholds, int num_footholds);
    static void select_foothold(const std::vector<Ravelin::Vector3d>& footholds,const Ravelin::Origin3d &x, Ravelin::Origin3d& x_fh);

    void workspace_trajectory_goal(const Ravelin::SVector6d& v_base, const std::vector<Ravelin::Vector3d>& foot_pos,const std::vector<Ravelin::Vector3d>& foot_vel,const std::vector<Ravelin::Vector3d>& foot_acc,
                                              double beta, double dt, Ravelin::VectorNd& v_bar);

    void trajectory_ik(const std::vector<Ravelin::Vector3d>& foot_pos,const std::vector<Ravelin::Vector3d>& foot_vel,const std::vector<Ravelin::Vector3d>& foot_acc,
                                  Ravelin::VectorNd& q_des,Ravelin::VectorNd& qd_des,Ravelin::VectorNd& qdd_des);

    /// Walks while trying to match COM velocity "command" in base_frame
     void walk_toward(const Ravelin::SVector6d& command,const std::vector<double>& touchdown,const std::vector<Ravelin::Vector3d>& footholds,
                                const std::vector<double> duty_factor, double gait_duration, double step_height,
                                const std::vector<Ravelin::Vector3d>& foot_origin, double t, const Ravelin::VectorNd& q,const Ravelin::VectorNd& qd,const Ravelin::VectorNd& qdd,
                                std::vector<Ravelin::Vector3d>& foot_pos, std::vector<Ravelin::Vector3d>& foot_vel, std::vector<Ravelin::Vector3d>& foot_acc);

    void eef_stiffness_fb(const std::vector<Ravelin::Matrix3d>& W,double Kp, double Kv, double Ki, const std::vector<Ravelin::Vector3d>& x_des,const std::vector<Ravelin::Vector3d>& xd_des,const Ravelin::VectorNd& q,const Ravelin::VectorNd& qd,Ravelin::VectorNd& ufb);

public:
    std::vector<double> unknown_base_perturbation;// = boost::assign::list_of(0.0)(0.0)(0.0)(0.0)(0.0)(0.0).convert_to_container<std::vector<double> >();
    std::vector<double> known_base_perturbation;// = boost::assign::list_of(0.0)(0.0)(0.0)(0.0)(0.0)(0.0).convert_to_container<std::vector<double> >();
    std::vector<double> known_leading_force;// = boost::assign::list_of(0.13)(0.0)(0.0)(0.0)(0.0)(0.0)(0.0)(0.0)(0.0).convert_to_container<std::vector<double> >();

};
#endif // CONTROL_H
