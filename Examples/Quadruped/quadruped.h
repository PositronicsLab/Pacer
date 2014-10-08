#ifndef CONTROL_H
#define CONTROL_H

#include <robot.h>
#include <CVars/CVar.h>

class Quadruped : public Robot{
  public:
    Quadruped(){}
    Quadruped(std::string name) : Robot(name){
      init();
    }

    Ravelin::VectorNd& control(double dt,
                               const Ravelin::VectorNd& generalized_q,
                               const Ravelin::VectorNd& generalized_qd,
                               const Ravelin::VectorNd& generalized_qdd,
                               const Ravelin::VectorNd& generalized_fext,
                               Ravelin::VectorNd& q_des,
                               Ravelin::VectorNd& qd_des,
                               Ravelin::VectorNd& qdd_des,
                               Ravelin::VectorNd& u);

    void sinusoidal_trot(Ravelin::VectorNd& q_des,Ravelin::VectorNd& qd_des,Ravelin::VectorNd& qdd,double dt);
    static bool gait_phase(double touchdown,double duty_factor,double gait_progress);
    static double gait_phase(double touchdown,double duty_factor,double gait_progress,double stance_phase);
    void cpg_trot(
        const Ravelin::SVector6d& command,
        const std::vector<double>& touchdown,
        const std::vector<double>& duty_factor,
        double gait_duration,
        double step_height,
        // MODEL
        const std::vector<Ravelin::Vector3d>& foot_origin,
        double t,
        // OUT
        std::vector<Ravelin::Vector3d>& foot_pos,
        std::vector<Ravelin::Vector3d>& foot_vel,
        std::vector<Ravelin::Vector3d>& foot_acc);

    std::vector<Ravelin::Vector3d>& foot_oscilator(
      const std::vector<Ravelin::Vector3d>& x0,  const std::vector<Ravelin::Vector3d>& x, const Ravelin::MatrixNd& C,
      double Ls,const Ravelin::VectorNd& Hs,    const std::vector<double>& Df,double Vf,double bp,std::vector<Ravelin::Vector3d>& xd);

    void init();

    void fk_stance_adjustment(double dt);

    void find_footholds(std::vector<Ravelin::Vector3d>& footholds, int num_footholds);
    static void select_foothold(const std::vector<Ravelin::Vector3d>& footholds,const Ravelin::Origin3d &x, Ravelin::Origin3d& x_fh);

    void workspace_trajectory_goal(const Ravelin::SVector6d& v_base, const std::vector<Ravelin::Vector3d>& foot_pos,const std::vector<Ravelin::Vector3d>& foot_vel,const std::vector<Ravelin::Vector3d>& foot_acc,
                                              double beta, double dt, Ravelin::VectorNd& v_bar);

    void trajectory_ik(const std::vector<Ravelin::Vector3d>& foot_pos,const std::vector<Ravelin::Vector3d>& foot_vel,const std::vector<Ravelin::Vector3d>& foot_acc,
                                 const Ravelin::VectorNd& q, Ravelin::VectorNd& q_des,Ravelin::VectorNd& qd_des,Ravelin::VectorNd& qdd_des);

    /// Walks while trying to match COM velocity "command" in base_frame
     void walk_toward(const Ravelin::SVector6d& command,
                      const std::vector<double>& touchdown,
                      const std::vector<Ravelin::Vector3d>& footholds,
                      const std::vector<double>& duty_factor,
                      double gait_duration,
                      double step_height,
                      bool STANCE_ON_CONTACT,
                      std::vector<EndEffector*>& feet,
                      const Ravelin::SVector6d &base_velocity,
                      const Ravelin::Vector3d &center_of_mass_x,
                      double t,
                      const Ravelin::VectorNd& q,
                      const Ravelin::VectorNd& qd,
                      const Ravelin::VectorNd& qdd,
                      std::vector<Ravelin::Vector3d>& foot_pos,
                      std::vector<Ravelin::Vector3d>& foot_vel,
                      std::vector<Ravelin::Vector3d>& foot_acc
                      );
     void walk_toward(const Ravelin::SVector6d& command,const std::vector<std::vector<double>>& heightmap  = std::vector<std::vector<double>>());

    void eef_stiffness_fb(const std::vector<double>& Kp, const std::vector<double>& Kv, const std::vector<double>& Ki, const std::vector<Ravelin::Vector3d>& x_des,const std::vector<Ravelin::Vector3d>& xd_des,const Ravelin::VectorNd& q,const Ravelin::VectorNd& qd,Ravelin::VectorNd& ufb);
    void set_leading_force(const Ravelin::SForced& f){lead_force_ = f;}
    void set_known_force(const Ravelin::SForced& f){known_force_ = f;}

    void load_variables(std::string fname);

    std::vector<double>& get_variable(const char* tag,std::vector<double>& val);
    double& get_variable(const char* tag,double& val);

    std::vector<std::string>& get_variable(const char* tag,std::vector<std::string>& val);
    std::string& get_variable(const char* tag,std::string& val);

    std::vector<int>& get_variable(const char* tag,std::vector<int>& val);
    int& get_variable(const char* tag,int& val);
  private:
    Ravelin::SForced lead_force_;
    Ravelin::SForced known_force_;
};
#endif // CONTROL_H
