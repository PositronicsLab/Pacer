#ifndef ROBOT_H
#define ROBOT_H

#include <project_common.h>
#include <pid.h>
class Robot;

class EndEffector{
public:

    EndEffector(){    }
    EndEffector(Moby::RigidBodyPtr l,Ravelin::Vector3d& o,std::vector<std::string>& jn,Robot* robot){
      link = l;
      id = link->id;
      origin = o;
      joint_names_ = jn;
      init(robot);
    }

    // permanent data
    std::string           id;
    Ravelin::Vector3d     origin;
    // Rigid Body (moby data)
    Moby::RigidBodyPtr    link;
    // kinematic chain indexing data
    std::vector<unsigned> chain;
    std::vector<bool>     chain_bool;
    // Contact Data
    std::vector<Ravelin::Vector3d>    point,
                                      normal,tan1,tan2,
                                      impulse;

    bool                  active;
    std::vector<double>   mu_viscous,
                          mu_coulomb;
    int                   nk;
    boost::shared_ptr<const Ravelin::Pose3d>
                          frame_environment,
                          frame_robot_base;
  private:
    std::vector<std::string>
                          joint_names_;
    void init(Robot* robot);
};

class Robot {
  public:
  boost::shared_ptr<Moby::EventDrivenSimulator> sim;
    Robot(){}
    Robot(std::string name) : robot_name_(name){}
    // This is the Simplest Controller (policy is determined within fn)
    virtual Ravelin::VectorNd& control(double dt,
                               const Ravelin::VectorNd& generalized_q,
                               const Ravelin::VectorNd& generalized_qd,
                               const Ravelin::VectorNd& generalized_qdd,
                               const Ravelin::VectorNd& generalized_fext,
                               Ravelin::VectorNd& q_des,
                               Ravelin::VectorNd& qd_des,
                               Ravelin::VectorNd& qdd_des,
                               Ravelin::VectorNd& u) = 0;
    void reset_contact();
    std::vector<EndEffector>& get_end_effectors()  { return eefs_; }
    std::vector<std::string>& get_end_effector_names()  { return eef_names_; }
    std::vector<Moby::JointPtr>& get_joints()  { return joints_; }
    std::vector<std::string>& get_joint_names()  { return joint_names_; }
    Moby::RCArticulatedBodyPtr& get_articulated_body()  { return abrobot_; }
    Moby::DynamicBodyPtr& get_dynamic_body()  { return dbrobot_; }
    boost::shared_ptr<const Ravelin::Pose3d>& get_base_link_frame(){return base_link_frame;}
    std::map<int, int>& get_joint_map()  { return joint_map_; }
    std::map<std::string, double>& get_q0()  { return q0_; }
    std::map<std::string,bool>& get_active_joints()  { return active_joints_; }

  protected:

    void compile();
    void calculate_dyn_properties(Ravelin::MatrixNd& M, Ravelin::VectorNd& fext);
    double calc_energy(Ravelin::VectorNd& v, Ravelin::MatrixNd& M);
    void calc_com();
    double friction_estimation(const Ravelin::VectorNd& v, const Ravelin::VectorNd& fext,
                               double dt, const Ravelin::MatrixNd& N,
                               const Ravelin::MatrixNd& ST, const Ravelin::MatrixNd& M,
                               Ravelin::MatrixNd& MU, Ravelin::VectorNd& cf);

    void contact_jacobian_null_stabilizer(const Ravelin::MatrixNd& R, const Ravelin::SVector6d& vel_des, Ravelin::VectorNd& ufb);
    /** FUNCTIONAL
     *  Applys *compresssive* forces and unlimited tangential forces
     * to correct the [p; v] state of the robot base
     * acording to Kp and Kv Respectively
    **/
    void contact_jacobian_stabilizer(const Ravelin::MatrixNd& R,const std::vector<double>& Kp,const std::vector<double>& Kv,const std::vector<double>& Ki,
                                     const std::vector<double>& pos_des, const std::vector<double>& vel_des, Ravelin::VectorNd& ufb);
    void zmp_stabilizer(const Ravelin::MatrixNd& R,const Ravelin::Vector2d& zmp_goal, Ravelin::VectorNd& ufb);
    bool inverse_dynamics(const Ravelin::VectorNd& v, const Ravelin::VectorNd& qdd, const Ravelin::MatrixNd& M,
                          const  Ravelin::MatrixNd& N, const Ravelin::MatrixNd& ST, const Ravelin::VectorNd& fext,
                          double h, const Ravelin::MatrixNd& MU, Ravelin::VectorNd& uff, Ravelin::VectorNd& cf_final);
    bool workspace_inverse_dynamics(const Ravelin::VectorNd& v, const Ravelin::VectorNd& v_bar, const Ravelin::MatrixNd& M, const Ravelin::VectorNd& fext, double h, const Ravelin::MatrixNd& MU, Ravelin::VectorNd& x, Ravelin::VectorNd& z);
    bool workspace_inverse_dynamics(const Ravelin::VectorNd& v, const Ravelin::VectorNd& v_bar, const Ravelin::MatrixNd& M, const Ravelin::VectorNd& fext, double h, const Ravelin::MatrixNd& MU, Ravelin::VectorNd& x);
    void calc_contact_jacobians(Ravelin::MatrixNd& N,Ravelin::MatrixNd& D,Ravelin::MatrixNd& R);
    void calc_contact_jacobians2(Ravelin::MatrixNd& N,Ravelin::MatrixNd& D,Ravelin::MatrixNd& R);
    void calc_base_jacobian(Ravelin::MatrixNd& R);

    void calc_workspace_jacobian(Ravelin::MatrixNd& Rw);
    void RMRC(const EndEffector& foot,const Ravelin::VectorNd& q,const Ravelin::Vector3d& goal,Ravelin::VectorNd& q_des);

  //  Ravelin::VectorNd& kinematics(const Ravelin::VectorNd& x, Ravelin::VectorNd& fk, Ravelin::MatrixNd& gk);
    Ravelin::VectorNd& foot_kinematics(const Ravelin::VectorNd& x,const EndEffector& foot, Ravelin::VectorNd& fk, Ravelin::MatrixNd& gk);
    Ravelin::VectorNd& foot_kinematics(const Ravelin::VectorNd& x,const EndEffector& foot,const boost::shared_ptr<const Ravelin::Pose3d> frame, const Ravelin::Vector3d& goal, Ravelin::VectorNd& fk, Ravelin::MatrixNd& gk);
    Ravelin::MatrixNd& foot_jacobian(const Ravelin::VectorNd& x,const EndEffector& foot,const boost::shared_ptr<const Ravelin::Pose3d> frame, Ravelin::MatrixNd& gk);
    void update();
    void update_poses();
    void set_model_state(const Ravelin::VectorNd& q,const Ravelin::VectorNd& qd = Ravelin::VectorNd::zero(0));

  protected:
    std::string                       robot_name_;
    // Robot Dynamics Datastructures
    Moby::RCArticulatedBodyPtr        abrobot_;
    Moby::DynamicBodyPtr              dbrobot_;
    std::vector<Moby::JointPtr>       joints_;
    std::vector<Moby::RigidBodyPtr>   links_;

    std::vector<std::string>          joint_names_;

    // End Effector data
    std::vector<std::string>          eef_names_;
    std::vector<EndEffector>          eefs_;
    std::map<std::string,bool>        active_joints_;


    unsigned                          NUM_FIXED_JOINTS;
    unsigned                          NUM_EEFS;
    unsigned                          NUM_JOINTS;
    unsigned                          NUM_LINKS;

    // Useful Stored Data
    boost::shared_ptr<const Ravelin::Pose3d>   base_horizontal_frame,
                                               base_frame,
                                               environment_frame,
                                               base_link_frame;

    unsigned          NC;
    EndEffector       center_of_contact;
    Ravelin::Vector3d center_of_mass_x,
                      center_of_mass_xd,
                      center_of_mass_xdd,
                      center_of_feet_x,
                      roll_pitch_yaw;
    Ravelin::Vector2d zero_moment_point;
    Ravelin::VectorNd q,qd,qdd;
    Ravelin::VectorNd generalized_q,generalized_qd,generalized_qdd;
    Ravelin::MatrixNd N,D,M,R,Rw;
    Ravelin::VectorNd generalized_fext;
    // NDFOFS for forces, accel, & velocities
    unsigned                          NDOFS,NUM_JOINT_DOFS;
    unsigned                          NSPATIAL;
    unsigned                          NEULER;
    Ravelin::SVector6d                displace_base_link;

  // PHYSICAL ROBOT LIMITS AND VARIABLES
    std::map<std::string, double>     q0_;
    std::map<int, int>     joint_map_;
    Ravelin::VectorNd     torque_limits_l,torque_limits_u;

  // All Names, vectors and, maps must be aligned,
  // this function sorts everything to be sure of that
};

#endif // ROBOT_H
