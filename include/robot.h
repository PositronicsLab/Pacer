#ifndef ROBOT_H
#define ROBOT_H

#include <project_common.h>
#include <pid.h>
class Robot;

struct EndEffector{
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
    bool                  stance;
    std::vector<double>   mu_viscous,
                          mu_coulomb;
    int                   nk;
};

struct RobotData{
  Ravelin::Vector2d zero_moment_point;
  Ravelin::VectorNd q,
                    qd,
                    qdd;
  Ravelin::VectorNd generalized_q,
                    generalized_qd,
                    generalized_qdd;
  Ravelin::MatrixNd N,D,M,R;
  Ravelin::VectorNd generalized_fext;
  Ravelin::Vector3d center_of_mass_x,
                    center_of_mass_xd,
                    center_of_mass_xdd,
                    roll_pitch_yaw;
};

class Robot{
  public:
#ifdef VISUALIZE_MOBY
    boost::shared_ptr<Moby::EventDrivenSimulator> sim;
#endif
    Robot(){}
    Robot(std::string name) : robot_name_(name){
      init();
    }

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

    /// ---------------------------  Getters  ---------------------------
    std::vector<EndEffector>& get_end_effectors()  { return eefs_; }
    std::vector<std::string>& get_end_effector_names()  { return eef_names_; }
    std::vector<Moby::JointPtr>& get_joints()  { return joints_; }
    std::vector<std::string>& get_joint_names()  { return joint_names_; }

    /// Return Robot's internal model
    Moby::RCArticulatedBodyPtr get_articulated_body()  { return abrobot_; }
    Moby::DynamicBodyPtr get_dynamic_body()  { return dbrobot_; }

    boost::shared_ptr<const Ravelin::Pose3d> get_base_link_frame(){return base_link_frame;}

    std::map<int, int>& get_joint_map()  { return joint_map_; }
    std::map<std::string, double>& get_q0()  { return q0_; }
    std::map<std::string, bool>& get_active_joints()  { return active_joints_; }

    /// Reset eefs_ data
    void reset_contact();

  protected:
    /// Update robot using currently set 'generalized' parameters
    void update(
        const Ravelin::VectorNd& generalized_q_in,
        const Ravelin::VectorNd& generalized_qd_in,
        const Ravelin::VectorNd& generalized_qdd_in,
        const Ravelin::VectorNd& generalized_fext_in);

    /// Update poses of robot based on  currently set 'generalized' parameters
    void update_poses();

    /// Get M and fext from model internal to plugin NOT simulator
    void calculate_dyn_properties(Ravelin::MatrixNd &M, Ravelin::VectorNd &fext);

    /// Calcultate kinetic energy of robot
    double calc_energy(Ravelin::VectorNd& v, Ravelin::MatrixNd& M);

    /// Calc Center of mass(x,xd,xdd,zmp)
    void calc_com();

    /// Set Plugin internal model to input state
    void set_model_state(const Ravelin::VectorNd& q,const Ravelin::VectorNd& qd = Ravelin::VectorNd::zero(0));

    /// Calculate N (normal), D (positive tangent), R ([N D]) contact jacobians
    void calc_contact_jacobians(Ravelin::MatrixNd& N,Ravelin::MatrixNd& D,Ravelin::MatrixNd& R);

    /// Resolved Motion Rate control (iterative inverse kinematics)
    void RMRC(const EndEffector& foot,const Ravelin::VectorNd& q,const Ravelin::Vector3d& goal,Ravelin::VectorNd& q_des);
    void RMRC(const EndEffector& foot,const Ravelin::VectorNd& q,const Ravelin::SVector6d& goal,Ravelin::VectorNd& q_des);

    // Simple RMRC base frame kinematics
    Ravelin::VectorNd& foot_kinematics(const Ravelin::VectorNd& x,const EndEffector& foot, Ravelin::VectorNd& fk, Ravelin::MatrixNd& gk);
    // N x 3d kinematics
    Ravelin::VectorNd& foot_kinematics(const Ravelin::VectorNd& x,const EndEffector& foot,const boost::shared_ptr<const Ravelin::Pose3d> frame, const Ravelin::Vector3d& goal, Ravelin::VectorNd& fk, Ravelin::MatrixNd& gk);
    // N x 6d Jacobian
    Ravelin::MatrixNd& foot_jacobian(const Ravelin::VectorNd& x,const EndEffector& foot,const boost::shared_ptr<const Ravelin::Pose3d> frame, Ravelin::MatrixNd& gk);
    // N x 6d kinematics
    Ravelin::VectorNd& foot_kinematics(const Ravelin::VectorNd& x,const EndEffector& foot,const boost::shared_ptr<const Ravelin::Pose3d> frame, const Ravelin::SVector6d& goal, Ravelin::VectorNd& fk, Ravelin::MatrixNd& gk);

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
    std::map<std::string, bool>                 active_joints_;


    unsigned                          NUM_FIXED_JOINTS;
    unsigned                          NUM_EEFS;
    unsigned                          NUM_JOINTS;
    unsigned                          NUM_LINKS;

    // Useful Stored Data
    boost::shared_ptr<const Ravelin::Pose3d>   base_horizontal_frame,
                                               base_frame,
                                               environment_frame,
                                               base_link_frame;

    EndEffector       center_of_contact;
    Ravelin::Vector3d center_of_feet_x,
                      center_of_feet_xd;
    const RobotData * data;
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

private:
    RobotData new_data;

    // Import necessary info and then compile model
    void init();

    // set up internal models after kineamtic model is set (called from init)
    void compile();

    void init_end_effector(EndEffector& eef);

};

#endif // ROBOT_H
