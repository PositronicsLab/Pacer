#ifndef ROBOT_H
#define ROBOT_H

#include <project_common.h>
#include <pid.h>

class EndEffector{
public:

    EndEffector(){    }
    EndEffector(Moby::RigidBodyPtr l,Ravelin::Vector3d& o,std::vector<std::string>& jn){
      link = l;
      id = link->id;
      origin = o;
      joint_names_ = jn;
      init();
    }

    // permanent data
    std::string           id;
    Ravelin::Vector3d     origin;
    // Rigid Body (moby data)
    Moby::RigidBodyPtr    link;
    // kinematic chain indexing data
    std::vector<unsigned> chain;
    std::vector<bool> chain_bool;
    // Contact Data

    boost::shared_ptr<const Moby::Event> event;
    Ravelin::Vector3d     point,normal,tan1,tan2;
    std::vector<Ravelin::Vector3d> contacts;
    std::vector<Ravelin::Vector3d> contact_impulses;

    bool                  active;

  private:
    std::vector<std::string> joint_names_;
    void init();
};

class Robot {
  public:
    Robot(){}
    // This is the Simplest Controller (policy is determined within fn)
    virtual Ravelin::VectorNd& control(double dt,
                               const Ravelin::VectorNd& q,
                               const Ravelin::VectorNd& qd,
                               Ravelin::VectorNd& q_des,
                               Ravelin::VectorNd& qd_des,
                               Ravelin::VectorNd& u) = 0;
  void compile();
  void calculate_dyn_properties(Ravelin::MatrixNd& M, Ravelin::VectorNd& fext);
  double calc_energy(Ravelin::VectorNd& v, Ravelin::MatrixNd& M);
  void calc_com();
  std::vector<EndEffector>& get_end_effectors()  { return eefs_; }
  std::vector<std::string>& get_end_effector_names()  { return eef_names_; }
  std::vector<Moby::JointPtr>& get_joints()  { return joints_; }
  std::vector<std::string>& get_joint_names()  { return joint_names_; }
  Moby::RCArticulatedBodyPtr& get_articulated_body()  { return abrobot_; }
  Moby::DynamicBodyPtr& get_dynamic_body()  { return dbrobot_; }
  double friction_estimation(const Ravelin::VectorNd& v, const Ravelin::VectorNd& fext,
                             double dt, const Ravelin::MatrixNd& N,
                             const Ravelin::MatrixNd& ST, const Ravelin::MatrixNd& M,
                             Ravelin::MatrixNd& MU, Ravelin::VectorNd& cf);

  void contact_jacobian_null_stabilizer(const Ravelin::MatrixNd& R, const Ravelin::SVelocityd& vel_des, Ravelin::VectorNd& ufb);
  void zmp_stabilizer(const Ravelin::MatrixNd& R,const Ravelin::Vector2d& zmp_goal, Ravelin::VectorNd& ufb);
  bool inverse_dynamics(const Ravelin::VectorNd& v, const Ravelin::VectorNd& qdd, const Ravelin::MatrixNd& M,
                        const  Ravelin::MatrixNd& N, const Ravelin::MatrixNd& ST, const Ravelin::VectorNd& fext,
                        double h, const Ravelin::MatrixNd& MU, Ravelin::VectorNd& uff, Ravelin::VectorNd& cf_final);
  bool inverse_dynamics(const Ravelin::VectorNd& v, const Ravelin::VectorNd& v_bar, const Ravelin::MatrixNd& M, const Ravelin::VectorNd& fext, double h, const Ravelin::MatrixNd& MU, Ravelin::VectorNd& x, Ravelin::VectorNd& z);
  void calc_contact_jacobians(Ravelin::MatrixNd& N,Ravelin::MatrixNd& ST,Ravelin::MatrixNd& D,Ravelin::MatrixNd& R);
  void calc_base_jacobian(Ravelin::MatrixNd& R);
  void calc_workspace_jacobian(Ravelin::MatrixNd& Rw);
  void RRMC(const EndEffector& foot,const Ravelin::VectorNd& q,const Ravelin::Vector3d& goal,Ravelin::VectorNd& q_des);

  void eef_stiffness_fb(const Ravelin::VectorNd& q_des,const Ravelin::VectorNd&  qd_des,const Ravelin::VectorNd& q,const Ravelin::VectorNd& qd,Ravelin::VectorNd& ufb);
//  Ravelin::VectorNd& kinematics(const Ravelin::VectorNd& x, Ravelin::VectorNd& fk, Ravelin::MatrixNd& gk);
  Ravelin::Vector3d& foot_kinematics(const Ravelin::VectorNd& x,const EndEffector& foot, Ravelin::Vector3d& fk, Ravelin::MatrixNd& gk);
  Ravelin::Vector3d& foot_kinematics(const Ravelin::VectorNd& x,const EndEffector& foot,const boost::shared_ptr<Ravelin::Pose3d> frame, const Ravelin::Vector3d& goal, Ravelin::Vector3d& fk, Ravelin::MatrixNd& gk);
  Ravelin::MatrixNd& foot_jacobian(const Ravelin::Origin3d& x,const EndEffector& foot,const boost::shared_ptr<Ravelin::Pose3d> frame, Ravelin::MatrixNd& gk);
  void update();
  protected:
    // Robot Dynamics Datastructures
    Moby::RCArticulatedBodyPtr        abrobot_;
    Moby::DynamicBodyPtr              dbrobot_;
    std::vector<Moby::JointPtr>       joints_;
    std::vector<Moby::RigidBodyPtr>   links_;

    std::vector<std::string>          joint_names_;

    // End Effector data
    std::vector<std::string>          eef_names_;
    std::vector<EndEffector>          eefs_;

    unsigned                          NUM_FIXED_JOINTS;
    unsigned                          NUM_EEFS;
    unsigned                          NUM_JOINTS;
    unsigned                          NUM_LINKS;

    // wrt: base_frame
    std::map<std::string, Ravelin::Vector3d> eef_origins_;
    // Useful Stored Data
    boost::shared_ptr<Ravelin::Pose3d>   base_horizontal_frame,
                                         base_frame,
                                         environment_frame,
                                         base_link_frame;
    Ravelin::MatrixNd                    base_stability_offset;

    unsigned          NC;
    EndEffector       center_of_contact;
    Ravelin::Vector3d center_of_mass_x,
                      center_of_mass_xd,
                      center_of_mass_xdd,
                      center_of_feet_x,
                      roll_pitch_yaw;
    Ravelin::Vector2d zero_moment_point;
    Ravelin::VectorNd uff, ufb;
    Ravelin::VectorNd qdd_des, qdd;
    Ravelin::MatrixNd N,D,M,ST,R,Rw;
    Ravelin::VectorNd fext;
    Ravelin::VectorNd vel, gc, acc,vel_w;
    // NDFOFS for forces, accel, & velocities
    unsigned                          NDOFS;
    unsigned                          NSPATIAL;
    unsigned                          NEULER;
    unsigned                          NK;

    Ravelin::VectorNd BASE_ORIGIN;

  // PHYSICAL ROBOT LIMITS AND VARIABLES
    std::map<std::string, double>     q0_;
    Ravelin::VectorNd     torque_limits_l,torque_limits_u;

  // All Names, vectors and, maps must be aligned,
  // this function sorts everything to be sure of that
};

extern boost::shared_ptr<Moby::EventDrivenSimulator> sim;
extern boost::shared_ptr<Robot> robot_ptr;

#endif // ROBOT_H
