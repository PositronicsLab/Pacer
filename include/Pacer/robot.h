/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#ifndef ROBOT_H
#define ROBOT_H

#include <Pacer/project_common.h>
namespace Pacer{
class Robot;

/**
 * @brief The EndEffector struct
 *
 * Stores relevent contact and link data for an end effector.
 */
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

class Visualizable{
public:
  double size;
  double shade;
  Ravelin::Vector3d color;
};

class Ray : public Visualizable{
public:
  Ravelin::Vector3d point1,point2;
  Ray(Ravelin::Vector3d p1,Ravelin::Vector3d p2,Ravelin::Vector3d c = Ravelin::Vector3d(1.0,1.0,1.0),double s = 1.0){
    size = s;
    color = c;
  }
};

class Point : public Visualizable{
public:
  Ravelin::Vector3d point;
  Point(Ravelin::Vector3d p,Ravelin::Vector3d c = Ravelin::Vector3d(1.0,1.0,1.0),double s = 1.0){
    size = s;
    color = c;
  }
};

class Pose : public Visualizable{
public:
  double shade;
  Ravelin::Pose3d pose;
  Pose(const Ravelin::Pose3d& p,double sd = 0.5,double s = 1.0){
    shade = sd;
    size = s;
    pose = p;
  }
};
/**
 * @brief The RobotData struct stores const data for use by the controller.
 */
struct RobotData{
  Ravelin::Vector3d zero_moment_point;
  Ravelin::VectorNd q   /*!< position of robot joint dofs, size: [NUM_JOINT_DOFS] */ ,
                    qd  /*!< velocity of robot joint dofs, size: [NUM_JOINT_DOFS] */ ,
                    qdd /*!< acceleration of robot joint dofs, size: [NUM_JOINT_DOFS] */ ;
  Ravelin::VectorNd generalized_q   /*!< generalized coordinates of robot, size: [NUM_JOINT_DOFS ,7 base dofs (3 linear, 4 angular) ] */,
                    generalized_qd  /*!< generalized velocity of robot, size: [NUM_JOINT_DOFS ,6 base dofs (3 linear, 3 angular) ] */,
                    generalized_qdd /*!< generalized acceleration of robot, size: [NUM_JOINT_DOFS ,6 base dofs (3 linear, 3 angular) ] */;
  Ravelin::MatrixNd N /*!< Normal Contact Jacobian, size: [NUM_JOINT_DOFS ,6 base dofs (3 linear, 3 angular) ] x [Num Contacts] */,
                    D /*!< Tangent Contact Jacobian, size: [NUM_JOINT_DOFS ,6 base dofs (3 linear, 3 angular) ] x [Num Contacts x 4] */,
                    M /*!< generalized inertia matrix, size: [NUM_JOINT_DOFS ,6 base dofs (3 linear, 3 angular) ] */,
                    R /*!< Contact Jacobian [N,D] */;
  Ravelin::VectorNd generalized_fext /*!< generalized external forces on robot (excluding contact), size: [NUM_JOINT_DOFS ,6 base dofs (3 linear, 3 angular) ] */;
  Moby::Point3d center_of_mass_x /*!< Global frame coordinates of the center of mass of robot links */;
  Ravelin::Vector3d center_of_mass_xd /*!< Global frame velocity of the center of mass of robot links */,
                    center_of_mass_xdd /*!< Global frame acceleration of the center of mass of robot links */;
  Ravelin::Origin3d roll_pitch_yaw /*!< Roll Pitch Yaw (Tait Bryan) of robot base link */;
};

class Robot /*: public boost::enable_shared_from_this<Robot>*/{
  public:
   std::vector<Visualizable> visualize;

//    boost::shared_ptr<Robot> ptr(){ return shared_from_this(); }
    Robot(){
    }
    Robot(const std::string& model_f, const std::string& vars_f) : robot_model_file(model_f), robot_vars_file(vars_f) {
      std::cout << "initing robot";
      Init();
    }

    /// ---------------------------  Getters  ---------------------------
    std::vector<EndEffector>& get_end_effectors()  { return eefs_; }
    std::map<std::string, EndEffector*>& get_end_effectors_map()  { return eefs_map_; }
    std::vector<std::string>& get_end_effector_names()  { return eef_names_; }
    std::vector<Moby::JointPtr>& get_joints()  { return joints_; }
    std::vector<Moby::RigidBodyPtr>& get_links()  { return links_; }

    std::vector<std::string>& get_joint_names()  { return joint_names_; }

    /// Return Robot's internal model
    Moby::RCArticulatedBodyPtr get_articulated_body()  { return abrobot_; }
    Moby::DynamicBodyPtr get_dynamic_body()  { return dbrobot_; }

    boost::shared_ptr<const Ravelin::Pose3d> get_base_link_frame(){return base_link_frame;}
    boost::shared_ptr<const RobotData> get_robot_data(){return data;}

    std::map<int, int>& get_joint_map()  { return joint_map_; }
    std::map<std::string, double>& get_q0()  { return q0_; }
    std::map<std::string, bool>& get_active_joints()  { return active_joints_; }

    /// Reset eefs_ data
    void reset_contact();

    /// Function Warm Starts
    static boost::shared_ptr<const RobotData> gen_vars_from_model(
        const std::map<std::string, double>& q,
        const std::map<std::string, double>& qd,
        boost::shared_ptr<const Ravelin::Pose3d> base_x,
        const Ravelin::SVector6d &base_xd,
        boost::shared_ptr<Robot>& robot);

    std::string robot_vars_file, robot_model_file;
  protected:
    /**
     * @brief Update robot internal model using 'generalized' (minimal) parameters
     * @param generalized_q_in
     * @param generalized_qd_in
     * @param generalized_qdd_in
     * @param generalized_fext_in
     */
    void update(
        const Ravelin::VectorNd& generalized_q_in,
        const Ravelin::VectorNd& generalized_qd_in,
        const Ravelin::VectorNd& generalized_qdd_in,
        const Ravelin::VectorNd& generalized_fext_in);

    /// Update poses of robot based on  currently set 'generalized' parameters
    void update_poses();

    /// Get M and fext from model internal to plugin NOT simulator
    void calculate_dyn_properties(Ravelin::MatrixNd &M, Ravelin::VectorNd &fext);

    /// Calcultate energy of robot
    double calc_energy(const Ravelin::VectorNd& v, const Ravelin::MatrixNd& M) const;

    /// Calculate Center of mass(x,xd,xdd,zmp)
    void calc_com();

    /// Set Plugin internal model to input state
    void set_model_state(const Ravelin::VectorNd& q,const Ravelin::VectorNd& qd = Ravelin::VectorNd::zero(0));

    /// Calculate N (normal), D (positive tangent), R ([N D]) contact jacobians
    void calc_contact_jacobians(Ravelin::MatrixNd& N,Ravelin::MatrixNd& D,Ravelin::MatrixNd& R);

    /// Resolved Motion Rate control (iterative inverse kinematics)
    /// iterative inverse kinematics for a 3d (linear) goal
    void RMRC(const EndEffector& foot,const Ravelin::VectorNd& q,const Ravelin::Vector3d& goal,Ravelin::VectorNd& q_des);

    /// Resolved Motion Rate control (iterative inverse kinematics)
    /// iterative inverse kinematics for a 6d (linear and angular) goal
    void RMRC(const EndEffector& foot,const Ravelin::VectorNd& q,const Ravelin::SVector6d& goal,Ravelin::VectorNd& q_des);

    /// N x (3/6)d kinematics for RMRC
    Ravelin::VectorNd& foot_kinematics(const Ravelin::VectorNd& x,const EndEffector& foot, Ravelin::VectorNd& fk, Ravelin::MatrixNd& gk);

    /// N x 3d kinematics
    Ravelin::VectorNd& foot_kinematics(const Ravelin::VectorNd& x,const EndEffector& foot,const boost::shared_ptr<const Ravelin::Pose3d> frame, const Ravelin::Vector3d& goal, Ravelin::VectorNd& fk, Ravelin::MatrixNd& gk);

    /// N x 6d Jacobian
    Ravelin::MatrixNd& foot_jacobian(const Ravelin::VectorNd& x,const EndEffector& foot,const boost::shared_ptr<const Ravelin::Pose3d> frame, Ravelin::MatrixNd& gk);

    /// N x 6d kinematics
    Ravelin::VectorNd& foot_kinematics(const Ravelin::VectorNd& x,const EndEffector& foot,const boost::shared_ptr<const Ravelin::Pose3d> frame, const Ravelin::SVector6d& goal, Ravelin::VectorNd& fk, Ravelin::MatrixNd& gk);

  protected:
    Moby::RCArticulatedBodyPtr        abrobot_;
    Moby::DynamicBodyPtr              dbrobot_;
    std::vector<Moby::JointPtr>       joints_;
    std::vector<Moby::RigidBodyPtr>   links_;

    std::vector<std::string>          joint_names_;

    // End Effector data
    std::vector<std::string>          eef_names_;
    std::vector<EndEffector>          eefs_;
    std::map<std::string,EndEffector*> eefs_map_;
    std::map<std::string, bool>       active_joints_;


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

    boost::shared_ptr<const RobotData> data;
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
    boost::shared_ptr<RobotData> new_data;

    // Import necessary info and then compile model
    void Init();

    // set up internal models after kineamtic model is set (called from init)
    void compile();

    void init_end_effector(EndEffector& eef);

};
}
#endif // ROBOT_H
