#ifndef ROBOT_H
#define ROBOT_H

#include <project_common.h>
#include <pid.h>

class EndEffector{
public:
  static std::vector<std::string> joint_names_;

    EndEffector(){
      init();
    }
    EndEffector(Moby::RigidBodyPtr l,Ravelin::Vector3d o){
      link = l;
      id = link->id;
      origin = o;
      init();
    }

    // permanent data
    std::string           id;
    Ravelin::Vector3d     origin;
    // Rigid Body (moby data)
    Moby::RigidBodyPtr    link;
    // kinematic chain indexing data
    std::vector<unsigned> chain;
    // Contact Data

    Ravelin::Vector3d     point;
    Ravelin::Vector3d     normal; //(pointing away from the ground)
    Ravelin::Vector3d     impulse;
    bool                  active;

  private:
    void init();
};

class Robot {
  public:
    Robot();
    // This is the Simplest Controller (policy is determined within fn)
    virtual Ravelin::VectorNd& control(const Ravelin::VectorNd& q,
                                       const Ravelin::VectorNd& qd,
                                       Ravelin::VectorNd& q_des,
                                       Ravelin::VectorNd& qd_des,
                                       Ravelin::VectorNd& u) = 0;
  void compile();
  void calculate_dyn_properties(Ravelin::MatrixNd& M, Ravelin::VectorNd& fext);
  double calc_energy(Ravelin::VectorNd& v, Ravelin::MatrixNd& M);
  void calc_com( Ravelin::Vector3d& weighted_com, Ravelin::Vector3d& com_acc);
  std::vector<EndEffector>& get_end_effectors()  { return eefs_; }
  std::vector<std::string>& get_end_effector_names()  { return eef_names_; }
  std::vector<Moby::JointPtr>& get_joints()  { return joints_; }
  Moby::RCArticulatedBodyPtr& get_articulated_body()  { return abrobot_; }
  Moby::DynamicBodyPtr& get_dynamic_body()  { return dbrobot_; }

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
    // NDFOFS for forces, accel, & velocities
    unsigned                          NDOFS;
    unsigned                          NSPATIAL;
    unsigned                          NEULER;
    unsigned                          NK;

  // PHYSICAL ROBOT LIMITS AND VARIABLES
    std::map<std::string, double>     q0_;
    std::map<std::string, double>     torque_limits_;
    std::map<std::string, Gains>      gains_;

  // All Names, vectors and, maps must be aligned,
  // this function sorts everything to be sure of that
};

#endif // ROBOT_H
