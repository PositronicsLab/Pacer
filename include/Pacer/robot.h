/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#ifndef ROBOT_H
#define ROBOT_H

#include <Pacer/project_common.h>
#include <Pacer/Visualizable.h>
namespace Pacer{

class Robot {

  public:
    std::string VARS_FILE("vars.h");

    Robot(){
      Utility::load_variables(VARS_FILE);
      init_robot();
    }

    /// Data storage
  private:
    std::map<std::string,boost::shared_ptr<void> > _data_map;
    std::mutex _data_map_mutex;
   
  public:   
	template<class T>
    void set_data(std::string& n, T& v){
	  _data_map_mutex.lock();
      _data_map[n] = boost::shared_ptr<void>(new T(v));
      _data_map_mutex.unlock();
    }
    
     template<class T>
     T get_data(std::string& n){
      std::map<std::string,boost::shared_ptr<void> >::iterator it;
      _data_map_mutex.lock();
      it = _data_map.find(n);
      _data_map_mutex.unlock();
      if(it != _data_map.end())
        return T(*(it->second.get()));
      else
        std::runtime_error("Variable: \"" + n + "\" not found in data!")
      return true;
    }


    /// ---------------------------  Getters  ---------------------------
  public:
    struct contact_s{
      std::string link_id;
      Ravelin::Vector3d point;
      Ravelin::Vector3d normal;
      Ravelin::Vector3d impulse;
      double mu_coulomb;
      double mu_viscous;
      double restitution;
    };

   /**
	 * @brief The EndEffector struct
	 *
	 * Stores relevent contact and link data for an end effector.
	 */
	struct end_effector_s{
		// permanent data
		std::string            id;
		// Foot link pointer
		Moby::RigidBodyPtr     link;
		// End effector location on link;
		Ravelin::Vector3d      origin;
		// kinematic chain indexing generalized coordinates
		std::vector<unsigned> chain;
		std::vector<bool>      chain_bool;

		// For locomotion, this informs us if this link should be used to calculate a jocobian
		bool                   active;
		bool                   stance;
		
		// Contact Data
		std::vector<boost::shared_ptr<const contact_s> > contacts;
	};
	
	
	private:
		std::map<std::string,std::vector< boost::shared_ptr<const contact_s> > > _link_contacts_map
		std::vector<boost::shared_ptr<const contact_s> > _contacts
		std::map<std::string,boost::shared_ptr<end_effector_s> > _end_effector_map;
	
	public:
    void add_contact(
        std::string& id;
        const Ravelin::Vector3d point,
        const Ravelin::Vector3d normal,
        const Ravelin::Vector3d impulse = Ravelin::Vector3d(),
        double mu_coulomb = 0,double mu_viscous = 0,double restitution = 0)
    {
      boost::shared_ptr<contact_s> c(new contact_s);
      c->link_id    = id; 
      c->point      = point; 
      c->normal     = normal; 
      c->impulse    = impulse; 
      c->mu_coulomb = mu_coulomb; 
      c->mu_viscous = mu_viscous;
      c->restitution= restitution;

      _link_contacts_map[c->link_id] = c;
      _contacts.push_back(c);
      
      // This will fail if there is no end effector by this name
      
	  std::map<std::string,boost::shared_ptr<end_effector_s> > it = _end_effector_map.find(c->link_id);
      if(it != _end_effector_map.end())
		it->second->contacts.push_back(c);      
    }
    
    const std::vector<boost::shared_ptr<const contact_s> >& get_contacts(){
      return _contacts;
	}

	const std::vector<boost::shared_ptr<const contact_s> >& get_contacts(){
	  return _contacts;
	}

    std::vector<std::string>& get_end_effector_names()  { return eef_names_; }
    std::vector<Moby::JointPtr>& get_joints()  { return joints_; }
    std::vector<Moby::RigidBodyPtr>& get_links()  { return links_; }
    std::vector<std::string>& get_joint_names()  { return joint_names_; }

    /// Return Robot's internal model
    Moby::RCArticulatedBodyPtr get_articulated_body()  { return abrobot_; }
    Moby::DynamicBodyPtr       get_dynamic_body()      { return dbrobot_; }

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
   

    enum unit{        //  REV  |  PRIS
      position_e,     //  rad  |   m 
      velocity_e,     // rad/s |  m/s
      acceleration_e, // rad/ss|  m/ss
      torque_e        //  N.m  |   N
    }
    
    double get_joint_param(std::string id,unit u, int dof = 0)
    {
      switch(u){
        case position_e:
          return q[_id_coord_map[id]+dof];
          break;
        case velocity_e:
          return qd[_id_coord_map[id]+dof];
          break;
        case acceleration_e:
          return qdd[_id_coord_map[id]+dof];
          break;
        case torque_e:
          return tau[_id_coord_map[id]+dof];
          break;
        default: break;
      }
    }

  protected:

    /// @brief Pulls data from q_joints,qd_joints,qdd_joints and sets up generalized vector
    void update(); // SRZ: TODO

  private:

    // TODO: Populate these values in compile
    std::map<std::string,int> _id_coord_map;
    std::map<int,std::string> _coord_id_map;

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
  
    // Import necessary info and then compile model
    void init_robot();

  public:

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

    // set up internal models after kineamtic model is set (called from init)
    void compile();

    void init_end_effector(EndEffector& eef);
};
}
#endif // ROBOT_H
