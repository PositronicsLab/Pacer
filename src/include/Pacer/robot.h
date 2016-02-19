/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#ifndef ROBOT_H
#define ROBOT_H

#include <Pacer/Log.h>

#include <Ravelin/RCArticulatedBodyd.h>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/icl/type_traits/to_string.hpp>

#include <Pacer/output.h>

#include <numeric>
#include <math.h>
#include <cmath>
#include <sys/types.h>
#include <sys/times.h>
#include <boost/any.hpp>
#ifdef USE_THREADS
#include <pthread.h>
#endif

namespace Pacer{
  
  const boost::shared_ptr<const Ravelin::Pose3d> GLOBAL;
  const double NEAR_ZERO = std::sqrt(std::numeric_limits<double>::epsilon());
  const Ravelin::Matrix3d ZEROS_3x3 = Ravelin::Matrix3d::zero();
  const Ravelin::Matrix3d IDENTITY_3x3 = Ravelin::Matrix3d::identity();
  const Ravelin::VectorNd EMPTY_VEC(0);
  
  class Robot;
  
  static const unsigned NSPATIAL = 6;
  static const unsigned NEULER = 7;
  
  class Robot{
  public:
    
    Robot(){
    }
    
  protected:
    /// --------------------  Data Storage  -------------------- ///
  private:
    template<typename T>
    struct is_pointer { static const bool value = false; };
    
    template<typename T>
    struct is_pointer<T*> { static const bool value = true; };
    
    // Map for storing arbitrary data
    std::map< std::string , boost::any > _data_map;
#ifdef USE_THREADS
    pthread_mutex_t _data_map_mutex;
#endif
    
    // Returns 'true' if new key was created in map
    bool set_data_internal(const std::string& n, boost::any to_append);
    // Returns 'true' if new key was created in map
    bool get_data_internal(const std::string& n, boost::any& operand);
    
  public:
    
    // Returns 'true' if new key was created in map
    template<class T>
    bool set_data(std::string n, const T& v){
      OUT_LOG(logINFO) << "Set: " << n << " <-- " << v;
      if(is_pointer<T>::value){
        throw std::runtime_error("Can't save pointers! : " + n);
      }
      return set_data_internal(n,v);
    }
    
    void remove_data(std::string n);

    template<class T>
    T get_data(std::string n){
      T v;
      
      using boost::any_cast;
      boost::any operand;
      
      if(get_data_internal(n,operand)){
        try
        {
          v = any_cast<T>(operand);
        }
        catch(const boost::bad_any_cast &)
        {
          throw std::runtime_error("Variable: \"" + n + "\" was requested as '" + typeid(T).name() + "' but is actually '" + operand.type().name() + "'");
        }
        OUT_LOG(logINFO) << "Get: " << n << " ("<< operand.type().name() <<") --> " << v;
        return v;
      }
      
      // else
      throw std::runtime_error("Variable: \"" + n + "\" not found in data!");
    }

    
    /// @brief Get data we're not sure exists.
    /// Return false and do nothing to data if it doesnt exist
    template<class T>
    bool get_data(std::string n,T& val){
      try {
        val = get_data<T>(n);
        return true;
      } catch(std::runtime_error& e) {
        OUT_LOG(logDEBUG) << e.what() << std::endl;
        return false;
      }
    }
    
    
    /// ---------------------------  Getters  ---------------------------
  public:
    struct contact_t{
      std::string id;
      Ravelin::Vector3d point;
      Ravelin::Vector3d normal, tangent;
      Ravelin::Vector3d impulse;
      double mu_coulomb;
      double mu_viscous;
      double restitution;
      bool compliant;
    };
    
    
  private:
    /**
     * @brief The end_effector_t struct
     *
     * Stores relevent contact and link data for an end effector.
     */
    struct end_effector_t{
      // permanent data
      std::string            id;
      // Foot link pointer
      boost::shared_ptr<Ravelin::RigidBodyd>     link;
      std::vector<boost::shared_ptr<Ravelin::Jointd> >        chain_joints;
      std::vector<boost::shared_ptr<Ravelin::RigidBodyd> >    chain_links;
      // kinematic chain indexing generalized coordinates
      std::vector<unsigned> chain;
      std::vector<bool>     chain_bool;
      
      // For locomotion, this informs us if this link should be used to calculate a jocobian
      bool                   active;
      bool                   stance;
    };
    
    std::map<std::string,std::vector< boost::shared_ptr<const contact_t> > > _id_contacts_map;
    std::map<std::string,boost::shared_ptr<end_effector_t> > _id_end_effector_map;
    
  public:
    
    /// @brief Returns true if the link with name 'id' is in the list of designanted end effectors.
    bool is_end_effector(const std::string& id);
    
    /// @brief Returns a vector of link names for links designanted end effectors.
    std::vector<std::string> get_end_effector_names();
    
    /// @brief Returns a 6xN jacobian for the origin of link with name 'id' for the robot in position q.
    Ravelin::MatrixNd calc_link_jacobian(const Ravelin::VectorNd& q, const std::string& id);
    
    /// @brief Returns key,value pairs (link id, reference to Ravelin RigidBodyd) for all robot links in the form of a map.
    const std::map<std::string,boost::shared_ptr<Ravelin::RigidBodyd> >& get_links(){
      return _id_link_map;
    }
    
    /// @brief Returns a reference to the Ravelin RigidBodyd structure used by Robot for link with name 'id'
    const boost::shared_ptr<Ravelin::RigidBodyd> get_link(const std::string& id){
      return _id_link_map[id];
    }
    
    
    /// @brief This is a 'misc_sensor' operation.  Adds a contact with listed parameter information to list of known contacts.
    void add_contact(std::string& id,
                     Ravelin::Vector3d point,
                     Ravelin::Vector3d normal,
                     Ravelin::Vector3d tangent,
                     Ravelin::Vector3d impulse = Ravelin::Vector3d(),
                     double mu_coulomb = 0,double mu_viscous = 0,double restitution = 0, bool compliant = false);
    
    boost::shared_ptr<contact_t> create_contact(std::string& id,
                                                Ravelin::Vector3d point,
                                                Ravelin::Vector3d normal,
                                                Ravelin::Vector3d tangent,
                                                Ravelin::Vector3d impulse = Ravelin::Vector3d(),
                                                double mu_coulomb = 0,double mu_viscous = 0,double restitution = 0, bool compliant = false);
    
    /// @brief This is a 'misc_sensor' operation.  Adds contact 'c' to list of known contacts on the robot
    void add_contact(boost::shared_ptr<const contact_t>& c);
    
    /// @brief collects all contact information into (link name, contact) key,value pairs in map 'id_contacts_map'
    void get_contacts(std::map<std::string,std::vector< boost::shared_ptr<const contact_t> > >& id_contacts_map);
    
    /// @brief collects all contact information into vector 'contacts'
    int get_all_contacts(std::vector< boost::shared_ptr<const contact_t> >& contacts);
    
    /// @brief collects all contact information for link name: 'link_id' into vector 'contacts'
    int get_link_contacts(const std::string& link_id, std::vector< boost::shared_ptr<const contact_t> >& contacts);
    
    /// @brief collects all contact information for link names listed in 'link_ids' into vector 'contacts'
    void get_link_contacts(const std::vector<std::string> link_ids,std::vector< boost::shared_ptr<const contact_t> >& contacts);
    
    /// @brief This is a 'clean_up' operation. Clears: _id_contacts_map
    void reset_contact();
    
  public:
    enum unit_e  {       //  ang  |  lin
       // ---- SET OUTSIDE CONTROL LOOP ---- //
      misc_sensor=0,           //  SENSORS
      position=1,           //  rad  |   m
      velocity=2,           // rad/s |  m/s
      acceleration=3,       // rad/ss|  m/ss
      load=4,               //  N.m  |   N
      // ---- SET IN CONROLLER ---- //
      misc_planner=5,
      position_goal=6,      //  rad  |   m
      velocity_goal=7,      // rad/s |  m/s
      acceleration_goal=8,  // rad/ss|  m/ss
      misc_controller=9,
      load_goal=10,// TORQUE//  N.m  |   N
      initialization=11,
      clean_up=12
    };
    
    const char * unit_enum_string(const unit_e& e){
      int i = static_cast<int>(e);
      return (const char *[]) {
        "misc_sensor",
        "position",
        "velocity",
        "acceleration",
        "load",
        "misc_planner",
        "position_goal",
        "velocity_goal",
        "acceleration_goal",
        "misc_controller",
        "load_goal",
        "initialization",
        "clean_up"
      }[i];
    }
    
  private:
    std::map<unit_e , std::map<std::string, Ravelin::VectorNd > > _state;
    std::map<unit_e , Ravelin::VectorNd> _base_state;
    std::map<unit_e , std::map<std::string, Ravelin::Origin3d > > _end_effector_state;
    std::map<std::string, bool > _end_effector_is_set;
    
#ifdef USE_THREADS
    pthread_mutex_t _state_mutex;
    pthread_mutex_t _base_state_mutex;
    pthread_mutex_t _end_effector_state_mutex;
#endif
    // TODO: Populate these value in Robot::compile()
    
    // JOINT_NAME --> {gcoord_dof1,...}
    std::map<std::string,std::vector<int> > _id_dof_coord_map;
    
    // gcoord --> (JOINT_NAME,dof)
    std::map<int,std::pair<std::string, int> > _coord_id_map;

  protected:
    virtual bool check_phase_internal(const unit_e& u) = 0;

  public:
    int get_joint_dofs(const std::string& id){ return _id_dof_coord_map[id].size(); }
    
    //////////////////////////////////////////////////////
    /// ------------ GET/SET JOINT value  ------------ ///
    
    double get_joint_value(const std::string& id, unit_e u, int dof);
    
    Ravelin::VectorNd get_joint_value(const std::string& id, unit_e u);
    
    void get_joint_value(const std::string& id, unit_e u, Ravelin::VectorNd& dof_val);
    
    void get_joint_value(const std::string& id, unit_e u,std::vector<double>& dof_val);
    
    void set_joint_value(const std::string& id, unit_e u, int dof, double val);
    
    void set_joint_value(const std::string& id, unit_e u, const Ravelin::VectorNd& dof_val);
    
    void set_joint_value(const std::string& id, unit_e u, const std::vector<double>& dof_val);
    
    void get_joint_value(unit_e u, std::map<std::string,std::vector<double> >& id_dof_val_map);
    
    void get_joint_value(unit_e u, std::map<std::string,Ravelin::VectorNd >& id_dof_val_map);
    
    void set_joint_value(unit_e u,const std::map<std::string,std::vector<double> >& id_dof_val_map);
    
    void set_joint_value(unit_e u,const std::map<std::string,Ravelin::VectorNd >& id_dof_val_map);
    /// ------------- GENERALIZED VECTOR CONVERSIONS  ------------- ///
    
    
    void convert_to_generalized(const std::map<std::string,std::vector<double> >& id_dof_val_map, Ravelin::VectorNd& generalized_vec){
      generalized_vec.set_zero(NUM_JOINT_DOFS);
      std::map<std::string,std::vector<double> >::const_iterator it;
      for(it=id_dof_val_map.begin();it!=id_dof_val_map.end();it++){
        const std::vector<int>& dof = _id_dof_coord_map[(*it).first];
        const std::vector<double>& dof_val = (*it).second;
        if(dof.size() != dof_val.size())
          throw std::runtime_error("Missized dofs in joint "+(*it).first+": internal="+boost::icl::to_string<double>::apply(dof.size())+" , provided="+boost::icl::to_string<double>::apply(dof_val.size()));
        for(int j=0;j<(*it).second.size();j++){
          generalized_vec[dof[j]] = dof_val[j];
        }
      }
    }
    
    template <typename T>
    void convert_to_generalized(const std::map<std::string,std::vector<T> >& id_dof_val_map, std::vector<T>& generalized_vec){
      typename std::map<std::string,std::vector<T> >::const_iterator it;
      generalized_vec.resize(NUM_JOINT_DOFS);
      it = id_dof_val_map.begin();
      for(it=id_dof_val_map.begin();it!=id_dof_val_map.end();it++){
        const std::vector<int>& dof = _id_dof_coord_map[(*it).first];
        const std::vector<T>& dof_val = (*it).second;
        if(dof.size() != dof_val.size())
          throw std::runtime_error("Missized dofs in joint "+(*it).first+": internal="+boost::icl::to_string<double>::apply(dof.size())+" , provided="+boost::icl::to_string<double>::apply(dof_val.size()));
        for(int j=0;j<(*it).second.size();j++){
          generalized_vec[dof[j]] = dof_val[j];
        }
      }
    }
    
    
    void convert_to_generalized(const std::map<std::string,Ravelin::VectorNd >& id_dof_val_map, Ravelin::VectorNd& generalized_vec){
      generalized_vec.set_zero(NUM_JOINT_DOFS);
      std::map<std::string,Ravelin::VectorNd >::const_iterator it;
      for(it=id_dof_val_map.begin();it!=id_dof_val_map.end();it++){
        const std::vector<int>& dof = _id_dof_coord_map[(*it).first];
        const Ravelin::VectorNd& dof_val = (*it).second;
        if(dof.size() != dof_val.rows())
          throw std::runtime_error("Missized dofs in joint "+(*it).first+": internal="+boost::icl::to_string<double>::apply(dof.size())+" , provided="+boost::icl::to_string<double>::apply(dof_val.rows()));
        for(int j=0;j<(*it).second.rows();j++){
          generalized_vec[dof[j]] = dof_val[j];
        }
      }
    }
    
    template <typename K, typename V>
    std::vector<K> get_map_keys(const std::map<K,V>& m){
      std::vector<K> v;
      typedef const std::pair<K,V> map_pair;
      BOOST_FOREACH(map_pair me, m) {
        v.push_back(me.first);
      }
      return v;
    }
    
    template <typename T>
    std::map<std::string,std::vector<T> > make_id_value_map(){
      std::map<std::string,std::vector<T> > id_dof_val_map;
      std::map<std::string,std::vector<int> >::iterator it;
      for(it=_id_dof_coord_map.begin();it!=_id_dof_coord_map.end();it++){
        const std::vector<int>& dof = (*it).second;
        std::vector<T>& dof_val = id_dof_val_map[(*it).first];
        dof_val.resize(dof.size());
      }
      return id_dof_val_map;
    }
    
    template <typename T>
    void convert_from_generalized(const std::vector<T>& generalized_vec, std::map<std::string,std::vector<T> >& id_dof_val_map){
      if(generalized_vec.size() != NUM_JOINT_DOFS)
        throw std::runtime_error("Missized generalized vector: internal="+boost::icl::to_string<double>::apply(NUM_JOINT_DOFS)+" , provided="+boost::icl::to_string<double>::apply(generalized_vec.size()));
      
      //      std::map<std::string,std::vector<int> >::iterator it;
      std::vector<std::string> keys = get_map_keys(_id_dof_coord_map);
      
      for(int i=0;i<keys.size();i++){
        const std::vector<int>& dof = _id_dof_coord_map[keys[i]];
        std::vector<T>& dof_val = id_dof_val_map[keys[i]];
        dof_val.resize(dof.size());
        for(int j=0;j<dof.size();j++){
          dof_val[j] = generalized_vec[dof[j]];
        }
      }
    }
    
    
    void convert_from_generalized(const Ravelin::VectorNd& generalized_vec, std::map<std::string,std::vector<double> >& id_dof_val_map){
      if(generalized_vec.rows() != NUM_JOINT_DOFS)
        throw std::runtime_error("Missized generalized vector: internal="+boost::icl::to_string<double>::apply(NUM_JOINT_DOFS)+" , provided="+boost::icl::to_string<double>::apply(generalized_vec.rows()));
      
      //      std::map<std::string,std::vector<int> >::iterator it;
      std::vector<std::string> keys = get_map_keys(_id_dof_coord_map);
      
      for(int i=0;i<keys.size();i++){
        const std::vector<int>& dof = _id_dof_coord_map[keys[i]];
        std::vector<double>& dof_val = id_dof_val_map[keys[i]];
        dof_val.resize(dof.size());
        for(int j=0;j<dof.size();j++){
          dof_val[j] = generalized_vec[dof[j]];
        }
      }
    }
    
    void convert_from_generalized(const Ravelin::VectorNd& generalized_vec, std::map<std::string,Ravelin::VectorNd >& id_dof_val_map){
      if(generalized_vec.rows() != NUM_JOINT_DOFS)
        throw std::runtime_error("Missized generalized vector: internal="+boost::icl::to_string<double>::apply(NUM_JOINT_DOFS)+" , provided="+boost::icl::to_string<double>::apply(generalized_vec.rows()));
      
      //      std::map<std::string,std::vector<int> >::iterator it;
      std::vector<std::string> keys = get_map_keys(_id_dof_coord_map);
      
      for(int i=0;i<keys.size();i++){
        const std::vector<int>& dof = _id_dof_coord_map[keys[i]];
        Ravelin::VectorNd& dof_val = id_dof_val_map[keys[i]];
        dof_val.resize(dof.size());
        for(int j=0;j<dof.size();j++){
          dof_val[j] = generalized_vec[dof[j]];
        }
      }
    }
    
    /// ------------ GET/SET JOINT GENERALIZED value  ------------ ///
    
    void set_joint_generalized_value(unit_e u, const Ravelin::VectorNd& generalized_vec);
    
    void get_joint_generalized_value(unit_e u, Ravelin::VectorNd& generalized_vec);
    
    Ravelin::VectorNd get_joint_generalized_value(unit_e u);
    /// With Base
    void set_generalized_value(unit_e u,const Ravelin::VectorNd& generalized_vec);
    /// With Base
    void get_generalized_value(unit_e u, Ravelin::VectorNd& generalized_vec);
    /// With Base
    Ravelin::VectorNd get_generalized_value(unit_e u);
    
    void set_base_value(unit_e u,const Ravelin::VectorNd& vec);
    
    void get_base_value(unit_e u, Ravelin::VectorNd& vec);
    
    Ravelin::VectorNd get_base_value(unit_e u);
    
    void set_end_effector_value(const std::string& id, unit_e u, const Ravelin::Origin3d& val);
    
    Ravelin::Origin3d& get_end_effector_value(const std::string& id, unit_e u, Ravelin::Origin3d& val);
    
    Ravelin::Origin3d get_end_effector_value(const std::string& id, unit_e u);
    void set_end_effector_value(unit_e u, const std::map<std::string,Ravelin::Origin3d>& val);
    
    std::map<std::string,Ravelin::Origin3d>& get_end_effector_value(unit_e u, std::map<std::string,Ravelin::Origin3d>& val);
    std::map<std::string,Ravelin::Origin3d> get_end_effector_value(unit_e u);
    
    const std::vector<std::string>& get_end_effector_ids(){
      return _end_effector_ids;
    }
    
    void init_state();
    
    void reset_state();
  protected:
    
    /// @brief Sets up robot after construction and parameter import
    void init_robot();
    
    /// @brief Pulls data from _state and updates robot data
    void update();
    
  private:
    /// @brief Update poses of robot based on  currently set 'generalized' parameters
    void update_poses();
    
  public:
    /// @brief Calcultate energy of robot
    double calc_energy(const Ravelin::VectorNd& v, const Ravelin::MatrixNd& M) const;
    
    /// @brief Calculate Center of mass(x,xd,xdd,zmp)
    void calc_com();
    
    /// @brief Set Plugin internal model to input state
    void set_model_state(const Ravelin::VectorNd& q,const Ravelin::VectorNd& qd = Ravelin::VectorNd::zero(0));
    
    /// @brief Calculate N (normal), S (1st tangent), T (2nd tangent) contact jacobians
    void calc_contact_jacobians(const Ravelin::VectorNd& q, std::vector<boost::shared_ptr<const contact_t> > c ,Ravelin::MatrixNd& N,Ravelin::MatrixNd& S,Ravelin::MatrixNd& T);
    
    /// @brief Calculate N (normal), S (1st tangent), T (2nd tangent) contact jacobians
    void calc_contact_jacobians(const Ravelin::VectorNd& q, std::vector<boost::shared_ptr<contact_t> > c ,Ravelin::MatrixNd& N,Ravelin::MatrixNd& S,Ravelin::MatrixNd& T);
    
    /// @brief Calculate 6x(N+6) jacobian for point(in frame) on link at state q
    Ravelin::MatrixNd calc_jacobian(const Ravelin::VectorNd& q, const std::string& link, Ravelin::Origin3d point);
    
    /// @brief Resolved Motion Rate control (iterative inverse kinematics)
    /// iterative inverse kinematics for a 3d (linear) goal
    void RMRC(const end_effector_t& foot,const Ravelin::VectorNd& q,const Ravelin::Origin3d& goal,Ravelin::VectorNd& q_des, double TOL = 1e-4);
    
    /// @brief Resolved Motion Rate control (iterative inverse kinematics)
    /// iterative inverse kinematics for a 6d (linear and angular) goal
    void RMRC(const end_effector_t& foot,const Ravelin::VectorNd& q,const Ravelin::VectorNd& goal,Ravelin::VectorNd& q_des, double TOL = 1e-4);
    
    /// @brief N x 3d kinematics
    Ravelin::VectorNd& link_kinematics(const Ravelin::VectorNd& x,const end_effector_t& foot,const boost::shared_ptr<const Ravelin::Pose3d> frame, const Ravelin::Origin3d& goal, Ravelin::VectorNd& fk, Ravelin::MatrixNd& gk);
    
    /// @brief N x 6d Jacobian
    Ravelin::MatrixNd& link_jacobian(const Ravelin::VectorNd& x,const end_effector_t& foot,const boost::shared_ptr<const Ravelin::Pose3d> frame, Ravelin::MatrixNd& gk);
    
    /// @brief N x 6d kinematics
    Ravelin::VectorNd& link_kinematics(const Ravelin::VectorNd& x,const end_effector_t& foot,const boost::shared_ptr<const Ravelin::Pose3d> frame, const Ravelin::VectorNd& goal, Ravelin::VectorNd& fk, Ravelin::MatrixNd& gk);
    
    Ravelin::VectorNd& dist_to_goal(const Ravelin::VectorNd& x,const end_effector_t& foot,const boost::shared_ptr<const Ravelin::Pose3d> frame, const Ravelin::Origin3d& goal, Ravelin::VectorNd& dist);
    
    
    void end_effector_inverse_kinematics(
                                         const std::vector<std::string>& foot_id,
                                         const std::vector<Ravelin::Origin3d>& foot_pos,
                                         const std::vector<Ravelin::Origin3d>& foot_vel,
                                         const std::vector<Ravelin::Origin3d>& foot_acc,
                                         const Ravelin::VectorNd& q,
                                         Ravelin::VectorNd& q_des,
                                         Ravelin::VectorNd& qd_des,
                                         Ravelin::VectorNd& qdd_des, double TOL = 1e-4);
    
    void calc_generalized_inertia(const Ravelin::VectorNd& q, Ravelin::MatrixNd& M);
    
    const boost::shared_ptr<Ravelin::RigidBodyd> get_root_link(){return _root_link;}
    
    int joint_dofs(){return NUM_JOINT_DOFS;}
    
    boost::shared_ptr<Ravelin::ArticulatedBodyd>& get_abrobot(){
      return _abrobot;
    };
    
  protected:
    bool fixed_base(){return FIXED_BASE;}
    bool floating_base(){return !FIXED_BASE;}

  private:
    /// @brief N x (3/6)d kinematics for RMRC
    Ravelin::VectorNd& contact_kinematics(const Ravelin::VectorNd& x,const end_effector_t& foot, Ravelin::VectorNd& fk, Ravelin::MatrixNd& gk);
    
    boost::shared_ptr<Ravelin::ArticulatedBodyd>        _abrobot;
    
    std::map<std::string,boost::shared_ptr<Ravelin::RigidBodyd> > _id_link_map;
//    std::map<std::string,boost::shared_ptr<Ravelin::RigidBodyd> > _id_end_effector_map;
    boost::shared_ptr<Ravelin::RigidBodyd> _root_link;
    std::map<std::string,boost::shared_ptr<Ravelin::Jointd> > _id_joint_map;
    std::vector<std::string> _link_ids;
    std::vector<std::string> _joint_ids;
    std::vector<std::string> _end_effector_ids;
    
    // NDFOFS for forces, accel, & velocities
    unsigned NDOFS, NUM_JOINT_DOFS;
    
    bool FIXED_BASE = false;

    std::vector<bool> _disabled_dofs;
    
    /// set up internal models after kineamtic model is set (called from init)
    void compile();
  };
}

#endif // ROBOT_H
