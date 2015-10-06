/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#ifndef ROBOT_H
#define ROBOT_H

#include <Ravelin/RCArticulatedBodyd.h>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/icl/type_traits/to_string.hpp>

#include <Pacer/output.h>

#include <numeric>
#include <math.h>
#include <cmath>
#include <boost/thread/mutex.hpp>
#include <sys/types.h>
#include <sys/times.h>

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
    std::string PARAMS_FILE;
    void load_variables(std::string xml_file,boost::shared_ptr<Robot> robot_ptr);

  /// --------------------  Data Storage  -------------------- ///
  private:
    template<typename T>
    struct is_pointer { static const bool value = false; };
    
    template<typename T>
    struct is_pointer<T*> { static const bool value = true; };
    
    std::map<std::string,boost::shared_ptr<void> > _data_map;
    boost::mutex _data_map_mutex;
  public:
    
    // Returns 'true' if new key was created in map
    template<class T>
    bool set_data(std::string n, const T& v){
      OUT_LOG(logDEBUG) << "Set: " << n << " <-- " << v;
      if(is_pointer<T>::value){
        throw std::runtime_error("Can't save pointer: " + n);
      }
      _data_map_mutex.lock();
      // TODO: Improve this functionality, shouldn't be copying into new class
      std::map<std::string,boost::shared_ptr<void> >::iterator it
        =_data_map.find(n);
      bool new_var = (it == _data_map.end());
      if(new_var){
        _data_map[n] = boost::shared_ptr<T>(new T(v));
      }else{
        (*it).second = boost::shared_ptr<T>(new T(v));
      }
      _data_map_mutex.unlock();

      return new_var;
    }
    
    void remove_data(std::string n){
      _data_map_mutex.lock();
      // TODO: Improve this functionality, shouldn't be copying into new class
      std::map<std::string,boost::shared_ptr<void> >::iterator it
      =_data_map.find(n);
      if (it != _data_map.end()){
        _data_map.erase(it);
      }
      _data_map_mutex.unlock();
    }
  
    template<class T>
    T get_data(std::string n){
      std::map<std::string,boost::shared_ptr<void> >::iterator it;
      _data_map_mutex.lock();
      it = _data_map.find(n);
      if(it != _data_map.end()){
        T* v = (T*) (((*it).second).get());
        _data_map_mutex.unlock();
        OUT_LOG(logDEBUG) << "Get: " << n << " --> " << *v;
        return *v;
      }
      _data_map_mutex.unlock();

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
      // kinematic chain indexing generalized coordinates
      std::vector<unsigned> chain;
      std::vector<bool>      chain_bool;
      
      std::vector<std::string>      supporting_joints;
      std::vector<std::string>      supporting_links;

      // For locomotion, this informs us if this link should be used to calculate a jocobian
      bool                   active;
      bool                   stance;

    };
    
		std::map<std::string,std::vector< boost::shared_ptr<const contact_t> > > _id_contacts_map;
		std::map<std::string,boost::shared_ptr<end_effector_t> > _id_end_effector_map;

	public:
    bool is_end_effector(const std::string& id){
      std::map<std::string,boost::shared_ptr<end_effector_t> >::iterator 
        it = _id_end_effector_map.find(id);
      if(it != _id_end_effector_map.end())
        return true;
      return false;
    }
    
    std::vector<std::string> get_end_effector_names(){
      std::vector<std::string> eefs;
      std::map<std::string,boost::shared_ptr<end_effector_t> >::iterator 
        it = _id_end_effector_map.begin();
      for(;it != _id_end_effector_map.end();it++)
        eefs.push_back((*it).first);
      return eefs;
    }
      
    Ravelin::MatrixNd calc_link_jacobian(const Ravelin::VectorNd& q, const std::string& link){
      return calc_jacobian(q,link,Ravelin::Origin3d(0,0,0));
    }
    
    const std::map<std::string,boost::shared_ptr<Ravelin::RigidBodyd> >& get_links(){
      return _id_link_map;
    }
    
    const boost::shared_ptr<Ravelin::RigidBodyd> get_link(const std::string& link){
      return _id_link_map[link];
    }
   
  
    void add_contact(
        std::string& id,
        Ravelin::Vector3d point,
        Ravelin::Vector3d normal,
        Ravelin::Vector3d tangent,
        Ravelin::Vector3d impulse = Ravelin::Vector3d(),
        double mu_coulomb = 0,double mu_viscous = 0,double restitution = 0, bool compliant = false)
    {
      if(_lock_state)
        throw std::runtime_error("Robot state has been locked after PERCEPTION plugins are called and internal model is updated");
      _id_contacts_map[id].push_back(create_contact(id,point,normal, tangent, impulse,mu_coulomb,mu_viscous,restitution,compliant));
    }
    
    boost::shared_ptr<contact_t> create_contact(
        std::string& id,
        Ravelin::Vector3d point,
        Ravelin::Vector3d normal,
        Ravelin::Vector3d tangent,
        Ravelin::Vector3d impulse = Ravelin::Vector3d(),
        double mu_coulomb = 0,double mu_viscous = 0,double restitution = 0, bool compliant = false)
    {
      boost::shared_ptr<contact_t> c(new contact_t);
      c->id         = id; 
      c->point      = point; 
      c->normal     = normal;
      c->tangent    = tangent;
      c->impulse    = impulse;
      c->mu_coulomb = mu_coulomb; 
      c->mu_viscous = mu_viscous;
      c->restitution= restitution;
      c->compliant  = compliant;
      return c;
    }
    
    void add_contact(boost::shared_ptr<const contact_t>& c)
    {
      if(_lock_state)
        throw std::runtime_error("Robot state has been locked after PERCEPTION plugins are called and internal model is updated");
      _id_contacts_map[c->id].push_back(c);
    }
    
    void get_contacts(std::map<std::string,std::vector< boost::shared_ptr<const contact_t> > >& id_contacts_map){
      id_contacts_map = _id_contacts_map;
    }
    
    void get_link_contacts(const std::string& link_id, std::vector< boost::shared_ptr<const contact_t> >& contacts){
      contacts = _id_contacts_map[link_id];
    }
    
    void get_link_contacts(const std::vector<std::string> link_id,std::vector< boost::shared_ptr<const contact_t> >& contacts){
      contacts.clear();
      for(int i=0;i<link_id.size();i++){
        std::vector< boost::shared_ptr<const contact_t> >& c = _id_contacts_map[link_id[i]];
        contacts.insert(contacts.end(), c.begin(), c.end());
      }
    }
    
    void reset_contact(){
      if(_lock_state)
        throw std::runtime_error("Robot state has been locked after PERCEPTION plugins are called and internal model is updated");
      _id_contacts_map.clear();
    }

  public:
    enum unit_e{          //  ang  |  lin
     // SET OUTSIDE CONTROL LOOP
      position=0,           //  rad  |   m 
      velocity=1,           // rad/s |  m/s
      acceleration=2,       // rad/ss|  m/ss
      load=3,               //  N.m  |   N
     // SET IN CONROLLER
      position_goal=4,      //  rad  |   m 
      velocity_goal=5,      // rad/s |  m/s
      acceleration_goal=6,  // rad/ss|  m/ss
      load_goal=7,// TORQUE //  N.m  |   N
    };

  private:
	
    std::map<unit_e , std::map<std::string, Ravelin::VectorNd > > _state;    
    std::map<unit_e , Ravelin::VectorNd> _base_state;    
    boost::mutex _state_mutex;
    bool _lock_state;

    // TODO: Populate these value in Robot::compile()
    
    // JOINT_NAME --> {gcoord_dof1,...}
    std::map<std::string,std::vector<int> > _id_dof_coord_map;
    
    // gcoord --> (JOINT_NAME,dof)
    std::map<int,std::pair<std::string, int> > _coord_id_map;

  protected:

    void lock_state(){_lock_state = true;};
    void unlock_state(){_lock_state = false;};

  public:
    //void get_foot_value(const std::string& id,unit_e u, Ravelin::Vector3d val){
    //  val = Ravelin::Vector3d(_foot_state[u].segment(0,3).data(),_id_end_effector_map[id]->link->get_pose());
    //}
    int get_joint_dofs(const std::string& id)
    {
      return _id_dof_coord_map[id].size();
    }

    /// ------------ GET/SET JOINT value  ------------ ///

    double get_joint_value(const std::string& id, unit_e u, int dof)
    {
      return _state[u][id][dof];
    }
    
    Ravelin::VectorNd get_joint_value(const std::string& id, unit_e u)
    {
      return _state[u][id];
    }
    
    void get_joint_value(const std::string& id, unit_e u, Ravelin::VectorNd& dof_val)
    {
      dof_val = _state[u][id];
    }
    
    void get_joint_value(const std::string& id, unit_e u,std::vector<double>& dof_val)
    {
      Ravelin::VectorNd& dof = _state[u][id];
      dof_val.resize(dof.rows());
      for(int i=0;i<dof.rows();i++)
        dof_val[i] = dof[i];
    }
    
    void set_joint_value(const std::string& id, unit_e u, int dof, double val)
    {
      if(_lock_state && u <= load)
        throw std::runtime_error("Robot state has been locked after PERCEPTION plugins are called and internal model is updated");
      _state_mutex.lock();
      _state[u][id][dof] = val;
      _state_mutex.unlock();
    }
    
    void set_joint_value(const std::string& id, unit_e u, const Ravelin::VectorNd& dof_val)
    {
      if(_lock_state && u <= load)
        throw std::runtime_error("Robot state has been locked after PERCEPTION plugins are called and internal model is updated");
      _state_mutex.lock();
      Ravelin::VectorNd& dof = _state[u][id];
      if(dof.rows() != dof_val.rows())
        throw std::runtime_error("Missized dofs in joint "+id+": internal="+boost::icl::to_string<double>::apply(dof.rows())+" , provided="+boost::icl::to_string<double>::apply(dof_val.rows()));
      dof = dof_val;
      _state_mutex.unlock();
    }
    
    void set_joint_value(const std::string& id, unit_e u, const std::vector<double>& dof_val)
    {
      if(_lock_state && u <= load)
        throw std::runtime_error("Robot state has been locked after PERCEPTION plugins are called and internal model is updated");
      _state_mutex.lock();
      Ravelin::VectorNd& dof = _state[u][id];
      if(dof.rows() != dof_val.size())
        throw std::runtime_error("Missized dofs in joint "+id+": internal="+boost::icl::to_string<double>::apply(dof.rows())+" , provided="+boost::icl::to_string<double>::apply(dof_val.size()));
      for(int i=0;i<dof.rows();i++)
        dof[i] = dof_val[i];
      _state_mutex.unlock();
    }
    
    void get_joint_value(unit_e u, std::map<std::string,std::vector<double> >& id_dof_val_map){
      std::map<std::string,Ravelin::VectorNd>::iterator it;
      for(it=_state[u].begin();it!=_state[u].end();it++){
        const Ravelin::VectorNd& dof_val_internal = (*it).second;
        std::vector<double>& dof_val = id_dof_val_map[(*it).first];
        dof_val.resize(dof_val_internal.size());
        for(int j=0;j<(*it).second.rows();j++){
          dof_val[j] = dof_val_internal[j]; 
        }
      }
    }
    
    void get_joint_value(unit_e u, std::map<std::string,Ravelin::VectorNd >& id_dof_val_map){
      std::map<std::string,Ravelin::VectorNd>& m = _state[u];
      std::map<std::string,Ravelin::VectorNd >::iterator it;
      for(it=m.begin();it!=m.end();it++){
        const Ravelin::VectorNd& dof_val_internal = (*it).second;
        id_dof_val_map[(*it).first] = dof_val_internal;
      }
    }

    void set_joint_value(unit_e u,const std::map<std::string,std::vector<double> >& id_dof_val_map){
      _state_mutex.lock();
      std::map<std::string,std::vector<double> >::const_iterator it;
      for(it=id_dof_val_map.begin();it!=id_dof_val_map.end();it++){
        Ravelin::VectorNd& dof_val_internal = _state[u][(*it).first];
        const std::vector<double>& dof_val = (*it).second;
        if(dof_val_internal.rows() != dof_val.size()){
          std::cerr << "Missized dofs in joint "+(*it).first+": internal="+boost::icl::to_string<double>::apply(dof_val_internal.rows())+" , provided="+boost::icl::to_string<double>::apply(dof_val.size()) << std::endl;
          throw std::runtime_error("Missized dofs in joint "+(*it).first+": internal="+boost::icl::to_string<double>::apply(dof_val_internal.rows())+" , provided="+boost::icl::to_string<double>::apply(dof_val.size()));
        }
        for(int j=0;j<(*it).second.size();j++){
          dof_val_internal[j] = dof_val[j]; 
        }
      }
      _state_mutex.unlock();
    }
    
    void set_joint_value(unit_e u,const std::map<std::string,Ravelin::VectorNd >& id_dof_val_map){
      _state_mutex.lock();
      std::map<std::string,Ravelin::VectorNd >::const_iterator it;
      for(it=id_dof_val_map.begin();it!=id_dof_val_map.end();it++){
        Ravelin::VectorNd& dof_val_internal = _state[u][(*it).first];
        if(dof_val_internal.rows() != (*it).second.rows())
          throw std::runtime_error("Missized dofs in joint "+(*it).first+": internal="+boost::icl::to_string<double>::apply(dof_val_internal.rows())+" , provided="+boost::icl::to_string<double>::apply((*it).second.rows()));
        dof_val_internal = (*it).second;
      }
      _state_mutex.unlock();
    }

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

      std::map<std::string,std::vector<int> >::iterator it;
      for(it=_id_dof_coord_map.begin();it!=_id_dof_coord_map.end();it++){
        const std::vector<int>& dof = (*it).second;
        std::vector<T>& dof_val = id_dof_val_map[(*it).first];
        dof_val.resize(dof.size());
        for(int j=0;j<(*it).second.size();j++){
          dof_val[j] = generalized_vec[dof[j]]; 
        }
      }
    }
    
    void convert_from_generalized(const Ravelin::VectorNd& generalized_vec, std::map<std::string,std::vector<double> >& id_dof_val_map){
      if(generalized_vec.rows() != NUM_JOINT_DOFS)
        throw std::runtime_error("Missized generalized vector: internal="+boost::icl::to_string<double>::apply(NUM_JOINT_DOFS)+" , provided="+boost::icl::to_string<double>::apply(generalized_vec.rows()));

      std::map<std::string,std::vector<int> >::iterator it;
      for(it=_id_dof_coord_map.begin();it!=_id_dof_coord_map.end();it++){
        const std::vector<int>& dof = (*it).second;
        std::vector<double>& dof_val = id_dof_val_map[(*it).first];
        dof_val.resize(dof.size());
        for(int j=0;j<(*it).second.size();j++){
          dof_val[j] = generalized_vec[dof[j]]; 
        }
      }
    }
    
    void convert_from_generalized(const Ravelin::VectorNd& generalized_vec, std::map<std::string,Ravelin::VectorNd >& id_dof_val_map){
      if(generalized_vec.rows() != NUM_JOINT_DOFS)
        throw std::runtime_error("Missized generalized vector: internal="+boost::icl::to_string<double>::apply(NUM_JOINT_DOFS)+" , provided="+boost::icl::to_string<double>::apply(generalized_vec.rows()));

      std::map<std::string,std::vector<int> >::iterator it;
      for(it=_id_dof_coord_map.begin();it!=_id_dof_coord_map.end();it++){
        const std::vector<int>& dof = (*it).second;
        Ravelin::VectorNd& dof_val = id_dof_val_map[(*it).first];
        dof_val.set_zero(dof.size());
        for(int j=0;j<(*it).second.size();j++){
          dof_val[j] = generalized_vec[dof[j]]; 
        }
      }
    }
    
    /// ------------ GET/SET JOINT GENERALIZED value  ------------ ///

    void set_joint_generalized_value(unit_e u, const Ravelin::VectorNd& generalized_vec)
    {
      if(_lock_state && u <= load)
        throw std::runtime_error("Robot state has been locked after PERCEPTION plugins are called and internal model is updated");
      if(generalized_vec.rows() != NUM_JOINT_DOFS)
        throw std::runtime_error("Missized generalized vector: internal="+boost::icl::to_string<double>::apply(NUM_JOINT_DOFS)+" , provided="+boost::icl::to_string<double>::apply(generalized_vec.rows()));

      _state_mutex.lock();
      std::map<std::string,Ravelin::VectorNd>::iterator it;
      for(it=_state[u].begin();it!=_state[u].end();it++){
        const std::vector<int>& dof = _id_dof_coord_map[(*it).first];
        Ravelin::VectorNd& dof_val = (*it).second;
        if(dof.size() != dof_val.rows())
          throw std::runtime_error("Missized dofs in joint "+(*it).first+": internal="+boost::icl::to_string<double>::apply(dof.size())+" , provided="+boost::icl::to_string<double>::apply(dof_val.rows()));
        for(int j=0;j<(*it).second.rows();j++){
          dof_val[j] = generalized_vec[dof[j]]; 
        }
      }
      _state_mutex.unlock();
    }
    
    void get_joint_generalized_value(unit_e u, Ravelin::VectorNd& generalized_vec){
      generalized_vec.set_zero(NUM_JOINT_DOFS);
      std::map<std::string,Ravelin::VectorNd>::iterator it;
      for(it=_state[u].begin();it!=_state[u].end();it++){
        const std::vector<int>& dof = _id_dof_coord_map[(*it).first];
        const Ravelin::VectorNd& dof_val = (*it).second;
        for(int j=0;j<(*it).second.size();j++){
          generalized_vec[dof[j]] = dof_val[j];
        }
      }
    }
    
    Ravelin::VectorNd get_joint_generalized_value(unit_e u){
      Ravelin::VectorNd generalized_vec;
      get_joint_generalized_value(u,generalized_vec);
      return generalized_vec;
    }
   
    /// With Base
    void set_generalized_value(unit_e u,const Ravelin::VectorNd& generalized_vec){
      if(_lock_state && u <= load)
        throw std::runtime_error("Robot state has been locked after PERCEPTION plugins are called and internal model is updated");
      switch(u){
        case(position_goal):
        case(position):
          set_base_value(u,generalized_vec.segment(NUM_JOINT_DOFS,NUM_JOINT_DOFS+NEULER));
          break;
        default:
          set_base_value(u,generalized_vec.segment(NUM_JOINT_DOFS,NUM_JOINT_DOFS+NSPATIAL));
          break;
      }
      set_joint_generalized_value(u,generalized_vec.segment(0,NUM_JOINT_DOFS));
    }

    /// With Base
    void get_generalized_value(unit_e u, Ravelin::VectorNd& generalized_vec){
      switch(u){
        case(position_goal):
        case(position):
          generalized_vec.set_zero(NUM_JOINT_DOFS+NEULER); 
          break;
        default:
          generalized_vec.set_zero(NUM_JOINT_DOFS+NSPATIAL);
          break;
      }
      generalized_vec.set_sub_vec(0,get_joint_generalized_value(u));
      generalized_vec.set_sub_vec(NUM_JOINT_DOFS,get_base_value(u));
    }
    
    /// With Base
    Ravelin::VectorNd get_generalized_value(unit_e u){
      Ravelin::VectorNd generalized_vec;
      get_generalized_value(u,generalized_vec);
      return generalized_vec;
    }

    void set_base_value(unit_e u,const Ravelin::VectorNd& vec){
      if(_lock_state && u <= load)
        throw std::runtime_error("Robot state has been locked after PERCEPTION plugins are called and internal model is updated");
      switch(u){
        case(position_goal):
        case(position):
          if(vec.rows() != NEULER)
            throw std::runtime_error("position vector should have 7 rows [lin(x y z), quat(x y z w)]");
          break;
        default:
          if(vec.rows() != NSPATIAL)
            throw std::runtime_error("position vector should have 7 rows [lin(x y z), ang(x y z)]");
          break;
      }
       
      _state_mutex.lock();
      _base_state[u] = vec;  
      _state_mutex.unlock();
    }

    void get_base_value(unit_e u, Ravelin::VectorNd& vec){
      vec = _base_state[u];  
    }

    Ravelin::VectorNd get_base_value(unit_e u){
      Ravelin::VectorNd vec;
      get_base_value(u,vec);
      return vec;
    }

    void reset_state(){
      reset_contact();
      std::map<unit_e , std::map<std::string, Ravelin::VectorNd > >::iterator it;
      std::map<std::string, Ravelin::VectorNd >::iterator jt;
      for(it=_state.begin();it!=_state.end();it++){
        for(jt=(*it).second.begin();jt!=(*it).second.end();jt++){
          int N = _id_dof_coord_map[(*jt).first].size();
          (*jt).second.set_zero(N);
        }
      }
    }
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
    void RMRC(const end_effector_t& foot,const Ravelin::VectorNd& q,const Ravelin::Vector3d& goal,Ravelin::VectorNd& q_des, double TOL = 1e-4);

    /// @brief Resolved Motion Rate control (iterative inverse kinematics)
    /// iterative inverse kinematics for a 6d (linear and angular) goal
    void RMRC(const end_effector_t& foot,const Ravelin::VectorNd& q,const Ravelin::VectorNd& goal,Ravelin::VectorNd& q_des, double TOL = 1e-4);

    /// @brief N x 3d kinematics
    Ravelin::VectorNd& link_kinematics(const Ravelin::VectorNd& x,const end_effector_t& foot,const boost::shared_ptr<const Ravelin::Pose3d> frame, const Ravelin::Vector3d& goal, Ravelin::VectorNd& fk, Ravelin::MatrixNd& gk);

    /// @brief N x 6d Jacobian
    Ravelin::MatrixNd& link_jacobian(const Ravelin::VectorNd& x,const end_effector_t& foot,const boost::shared_ptr<const Ravelin::Pose3d> frame, Ravelin::MatrixNd& gk);

    /// @brief N x 6d kinematics
    Ravelin::VectorNd& link_kinematics(const Ravelin::VectorNd& x,const end_effector_t& foot,const boost::shared_ptr<const Ravelin::Pose3d> frame, const Ravelin::VectorNd& goal, Ravelin::VectorNd& fk, Ravelin::MatrixNd& gk);

    void end_effector_inverse_kinematics(
      const std::vector<std::string>& foot_id,
      const std::vector<Ravelin::Vector3d>& foot_pos,
      const std::vector<Ravelin::Vector3d>& foot_vel,
      const std::vector<Ravelin::Vector3d>& foot_acc,
      const Ravelin::VectorNd& q,
      Ravelin::VectorNd& q_des,
      Ravelin::VectorNd& qd_des,
      Ravelin::VectorNd& qdd_des, double TOL = 1e-4);

    void calc_generalized_inertia(const Ravelin::VectorNd& q, Ravelin::MatrixNd& M);
   
    const boost::shared_ptr<Ravelin::RigidBodyd> get_root_link(){return _root_link;}
    
    int joint_dofs(){return NUM_JOINT_DOFS;}
  private:
    /// @brief N x (3/6)d kinematics for RMRC
    Ravelin::VectorNd& contact_kinematics(const Ravelin::VectorNd& x,const end_effector_t& foot, Ravelin::VectorNd& fk, Ravelin::MatrixNd& gk);
 
    boost::shared_ptr<Ravelin::ArticulatedBodyd>        _abrobot;
    
    std::map<std::string,boost::shared_ptr<Ravelin::RigidBodyd> > _id_link_map;
    boost::shared_ptr<Ravelin::RigidBodyd> _root_link;
    std::map<std::string,boost::shared_ptr<Ravelin::Jointd> > _id_joint_map;

    // NDFOFS for forces, accel, & velocities
    unsigned NDOFS, NUM_JOINT_DOFS;

    std::vector<bool> _disabled_dofs; 

    /// set up internal models after kineamtic model is set (called from init)
    void compile();
};
}

#endif // ROBOT_H
