/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/robot.h>
#include <boost/algorithm/string.hpp>
#include <Pacer/utilities.h>

using namespace Pacer;

void Robot::set_model_state(const Ravelin::VectorNd& q,const Ravelin::VectorNd& qd){
  Ravelin::VectorNd set_q,set_qd;
  if(get_data<Ravelin::VectorNd>("generalized_q", set_q)){
    set_q.set_sub_vec(0,q);
    _abrobot->set_generalized_coordinates_euler(set_q);
  }

  if(qd.rows() > 0)
  if(get_data<Ravelin::VectorNd>("generalized_qd", set_qd)){
    set_qd.set_sub_vec(0,qd);
    _abrobot->set_generalized_velocity(Ravelin::DynamicBodyd::eSpatial,set_qd);
  }
}

void Robot::calc_generalized_inertia(const Ravelin::VectorNd& q, Ravelin::MatrixNd& M){
  set_model_state(q);
  M.resize(NDOFS,NDOFS);
  _abrobot->get_generalized_inertia(M);
}

void Robot::compile(){
  check_phase(initialization);
  NDOFS = _abrobot->num_generalized_coordinates(Ravelin::DynamicBodyd::eSpatial);
  NUM_JOINT_DOFS = NDOFS - NSPATIAL;
  
  // Find Joint Names
  std::vector<boost::shared_ptr<Ravelin::Jointd> > joints = _abrobot->get_joints();
  
  for(unsigned i=0;i<joints.size();i++){
    OUT_LOG(logINFO) << joints[i]->joint_id << ", dofs = " << joints[i]->num_dof();
    if(joints[i]->num_dof() != 0){
      _id_joint_map[joints[i]->joint_id] = joints[i];
    }
  }
  
  {
    OUT_LOG(logDEBUG1) << "Joint Map";
    std::map<std::string, boost::shared_ptr<Ravelin::Jointd> >::iterator it;
    for(it=_id_joint_map.begin();it!=_id_joint_map.end();it++)
      OUT_LOG(logDEBUG1) << (*it).first << ", " << (*it).second;
  }
  _joint_ids = get_map_keys(_id_joint_map);
  // Find link (and eef) names
  // Set up link references
  std::vector<boost::shared_ptr<Ravelin::RigidBodyd> > links = _abrobot->get_links();
  {
    OUT_LOG(logDEBUG1) << "Link vector";
    std::vector<boost::shared_ptr<Ravelin::RigidBodyd> >::iterator it;
    for(it=links.begin();it!=links.end();it++)
      OUT_LOG(logDEBUG1) << (*it)->body_id;
  }
  
  _foot_ids.clear();
  _root_link = links[0];
  for(unsigned i=0;i<links.size();i++){
    _id_link_map[links[i]->body_id] = links[i];
    if (links[i]->is_end_effector()) {
      _id_foot_map[links[i]->body_id] = links[i];
      _foot_ids.push_back(links[i]->body_id);
    }
  }
  
  {
    OUT_LOG(logDEBUG1) << "Link Map";
    std::map<std::string, boost::shared_ptr<Ravelin::RigidBodyd> >::iterator it;
    for(it=_id_link_map.begin();it!=_id_link_map.end();it++)
      OUT_LOG(logDEBUG1) << (*it).first << ", " << (*it).second;
  }
  _link_ids = get_map_keys(_id_link_map);
  
  // NOTE: This data comes from the robot model now
//  _foot_ids = get_data<std::vector<std::string> >("init.end-effector.id");
  
  // initialize state data with name data
  init_state();
  
  // set up other joint data
  _disabled_dofs.resize(NDOFS);
  std::fill(_disabled_dofs.begin(),_disabled_dofs.end(),true);
  
  std::map<std::string, boost::shared_ptr<Ravelin::Jointd> >::iterator it;
  for(it=_id_joint_map.begin();it!=_id_joint_map.end();it++){
    const std::pair<std::string, boost::shared_ptr<Ravelin::Jointd> >& id_joint = (*it);
    _id_dof_coord_map[id_joint.first] = std::vector<int>(id_joint.second->num_dof());
    for(int j=0;j<id_joint.second->num_dof();j++){
      if(id_joint.second->num_dof() == 0)
        _disabled_dofs[id_joint.second->get_coord_index() + j] = false;
      _id_dof_coord_map[id_joint.first][j]
      = id_joint.second->get_coord_index() + j;
      
      _coord_id_map[id_joint.second->get_coord_index() + j]
      = std::pair<std::string,int>(id_joint.first,j);
    }
    
    OUTLOG(_id_dof_coord_map[id_joint.first],id_joint.first+"_dofs",logDEBUG1);
    
    std::map<unit_e , std::map<std::string, Ravelin::VectorNd > >::iterator it;
    for(it=_state.begin();it!=_state.end();it++){
      std::map<std::string, Ravelin::VectorNd >& dof = (*it).second;
      dof[id_joint.first] = Ravelin::VectorNd(_id_dof_coord_map[id_joint.first].size());
    }
  }
  
  // set up enbd effectors
  // TODO: (Depricated?)
  // Initialize end effectors
  for(unsigned i=0;i<_foot_ids.size();i++){
    boost::shared_ptr<end_effector_t> eef
    = boost::shared_ptr<end_effector_t>(new end_effector_t);
    eef->link = _id_link_map[_foot_ids[i]];
    OUT_LOG(logDEBUG1) << "eef link id = " << _foot_ids[i] << ", " << eef->link;
    
    eef->id = eef->link->body_id;
    
    boost::shared_ptr<Ravelin::RigidBodyd> rb_ptr = eef->link;
    OUT_LOG(logDEBUG) << eef->id ;
    eef->chain_bool.resize(NUM_JOINT_DOFS+6);
    std:fill(eef->chain_bool.begin(),eef->chain_bool.end(),false);
    do {
      OUT_LOG(logDEBUG) << "  " << rb_ptr->body_id;
      eef->chain_links.push_back(rb_ptr);

      boost::shared_ptr<Ravelin::Jointd> joint_ptr = rb_ptr->get_inner_joint_explicit();
      OUT_LOG(logDEBUG) << "  " << joint_ptr->joint_id;
      
      const std::vector<int>& dof = _id_dof_coord_map[joint_ptr->joint_id];
      OUT_LOG(logDEBUG) << "  dofs = " << dof.size() ;
      eef->chain_joints.push_back(joint_ptr);
      for(int i=0;i<dof.size();i++){
        OUT_LOG(logDEBUG) << "  " << dof[i] <<  " "<< joint_ptr->joint_id;
        eef->chain.push_back(dof[i]);
        eef->chain_bool[dof[i]] = true;
      }
      rb_ptr = joint_ptr->get_inboard_link();
    }
    while (rb_ptr != _abrobot->get_base_link());
    
    _id_end_effector_map[_foot_ids[i]] = eef;
    
  }
}

void Robot::update(){
  check_phase(misc_sensor);

  // Propagate data set in _state into robot model
  Ravelin::VectorNd generalized_q = get_generalized_value(position);
  Ravelin::VectorNd generalized_qd = get_generalized_value(velocity);
  Ravelin::VectorNd generalized_qdd = get_generalized_value(acceleration);
  Ravelin::VectorNd generalized_fext = get_generalized_value(load);
  set_joint_generalized_value(position_goal,generalized_q.segment(0,NUM_JOINT_DOFS));
  set_joint_generalized_value(velocity_goal,generalized_qd.segment(0,NUM_JOINT_DOFS));
  set_joint_generalized_value(acceleration_goal,Ravelin::VectorNd::zero(NUM_JOINT_DOFS));

  set_data<Ravelin::VectorNd>("generalized_q", generalized_q);
  set_data<Ravelin::VectorNd>("generalized_qd", generalized_qd);
  set_data<Ravelin::VectorNd>("generalized_qdd", generalized_qdd);
  set_data<Ravelin::VectorNd>("generalized_fext", generalized_fext);
  set_generalized_value(load_goal,Ravelin::VectorNd::zero(generalized_fext.rows()));

  Ravelin::Vector3d workv;
  set_data<Ravelin::Vector3d>("base.state.x",Ravelin::Vector3d(generalized_q.segment(NUM_JOINT_DOFS, NUM_JOINT_DOFS+3).data(), _abrobot->get_gc_pose()));
  set_data<Ravelin::Quatd>("base.state.q",Ravelin::Quatd(generalized_q[NUM_JOINT_DOFS+3],generalized_q[NUM_JOINT_DOFS+4],generalized_q[NUM_JOINT_DOFS+5],generalized_q[NUM_JOINT_DOFS+6]));

  workv = Ravelin::Vector3d(generalized_qd.segment(NUM_JOINT_DOFS, NUM_JOINT_DOFS+3).data(), _abrobot->get_gc_pose());
  set_data<Ravelin::Vector3d>("base.state.xd", workv);
  workv = Ravelin::Vector3d(generalized_qd.segment(NUM_JOINT_DOFS+3, NUM_JOINT_DOFS+NSPATIAL).data(), _abrobot->get_gc_pose());
  set_data<Ravelin::Vector3d>("base.state.omega", workv);
  workv = Ravelin::Vector3d(generalized_qdd.segment(NUM_JOINT_DOFS, NUM_JOINT_DOFS+3).data(), _abrobot->get_gc_pose());
  set_data<Ravelin::Vector3d>("base.state.xdd", workv);
  workv = Ravelin::Vector3d(generalized_qdd.segment(NUM_JOINT_DOFS+3, NUM_JOINT_DOFS+NSPATIAL).data(), _abrobot->get_gc_pose());
  set_data<Ravelin::Vector3d>("base.state.alpha", workv);
  
  Ravelin::VectorNd q = generalized_q.segment(0,NUM_JOINT_DOFS);
  Ravelin::VectorNd qd = generalized_qd.segment(0,NUM_JOINT_DOFS);
  Ravelin::VectorNd qdd = generalized_qdd.segment(0,NUM_JOINT_DOFS);
  Ravelin::VectorNd fext = generalized_fext.segment(0,NUM_JOINT_DOFS);
  
  if(set_data<Ravelin::VectorNd>("q",q))
    set_data<Ravelin::VectorNd>("init.q",q);
  set_data<Ravelin::VectorNd>("qd",qd);
  set_data<Ravelin::VectorNd>("qdd",qdd);
  
  set_model_state(generalized_q);
  update_poses();
}

void Robot::update_poses(){
  check_phase(misc_sensor);

  Ravelin::Pose3d base_link_frame(*(_root_link->get_pose().get()));
  
  const std::vector<double>
  &base_start = get_data<std::vector<double> >("init.base.x");
  
  Ravelin::Matrix3d R_base(base_link_frame.q),
  R_pitch = Ravelin::Matrix3d::rot_Y(base_start[4]),
  new_R;
  OUT_LOG(logINFO) << "Rotating base_link_frame saggitally by: theta = " << -base_start[4];
  R_base.mult(R_pitch,new_R);
  base_link_frame = Ravelin::Pose3d( new_R,base_link_frame.x,GLOBAL);
  Ravelin::Origin3d roll_pitch_yaw;
  base_link_frame.q.to_rpy(roll_pitch_yaw);
  
  
  // preserve yaw
  Ravelin::Pose3d base_horizontal_frame(
                                        Ravelin::AAngled(0,0,1,roll_pitch_yaw[2]),
                                        base_link_frame.x,GLOBAL);
  
  set_data<Ravelin::Pose3d>("base_stability_frame",base_link_frame);
  set_data<Ravelin::Pose3d>("base_link_frame",base_link_frame);
  set_data<Ravelin::Origin3d>("roll_pitch_yaw",roll_pitch_yaw);
  set_data<Ravelin::Pose3d>("base_horizontal_frame",base_horizontal_frame);
  
  Ravelin::Pose3d GLOBAL_POSE;
  Utility::visualize.push_back( Pacer::VisualizablePtr( new Pose(base_link_frame,0.8)));
  Utility::visualize.push_back( Pacer::VisualizablePtr( new Pose(base_horizontal_frame,1.5)));
  Utility::visualize.push_back( Pacer::VisualizablePtr( new Pose(GLOBAL_POSE,1.0)));
  
}

// ============================================================================
// ===========================  BEGIN ROBOT INIT  =============================

#include <stdlib.h>
#include <Moby/SDFReader.h>
#include <Moby/XMLReader.h>
#include <Moby/ArticulatedBody.h>
void Robot::init_robot(){
  check_phase(initialization);
  OUT_LOG(logDEBUG) << "> > Robot::init_robot(.)";
  // ================= LOAD SCRIPT DATA ==========================
  
  std::string robot_model_file = get_data<std::string>("robot-model");

  std::string model_type;
  
  {
    std::vector<std::string> strs;
    boost::split(strs,robot_model_file,boost::is_any_of("."));
    if (strs.size() < 1) {
      OUT_LOG(logERROR) << robot_model_file << " has no file extension!";
      throw std::runtime_error(robot_model_file + " has no file extension!");
    }
    model_type = strs.back();
  }

  if (robot_model_file[0] != '/') {
    if (!getenv("PACER_MODEL_PATH"))
      throw std::runtime_error("Environment variable PACER_MODEL_PATH not defined");
    
    std::string pPath(getenv ("PACER_MODEL_PATH"));
    OUT_LOG(logDEBUG) << "PACER_MODEL_PATH = " << pPath;
    robot_model_file = pPath+"/"+robot_model_file;
  }
  
  
  // ================= BUILD ROBOT ==========================
  // Get Model type
  
  
  OUT_LOG(logINFO) << "Using robot model : " << robot_model_file;
  
  /// The map of objects read from the simulation XML file
  if(model_type.compare("sdf") == 0){
    std::map<std::string, Moby::ControlledBodyPtr> READ_MAP = Moby::SDFReader::read_models(robot_model_file);
    for (std::map<std::string, Moby::ControlledBodyPtr>::const_iterator i = READ_MAP.begin();
         i !=READ_MAP.end(); i++)
    {
      OUT_LOG(logINFO) << "Checking: " << i->first;
      // find the robot reference
      if (!_abrobot)
      {
        _abrobot = boost::dynamic_pointer_cast<Moby::ArticulatedBody>(i->second);
      }
      
      if (_abrobot)
      {
        OUT_LOG(logINFO) << "Using robot: " << i->first;
        break;
      }
    }
    
    if (!_abrobot){
      throw std::runtime_error("could not find RCArticulatedBody for robot SDF: " + robot_model_file);
    }
  } else if(model_type.compare("xml") == 0){
    std::cerr << "look for model: " << robot_model_file << std::endl;
    
    std::map<std::string, Moby::BasePtr> READ_MAP = Moby::XMLReader::read(std::string(robot_model_file));
    for (std::map<std::string, Moby::BasePtr>::const_iterator i = READ_MAP.begin();
         i !=READ_MAP.end(); i++)
    {
      // find the robot reference
      if (!_abrobot)
      {
        _abrobot = boost::dynamic_pointer_cast<Ravelin::ArticulatedBodyd>(i->second);
      }
      
      if (_abrobot)
      {
        OUT_LOG(logINFO) << "Using robot: " << i->first;
        break;
      }
    }
    
    if (!_abrobot){
      throw std::runtime_error("could not find RCArticulatedBody for robot XML: " + robot_model_file);
    }
  } else {
    throw std::runtime_error("Robot model file ["+robot_model_file+"] has unknown extension : " + model_type);
  }
  compile();
  
  // Initialized Joints
  std::vector<std::string>
  joint_names = get_data<std::vector<std::string> >("init.joint.id");
  std::vector<double>
  joint_dofs = get_data<std::vector<double> >("init.joint.dofs");
  std::vector<double>
  joints_start = get_data<std::vector<double> >("init.joint.q"),
  joints_vel_start;
  bool init_vel = get_data<std::vector<double> >("init.joint.qd",joints_vel_start);
  
  std::map<std::string, std::vector<double> > init_q, init_qd;
  for(int i=0,ii=0;i<joint_names.size();i++){
    init_q[joint_names[i]] = std::vector<double>(joint_dofs[i]);
    init_qd[joint_names[i]] = std::vector<double>(joint_dofs[i]);
    for(int j=0;j<joint_dofs[i];j++,ii++){
      init_q[joint_names[i]][j] = joints_start[ii];
      init_qd[joint_names[i]][j] = (init_vel)? joints_vel_start[ii] : 0.0;
    }
    OUTLOG(init_q[joint_names[i]],joint_names[i]+"_q (start)",logINFO);
    OUTLOG(init_qd[joint_names[i]],joint_names[i]+"_qd (start)",logINFO);
  }
  
  
  // Initialize base
  std::vector<double>
  base_start = get_data<std::vector<double> >("init.base.x");
  Ravelin::Quatd init_quat =
  Ravelin::Quatd::rpy(base_start[3],base_start[4],base_start[5]);
  
  Ravelin::VectorNd init_base(7);
  init_base[0] = base_start[0];
  init_base[1] = base_start[1];
  init_base[2] = base_start[2];
  init_base[3] = init_quat.x;
  init_base[4] = init_quat.y;
  init_base[5] = init_quat.z;
  init_base[6] = init_quat.w;
  
  std::vector<double> base_startv(6);
  std::fill(base_startv.begin(),base_startv.end(),0);
  get_data<std::vector<double> >("init.base.xd",base_startv);
  Ravelin::VectorNd init_basev(6,&base_startv[0]);
  
  set_joint_value(position,init_q);
  set_base_value(position,init_base);
  
  set_joint_value(velocity,init_qd);
  set_base_value(velocity,init_basev);
  OUT_LOG(logDEBUG) << "<< Robot::init_robot(.)";
}
