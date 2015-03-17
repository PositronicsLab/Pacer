/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/robot.h>
using namespace Pacer;

std::vector<Pacer::VisualizablePtr> visualize;
void Robot::calc_com(){

  Ravelin::Vector3d center_of_mass_x;
  center_of_mass_x.pose = Moby::GLOBAL;
  double total_mass=0;

  std::map<std::string, Moby::RigidBodyPtr>::iterator it;
  for(it=_id_link_map.begin();it!=_id_link_map.end();it++){
    double m = (*it).second->get_mass();
    total_mass += m;
    center_of_mass_x += (Ravelin::Pose3d::transform_point(Moby::GLOBAL,Ravelin::Vector3d(0,0,0,(*it).second->get_inertial_pose())) *= m);
  }
  center_of_mass_x /= total_mass;

  boost::shared_ptr<Ravelin::Pose3d> base_com_w = boost::shared_ptr<Ravelin::Pose3d>(new Ravelin::Pose3d(Moby::GLOBAL));
  base_com_w->x = Ravelin::Origin3d(center_of_mass_x);
  Ravelin::SVector6d com_vel = Ravelin::Pose3d::transform(base_com_w, _root_link->get_velocity());
  
  Ravelin::Vector3d center_of_mass_xd = com_vel.get_upper();

  Ravelin::SAcceld com_acc = Ravelin::Pose3d::transform(base_com_w, _root_link->get_accel());
  Ravelin::Vector3d center_of_mass_xdd = com_acc.get_linear();

  center_of_mass_xd.pose = center_of_mass_xdd.pose = Moby::GLOBAL;

  set_data<Ravelin::Vector3d>("center_of_mass.x",center_of_mass_x);
  set_data<Ravelin::Vector3d>("center_of_mass.xd",center_of_mass_xd);
  set_data<Ravelin::Vector3d>("center_of_mass.xdd",center_of_mass_xdd);

//  center_of_mass_wd = com_acc.get_angular();
//  center_of_mass_w = com_vel.get_angular();

  // ZMP
  // x(k+1) = A x(k) + B u(k)
  // p(k)   = C x(k)

  // From Kajita et al. 2003
  // x(k) = [x(kT),xd(kT),xdd(kT)]'
  // u(k) = u_x(kT)
  // p(k) = p_x(kT)
  // A = [1  T  (T^2)/2;
  //      0  1  T      ;
  //      0  0  1      ]
  // B = [(T^3)/2 ;
  //      (T^2)/2 ;
  //          T   ]
  // C = [1 0 -z/g]

  // e = p - p_ref
  //
  Ravelin::Vector3d C(1,0,-center_of_mass_x[2]/grav,Moby::GLOBAL);
  Ravelin::Vector3d zero_moment_point =
      Ravelin::Vector3d(C.dot(Ravelin::Vector3d(center_of_mass_x[0],center_of_mass_xd[0],center_of_mass_xdd[0],Moby::GLOBAL)),
                        C.dot(Ravelin::Vector3d(center_of_mass_x[1],center_of_mass_xd[1],center_of_mass_xdd[1],Moby::GLOBAL)),
                        0,Moby::GLOBAL);
  set_data<Ravelin::Vector3d>("zero_moment_point",zero_moment_point);

  // ZMP and COM
  Ravelin::Vector3d CoM_2D(center_of_mass_x[0],center_of_mass_x[1],center_of_mass_x[2]-0.10,Moby::GLOBAL);
  visualize.push_back( Pacer::VisualizablePtr( new Ray(CoM_2D,center_of_mass_x,Ravelin::Vector3d(0,0,1))));
//  visualize.push_back( Pacer::VisualizablePtr( new Ray(CoM_2D + center_of_mass_xd*0.1,CoM_2D,Ravelin::Vector3d(0.5,0,1)));
//  visualize.push_back( Pacer::VisualizablePtr( new Ray(CoM_2D + center_of_mass_xd*0.1 + center_of_mass_xdd*0.01,CoM_2D + center_of_mass_xd*0.1,Ravelin::Vector3d(1,0,0)));
  visualize.push_back( Pacer::VisualizablePtr( new Ray(CoM_2D+Ravelin::Vector3d(zero_moment_point[0],zero_moment_point[1],0,Moby::GLOBAL)*0.1,CoM_2D,Ravelin::Vector3d(0,1,0))));
}

double Robot::calc_energy(const Ravelin::VectorNd& v, const Ravelin::MatrixNd& M) const {
  // Potential Energy: this calculation assumes that ground is always at zero
  Ravelin::VectorNd workv_;
  double PE = 0;
  std::map<std::string, Moby::RigidBodyPtr>::const_iterator it;
  for(it=_id_link_map.begin();it!=_id_link_map.end();it++){
     const Moby::RigidBodyPtr link = (*it).second;
     double m = link->get_mass();
     Ravelin::Pose3d 
       link_com(link->get_inertial_pose());
     link_com.update_relative_pose(Moby::GLOBAL);
     PE += link_com.x[2] * m * grav;
  }
  M.mult(v, workv_);
  double KE = workv_.dot(v)*0.5;
//#ifndef NDEBUG
  OUT_LOG(logDEBUG) << "KE = " << KE << ", PE = " << PE;
  OUT_LOG(logDEBUG) << "Total Energy = " << (KE + PE);
//#endif
  return KE;
  // Kinetic Energy
}

void Robot::set_model_state(const Ravelin::VectorNd& q,const Ravelin::VectorNd& qd){
  Ravelin::VectorNd set_q,set_qd;
  _abrobot->get_generalized_coordinates(Moby::DynamicBody::eEuler,set_q);
  _abrobot->get_generalized_velocity(Moby::DynamicBody::eSpatial,set_qd);

  set_q.set_sub_vec(0,q);
  set_qd.set_sub_vec(0,qd);

  _abrobot->set_generalized_coordinates(Moby::DynamicBody::eEuler,set_q);
  _abrobot->set_generalized_velocity(Moby::DynamicBody::eSpatial,set_qd);
  _abrobot->update_link_poses();
  _abrobot->update_link_velocities();
}

void Robot::calc_generalized_inertia(const Ravelin::VectorNd& q, Ravelin::MatrixNd& M){
  set_model_state(q);
  M.resize(NDOFS,NDOFS);
  _abrobot->get_generalized_inertia(M);
}

void Robot::compile(){
  NDOFS = _abrobot->num_generalized_coordinates(Moby::DynamicBody::eSpatial);
  NUM_JOINT_DOFS = NDOFS - NSPATIAL;

  _dbrobot = boost::dynamic_pointer_cast<Moby::DynamicBody>(_abrobot);
  
  std::vector<Moby::JointPtr> joints = _abrobot->get_joints();
    
  for(unsigned i=0;i<joints.size();i++){
    OUT_LOG(logINFO) << joints[i]->id << ", dofs = " << joints[i]->num_dof();
    if(joints[i]->num_dof() != 0){
      _id_joint_map[joints[i]->id] = joints[i];
    }
  }

  std::map<std::string, Moby::JointPtr>::iterator it;
  for(it=_id_joint_map.begin();it!=_id_joint_map.end();it++){
    const std::pair<std::string, Moby::JointPtr>& id_joint = (*it);
    _id_dof_coord_map[id_joint.first] = std::vector<int>(id_joint.second->num_dof());
    for(int j=0;j<id_joint.second->num_dof();j++){
      _id_dof_coord_map[id_joint.first][j] 
        = id_joint.second->get_coord_index() + j;

      _coord_id_map[id_joint.second->get_coord_index() + j] 
        = std::pair<std::string,int>(id_joint.first,j);
    }
  }
  reset_state();

  // Set up link references
  std::vector<Moby::RigidBodyPtr> links = _abrobot->get_links();
  _root_link = links[0];
  for(unsigned i=0;i<joints.size();i++){
    _id_link_map[links[i]->id] = links[i];
  }
}

void Robot::init_end_effector(boost::shared_ptr<end_effector_t>& eef_ptr){
  end_effector_t& eef = *(eef_ptr.get());
  eef.id = eef.link->id;

  Moby::JointPtr joint_ptr = eef.link->get_inner_joint_explicit();
  Moby::RigidBodyPtr rb_ptr = eef.link;
  OUT_LOG(logDEBUG) << eef.id ;
  eef.chain_bool.resize(NUM_JOINT_DOFS);
  rb_ptr = joint_ptr->get_inboard_link();
  while (rb_ptr != _abrobot->get_base_link()) {
    OUT_LOG(logDEBUG) << "  " << rb_ptr->id;
    joint_ptr = rb_ptr->get_inner_joint_explicit();
    OUT_LOG(logDEBUG) << "  " << joint_ptr->id;

    const std::vector<int>& dof = _id_dof_coord_map[joint_ptr->id];
    for(int i=0;i<dof.size();i++){
      OUT_LOG(logDEBUG) << "  " << dof[i] <<  " "<< joint_ptr->id;
      eef.chain.push_back(dof[i]);
      eef.chain_bool[dof[i]] = true;
    }
    rb_ptr = joint_ptr->get_inboard_link();
  }
}
/*
void Robot::update(
    const Ravelin::VectorNd& generalized_q,
    const Ravelin::VectorNd& generalized_qd,
    const Ravelin::VectorNd& generalized_qdd,
    const Ravelin::VectorNd& generalized_fext,
    std::vector< boost::shared_ptr<const contact_t> >& contacts
    ){
  unlock_state();
  set_generalized_value(position,generalized_q);
  set_generalized_value(position,generalized_qd);
  set_generalized_value(position,generalized_qdd);
  set_generalized_value(position,generalized_fext);
  for(int i=0;i<contacts.size();i++){
    add_contact(contacts[i]);
  }
  lock_state();
}
*/

void Robot::update(){

  // Propagate data set in _state into robot model
  Ravelin::VectorNd generalized_q = get_generalized_value(position);
  Ravelin::VectorNd generalized_qd = get_generalized_value(velocity);
  Ravelin::VectorNd generalized_qdd = get_generalized_value(acceleration);
  Ravelin::VectorNd generalized_fext = get_generalized_value(load);

  set_data<Ravelin::VectorNd>("generalized_q", generalized_q);
  set_data<Ravelin::VectorNd>("generalized_qd", generalized_qd);
  set_data<Ravelin::VectorNd>("generalized_qdd", generalized_qdd);
  set_data<Ravelin::VectorNd>("generalized_fext", generalized_fext);

  Ravelin::VectorNd q = generalized_q.segment(0,NUM_JOINT_DOFS); 
  Ravelin::VectorNd qd = generalized_qd.segment(0,NUM_JOINT_DOFS);
  Ravelin::VectorNd qdd = generalized_qdd.segment(0,NUM_JOINT_DOFS);
  Ravelin::VectorNd fext = generalized_fext.segment(0,NUM_JOINT_DOFS);

	set_data<Ravelin::VectorNd>("q",q);
	set_data<Ravelin::VectorNd>("qd",qd);
	set_data<Ravelin::VectorNd>("qdd",qdd);

  update_poses(generalized_q);
  
  boost::shared_ptr<Ravelin::Pose3d> base_link_frame( new Ravelin::Pose3d(
        get_data<Ravelin::Pose3d>("base_link_frame")));
  boost::shared_ptr<Ravelin::Pose3d> base_horizontal_frame( new Ravelin::Pose3d(
        get_data<Ravelin::Pose3d>("base_horizontal_frame")));

#ifndef NDEBUG
  visualize.push_back( Pacer::VisualizablePtr( new Pose(*(base_link_frame.get()),0.8)));
  visualize.push_back( Pacer::VisualizablePtr( new Pose(*(base_horizontal_frame.get()),1.5)));
  visualize.push_back( Pacer::VisualizablePtr( new Pose(Moby::GLOBAL,1.0)));
#endif

  //Ravelin::MatrixNd M;
  //calc_generalized_inertia(generalized_q,M);
  //set_data<Ravelin::VectorNd>("generalized_inertia",M);
  
  calc_com();

#ifndef NDEBUG
  //calc_energy(generalized_qd,M);
#endif

}

void Robot::update_poses(const Ravelin::VectorNd& q){
  set_model_state(q); 
  
  Ravelin::Pose3d base_link_frame(*(_root_link->get_pose().get()));

  const std::vector<double>
    &base_start = Utility::get_variable<std::vector<double> >("init.base.x");

  Ravelin::Matrix3d R_base(base_link_frame.q),
      R_pitch = Ravelin::Matrix3d::rot_Y(base_start[4]),
      new_R;
  OUT_LOG(logINFO) << "Rotating base_link_frame saggitally by: theta = " << -base_start[4];
  R_base.mult(R_pitch,new_R);
  base_link_frame = Ravelin::Pose3d( new_R,base_link_frame.x,Moby::GLOBAL);
  Ravelin::Origin3d roll_pitch_yaw;
  base_link_frame.q.to_rpy(roll_pitch_yaw);


  // preserve yaw
  Ravelin::Pose3d base_horizontal_frame(
            Ravelin::Matrix3d::rot_Z(roll_pitch_yaw[2]),
          base_link_frame.x,Moby::GLOBAL);

  set_data<Ravelin::Pose3d>("base_link_frame",base_link_frame);
  set_data<Ravelin::Origin3d>("roll_pitch_yaw",roll_pitch_yaw);
  set_data<Ravelin::Pose3d>("base_horizontal_frame",base_horizontal_frame);
}

// ============================================================================
// ===========================  BEGIN ROBOT INIT  =============================

#include <stdlib.h>     /* getenv */
#include <boost/filesystem.hpp>

#include <Moby/SDFReader.h>
#include <Moby/XMLReader.h>

 void Robot::init_robot(){
  OUT_LOG(logDEBUG) << ">> Robot::init_robot(.)";
  // ================= LOAD SCRIPT DATA ==========================
  std::string pPath(getenv ("PACER_MODEL_PATH"));
  OUT_LOG(logDEBUG) << "PACER_MODEL_PATH = " << pPath;

  // ================= SETUP LOGGING ==========================

  std::string& LOG_TYPE = Utility::get_variable<std::string>("logging");

  FILELog::ReportingLevel() =
      FILELog::FromString( (!LOG_TYPE.empty()) ? LOG_TYPE : "INFO");
#ifdef LOGGING
    FILE * pFile;
    pFile = fopen ("out.log","w");
    fprintf(pFile, "");
    fflush(pFile);
    fclose (pFile);
#endif
  OUT_LOG(logDEBUG1) << "Log Type : " << LOG_TYPE;

  // ================= BUILD ROBOT ==========================
  std::string robot_model_file = Utility::get_variable<std::string>("robot-model");
  
  // Get Model type
  std::string model_type = boost::filesystem::extension(robot_model_file);
  robot_model_file = robot_model_file+pPath;
  OUT_LOG(logINFO) << "Using robot model : " << robot_model_file;

  (robot_model_file.substr(robot_model_file.size()-4,robot_model_file.size()));
  /// The map of objects read from the simulation XML file
  if(model_type.compare("sdf") == 0){
    std::map<std::string, Moby::DynamicBodyPtr> READ_MAP = Moby::SDFReader::read_models(robot_model_file);

    for (std::map<std::string, Moby::DynamicBodyPtr>::const_iterator i = READ_MAP.begin();
         i !=READ_MAP.end(); i++)
    {
      // find the robot reference
      if (!_abrobot)
      {
        _abrobot = boost::dynamic_pointer_cast<Moby::RCArticulatedBody>(i->second);
      }
    }

    if (!_abrobot){
      throw std::runtime_error("could not find RCArticulatedBody for robot SDF");
    }
  } else if(model_type.compare(".xml") == 0){
    std::cerr << "look for model: " << robot_model_file << std::endl;

    std::map<std::string, Moby::BasePtr> READ_MAP = Moby::XMLReader::read(std::string(robot_model_file));
    for (std::map<std::string, Moby::BasePtr>::const_iterator i = READ_MAP.begin();
       i !=READ_MAP.end(); i++)
    {
    // find the robot reference
    if (!_abrobot)
    {
      _abrobot = boost::dynamic_pointer_cast<Moby::RCArticulatedBody>(i->second);
    }
    }

    if (!_abrobot){
      throw std::runtime_error("could not find RCArticulatedBody for robot XML");
    }
  } else {
    throw std::runtime_error("Robot model file has unknown extension : " + model_type);
  }
  compile();

  // ================= SET UP END EFFECTORS ==========================

  const std::vector<std::string> 
    &eef_names_ = Utility::get_variable<std::vector<std::string> >("init.end-effector.id");

  // Initialize end effectors
  for(unsigned i=0;i<eef_names_.size();i++){
    boost::shared_ptr<end_effector_t> eef;
    eef->link = _id_link_map[eef_names_[i]];
    init_end_effector(eef);
    _id_end_effector_map[eef_names_[i]] = eef;
  }

  // Initialized Joints
  const std::vector<std::string>
     &joint_names = Utility::get_variable<std::vector<std::string> >("init.joint.id");
  const std::vector<int>
     &joint_dofs = Utility::get_variable<std::vector<int> >("init.joint.dofs");
  const std::vector<double>
    &joints_start = Utility::get_variable<std::vector<double> >("init.joint.q");

  std::map<std::string, std::vector<double> > init_q;
  for(int i=0,ii=0;i<joint_names.size();i++){
    init_q[joint_names[i]] = std::vector<double>(joint_dofs[i]);
    for(int j=0;j<joint_dofs[i];j++,ii++){
      init_q[joint_names[i]][j] = joints_start[ii];
    }
    OUTLOG(init_q[joint_names[i]],joint_names[i]+"_zero",logINFO);
  }

  set_data<std::map<std::string, std::vector<double> > >("init.q",init_q);
  
  // Initialize base
  const std::vector<double>
    &base_start = Utility::get_variable<std::vector<double> >("init.base.x");

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

  unlock_state();
  set_joint_value(position,init_q);
  set_base_value(position,init_base);
  lock_state();
  OUT_LOG(logDEBUG) << "<< Robot::init_robot(.)";

}
