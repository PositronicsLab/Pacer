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
  center_of_mass_x.pose = environment_frame;
  double total_mass=0;
  for(int i=0;i<links_.size();i++){
    double m = links_[i]->get_mass();
    total_mass += m;
    center_of_mass_x += (Ravelin::Pose3d::transform_point(environment_frame,Ravelin::Vector3d(0,0,0,links_[i]->get_inertial_pose())) *= m);
  }
  center_of_mass_x /= total_mass;

  boost::shared_ptr<Ravelin::Pose3d> base_com_w = boost::shared_ptr<Ravelin::Pose3d>(new Ravelin::Pose3d(environment_frame));
  base_com_w->x = Ravelin::Origin3d(center_of_mass_x);
  Ravelin::SVector6d com_vel = Ravelin::Pose3d::transform(base_com_w, links_[0]->get_velocity());
  
  Ravelin::Vector3d center_of_mass_xd = com_vel.get_upper();

  Ravelin::SAcceld com_acc = Ravelin::Pose3d::transform(base_com_w, links_[0]->get_accel());
  Ravelin::Vector3d center_of_mass_xdd = com_acc.get_linear();

  center_of_mass_xd.pose = center_of_mass_xdd.pose = environment_frame;

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
  Ravelin::Vector3d C(1,0,-center_of_mass_x[2]/grav,environment_frame);
  Ravelin::Vector3d zero_moment_point =
      Ravelin::Vector3d(C.dot(Ravelin::Vector3d(center_of_mass_x[0],center_of_mass_xd[0],center_of_mass_xdd[0],environment_frame)),
                        C.dot(Ravelin::Vector3d(center_of_mass_x[1],center_of_mass_xd[1],center_of_mass_xdd[1],environment_frame)),
                        0,environment_frame);
  set_data<Ravelin::Vector3d>("zero_moment_point",zero_moment_point);

  // ZMP and COM
  Ravelin::Vector3d CoM_2D(center_of_mass_x[0],center_of_mass_x[1],center_of_mass_x[2]-0.10,environment_frame);
  visualize.push_back( Pacer::VisualizablePtr( new Ray(CoM_2D,center_of_mass_x,Ravelin::Vector3d(0,0,1))));
//  visualize.push_back( Pacer::VisualizablePtr( new Ray(CoM_2D + center_of_mass_xd*0.1,CoM_2D,Ravelin::Vector3d(0.5,0,1)));
//  visualize.push_back( Pacer::VisualizablePtr( new Ray(CoM_2D + center_of_mass_xd*0.1 + center_of_mass_xdd*0.01,CoM_2D + center_of_mass_xd*0.1,Ravelin::Vector3d(1,0,0)));
  visualize.push_back( Pacer::VisualizablePtr( new Ray(CoM_2D+Ravelin::Vector3d(zero_moment_point[0],zero_moment_point[1],0,environment_frame)*0.1,CoM_2D,Ravelin::Vector3d(0,1,0))));
}

double Robot::calc_energy(const Ravelin::VectorNd& v, const Ravelin::MatrixNd& M) const {
  // Potential Energy: this calculation assumes that ground is always at zero
  double PE = 0;
  for(int i=0;i<links_.size();i++){
     Moby::RigidBody& link = *links_[i];
     double m = link.get_mass();
     Ravelin::Pose3d link_com = *link.get_inertial_pose();
     link_com.update_relative_pose(Moby::GLOBAL);
     PE += link_com.x[2] * m * grav;
  }
  M.mult(v, workv_);
  double KE = workv_.dot(v)*0.5;
//#ifndef NDEBUG
  std::cout << "KE = " << KE << ", PE = " << PE << std::endl;
  std::cout << "Total Energy = " << (KE + PE) << std::endl;
//#endif
  return (KE + PE);
  // Kinetic Energy
}

void Robot::set_model_state(const Ravelin::VectorNd& q,const Ravelin::VectorNd& qd){
  Ravelin::VectorNd set_q,set_qd;
  abrobot_->get_generalized_coordinates(Moby::DynamicBody::eEuler,set_q);
  abrobot_->get_generalized_velocity(Moby::DynamicBody::eSpatial,set_qd);

  set_q.set_sub_vec(0,q);
  set_qd.set_sub_vec(0,qd);

  abrobot_->set_generalized_coordinates(Moby::DynamicBody::eEuler,set_q);
  abrobot_->set_generalized_velocity(Moby::DynamicBody::eSpatial,set_qd);
  abrobot_->update_link_poses();
  abrobot_->update_link_velocities();
}

void Robot::calculate_generalized_inertia(const Ravelin::VectorNd& q, Ravelin::MatrixNd& M){
  set_model_state(q);
  M.resize(NDOFS,NDOFS);
  abrobot->get_generalized_inertia(M);
}

void Robot::compile(){
  NDOFS = abrobot_->num_generalized_coordinates(Moby::DynamicBody::eSpatial);
  NUM_JOINT_DOFS = NDOFS - NSPATIAL;

  _dbrobot = boost::dynamic_pointer_cast<Moby::DynamicBody>(_abrobot);
  std::vector<Moby::JointPtr> joints = abrobot_->get_joints();
    
  for(unsigned i=0;i<joints.size();i++){
    OUT_LOG(logINFO) << joints[i]->id << ", dofs = " << joints[i]->num_dof();
    if(joints[i]->num_dof() != 0){
      _id_joint_map[joints[i]->id] = joints[i];
    }
  }

  // Init robot params based on joint map
  std
  for(unsigned i=0;i<_joints.size();i++){
    for(int j=0;j<_joints[i]->num_dof();j++){
      _id_coord_map
    }
  }


  std::map<unit_e , std::map<int, Ravelin::VectorNd > >::iterator it;
  std::map<int, Ravelin::VectorNd >::iterator jt;
  for(it=_state.begin();it!=_state.end();it++){
    for(jt=(*it).second.begin();jt!=(*it).second.end();jt++)
      (*jt).second.set_zero();
  }

  OUT_LOG(logINFO) <<"\tNum Fixed: "<< NUM_FIXED_JOINTS;
  NUM_JOINTS = joints_.size();

  for(int i=0,ii=0;i<NUM_JOINTS;i++){
    if(joints_[i])
    for(int j=0;j<joints_[i]->num_dof();j++,ii++){
      OUT_LOG(logINFO)  << joints_[i]->id;
      joint_names_.push_back(std::to_string(j) + joints_[i]->id);
      OUT_LOG(logINFO)  << joints_[i]->get_coord_index() << " "
                      << joint_names_[ii];
    }
  }
  // Set up link references
  links_ = abrobot_->get_links();

  // set up initial stance if it exists
//  for(int i=0;i<NUM_FIXED_JOINTS;i++)
//    joints_.pop_back();
  NUM_LINKS = links_.size();

  environment_frame = boost::shared_ptr<const Ravelin::Pose3d>( new Ravelin::Pose3d(Ravelin::Quatd::identity(),Ravelin::Origin3d(0,0,0),Moby::GLOBAL));
//  environment_frame->x = Ravelin::Origin3d(0,0,0);
//  environment_frame->q.set_identity();
}

void Robot::init_end_effector(end_effector_s& eef){
  eef.id = eef.link->id;

  Moby::JointPtr joint_ptr = eef.link->get_inner_joint_explicit();
  Moby::RigidBodyPtr rb_ptr = eef.link;
  OUT_LOG(logDEBUG) << eef.id ;
  eef.chain_bool.resize(joint_names_.size());
  rb_ptr = joint_ptr->get_inboard_link();
  while (rb_ptr != abrobot_->get_base_link()) {
    OUT_LOG(logDEBUG) << "  " << rb_ptr->id;
    joint_ptr = rb_ptr->get_inner_joint_explicit();
    OUT_LOG(logDEBUG) << "  " << joint_ptr->id;

    for(int j=0;j<joint_names_.size();j++){
      if(joint_ptr->id.compare(joint_names_[j].substr(1,joint_names_[j].size())) == 0){
        if(get_active_joints()[joint_names_[j]]){
          OUT_LOG(logDEBUG) << "  " << j <<  " "<< joint_ptr->id;
          eef.chain.push_back(j);
          eef.chain_bool[j] = true;
        } else {
          OUT_LOG(logDEBUG) << "DISABLED:  " << j <<  " "<< joint_ptr->id;
        }
      }
    }
    OUT_LOG(logDEBUG);
    rb_ptr = joint_ptr->get_inboard_link();
  }
  OUT_LOG(logDEBUG);
}

void Robot::update(){
	
}

void Robot::update(
    const Ravelin::VectorNd& generalized_q,
    const Ravelin::VectorNd& generalized_qd,
    const Ravelin::VectorNd& generalized_qdd,
    const Ravelin::VectorNd& generalized_fext){

  set_data<Ravelin::VectorNd>("generalized_q", generalized_q);
  set_data<Ravelin::VectorNd>("generalized_qd", generalized_qd);
  set_data<Ravelin::VectorNd>("generalized_qdd", generalized_qdd);
  set_data<Ravelin::VectorNd>("generalized_fext", generalized_fext);

  const Ravelin::VectorNd& q 
	= set_data<Ravelin::VectorNd>("q",generalized_q.segment(0,NUM_JOINT_DOFS));
  const Ravelin::VectorNd& qd 
	= set_data<Ravelin::VectorNd>("qd",generalized_qd.segment(0,NUM_JOINT_DOFS));
  const Ravelin::VectorNd& qdd 
	= set_data<Ravelin::VectorNd>("qdd",generalized_qdd.segment(0,NUM_JOINT_DOFS));

  abrobot_->reset_accumulators();
  abrobot_->set_generalized_coordinates(Moby::DynamicBody::eEuler,generalized_q);
  abrobot_->set_generalized_velocity(Moby::DynamicBody::eSpatial,generalized_qd);
  abrobot_->update_link_poses();
  abrobot_->update_link_velocities();
  
  update_poses();
  
  boost::shared_ptr<Ravelin::Pose3d> base_link_frame = get_data<boost::shared_ptr<Ravelin::Pose3d> >("base_link_frame");
  boost::shared_ptr<Ravelin::Pose3d> base_horizontal_frame =
      boost::shared_ptr<Ravelin::Pose3d>(new Ravelin::Pose3d(get_data<Ravelin::Pose3d>("base_horizontal_frame")));
#ifndef NDEBUG
  visualize.push_back( Pacer::VisualizablePtr( new Pose(*(base_frame.get()),0.8)));
  visualize.push_back( Pacer::VisualizablePtr( new Pose(*(base_horizontal_frame.get()),1.5)));
  visualize.push_back( Pacer::VisualizablePtr( new Pose(Moby::GLOBAL,1.0)));
#endif
//  abrobot_->set_generalized_acceleration(generalized_qdd);
  for(int i=0,ii=0;i<NUM_JOINTS;i++){
    if(joints_[i])
    for(int j=0;j<joints_[i]->num_dof();j++,ii++){
      joints_[i]->qdd[j] = generalized_qdd[ii];
    }
  }
  abrobot_->get_base_link()->set_accel(Ravelin::SAcceld(generalized_qdd.segment(NUM_JOINT_DOFS,NDOFS)));
//  abrobot_->add_generalized_force(generalized_fext);
//  abrobot_->calc_fwd_dyn();

  // fetch robot state vectors
  Ravelin::MatrixNd N,S,T;
  calc_contact_jacobians(_contacts,N,S,T);

  set_data<Ravelin::MatrixNd>("N",N);
  set_data<Ravelin::MatrixNd>("S",S);
  set_data<Ravelin::MatrixNd>("T",T);

  // Get robot dynamics state
  // SRZ: Very Heavy Computation
  // SRZ: updating generalized_fext disabled (for now)
  // generalized_fext is supplied by Sim (controller input)
  Ravelin::MatrixNd M;
  Ravelin::VectorNd fext;
  calculate_dyn_properties(M,fext);
  set_data<Ravelin::VectorNd>("generalized_inertia",M);
  
  calc_com();

#ifndef NDEBUG
  calc_energy(generalized_qd,M);
#endif

}

void Robot::update_poses(){
  // Get base frame
  boost::shared_ptr<const Ravelin::Pose3d> base_link_frame = links_[0]->get_pose();

  Ravelin::Matrix3d R_base(base_link_frame->q),
      R_pitch = Ravelin::Matrix3d::rot_Y(displace_base_link[4]),
      new_R;
  OUT_LOG(logINFO) << "Rotating base_link_frame saggitally by: theta = " << -displace_base_link[4];
  R_base.mult(R_pitch,new_R);
  base_link_frame = boost::shared_ptr<const Ravelin::Pose3d>(
                         new Ravelin::Pose3d( new_R,base_link_frame->x,Moby::GLOBAL));
  Ravelin::Origin3d roll_pitch_yaw;
  base_link_frame->q.to_rpy(roll_pitch_yaw);


  // preserve yaw
  base_horizontal_frame
      = boost::shared_ptr<const Ravelin::Pose3d>(
          new Ravelin::Pose3d(
            Ravelin::Matrix3d::rot_Z(roll_pitch_yaw[2]),
          base_link_frame->x,Moby::GLOBAL));

  set_data<Ravelin::Pose3d>("base_link_frame",base_link_frame);
  set_data<Ravelin::Origin3d>("base_link_frame",roll_pitch_yaw);
  set_data<Ravelin::Pose3d>("base_horizontal_frame",base_horizontal_frame);
}

void Robot::reset_contact(){
	std::map<std::string,boost::shared_ptr<end_effector_s> >::iterator it = _end_effector_map.begin();
	for(;it != _end_effector_map.end();it++)
      (*it).second->contacts.clear();
}

// ============================================================================
// ===========================  BEGIN ROBOT INIT  =============================
#include <CVars/CVar.h>
#include <stdlib.h>     /* getenv */
#include <boost/filesystem.hpp>

#include <Moby/SDFReader.h>
#include <Moby/XMLReader.h>

 void Robot::init_robot(){
  // ================= LOAD SCRIPT DATA ==========================
  std::string pPath(getenv ("PACER_MODELS_PATH"));
  OUT_LOG(logDEBUG) << "PACER_MODELS_PATH = " << pPath;

  // ================= SETUP LOGGING ==========================

  std::string LOG_TYPE = CVarUtils::GetCVarRef<std::string>("logging");

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
  std::string robot_model_file = CVarUtils::GetCVarRef<std::string>("robot-model");
  
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
      if (!abrobot_)
      {
        abrobot_ = boost::dynamic_pointer_cast<Moby::RCArticulatedBody>(i->second);
      }
    }

    if (!abrobot_){
      throw std::runtime_error("could not find RCArticulatedBody for robot SDF");
    }
  } else if(model_type.compare(".xml") == 0){
    std::cerr << "look for model: " << robot_model_file << std::endl;

    std::map<std::string, Moby::BasePtr> READ_MAP = Moby::XMLReader::read(std::string(robot_model_file));
    for (std::map<std::string, Moby::BasePtr>::const_iterator i = READ_MAP.begin();
       i !=READ_MAP.end(); i++)
    {
    // find the robot reference
    if (!abrobot_)
    {
      abrobot_ = boost::dynamic_pointer_cast<Moby::RCArticulatedBody>(i->second);
    }
    }

    if (!abrobot_){
      throw std::runtime_error("could not find RCArticulatedBody for robot XML");
    }
  } else {
    throw std::runtime_error("Robot model file has unknown extension : " + model_type);
  }
  compile();

  // ================= SET UP END EFFECTORS ==========================

  eef_names_
      = CVarUtils::GetCVarRef<std::vector<std::string> >("init.end-effector.id");

  std::vector<std::string>
     &joint_names = CVarUtils::GetCVarRef<std::vector<std::string> >("init.joint.id");

	std::vector<double>
		&joints_start = CVarUtils::GetCVarRef<std::vector<double> >("init.joint.q"),
		&torque_limits = CVarUtils::GetCVarRef<std::vector<double> >("init.joint.max-torque"),
		&base_start = CVarUtils::GetCVarRef<std::vector<double> >("init.base.x");

 std::vector<int>
    &active_joints = CVarUtils::GetCVarRef<std::vector<int> >("init.joint.active");


 displace_base_link = Ravelin::SVector6d(
     Ravelin::VectorNd(base_start.size(),&base_start[0]));

 OUTLOG(joint_names,"joint_names",logDEBUG1);
 OUTLOG(joints_start,"joints_start",logDEBUG1);

 assert(joint_names.size() == joints_start.size());

  for(int ii=0;ii<joint_names.size();ii++){
      OUT_LOG(logDEBUG) << joint_names[ii] << " " << ((active_joints[ii] == 0)? "false":"true") << std::endl;

      active_joints_[joint_names[ii]] = (active_joints[ii] == 0)? false:true;

      q0_[joint_names[ii]] = joints_start[ii];
  }

  // Initialize Foot Data Structures
  OUT_LOG(logINFO)<< eef_names_.size() << " end effectors LISTED:" ;
  for(unsigned j=0;j<eef_names_.size();j++){
    for(unsigned i=0;i<links_.size();i++){
      if(eef_names_[j].compare(links_[i]->id) == 0){
        end_effector_s eef;
        eef.link = links_[i];
        init_end_effector(eef);
        eefs_.push_back(eef);
        eefs_map_[eef_names_[j]] = &eefs_[eefs_.size()-1];

        break;
      }
    }
  }
  NUM_EEFS = eefs_.size();

  OUT_LOG(logINFO)<< "NUM_EEFS: " << NUM_EEFS ;
  OUT_LOG(logINFO)<< "N_FIXED_JOINTS: " << NUM_FIXED_JOINTS ;
  OUT_LOG(logINFO)<< "NUM_JOINTS: " << NUM_JOINTS ;
  OUT_LOG(logINFO)<< "NDOFS: " << NDOFS ;
  OUT_LOG(logINFO)<< "NSPATIAL: " << NSPATIAL ;
  OUT_LOG(logINFO)<< "NEULER: " << NEULER ;
}
/*
boost::shared_ptr<const RobotData> Robot::gen_vars_from_model(const std::map<std::string, double>& q,
    const std::map<std::string, double>& qd,
    boost::shared_ptr<const Ravelin::Pose3d> base_pose,
    const Ravelin::SVector6d& base_xd,
    boost::shared_ptr<Robot>& robot)
{
  std::vector<std::string>& joint_names = robot->get_joint_names();
  int N = q.size();

  Ravelin::VectorNd
      generalized_q(N+7),
      generalized_qd(N+6),
      generalized_qdd(N+6),
      generalized_fext(N+6);
  generalized_q.set_zero();
  generalized_qd.set_zero();
  generalized_qdd.set_zero();
  generalized_fext.set_zero();
  for(int i=0;i<N;i++){
    generalized_q[i] = q.at(joint_names[i]);
    generalized_qd[i] = qd.at(joint_names[i]);
  }

  Ravelin::VectorNd base_x(7);
  base_x.set_sub_vec(0,base_pose->x);
//  base_x.set_sub_vec(3,base_pose->q);
  base_x[3] = base_pose->q.x;
  base_x[4] = base_pose->q.y;
  base_x[5] = base_pose->q.z;
  base_x[6] = base_pose->q.w;
  generalized_q.set_sub_vec(N,base_x);
  generalized_qd.set_sub_vec(N,Ravelin::VectorNd(base_xd.size(),&base_x[0]));
  robot->update(generalized_q,generalized_qd,generalized_qdd,generalized_fext);

  return robot->get_robot_data();
}
*/
