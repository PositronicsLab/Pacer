/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <robot.h>
#include <utilities.h>
void Robot::calc_com(){
  new_data.center_of_mass_x.set_zero();
  new_data.center_of_mass_x.pose = environment_frame;
  double total_mass=0;
  for(int i=0;i<links_.size();i++){
    double m = links_[i]->get_mass();
    total_mass += m;
    new_data.center_of_mass_x += (Ravelin::Pose3d::transform_point(environment_frame,Ravelin::Vector3d(0,0,0,links_[i]->get_inertial_pose())) *= m);
  }
  new_data.center_of_mass_x /= total_mass;

  boost::shared_ptr<Ravelin::Pose3d> base_com_w(new Ravelin::Pose3d(environment_frame));
  base_com_w->x = Ravelin::Origin3d(new_data.center_of_mass_x);
  Ravelin::SVector6d com_vel = Ravelin::Pose3d::transform(base_com_w, links_[0]->get_velocity());
  new_data.center_of_mass_xd = com_vel.get_upper();

  Ravelin::SAcceld com_acc = Ravelin::Pose3d::transform(base_com_w, links_[0]->get_accel());
  new_data.center_of_mass_xdd = com_acc.get_linear();
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
  Ravelin::Vector3d C(1,0,-new_data.center_of_mass_x[2]/grav,environment_frame);
  new_data.zero_moment_point =
      Ravelin::Vector2d(C.dot(Ravelin::Vector3d(new_data.center_of_mass_x[0],new_data.center_of_mass_xd[0],new_data.center_of_mass_xdd[0],environment_frame)),
                        C.dot(Ravelin::Vector3d(new_data.center_of_mass_x[1],new_data.center_of_mass_xd[1],new_data.center_of_mass_xdd[1],environment_frame)));

  new_data.center_of_mass_x.pose = new_data.center_of_mass_xd.pose = new_data.center_of_mass_xdd.pose = environment_frame;

#ifdef VISUALIZE_MOBY
  // ZMP and COM
  Ravelin::Vector3d CoM_2D(new_data.center_of_mass_x[0],new_data.center_of_mass_x[1],new_data.center_of_mass_x[2]-0.10,environment_frame);
  visualize_ray(CoM_2D,new_data.center_of_mass_x,Ravelin::Vector3d(0,0,1),sim);
//  visualize_ray(CoM_2D + new_data.center_of_mass_xd*0.1,CoM_2D,Ravelin::Vector3d(0.5,0,1),sim);
//  visualize_ray(CoM_2D + new_data.center_of_mass_xd*0.1 + new_data.center_of_mass_xdd*0.01,CoM_2D + new_data.center_of_mass_xd*0.1,Ravelin::Vector3d(1,0,0),sim);
  visualize_ray(CoM_2D+Ravelin::Vector3d(new_data.zero_moment_point[0],new_data.zero_moment_point[1],0,environment_frame)*0.1,CoM_2D,Ravelin::Vector3d(0,1,0),sim);
#endif

}

double Robot::calc_energy(Ravelin::VectorNd& v, Ravelin::MatrixNd& M){
  // Potential Energy
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
//  for(unsigned i=0,ii=0;i< NUM_JOINTS;i++){
//    if(joints_[i])
//    for(unsigned j=0;i<joints_[i]->num_dof();j++,ii++){
//      joints_[i]->q[j] = q[ii];
//      if(!qd.rows() > 0)
//        joints_[i]->qd[j] = qd[ii];
//    }
//  }
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

void Robot::calculate_dyn_properties(Ravelin::MatrixNd& M, Ravelin::VectorNd& fext){
  //   fext.resize(NDOFS);
   M.resize(NDOFS,NDOFS);
   abrobot_->get_generalized_inertia(M);
//   abrobot_->get_generalized_forces(fext);
}

void Robot::compile(){

  NSPATIAL = 6;
  NEULER = 7;
  NDOFS = abrobot_->num_generalized_coordinates(Moby::DynamicBody::eSpatial); // for generalized velocity, forces. accel
  NUM_JOINT_DOFS = NDOFS - NSPATIAL;

  dbrobot_ = boost::dynamic_pointer_cast<Moby::DynamicBody>(abrobot_);
  std::vector<Moby::JointPtr> joints = abrobot_->get_joints();
  joints_.resize(NUM_JOINT_DOFS);

  NUM_FIXED_JOINTS = 0;
  for(unsigned i=0;i<joints.size();i++){
    OUT_LOG(logINFO)  << joints[i]->id;
    if(joints[i]->num_dof() == 0){
      OUT_LOG(logINFO) <<"\tFixed: "<< joints[i]->id;

      NUM_FIXED_JOINTS ++;
      continue;
    }
    joints_[joints[i]->get_coord_index()] = joints[i];
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

void Robot::init_end_effector(EndEffector& eef){
  eef.id = eef.link->id;

  Moby::JointPtr joint_ptr = eef.link->get_inner_joint_explicit();
  Moby::RigidBodyPtr rb_ptr = eef.link;
  OUT_LOG(logDEBUG) << eef.id ;
  eef.chain_bool.resize(joint_names_.size());
  rb_ptr = joint_ptr->get_inboard_link();
  while (rb_ptr->id.substr(0,4).compare("BODY") != 0){
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
    OUT_LOG(logDEBUG) ;
    rb_ptr = joint_ptr->get_inboard_link();
  }
  OUT_LOG(logDEBUG) ;
  OUT_LOG(logDEBUG) ;

  eef.active = false;
}

void Robot::update(
    const Ravelin::VectorNd& generalized_q_in,
    const Ravelin::VectorNd& generalized_qd_in,
    const Ravelin::VectorNd& generalized_qdd_in,
    const Ravelin::VectorNd& generalized_fext_in){
  new_data.generalized_q = generalized_q_in;
  new_data.generalized_qd = generalized_qd_in;
  new_data.generalized_qdd = generalized_qdd_in;
  new_data.generalized_fext = generalized_fext_in;

  new_data.q   = new_data.generalized_q.segment(0,NUM_JOINT_DOFS);
  new_data.qd  = new_data.generalized_qd.segment(0,NUM_JOINT_DOFS);
  new_data.qdd = new_data.generalized_qdd.segment(0,NUM_JOINT_DOFS);

  abrobot_->reset_accumulators();
  abrobot_->set_generalized_coordinates(Moby::DynamicBody::eEuler,new_data.generalized_q);
  abrobot_->set_generalized_velocity(Moby::DynamicBody::eSpatial,new_data.generalized_qd);
  abrobot_->update_link_poses();
  abrobot_->update_link_velocities();
  update_poses();

//  abrobot_->set_generalized_acceleration(generalized_qdd);
  for(int i=0,ii=0;i<NUM_JOINTS;i++){
    if(joints_[i])
    for(int j=0;j<joints_[i]->num_dof();j++,ii++){
      joints_[i]->qdd[j] = new_data.generalized_qdd[ii];
    }
  }
  abrobot_->get_base_link()->set_accel(Ravelin::SAcceld(new_data.generalized_qdd.segment(NUM_JOINT_DOFS,NDOFS)));
//  abrobot_->add_generalized_force(generalized_fext);
//  abrobot_->calc_fwd_dyn();

  // fetch robot state vectors
  calc_contact_jacobians(new_data.N,new_data.D,new_data.R);
//  calc_workspace_jacobian(Rw);
  int NC = new_data.N.columns();
  // Get robot dynamics state
  // SRZ: Very Heavy Computation
  // SRZ: updating generalized_fext disabled (for now)
  // generalized_fext is supplied by Sim (controller input)
  calculate_dyn_properties(new_data.M,new_data.generalized_fext);
//  calc_energy(generalized_qd,M);
  calc_com();

  center_of_contact.point.resize(1);
  center_of_contact.normal.resize(1);
  if(NC != 0) {
    center_of_contact.point[0] = Ravelin::Vector3d::zero();
    center_of_contact.normal[0] = Ravelin::Vector3d::zero();
    center_of_contact.point[0].pose = environment_frame;
    center_of_contact.normal[0].pose = environment_frame;
    for(int i=0;i<NUM_EEFS;i++){
      // set gait centers
      if(eefs_[i].active){
        for(int j=0;j<eefs_[i].point.size();j++){
          center_of_contact.point[0] += Ravelin::Vector3d(eefs_[i].point[j].data(),environment_frame)/NC;
          center_of_contact.normal[0] += Ravelin::Vector3d(eefs_[i].normal[j].data(),environment_frame)/NC;
        }
      }
    }
    center_of_contact.active = true;
  }
  else {
    center_of_contact.normal[0] = Ravelin::Vector3d(0,0,1,environment_frame);
    center_of_contact.active = false;
  }
#ifdef VISUALIZE_MOBY
  draw_pose(*base_frame,sim,0.8);
  draw_pose(*base_horizontal_frame,sim,1.5);
  draw_pose(Moby::GLOBAL,sim,1.0);
#endif

#ifdef VISUALIZE_MOBY
       // CONTACTS
       if(NC != 0){
//         std::vector<const EndEffector * > active_eefs;
//         if(eefs_[0].active)
//           active_eefs.push_back(&eefs_[0]);
//         if(eefs_[1].active)
//           active_eefs.push_back(&eefs_[1]);
//         if(eefs_[3].active)
//           active_eefs.push_back(&eefs_[3]);
//         if(eefs_[2].active)
//           active_eefs.push_back(&eefs_[2]);

//         // Draw Contact Polygon
//         for(int i=0;i<NC;i++){
//           visualize_ray(active_eefs[i]->point + Ravelin::Vector3d(0,0,0.001),
//                         active_eefs[(i+1)%NC]->point  + Ravelin::Vector3d(0,0,0.001),
//                         Ravelin::Vector3d(0.5,0.5,1),
//                         sim);
//         }
         // Draw all Contacts
//         for(int i=0;i<NC;i++)
//           for(int j=0;j<active_eefs[i]->contacts.size();j++){
//             visualize_ray(active_eefs[i]->contacts[j],
//                           active_eefs[i]->point,
//                           Ravelin::Vector3d(1,1,1),
//                           sim);
//           }
         visualize_ray(center_of_contact.point[0],
                    center_of_contact.normal[0]*0.1 + center_of_contact.point[0],
                    Ravelin::Vector3d(1,1,0),
                    sim);
       }

#endif

       data = &new_data;
}

void Robot::update_poses(){
  // Get base frame
  base_link_frame = links_[0]->get_pose();

  Ravelin::Matrix3d R_base(base_link_frame->q),
      R_pitch(cos(displace_base_link[4]),0,sin(displace_base_link[4]),0,1,0,-sin(displace_base_link[4]),0,cos(displace_base_link[4])),
//      R_yaw(cos(displace_base_link[4]),0,sin(displace_base_link[4]),0,1,0,-sin(displace_base_link[4]),0,cos(displace_base_link[4])),
//      R_roll(cos(displace_base_link[4]),0,sin(displace_base_link[4]),0,1,0,-sin(displace_base_link[4]),0,cos(displace_base_link[4])),
      new_R;
  OUT_LOG(logINFO) << "Rotating base_link_frame saggitally by: theta = " << -displace_base_link[4];
  R_base.mult_transpose(R_pitch,new_R);
  base_link_frame = boost::shared_ptr<const Ravelin::Pose3d>(
                         new Ravelin::Pose3d( new_R,base_link_frame->x,Moby::GLOBAL));
  Utility::quat2TaitBryanZ(base_link_frame->q,new_data.roll_pitch_yaw);

  // preserve yaw
  Ravelin::AAngled yaw(0,0,1,new_data.roll_pitch_yaw[2]);
  base_horizontal_frame = boost::shared_ptr<const Ravelin::Pose3d>(new Ravelin::Pose3d(yaw,base_link_frame->x,Moby::GLOBAL));

//  base_frame = base_horizontal_frame;//boost::shared_ptr<Ravelin::Pose3d>( new Ravelin::Pose3d(base_horizontal_frame->q,Ravelin::Origin3d(Ravelin::Pose3d::transform_point(base_link_frame,center_of_mass_x)),base_link_frame));
  base_frame = base_link_frame;

  for(int i=0;i<NUM_EEFS;i++)
    eefs_[i].origin.pose = base_frame;
}

void Robot::reset_contact(){
  for(int i=0;i<eefs_.size();i++){
    eefs_[i].active = false;
    eefs_[i].point.clear();
    eefs_[i].normal.clear();
    eefs_[i].impulse.clear();
    eefs_[i].tan1.clear();
    eefs_[i].tan2.clear();
  }
}




// ============================================================================
// ===========================  BEGIN ROBOT INIT  =============================
#include <CVars/CVar.h>

#if defined(VISUALIZE_MOBY) && defined(USE_GLCONSOLE)
# include <thread>
# include <GLConsole/GLConsole.h>
  GLConsole theConsole;
  extern void init_glconsole();
  std::thread * tglc;
#endif

#include <Moby/XMLReader.h>

void Robot::init(){

#if defined(VISUALIZE_MOBY) && defined(USE_GLCONSOLE)
   tglc = new std::thread(init_glconsole);
#endif
  // ================= LOAD SCRIPT DATA ==========================
  Utility::load_variables("INIT/startup.xml");
  std::string robot_start_file = CVarUtils::GetCVarRef<std::string>("robot");
  std::cerr << "Using Robot: " << robot_start_file << std::endl;
  Utility::load_variables("INIT/startup-"+robot_start_file+".xml");

  // ================= SETUP LOGGING ==========================

  std::string LOG_TYPE = CVarUtils::GetCVarRef<std::string>("logging");

  std::cerr << LOG_TYPE << std::endl;
  FILELog::ReportingLevel() =
      FILELog::FromString( (!LOG_TYPE.empty()) ? LOG_TYPE : "INFO");

  OUT_LOG(logDEBUG1) << "Log Type : " << LOG_TYPE;
  OUT_LOG(logDEBUG1) << "logDEBUG1";
  OUT_LOG(logINFO) << "logINFO";
  OUT_LOG(logDEBUG) << "logDEBUG";
  OUT_LOG(logDEBUG1) << "logDEBUG1";

  // ================= BUILD ROBOT ==========================
  /// The map of objects read from the simulation XML file
  std::map<std::string, Moby::BasePtr> READ_MAP;
  READ_MAP = Moby::XMLReader::read(std::string("MODELS/"+robot_start_file+".xml"));
  for (std::map<std::string, Moby::BasePtr>::const_iterator i = READ_MAP.begin();
       i !=READ_MAP.end(); i++)
  {
    // find the robot reference
    if (!abrobot_)
    {
      abrobot_ = boost::dynamic_pointer_cast<Moby::RCArticulatedBody>(i->second);
    }
  }

  compile();

  // ================= SET UP END EFFECTORS ==========================

  eef_names_
      = CVarUtils::GetCVarRef<std::vector<std::string> >("init.end-effector.id");

  std::vector<double> &eefs_start
      = CVarUtils::GetCVarRef<std::vector<double> >("init.end-effector.x");

  static std::vector<std::string>
     &joint_names = CVarUtils::GetCVarRef<std::vector<std::string> >("init.joint.id");

 static std::vector<double>
    &joints_start = CVarUtils::GetCVarRef<std::vector<double> >("init.joint.q"),
    &torque_limits = CVarUtils::GetCVarRef<std::vector<double> >("init.joint.max-torque"),
    &base_start = CVarUtils::GetCVarRef<std::vector<double> >("init.base.x");

 static std::vector<int>
    &active_joints = CVarUtils::GetCVarRef<std::vector<int> >("init.joint.active");

 const double* data = &base_start.front();
 displace_base_link = Ravelin::SVector6d(data);

 OUTLOG(joint_names,"joint_names",logDEBUG1);
 OUTLOG(joints_start,"joints_start",logDEBUG1);

 // MAKE SURE DATA PARSED PROPERLY

// assert(joint_names.size() == joints_.size());
 assert(joint_names.size() == joints_start.size());
 assert(joint_names.size() == torque_limits.size());

  std::map<std::string, double> torque_limits_;
  for(int i=0,ii=0;i<NUM_JOINTS;i++){
    if(joints_[i])
    for(int j=0;j<joints_[i]->num_dof();j++,ii++){
      OUT_LOG(logDEBUG) << joint_names[ii] << " " << ((active_joints[ii] == 0)? "false":"true") << std::endl;
      active_joints_[joint_names[ii]] = (active_joints[ii] == 0)? false:true;
      q0_[joint_names[ii]] = joints_start[ii];
      torque_limits_[joint_names[ii]] = torque_limits[ii];
    }
  }

  // push into robot
  torque_limits_l.resize(NUM_JOINT_DOFS);
  torque_limits_u.resize(NUM_JOINT_DOFS);
  for(int i=0,ii=0;i<NUM_JOINTS;i++){
    if(joints_[i])
    for(int j=0;j<joints_[i]->num_dof();j++,ii++){
//      assert(joint_names[ii].substr(0,2).compare(joints_[ii-j]->id.substr(0,2)) &&
//             joint_names[ii].substr(joint_names[ii].size()-2,1).compare(joints_[ii-j]->id.substr(joints_[ii-j]->id.size()-2,1)));
      OUT_LOG(logINFO)<< "torque_limit: " << joints_[ii-j]->id << " = " <<  torque_limits_[joint_names[ii]];
      torque_limits_l[ii] = -torque_limits_[std::to_string(j)+joints_[i]->id];
      torque_limits_u[ii] =  torque_limits_[std::to_string(j)+joints_[i]->id];
    }
  }
  OUTLOG(torque_limits_l,"torque_limits_l",logDEBUG1);
  OUTLOG(torque_limits_u,"torque_limits_u",logDEBUG1);

  // Initialize Foot Data Structures
  OUT_LOG(logINFO)<< eef_names_.size() << " end effectors LISTED:" ;
  for(unsigned j=0;j<eef_names_.size();j++){
    for(unsigned i=0;i<links_.size();i++){
      if(eef_names_[j].compare(links_[i]->id) == 0){
        EndEffector eef;
        eef.link = links_[i];
        init_end_effector(eef);
        eefs_.push_back(eef);

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

