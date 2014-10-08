#include <robot.h>
#include <utilities.h>

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
   M.resize(NDOFS,NDOFS);
//   fext.resize(NDOFS);
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

void EndEffector::init(Robot* robot){
  Moby::JointPtr joint_ptr = link->get_inner_joint_explicit();
  Moby::RigidBodyPtr rb_ptr = link;
  OUT_LOG(logDEBUG) << id ;
  chain_bool.resize(joint_names_.size());
  rb_ptr = joint_ptr->get_inboard_link();
  while (rb_ptr->id.substr(0,4).compare("BODY") != 0){
    OUT_LOG(logDEBUG) << rb_ptr->id ;
    for(int j=0;j<joint_names_.size();j++){
      if(joint_ptr->id.compare(joint_names_[j].substr(1,joint_names_[j].size())) == 0){
        if(robot->get_active_joints()[joint_names_[j]]){
          OUT_LOG(logDEBUG) << "  " << j <<  " "<< joint_ptr->id;
          chain.push_back(j);
          chain_bool[j] = true;
        } else {
          OUT_LOG(logDEBUG) << "DISABLED:  " << j <<  " "<< joint_ptr->id;
        }
      }
    }
    OUT_LOG(logDEBUG) ;
    rb_ptr = joint_ptr->get_inboard_link();
    joint_ptr = rb_ptr->get_inner_joint_explicit();
  }
  OUT_LOG(logDEBUG) ;
  OUT_LOG(logDEBUG) ;

  active = false;
}

void Robot::update(){
  q   = generalized_q.segment(0,NUM_JOINT_DOFS);
  qd  = generalized_qd.segment(0,NUM_JOINT_DOFS);
  qdd = generalized_qdd.segment(0,NUM_JOINT_DOFS);

  abrobot_->reset_accumulators();
  abrobot_->set_generalized_coordinates(Moby::DynamicBody::eEuler,generalized_q);
  abrobot_->set_generalized_velocity(Moby::DynamicBody::eSpatial,generalized_qd);
  abrobot_->update_link_poses();
  abrobot_->update_link_velocities();
  update_poses();

//  abrobot_->set_generalized_acceleration(generalized_qdd);
  for(int i=0,ii=0;i<NUM_JOINTS;i++){
    if(joints_[i])
    for(int j=0;j<joints_[i]->num_dof();j++,ii++){
      joints_[i]->qdd[j] = generalized_qdd[ii];
    }
  }
  abrobot_->get_base_link()->set_accel(Ravelin::SAcceld(generalized_qdd.segment(NUM_JOINT_DOFS,NDOFS)));
//  abrobot_->add_generalized_force(generalized_fext);
  for(int i = 0;i<NUM_EEFS;i++){
    Ravelin::Pose3d * fp;
    EndEffector& foot =  eefs_[i];

    fp = new Ravelin::Pose3d(
                 Ravelin::Quatd::identity(),
                 Ravelin::Pose3d::transform_point(environment_frame,Ravelin::Vector3d(0,0,0,foot.link->get_pose())).data(),
                 environment_frame
               );
    fp->update_relative_pose(Moby::GLOBAL);
    // Impulse is always in global orientation
    foot.frame_environment = boost::shared_ptr<const Ravelin::Pose3d>(fp);

    fp = new Ravelin::Pose3d(
           Ravelin::Quatd::identity(),
           Ravelin::Pose3d::transform_point(base_frame,Ravelin::Vector3d(0,0,0,foot.link->get_pose())).data(),
           base_frame
         );
    fp->update_relative_pose(Moby::GLOBAL);
    foot.frame_robot_base = boost::shared_ptr<const Ravelin::Pose3d>(fp);
    ////    foot.link->apply_impulse(Ravelin::SMomentumd(foot.impulse,Ravelin::Vector3d(0,0,0),foot.impulse_frame));
//    foot.link->add_force(Ravelin::SForced(foot.impulse/0.001,Ravelin::Vector3d(0,0,0),foot.impulse_frame));
  }
//  abrobot_->calc_fwd_dyn();

  NC = 0;
  for (unsigned i=0; i< NUM_EEFS;i++)
    if(eefs_[i].active)
      NC+=eefs_[i].point.size();

  // fetch robot state vectors
  calc_contact_jacobians(N,D,R);
  calc_workspace_jacobian(Rw);

  // Get robot dynamics state
  // SRZ: Very Heavy Computation
  // SRZ: updating generalized_fext disabled (for now)
  // generalized_fext is supplied by Sim (controller input)
  calculate_dyn_properties(M,generalized_fext);
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
  Utility::quat2TaitBryanZ(base_link_frame->q,roll_pitch_yaw);

  // preserve yaw
  Ravelin::AAngled yaw(0,0,1,roll_pitch_yaw[2]);
  base_horizontal_frame = boost::shared_ptr<const Ravelin::Pose3d>(new Ravelin::Pose3d(yaw,base_link_frame->x,Moby::GLOBAL));

//  base_frame = base_horizontal_frame;//boost::shared_ptr<Ravelin::Pose3d>( new Ravelin::Pose3d(base_horizontal_frame->q,Ravelin::Origin3d(Ravelin::Pose3d::transform_point(base_link_frame,center_of_mass_x)),base_link_frame));
  base_frame = base_link_frame;

  for(int i=0;i<NUM_EEFS;i++)
    eefs_[i].origin.pose = base_frame;
}

void Robot::reset_contact(){
  NC = 0;
  for(int i=0;i<eefs_.size();i++){
    eefs_[i].active = false;
    eefs_[i].point.clear();
    eefs_[i].normal.clear();
    eefs_[i].impulse.clear();
    eefs_[i].tan1.clear();
    eefs_[i].tan2.clear();
  }
}
