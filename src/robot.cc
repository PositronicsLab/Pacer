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

void Robot::calculate_dyn_properties(Ravelin::MatrixNd& M, Ravelin::VectorNd& fext){
   M.resize(NDOFS,NDOFS);
//   fext.resize(NDOFS);
   abrobot_->get_generalized_inertia(M);
//   abrobot_->get_generalized_forces(fext);
}

void Robot::compile(){

  NSPATIAL = 6;
  NEULER = 7;

  dbrobot_ = boost::dynamic_pointer_cast<Moby::DynamicBody>(abrobot_);
  std::vector<Moby::JointPtr> joints = abrobot_->get_joints();
  joints_.resize(joints.size());

  NUM_FIXED_JOINTS = 0;
  for(unsigned i=0;i<joints.size();i++){
    OUT_LOG(logINFO)  << joints[i]->id;
    if(joints[i]->q.rows() == 0){
      OUT_LOG(logINFO) <<"\tFixed: "<< joints[i]->id;

      NUM_FIXED_JOINTS ++;
      continue;
    }
    joints_[joints[i]->get_coord_index()] = joints[i];
  }

  for(unsigned i=0;i<joints_.size()-NUM_FIXED_JOINTS;i++){
    if(joints_[i]->q.rows() == 0){
      continue;
    }
    joint_names_.push_back(joints_[i]->id);
    OUT_LOG(logINFO)  << joints_[i]->get_coord_index() << " "
              << joints_[i]->id;
  }

  // Set up link references
  links_ = abrobot_->get_links();

  environment_frame = boost::shared_ptr<const Ravelin::Pose3d>( new Ravelin::Pose3d(Ravelin::Quatd::identity(),Ravelin::Origin3d(0,0,0),Moby::GLOBAL));
//  environment_frame->x = Ravelin::Origin3d(0,0,0);
//  environment_frame->q.set_identity();
}

void EndEffector::init(){
  Moby::JointPtr joint_ptr = link->get_inner_joint_explicit();
  Moby::RigidBodyPtr rb_ptr = link;
  OUT_LOG(logDEBUG) << id ;
  chain_bool.resize(joint_names_.size());
  rb_ptr = joint_ptr->get_inboard_link();
  while (rb_ptr->id.compare("ABDOMEN") != 0){
    OUT_LOG(logDEBUG) << rb_ptr->id ;
    for(int j=0;j<joint_names_.size();j++){
      if(joint_ptr->id.compare(joint_names_[j]) == 0){
        OUT_LOG(logDEBUG) << "  " << j <<  " "<< joint_ptr->id;
        chain.push_back(j);
        chain_bool[j] = true;
      }
    }
    OUT_LOG(logDEBUG) ;
    rb_ptr = joint_ptr->get_inboard_link();
    joint_ptr = rb_ptr->get_inner_joint_explicit();
  }
  OUT_LOG(logDEBUG) ;
  OUT_LOG(logDEBUG) ;

  Ravelin::Pose3d pose = *link->get_pose();
  normal = Ravelin::Vector3d(0,0,1);
  point = pose.x;
  active = false;
}

void Robot::update(){
  generalized_q.get_sub_vec(0,NUM_JOINTS,q);
  generalized_qd.get_sub_vec(0,NUM_JOINTS,qd);
  generalized_qdd.get_sub_vec(0,NUM_JOINTS,qdd);

//  abrobot_->reset_accumulators();
  abrobot_->set_generalized_coordinates(Moby::DynamicBody::eEuler,generalized_q);
  abrobot_->set_generalized_velocity(Moby::DynamicBody::eSpatial,generalized_qd);
  abrobot_->update_link_poses();
  abrobot_->update_link_velocities();
//  abrobot_->set_generalized_acceleration(Moby::DynamicBody::eSpatial,generalized_qdd);
//  abrobot_->add_generalized_force(generalized_fext);

  update_poses();

  NC = 0;
  for (unsigned i=0; i< NUM_EEFS;i++)
    if(eefs_[i].active)
      NC++;

  // fetch robot state vectors
  calc_contact_jacobians(N,D,R);
  calc_workspace_jacobian(Rw,base_link_frame);

  // Get robot dynamics state
  // SRZ: Very Heavy Computation
  // SRZ: updating generalized_fext disabled (for now)
  calculate_dyn_properties(M,generalized_fext);
//  calc_energy(generalized_qd,M);
  calc_com();

  if(NC != 0) {
    center_of_contact.point.set_zero();
    center_of_contact.normal.set_zero();
    center_of_contact.point.pose = environment_frame;
    center_of_contact.normal.pose = environment_frame;
    for(int f=0;f<NUM_EEFS;f++){
      // set gait centers
      if(eefs_[f].active){
        center_of_contact.point += Ravelin::Vector3d(eefs_[f].point.data(),environment_frame)/NC;
        center_of_contact.normal += Ravelin::Vector3d(eefs_[f].normal.data(),environment_frame)/NC;
      }
    }
    center_of_contact.active = true;
  }
  else {
    center_of_contact.normal = Ravelin::Vector3d(0,0,1,environment_frame);
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
         std::vector<const EndEffector * > active_eefs;
         if(eefs_[0].active)
           active_eefs.push_back(&eefs_[0]);
         if(eefs_[1].active)
           active_eefs.push_back(&eefs_[1]);
         if(eefs_[3].active)
           active_eefs.push_back(&eefs_[3]);
         if(eefs_[2].active)
           active_eefs.push_back(&eefs_[2]);

         // Draw Contact Polygon
         for(int i=0;i<NC;i++){
           visualize_ray(active_eefs[i]->point + Ravelin::Vector3d(0,0,0.001),
                         active_eefs[(i+1)%NC]->point  + Ravelin::Vector3d(0,0,0.001),
                         Ravelin::Vector3d(0.5,0.5,1),
                         sim);
         }
         // Draw all Contacts
//         for(int i=0;i<NC;i++)
//           for(int j=0;j<active_eefs[i]->contacts.size();j++){
//             visualize_ray(active_eefs[i]->contacts[j],
//                           active_eefs[i]->point,
//                           Ravelin::Vector3d(1,1,1),
//                           sim);
//           }
//         visualize_ray(center_of_contact.point,
//                    center_of_contact.normal + center_of_contact.point,
//                    Ravelin::Vector3d(1,1,0),
//                    sim);
       }

#endif
}

void Robot::update_poses(){
  // Get base frame
  base_link_frame = links_[0]->get_pose();

  Utility::quat2TaitBryan(base_link_frame->q,roll_pitch_yaw);

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
  }
}
