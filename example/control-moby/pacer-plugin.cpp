/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/controller.h>
using namespace Pacer;

 boost::shared_ptr<Moby::EventDrivenSimulator> sim;
 boost::shared_ptr<Controller> robot_ptr;
 Moby::RCArticulatedBodyPtr abrobot;

 // ============================================================================
 // ================================ CUSTOM FNS ================================
 // ============================================================================

 void apply_sim_perturbations(){
   static std::vector<Moby::JointPtr>& joints_ = robot_ptr->get_joints();
   int num_joints = joints_.size();

   static std::vector<double> workv_,
       &unknown_base_perturbation = Utility::get_variable("sim.unknown-base-perturbation",workv_),
       &known_base_perturbation = Utility::get_variable("sim.known-base-perturbation",workv_),
       &known_leading_force = Utility::get_variable("sim.known-leading-force",workv_);

   Ravelin::Vector3d lead(known_leading_force[3],
                          known_leading_force[4],
                          known_leading_force[5],
                          Moby::GLOBAL);
   OUTLOG(lead,"LEAD_g",logDEBUG);

   Ravelin::Vector3d point_on_robot(known_leading_force[0],
                                    known_leading_force[1],
                                    known_leading_force[2],
                                    robot_ptr->get_base_link_frame());
   boost::shared_ptr<Ravelin::Pose3d> lead_transform =
       boost::shared_ptr<Ravelin::Pose3d>(new Ravelin::Pose3d(
         Ravelin::Quatd::identity(),
         Ravelin::Origin3d(
           Ravelin::Pose3d::transform_point(Moby::GLOBAL,point_on_robot)
         )
       ));
   Ravelin::SForced lead_force = Ravelin::SForced(0,0,0,0,0,0,Moby::GLOBAL);
   if(lead.norm() > Moby::NEAR_ZERO)
     lead_force = Ravelin::Pose3d::transform(
                                              Moby::GLOBAL,
                                              Ravelin::SForced(lead,Ravelin::Vector3d(0,0,0),lead_transform)
                                            );
   robot_ptr->set_leading_force(lead_force);

   Ravelin::SForced known_force(&known_leading_force[0],Moby::GLOBAL);
   robot_ptr->set_known_force(known_force);

   OUTLOG(lead,"LEAD_bt",logDEBUG);

   Ravelin::VectorNd perturbation(num_joints+6);
   perturbation.set_zero();

   for(int i=0;i<6;i++){
     perturbation[num_joints+i] += ( unknown_base_perturbation[i]
                                    + known_base_perturbation[i]
                                    + lead_force[i]);
   }

   abrobot->add_generalized_force(perturbation);
 }

 Ravelin::VectorNd& remap_values(const std::map<int,int>& value_map,const Ravelin::VectorNd v1,Ravelin::VectorNd& v2,bool reverse = false){
   v2 = v1;
   for(int i=0;i<value_map.size();i++){
     if(reverse)
       v2[i] = v1[value_map.at(i)];
     else
       v2[value_map.at(i)] = v1[i];
   }
    return v2;
 }

 // ============================================================================
 // ================================ CONTROLLER ================================
 // ============================================================================

void controller_callback(Moby::DynamicBodyPtr dbp, double t, void*)
{
  std::vector<Moby::JointPtr> joints_quad = robot_ptr->get_joints();
  std::vector<Moby::JointPtr> joints = abrobot->get_joints();

  std::map<int,int> joint_map;
  if(joint_map.empty())
    for(int i=0;i<joints.size();i++){
      if(joints[i]->num_dof() == 0) continue;
      for(int j=0;j<joints_quad.size();j++){
        if(!joints_quad[j]) continue;
        if(joints_quad[j]->num_dof() == 0) continue;
        if(joints_quad[j]->id.compare(joints[i]->id) == 0){
          for(int d = 0;d<joints_quad[j]->num_dof();d++)
            joint_map[joints[i]->get_coord_index()+d] = joints_quad[j]->get_coord_index()+d;
        }
      }
    }

  Ravelin::VectorNd generalized_q,generalized_qd,generalized_qdd, generalized_fext;
  int num_joints = joints_quad.size();
  static double last_time = -0.001;
  double dt = t - last_time;
  last_time = t;

  // Re-map state simulation->robot joints
  {
    abrobot->get_generalized_coordinates(Moby::DynamicBody::eEuler,generalized_q);
    generalized_q = remap_values(joint_map,generalized_q,workv_);
    abrobot->get_generalized_velocity(Moby::DynamicBody::eSpatial,generalized_qd);
    generalized_qd = remap_values(joint_map,generalized_qd,workv_);
    abrobot->get_generalized_forces(generalized_fext);
    generalized_fext = remap_values(joint_map,generalized_fext,workv_);
  }

  static Ravelin::VectorNd  generalized_qd_last = generalized_qd;
  //NOTE: Pre-contact    abrobot->get_generalized_acceleration(Moby::DynamicBody::eSpatial,generalized_qdd);
  ((generalized_qdd = generalized_qd) -= generalized_qd_last) /= dt;
  generalized_qd_last = generalized_qd;


  Ravelin::VectorNd q_des(num_joints),
                    qd_des(num_joints),
                    qdd_des(num_joints);
  Ravelin::VectorNd u(num_joints);

  robot_ptr->control(t,generalized_q,generalized_qd,generalized_qdd,generalized_fext,q_des,qd_des,qdd_des,u);

  // Re-map goals robot->simulation joints
  {
    q_des = remap_values(joint_map,q_des,workv_,true);
    qd_des = remap_values(joint_map,qd_des,workv_,true);
    qdd_des = remap_values(joint_map,qdd_des,workv_,true);
    u = remap_values(joint_map,u,workv_,true);
  }

//  apply_sim_perturbations();
  for(int i=0;i<joints.size();i++){
    Ravelin::VectorNd U(joints[i]->num_dof());
    for(int j=0;j<joints[i]->num_dof();j++)
      U[j] = u[joints[i]->get_coord_index()+j];
    joints[i]->add_force(U);
  }

  std::cout << std::endl;
}

// ============================================================================
// ================================ CALLBACKS =================================
// ============================================================================

void post_event_callback_fn(const std::vector<Moby::UnilateralConstraint>& e,
                            boost::shared_ptr<void> empty)
{
  std::vector<std::string>& eef_names_ = robot_ptr->get_end_effector_names();
  std::vector<EndEffector>& eefs_ = robot_ptr->get_end_effectors();
  robot_ptr->reset_contact();
  // PROCESS CONTACTS
  for(unsigned i=0;i<e.size();i++){
    if (e[i].constraint_type == Moby::UnilateralConstraint::eContact)
    {
      bool MIRROR_FLAG = false;

      Moby::SingleBodyPtr sb1 = e[i].contact_geom1->get_single_body();
      Moby::SingleBodyPtr sb2 = e[i].contact_geom2->get_single_body();

      std::vector<std::string>::iterator iter =
          std::find(eef_names_.begin(), eef_names_.end(), sb1->id);
      //if end effector doesnt exist, check other SB
      if(iter  == eef_names_.end()){
        iter = std::find(eef_names_.begin(), eef_names_.end(), sb2->id);
        if(iter  == eef_names_.end())
          continue;  // Contact doesn't include an end-effector
        else{
          // contact is upside-down, mark for negation
          MIRROR_FLAG = true;
          std::swap(sb1,sb2);
        }
      }

      size_t index = std::distance(eef_names_.begin(), iter);

      if(MIRROR_FLAG){
        eefs_[index].impulse.push_back(-e[i].contact_impulse.get_linear());
      } else {
        eefs_[index].impulse.push_back( e[i].contact_impulse.get_linear());
      }

      // Push Active contact info to EEF
      eefs_[index].active = true;
      eefs_[index].point.push_back(e[i].contact_point);
      if(MIRROR_FLAG){
        eefs_[index].impulse.push_back(-e[i].contact_impulse.get_linear());
        eefs_[index].normal.push_back(-e[i].contact_normal);
        eefs_[index].tan1.push_back(-e[i].contact_tan1);
        eefs_[index].tan2.push_back(-e[i].contact_tan2);

      } else {
        eefs_[index].impulse.push_back(e[i].contact_impulse.get_linear());
        eefs_[index].normal.push_back(e[i].contact_normal);
        eefs_[index].tan1.push_back(e[i].contact_tan1);
        eefs_[index].tan2.push_back(e[i].contact_tan2);
      }
      eefs_[index].mu_coulomb.push_back(e[i].contact_mu_coulomb);
      eefs_[index].mu_viscous.push_back(e[i].contact_mu_viscous);
      eefs_[index].impulse[eefs_[index].impulse.size()-1].pose = eefs_[index].point[eefs_[index].point.size()-1].pose;
    }
  }

  for(int i=0, ii = 0;i<eefs_.size();i++){
    if(eefs_[i].active){
      for(int j=0;j<eefs_[i].point.size();j++){
#ifdef VISUALIZE_MOBY
      visualize_ray(  eefs_[i].point[j],
                      eefs_[i].point[j] + eefs_[i].impulse[j]*10.0,
                      Ravelin::Vector3d(1,0.5,0),
                      0.1,
                      sim
                    );
#endif
      ii++;
      }
    }
  }
}

#include <random>
boost::shared_ptr<Moby::ContactParameters> get_contact_parameters(Moby::CollisionGeometryPtr geom1, Moby::CollisionGeometryPtr geom2){

  boost::shared_ptr<Moby::ContactParameters> e = boost::shared_ptr<Moby::ContactParameters>(new Moby::ContactParameters());

  static std::default_random_engine generator;
  static std::uniform_real_distribution<double> distribution(0.1,1.4);

  e->mu_coulomb = distribution(generator);

  return e;
}

void post_step_callback_fn(Moby::Simulator* s){}

/// Event callback function for setting friction vars pre-event
void pre_event_callback_fn(std::vector<Moby::UnilateralConstraint>& e, boost::shared_ptr<void> empty){
  for(int i=0;i< e.size();i++){
    OUT_LOG(logDEBUG1) << e[i] << std::endl;
  }
}

// ============================================================================
// ================================ INIT ======================================
// ============================================================================


//boost::shared_ptr<Moby::ContactParameters> get_contact_parameters(Moby::CollisionGeometryPtr geom1, Moby::CollisionGeometryPtr geom2);
//void post_event_callback_fn(const std::vector<Moby::UnilateralConstraint>& e, boost::shared_ptr<void> empty);
//void pre_event_callback_fn(std::vector<Moby::UnilateralConstraint>& e, boost::shared_ptr<void> empty);

/// plugin must be "extern C"
extern "C" {

void init(void* separator, const std::map<std::string, Moby::BasePtr>& read_map, double time)
{
  // If use robot is active also init dynamixel controllers
  // get a reference to the EventDrivenSimulator instance
  for (std::map<std::string, Moby::BasePtr>::const_iterator i = read_map.begin();
       i !=read_map.end(); i++)
  {
    // Find the simulator reference
    if (!sim)
      sim = boost::dynamic_pointer_cast<Moby::EventDrivenSimulator>(i->second);

    // find the robot reference
    if (!abrobot)
    {
      abrobot = boost::dynamic_pointer_cast<Moby::RCArticulatedBody>(i->second);
    }
  }

  /// Set up quadruped robot, linking data from moby's articulated body
  /// to the quadruped model used by Control-Moby
  robot_ptr = boost::shared_ptr<Controller>(new Controller());
#ifdef VISUALIZE_MOBY
  robot_ptr->sim = sim;
#endif
  // CONTACT PARAMETER CALLBACK (MUST BE SET)
#ifdef RANDOM_FRICTION
  sim->get_contact_parameters_callback_fn = &get_contact_parameters;
#endif
  // CONTACT CALLBACK
//  sim->constraint_callback_fn             = &pre_event_callback_fn;
  sim->constraint_post_callback_fn        = &post_event_callback_fn;
  // CONTROLLER CALLBACK
  abrobot->controller                     = &controller_callback;

  // ================= INIT ROBOT STATE ==========================
  std::vector<Moby::JointPtr> joints_quad = robot_ptr->get_joints();
  std::vector<Moby::JointPtr> joints = abrobot->get_joints();

  std::map<std::string,double> q0 = robot_ptr->get_q0();

  std::vector<double>
      workvd,
      base_start = Utility::get_variable("init.base.x",workvd);

  Ravelin::VectorNd q_start;
  abrobot->get_generalized_coordinates(Moby::DynamicBody::eSpatial,q_start);
  unsigned NDOFS = abrobot->num_generalized_coordinates(Moby::DynamicBody::eSpatial) - 6;


  for(unsigned i=NDOFS;i<q_start.rows();i++)
    q_start[i] = base_start[i-NDOFS];

  abrobot->set_generalized_coordinates(Moby::DynamicBody::eSpatial,q_start);
  abrobot->update_link_poses();
  for(int i=0,ii=0;i<joints.size();i++){
    for(int j=0;j<joints[i]->num_dof();j++,ii++){
      joints[i]->q[j] = q0[std::to_string(j)+joints[i]->id];
    }
  }
  abrobot->update_link_poses();
}
} // end extern C
