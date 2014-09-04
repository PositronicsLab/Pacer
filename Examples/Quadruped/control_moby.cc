/*****************************************************************************
 * Controller for LINKS robot
 ****************************************************************************/
#include <quadruped.h>

 boost::shared_ptr<Moby::EventDrivenSimulator> sim;
 boost::shared_ptr<Quadruped> quad_ptr;
 Moby::RCArticulatedBodyPtr abrobot;

 // ============================================================================
 // ================================ CUSTOM FNS ================================
 // ============================================================================

 void apply_sim_perturbations(){
   static std::vector<Moby::JointPtr>& joints_ = quad_ptr->get_joints();
   int num_joints = joints_.size();

   static std::vector<double> workv_,
       &unknown_base_perturbation = quad_ptr->get_variable("sim.unknown-base-perturbation",workv_),
       &known_base_perturbation = quad_ptr->get_variable("sim.known-base-perturbation",workv_),
       &known_leading_force = quad_ptr->get_variable("sim.known-leading-force",workv_);

   Ravelin::Vector3d lead(known_leading_force[3],
                          known_leading_force[4],
                          known_leading_force[5],
                          Moby::GLOBAL);
   OUTLOG(lead,"LEAD_g",logDEBUG);

   Ravelin::Vector3d point_on_robot(known_leading_force[0],
                                    known_leading_force[1],
                                    known_leading_force[2],
                                    quad_ptr->get_base_link_frame());
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
   quad_ptr->set_leading_force(lead_force);

   Ravelin::SForced known_force(&known_leading_force[0],Moby::GLOBAL);
   quad_ptr->set_known_force(known_force);

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

 // ============================================================================
 // ================================ CONTROLLER ================================
 // ============================================================================

void controller_callback(Moby::DynamicBodyPtr dbp, double t, void*)
{
  std::vector<Moby::JointPtr> joints_quad = quad_ptr->get_joints();
  std::vector<Moby::JointPtr> joints = abrobot->get_joints();
  bool init_index_fix = false;
  for(int i=0;i<joints.size();i++){
    if(joints[i]->num_dof() == 0) continue;
    for(int j=0;j<joints_quad.size();j++){
      if(joints_quad[j]->num_dof() == 0) continue;
      if(joints_quad[j]->id.compare(joints[i]->id) == 0){
        if(joints_quad[j]->get_coord_index() == joints[j]->get_coord_index()) continue;
        OUT_LOG(logDEBUG1)<< "Sim and robot joint indicies not in agreement,"
                             "fixing sim...\n Will skip this controller step" << std::endl;
        OUT_LOG(logDEBUG1)<< joints_quad[j]->id << "-rob : " << joints_quad[j]->get_coord_index() << std::endl;
        OUT_LOG(logDEBUG1)<< joints[i]->id << "-sim : " << joints[i]->get_coord_index() << std::endl;
        joints[i]->set_coord_index(joints_quad[j]->get_coord_index());
        OUT_LOG(logDEBUG1)<< joints_quad[j]->id << "-rob NEW : " << joints_quad[j]->get_coord_index() << std::endl;
        OUT_LOG(logDEBUG1)<< joints[i]->id << "-sim NEW : " << joints[i]->get_coord_index() << std::endl;
      }
    }
  }
  if(init_index_fix) return;

  Ravelin::VectorNd generalized_q,generalized_qd,generalized_qdd, generalized_fext;
  int num_joints = joints_quad.size();
  static double last_time = -0.001;
  double dt = t - last_time;
  last_time = t;

  abrobot->get_generalized_coordinates(Moby::DynamicBody::eEuler,generalized_q);
  abrobot->get_generalized_velocity(Moby::DynamicBody::eSpatial,generalized_qd);

  static Ravelin::VectorNd  generalized_qd_last = generalized_qd;
  //NOTE: Pre-contact    abrobot->get_generalized_acceleration(Moby::DynamicBody::eSpatial,generalized_qdd);
  ((generalized_qdd = generalized_qd) -= generalized_qd_last) /= dt;
  generalized_qd_last = generalized_qd;

  abrobot->get_generalized_forces(generalized_fext);

  Ravelin::VectorNd q_des(num_joints),
                    qd_des(num_joints),
                    qdd_des(num_joints);
  Ravelin::VectorNd u(num_joints);

  quad_ptr->control(t,generalized_q,generalized_qd,generalized_qdd,generalized_fext,q_des,qd_des,qdd_des,u);

  apply_sim_perturbations();
  for(int i=0;i<joints.size();i++){
    if(joints[i]->num_dof() == 0) continue;
    static Ravelin::VectorNd U(1);
//    std::cout<< joints[i]->get_coord_index() << " = " << u[joints[i]->get_coord_index()] << std::endl;
    U[0] = u[joints[i]->get_coord_index()];
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
  unsigned NC = 0;
  std::vector<EndEffector>& eefs_ = quad_ptr->get_end_effectors();
  std::vector<std::string>& eef_names_ = quad_ptr->get_end_effector_names();

  quad_ptr->reset_contact();

  // PROCESS CONTACTS
  for(unsigned i=0;i<e.size();i++){
    if (e[i].constraint_type == Moby::UnilateralConstraint::eContact)
    {
      bool MIRROR_FLAG = false;

      Moby::SingleBodyPtr sb1 = e[i].contact_geom1->get_single_body();
      Moby::SingleBodyPtr sb2 = e[i].contact_geom2->get_single_body();

      // Quit if the body hits the ground, this is a failure condition
      assert(!( (sb2->id.compare("ABDOMEN") == 0) || (sb1->id.compare("ABDOMEN") == 0) ));

      std::vector<std::string>::iterator iter =
          std::find(eef_names_.begin(), eef_names_.end(), sb1->id);
      //if end effector doesnt exist, check other SB
      if(iter  == eef_names_.end()){
        iter = std::find(eef_names_.begin(), eef_names_.end(), sb2->id);
        if(iter  == eef_names_.end())
          continue;
        else{
          MIRROR_FLAG = true;
          std::swap(sb1,sb2);
        }
      }

      size_t index = std::distance(eef_names_.begin(), iter);
      eefs_[index].contacts.push_back(e[i].contact_point);
      if(MIRROR_FLAG){
        eefs_[index].contact_impulses.push_back(-e[i].contact_impulse.get_linear());
      } else {
        eefs_[index].contact_impulses.push_back(e[i].contact_impulse.get_linear());
      }

      if (eefs_[index].active)
        continue;

      Ravelin::Pose3d foot_pose = *eefs_[index].link->get_pose();
      foot_pose.update_relative_pose(Moby::GLOBAL);

      // Increment number of active contacts
      NC++;

      // Push Active contact info to EEF
      eefs_[index].active = true;
      eefs_[index].point = e[i].contact_point;
      if(MIRROR_FLAG){
        eefs_[index].normal = -e[i].contact_normal;
        eefs_[index].tan1   = -e[i].contact_tan1;
        eefs_[index].tan2   = -e[i].contact_tan2;

      } else {
        eefs_[index].normal = e[i].contact_normal;
        eefs_[index].tan1   = e[i].contact_tan1;
        eefs_[index].tan2   = e[i].contact_tan2;
      }
      eefs_[index].event = boost::shared_ptr<const Moby::UnilateralConstraint>(new Moby::UnilateralConstraint(e[i]));
    }
  }

  OUT_LOG(logDEBUG)<< "cfs_moby = [";
  for(int i=0, ii = 0;i<eefs_.size();i++){
    if(eefs_[i].active){
     OUT_LOG(logDEBUG) << " " << eefs_[i].contact_impulses[0];
      ii++;
    } else {
      OUT_LOG(logDEBUG) << " [0.0, 0.0, 0.0] ";
    }
  }
  OUT_LOG(logDEBUG) << "]';" << std::endl;
}

boost::shared_ptr<Moby::ContactParameters> get_contact_parameters(Moby::CollisionGeometryPtr geom1, Moby::CollisionGeometryPtr geom2){
  boost::shared_ptr<Moby::ContactParameters> e = boost::shared_ptr<Moby::ContactParameters>(new Moby::ContactParameters());
  double workd;
  static double
      &SIM_PENALTY_KP = quad_ptr->get_variable("sim.penalty-kp",workd),
      &SIM_PENALTY_KV = quad_ptr->get_variable("sim.penalty-kv",workd),
      &SIM_MU_COULOMB = quad_ptr->get_variable("sim.mu-coulomb",workd),
      &SIM_MU_VISCOUS = quad_ptr->get_variable("sim.mu-viscous",workd);

  e->penalty_Kp = SIM_PENALTY_KP;
  e->penalty_Kv = SIM_PENALTY_KV;
  e->mu_coulomb = SIM_MU_COULOMB;
  e->mu_viscous = SIM_MU_VISCOUS;
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
//  quad_ptr = boost::shared_ptr<Quadruped>(new Quadruped(abrobot));
  quad_ptr = boost::shared_ptr<Quadruped>(new Quadruped());

  quad_ptr->sim = sim;
  // CONTACT PARAMETER CALLBACK (MUST BE SET)
  sim->get_contact_parameters_callback_fn = &get_contact_parameters;
  // CONTACT CALLBACK
  sim->constraint_callback_fn             = &pre_event_callback_fn;
  sim->constraint_post_callback_fn        = &post_event_callback_fn;
  // CONTROLLER CALLBACK
  abrobot->controller                     = &controller_callback;

  // ================= INIT ROBOT STATE ==========================
  std::vector<Moby::JointPtr> joints_quad = quad_ptr->get_joints();
  std::vector<Moby::JointPtr> joints = abrobot->get_joints();

  std::map<std::string,double> q0 = quad_ptr->get_q0();

  std::vector<double>
      workvd,
      base_start = quad_ptr->get_variable("quadruped.init.base.x",workvd);

  Ravelin::VectorNd q_start;
  abrobot->get_generalized_coordinates(Moby::DynamicBody::eSpatial,q_start);

  for(unsigned i=joints_quad.size();i<q_start.rows();i++)
    q_start[i] = base_start[i-joints_quad.size()];

  abrobot->set_generalized_coordinates(Moby::DynamicBody::eSpatial,q_start);
  abrobot->update_link_poses();
  for(int i=0;i<joints.size();i++){
    if(q0.find(joints[i]->id) == q0.end()) continue;
    joints[i]->q[0] = q0[joints[i]->id];
  }
  abrobot->update_link_poses();
}
} // end extern C
