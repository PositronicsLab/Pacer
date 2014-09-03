/*****************************************************************************
 * Controller for LINKS robot
 ****************************************************************************/
#include <quadruped.h>

 boost::shared_ptr<Moby::EventDrivenSimulator> sim;
 boost::shared_ptr<Quadruped> quad_ptr;
 Moby::RCArticulatedBodyPtr abrobot;

 // ============================================================================
 // ================================ CONTROLLER ================================
 // ============================================================================
void controller_callback(Moby::DynamicBodyPtr dbp, double t, void*)
{
  static std::vector<Moby::JointPtr>& joints_ = quad_ptr->get_joints();
  int num_joints = joints_.size();

  static Ravelin::MatrixNd U(num_joints,1);

  Ravelin::MatrixNd q(num_joints,1),
                   qd(num_joints,1);

    for(unsigned m=0,i=0;m< num_joints;m++){
        if(joints_[m]->q.size() == 0) continue;
        q.set_row(i,joints_[m]->q);
        qd.set_row(i,joints_[m]->qd);
        i++;
    }

    Ravelin::VectorNd q_des(num_joints);
    Ravelin::VectorNd qd_des(num_joints);
    Ravelin::VectorNd u_vec(num_joints);

    quad_ptr->control(t,q.column(0),qd.column(0),q_des,qd_des,u_vec);
    U.set_column(0,u_vec);

    for(unsigned m=0,i=0;m< num_joints;m++){
        if(joints_[m]->num_dof() == 0) continue;
        // reset motor torque
        Ravelin::VectorNd row;
        joints_[m]->reset_force();
        joints_[m]->add_force(U.get_row(i,row));
        i++;
    }
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
//  double workd;
//  static double
//      &SIM_PENALTY_KP = quad_ptr->get_variable("sim.penalty-kp",workd),
//      &SIM_PENALTY_KV = quad_ptr->get_variable("sim.penalty-kv",workd),
//      &SIM_MU_COULOMB = quad_ptr->get_variable("sim.mu-coulomb",workd),
//      &SIM_MU_VISCOSE = quad_ptr->get_variable("sim.mu-viscose",workd);

//  static double
//      &SIM_PENALTY_KP = quad_ptr->get_variable<double>("sim.penalty-kp"),
//      &SIM_PENALTY_KV = quad_ptr->get_variable<double>("sim.penalty-kv"),
//      &SIM_MU_COULOMB = quad_ptr->get_variable<double>("sim.mu-coulomb"),
//      &SIM_MU_VISCOSE = quad_ptr->get_variable<double>("sim.mu-viscose");

//  e->penalty_Kp = SIM_PENALTY_KP;
//  e->penalty_Kv = SIM_PENALTY_KV;
//  e->mu_coulomb = SIM_MU_COULOMB;
//  e->mu_viscous = SIM_MU_VISCOSE;
  return e;
}

void post_step_callback_fn(Moby::Simulator* s){}

/// Event callback function for setting friction vars pre-event
void pre_event_callback_fn(std::vector<Moby::UnilateralConstraint>& e, boost::shared_ptr<void> empty){
  for(int i=0;i< e.size();i++){
    OUT_LOG(logDEBUG1) << e[i] << std::endl;
  }
}


void apply_sim_perturbations(){
  static std::vector<Moby::JointPtr>& joints_ = quad_ptr->get_joints();
  int num_joints = joints_.size();

//  static std::vector<double>
//      &unknown_base_perturbation = quad_ptr->get_variable<std::vector<double> >("sim.unknown-base-perturbation"),
//      &known_base_perturbation = quad_ptr->get_variable<std::vector<double> >("sim.known-base-perturbation"),
//      &known_leading_force = quad_ptr->get_variable<std::vector<double> >("sim.known-leading-force");

  std::vector<double> workv_;
  std::vector<double>
      unknown_base_perturbation = quad_ptr->get_variable("sim.unknown-base-perturbation",workv_),
      known_base_perturbation = quad_ptr->get_variable("sim.known-base-perturbation",workv_),
      known_leading_force = quad_ptr->get_variable("sim.known-leading-force",workv_);

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

  Ravelin::SForced lead_force = Ravelin::Pose3d::transform(
                                   Moby::GLOBAL,
                                   Ravelin::SForced(lead,Ravelin::Vector3d(0,0,0),lead_transform)
                                 );
  quad_ptr->set_leading_force(lead_force);

  Ravelin::SForced known_force(&known_leading_force[0],Moby::GLOBAL);
  quad_ptr->set_known_force(known_force);

  #ifdef VISUALIZE_MOBY
    visualize_ray(  Ravelin::Vector3d(lead_transform->x.data()),
                    Ravelin::Vector3d(lead_transform->x.data())
                     + Ravelin::Vector3d(known_leading_force[3],
                                         known_leading_force[4],
                                         known_leading_force[5]),
                    Ravelin::Vector3d(1,0,1),
                    sim
                  );
  #endif

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
// ================================ INIT ======================================
// ============================================================================


boost::shared_ptr<Moby::ContactParameters> get_contact_parameters(Moby::CollisionGeometryPtr geom1, Moby::CollisionGeometryPtr geom2);
void post_event_callback_fn(const std::vector<Moby::UnilateralConstraint>& e, boost::shared_ptr<void> empty);
void pre_event_callback_fn(std::vector<Moby::UnilateralConstraint>& e, boost::shared_ptr<void> empty);

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
  quad_ptr = boost::shared_ptr<Quadruped>(new Quadruped(abrobot));

  // CONTACT PARAMETER CALLBACK (MUST BE SET)
  sim->get_contact_parameters_callback_fn = &get_contact_parameters;
  // CONTACT CALLBACK
  sim->constraint_callback_fn             = &pre_event_callback_fn;
  sim->constraint_post_callback_fn        = &post_event_callback_fn;
  // CONTROLLER CALLBACK
  abrobot->controller = &controller_callback;
}
} // end extern C
