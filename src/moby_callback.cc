#include <robot.h>

// simulator
extern boost::shared_ptr<Moby::EventDrivenSimulator> sim;
extern boost::shared_ptr<Robot> robot_ptr;
extern std::vector<std::string> joint_names_;
extern bool new_sim_step;

void post_event_callback_fn(const std::vector<Moby::UnilateralConstraint>& e,
                            boost::shared_ptr<void> empty)
{
  unsigned NC = 0;
  std::vector<EndEffector>& eefs_ = robot_ptr->get_end_effectors();
  std::vector<std::string>& eef_names_ = robot_ptr->get_end_effector_names();

  for(int i=0;i<eefs_.size();i++){
    eefs_[i].active = false;
    eefs_[i].contacts.clear();
    eefs_[i].contact_impulses.clear();
  }
//#define FAKE_CONTACTS
#ifndef FAKE_CONTACTS
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
        eefs_[index].tan1 = -e[i].contact_tan1;
        eefs_[index].tan2 = -e[i].contact_tan2;

      } else {
        eefs_[index].normal = e[i].contact_normal;
        eefs_[index].tan1 = e[i].contact_tan1;
        eefs_[index].tan2 = e[i].contact_tan2;
      }
      eefs_[index].event = boost::shared_ptr<const Moby::UnilateralConstraint>(new Moby::UnilateralConstraint(e[i]));
    }
  }
#else
  // PROCESS FAKE CONTACTS
  for(unsigned index=0;index<4;index++){
    Ravelin::Pose3d foot_pose = *eefs_[index].link->get_pose();
    foot_pose.update_relative_pose(Moby::GLOBAL);

    eefs_[index].contacts.push_back(Ravelin::Vector3d(foot_pose.x.data()));

      // Increment number of active contacts
      NC++;

      // Push Active contact info to EEF
      eefs_[index].active = true;
      eefs_[index].point = Ravelin::Vector3d(foot_pose.x.data());
      eefs_[index].normal = Ravelin::Vector3d(0,0,1);
      eefs_[index].tan1 = Ravelin::Vector3d(1,0,0);;
      eefs_[index].tan2 = Ravelin::Vector3d(0,1,0);;
    }
#endif
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
extern double
              SIM_MU_COULOMB,
              SIM_MU_VISCOSE,
              SIM_PENALTY_KV,
              SIM_PENALTY_KP;

boost::shared_ptr<Moby::ContactParameters> get_contact_parameters(Moby::CollisionGeometryPtr geom1, Moby::CollisionGeometryPtr geom2){
  boost::shared_ptr<Moby::ContactParameters> e = boost::shared_ptr<Moby::ContactParameters>(new Moby::ContactParameters());

  if(geom1->get_single_body())
  e->penalty_Kp = SIM_PENALTY_KP;
  e->penalty_Kv = SIM_PENALTY_KV;
  e->mu_coulomb = SIM_MU_COULOMB;
  e->mu_viscous = SIM_MU_VISCOSE;
  return e;
}

boost::shared_ptr<Moby::ContactParameters> cp_callback(Moby::CollisionGeometryPtr g1, Moby::CollisionGeometryPtr g2){
  boost::shared_ptr<Moby::ContactParameters> e = boost::shared_ptr<Moby::ContactParameters>(new Moby::ContactParameters());
  e->mu_viscous = SIM_MU_VISCOSE;
  return e;
}

void post_step_callback_fn(Moby::Simulator* s){
  new_sim_step = true;
}

/// Event callback function for setting friction vars pre-event
void pre_event_callback_fn(std::vector<Moby::UnilateralConstraint>& e, boost::shared_ptr<void> empty){
  for(int i=0;i< e.size();i++){
    OUT_LOG(logDEBUG1) << e[i] << std::endl;
  }
}

void apply_simulation_forces(const Ravelin::MatrixNd& u,std::vector<Moby::JointPtr>& joints){
    for(unsigned m=0,i=0;m< joints.size();m++){
        if(joints[m]->num_dof() == 0) continue;
        // reset motor torque
        Ravelin::VectorNd row;
        joints[m]->reset_force();
        joints[m]->add_force(u.get_row(i,row));
        i++;
    }
}


