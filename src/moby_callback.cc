#include <robot.h>

// simulator
extern boost::shared_ptr<Moby::EventDrivenSimulator> sim;
extern boost::shared_ptr<Robot> robot_ptr;
extern std::vector<std::string> joint_names_;

void post_event_callback_fn(const std::vector<Moby::Event>& e,
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
  // PROCESS CONTACTS
  for(unsigned i=0;i<e.size();i++){
    if (e[i].event_type == Moby::Event::eContact)
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
      eefs_[index].event = boost::shared_ptr<const Moby::Event>(new Moby::Event(e[i]));
    }
  }
  for(unsigned i=0;i< eefs_.size();i++){
    if(!eefs_[i].active) continue;
    Ravelin::Origin3d impulse(0,0,0),contact(0,0,0);
    for(unsigned j=0;j< eefs_[i].contacts.size();j++){
      impulse += Ravelin::Origin3d(eefs_[i].contact_impulses[j]);
      contact += Ravelin::Origin3d(Ravelin::Pose3d::transform_point(Moby::GLOBAL,eefs_[i].contacts[j]))/eefs_[i].contacts.size();
    }
    OUT_LOG(logINFO) << eefs_[i].id << "(" << eefs_[i].contacts.size()<< ")\t " <<  std::setprecision(5) << impulse
                    << "\t@  " << contact
                    << ",\tn =" << eefs_[i].normal << std::endl;
  }

}

/// Event callback function for setting friction vars pre-event
void pre_event_callback_fn(std::vector<Moby::Event>& e, boost::shared_ptr<void> empty){}

void apply_simulation_forces(const Ravelin::MatrixNd& u,std::vector<Moby::JointPtr>& joints){
    for(unsigned m=0,i=0;m< joint_names_.size();m++){
        if(joints[m]->q.size() == 0) continue;
        // reset motor torque
        Ravelin::VectorNd row;
        joints[m]->reset_force();
        joints[m]->add_force(u.get_row(i,row));
        i++;
    }
}


