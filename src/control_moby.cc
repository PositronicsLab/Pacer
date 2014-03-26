/*****************************************************************************
 * Controller for LINKS robot
 ****************************************************************************/
#include <quadruped.h>

/// THESE DEFINES DETERMINE WHAT TYPE OF CONTROLLER THIS CODE USES
//    #define NDEBUG
//    #define USE_DUMMY_CONTACTS

//    #define USE_ROBOT
//    #define CONTROL_KINEMATICS

std::string LOG_TYPE("INFO");
//std::string LOG_TYPE("ERROR");
/// END USER DEFINITIONS

    std::vector<std::string> joint_names_;
#ifdef USE_ROBOT
  #include <dxl/Dynamixel.h>
    Dynamixel* dxl_;
#endif

using namespace Moby;
using namespace Ravelin;

// simulator
boost::shared_ptr<Moby::EventDrivenSimulator> sim;
boost::shared_ptr<Quadruped> Lynx_ptr;

//////////////////////////////////////////////////////////////////////////////
/////////////////////////////// Controllers //////////////////////////////////

/// Event callback function for processing [contact] events
#ifndef USE_DUMMY_CONTACTS
void post_event_callback_fn(const std::vector<Event>& e,
                            boost::shared_ptr<void> empty)
{
  unsigned NC = 0;
  std::vector<EndEffector>& eefs_ = Lynx_ptr->get_end_effectors();
  std::vector<std::string>& eef_names_ = Lynx_ptr->get_end_effector_names();

  for(int i=0;i<eefs_.size();i++){
    eefs_[i].active = false;
    eefs_[i].contacts.clear();
    eefs_[i].contact_impulses.clear();
  }
  // PROCESS CONTACTS
  for(unsigned i=0;i<e.size();i++){
    if (e[i].event_type == Event::eContact)
    {
      bool MIRROR_FLAG = false;

      SingleBodyPtr sb1 = e[i].contact_geom1->get_single_body();
      SingleBodyPtr sb2 = e[i].contact_geom2->get_single_body();

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
//      std::cout << sb1->id << std::endl;
//      std::cout <<"\t"<< e[i].contact_impulse << std::endl;
//      std::cout <<"\t"<< e[i].contact_impulse.get_linear() << std::endl;
//      std::cout <<"\t"<< e[i].contact_impulse.get_angular() << std::endl;
//      std::cout <<"\t"<< e[i].contact_point << std::endl;

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
      eefs_[index].event = boost::shared_ptr<const Moby::Event>(new Event(e[i]));
    }
  }
  for(unsigned i=0;i< eefs_.size();i++){
    if(!eefs_[i].active) continue;
    Ravelin::Origin3d impulse(0,0,0),contact(0,0,0);
    for(unsigned j=0;j< eefs_[i].contacts.size();j++){
      impulse += Ravelin::Origin3d(eefs_[i].contact_impulses[j]);
      contact += Ravelin::Origin3d(Ravelin::Pose3d::transform_point(Moby::GLOBAL,eefs_[i].contacts[j]))/eefs_[i].contacts.size();
    }
    std::cout << eefs_[i].id << "(" << eefs_[i].contacts.size()<< ")\t " <<  std::setprecision(5) << impulse
              << "\t@  " << contact
              << ",\tn =" << eefs_[i].normal << std::endl;
  }

}
#endif

/// Event callback function for setting friction vars pre-event
void pre_event_callback_fn(std::vector<Event>& e, boost::shared_ptr<void> empty){}

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



/// The main control loop

void controller(DynamicBodyPtr dbp, double t, void*)
{
  std::cerr << " -- controller() entered" << std::endl;

  std::vector<EndEffector>& eefs_ = Lynx_ptr->get_end_effectors();
  std::vector<Moby::JointPtr>& joints_ = Lynx_ptr->get_joints();
  joint_names_ = Lynx_ptr->get_joint_names();
  Moby::RCArticulatedBodyPtr abrobot = Lynx_ptr->get_articulated_body();
  unsigned num_joints = joint_names_.size();

  static int ITER = 0;
  static double last_time = 0;

  static Ravelin::MatrixNd U(num_joints,1);

  Ravelin::MatrixNd q(num_joints,1),
                   qd(num_joints,1);
  ///  Record Robot State
#ifdef USE_ROBOT2
    // Query state of robot from R. Links

    Ravelin::VectorNd q_robot(Dynamixel::N_JOINTS),
                     qd_robot(Dynamixel::N_JOINTS);
    q_robot.set_zero();
    qd_robot.set_zero();
    dxl_->get_state(q_robot.data(),qd_robot.data());

     for(unsigned m=0,i=0;i< num_joints;m++){
       if(joints_[m]->q.size() == 0) continue;
       for(Dynamixel::Joints j=Dynamixel::BODY_JOINT;j<Dynamixel::N_JOINTS;j++)
         if(joints_[m]->id.compare(dxl_->JointName(j)) == 0){
           q(i,0) = q_robot[j];
           qd(i,0) = qd_robot[j];
         }
       i++;
     }

     // then set robot model to this state
     {
       Ravelin::VectorNd des_coordinates;
       abrobot->get_generalized_coordinates(DynamicBody::eEuler,des_coordinates);
       for(unsigned m=0,i=0;i< num_joints;m++){
           if(joints_[m]->q.size() == 0) continue;
           joints_[m]->q[0] = q(i,0);
           des_coordinates[i] = q(i,0);
           i++;
       }
       // Push initial state to robot
       abrobot->set_generalized_coordinates(DynamicBody::eEuler,des_coordinates);
       abrobot->update_link_poses();
       abrobot->update_link_velocities();
     }
     OUTLOG(q_robot,"q_robot");
     OUTLOG(qd_robot,"qd_robot");
# else
    // Query state of robot from simulator
    for(unsigned m=0,i=0;m< num_joints;m++){
        if(joints_[m]->q.size() == 0) continue;
        q.set_row(i,joints_[m]->q);
        qd.set_row(i,joints_[m]->qd);
        i++;
    }
#endif
//    return;

#ifdef USE_DUMMY_CONTACTS
    NC = eefs_.size();

    for(unsigned i=0;i<num_joints;i++){
      eefs_[i].active = true;
      Ravelin::Pose3d pose = *eefs_[i].link->get_pose();
      pose.update_relative_pose(Moby::GLOBAL);
      eefs_[i].normal = Ravelin::Vector3d(0,0,1);
      eefs_[i].point = pose.x;
    }
#endif

    Ravelin::VectorNd q_des(num_joints);
    Ravelin::VectorNd qd_des(num_joints);
    Ravelin::VectorNd u_vec(num_joints);


    Lynx_ptr->control(t,q.column(0),qd.column(0),q_des,qd_des,u_vec);
    U.set_column(0,u_vec);

      // send torque commands to robot
# ifdef CONTROL_KINEMATICS
    {
      Ravelin::VectorNd des_coordinates;
      abrobot->get_generalized_coordinates(DynamicBody::eEuler,des_coordinates);
      for(unsigned m=0,i=0;i< num_joints;m++){
          if(joints_[m]->q.size() == 0) continue;
          joints_[m]->q[0] = q_des[i];
          des_coordinates[i] = q_des[i];
          i++;
      }
      // Push initial state to robot
      abrobot->set_generalized_coordinates(DynamicBody::eEuler,des_coordinates);
      abrobot->update_link_poses();
      abrobot->update_link_velocities();
    }
#else
    Ravelin::VectorNd coordinates, velocity;
    abrobot->get_generalized_coordinates(DynamicBody::eEuler,coordinates);
    abrobot->get_generalized_velocity(DynamicBody::eSpatial,velocity);

    apply_simulation_forces(U,joints_);
#endif

     last_time = t;

#ifdef USE_ROBOT
# ifdef CONTROL_KINEMATICS
     Ravelin::VectorNd qdat(Dynamixel::N_JOINTS),
                      qddat(Dynamixel::N_JOINTS);
     qdat.set_zero();
     qddat.set_zero();

     for(unsigned m=0,i=0;i< num_joints;m++){
       if(joints_[m]->q.size() == 0) continue;
       for(int j=0;j<Dynamixel::N_JOINTS;j++)
         if(joints_[m]->id.compare(dxl_->JointName(j)) == 0){
           qdat[j] = q_des[i];
           qddat[j] = 0;//qd_des[i];
         }
       i++;
     }

     OUTLOG(qdat,"qdat");
     OUTLOG(qddat,"qddat");

//     dxl_->set_position(qdat.data());
     dxl_->set_state(qdat.data(),qddat.data());
# else
  Ravelin::VectorNd udat = u.column(0);
  dxl_->set_torque(udat.data());
# endif
#endif
  std::cerr << " -- controller() exited" << std::endl;

}

boost::shared_ptr<ContactParameters> cp_callback(CollisionGeometryPtr g1, CollisionGeometryPtr g2)
{
  boost::shared_ptr<ContactParameters> cp(new ContactParameters);
  const unsigned UINF = std::numeric_limits<unsigned>::max();

  // setup zero restitution and viscous friction
  cp->epsilon = 0.0;
  cp->mu_viscous = 0.0;

  // setup a random coefficient of friction
  cp->mu_coulomb = 0.0 + ((double) rand() / RAND_MAX)/2;

  // setup NK
//  cp->NK = UINF;
  cp->NK = 8;
  std::cout << "MU = " << cp->mu_coulomb << std::endl;

  return cp;
}

void init_cpp(){
#ifdef USE_ROBOT
  dxl_ = new Dynamixel("/dev/tty.usbserial-A9YL9ZZV",1000000);
  dxl_->relaxed(false);
#endif
}

/// plugin must be "extern C"
extern "C" {

void init(void* separator,
          const std::map<std::string, BasePtr>& read_map,
          double time)
{
  // If use robot is active also init dynamixel controllers
  init_cpp();

  std::cout << LOG_TYPE << std::endl;
  FILELog::ReportingLevel() =
      FILELog::FromString( (!LOG_TYPE.empty()) ? LOG_TYPE : "INFO");
Moby::RCArticulatedBodyPtr abrobot;
  // get a reference to the EventDrivenSimulator instance
  for (std::map<std::string, BasePtr>::const_iterator i = read_map.begin();
       i !=read_map.end(); i++)
  {
    // Find the simulator reference
    if (!sim)
      sim = boost::dynamic_pointer_cast<EventDrivenSimulator>(i->second);

    // find the robot reference
    if (!abrobot)
    {
      abrobot = boost::dynamic_pointer_cast<RCArticulatedBody>(i->second);
    }
  }
  Lynx_ptr = boost::shared_ptr<Quadruped>(new Quadruped(abrobot));

  // This will force us to updtae the robot state instead of Moby
#ifdef CONTROL_KINEMATICS
  abrobot->set_kinematic(true);
#endif

  // Set a random friction value in Moby for each contact made
//  sim->get_contact_parameters_callback_fn = cp_callback;

  // If using dummy contact ignore impact callback function
#ifndef USE_DUMMY_CONTACTS
  sim->event_post_impulse_callback_fn = &post_event_callback_fn;
#endif

  // setup the controller
  abrobot->controller = &controller;
}
} // end extern C

