/*****************************************************************************
 * Controller for LINKS robot
 ****************************************************************************/
#include <quadruped.h>

//std::string LOG_TYPE("ERROR");  // Only major failures from the system
//std::string LOG_TYPE("INFO");     // Normal print out
std::string LOG_TYPE("DEBUG");  // Basic Outline of progress with additinal vectors
//std::string LOG_TYPE("DEBUG1"); // all function parameters and results

bool new_sim_step = true;
Ravelin::VectorNd perturbation;
#ifdef USE_ROBOT
  #include <dxl/Dynamixel.h>
    Dynamixel* dxl_;
#endif

 boost::shared_ptr<Moby::EventDrivenSimulator> sim;
 boost::shared_ptr<Robot> robot_ptr;
 boost::shared_ptr<Quadruped> quad_ptr;
 std::vector<std::string> joint_names_;

void post_event_callback_fn(const std::vector<Moby::Event>& e,
                                   boost::shared_ptr<void> empty);
void pre_event_callback_fn(std::vector<Moby::Event>& e, boost::shared_ptr<void> empty);
void apply_simulation_forces(const Ravelin::MatrixNd& u,std::vector<Moby::JointPtr>& joints);

//////////////////////////////////////////////////////////////////////////////
/////////////////////////////// Controllers //////////////////////////////////

/// The main control loop

void controller_callback(Moby::DynamicBodyPtr dbp, double t, void*)
{
//  std::cerr << " -- controller() entered" << std::endl;

  std::vector<EndEffector>& eefs_ = robot_ptr->get_end_effectors();
  std::vector<Moby::JointPtr>& joints_ = robot_ptr->get_joints();
  joint_names_ = robot_ptr->get_joint_names();
  Moby::RCArticulatedBodyPtr abrobot = robot_ptr->get_articulated_body();
  unsigned num_joints = joint_names_.size();

  static int ITER = 0;
  static double last_time = 0;

  static Ravelin::MatrixNd U(num_joints,1);

  Ravelin::MatrixNd q(num_joints,1),
                   qd(num_joints,1);

  perturbation.set_zero(18);
//  perturbation[12 + 2] = -2;
//  perturbation[12 + 3] = 1;

//  abrobot->add_generalized_force(perturbation);

  ///  Record Robot State
#ifdef USE_ROBOT_STATE
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

    quad_ptr->control(t,q.column(0),qd.column(0),q_des,qd_des,u_vec);
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
#endif
    apply_simulation_forces(U,joints_);

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
//  std::cerr << " -- controller() exited" << std::endl;

  new_sim_step = false;
}

void init_cpp(){
  std::cerr << LOG_TYPE << std::endl;
  FILELog::ReportingLevel() =
      FILELog::FromString( (!LOG_TYPE.empty()) ? LOG_TYPE : "INFO");

  robot_ptr = boost::dynamic_pointer_cast<Robot>(quad_ptr);
#ifdef USE_ROBOT
  dxl_ = new Dynamixel("/dev/tty.usbserial-A9YL9ZZV",1000000);
  dxl_->relaxed(false);
#endif
  OUT_LOG(logERROR) << "Log Type : " << LOG_TYPE;
  OUT_LOG(logERROR) << "logERROR";
  OUT_LOG(logINFO) << "logINFO";
  OUT_LOG(logDEBUG) << "logDEBUG";
  OUT_LOG(logDEBUG1) << "logDEBUG1";

}
/// plugin must be "extern C"
extern "C" {

void init(void* separator,
          const std::map<std::string, Moby::BasePtr>& read_map,
          double time)
{
  // If use robot is active also init dynamixel controllers
  Moby::RCArticulatedBodyPtr abrobot;
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
  quad_ptr = boost::shared_ptr<Quadruped>(new Quadruped(abrobot));

#ifdef SET_KINEMATICS
  // This will force us to updtae the robot state instead of Moby
  abrobot->set_kinematic(true);
#endif
#ifdef CONTACT_CALLBACK
  // Set a random friction value in Moby for each contact made
  sim->get_contact_parameters_callback_fn = cp_callback;
#endif

  sim->event_post_impulse_callback_fn = &post_event_callback_fn;

#ifdef USE_TERRAIN
  unsigned num_spheres = 0;
  double width_ext = 1,
        height_ext = 0.00,
        max_radius = 0.2,
        min_radius =  0.2;
  RigidBodyPtr terrain = create_terrain(num_spheres,  width_ext,  height_ext,  max_radius,min_radius);
  set_terrain_robot_contact_parameters(terrain, abrobot,sim);
  create_terrain_visualization(terrain,sim);
  sim->add_dynamic_body(terrain);
#endif

  // setup the controller
  abrobot->controller = &controller_callback;

  init_cpp();
}
} // end extern C
