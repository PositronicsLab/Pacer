/*****************************************************************************
 * Controller for LINKS robot
 ****************************************************************************/
#include <monopod.h>

std::string LOG_TYPE("ERROR");  // Only major failures from the system
//std::string LOG_TYPE("INFO");     // Normal print out
//std::string LOG_TYPE("DEBUG");  // Basic Outline of progress with additinal vectors
//std::string LOG_TYPE("DEBUG1"); // all function parameters and results

bool new_sim_step = true;

// simulator
boost::shared_ptr<Moby::EventDrivenSimulator> sim;
boost::shared_ptr<Robot> robot_ptr;
boost::shared_ptr<Monopod> monopod_ptr;
std::vector<std::string> joint_names_;


void post_event_callback_fn(const std::vector<Moby::Event>& e,
                                   boost::shared_ptr<void> empty);
void pre_event_callback_fn(std::vector<Moby::Event>& e, boost::shared_ptr<void> empty);
void apply_simulation_forces(const Ravelin::MatrixNd& u,std::vector<Moby::JointPtr>& joints);

/// The main control loop

void controller(Moby::DynamicBodyPtr dbp, double t, void*)
{
//  std::cerr << " -- controller() entered" << std::endl;

  std::vector<EndEffector>& eefs_ = robot_ptr->get_end_effectors();
  std::vector<Moby::JointPtr>& joints_ = robot_ptr->get_joints();
  joint_names_ = robot_ptr->get_joint_names();
  Moby::RCArticulatedBodyPtr abrobot = robot_ptr->get_articulated_body();
  unsigned num_joints = joint_names_.size();

  static Ravelin::MatrixNd U = Ravelin::MatrixNd::zero(num_joints,1);

  static int ITER = 0;
  static double last_time = 0;

  double dt = t - last_time;
  if(!new_sim_step){
//    apply_simulation_forces(U,joints_);
//    std::cerr << " -- controller() exited (old controls)" << std::endl;
//    return;
  }

  Ravelin::MatrixNd q(num_joints,1),
                   qd(num_joints,1);

    // Query state of robot from simulator
    for(unsigned m=0,i=0;m< num_joints;m++){
        if(joints_[m]->q.size() == 0) continue;
        q.set_row(i,joints_[m]->q);
        qd.set_row(i,joints_[m]->qd);
        i++;
    }

    Ravelin::VectorNd q_des(num_joints);
    Ravelin::VectorNd qd_des(num_joints);
    Ravelin::VectorNd u_vec(num_joints);


    monopod_ptr->control(dt,q.column(0),qd.column(0),q_des,qd_des,u_vec);
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
    apply_simulation_forces(U,joints_);
#endif

     last_time = t;

     new_sim_step = false;
//  std::cerr << "u_joints = " << U << std::endl;
//  std::cerr << " -- controller() exited" << std::endl;
  return;

}

void init_cpp(){
  robot_ptr = boost::dynamic_pointer_cast<Robot>(monopod_ptr);

  FILELog::ReportingLevel() =
      FILELog::FromString( (!LOG_TYPE.empty()) ? LOG_TYPE : "INFO");
}

/// plugin must be "extern C"
boost::shared_ptr<Moby::ContactParameters> cp_callback(Moby::CollisionGeometryPtr g1, Moby::CollisionGeometryPtr g2);
Moby::RigidBodyPtr create_terrain(unsigned nsph, double width_ext, double height_ext, double max_radius, double min_radius);
void set_terrain_robot_contact_parameters(Moby::DynamicBodyPtr terrain, Moby::DynamicBodyPtr robot, boost::shared_ptr<Moby::EventDrivenSimulator> sim);
void create_terrain_visualization(Moby::RigidBodyPtr terrain, boost::shared_ptr<Moby::EventDrivenSimulator> sim);
void post_step_callback_fn(Moby::Simulator* s);

extern "C" {
void init(void* separator,
          const std::map<std::string, Moby::BasePtr>& read_map,
          double time)
{
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
  monopod_ptr = boost::shared_ptr<Monopod>(new Monopod(abrobot));

  // This will force us to updtae the robot state instead of Moby
#ifdef CONTROL_KINEMATICS
  abrobot->set_kinematic(true);
#endif

  // Set a random friction value in Moby for each contact made
#ifdef CONTACT_CALLBACK
    sim->get_contact_parameters_callback_fn = cp_callback;
#endif
  // If using dummy contact ignore impact callback function
    sim->event_post_impulse_callback_fn = &post_event_callback_fn;
    sim->post_step_callback_fn = &post_step_callback_fn;
  // setup the controller
  abrobot->controller = &controller;

#ifdef USE_TERRAIN
  unsigned num_spheres = 60;
  double width_ext = 1,
        height_ext = 0.00,
        max_radius = 0.2,
        min_radius =  0.2;
  Moby::RigidBodyPtr terrain = create_terrain(num_spheres,  width_ext,  height_ext,  max_radius,min_radius);
  set_terrain_robot_contact_parameters(terrain, abrobot,sim);
  create_terrain_visualization(terrain,sim);
  sim->add_dynamic_body(terrain);
#endif

  init_cpp();
}
} // end extern C

