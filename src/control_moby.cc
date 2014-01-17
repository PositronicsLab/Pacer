/*****************************************************************************
 * Controller for LINKS robot
 ****************************************************************************/
#include <quadruped.h>

/// THESE DEFINES DETERMINE WHAT TYPE OF CONTROLLER THIS CODE USES
//    #define NDEBUG
//    #define USE_DUMMY_CONTACTS
//    #define FRICTION_EST
//    #define CONTROL_IDYN

    #define CONTROL_ZMP
    #define RENDER_CONTACT
//    #define USE_ROBOT
//    #define FOOT_TRAJ
//    #define TRUNK_STABILIZATION
//    #define CONTROL_KINEMATICS
//    #define FIXED_BASE

    std::string LOG_TYPE("INFO");
/// END USER DEFINITIONS

#ifdef USE_ROBOT
  #include <dxl/Dynamixel.h>
    Dynamixel* dxl_;
#endif

using namespace Moby;
using namespace Ravelin;

static Ravelin::LinAlgd LA_;

// simulator
boost::shared_ptr<Moby::EventDrivenSimulator> sim;
boost::shared_ptr<Quadruped> Lynx_ptr;

static Ravelin::VectorNd workv_;
static Ravelin::MatrixNd workM_;


///////////////////////////////////////////////////////////////////////////////
/////////////////////////////// Contact DATA //////////////////////////////////

void visualize_contact( const Moby::Event& e,
                        boost::shared_ptr<Moby::EventDrivenSimulator> sim );

void visualize_polygon( const Ravelin::MatrixNd& verts,
                        boost::shared_ptr<Moby::EventDrivenSimulator> sim );

void visualize_ray(   const Ravelin::Vector3d& point, const Ravelin::Vector3d& vec, boost::shared_ptr<Moby::EventDrivenSimulator> sim ) ;

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

  // PROCESS CONTACTS
  int nc = e.size();
  for(unsigned i=0;i<e.size();i++){
    if (e[i].event_type == Event::eContact)
    {
      SingleBodyPtr sb1 = e[i].contact_geom1->get_single_body();
      SingleBodyPtr sb2 = e[i].contact_geom2->get_single_body();

      std::vector<std::string>::iterator iter =
          std::find(eef_names_.begin(), eef_names_.end(), sb1->id);

      //if end effector doesnt exist
      if(iter  == eef_names_.end())
        continue;

      size_t index = std::distance(eef_names_.begin(), iter);

      OUTLOG(e[i].contact_impulse.get_linear(),sb1->id);
//      OUTLOG(e[i].contact_impulse.get_angular(),"angular");
      if (eefs_[index].active)
        continue;

#ifdef RENDER_CONTACT
        visualize_contact(e[i],sim);
#endif

      // Increment number of active contacts
      NC++;

      // Push Active contact info to EEF
      eefs_[index].active = true;
      eefs_[index].point = e[i].contact_point;
      eefs_[index].normal = e[i].contact_normal;
      eefs_[index].impulse = e[i].contact_impulse.get_linear();
    }
  }
#ifdef RENDER_CONTACT
     Ravelin::MatrixNd support_poly(3,NC);
     static unsigned NUM_EEFS = eefs_.size();

     for(int c=0,cc=0;c<NUM_EEFS;c++)
       if(eefs_[c].active){
          support_poly.set_column(cc,eefs_[c].point);
         cc++;
       }

     if(NC > 0)
      visualize_polygon(support_poly,sim);
#endif

  //  SRZ compare contact force prediction to Moby contact force
}
#endif

/// Event callback function for setting friction vars pre-event
void pre_event_callback_fn(std::vector<Event>& e, boost::shared_ptr<void> empty){}

void apply_simulation_forces(const Ravelin::MatrixNd& u,std::vector<Moby::JointPtr>& joints){
  unsigned num_joints = joints.size();
    for(unsigned m=0,i=0;m< num_joints;m++){
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
  std::vector<EndEffector>& eefs_ = Lynx_ptr->get_end_effectors();
  std::vector<Moby::JointPtr>& joints_ = Lynx_ptr->get_joints();
  Moby::RCArticulatedBodyPtr& abrobot = Lynx_ptr->get_articulated_body();
  unsigned num_joints = joints_.size();

    static int ITER = 0;
    static double last_time = 0;

    Ravelin::MatrixNd u(num_joints,1);

    double dt = t - last_time;
    if (dt == 0) return;

    Ravelin::MatrixNd q(num_joints,1),
                     qd(num_joints,1);
    ///  Record Robot State
    // Query state of robot
    for(unsigned m=0,i=0;m< num_joints;m++){
        if(joints_[m]->q.size() == 0) continue;
        q.set_row(i,joints_[m]->q);
        qd.set_row(i,joints_[m]->qd);
        i++;
    }

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

    Ravelin::VectorNd q_des;
    Ravelin::VectorNd qd_des;
    Ravelin::VectorNd u_vec;

    Lynx_ptr->control(q.column(0),qd.column(0),q_des,qd_des,u_vec);
    u.set_column(0,u_vec);

      // send torque commands to robot
# ifdef CONTROL_KINEMATICS

        for(unsigned m=0,i=0;m< NUM_JOINTS;m++){
            if(joints_[m]->q.size() == 0) continue;
            joints_[m]->q[0] = q_des[i];
            i++;
        }
       abrobot->update_link_poses();
#else
       apply_simulation_forces(u,joints_);
#endif

     last_time = t;
}

/// plugin must be "extern C"
extern "C" {

void init(void* separator,
          const std::map<std::string, BasePtr>& read_map,
          double time)
{
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
//      dbrobot = boost::dynamic_pointer_cast<DynamicBody>(i->second);
    }
  }
  Lynx_ptr = boost::shared_ptr<Quadruped>(new Quadruped(abrobot));


  // This will force us to updtae the robot state instead of Moby
#ifdef CONTROL_KINEMATICS
  abrobot->set_kinematic(true);
#endif

  // If using dummy contact ignore impact callback function
#ifndef USE_DUMMY_CONTACTS
  sim->event_post_impulse_callback_fn = &post_event_callback_fn;
#endif

  // setup the controller
  abrobot->controller = &controller;

  // If use robot is active also init dynamixel controllers
#ifdef USE_ROBOT
  dxl_ = new Dynamixel;
#endif
}
} // end extern C

