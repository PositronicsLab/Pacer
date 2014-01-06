/*****************************************************************************
 * Controller for LINKS robot
 ****************************************************************************/
#include <quadruped_control.h>

/// THESE DEFINES DETERMINE WHAT TYPE OF CONTROLLER THIS CODE USES
//    #define NDEBUG
//    #define USE_DUMMY_CONTACTS
//    #define FRICTION_EST
//    #define CONTROL_IDYN

    #define CONTROL_ZMP
    #define RENDER_CONTACT
//    #define USE_ROBOT
//    #define CONTROL_KINEMATICS
//    #define FOLLOW_TRAJECTORY
    #define FOOT_TRAJ
    std::string LOG_TYPE("INFO");
/// END USER DEFINITIONS

#ifdef USE_ROBOT
  #include <dxl/Dynamixel.h>
    Dynamixel* dxl_;
#endif

#define grav 9.8 // M/s.s
#define M_PI_16 0.19634954084
#define M_PI_8 0.39269908169
const double TORQUE_LIMIT = 3; //N.m
Vec BASE_ORIGIN(7);

using namespace Moby;
using namespace Ravelin;

std::vector<std::string> eef_names_;
std::vector<ContactData> contacts;
static map<string, double> q0,u_max;

// simulator
boost::shared_ptr<Moby::EventDrivenSimulator> sim;
 Moby::RCArticulatedBodyPtr abrobot;
 Moby::DynamicBodyPtr dbrobot;
 std::vector<Moby::JointPtr> joints_;
 std::vector<Moby::RigidBodyPtr> links_,eefs_;
static Vec workv_;
static Mat workM_;

 unsigned NUM_EEFS,
          N_FIXED_JOINTS,
          NUM_JOINTS,
          NUM_LINKS,
          NEULER = 7,// for generalized coords
          NSPATIAL = 6,
          NDOFS, // for generalized velocity, forces. accel
          NK = 4;

///////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// PID ///////////////////////////////////////

struct Gains
{
  double kp;
  double kv;
  double ki;
};

// map of gains
map<string, Gains> gains;

// integration error
map<string, double> perr_sum;

template <typename T, typename U>
const U& get(const map<T, U>& m, const T& key)
{
  typename map<T, U>::const_iterator i = m.find(key);
  if (i == m.end())
    throw std::runtime_error("Unexpectedly couldn't find key in map!");
  return i->second;
}

/// Controls the robot
void control_PID(const map<string, double>& q_des,
                 const map<string, double>& qd_des,
                 const map<string, Gains>& gains,
                 double time,Mat& ufb)
{
  // clear and set motor torques
  for (unsigned m=0,i=0; m< NUM_JOINTS; m++)
  {
    // get the joint
    JointPtr j = joints_[m];
    if(j->q.rows() == 0) continue;

    // get the two gains
    const double KP = get(gains,j->id).kp;
    const double KV = get(gains,j->id).kv;
    const double KI = get(gains,j->id).ki;

    // add feedback torque to joints
    double perr = get(q_des,j->id) - j->q[0];
    perr_sum[j->id] += perr;
    double ierr = perr_sum[j->id];
    double derr = get(qd_des,j->id) - j->qd[0];

    Vec fb_torque(1);
    fb_torque[0] = perr*KP + derr*KV + ierr*KI;
    ufb.set_row(i,fb_torque);
    i++;
  }
}

///////////////////////////////////////////////////////////////////////////////
/////////////////////////////// Contact DATA //////////////////////////////////

void calculate_dyn_properties(Mat& M, Vec& fext){
    M.resize(NDOFS,NDOFS);
    fext.resize(NDOFS);
    abrobot->get_generalized_inertia(M);
    abrobot->get_generalized_forces(fext);
}

//////////////////////////////////////////////////////////////////////////////
/////////////////////////////// Controllers //////////////////////////////////

/// Event callback function for processing [contact] events
#ifndef USE_DUMMY_CONTACTS
void post_event_callback_fn(const vector<Event>& e,
                            boost::shared_ptr<void> empty)
{
  // PROCESS CONTACTS
  contacts.clear();
  int nc = e.size();
  Vec mobycf(4 + 4*NK);
  mobycf.set_zero(4 + 4*NK);
  for(unsigned i=0;i<e.size();i++){
    if (e[i].event_type == Event::eContact)
    {
#ifdef RENDER_CONTACT
        visualize_contact(e[i],sim);
#endif
      ContactData c;
      SingleBodyPtr sb1 = e[i].contact_geom1->get_single_body();
      SingleBodyPtr sb2 = e[i].contact_geom2->get_single_body();

      if (sb1->id.compare("ground") == 0){
          c.normal = -e[i].contact_normal;
          std::swap(sb1,sb2);
      } else
          c.normal = e[i].contact_normal;

      if (std::find(eef_names_.begin(), eef_names_.end(), sb1->id)
          == eef_names_.end())
        continue;

      bool contact_used = false;
      for(int j=0;j<contacts.size();j++)
        if(contacts[j].name.compare(sb1->id) == 0)
          contact_used = true;

      if(contact_used)
        continue;
      c.point = e[i].contact_point;
      c.name = sb1->id;

//      std::cout << "Moby cf: " << i << std::endl << e[i].contact_impulse.get_linear() << std::endl;

//      mobycf[contacts.size()] = e[i].contact_impulse[2];
//      if(e[i].contact_impulse[0] > 0)
//        mobycf[4 + contacts.size()*NK] = e[i].contact_impulse[0];
//      else
//        mobycf[4 + contacts.size()*NK + NK/2] = e[i].contact_impulse[0];

//      if(e[i].contact_impulse[1] > 0)
//        mobycf[4 + contacts.size()*NK + 1] = e[i].contact_impulse[1];
//      else
//        mobycf[4 + contacts.size()*NK + 1 + NK/2 ] = e[i].contact_impulse[1];

      contacts.push_back(c);
    }
  }

//  OUTLOG(mobycf,"MOBY_contact_force");
  //  Note compare contact force prediction to Moby contact force
}
#endif

/// Event callback function for setting friction vars pre-event
void pre_event_callback_fn(vector<Event>& e, boost::shared_ptr<void> empty){}

void apply_simulation_forces(const Mat& u){
    for(unsigned m=0,i=0;m< NUM_JOINTS;m++){
        if(joints_[m]->q.size() == 0) continue;
        // reset motor torque
        Vec row;
        joints_[m]->reset_force();
        joints_[m]->add_force(u.get_row(i,row));
        i++;
    }
}

double calc_energy(Vec& v, Mat& M){
  // Potential Energy
  double PE = 0;
  const std::vector<RigidBodyPtr>& links = abrobot->get_links();
  for(int i=0;i<links.size();i++){
     RigidBody& link = *links[i];
     double m = link.get_mass();
     Ravelin::Pose3d link_com = *link.get_inertial_pose();
     link_com.update_relative_pose(Moby::GLOBAL);
     PE += link_com.x[2] * m * grav;
  }
  M.mult(v, workv_);
  double KE = workv_.dot(v)*0.5;
#ifndef NDEBUG
  std::cout << "KE = " << KE << ", PE = " << PE << std::endl;
  std::cout << "Total Energy = " << (KE + PE) << std::endl;
#endif
  return (KE + PE);
  // Kinetic Energy
}

Vector3d& calc_com(Vector3d& weighted_com,Vector3d& com_acc){
  weighted_com.set_zero();
  const std::vector<RigidBodyPtr>& links = abrobot->get_links();
  const SAcceld& base_acc = links[0]->get_accel();
  double total_mass=0;
  for(int i=0;i<links.size();i++){

     RigidBody& link = *links[i];
     double m = link.get_mass();
     total_mass += m;
     Ravelin::Pose3d link_com = *link.get_inertial_pose();
     link_com.update_relative_pose(Moby::GLOBAL);
     weighted_com += (link_com.x *= m);
  }
  weighted_com /= total_mass;

  shared_ptr<Ravelin::Pose3d> base_com_w(new Ravelin::Pose3d());
  base_com_w->x = Ravelin::Origin3d(weighted_com);
  SAcceld com_xdd = Ravelin::Pose3d::transform(base_com_w, base_acc);
  com_acc = com_xdd.get_linear();

  return weighted_com;
}

void get_trajectory(double t,double dt,
                    map<string, double>& q_des,
                    map<string, double>& qd_des,
                    map<string, double>& qdd_des){
  double hip_val = sin(t),
         knee_val = sin(t);

  q_des["LF_HIP_AA"] = 0;
  q_des["LF_HIP_FE"] = hip_val;
  q_des["LF_LEG_FE"] = knee_val;

  q_des["RF_HIP_AA"] = 0;
  q_des["RF_HIP_FE"] = hip_val;
  q_des["RF_LEG_FE"] = knee_val;

  q_des["LH_HIP_AA"] = 0;
  q_des["LH_HIP_FE"] = hip_val;
  q_des["LH_LEG_FE"] = knee_val;

  q_des["RH_HIP_AA"] = 0;
  q_des["RH_HIP_FE"] = hip_val;
  q_des["RH_LEG_FE"] = knee_val;

  hip_val = cos(t);
  knee_val = cos(t);

  qd_des["LF_HIP_AA"] = 0;
  qd_des["LF_HIP_FE"] = hip_val;
  qd_des["LF_LEG_FE"] = knee_val;

  qd_des["RF_HIP_AA"] = 0;
  qd_des["RF_HIP_FE"] = hip_val;
  qd_des["RF_LEG_FE"] = knee_val;

  qd_des["LH_HIP_AA"] = 0;
  qd_des["LH_HIP_FE"] = hip_val;
  qd_des["LH_LEG_FE"] = knee_val;

  qd_des["RH_HIP_AA"] = 0;
  qd_des["RH_HIP_FE"] = hip_val;
  qd_des["RH_LEG_FE"] = knee_val;

  hip_val = -sin(t);
  knee_val = -sin(t);

  qdd_des["LF_HIP_AA"] = 0;
  qdd_des["LF_HIP_FE"] = hip_val;
  qdd_des["LF_LEG_FE"] = knee_val;

  qdd_des["RF_HIP_AA"] = 0;
  qdd_des["RF_HIP_FE"] = hip_val;
  qdd_des["RF_LEG_FE"] = knee_val;

  qdd_des["LH_HIP_AA"] = 0;
  qdd_des["LH_HIP_FE"] = hip_val;
  qdd_des["LH_LEG_FE"] = knee_val;

  qdd_des["RH_HIP_AA"] = 0;
  qdd_des["RH_HIP_FE"] = hip_val;
  qdd_des["RH_LEG_FE"] = knee_val;
}

/// The main control loop

void controller(DynamicBodyPtr dbp, double t, void*)
{
    static int ITER = 0;
    static double last_time = 0;
    static Mat uff(NUM_JOINTS,1),
        ufb(NUM_JOINTS,1),
        u(NUM_JOINTS,1),
        q(NUM_JOINTS,1),
        qd(NUM_JOINTS,1);
    static Vec qdd = Vec::zero(NUM_JOINTS);

    double dt = t - last_time;
    if (dt == 0) return;

    /// setup a steady state
    static map<string, double> q_des, qd_des, qdd_des;
    if (q_des.empty())
      for (unsigned m=0; m< NUM_JOINTS; m++)
      {
         q_des[joints_[m]->id] = q0[joints_[m]->id];
         qd_des[joints_[m]->id] = 0.0;
         qdd_des[joints_[m]->id] = 0.0;
      }

#ifdef FOOT_TRAJ
    double step_duration = 0.05;
    static int NSTEPS = step_duration/dt;

    static std::vector<std::vector<Ravelin::Vector3d> > joint_trajectory(NSTEPS);
    static int num_feet = eefs_.size();
    static std::vector<std::vector<Ravelin::Vector3d> > feet_trajectory(num_feet);
    if(feet_trajectory[0].size() == 0)
      for(int i=0;i<num_feet;i++)
        feet_trajectory[i].resize(NSTEPS);

    if(ITER%NSTEPS == 0){
        std::vector<Vector3d> foot_control_points(num_feet);
        for(int f=0;f<num_feet;f++){
          Ravelin::Pose3d link_pose = *eefs_[f]->get_pose();
          // Transform to global (MOBY::GLOBAL) coords
          link_pose.update_relative_pose(Moby::GLOBAL);
          std::cout << eefs_[f]->id << " coords (wrt global): "
                    << link_pose.x << std::endl;
          // Transform to base (ABDOMEN) coords
          link_pose.update_relative_pose(links_[0]->get_pose());
          std::cout << eefs_[f]->id << " coords (wrt base): "
                    << link_pose.x << std::endl;

          Ravelin::Vector3d x0_foot;
          switch(f){
          case 0:
            x0_foot = Ravelin::Vector3d(0.125, 0.0475, -0.101);
            break;
          case 1:
            x0_foot = Ravelin::Vector3d(0.125, -0.0475, -0.101);
            break;
          case 2:
            x0_foot = Ravelin::Vector3d(-0.0922, 0.0475, -0.101);
            break;
          case 3:
            x0_foot = Ravelin::Vector3d(-0.0922, -0.0475, -0.101);
            break;
          default: break;
          }

          // Bezier Curve Strategy
//          foot_control_points[0] = x0_foot;
//          foot_control_points[1] = Ravelin::Vector3d(-0.06,0.0,0.06) + x0_foot;
//          foot_control_points[2] = Ravelin::Vector3d(0.06,0.0,0.06) + x0_foot;
//          foot_control_points[3] = x0_foot;
//          stepTrajectory(foot_control_points,NSTEPS,feet_trajectory[f]);
          // Sinusoidal trajectory
          for(int t = 0;t<NSTEPS;t++){
              double step_time = 2*M_PI*(double)t/(double)NSTEPS;
              feet_trajectory[f][t] = x0_foot + Ravelin::Vector3d(0.03*sin(step_time),0,0);
              feet_trajectory[f][t] = x0_foot + Ravelin::Vector3d(0,0,0.02*sin(step_time));
//              std::cout << feet_trajectory[0][f] << std::endl;
          }
        }

        trajectoryIK(feet_trajectory,joint_trajectory);
    }

    get_q_qd_qdd(joint_trajectory,ITER%NSTEPS,q_des,qd_des,qdd_des);

#endif
    /// Get next Traj step
#ifdef FOLLOW_TRAJECTORY
    get_trajectory(t,dt,q_des,qd_des,qdd_des);
    for(int i=0;i<NUM_JOINTS; i++)
      q_des[joints_[i]->id] += q0[joints_[i]->id];
#endif

#ifdef USE_DUMMY_CONTACTS
    contacts.clear();
    int nc = eefs_.size();

    for(unsigned i=0;i<nc;i++){
      Ravelin::Pose3d pose = *eefs_[i]->get_pose();
      pose.update_relative_pose(Moby::GLOBAL);
      ContactData c;
      c.normal = Ravelin::Vector3d(0,0,1);
      c.point = pose.x;
      c.name = eefs_[i]->id;
      contacts.push_back(c);
    }
#else
     unsigned nc = contacts.size();
#endif

#ifdef RENDER_CONTACT
     Mat support_poly(3,nc);
     for(int c=0;c<nc;c++)
       for(int i=0;i<3;i++)
         support_poly(i,c) = contacts[c].point[i];

     if(nc > 0)
      visualize_polygon(support_poly,sim);
#endif

     uff.set_zero();
     ufb.set_zero();
     u.set_zero();

      ///  Record Robot State
      for(unsigned m=0;m< NUM_JOINTS;m++){
          if(joints_[m]->q.size() == 0) continue;
          q.set_row(m,joints_[m]->q);
          qd.set_row(m,joints_[m]->qd);
      }

      // Use CPG to determine next state

      /// Run friction estimation
      static Mat N,D,M(NDOFS,NDOFS),ST;
      static Vec fext(NDOFS);

      // Get robot dynamics state
      // NOTE: Very Heavy Computation
      calculate_dyn_properties(M,fext);

      static Vec vel(NDOFS), gc(NDOFS+1), acc(NDOFS);

      // et robot state vectors
      dbrobot->get_generalized_acceleration(acc);
      dbrobot->get_generalized_velocity(DynamicBody::eSpatial,vel);
      dbrobot->get_generalized_coordinates(DynamicBody::eSpatial,gc);

#if defined CONTROL_IDYN || defined FRICTION_EST
      // NOTE: Heavy Computation
      determine_contact_jacobians(contacts,N,D,ST);
#endif
      static Mat MU;
      MU.set_zero(nc,NK/2);
#ifdef FRICTION_EST
      static Vec cf;
      double err = friction_estimation(vel,fext,dt,N,D,M,MU,cf);
      OUTLOG(MU,"MU");
      OUTLOG(cf,"contact_forces");
#else
      for(int i=0;i<nc;i++)
        for(int k=0;k<NK/2;k++)
          MU(i,k) = 0.1;
#endif

#ifdef CONTROL_ZMP
      calc_energy(vel,M);
      Vector3d CoM,acc_CoM;
      calc_com(CoM,acc_CoM);

      Vector3d ZmP(CoM[0] - 10*(CoM[2]*acc_CoM[0])
                    /(acc_CoM[2]-9.8),
                   CoM[1] - 10*(CoM[2]*acc_CoM[1])
                    /(acc_CoM[2]-9.8),
                   0);

      Vector3d CoM_2D(CoM);
      CoM_2D[2] = 0;
      visualize_ray(CoM_2D,CoM,sim);
      visualize_ray(ZmP,CoM_2D,sim);
#endif
#ifdef CONTROL_IDYN
      for(int i=0;i<NUM_JOINTS; i++)
        qdd[i] = 0;// qdd_des[joints_[i]->id];
      static Vec ff(NUM_JOINTS);
      ff.set_zero();

      inverse_dynamics(vel,qdd,M,N,D,fext,0.1,MU,ff);
      uff.set_zero();
      uff.set_column(0,ff);
#endif
      contacts.clear();

      ///  Determine FB forces
      control_PID(q_des, qd_des, gains,t,ufb);

      // combine ufb and uff
      u = ufb;
      u += uff;
//      OUTLOG(ufb,"ufb");
//      OUTLOG(uff,"uff");

      /// Limit Torques
//      for(unsigned m=0;m< NUM_JOINTS;m++){
//        if(u(m,0) > u_max[joints_[m]->id])
//          u(m,0) = u_max[joints_[m]->id];
//        else if(u(m,0) < -u_max[joints_[m]->id])
//          u(m,0) = -u_max[joints_[m]->id];
//      }

      // send torque commands to robot
      apply_simulation_forces(u);

#ifdef USE_ROBOT
# ifdef CONTROL_KINEMATICS
      Vec qdat = q.column(0);
      Vec qddat = qd.column(0);
//      dxl_->set_state(qdat.data(),qddat.data());
      dxl_->set_position(qdat.data());
# else
      Vec udat = u.column(0);
      dxl_->set_torque(udat.data());
# endif
#endif

#ifndef NDEBUG
     std::cout << "JOINT\t: U\t| Q\t: des\t: err\t| "
                  "Qd\t: des\t: err\t| @ time = " << t << std::endl;
     for(unsigned m=0;m< NUM_JOINTS;m++)
       std::cout << joints_[m]->id
                 << "\t " <<  std::setprecision(4) << u(m,0)
                 << "\t| " << joints_[m]->q[0]
                 << "\t " << q_des[joints_[m]->id]
                 << "\t " << q(m,0) - q_des[joints_[m]->id]
                 << "\t| " << joints_[m]->qd[0]
                 << "\t " << qd_des[joints_[m]->id]
                 << "\t " <<  qd(m,0) - qd_des[joints_[m]->id]
                 << "\t| " << std::endl;
     std::cout <<"CoM : "<< CoM << std::endl;
     std::cout <<"ZmP : "<< ZmP << std::endl;
     std::cout << std::endl;
#endif
     last_time = t;

# ifdef CONTROL_KINEMATICS
        for(unsigned m=0;m< NUM_JOINTS;m++){
            if(joints_[m]->q.size() == 0) continue;
            joints_[m]->q[0] = q_des[joints_[m]->id];
        }

       Vec q_start(q0.size()+7), qd_start(q0.size()+6);
       qd_start.set_zero();
       q_start.set_zero();
       q_start.set_sub_vec(NUM_JOINTS,BASE_ORIGIN);
       for(int i=0;i<NUM_JOINTS;i++){
         q_start[i] = q_des[joints_[i]->id];
         qd_start[i] = 0;
       }

       abrobot->set_generalized_coordinates(DynamicBody::eEuler,q_start);
       abrobot->set_generalized_velocity(DynamicBody::eSpatial,qd_start);
       abrobot->update_link_poses();
       abrobot->update_link_velocities();
#endif
       ITER++;
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

  // get a reference to the EventDrivenSimulator instance
  for (std::map<std::string, BasePtr>::const_iterator i = read_map.begin();
       i !=read_map.end(); i++)
  {
    // Find the simulator reference
    if (!sim)
      sim = dynamic_pointer_cast<EventDrivenSimulator>(i->second);

    // find the robot reference
    if (!abrobot)
    {
      abrobot = dynamic_pointer_cast<RCArticulatedBody>(i->second);
      dbrobot = dynamic_pointer_cast<DynamicBody>(i->second);
    }
  }

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

  // Set up joint references
  std::vector<JointPtr> joints = abrobot->get_joints();
  joints_.resize(joints.size());

  N_FIXED_JOINTS = 0;
  for(unsigned i=0;i<joints.size();i++){
    if(joints[i]->q.rows() == 0){
      N_FIXED_JOINTS ++;
      continue;
    }
    joints_[joints[i]->get_coord_index()] = joints[i];
     std::cout << joints[i]->get_coord_index() << " "
               << joints_[joints[i]->get_coord_index()]->id << std::endl;
  }

  // Set up link references
  links_ = abrobot->get_links();
  for(unsigned i=0;i<NUM_LINKS;i++)
    std::cout << i << " " << links_[i]->id << std::endl;

  // Set up end effectors
  eef_names_.push_back("LF_FOOT");
  eef_names_.push_back("RF_FOOT");
  eef_names_.push_back("LH_FOOT");
  eef_names_.push_back("RH_FOOT");
  eefs_.resize(eef_names_.size());
  for(unsigned i=0;i<links_.size();i++)
      for(unsigned j=0;j<eef_names_.size();j++)
          if(eef_names_[j].compare(links_[i]->id) == 0)
             eefs_[j] = links_[i];

  NUM_EEFS = eefs_.size();
  NUM_JOINTS = joints_.size() - N_FIXED_JOINTS;
  NUM_LINKS = links_.size();
  NDOFS = NSPATIAL + NUM_JOINTS; // for generalized velocity, forces. accel

  std::cout << "NUM_EEFS: " << NUM_EEFS << std::endl;
  std::cout << "N_FIXED_JOINTS: " << N_FIXED_JOINTS << std::endl;
  std::cout << "NUM_JOINTS: " << NUM_JOINTS << std::endl;
  std::cout << "NDOFS: " << NDOFS << std::endl;
  std::cout << "NSPATIAL: " << NSPATIAL << std::endl;
  std::cout << "NEULER: " << NEULER << std::endl;
  std::cout << "NK: " << NK << std::endl;

  /// LOCALLY SET VALUES
  // robot's initial (ZERO) configuration
  BASE_ORIGIN.set_zero();
  BASE_ORIGIN[2] = 0.1009;
//  BASE_ORIGIN[2] = 0.2009;
  BASE_ORIGIN[6] = 1;

  q0["BODY_JOINT"] = 0;
  q0["LF_HIP_AA"] = 0;
  q0["LF_HIP_FE"] = 0.8;
  q0["LF_LEG_FE"] = 1.4;

  q0["RF_HIP_AA"] =  0;
  q0["RF_HIP_FE"] =  0.8;
  q0["RF_LEG_FE"] =  1.4;

  q0["LH_HIP_AA"] =  0;
  q0["LH_HIP_FE"] =  0.8;
  q0["LH_LEG_FE"] =  1.4;

  q0["RH_HIP_AA"] =  0;
  q0["RH_HIP_FE"] =  0.8;
  q0["RH_LEG_FE"] =  1.4;

  // Maximum torques
  u_max["BODY_JOINT"]=  2.60;
  u_max["LF_HIP_AA"] =  2.60;
  u_max["LF_HIP_FE"] =  2.60;
  u_max["LF_LEG_FE"] =  2.60;

  u_max["RF_HIP_AA"] =  2.60;
  u_max["RF_HIP_FE"] =  2.60;
  u_max["RF_LEG_FE"] =  2.60;

  u_max["LH_HIP_AA"] =  2.60;
  u_max["LH_HIP_FE"] =  6.00;
  u_max["LH_LEG_FE"] =  2.60;

  u_max["RH_HIP_AA"] =  2.60;
  u_max["RH_HIP_FE"] =  6.00;
  u_max["RH_LEG_FE"] =  2.60;

  // Setup gains
  for(unsigned i=0;i<NUM_JOINTS;i++){
    // pass gain values to respective joint
    gains[joints_[i]->id].kp = 0.1;
    gains[joints_[i]->id].kv = 0.02;
    gains[joints_[i]->id].ki = 0;
  }

  // Set Initial State
  Vec q_start(NUM_JOINTS+NEULER),qd_start(NDOFS);

  abrobot->get_generalized_coordinates(DynamicBody::eEuler,q_start);
  qd_start.set_zero();
  q_start.set_sub_vec(NUM_JOINTS,BASE_ORIGIN);

#ifdef FOLLOW_TRAJECTORY
  static map<string, double> q_des, qd_des, qdd_des;
  get_trajectory(0,0.001,q_des,qd_des,qdd_des);
  for(int i=0;i<NUM_JOINTS; i++)
    q_des[joints_[i]->id] += q0[joints_[i]->id];
  for(int i=0;i<NUM_JOINTS;i++){
    q_start[i] = q_des[joints_[i]->id];
    qd_start[i] = qd_des[joints_[i]->id];
  }
#else
  for(int i=0;i<NUM_JOINTS;i++){
    q_start[i] = q0[joints_[i]->id];
    qd_start[i] = 0;
  }
#endif

  // Push initial state to robot
  abrobot->set_generalized_coordinates(DynamicBody::eEuler,q_start);
  abrobot->set_generalized_velocity(DynamicBody::eSpatial,qd_start);

  // If use robot is active also init dynamixel controllers
#ifdef USE_ROBOT
  dxl_ = new Dynamixel;
#endif
}

} // end extern C

