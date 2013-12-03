/*****************************************************************************
 * Simple PD controller
 ****************************************************************************/

#include <Control.h>
#include <iomanip>      // std::setprecision

/// THESE DEFINES DETERMINE WHAT TYPE OF CONTROLLER THIS CODE USES
#define NDEBUG
//#define USE_DUMMY_CONTACTS
//#define CONTROL_IDYN
//#define FRICTION_EST
#define CONTROL_ZMP
//#define RENDER_CONTACT
//#define USE_ROBOT
#define CONTROL_KINEMATICS
//#define FOLLOW_TRAJECTORY
//#define FOOT_TRAJ

#ifdef USE_ROBOT
  #include <dxl/Dynamixel.h>
    Dynamixel* dxl_;
#endif

#define grav 9.8 // M/s.s
#define M_PI_16 0.19634954084
#define M_PI_8 0.39269908169
const double TORQUE_LIMIT = 3; //N.m

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
void control_PID(const map<string, double>& q_des, const map<string, double>& qd_des, const map<string, Gains>& gains, double time,Mat& ufb)
{
  // clear and set motor torques
  for (unsigned m=0,i=0; m< N_JOINTS; m++)
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
void post_event_callback_fn(const vector<Event>& e, boost::shared_ptr<void> empty)
{  
  // PROCESS CONTACTS
  contacts.clear();
  int nc = e.size();
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

      if (std::find(eef_names_.begin(), eef_names_.end(), sb1->id) == eef_names_.end())
        continue;

      bool contact_used = false;
      for(int j=0;j<contacts.size();j++)
        if(contacts[j].name.compare(sb1->id) == 0)
          contact_used = true;

      if(contact_used)
        continue;

      c.point = e[i].contact_point;
      c.name = sb1->id;

      contacts.push_back(c);
    }
  }
}

/// Event callback function for setting friction vars pre-event
void pre_event_callback_fn(vector<Event>& e, boost::shared_ptr<void> empty){}

void apply_simulation_forces(const Mat& u){
    for(unsigned m=0,i=0;m< N_JOINTS;m++){
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
  std::cout << "KE = " << KE << ", PE = " << PE << std::endl;
  std::cout << "Total Energy = " << (KE + PE) << std::endl;
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

void get_trajectory(double time,double dt, map<string, double>& q_des, map<string, double>& qd_des){
  double joint_vel = 0, joint_pos = 0;

  double t = time;

  if(0){

  } else {
    joint_pos = M_PI_8*sin(-t*3);
    q_des["LF_HIP_FE"] =  q0["LF_HIP_FE"] + joint_pos;
    joint_pos = M_PI_8*sin(-t*3-M_PI_2);
    q_des["LF_LEG_FE"] =  q0["LF_LEG_FE"] + joint_pos;

    joint_vel = -3*M_PI_8*cos(-t*3);
    qd_des["LF_HIP_FE"] =  joint_vel;
    joint_vel = -3*M_PI_8*cos(-t*3-M_PI_2);
    qd_des["LF_LEG_FE"] =  joint_vel;


    joint_pos = M_PI_8*sin(-t*3);
    q_des["RF_HIP_FE"] =  q0["RF_HIP_FE"] + joint_pos;
    joint_pos = M_PI_8*sin(-t*3-M_PI_2);
    q_des["RF_LEG_FE"] =  q0["RF_LEG_FE"] + joint_pos;

    joint_vel = -3*M_PI_8*cos(-t*3);
    qd_des["RF_HIP_FE"] =  joint_vel;
    joint_vel = -3*M_PI_8*cos(-t*3-M_PI_2);
    qd_des["RF_LEG_FE"] =  joint_vel;

    ///////////////

    joint_pos = M_PI_8*sin(t*3);
    q_des["LH_HIP_FE"] =  q0["LH_HIP_FE"] + joint_pos;
    joint_pos = M_PI_8*sin(t*3-M_PI_2);
    q_des["LH_LEG_FE"] =  q0["LH_LEG_FE"] + joint_pos;

    joint_vel = 3*M_PI_8*cos(t*3);
    qd_des["LH_HIP_FE"] =  joint_vel;
    joint_vel = 3*M_PI_8*cos(t*3-M_PI_2);
    qd_des["LH_LEG_FE"] =  joint_vel;


    joint_pos = M_PI_8*sin(t*3);
    q_des["RH_HIP_FE"] =  q0["RH_HIP_FE"] + joint_pos;
    joint_pos = M_PI_8*sin(t*3-M_PI_2);
    q_des["RH_LEG_FE"] =  q0["RH_LEG_FE"] + joint_pos;

    joint_vel = 3*M_PI_8*cos(t*3);
    qd_des["RH_HIP_FE"] =  joint_vel;
    joint_vel = 3*M_PI_8*cos(t*3-M_PI_2);
    qd_des["RH_LEG_FE"] =  joint_vel;

    q_des["BODY_JOINT"] = 0;
    q_des["LF_HIP_AA"] =  M_PI_8;
    q_des["RF_HIP_AA"] = -M_PI_8;
    q_des["LH_HIP_AA"] =  M_PI_8;
    q_des["RH_HIP_AA"] = -M_PI_8;
    qd_des["BODY_JOINT"] = 0;
    qd_des["LF_HIP_AA"] = 0;
    qd_des["RF_HIP_AA"] = 0;
    qd_des["LH_HIP_AA"] = 0;
    qd_des["RH_HIP_AA"] = 0;
  }
}

/// The main control loop

void controller(DynamicBodyPtr dbp, double t, void*)
{
    static int ITER = 0;
    static double last_time = 0;
    static Mat uff(NJOINT,1),ufb(NJOINT,1),u(NJOINT,1), q(NJOINT,1), qd(NJOINT,1);
    double dt = t - last_time;

    /// setup a steady state
    static map<string, double> q_des, qd_des;
    if (q_des.empty())
      for (unsigned m=0; m< N_JOINTS; m++)
      {
        if(joints_[m]->q.size() == 0) continue; // NOTE: Currently this is not used
         q_des[joints_[m]->id] = q0[joints_[m]->id];
         qd_des[joints_[m]->id] = 0.0;
      }

#ifdef FOOT_TRAJ
    static int NSTEPS = 10;
    static std::vector<std::vector<Ravelin::Vector3d> > joint_trajectory(NSTEPS);
    if(ITER%NSTEPS == 0){
      std::vector<Vector3d> foot_control_points(4);
      std::vector<Ravelin::Vector3d> feet_trajectory(4);

      Ravelin::Pose3d link_pose = *eefs_[0]->get_pose();
      link_pose.x[0] += 0.04;
      link_pose.update_relative_pose(links_[0]->get_pose());
      std::cout << "link_pose: " << link_pose.x << std::endl;

      foot_control_points[0] = link_pose.x;
      foot_control_points[1] = link_pose.x;
      foot_control_points[1][2] += 0.2;
      foot_control_points[2] = link_pose.x;
      foot_control_points[2][0] += 0.2;
      foot_control_points[3] = link_pose.x;

      for(unsigned j=0;j<foot_control_points.size();j++)
        std::cout << "Foot CP: " << foot_control_points[j] << std::endl;

      std::vector<Vector3d> foot_trajectory(NSTEPS);
      stepTrajectory(foot_control_points,foot_trajectory);
      foot_trajectory[0] = link_pose.x;
      for(unsigned j=0;j<foot_trajectory.size();j++){
          feet_trajectory[0] = foot_trajectory[j];
          feet_trajectory[1] = Ravelin::Vector3d::zero();
          feet_trajectory[2] = Ravelin::Vector3d::zero();
          feet_trajectory[3] = Ravelin::Vector3d::zero();
          std::cout << "Step" << std::endl;
          std::cout << "Foot Loc: " << foot_trajectory[0] << std::endl;
          joint_trajectory[j] = std::vector<Vector3d>(4);
          trajectoryIK(feet_trajectory,joint_trajectory[j]);
          std::cout << "Joint Loc: " << joint_trajectory[j][0] << std::endl;
      }
    }
    std::cout << "Joint Des Loc[" <<  ITER%NSTEPS << "]: " << joint_trajectory[ITER%NSTEPS][0] << std::endl;
    q_des["LF_HIP_AA"] = 0;
    q_des["LF_HIP_FE"] = joint_trajectory[ITER%NSTEPS][0][1];
    q_des["LF_LEG_FE"] = joint_trajectory[ITER%NSTEPS][0][2];

    std::cout << "Joint True Loc: " << joints_[3]->q[0] << "," << joints_[7]->q[0]<< "," << joints_[11]->q[0] << std::endl;
    std::cout << "Joint Des Loc: " << q_des["LF_HIP_AA"] << "," << q_des["LF_HIP_FE"] << "," <<  q_des["LF_LEG_FE"] << std::endl;
#endif

#ifdef USE_DUMMY_CONTACTS
    contacts.clear();
    int nc = 4;
    std::vector<RigidBodyPtr> eefs_(nc);

    for(unsigned i=0;i<links.size();i++)
        for(unsigned j=0;j<nc;j++)
            if(eef_names_[j].compare(links[i]->id) == 0)
                eefs_[j] = links[i];

    for(unsigned i=0;i<nc;i++){
      Ravelin::Pose3d pose = *eefs_[i]->get_gc_pose();
      pose->x[0] += 0.04;
      pose.update_relative_pose(Moby::GLOBAL);
      ContactData c;
      c.normal = Ravelin::Vector3d(0,0,1);
      c.point = pose->x;
      c.point =
      c.name = eefs_[i]->id;
      std::cout << c.name << std::endl;
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

      /// Get next Traj step
#ifdef FOLLOW_TRAJECTORY
      get_trajectory(t,dt,q_des,qd_des);
#endif

      ///  Record Robot State
      for(unsigned m=0;m< N_JOINTS;m++){
          if(joints_[m]->q.size() == 0) continue;
          q.set_row(m,joints_[m]->q);
          qd.set_row(m,joints_[m]->qd);
      }

      /// Run friction estimation
      static Mat N,D,M(NDOFS,NDOFS);
      static Vec fext(NDOFS);

      calculate_dyn_properties(M,fext);

      //floating robot
//      Vec resist_base_forces = fext;
//      resist_base_forces.negate();
//      for(int i=0;i<NJOINT;i++)
//        resist_base_forces[i] = 0;
//      abrobot->add_generalized_force(resist_base_forces);
//      outlog2(fext,"fext");
//      outlog2(resist_base_forces,"resist_base_forces");

      static Vec vel(NDOFS), gc(NDOFS+1), acc(NDOFS);
      dbrobot->get_generalized_acceleration(acc);
      dbrobot->get_generalized_velocity(DynamicBody::eSpatial,vel);
      dbrobot->get_generalized_coordinates(DynamicBody::eSpatial,gc);

      determine_N_D(contacts,N,D);
      static Mat ST;
      determine_N_D2(contacts,N,ST);

      static Mat MU;
      MU.set_zero(nc,1);
#ifdef FRICTION_EST
      static Vec cf;


      double err = friction_estimation(vel,fext,dt,N,D,M,MU,cf);
      outlog2(MU,"MU");
      outlog2(cf,"contact_forces");
#else
      for(int i=0;i<nc;i++)
         MU(i,0) = 0.1;
#endif

#ifdef CONTROL_ZMP
      calc_energy(vel,M);
      Vector3d CoM,acc_CoM;
      calc_com(CoM,acc_CoM);

      Vector3d ZmP( CoM[0] - 10*(CoM[2]*acc_CoM[0])/(acc_CoM[2]-9.8),
                    CoM[1] - 10*(CoM[2]*acc_CoM[1])/(acc_CoM[2]-9.8),
                    0 );

      Vector3d CoM_2D(CoM);
      CoM_2D[2] = 0;
      visualize_ray(CoM_2D,CoM,sim);
      visualize_ray(ZmP,CoM_2D,sim);
#endif
      static Vec qdd = Vec::zero(NJOINT);
      static Vec ff = uff.column(0);

#ifdef CONTROL_IDYN
      if(nc > 0)
        idyn(vel,qdd,M,N,D,fext,0.1,MU,ff);
      uff.set_column(0,ff);
#endif
      contacts.clear();

      ///  Determine FB forces
      control_PID(q_des, qd_des, gains,t,ufb);

      // combine ufb and uff
      u += ufb;
      u += uff;
//      outlog2(ufb,"ufb");
//      outlog2(uff,"uff");

      /// Limit Torques
      for(unsigned m=0;m< N_JOINTS;m++){
        if(u(m,0) > u_max[joints_[m]->id])
          u(m,0) = u_max[joints_[m]->id];
        else if(u(m,0) < -u_max[joints_[m]->id])
          u(m,0) = -u_max[joints_[m]->id];
      }

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
      outlog2(q.column(0),"q");
      std::cout << "q_des: ";
      for(unsigned m=0;m< N_JOINTS;m++){
        std::cout << q_des[joints_[m]->id] << " ";
      }
      std::cout << std::endl;

      outlog2(qd.column(0),"qd");
      std::cout << "qd_des: ";
      for(unsigned m=0;m< N_JOINTS;m++){
        std::cout << qd_des[joints_[m]->id] << " ";
      }
      std::cout << std::endl;


# ifdef CONTROL_ZMP
      std::cout <<"CoM : "<< gc.get_sub_vec((NJOINT-1),(NJOINT-1)+2,workv_) << std::endl;
      std::cout <<"ZmP : "<< ZmP << std::endl;
# endif

# ifdef RENDER_CONTACT
     outlog2(support_poly,"Support Polygon");
# endif


#endif

     std::cout << "JOINT\t: U\t| Q\t: des\t: err\t| Qd\t: des\t: err\t| @ time = " << t << std::endl;
     for(unsigned m=0;m< N_JOINTS;m++)
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

     last_time = t;


# ifdef CONTROL_KINEMATICS
     //       for(unsigned m=0;m< N_JOINTS;m++){
     //           if(joints_[m]->q.size() == 0) continue;
     //           q(m,0) = q_des[joints_[m]->id];
     //           qd(m,0) = qd_des[joints_[m]->id];
     //           joints_[m]->q[0] = q_des[joints_[m]->id];
     //           joints_[m]->qd[0] = qd_des[joints_[m]->id];
     //       }

       Vec q_start(q0.size()+7),qd_start(q0.size()+6);
       qd_start.set_zero();
       q_start.set_zero();
       q_start[NJOINT+2] = 0.1;
       for(int i=0;i<N_JOINTS;i++){
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

void init(void* separator, const std::map<std::string, BasePtr>& read_map, double time)
{
  // get a reference to the EventDrivenSimulator instance
  for (std::map<std::string, BasePtr>::const_iterator i = read_map.begin(); i !=read_map.end(); i++)
  {
    if (!sim)
      sim = dynamic_pointer_cast<EventDrivenSimulator>(i->second);

    // find the robot
    if (!abrobot)
    {
      abrobot = dynamic_pointer_cast<RCArticulatedBody>(i->second);
      dbrobot = dynamic_pointer_cast<DynamicBody>(i->second);
    }
  }

#ifdef CONTROL_KINEMATICS
  abrobot->set_kinematic(true);
#endif

  sim->event_post_impulse_callback_fn = &post_event_callback_fn;
    // setup the controller
  abrobot->controller = &controller;

  std::vector<JointPtr> joints = abrobot->get_joints();
  joints_.resize(joints.size());
  for(unsigned i=0;i<joints.size();i++){
    joints_[joints[i]->get_coord_index()] = joints[i];
     std::cout << joints[i]->get_coord_index() << " "<< joints_[joints[i]->get_coord_index()]->id << std::endl;
  }
  links_ = abrobot->get_links();
  for(unsigned i=0;i<links_.size();i++)
    std::cout << i << " " << links_[i]->id << std::endl;

  eef_names_.push_back("LF_FOOT");
  eef_names_.push_back("RF_FOOT");
  eef_names_.push_back("LH_FOOT");
  eef_names_.push_back("RH_FOOT");

  eefs_.resize(eef_names_.size());
  for(unsigned i=0;i<links_.size();i++)
      for(unsigned j=0;j<eef_names_.size();j++)
          if(eef_names_[j].compare(links_[i]->id) == 0)
             eefs_[j] = links_[i];

  /// LOCALLY SET VALUES
  // robot's go0 configuration
  q0["BODY_JOINT"] = 0;
  q0["LF_HIP_AA"] =  0;
  q0["LF_HIP_FE"] = 0.6;
  q0["LF_LEG_FE"] = 1.6;

  q0["RF_HIP_AA"] =  0.1;
  q0["RF_HIP_FE"] =  0.8;
  q0["RF_LEG_FE"] =  1.4;

  q0["LH_HIP_AA"] =  0.1;
  q0["LH_HIP_FE"] =  0.8;
  q0["LH_LEG_FE"] =  1.4;

  q0["RH_HIP_AA"] =  0.1;
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

  gains[joints_[0]->id].kp = 0.1;
  gains[joints_[0]->id].kv = 0.01;
  gains[joints_[0]->id].ki = 0;
  // now, setup gains
  for(unsigned i=1;i<N_JOINTS;i++){
    double kp,kv,ki;
    switch((i-1)%3){
    case 0:
      kp = 0.1;
      kv = 0.01;
      ki = 0;
      break;
    case 1:
      kp = 0.1;
      kv = 0.01;
      ki = 0;
      break;
    case 2:
      kp = 0.1;
      kv = 0.01;
      ki = 0;
      break;
    default: break;
    }
    // pass gain values to respective joint
    gains[joints_[i]->id].kp = kp;
    gains[joints_[i]->id].kv = kv;
    gains[joints_[i]->id].ki = ki;
  }

  Vec q_start(q0.size()+7),qd_start(q0.size()+6);
  abrobot->get_generalized_coordinates(DynamicBody::eEuler,q_start);
  qd_start.set_zero();

  for(int i=0;i<N_JOINTS;i++){
    q_start[i] = q0[joints_[i]->id];
    qd_start[i] = 0;
  }

  abrobot->set_generalized_coordinates(DynamicBody::eEuler,q_start);
  abrobot->set_generalized_velocity(DynamicBody::eSpatial,qd_start);

#ifdef USE_ROBOT
  dxl_ = new Dynamixel;
#endif
}

} // end extern C

