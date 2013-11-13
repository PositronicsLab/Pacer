/*****************************************************************************
 * Simple PD controller
 ****************************************************************************/

#include <Control.h>
#include <iomanip>      // std::setprecision

/// THESE DEFINES DETERMINE WHAT TYPE OF CONTROLLER THIS CODE USES
#define NDEBUG
//#define USE_DUMMY_CONTACTS
//#define CONTROL_IDYN
#define FRICTION_EST
#define CONTROL_ZMP
#define RENDER_CONTACT
//#define USE_ROBOT
#define CONTROL_KINEMATICS

#ifdef USE_ROBOT
  #include <dxl/Dynamixel.h>
    Dynamixel* dxl_;
#endif
#define M_PI_16 0.19634954084
#define M_PI_8 0.39269908169
const double TORQUE_LIMIT = 3; //N.m

using namespace Moby;
using namespace Ravelin;

std::vector<std::string> eef_names_;
std::vector<ContactData> contacts;
std::vector<RigidBodyPtr> eefs;
static map<string, double> q0,u_max;

// simulator
boost::shared_ptr<EventDrivenSimulator> sim;
// robot pointer
RCArticulatedBodyPtr abrobot;
DynamicBodyPtr dbrobot;
 std::vector<JointPtr> joints_;
 std::vector<RigidBodyPtr> links_;


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
  for (unsigned m=0,i=0; m< joints_.size(); m++)
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

void determine_N_D(std::vector<ContactData>& contacts, Mat& N, Mat& D)
{
  int nc = contacts.size();
  const std::vector<RigidBodyPtr>& links = abrobot->get_links();
  std::vector<RigidBodyPtr> eefs(nc);

  for(unsigned i=0;i<links.size();i++)
    for(unsigned j=0;j<nc;j++)
      if(contacts[j].name.compare(links[i]->id) == 0)
        eefs[j] = links[i];

  static Mat J, Jsub;
  static Vec col;

  // resize temporary N and ST
  N.resize(NJOINT+6,nc);
  D.resize(NJOINT+6,nc*nk);

  J.resize(NSPATIAL, NJOINT);

  // loop over all contacts
  for(int i = 0; i < nc; i++){
    // get the contact point and normal
    ContactData& c = contacts[i];

    // generate the contact tangents here...
    Vector3d tan1, tan2;
    Vector3d::determine_orthonormal_basis(c.normal, tan1, tan2);
    Vector3d torque;
    torque.set_zero();

    RigidBodyPtr sbfoot = eefs[i];

    Vec col(NSPATIAL);
    AAngled aa(0,0,1,0);
    Origin3d o(c.point);
    boost::shared_ptr<const Ravelin::Pose3d> pose(new Pose3d(aa,o));
    SForced sfn(c.normal,torque,pose);
    abrobot->convert_to_generalized_force(sbfoot,sfn, c.point, col);
    N.set_column(i,col);
    for(int k=0;k<nk;k++){
      if(k%2 == 0) {
        SForced sfs(tan1,torque,pose);
        abrobot->convert_to_generalized_force(sbfoot,sfs, c.point, col);
      } else {
        SForced sft(tan2,torque,pose);
        abrobot->convert_to_generalized_force(sbfoot,sft, c.point, col);
      }
      if(k>=2) col.negate();
      D.set_column(i*nk + k,col);
    }
  }
}

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
    for(unsigned m=0,i=0;m< joints_.size();m++){
        if(joints_[m]->q.size() == 0) continue;
        // reset motor torque
        Vec row;
        joints_[m]->reset_force();
        joints_[m]->add_force(u.get_row(i,row));
        i++;
    }
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
    static double last_time = 0;
    static Mat uff(NJOINT,1),ufb(NJOINT,1),u(NJOINT,1), q(NJOINT,1), qd(NJOINT,1);
    double dt = t - last_time;

#ifdef USE_DUMMY_CONTACTS
    contacts.clear();
    int nc = 4;
    const std::vector<RigidBodyPtr>& links = abrobot->get_links();
    std::vector<RigidBodyPtr> eefs(nc);

    for(unsigned i=0;i<links.size();i++)
        for(unsigned j=0;j<nc;j++)
            if(eef_names_[j].compare(links[i]->id) == 0)
                eefs[j] = links[i];

    for(unsigned i=0;i<nc;i++){
      boost::shared_ptr<const Ravelin::Pose3d> pose = eefs[i]->get_gc_pose();
      ContactData c;
      c.normal = Ravelin::Vector3d(0,0,1);
      c.point = -pose->x;
      c.name = eefs[i]->id;
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

      /// setup a steady state
      static map<string, double> q_des, qd_des;
      if (q_des.empty())
        get_trajectory(0,0.001,q_des,qd_des);
//        for (unsigned m=0; m< joints_.size(); m++)
//        {
//          if(joints_[m]->q.size() == 0) continue; // NOTE: Currently this is not used
//           q_des[joints_[m]->id] = q0[joints_[m]->id];
//           qd_des[joints_[m]->id] = 0.0;
//        }

      /// Get next Traj step
//      get_trajectory(t,dt,q_des,qd_des);

      ///  Record Robot State
      for(unsigned m=0;m< joints_.size();m++){
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

      static Mat MU;
      MU.set_zero(nc,1);
#ifdef FRICTION_EST
      static Mat ST;
      static Vec cf;
      ST.set_zero(D.rows(),D.columns()/2);
      // remove negations from D to create ST
      // of the form [S T]

      for(int i=0;i<N.columns();i++){
        for(int j=0;j<N.rows();j++){
          ST(j,i) = D(j,i*nk);
          ST(j,nc+i) = D(j,i*nk+1);
        }
      }

      double err = friction_estimation(vel,fext,dt,N,D,M,MU,cf);
      outlog2(MU,"MU");
      outlog2(cf,"contact_forces");
#else
      for(int i=0;i<nc;i++)
         MU(i,0) = 0.1;
#endif

#ifdef CONTROL_ZMP
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
      outlog2(ufb,"ufb");
      outlog2(uff,"uff");

      /// Limit Torques
      for(unsigned m=0;m< joints_.size();m++){
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
      for(unsigned m=0;m< joints_.size();m++){
        std::cout << q_des[joints_[m]->id] << " ";
      }
      std::cout << std::endl;

      outlog2(qd.column(0),"qd");
      std::cout << "qd_des: ";
      for(unsigned m=0;m< joints_.size();m++){
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
     for(unsigned m=0;m< joints_.size();m++)
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


     { // NOTE: Code for kinematic simulation
       for(unsigned m=0;m< joints_.size();m++){
           if(joints_[m]->q.size() == 0) continue;
           q(m,0) = q_des[joints_[m]->id];
           qd(m,0) = qd_des[joints_[m]->id];
           joints_[m]->q[0] = q_des[joints_[m]->id];
           joints_[m]->qd[0] = qd_des[joints_[m]->id];
       }
       abrobot->update_link_poses();
       abrobot->update_link_velocities();
     }
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

//  sim->event_post_impulse_callback_fn = &post_event_callback_fn;
    // setup the controller
//  abrobot->controller = &controller;

  std::vector<JointPtr> joints = abrobot->get_joints();
  joints_.resize(joints.size());
  for(unsigned i=0;i<joints.size();i++){
    joints_[joints[i]->get_coord_index()] = joints[i];
  }
  links_ = abrobot->get_links();

  /// LOCALLY SET VALUES
  // robot's go0 configuration
  q0["BODY_JOINT"] = 0;
  q0["LF_HIP_AA"] =  M_PI_8;
  q0["LF_HIP_FE"] =  M_PI_2;
  q0["LF_LEG_FE"] = -M_PI_2;

  q0["RF_HIP_AA"] = -M_PI_8;
  q0["RF_HIP_FE"] =  M_PI_2;
  q0["RF_LEG_FE"] =  -M_PI_2;

  q0["LH_HIP_AA"] =  M_PI_8;
  q0["LH_HIP_FE"] =  M_PI_2;
  q0["LH_LEG_FE"] =  -M_PI_2;

  q0["RH_HIP_AA"] = -M_PI_8;
  q0["RH_HIP_FE"] =  M_PI_2;
  q0["RH_LEG_FE"] =  -M_PI_2;

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

  eef_names_.push_back("LF_LLEG");
  eef_names_.push_back("RF_LLEG");
  eef_names_.push_back("LH_LLEG");
  eef_names_.push_back("RH_LLEG");

  gains[joints_[0]->id].kp = 0.1;
  gains[joints_[0]->id].kv = 0.01;
  gains[joints_[0]->id].ki = 0;
  // now, setup gains
  for(unsigned i=1;i<joints_.size();i++){
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

  static map<string, double> q_des, qd_des;
  get_trajectory(0,0.001,q_des,qd_des);

  for(int i=0;i<joints_.size();i++){
    q_start[i] = q_des[joints_[i]->id];
    qd_start[i] = 0;//qd_des[joints_[i]->id];
  }

  abrobot->set_generalized_coordinates(DynamicBody::eEuler,q_start);
  abrobot->set_generalized_velocity(DynamicBody::eSpatial,qd_start);


#ifdef USE_ROBOT
  dxl_ = new Dynamixel;
#endif
}

} // end extern C

