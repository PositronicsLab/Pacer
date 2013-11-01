/*****************************************************************************
 * Simple PD controller
 ****************************************************************************/

#include <Control.h>
#include <iomanip>      // std::setprecision

/// THESE DEFINES DETERMINE WHAT TYPE OF CONTROLLER THIS CODE USES
//#define USE_DUMMY_CONTACTS
//#define CONTROL_IDYN
//#define FRICTION_EST
#define CONTROL_ZMP
#define RENDER_CONTACT
//#define USE_ROBOT

#ifdef USE_ROBOT
  #include <Dynamixel.h>
#endif

using namespace Moby;
using namespace Ravelin;

std::vector<std::string> eef_names_, joint_name_,link_names_;
std::vector<ContactData> contacts;
std::vector<RigidBodyPtr> eefs;
static map<string, double> q0;

// simulator
boost::shared_ptr<EventDrivenSimulator> sim;
// robot pointer
RCArticulatedBodyPtr abrobot;
DynamicBodyPtr dbrobot;
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
  const std::vector<JointPtr>& joints = abrobot->get_joints();

  // clear and set motor torques
  for (unsigned m=0,i=0; m< joints.size(); m++)
  {
    // get the joint
    JointPtr j = joints[m];
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
      visualize_contact(e[i],sim);
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
    const std::vector<JointPtr>& joints = abrobot->get_joints();
    for(unsigned m=0,i=0;m< joints.size();m++){
        if(joints[m]->q.size() == 0) continue;
        // reset motor torque
        Vec row;
        joints[m]->reset_force();
        joints[m]->add_force(u.get_row(i,row));
        i++;
    }
}


Vector3d& calc_com(Vector3d& com){
  const std::vector<RigidBodyPtr>& links = abrobot->get_links();
  for(int i=0;i<links.size();i++){
     RigidBody& link = *links[i];
     double m = link.get_mass();
//     Vector3d comi = link.get_inertial_pose(); // NOTE: where is this func?
  }
}

/// The main control loop
void controller(DynamicBodyPtr dbp, double t, void*)
{
    static double last_time = 0;
    static Mat uff(NJOINT,1),ufb(NJOINT,1),u(NJOINT,1), q(NJOINT,1), qd(NJOINT,1);
    double dt = t - last_time;

#ifdef USE_DUMMY_CONTACTS
    // PROCESS CONTACTS
    contacts.clear();
    int nc = 4;
    const std::vector<RigidBodyPtr>& links = abrobot->get_links();
    std::vector<RigidBodyPtr> eefs(nc);

    for(unsigned i=0;i<links.size();i++)
        for(unsigned j=0;j<nc;j++)
            if(eef_names_[j].compare(links[i]->id) == 0)
                eefs[j] = links[i];

    for(unsigned i=0;i<nc;i++){
      Ravelin::VectorNd gc;
      eefs[i]->get_generalized_coordinates(DynamicBody::eSpatial,gc);
      ContactData c;
      c.normal = Ravelin::Vector3d(0,0,1);
      gc.get_sub_vec(0,3,c.point);
      c.name = eef_names_[i];
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
     outlog2(support_poly,"Support Polygon");

     if(nc > 0)
      visualize_polygon(support_poly,sim);


#endif

     //static double test_frict_val = 0.5;
     static unsigned ITER = 1;

     uff.set_zero();
     ufb.set_zero();
     u.set_zero();

      /// setup a steady state
      static map<string, double> q_des, qd_des;
      const std::vector<JointPtr>& joints = abrobot->get_joints();
      if (q_des.empty())
        for (unsigned m=0; m< joints.size(); m++)
        {
            if(joints[m]->q.size() == 0) continue;
            q_des[joints[m]->id] = q0[joints[m]->id];
            qd_des[joints[m]->id] = 0.0;
        }

      ///  Record Robot State
      for(unsigned m=0;m< joints.size();m++){
          if(joints[m]->q.size() == 0) continue;
          unsigned ind = joints[m]->get_coord_index();
          q.set_row(ind,joints[m]->q);
          qd.set_row(ind,joints[m]->qd);
      }

      outlog2(q.column(0),"q");
      outlog2(qd.column(0),"qd");

      /// Run friction estimation
      static Mat N,D,M(NDOFS,NDOFS);
      static Vec fext(NDOFS);

      calculate_dyn_properties(M,fext);
      static Vec vel(NDOFS), gc(NDOFS+1),acc(NDOFS);
      dbrobot->get_generalized_acceleration(acc);
      dbrobot->get_generalized_velocity(DynamicBody::eSpatial,vel);
      dbrobot->get_generalized_coordinates(DynamicBody::eSpatial,gc);

      determine_N_D(contacts,N,D);

      static Mat MU;
      MU.set_zero(nc,1);
#ifdef FRICTION_EST
      static Mat ST;
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
         MU(i,0) = 0.01;
#endif

#ifdef CONTROL_ZMP
      // NOTE: This only finds ZMP of Base
      Vector2d ZmP(
                    gc[(NJOINT-1)] - (gc[(NJOINT-1)+2]*acc[(NJOINT-1)])/acc[(NJOINT-1)+2],
                    gc[(NJOINT-1)+1] - (gc[(NJOINT-1)+2]*acc[(NJOINT-1)+1])/acc[(NJOINT-1)+2]
                    );

      // Find true COM
      Vector3d CoM;
      calc_com(CoM);
      std::cout <<"CoM : "<< gc.get_sub_vec((NJOINT-1),(NJOINT-1)+2,workv_) << std::endl;
      std::cout <<"ZmP : "<< ZmP << std::endl;
#endif
      static Vec qdd = Vec::zero(NJOINT);
      static Vec cf, ff = uff.column(0);

#ifdef CONTROL_IDYN
      idyn(vel,qdd,M,N,D,fext,dt,MU,ff);
      uff.set_column(0,ff);
#endif
      contacts.clear();

      ///  Determine FB forces
      control_PID(q_des, qd_des, gains,t,ufb);

      // combine ufb and uff
      u += ufb;
      u += uff;

      apply_simulation_forces(u);

//    outlog2(u.transpose(),"u"); u.transpose();
    last_time = t;
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

  sim->event_post_impulse_callback_fn = &post_event_callback_fn;
    // setup the controller
  abrobot->controller = &controller;

  const std::vector<JointPtr>& joints = abrobot->get_joints();
  for(unsigned i=0;i<joints.size();i++){
    std::cout << i << " : " << joints[i]->id  << joints[i]->q<< std::endl;
    joint_name_.push_back(joints[i]->id);
  }

  const std::vector<RigidBodyPtr>& links = abrobot->get_links();
  for(unsigned i=0;i<links.size();i++){
    std::cout << i << " : " << links[i]->id <<std::endl;
    link_names_.push_back(links[i]->id);
  }

  /// LOCALLY SET VALUES
  // robot's go0 configuration
  q0["BODY_JOINT"] =  0.0;
  q0["LF_HIP_AA"] = 0;
  q0["LF_HIP_FE"] = 3*M_PI_4;
  q0["LF_LEG_FE"] = -M_PI_2;

  q0["RF_HIP_AA"] = 0;
  q0["RF_HIP_FE"] =  3*M_PI_4;
  q0["RF_LEG_FE"] = -M_PI_2;

  q0["LH_HIP_AA"] = 0;
  q0["LH_HIP_FE"] = 3*M_PI_4;
  q0["LH_LEG_FE"] = -M_PI_2;

  q0["RH_HIP_AA"] = 0;
  q0["RH_HIP_FE"] = 3*M_PI_4;
  q0["RH_LEG_FE"] = -M_PI_2;

  eef_names_.push_back("LF_LLEG");
  eef_names_.push_back("RF_LLEG");
  eef_names_.push_back("LH_LLEG");
  eef_names_.push_back("RH_LLEG");

  gains[joint_name_[0]].kp = 100;
  gains[joint_name_[0]].kv = 10;
  gains[joint_name_[0]].ki = 0;
  // now, setup gains
  for(unsigned i=1;i<joint_name_.size();i++){
    double kp,kv,ki;
    switch((i-1)%3){
    case 0:
      kp = 100;
      kv = 10;
      ki = 0;
      break;
    case 1:
      kp = 100;
      kv = 10;
      ki = 0;
      break;
    case 2:
      kp = 100;
      kv = 10;
      ki = 0;
      break;
    default: break;
    }
    // pass gain values to respective joint
    gains[joint_name_[i]].kp = kp;
    gains[joint_name_[i]].kv = kv;
    gains[joint_name_[i]].ki = ki;
  }

  Vec q_start(q0.size()+7),qd_start(q0.size()+6);
  abrobot->get_generalized_coordinates(DynamicBody::eEuler,q_start);
  qd_start.set_zero();

  for(int i=0;i<q0.size();i++){
      int ind = abrobot->find_joint(joint_name_[i])->get_coord_index();
      q_start[ind] = q0[joint_name_[i]];
  }

  abrobot->set_generalized_coordinates(DynamicBody::eEuler,q_start);
  abrobot->set_generalized_velocity(DynamicBody::eSpatial,qd_start);

}

} // end extern C

