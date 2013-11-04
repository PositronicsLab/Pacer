/*****************************************************************************
 * Simple PD controller 
 ****************************************************************************/

#include <cmath>
#include <sys/types.h>
#include <sys/times.h>
#include <fstream>

#include <boost/shared_ptr.hpp>
#include <Ravelin/MatrixNd.h>
#include <Ravelin/VectorNd.h>
#include <Ravelin/Vector3d.h>
#include <Ravelin/LinAlgd.h>
#include <Ravelin/AAngled.h>
#include <Ravelin/SForced.h>


#include <Moby/Simulator.h>
#include <Moby/EventDrivenSimulator.h>
#include <Moby/RCArticulatedBody.h>
#include <Moby/DynamicBody.h>
#include <Moby/RevoluteJoint.h>
#include <Moby/GravityForce.h>
#include <Moby/Constants.h>
#include <Moby/RNEAlgorithm.h>

using namespace Moby;
using namespace Ravelin;
using boost::shared_ptr;
using boost::dynamic_pointer_cast;
using std::string;
using std::map;
using std::pair;
using std::vector;

enum RobotDOFs {
  LF_HFE = 0,
  LF_HAA,
  LF_KFE,

  LH_HFE,
  LH_HAA,
  LH_KFE,

  RF_HFE,
  RF_HAA,
  RF_KFE,

  RH_HFE,
  RH_HAA,
  RH_KFE,

  NJOINT
};

const unsigned NUM_EEFS = 4,
               NDOFS = NJOINT+6,
               N_FIXED_JOINTS = 4,
	       NSPATIAL = 6;

typedef Ravelin::MatrixNd Mat;
typedef Ravelin::VectorNd Vec;

std::vector<std::string> eef_names_;
std::vector<RigidBodyPtr> eefs;
static map<string, double> q0;
// simulator 
boost::shared_ptr<EventDrivenSimulator> sim;
// robot pointer
RCArticulatedBodyPtr abrobot;
DynamicBodyPtr dbrobot;

// integration error
map<string, double> perr_sum;

struct Gains
{
  double kp;
  double kv;
  double ki;
};

// map of gains
map<string, Gains> gains;

template <typename T, typename U>
const U& get(const map<T, U>& m, const T& key)
{
  typename map<T, U>::const_iterator i = m.find(key);
  if (i == m.end())
    throw std::runtime_error("Unexpectedly couldn't find key in map!");
  return i->second;
}

    double last_time = 0;
/// Controls the robot
void control_PID(const map<string, double>& q_des, const map<string, double>& qd_des, const map<string, Gains>& gains, double time,Vec& u)
{
  const std::vector<JointPtr>& joints = abrobot->get_joints();

  // reset u
  u.set_zero();

  // clear and set motor torques
  for (unsigned m=0; m< joints.size(); m++)
  {
    // get the joint
    JointPtr j = joints[m];
    if(j->q.rows() == 0) continue;
    unsigned i = j->get_coord_index();

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
    u.set_sub_vec(i,fb_torque);
    i++;
  }
}



void apply_simulation_forces(const Vec& u){
  const std::vector<JointPtr>& joints = abrobot->get_joints();
  for(unsigned m=0;m< joints.size();m++){
    if(joints[m]->q.size() == 0) continue;
    // reset motor torque
    unsigned i = joints[m]->get_coord_index();
    Vec row;
    joints[m]->reset_force();
    joints[m]->add_force(u.get_sub_vec(i,i+joints[m]->num_dof(), row));
    i++;
  }
}

/// The main control loop
void controller(DynamicBodyPtr dbp, double t, void*)
{
  static Vec u(NJOINT), q(NJOINT), qd(NJOINT);
  double dt = t - last_time;

  static unsigned ITER = 1;

  u.set_zero();

  /// setup a steady state
  static map<string, double> q_des, qd_des;
  const std::vector<JointPtr>& joints = abrobot->get_joints();
  if (q_des.empty())
  {
    std::cout << "desired q/qd: ";
    for (unsigned i=0,m=0; m< joints.size(); m++)
    {
      if(joints[m]->q.size() == 0) continue;
      unsigned ind = joints[m]->get_coord_index();
      q_des[joints[m]->id] = q0[joints[m]->id];// joints[m]->q[0];
      qd_des[joints[m]->id] = 0.0;
      std::cout << "(" << q_des[joints[m]->id] << ",0) ";
      i++;
    }
  }
  std::cout << std::endl;

  std::cout << "current q/qd: ";
  ///  Record Robot State
  for(unsigned m=0;m< joints.size();m++){
    if(joints[m]->q.size() == 0) continue;
    unsigned ind = joints[m]->get_coord_index();
    q.set_sub_vec(ind,joints[m]->q);
    qd.set_sub_vec(ind,joints[m]->qd);
    std::cout << "(" << joints[m]->q[0] << "," << joints[m]->qd[0] << ") ";
  }
  std::cout << std::endl;

  ///  Determine FB forces
  control_PID(q_des, qd_des, gains,t,u);

  //apply_simulation_forces(u);

  std::cout << " u = (" << u << ") \n";

  if(dt>0){
    std::cerr << "iteration: " << ITER << std::endl;
    ITER++;
  }

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
  
  const std::vector<RigidBodyPtr>& links = abrobot->get_links();
  const std::vector<JointPtr>& joints = abrobot->get_joints();

  for (unsigned i=0; i< links.size(); i++)
  {
    Pose3d P = *links[i]->get_pose();
    P.update_relative_pose(GLOBAL);
    std::cout << "link " << links[i]->id << " pose: " << P << std::endl;
  }
  for (unsigned i=0; i< joints.size(); i++)
  {
    Point3d p = joints[i]->get_location();
    shared_ptr<RevoluteJoint> j = dynamic_pointer_cast<RevoluteJoint>(joints[i]);
    if (!j)
      continue;
    std::cout << "joint " << joints[i]->id << " location: " << Pose3d::transform_point(GLOBAL, p) << " axis: " << Pose3d::transform_vector(GLOBAL, j->get_axis()) << std::endl;
  }
  joints.front()->qd[0] = 1.0;

//  abrobot->controller = &controller;

  eef_names_.push_back("LF_FOOT");
  eef_names_.push_back("RF_FOOT");
  eef_names_.push_back("LH_FOOT");
  eef_names_.push_back("RH_FOOT");

  q0["LF_HFE"] =  0.6;
  q0["LF_HAA"] = -0.35;
  q0["LF_KFE"] = -1.4;

  q0["RF_HFE"] =  0.6;
  q0["RF_HAA"] = -0.35;
  q0["RF_KFE"] = -1.4;

  q0["LH_HFE"] = -0.6;
  q0["LH_HAA"] = -0.35;
  q0["LH_KFE"] =  1.4;

  q0["RH_HFE"] = -0.6;
  q0["RH_HAA"] = -0.35;
  q0["RH_KFE"] =  1.4;

  // now, setup gains

  for (unsigned i=0,m=0; m< joints.size(); m++)
  {
    if(joints[m]->q.size() == 0) continue;
    double kp,kv,ki;
    switch(i%3){
      case 0:
        kp = 300;
        kv = 6;
        ki = 0;
        break;
      case 1:
        kp = 10;
        kv = 6;
        ki = 0;
        break;
      case 2:
        kp = 300;
        kv = 6;
        ki = 0;
        break;
      default: break;
    }
    // pass gain values to respective joint
    gains[joints[m]->id].kp = kp;
    gains[joints[m]->id].kv = kv;
    gains[joints[m]->id].ki = ki;
    i++;
  }
}

} // end extern C

