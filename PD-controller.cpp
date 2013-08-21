/*****************************************************************************
 * Simple PD controller 
 ****************************************************************************/

#include <cmath>
#include <sys/types.h>
#include <sys/times.h>
#include <fstream>
#include <Moby/Simulator.h>
#include <Moby/EventDrivenSimulator.h>
#include <Moby/RCArticulatedBody.h>
#include <Moby/DeformableBody.h>
#include <Moby/DynamicBody.h>
#include <Moby/RevoluteJoint.h>
#include <Moby/GravityForce.h>
#include <Moby/LinAlg.h>
#include <Moby/Constants.h>
#include <Moby/RNEAlgorithm.h>
#include <osgDB/WriteFile>
//#include <osgDB/ReaderWriterSTL>

using namespace Moby;
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

typedef Moby::MatrixN Mat;
typedef Moby::VectorN Vec;

void outlog(const Mat& M, std::string name);
void outlog(const Vec& z, std::string name);
void outlog2(const Vec& M, std::string name);
void outlog2(const Mat& z, std::string name);
double friction_estimation(const Vec& v, const Vec& f, double dt,
                         const Mat& N,const Mat& D, const Mat& M, bool post_event, Mat& MU, Vec& cf);

struct ContactData
{
  Vector3 point;  // contact point
  Vector3 normal; // contact normal (pointing away from the ground)
  std::string name;
};


std::vector<std::string> eef_names_, joint_name_;
std::vector<ContactData> contacts;
std::vector<RigidBodyPtr> eefs;

// robot pointer
static map<string, Real> q_go0;


// simulator 
boost::shared_ptr<EventDrivenSimulator> sim;
RCArticulatedBodyPtr abrobot;
DynamicBodyPtr dbrobot;

// integration error
Vec cf_moby;
map<string, Real> perr_sum;

struct Gains
{
  Real kp;
  Real kv;
  Real ki;
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

void determine_N_D(std::vector<ContactData>& contacts, MatrixN& N, MatrixN& D)
{
    int nc = contacts.size();
    const std::vector<RigidBodyPtr>& links = abrobot->get_links();
    std::vector<RigidBodyPtr> eefs(nc);

    for(unsigned i=0;i<links.size();i++)
        for(unsigned j=0;j<nc;j++)
            if(contacts[j].name.compare(links[i]->id) == 0)
                eefs[j] = links[i];

    int nk = 4;
    static MatrixN J, Jsub;
    static VectorN col;

      // resize temporary N and ST
      N.resize(NJOINT+6,nc);
      D.resize(NJOINT+6,nc*nk);

      J.resize(NSPATIAL, NJOINT);

      // loop over all contacts
      for(int i = 0; i < nc; i++){
          // get the contact point and normal
          ContactData& c = contacts[i];

          // generate the contact tangents here...
          Vector3 tan1, tan2;
          Vector3::determine_orthonormal_basis(c.normal, tan1, tan2);
          // get the base Jacobian components
          abrobot->calc_jacobian(c.point, eefs[i], J);

          // get the translational components of the Jacobian
          J.get_sub_mat(0,3,0,J.columns(), Jsub);
          Jsub.transpose_mult(c.normal, col);
          N.set_column(i, col);

          // below assumes nk=4
          assert(nk%2 == 0 && nk == 4);
          for(unsigned k=0;k<nk;k++){
              Jsub.transpose_mult((k%2==0)?tan1:tan2, col);
              if(k>=nk/2) col.negate();
              D.set_column(i*nk+k, col);
          }          
      }
}

void calculate_dyn_properties(MatrixN& M, VectorN& fext){
    M.resize(NDOFS,NDOFS);
    fext.resize(NDOFS);
    abrobot->get_generalized_inertia(DynamicBody::eAxisAngle, M);
    abrobot->get_generalized_forces(DynamicBody::eAxisAngle, fext);
}

double last_time = 0;
/// Event callback function for processing events
void post_event_callback_fn(const vector<Event>& e, boost::shared_ptr<void> empty)
{
    double t = sim->current_time;
    double dt = t - last_time;

    // PROCESS CONTACTS
    contacts.clear();
    int nc = e.size();
    cf_moby.set_zero(nc*3);

    for(unsigned i=0;i<e.size();i++){
        if (e[i].event_type == Event::eContact)
        {
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

            c.point = e[i].contact_point;
            c.name = sb1->id;

            contacts.push_back(c);
        }
    }

      Mat N,D,M(NDOFS,NDOFS);
      Vec fext(NDOFS);

      MatrixN MU;
      MU.set_zero(nc,1);

      determine_N_D(contacts,N,D);

      calculate_dyn_properties(M,fext);

      Vec v(NDOFS);
      dbrobot->get_generalized_velocity(DynamicBody::eAxisAngle,v);

    // estimated contact forces
    Vec cf;
    double err = friction_estimation(v,fext,dt,N,D,M,true,MU,cf);
    contacts.clear();
    /// Output to file
    std::ofstream out;

    { // output mu vals
        out.open("mu.out",std::ios::out | std::ios::app);
        out << err << " " << v.norm() << " " << nc << " " << dt;
        for(int i=0;i<nc;i++)
          out << " " << MU(i,0);
        out << std::endl;
        out.close();
    }

    { // output cf vals
        out.open("cf.out",std::ios::out | std::ios::app);
        for(int i=0;i<cf.rows();i++)
          out << ((i==0)? "":" ") << cf[i];
        out << std::endl;
        out.close();
    }


    last_time = t;
}


/// Event callback function for setting friction vars pre-event
void pre_event_callback_fn(vector<Event>& e, boost::shared_ptr<void> empty)
{
    double t = sim->current_time;
    double dt = t - last_time;


    Mat N,D,M;
    Vec fext, cf;
    Vec v(NSPATIAL);
    dbrobot->get_generalized_velocity(DynamicBody::eAxisAngle,v);

    MatrixN MU;

//    determine_N_D(contacts,N,D);

    calculate_dyn_properties(M,fext);

    friction_estimation(v,fext,dt,N,D,M,false,MU,cf);

    //last_time = t;
}

/// Controls the robot
void control_PID(const map<string, Real>& q_des, const map<string, Real>& qd_des, const map<string, Gains>& gains, Real time,Mat& ufb)
{
      const std::vector<JointPtr>& joints = abrobot->get_joints();

      // clear and set motor torques
      for (unsigned m=0,i=0; m< joints.size(); m++)
      {
            // get the joint
            JointPtr j = joints[m];
            if(j->q.rows() == 0) continue;

            // get the two gains
            const Real KP = get(gains,j->id).kp;
            const Real KV = get(gains,j->id).kv;
            const Real KI = get(gains,j->id).ki;

            // add feedback torque to joints
            Real perr = get(q_des,j->id) - j->q[0];
            perr_sum[j->id] += perr;
            Real ierr = perr_sum[j->id];
            Real derr = get(qd_des,j->id) - j->qd[0];
            VectorN fb_torque(1);
            fb_torque[0] = perr*KP + derr*KV + ierr*KI;
            ufb.set_row(i,fb_torque);
            i++;
      }
}



void apply_simulation_forces(const Mat& u){
    const std::vector<JointPtr>& joints = abrobot->get_joints();
    for(unsigned m=0,i=0;m< joints.size();m++){
        if(joints[m]->q.size() == 0) continue;
        // reset motor torque
        joints[m]->reset_force();
        joints[m]->add_force(u.get_row(i));
        i++;
    }
}

/// The main control loop
void controller(DynamicBodyPtr dbp, Real t, void*)
{
     static Mat uff(NJOINT,1),ufb(NJOINT,1),u(NJOINT,1), q(NJOINT,1), qd(NJOINT,1);
    double dt = t - last_time;
     
     unsigned nc = contacts.size();

     static double test_frict_val = 0.5;
     static unsigned ITER = 1;

     uff.set_zero();
     ufb.set_zero();
     u.set_zero();

      /// setup a steady state
      static map<string, Real> q_des, qd_des;
      const std::vector<JointPtr>& joints = abrobot->get_joints();
      if (q_des.empty())
        for (unsigned i=0,m=0; m< joints.size(); m++)
        {
            if(joints[m]->q.size() == 0) continue;
            q_des[joints[m]->id] = q_go0[joints[m]->id];
            qd_des[joints[m]->id] = 0.0;
            i++;
        }

      ///  Record Robot State
      for(unsigned m=0;m< joints.size();m++){
          if(joints[m]->q.size() == 0) continue;
          unsigned ind = joints[m]->get_coord_index() - NSPATIAL;
          q.set_row(ind,joints[m]->q);
          qd.set_row(ind,joints[m]->qd);
      }

      ///  Determine FB forces
      control_PID(q_des, qd_des, gains,t,ufb);

      // combine ufb and uff
      u += ufb; 
      u += uff;

      apply_simulation_forces(u);

      Vec fext;
      fext.set_zero(NDOFS);
      fext[5] = test_frict_val;
      abrobot->add_generalized_force(DynamicBody::eAxisAngle, fext);
      if(dt>0){
        std::cerr << "iteration: " << ITER << std::endl;
      	test_frict_val *= 1.001;
      	ITER++;
      }


    last_time = t;
}


/// plugin must be "extern C"
extern "C" {

void init(void* separator, const std::map<std::string, BasePtr>& read_map, Real time)
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
  if (sim && abrobot){
      sim->event_post_impulse_callback_fn = &post_event_callback_fn;
      // setup the controller
    abrobot->controller = &controller;
  } else
    assert(false);
  // Create joint name vector
  joint_name_.push_back("LF_HFE");
  joint_name_.push_back("LF_HAA");
  joint_name_.push_back("LF_KFE");
  joint_name_.push_back("LH_HFE");
  joint_name_.push_back("LH_HAA");
  joint_name_.push_back("LH_KFE");
  joint_name_.push_back("RF_HFE");
  joint_name_.push_back("RF_HAA");
  joint_name_.push_back("RF_KFE");
  joint_name_.push_back("RH_HFE");
  joint_name_.push_back("RH_HAA");
  joint_name_.push_back("RH_KFE");
  
  // robot's go0 configuration
  q_go0["LF_HFE"] =  0.6;
  q_go0["LF_HAA"] = -0.35;
  q_go0["LF_KFE"] = -1.4;

  q_go0["RF_HFE"] =  0.6;
  q_go0["RF_HAA"] = -0.35;
  q_go0["RF_KFE"] = -1.4;

  q_go0["LH_HFE"] = -0.6;
  q_go0["LH_HAA"] = -0.35;
  q_go0["LH_KFE"] =  1.4;

  q_go0["RH_HFE"] = -0.6;
  q_go0["RH_HAA"] = -0.35;
  q_go0["RH_KFE"] =  1.4;

  eef_names_.push_back("LF_FOOT");
  eef_names_.push_back("RF_FOOT");
  eef_names_.push_back("LH_FOOT");
  eef_names_.push_back("RH_FOOT");

  // now, setup gains
  for(unsigned i=0;i<joint_name_.size();i++){
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
      gains[joint_name_[i]].kp = kp;
      gains[joint_name_[i]].kv = kv;
      gains[joint_name_[i]].ki = ki;
  }

      VectorN q_start(q_go0.size()+7),qd_start(q_go0.size()+6);
      q_start.set_zero();
      qd_start.set_zero();

      for(int i=0;i<q_go0.size();i++){
          int ind = abrobot->find_joint(joint_name_[i])->get_coord_index();
          q_start[ind+1] = q_go0[joint_name_[i]];
          std::cout << ind << " " << joint_name_[i] << std::endl;
      }

      q_start[2] = 0.11;
      for(int i=0;i<q_start.rows();i++)
          std::cerr << q_start[i] << std::endl;

      abrobot->set_generalized_coordinates(DynamicBody::eRodrigues,q_start);
      abrobot->set_generalized_velocity(DynamicBody::eAxisAngle,qd_start);

    // RUN OPTIMIZATION TO FIND CFs
    Mat N,D,M;
    Vec fext;
    Vec v(NSPATIAL);
    dbrobot->get_generalized_velocity(DynamicBody::eAxisAngle,v);
    MatrixN MU;
    calculate_dyn_properties(M,fext);

    // estimated contact forces
    Vec cf;
    friction_estimation(v,fext,0,N,D,M,false,MU,cf);
}

} // end extern C

