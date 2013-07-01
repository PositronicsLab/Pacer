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
RCArticulatedBodyPtr robot;


static map<string, Real> q_go0;


// simulator 
boost::shared_ptr<EventDrivenSimulator> sim;

// integration error
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

/// Event callback function for processing events
void event_callback_fn(const vector<Event>& e, boost::shared_ptr<void> empty)
{
    contacts.clear();
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

            if(c.normal[2] < 0)
                c.normal *= -1.0;

            std::cout << c.name << " : " << c.normal << " : " << c.point << " : " << e[i].contact_impulse <<  std::endl;
            if(e[i].contact_impulse[2] != 0)
                std::cout << "MU_APPLIED : " << (sqrt(e[i].contact_impulse[0] * e[i].contact_impulse[0] + e[i].contact_impulse[1] * e[i].contact_impulse[1])/e[i].contact_impulse[2]) <<  std::endl;

            contacts.push_back(c);
        }
    }
}

/// Controls the robot
void control_PID(RCArticulatedBodyPtr robot, const map<string, Real>& q_des, const map<string, Real>& qd_des, const map<string, Gains>& gains, Real time,Mat& ufb)
{
      const std::vector<JointPtr>& joints = robot->get_joints();

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

void determine_N_D(RCArticulatedBodyPtr robot,std::vector<ContactData>& contacts, MatrixN& N, MatrixN& D)
{
    int nc = contacts.size();
    const std::vector<RigidBodyPtr>& links = robot->get_links();
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
          robot->calc_jacobian(c.point, eefs[i], J);

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

void apply_simulation_forces(RCArticulatedBodyPtr rcab, const Mat& u){
    const std::vector<JointPtr>& joints = rcab->get_joints();
    for(unsigned m=0,i=0;m< joints.size();m++){
        if(joints[m]->q.size() == 0) continue;
        // reset motor torque
        joints[m]->reset_force();
        joints[m]->add_force(u.get_row(i));
        i++;
    }
}


double friction_estimation(const Vec& v, const Vec& fext, double dt,
                         const Mat& N,const Mat& ST, const Mat& M, Mat& MU, Vec& cf);

/// The main control loop
void controller(DynamicBodyPtr robot, Real t, void*)
{
     static Mat uff(NJOINT,1),ufb(NJOINT,1),u(NJOINT,1), q(NJOINT,1), qd(NJOINT,1);
     unsigned NC = contacts.size();
     static double last_time = t;
     static double test_frict_val = 0.5;
     static unsigned ITER = 1;
     // setup dt
     double dt = t - last_time;
     last_time = t;

     uff.set_zero();
     ufb.set_zero();
     u.set_zero();

     std::cout << "iteration: " << ITER << std::endl;
     std::cout << "simulation time : " << t << std::endl;
     std::cout << "dt : " << dt << std::endl;

     std::cerr << "iteration: " << ITER << std::endl;
     std::cerr << "simulation time : " << t << std::endl;
     std::cerr << "dt : " << dt << std::endl;

      RCArticulatedBodyPtr rcab = dynamic_pointer_cast<RCArticulatedBody>(robot);


      /// setup a steady state
      static map<string, Real> q_des, qd_des;
      const std::vector<JointPtr>& joints = rcab->get_joints();
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
      control_PID(rcab, q_des, qd_des, gains,t,ufb);

      // combine ufb and uff
      u = ufb + uff;

      apply_simulation_forces(rcab,u);



      Mat N,D,M(NDOFS,NDOFS);
      Vec fext(NDOFS);
      
      fext.set_zero();
      fext[5] = test_frict_val;
      rcab->add_generalized_force(DynamicBody::eAxisAngle, fext);
      test_frict_val *= 1.001;

      determine_N_D(rcab,contacts,N,D);

      robot->get_generalized_inertia(DynamicBody::eAxisAngle, M);
      fext.set_zero();
      robot->get_generalized_forces(DynamicBody::eAxisAngle, fext);

      Vec v(NDOFS);
      robot->get_generalized_velocity(DynamicBody::eAxisAngle,v);

      // estimated contact forces & friction
      MatrixN MU(NC,1);
      Vec cf(NC*3);
      double err = -1;
      if(NC > 0 && dt>0){
          outlog(N,"N");
          outlog(D,"D");
          outlog2(M,"M");
          outlog2(v,"vel");
          outlog2(fext,"f_external");

          err = friction_estimation(v,fext,dt,N,D,M,MU,cf);
          std::cerr << "Friction: " << MU.norm_inf() << std::endl;
          contacts.clear();
      }

      /// Output to file
      std::ofstream out;

      { // output mu vals
          out.open("mu.out",std::ios::out | std::ios::app);
          out << err << " " << test_frict_val << " " << v.norm() << " " << NC;
          for(int i=0;i<NC;i++)
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
      ITER++;
}


/// plugin must be "extern C"
extern "C" {

void init(void* separator, const std::map<std::string, BasePtr>& read_map, Real time)
{
  // get a reference to the EventDrivenSimulator instance
  for (std::map<std::string, BasePtr>::const_iterator i = read_map.begin(); i !=read_map.end(); i++) 
  {
    if (!sim)
    {
      sim = dynamic_pointer_cast<EventDrivenSimulator>(i->second);
      if (sim)
          sim->event_post_impulse_callback_fn = &event_callback_fn;
    }

    // find the robot
    if (!robot)
    {
      robot = dynamic_pointer_cast<RCArticulatedBody>(i->second);

      // setup the controller
      if (robot)
        robot->controller = &controller;
    }
  }


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
          int ind = robot->find_joint(joint_name_[i])->get_coord_index();
          q_start[ind+1] = q_go0[joint_name_[i]];
      }

      q_start[2] = 0.11;
      for(int i=0;i<q_start.rows();i++)
          std::cerr << q_start[i] << std::endl;

      robot->set_generalized_coordinates(DynamicBody::eRodrigues,q_start);
      robot->set_generalized_velocity(DynamicBody::eAxisAngle,qd_start);

}

} // end extern C

