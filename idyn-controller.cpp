/*****************************************************************************
 * Simple PD controller 
 ****************************************************************************/

#include <Control.h>

//#define USE_DUMMY_CONTACTS

using namespace Moby;
using namespace Ravelin;

std::vector<std::string> eef_names_, joint_name_;
std::vector<ContactData> contacts;
std::vector<RigidBodyPtr> eefs;
static map<string, double> q_go0;

// simulator 
boost::shared_ptr<EventDrivenSimulator> sim;
// robot pointer
RCArticulatedBodyPtr abrobot;
DynamicBodyPtr dbrobot;
Vec cf_moby;
Mat MU_moby;

///////////////////////// PID /////////////////////////////

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

///////////////////// Contact DATA ////////////////////////

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
//          outlog2(c.point,"point");

          RigidBodyPtr sbfoot = eefs[i];
 
          Vec col(NSPATIAL);
          AAngled aa(0,0,1,0);
          Origin3d o(c.point);
          boost::shared_ptr<const Ravelin::Pose3d> pose(new Pose3d(aa,o));
          SForced sfn(c.normal,torque,pose);
//          outlog2(c.normal,"norm");
          abrobot->convert_to_generalized_force(sbfoot,sfn, c.point, col);
          N.set_column(i,col);
//          outlog2(col,"N_col");
          for(int k=0;k<nk;k++){
              if(k%2 == 0) {
//                  outlog2(tan1,"tanS");
                  SForced sfs(tan1,torque,pose);
                  abrobot->convert_to_generalized_force(sbfoot,sfs, c.point, col);
              } else {
//                  outlog2(tan2,"tanT");
                  SForced sft(tan2,torque,pose);
                  abrobot->convert_to_generalized_force(sbfoot,sft, c.point, col);
              }
              if(k>=2) col.negate();
              D.set_column(i*nk + k,col);
//              outlog2(col,"D_col");
          }
     }
}

void calculate_dyn_properties(Mat& M, Vec& fext){
    M.resize(NDOFS,NDOFS);
    fext.resize(NDOFS);
    abrobot->get_generalized_inertia(M);
    abrobot->get_generalized_forces(fext);
}

///////////////////// Controllers ////////////////////////

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

//            cf_moby[i] = e[i].contact_impulse[2];
//            cf_moby[i+nc] = e[i].contact_impulse[0];
//            cf_moby[i+nc*2] = e[i].contact_impulse[1];
//            MU_moby(i,0) = sqrt(e[i].contact_impulse[0]*e[i].contact_impulse[0] + e[i].contact_impulse[1]*e[i].contact_impulse[1])/e[i].contact_impulse[2];

            contacts.push_back(c);
        }
    }
//    last_time = t;
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

/// The main control loop
void controller(DynamicBodyPtr dbp, double t, void*)
{
    static double test_frict_val = 0.1;
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
            q_des[joints[m]->id] = q_go0[joints[m]->id];
            qd_des[joints[m]->id] = 0.0;
        }


      for (unsigned i=0, m=0; m< joints.size(); m++)
      {
          if(joints[m]->q.size() == 0) continue;
          std::cout << "\t" << q_des[joints[m]->id]<< "\t" << q(joints[m]->get_coord_index(),0) << "\t" << qd_des[joints[m]->id]  << "\t" << qd(joints[m]->get_coord_index(),0) << std::endl;
          i++;
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
        Mat N,D,M(NDOFS,NDOFS);
        Vec fext(NDOFS);

        Mat MU;
        MU.set_zero(nc,1);
        for(int i=0;i<nc;i++)
          MU(i,0) = 0.4;

        determine_N_D(contacts,N,D);

        Mat ST;
        ST.resize(D.rows(),D.columns()/2);
        ST.set_zero();
        // remove negations from D to create ST
        // of the form [S T]

        for(int i=0;i<N.columns();i++){
          for(int j=0;j<N.rows();j++){
                ST(j,i) = D(j,i*nk);
                ST(j,nc+i) = D(j,i*nk+1);
            }
        }
//        outlog2(ST,"ST");

        calculate_dyn_properties(M,fext);

        Vec v(NDOFS);
        dbrobot->get_generalized_velocity(DynamicBody::eSpatial,v);
        // estimated contact forces

        Vec qdd = Vec::zero(NJOINT);
        Vec cf, ff = uff.column(0);
//        idyn(v,qdd,M,N,D,fext,dt,MU,ff);
//        uff.set_column(0,ff);


//        double err = friction_estimation(v,fext,dt,N,D,M,true,MU,cf);
        contacts.clear();

        /// Output to file
        /*
        {
          std::ofstream out;
          // output mu vals
          out.open("mu.out",std::ios::out | std::ios::app);
          out << err << " " << v.norm() << " " << nc << " " << dt;
          for(int i=0;i<nc;i++)
            out << " " << MU(i,0);
          out << std::endl;
          out.close();

          out.open("cf.out",std::ios::out | std::ios::app);
          for(int i=0;i<cf.rows();i++)
            out << ((i==0)? "":" ") << cf[i];
          out << std::endl;
          out.close();
        // output moby vals
          out.open("muM.out",std::ios::out | std::ios::app);
          for(int i=0;i<nc;i++)
            out << ((i==0)? "":" ") << MU_moby(i,0);
          out << std::endl;
          out.close();

          out.open("cfM.out",std::ios::out | std::ios::app);
          for(int i=0;i<cf_moby.rows();i++)
            out << ((i==0)? "":" ") << cf_moby[i];
          out << std::endl;
          out.close();
        }
        */

      ///  Determine FB forces
      control_PID(q_des, qd_des, gains,t,ufb);

      // combine ufb and uff
      u += ufb; 
//      u += uff;

      apply_simulation_forces(u);
      outlog2(u,"u");

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
          kp = 100;
          kv = 0;
          ki = 0;
          break;
      case 1:
          kp = 100;
          kv = 0;
          ki = 0;
          break;
      case 2:
          kp = 10;
          kv = 0;
          ki = 0;
          break;
      default: break;
      }
      // pass gain values to respective joint
      gains[joint_name_[i]].kp = kp;
      gains[joint_name_[i]].kv = kv;
      gains[joint_name_[i]].ki = ki;
  }

      Vec q_start(q_go0.size()+7),qd_start(q_go0.size()+6);
      q_start.set_zero();
      qd_start.set_zero();

      q_start[14] = 1.9;

      for(int i=0;i<q_go0.size();i++){
          int ind = abrobot->find_joint(joint_name_[i])->get_coord_index();
          q_start[ind] = q_go0[joint_name_[i]];
      }

      abrobot->set_generalized_coordinates(DynamicBody::eEuler,q_start);
      abrobot->set_generalized_velocity(DynamicBody::eSpatial,qd_start);

      for(int i=0;i<q_go0.size();i++){
          int ind = abrobot->find_joint(joint_name_[i])->get_coord_index();
          std::cout << ind << " " << joint_name_[i] << " " << abrobot->find_joint(joint_name_[i])->q << std::endl;
//          abrobot->find_joint(joint_name_[i])->determine_q_tare();
//          std::cout << ind << " " << joint_name_[i] << " " << abrobot->find_joint(joint_name_[i])->q << std::endl;
      }

    // RUN OPTIMIZATION TO FIND CFs
    Mat N,D,M;
    Vec fext;
    Vec v(NSPATIAL);
    dbrobot->get_generalized_velocity(DynamicBody::eSpatial,v);
    Mat MU;
    calculate_dyn_properties(M,fext);
}

} // end extern C

