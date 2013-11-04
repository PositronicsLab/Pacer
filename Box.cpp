/*****************************************************************************
 * Simple Box Sliding
 ****************************************************************************/

#include <cmath>
#include <sys/types.h>
#include <sys/times.h>
#include <fstream>
#include <boost/shared_ptr.hpp>
#include <Ravelin/MatrixNd.h>
#include <Ravelin/VectorNd.h>
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

typedef Ravelin::MatrixNd Mat;
typedef Ravelin::VectorNd Vec;

RigidBodyPtr obj;
DynamicBodyPtr body;
Vec cf_moby;
Mat MU_moby;

void outlog(const Mat& M, std::string name);
void outlog(const Vec& z, std::string name);
void outlog2(const Vec& M, std::string name);
void outlog2(const Mat& z, std::string name);

struct ContactData
{
  Vector3d point;  // contact point
  Vector3d normal; // contact normal (pointing away from the ground)
  std::string name;
};

double friction_estimation(const Vec& v, const Vec& fext, double dt,
                         const Mat& N,const Mat& ST, const Mat& M, bool post_event, Mat& MU, Vec& cf);

const unsigned NSPATIAL = 6;
std::vector<ContactData> contacts;

// simulator
boost::shared_ptr<EventDrivenSimulator> sim;


void determine_N_D( RigidBodyPtr obj,std::vector<ContactData>& contacts, Mat& N, Mat& D)
{
      int nc = contacts.size();

      // resize temporary N and ST
      int nk = 4;

      N.resize(NSPATIAL,nc);
      D.resize(NSPATIAL,nc*nk);

      // loop over all contacts
      for(int i = 0; i < nc; i++){
          // get the contact point and normal
          ContactData& c = contacts[i];

          // generate the contact tangents here...
          Vector3d tan1, tan2;
          Vector3d::determine_orthonormal_basis(c.normal, tan1, tan2);

          Vector3d torque;
          torque.set_zero();
          outlog2(c.point,"point");

          Vec col(NSPATIAL);
          AAngled aa(0,0,1,0);
          Origin3d o(c.point);
          boost::shared_ptr<const Ravelin::Pose3d> pose(new Pose3d(aa,o));
          SForced sfn(c.normal,torque,pose);
          outlog2(c.normal,"norm");
          obj->convert_to_generalized_force(obj,sfn, c.point, col);
          N.set_column(i,col);
          outlog2(col,"N_col");
          for(int k=0;k<nk;k++){
              if(k%2 == 0) {
                  outlog2(tan1,"tanS");
          	  SForced sfs(tan1,torque,pose);
          	  obj->convert_to_generalized_force(obj,sfs, c.point, col);
              } else {
                  outlog2(tan2,"tanT");
          	  SForced sft(tan2,torque,pose);
          	  obj->convert_to_generalized_force(obj,sft, c.point, col);
              }
              if(k>=2) col.negate();
              D.set_column(i*nk + k,col);
              outlog2(col,"D_col");
          }
      }
}

void calculate_dyn_properties(RigidBodyPtr obj, Mat& M, Vec& fext){
    M.resize(NSPATIAL,NSPATIAL);
    fext.resize(NSPATIAL);
    obj->get_generalized_inertia(M);
    obj->get_generalized_forces(fext);
}

double last_time = 0;

/// Event callback function for processing events
void post_event_callback_fn(const vector<Event>& e, boost::shared_ptr<void> empty)
{
    double t = sim->current_time;
    double dt = t - last_time;

    Vec q;
    body->get_generalized_coordinates(DynamicBody::eEuler,q);
    outlog2(q,"G Coords");

    /// PROCESS CONTACTS
    contacts.clear();
    int nc = e.size();
    cf_moby.set_zero(nc*3);
    MU_moby.set_zero(nc,1);
    for(unsigned i=0;i<nc;i++){
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

            c.point = e[i].contact_point;
            outlog2(c.point,"C Coords");

            c.name = sb1->id;

            cf_moby[i] = e[i].contact_impulse[2];
            cf_moby[i+nc] = e[i].contact_impulse[0];
            cf_moby[i+nc*2] = e[i].contact_impulse[1];
            MU_moby(i,0) = sqrt(e[i].contact_impulse[0]*e[i].contact_impulse[0] + e[i].contact_impulse[1]*e[i].contact_impulse[1])/e[i].contact_impulse[2];
            //std::cout << "@"<< t << " : " << last_time << "+" << (e[i].t_true-last_time) << " -- " << c.name << " : " << c.normal << " : " << c.point << " : " << e[i].contact_impulse <<  std::endl;
//            std::cerr << "@"<< t << " : " << last_time << "+" << (e[i].t_true-last_time) << " -- " << c.name << " : " << c.normal << " : " << c.point << " : " << e[i].contact_impulse <<  std::endl;
            contacts.push_back(c);
        }
    }

    //last_time = t;
}


/// Event callback function for setting friction vars pre-event
void pre_event_callback_fn(vector<Event>& e, boost::shared_ptr<void> empty){}

/// The main control loop
void controller(DynamicBodyPtr body, double time, void*)
{
  /*
    double t = sim->current_time;
    int nc = contacts.size();
    double dt = t - last_time;
    last_time = t;

    static double test_frict_val = 0.2;
    static unsigned ITER = 1;

    Vec uff;

    /// Apply control torques
    // setup impulse
    uff.set_zero(NSPATIAL);
    uff[1] = test_frict_val;
    uff[5] = test_frict_val;
    body->add_generalized_force(uff);
    
    Vec v(NSPATIAL);
    body->get_generalized_velocity(DynamicBody::eSpatial,v);
    std::cerr << "vel = " << v << std::endl;

    if(v.norm() < 0.01)
        test_frict_val *= 1.001;
    else
        test_frict_val /= 1.001;


    std::cerr << "ITER: " << ITER << std::endl;
    std::cerr << "dt = " << dt << std::endl;
    std::cerr << "t = " << t << std::endl;
    std::cerr << "nc = " << nc << std::endl;
    // RUN OPTIMIZATION TO FIND CFs
    Mat N,D,M;
    Vec fext;

    Mat MU;
    MU.set_zero(nc,1);

    calculate_dyn_properties(obj,M,fext);
    std::cerr << "fext = " << fext << std::endl;

    // estimated contact forces
    Vec cf;
    double err;
    ITER++;
    if (nc == 0){
    	err = friction_estimation(v,fext,dt,N,D,M,false,MU,cf);
	return;
    } else {
        determine_N_D(obj,contacts,N,D);
    	err = friction_estimation(v,fext,dt,N,D,M,true,MU,cf);
    }
    std::cout << "err = " << err << std::endl;
    std::cout << "cf = " << cf << std::endl;
    
    std::cout << "mu = " << MU << std::endl;
    std::cout << "moby_cf = " << cf_moby << std::endl;
    
    std::cout << "moby_MU = " << MU_moby << std::endl;
    
    /// Output to file
    std::ofstream out;
    { // output vals
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
    }
    

    { // output moby vals
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

    contacts.clear(); 
    */

  Vec q(NSPATIAL+1), qd(NSPATIAL);
  body->get_generalized_coordinates(DynamicBody::eEuler,q);
  std::cerr << "q = " << q << std::endl;

  body->get_generalized_velocity(DynamicBody::eSpatial,qd);
  std::cerr << "qd = " << qd << std::endl;
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
    }

    // find the obj
    if (read_map.find("obj") == read_map.end())
      throw std::runtime_error("controller.cpp:init() - unable to find obj!");
    body = dynamic_pointer_cast<DynamicBody>(read_map.find("obj")->second);
    if (!body)
      throw std::runtime_error("controller.cpp:init() - unable to cast obj to type DynamicBody");

    Vec q_start(NSPATIAL+1),qd_start(NSPATIAL);
    q_start.set_zero();
    qd_start.set_zero();

    //    q_start[2] = 0.526;
    //    q_start[0] = 0;
    //    q_start[1] = 0;

//    qd_start[0] = 1;

//    body->set_generalized_coordinates(DynamicBody::eEuler,q_start);
//    body->set_generalized_velocity(DynamicBody::eSpatial,qd_start);

    // setup the controller
    body->controller = controller;

    obj = dynamic_pointer_cast<RigidBody>(body);

//    sim->event_post_impulse_callback_fn = post_event_callback_fn;

 //   sim->event_callback_fn = pre_event_callback_fn;

    // RUN OPTIMIZATION TO FIND CFs
//    Mat N,D,M;
//    Vec fext;
//    Vec v(NSPATIAL);
//    body->get_generalized_velocity(DynamicBody::eSpatial,v);
//    Mat MU;
//    calculate_dyn_properties(obj,M,fext);

//    // estimated contact forces
//    Vec cf;
//    friction_estimation(v,fext,0,N,D,M,false,MU,cf);
}
} // end extern C
