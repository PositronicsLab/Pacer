/*****************************************************************************
 * Simple Box Sliding
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

typedef Moby::MatrixN Mat;
typedef Moby::VectorN Vec;

RigidBodyPtr obj;
DynamicBodyPtr body;
Vec cf_moby;
MatrixN MU_moby;

void outlog(const Mat& M, std::string name);
void outlog(const Vec& z, std::string name);
void outlog2(const Vec& M, std::string name);
void outlog2(const Mat& z, std::string name);

struct ContactData
{
  bool on;
  Vector3 point;  // contact point
  Vector3 normal; // contact normal (pointing away from the ground)
  std::string name;
};

double friction_estimation(const Vec& v, const Vec& fext, double dt,
                         const Mat& N,const Mat& ST, const Mat& M, bool post_event, Mat& MU, Vec& cf);

const unsigned NSPATIAL = 6;
std::vector<ContactData> contacts;

// simulator
boost::shared_ptr<EventDrivenSimulator> sim;


void determine_N_D( RigidBodyPtr obj,std::vector<ContactData>& contacts, MatrixN& N, MatrixN& D)
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
          Vector3 tan1, tan2;
          Vector3::determine_orthonormal_basis(c.normal, tan1, tan2);

          Vector3 torque;
          torque.set_zero();

          VectorN col(NSPATIAL);

          obj->convert_to_generalized_force(DynamicBody::eAxisAngle,obj, c.point, c.normal, torque, col);
          N.set_column(i,col);
          for(int k=0;k<nk;k++){
              if(k%2 == 0)
                  obj->convert_to_generalized_force(DynamicBody::eAxisAngle,obj, c.point, tan1, torque, col);
              else
                  obj->convert_to_generalized_force(DynamicBody::eAxisAngle,obj, c.point, tan2, torque, col);
              if(k>=2) col.negate();
              D.set_column(i*nk + k,col);
          }
      }
}

void calculate_dyn_properties(RigidBodyPtr obj, MatrixN& M, VectorN& fext){
    M.resize(NSPATIAL,NSPATIAL);
    fext.resize(NSPATIAL);
    obj->get_generalized_inertia(DynamicBody::eAxisAngle, M);
    obj->get_generalized_forces(DynamicBody::eAxisAngle, fext);
}

double last_time = 0;

/// Event callback function for processing events
void post_event_callback_fn(const vector<Event>& e, boost::shared_ptr<void> empty)
{
    double t = sim->current_time;
    double dt = t - last_time;

    /// PROCESS CONTACTS
    contacts.clear();
    int nc = e.size();
    cf_moby.set_zero(nc*3);
    MU_moby.set_zero(nc,1);
    for(unsigned i=0;i<nc;i++){
        if (e[i].event_type == Event::eContact)
        {
            ContactData c;
            c.on = true;
            SingleBodyPtr sb1 = e[i].contact_geom1->get_single_body();
            SingleBodyPtr sb2 = e[i].contact_geom2->get_single_body();

            if (sb1->id.compare("ground") == 0){
                c.normal = -e[i].contact_normal;
                std::swap(sb1,sb2);
            } else
                c.normal = e[i].contact_normal;

            c.point = e[i].contact_point;

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
    double t = sim->current_time;
    int nc = contacts.size();
    double dt = t - last_time;
    last_time = t;

    static double test_frict_val = 0.1;
    static unsigned ITER = 1;

    Vec uff;

    /// Apply control torques
    // setup impulse
    uff.set_zero(NSPATIAL);
    //uff[1] = test_frict_val;
    uff[5] = test_frict_val;
    body->add_generalized_force(DynamicBody::eAxisAngle, uff);
    if(dt>0){
        test_frict_val *= 1.01;
    }

    std::cerr << "ITER: " << ITER << std::endl;
    std::cerr << "dt = " << dt << std::endl;
    std::cerr << "t = " << t << std::endl;
    std::cerr << "nc = " << nc << std::endl;
    // RUN OPTIMIZATION TO FIND CFs
    Mat N,D,M;
    Vec fext;
    Vec v(NSPATIAL);
    body->get_generalized_velocity(DynamicBody::eAxisAngle,v);
    std::cerr << "vel = " << v << std::endl;

    MatrixN MU;
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
          out << " " << MU_moby(i,0);
        out << std::endl;
        out.close();

        out.open("cfM.out",std::ios::out | std::ios::app);
        for(int i=0;i<cf_moby.rows();i++)
          out << ((i==0)? "":" ") << cf_moby[i];
        out << std::endl;
        out.close();
    }

    contacts.clear(); 
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
    }

    // find the obj
    if (read_map.find("obj") == read_map.end())
      throw std::runtime_error("controller.cpp:init() - unable to find obj!");
    body = dynamic_pointer_cast<DynamicBody>(read_map.find("obj")->second);
    if (!body)
      throw std::runtime_error("controller.cpp:init() - unable to cast obj to type DynamicBody");

    VectorN q_start(NSPATIAL+1),qd_start(NSPATIAL);
    q_start.set_zero();
    qd_start.set_zero();

    q_start[2] = 0.526;
    q_start[0] = 0;
    q_start[1] = 0;

//    qd_start[0] = 1;

    body->set_generalized_coordinates(DynamicBody::eRodrigues,q_start);
    body->set_generalized_velocity(DynamicBody::eAxisAngle,qd_start);

    // setup the controller
    body->controller = controller;

    obj = dynamic_pointer_cast<RigidBody>(body);

    sim->event_post_impulse_callback_fn = post_event_callback_fn;

 //   sim->event_callback_fn = pre_event_callback_fn;

    // RUN OPTIMIZATION TO FIND CFs
    Mat N,D,M;
    Vec fext;
    Vec v(NSPATIAL);
    body->get_generalized_velocity(DynamicBody::eAxisAngle,v);
    MatrixN MU;
    calculate_dyn_properties(obj,M,fext);

    // estimated contact forces
    Vec cf;
    friction_estimation(v,fext,0,N,D,M,false,MU,cf);
}
} // end extern C
