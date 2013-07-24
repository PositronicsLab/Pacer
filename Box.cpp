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
                         const Mat& N,const Mat& ST, const Mat& M, Mat& MU, Vec& cf);

const unsigned NSPATIAL = 6;
std::vector<ContactData> contacts;

// simulator
boost::shared_ptr<EventDrivenSimulator> sim;

Vec cf_moby;

/// Event callback function for processing events
void event_callback_fn(const vector<Event>& e, boost::shared_ptr<void> empty)
{
    contacts.clear();
    // limit to minimum number of contacts (NX)
    int nc = e.size();
    cf_moby.set_zero(nc*3);
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

            c.point = e[i].contact_point - sb1->get_position();

            c.name = sb1->id;

            cf_moby[i] = e[i].contact_impulse[2];
            cf_moby[i+nc] = e[i].contact_impulse[0];
            cf_moby[i+nc*2] = e[i].contact_impulse[1];
            std::cout << c.name << " : " << c.normal << " : " << c.point << " : " << e[i].contact_impulse <<  std::endl;
            if(e[i].contact_impulse[2] != 0)
                std::cout << "MU_APPLIED : " << (sqrt(e[i].contact_impulse[0] * e[i].contact_impulse[0] + e[i].contact_impulse[1] * e[i].contact_impulse[1])/e[i].contact_impulse[2]) <<  std::endl;

            contacts.push_back(c);
        }
    }
}

void determine_N_D( RigidBodyPtr box,std::vector<ContactData>& contacts, MatrixN& N, MatrixN& D)
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

          // M = p x F
          Vector3 Na = Vector3::cross(c.point,c.normal);
          Vector3 Sa = Vector3::cross(c.point,tan1);
          Vector3 Ta = Vector3::cross(c.point,tan2);

          Vector3 torque;
          torque.set_zero();

          VectorN col(NSPATIAL);

          box->convert_to_generalized_force(DynamicBody::eAxisAngle,box, c.point, c.normal, torque, col);
          N.set_column(i,col);
          for(int k=0;k<nk;k++){
              if(k%2 == 0)
                  box->convert_to_generalized_force(DynamicBody::eAxisAngle,box, c.point, tan1, torque, col);
              else
                  box->convert_to_generalized_force(DynamicBody::eAxisAngle,box, c.point, tan2, torque, col);
              if(k>=2) col.negate();
              D.set_column(i*nk + k,col);
          }
      }
}

void calculate_dyn_properties(RigidBodyPtr box, MatrixN& M, VectorN& fext){
    M.resize(NSPATIAL,NSPATIAL);
    fext.resize(NSPATIAL);
    box->get_generalized_inertia(DynamicBody::eAxisAngle, M);
    box->get_generalized_forces(DynamicBody::eAxisAngle, fext);
}

/// The main control loop
void controller(DynamicBodyPtr body, double t, void*)
{
    RigidBodyPtr box = dynamic_pointer_cast<RigidBody>(body);

    unsigned NC = contacts.size();
    static double last_time = t;
    static double test_frict_val = 0.5;
    static unsigned ITER = 1;
    // setup dt
    double dt = t - last_time;
    last_time = t;

    std::cout << "iteration: " << ITER << std::endl;
    std::cout << "simulation time : " << t << std::endl;
    std::cout << "dt : " << dt << std::endl;

    std::cerr << "iteration: " << ITER << std::endl;
    std::cerr << "simulation time : " << t << std::endl;
    std::cerr << "dt : " << dt << std::endl;

    Mat N,D,M;
    Vec fext, uff;
    Vec v(NSPATIAL), acc(NSPATIAL);
    body->get_generalized_velocity(DynamicBody::eAxisAngle,v);
//    body->get_generalized_acceleration(DynamicBody::eAxisAngle,acc);
    outlog2(v,"vel");

    /// Apply control torques
    // setup impulse
    uff.set_zero(NSPATIAL);
//    uff[1] = test_frict_val;
//    uff[5] = -test_frict_val;
//    body->add_generalized_force(DynamicBody::eAxisAngle, uff);
//    if(dt>0)
//        test_frict_val *= 1.001;

    MatrixN MU;
    MU.set_zero(NC,1);

    determine_N_D(box,contacts,N,D);

    calculate_dyn_properties(box,M,fext);

    // estimated contact forces
    Vec cf;
    double err = -1;
    if(NC > 0 && dt>0){
        outlog(N,"N");
        outlog(D,"D");
        outlog2(M,"M");
        outlog2(v,"vel");
        outlog2(fext,"f_external");
        outlog2(uff,"f_applied");

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
              // Post-Contact Event callback
              sim->event_post_impulse_callback_fn = event_callback_fn;
        }
    }

    // find the box
    if (read_map.find("box") == read_map.end())
      throw std::runtime_error("controller.cpp:init() - unable to find box!");
    DynamicBodyPtr box = dynamic_pointer_cast<DynamicBody>(read_map.find("box")->second);
    if (!box)
      throw std::runtime_error("controller.cpp:init() - unable to cast box to type DynamicBody");

    // setup the controller
    box->controller = controller;

    VectorN q_start(NSPATIAL+1),qd_start(NSPATIAL);
    q_start.set_zero();
    qd_start.set_zero();

    q_start[2] = 0.53;

    qd_start[0] = 3;

    box->set_generalized_coordinates(DynamicBody::eRodrigues,q_start);
    box->set_generalized_velocity(DynamicBody::eAxisAngle,qd_start);
}
} // end extern C
