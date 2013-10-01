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
#include <Moby/DeformableBody.h>
#include <Moby/DynamicBody.h>
#include <Moby/RevoluteJoint.h>
#include <Moby/GravityForce.h>
#include <Moby/Constants.h>
#include <Moby/RNEAlgorithm.h>

#include <Moby/RestingContactHandler.h>
#include <Moby/Event.h>
#include <Moby/ContactProblemData.h>

using namespace Ravelin;
using namespace Moby;
using std::list;
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
boost::shared_ptr<EventDrivenSimulator> sim;
const unsigned NSPATIAL = 6;
/// Event callback function for processing events
void post_event_callback_fn(const vector<Event>& e, boost::shared_ptr<void> empty)
{
//  vector<Event> c(e.size());
//  for(int i=0;i<e.size();i++)
//  {
//    c[i] = e[i];
//    c[i].deriv_type = Event::eAccel;
//  }

//  RestingContactHandler rch_;

//  rch_.process_contacts(c);
}


/// Event callback function for setting friction vars pre-event
void pre_event_callback_fn(vector<Event>& e, boost::shared_ptr<void> empty)
{


//  vector<Event> c(e.size());
//  for(int i=0;i<e.size();i++)
//  {
//    c[i] = e[i];
//    c[i].deriv_type = Event::eAccel;
//    c[i].contact_mu_coulomb = 0.1;
//  }

//  RestingContactHandler rch_;

//  rch_.process_events(c);
}

/// The main control loop
void controller(DynamicBodyPtr body, double time, void*){

  Vec q(NSPATIAL+1),qd(NSPATIAL);
  body->get_generalized_coordinates(DynamicBody::eEuler,q);
  std::cout << "q " << q << std::endl;
  body->get_generalized_velocity(DynamicBody::eSpatial,qd);
  std::cout << "qd " << qd << std::endl;
}

/// plugin must be "extern C"
extern "C" {
void init(
          void* separator,
          const std::map<std::string,
          BasePtr>& read_map,
          double time
         ){
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

  q_start[2] = 0.52484665;
  qd_start[2] = 0;
  qd_start[1] = 0.5;
  qd_start[0] = 1;

  body->set_generalized_coordinates(DynamicBody::eEuler,q_start);
  body->set_generalized_velocity(DynamicBody::eSpatial,qd_start);

  // setup the controller
  body->controller = controller;

  obj = dynamic_pointer_cast<RigidBody>(body);

//  sim->event_post_impulse_callback_fn = post_event_callback_fn;

  sim->event_callback_fn = pre_event_callback_fn;

}
} // end extern C
