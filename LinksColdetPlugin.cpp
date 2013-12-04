#include <Moby/RCArticulatedBody.h>
#include "LinksColdetPlugin.h"

using std::vector;
using std::pair;
using namespace Ravelin;
using namespace Moby;

bool LinksColdetPlugin::is_contact(double dt, const vector<pair<DynamicBodyPtr, VectorNd> >& q0, const vector<pair<DynamicBodyPtr, VectorNd> >& q1, vector<Event>& contacts)
{
  // set bodies to all q0 coordinates
  for (unsigned i=0; i< q0.size(); i++)
    q0[i].first->set_generalized_coordinates(DynamicBody::eEuler, q0[i].second);

  // clear the vector of contacts
  contacts.clear();

  // get the links model
  RCArticulatedBodyPtr rcab = boost::dynamic_pointer_cast<RCArticulatedBody>(_robot);

  // look through all end-effectors
  for (unsigned i=0; i< rcab->get_links().size(); i++)
  {
    // if the link is not an end-effector, skip it
    if (!rcab->get_links()[i]->is_end_effector())
      continue;

    // the link is an end-effector; get its center-of-mass
    RigidBodyPtr link = rcab->get_links()[i];
    Point3d comx(0,0,0,link->get_inertial_pose());
    Point3d com = Pose3d::transform_point(GLOBAL, comx);

    // check whether com is below ground plane 0.0
    if (com[2] <= 0.0)
    {
      // setup the contact event
      Event e;
      e.t = 0.0;
      e.event_type = Event::eContact;
      e.contact_geom1 = link->geometries.front();
      e.contact_geom2 = _ground_cg;
      e.contact_normal = Vector3d(0,0,1,GLOBAL);
      e.contact_point = com;
      e.contact_point[2] = 0.0;  // make sure it contacts on the plane
      contacts.push_back(e);
    }
  }

  return (!contacts.empty());
} 

bool LinksColdetPlugin::is_collision(double epsilon)
{
  // get the links model
  RCArticulatedBodyPtr rcab = boost::dynamic_pointer_cast<RCArticulatedBody>(_robot);

  for (unsigned i=0; i< rcab->get_links().size(); i++)
  {
    // if the link is not an end-effector, skip it
    if (!rcab->get_links()[i]->is_end_effector())
      continue;

    // the link is an end-effector; get its center-of-mass
    RigidBodyPtr link = rcab->get_links()[i];
    Point3d comx(0,0,0,link->get_inertial_pose());
    Point3d com = Pose3d::transform_point(GLOBAL, comx);

    // check whether com is below ground plane 0.0
    if (com[2] <= 0.0)
      return true; 
  }
}


