#include <Moby/RigidBody.h>
#include <Moby/SpherePrimitive.h>
#include <Moby/ContactParameters.h>
#include <Moby/EventDrivenSimulator.h>

using boost::shared_ptr;
using namespace Ravelin;
using namespace Moby;

RigidBodyPtr create_terrain(unsigned nsph, double width_ext, double height_ext, double max_radius, double min_radius)
{
  RigidBodyPtr rb(new RigidBody);

  // make the terrain primitive be disabled
  rb->set_enabled(false);

  for (unsigned i=0; i< nsph; i++)
  {
    // set the radius
    double rad = (double) rand() / RAND_MAX * (max_radius -min_radius) + min_radius;

    // set the pose on the collision geometry
    Pose3d T;
//    T.update_relative_pose(rb->get_pose());
    T.update_relative_pose(Moby::GLOBAL);
    T.x[0] = (double) rand()/RAND_MAX * 2.0 * width_ext - width_ext;
    T.x[2] = (double) rand()/RAND_MAX * 2.0 * height_ext - height_ext;
    T.x[1] = (double) rand()/RAND_MAX * 2.0 * width_ext - width_ext;

    // create the sphere primitive
    shared_ptr<SpherePrimitive> s(new SpherePrimitive(rad,T));

    // create the collision geometry
    CollisionGeometryPtr cg(new CollisionGeometry);
    cg->set_single_body(rb);
    cg->set_geometry(s);

    rb->geometries.push_back(cg);
  }

  return rb;
}

void set_terrain_robot_contact_parameters(DynamicBodyPtr terrain, DynamicBodyPtr robot, shared_ptr<EventDrivenSimulator> sim)
{
  // create the contact parameters
  shared_ptr<ContactParameters> cp(new ContactParameters);

  // note: alter contact parameters here
  cp->objects = make_sorted_pair(terrain, robot);
  cp->epsilon = 0.0;
  cp->mu_coulomb = 0;
  cp->mu_viscous = 1;
  cp->NK = 4;

  // set it up in the simulator
  sim->contact_params[make_sorted_pair(terrain, robot)] = cp;
}

boost::shared_ptr<ContactParameters> cp_callback(CollisionGeometryPtr g1, CollisionGeometryPtr g2)
{
  boost::shared_ptr<ContactParameters> cp(new ContactParameters);
  const unsigned UINF = std::numeric_limits<unsigned>::max();

  // setup zero restitution and viscous friction
  cp->epsilon = 0.0;
  cp->mu_viscous = 0.0;

  // setup a random coefficient of friction
  cp->mu_coulomb = 0.0 + ((double) rand() / RAND_MAX)/2;

  // setup NK
//  cp->NK = UINF;
  cp->NK = 4;
  std::cout << "MU = " << cp->mu_coulomb << std::endl;

  return cp;
}


#include <osg/Group>

void create_terrain_visualization(RigidBodyPtr rb, shared_ptr<EventDrivenSimulator> sim)
{
 osg::Group * ogg = (osg::Group*) sim->get_persistent_vdata();
 BOOST_FOREACH(CollisionGeometryPtr cg, rb->geometries)
   ogg->addChild(cg->get_geometry()->get_visualization());
}
