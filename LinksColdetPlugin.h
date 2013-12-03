/****************************************************************************
 * Copyright 2013 Evan Drumwright
 * This library is distributed under the terms of the GNU Lesser General Public 
 * License (found in COPYING).
 ****************************************************************************/

#ifndef _LINKS_COLLISION_DETECTION_H
#define _LINKSCOLLISION_DETECTION_H

#include <map>
#include <set>
#include <Ravelin/VectorNd.h>
#include <Ravelin/Pose3d.h>
#include <Moby/sorted_pair>
#include <Moby/Base.h>
#include <Moby/Event.h>
#include <Moby/CollisionDetection.h>
#include <Moby/RigidBody.h>

namespace Moby {

/// Defines an abstract collision detection mechanism
/**
 * Contact finding and collision detection are two separate, but related, 
 * procedures. Collision detection quickly determines whether two bodies are 
 * geometrically intersecting at their current configurations. Contact finding
 * determines whether two bodies will come into contact (or are already in 
 * contact) in a given time interval; if the bodies do/will contact, contact
 * finding computes contact points and normals. 
 */
class LinksColdetPlugin : public CollisionDetection 
{
  public:
    virtual ~LinksColdetPlugin() {}
    virtual void load_from_xml(boost::shared_ptr<const XMLTree> node, std::map<std::string, BasePtr>& id_map);
    virtual void add_rigid_body(RigidBodyPtr body) { assert(!body->geometries.empty()); _ground_cg = body->geometries.front(); }
    virtual void add_articulated_body(ArticulatedBodyPtr abody, bool disable_adjacent) { _robot = abody; }

    /// Determines whether there is a contact between the given pairs of states 
    /**
     * Generalized coordinates are set to q1 on entry; generalized coordinates
     * should be set to q0 on return.
     * \param events the set of determined contacts, on return
     * \pre body states are set to appropriate states in q1
     * \post body states are arbitrary on return
     * \return <b>true</b> if there is contact in the time interval, 
     *           <b>false</b> otherwise
     */
    virtual bool is_contact(double dt, const std::vector<std::pair<DynamicBodyPtr, Ravelin::VectorNd> >& q0, const std::vector<std::pair<DynamicBodyPtr, Ravelin::VectorNd> >& q1, std::vector<Event>& contacts);

    /// Determines whether there is a collision at the current simulation state with the given tolerance
    /**
     * \param epsilon the tolerance to check for collision; two bodies are 
     *        considered to be in collision if the distance between the 
     *        bodies is less than epsilon
     * \note implementing classes must store the colliding pairs in 
     *         _colliding_pairs
     */
    virtual bool is_collision(double epsilon = 0.0);

  private:
    DynamicBodyPtr _robot;
    CollisionGeometryPtr _ground_cg;
};
} // end namespace Moby

#endif

