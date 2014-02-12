///////////////////////////////////////////////////////////////////////////////
/////////////////////////////// Visualization /////////////////////////////////

#include <boost/shared_ptr.hpp>
#include <Moby/EventDrivenSimulator.h>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>
#include <osg/Plane>
#include <osg/LineSegment>
#include <osg/LineWidth>

#include <project_common.h>

using namespace Moby;
using namespace Ravelin;

const double VIBRANCY = 0.5;

/// Draws a ray directed from a contact point along the contact normal
void visualize_contact( const Event& event, boost::shared_ptr<EventDrivenSimulator> sim ) {

  // random color for this contact visualization
  double r = (double) rand() / (double) RAND_MAX * VIBRANCY;
  double g = (double) rand() / (double) RAND_MAX * VIBRANCY;
  double b = (double) rand() / (double) RAND_MAX * VIBRANCY;
  osg::Vec4 color = osg::Vec4( r, g, b, 1.0 );

  // knobs for tweaking
  double impulse_norm = event.contact_impulse[0]*event.contact_impulse[0] +
                        event.contact_impulse[1]*event.contact_impulse[1] +
                        event.contact_impulse[2]*event.contact_impulse[2] ;
  const double point_radius = 0.25;
  const double point_scale = 0.01;
  const double line_length = 5.0*impulse_norm;
  const double line_radius = 0.1;
  const double head_radius = 0.5;
  const double head_height = 2.0;

  // the osg node this event visualization will attach to
  osg::Group* contact_root = new osg::Group();

  // turn off lighting for this node
  osg::StateSet *contact_state = contact_root->getOrCreateStateSet();
  contact_state->setMode( GL_LIGHTING, osg::StateAttribute::PROTECTED | osg::StateAttribute::OFF );

  // a geode for the visualization geometry
  osg::Geode* contact_geode = new osg::Geode();

  // add some hints to reduce the polygonal complexity of the visualization
  osg::TessellationHints *hints = new osg::TessellationHints();
  hints->setTessellationMode( osg::TessellationHints::USE_TARGET_NUM_FACES );
  hints->setCreateNormals( true );
  hints->setDetailRatio( 0.001 );

  // add the contact point as a sphere at the origin of the geode's frame
  osg::Sphere* point_geometry = new osg::Sphere( osg::Vec3( 0, 0, 0 ), point_radius );
  osg::ShapeDrawable* point_shape = new osg::ShapeDrawable( point_geometry, hints );
  point_shape->setColor( color );
  contact_geode->addDrawable( point_shape );

  // add the contact normal as a cylinder in the geode's frame
  osg::Cylinder* line_geometry = new osg::Cylinder( osg::Vec3( 0.0, 0.0, line_length / 2 ), line_radius, line_length );
  osg::ShapeDrawable* line_shape = new osg::ShapeDrawable( line_geometry, hints );
  line_shape->setColor( color );
  contact_geode->addDrawable( line_shape );

  // add the arrow head as a cone in the geode's frame
  osg::Cone* head_geometry = new osg::Cone( osg::Vec3( 0, 0, line_length ), head_radius, head_height );
  osg::ShapeDrawable* head_shape = new osg::ShapeDrawable( head_geometry, hints );
  head_shape->setColor( color );
  contact_geode->addDrawable( head_shape );

  // calculate the orientation based upon the direction of the normal vector.
  // Note: the default orientation of the osg model is along the z-axis
  double theta;
  Vector3d z = Vector3d( 0.0, 0.0, 1.0 );
  Vector3d axis = Vector3d::cross( event.contact_normal, z );
  if( axis.norm_inf() < NEAR_ZERO) {
    // z and normal are parallel, axis ill defined
    if( event.contact_normal[2] > 0 ) {
      // normal is z
      axis = Vector3d( 0.0, 1.0, 0.0 );
      theta = 0.0;
    } else {
      // normal is -z
      axis = Vector3d( 0.0, 1.0, 0.0 );
      theta = osg::PI;
    }
  } else {
    // axis is well defined
    axis = Vector3d::normalize(axis);
    theta = -std::acos( Vector3d::dot( event.contact_normal, z ) );
    // Note: theta calculation works but is not robust, could be rotated in opposite direction
  }
  osg::Quat q = osg::Quat( axis[0]*std::sin(theta/2), axis[1]*std::sin(theta/2), axis[2]*std::sin(theta/2), std::cos(theta/2) );

  // create the visualization transform
  osg::PositionAttitudeTransform* contact_transform;
  contact_transform = new osg::PositionAttitudeTransform();
  contact_transform->setPosition( osg::Vec3( event.contact_point[0], event.contact_point[1], event.contact_point[2] ) );
  contact_transform->setScale( osg::Vec3( point_scale, point_scale, point_scale ) );
  contact_transform->setAttitude( q );

  // add the geode to the transform
  contact_transform->addChild( contact_geode );

  // add the transform to the root
  contact_root->addChild( contact_transform );

  // add the root to the transient data scene graph
  sim->add_transient_vdata( contact_root );

  // JRT : remove validator once theta 100% proven
  // -----------------------------------------
  // Rotational Validator
  // -----------------------------------------

  // Validator is a simple sphere translated along the normal
  // such that the visualization above should point at the center
  // of the validator.  If it doesn't, then the calculation of
  // theta in the rotational code above needs correction for that case

  // knobs for tweaking
  const double validator_scale = point_scale / 3;
  const double validator_ray_length = line_length * 2.5;

  // a root for the validator
  osg::Group* validator_root = new osg::Group();

  // turn off lighting for this node
  osg::StateSet *validator_state = validator_root->getOrCreateStateSet();
  validator_state->setMode( GL_LIGHTING, osg::StateAttribute::PROTECTED | osg::StateAttribute::OFF );

  // colocate the validator position to the contact point
  osg::PositionAttitudeTransform* validator_transform = new osg::PositionAttitudeTransform();
  validator_transform->setPosition( osg::Vec3( event.contact_point[0], event.contact_point[1], event.contact_point[2] ) );
  validator_transform->setScale( osg::Vec3( validator_scale, validator_scale, validator_scale ) );
  validator_root->addChild( validator_transform );

  // validator geometry
  osg::Sphere* validator_geometry = new osg::Sphere( osg::Vec3( 0, 0, 0 ), 1.0 );
  osg::ShapeDrawable* validator_shape = new osg::ShapeDrawable( validator_geometry, hints );
  validator_shape->setColor( color );

  // validator transform follows the normal out to a distance of validator_ray_length
  // Note: the validator is not rotated at all.  It is translated from the point along the normal
  osg::PositionAttitudeTransform* validator_end_transform = new osg::PositionAttitudeTransform();
  validator_end_transform->setPosition( osg::Vec3( event.contact_normal[0] * validator_ray_length, event.contact_normal[1] * validator_ray_length, event.contact_normal[2] * validator_ray_length ) );
  validator_transform->addChild( validator_end_transform );

  // add all validator constituents to the group
  osg::Geode* validator_geode = new osg::Geode();
  validator_transform->addChild( validator_end_transform );
  validator_end_transform->addChild( validator_geode );
  validator_geode->addDrawable( validator_shape );

  sim->add_transient_vdata( validator_root );
}

/// Draws a ray directed from a contact point along the contact normal
void visualize_ray( const Ravelin::Vector3d& point, const Ravelin::Vector3d& vec, boost::shared_ptr<EventDrivenSimulator> sim ) {

  // random color for this contact visualization
  double r = (double) rand() / (double) RAND_MAX * VIBRANCY;
  double g = (double) rand() / (double) RAND_MAX * VIBRANCY;
  double b = (double) rand() / (double) RAND_MAX * VIBRANCY;
  osg::Vec4 color = osg::Vec4( r, g, 1.0, 1.0 );

  const double point_radius = 0.25;
  const double point_scale = 0.01;

  // the osg node this event visualization will attach to
  osg::Group* point_root = new osg::Group();

  // turn off lighting for this node
  osg::StateSet *point_state = point_root->getOrCreateStateSet();
  point_state->setMode( GL_LIGHTING, osg::StateAttribute::PROTECTED | osg::StateAttribute::OFF );

  // a geode for the visualization geometry
  osg::Geode* point_geode = new osg::Geode();

  // add some hints to reduce the polygonal complexity of the visualization
  osg::TessellationHints *hints = new osg::TessellationHints();
  hints->setTessellationMode( osg::TessellationHints::USE_TARGET_NUM_FACES );
  hints->setCreateNormals( true );
  hints->setDetailRatio( 0.001 );

  osg::Sphere* point_geometry = new osg::Sphere( osg::Vec3( 0,0,0 ), point_radius );
  osg::ShapeDrawable* point_shape = new osg::ShapeDrawable( point_geometry, hints );
  point_shape->setColor( color );
  point_geode->addDrawable( point_shape );

  osg::PositionAttitudeTransform* point_transform;
  point_transform = new osg::PositionAttitudeTransform();
  point_transform->setPosition( osg::Vec3( point[0], point[1], point[2] ) );
  point_transform->setScale( osg::Vec3( point_scale, point_scale, point_scale ) );

  // add the geode to the transform
  point_transform->addChild( point_geode );

  // add the transform to the root
  point_root->addChild( point_transform );

  // add the root to the transient data scene graph
  sim->add_transient_vdata( point_root );

  // ----- LINE -------

  osg::Group* vec_root = new osg::Group();
  osg::Geode* vec_geode = new osg::Geode();
  osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
  osg::ref_ptr<osg::DrawArrays> drawArrayLines =
      new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP);

  osg::ref_ptr<osg::Vec3Array> vertexData = new osg::Vec3Array;

  geom->addPrimitiveSet(drawArrayLines);
  geom->setVertexArray(vertexData);

  //loop through points
  vertexData->push_back(osg::Vec3d(point[0],point[1],point[2]));
  vertexData->push_back(osg::Vec3d(vec[0],vec[1],vec[2]));

  drawArrayLines->setFirst(0);
  drawArrayLines->setCount(vertexData->size());

  // Add the Geometry (Drawable) to a Geode and return the Geode.
  vec_geode->addDrawable( geom.get() );
  // the osg node this event visualization will attach to

  // add the root to the transient data scene graph
  // create the visualization transform
  osg::PositionAttitudeTransform* vec_transform;
  vec_transform = new osg::PositionAttitudeTransform();

  // add the geode to the transform
  vec_transform->addChild( vec_geode );

  // add the transform to the root
  vec_root->addChild( vec_transform );

  // add the root to the transient data scene graph
  sim->add_transient_vdata( vec_root );
}

/// Draws a ray directed from a contact point along the contact normal
void visualize_polygon( const Ravelin::MatrixNd& verts, boost::shared_ptr<EventDrivenSimulator> sim ) {

    /* must be ordered:
     *  1 - 2
     *  |   |
     *  3 - 4
     */
      double r = (double) rand() / (double) RAND_MAX * VIBRANCY;
      double g = (double) rand() / (double) RAND_MAX * VIBRANCY;
      double b = (double) rand() / (double) RAND_MAX * VIBRANCY;
      osg::Vec4 color = osg::Vec4( r, g, b, 1.0 );
      int nc = verts.columns();
      // knobs for tweaking
      const double point_radius = 0.75;
      const double point_scale = 0.01;
      const double line_radius = 0.1;

      // the osg node this event visualization will attach to
      osg::Group* polygon_root = new osg::Group();

      // turn off lighting for this node
      osg::StateSet *polygon_state = polygon_root->getOrCreateStateSet();
      polygon_state->setMode( GL_LIGHTING, osg::StateAttribute::PROTECTED | osg::StateAttribute::OFF );

      // a geode for the visualization geometry
      osg::Geode* polygon_geode = new osg::Geode();

      // add some hints to reduce the polygonal complexity of the visualization
      osg::TessellationHints *hints = new osg::TessellationHints();
      hints->setTessellationMode( osg::TessellationHints::USE_TARGET_NUM_FACES );
      hints->setCreateNormals( true );
      hints->setDetailRatio( 0.001 );

      /// -----------------------------------------------------
      /// -----------------------------------------------------

      osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
      osg::ref_ptr<osg::DrawArrays> drawArrayLines =
          new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP);

      osg::ref_ptr<osg::Vec3Array> vertexData = new osg::Vec3Array;

      geom->addPrimitiveSet(drawArrayLines);
      geom->setVertexArray(vertexData);

      //loop through points
      switch(nc){
      case 4:
        vertexData->push_back(osg::Vec3d(verts(0,0),verts(1,0),verts(2,0)));
        vertexData->push_back(osg::Vec3d(verts(0,1),verts(1,1),verts(2,1)));
        vertexData->push_back(osg::Vec3d(verts(0,3),verts(1,3),verts(2,3)));
        vertexData->push_back(osg::Vec3d(verts(0,2),verts(1,2),verts(2,2)));
        vertexData->push_back(osg::Vec3d(verts(0,0),verts(1,0),verts(2,0)));
        break;
      case 3:
        vertexData->push_back(osg::Vec3d(verts(0,1),verts(1,1),verts(2,1)));
        vertexData->push_back(osg::Vec3d(verts(0,0),verts(1,0),verts(2,0)));
        vertexData->push_back(osg::Vec3d(verts(0,2),verts(1,2),verts(2,2)));
        vertexData->push_back(osg::Vec3d(verts(0,1),verts(1,1),verts(2,1)));
        break;
      case 2:
        vertexData->push_back(osg::Vec3d(verts(0,1),verts(1,1),verts(2,1)));
      case 1:
        vertexData->push_back(osg::Vec3d(verts(0,0),verts(1,0),verts(2,0)));
      }
      drawArrayLines->setFirst(0);
      drawArrayLines->setCount(vertexData->size());

//      osg::LineWidth* lineWidth = new osg::LineWidth();
//      lineWidth->setWidth(2.0f);
//      osg::StateSet* ss;
//      ss->setAttributeAndModes(lineWidth,
//      osg::StateAttribute::ON);
//      geom->setStateSet(ss);

      // Add the Geometry (Drawable) to a Geode and return the Geode.
      polygon_geode->addDrawable( geom.get() );

      // create the visualization transform
      osg::PositionAttitudeTransform* polygon_transform;
      polygon_transform = new osg::PositionAttitudeTransform();

      // add the geode to the transform
      polygon_transform->addChild( polygon_geode );

      // add the transform to the root
      polygon_root->addChild( polygon_transform );

      // add the root to the transient data scene graph
      sim->add_transient_vdata( polygon_root );

}
