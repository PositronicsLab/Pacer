#include <boost/shared_ptr.hpp>
#include <Moby/Simulator.h>

#include <Moby/Primitive.h>
#include <Moby/BoxPrimitive.h>
#include <Moby/SpherePrimitive.h>
#include <Moby/CylinderPrimitive.h>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>
#include <osg/Plane>
#include <osg/LineSegment>
#include <osg/LineWidth>
#include <osgText/Text>

using namespace Moby;
using namespace Ravelin;
const double VIBRANCY = 1;

double fRand(double fMin, double fMax)
{
  double f = (double)rand() / RAND_MAX;
  return fMin + f * (fMax - fMin);
}

osg::ref_ptr<osg::Geode> buildSphere( const double radius,
                                     const unsigned int rings,
                                     const unsigned int sectors )
{
  osg::ref_ptr<osg::Geode>      sphereGeode = new osg::Geode;
  osg::ref_ptr<osg::Geometry>   sphereGeometry = new osg::Geometry;
  osg::ref_ptr<osg::Vec3Array>  sphereVertices = new osg::Vec3Array;
  osg::ref_ptr<osg::Vec3Array>  sphereNormals = new osg::Vec3Array;
  osg::ref_ptr<osg::Vec2Array>  sphereTexCoords = new osg::Vec2Array;
  
  float const R = 1. / static_cast<float>( rings - 1 );
  float const S = 1. / static_cast<float>( sectors - 1 );
  
  sphereGeode->addDrawable( sphereGeometry );
  
  // Establish texture coordinates, vertex list, and normals
  for( unsigned int r( 0 ); r < rings; ++r ) {
    for( unsigned int s( 0) ; s < sectors; ++s ) {
      float const y = sin( -M_PI_2 + M_PI * r * R );
      float const x = cos( 2 * M_PI * s * S) * sin( M_PI * r * R );
      float const z = sin( 2 * M_PI * s * S) * sin( M_PI * r * R );
      
      sphereTexCoords->push_back( osg::Vec2( s * R, r * R ) );
      
      sphereVertices->push_back ( osg::Vec3( x * radius,
                                            y * radius,
                                            z * radius) )
      ;
      sphereNormals->push_back  ( osg::Vec3( x, y, z ) );
      
    }
  }
  
  sphereGeometry->setVertexArray  ( sphereVertices  );
  sphereGeometry->setTexCoordArray( 0, sphereTexCoords );
  
  osg::Vec4 color;
  {
    double R = fRand(0,0.25) * VIBRANCY;
    double G = fRand(0,0.25) * VIBRANCY;
    double B = fRand(0,0.25) * VIBRANCY;
    color = osg::Vec4(R,G,B, 1.0 );
  }
  osg::Vec4Array* colors = new osg::Vec4Array;
  
  // Generate quads for each face.
  for( unsigned int r( 0 ); r < rings - 1; ++r ) {
    for( unsigned int s( 0 ); s < sectors - 1; ++s ) {
      
      osg::ref_ptr<osg::DrawElementsUInt> face =
      new osg::DrawElementsUInt( osg::PrimitiveSet::QUADS,
                                4 )
      ;
      // Corners of quads should be in CCW order.
      face->push_back( ( r + 0 ) * sectors + ( s + 0 ) );
      face->push_back( ( r + 0 ) * sectors + ( s + 1 ) );
      face->push_back( ( r + 1 ) * sectors + ( s + 1 ) );
      face->push_back( ( r + 1 ) * sectors + ( s + 0 ) );
      colors->push_back( color );
      sphereGeometry->addPrimitiveSet( face );
    }
  }
  sphereGeometry->setColorArray(colors);
  sphereGeometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
  return sphereGeode;
}

osg::ref_ptr<osg::Geode> buildBox( const double x,const double y,const double z )
{
  osg::ref_ptr<osg::Geode>      boxGeode = new osg::Geode;
  osg::ref_ptr<osg::Geometry>   boxGeometry = new osg::Geometry;
 
  osg::Vec3 center(0,0,0);
  osg::Box* box = new osg::Box(center, x,y,z );

  osg::TessellationHints *hints = new osg::TessellationHints();
  hints->setTessellationMode( osg::TessellationHints::USE_TARGET_NUM_FACES );
  hints->setCreateNormals( true );
  hints->setDetailRatio( 0.1 );

  osg::ShapeDrawable* shape = new osg::ShapeDrawable( box, hints );
  
  double R = fRand(0,0.25) * VIBRANCY;
  double G = fRand(0,0.25) * VIBRANCY;
  double B = fRand(0,0.25) * VIBRANCY;
  osg::Vec4 color = osg::Vec4(R,G,B, 1.0 );
  shape->setColor( color );
  
  boxGeode->addDrawable( shape );
  
  return boxGeode;
}

osg::ref_ptr<osg::Geode> buildCylinder( const double r,const double h )
{
  osg::ref_ptr<osg::Geode>      cylinderGeode = new osg::Geode;
  osg::ref_ptr<osg::Geometry>   cylinderGeometry = new osg::Geometry;
  
  osg::Vec3 center(0,0,0);
  osg::Cylinder* cylinder = new osg::Cylinder(center, r,h );
  
  osg::TessellationHints *hints = new osg::TessellationHints();
  hints->setTessellationMode( osg::TessellationHints::USE_TARGET_NUM_FACES );
  hints->setCreateNormals( true );
  hints->setDetailRatio( 0.1 );
  
  osg::ShapeDrawable* shape = new osg::ShapeDrawable( cylinder, hints );
  
  double R = fRand(0,0.25) * VIBRANCY;
  double G = fRand(0,0.25) * VIBRANCY;
  double B = fRand(0,0.25) * VIBRANCY;
  osg::Vec4 color = osg::Vec4(R,G,B, 1.0 );
  shape->setColor( color );
  
  cylinderGeode->addDrawable( shape );
  
  return cylinderGeode;
}

osg::Group* visualize_box(double x, double y, double z,const Ravelin::Pose3d& pose){

  // the osg node this event visualization will attach to
  osg::Group* group_root = new osg::Group();
  
  // turn off lighting for this node
  osg::StateSet *lighting = group_root->getOrCreateStateSet();
  lighting->setMode( GL_LIGHTING, osg::StateAttribute::PROTECTED | osg::StateAttribute::OFF );
  
  // a geode for the visualization geometry
  osg::ref_ptr<osg::Geode> geode = buildBox(x,y,z);
  
  // add some hints to reduce the polygonal complexity of the visualization
  osg::TessellationHints *hints = new osg::TessellationHints();
  hints->setTessellationMode( osg::TessellationHints::USE_TARGET_NUM_FACES );
  hints->setCreateNormals( true );
  hints->setDetailRatio( 0.1 );
  
  osg::PositionAttitudeTransform* transform;
  transform = new osg::PositionAttitudeTransform();
  transform->setPosition( osg::Vec3( pose.x[0], pose.x[1], pose.x[2] ) );
  transform->setAttitude( osg::Quat( pose.q.x,pose.q.y,pose.q.z,pose.q.w ) );
  transform->setScale( osg::Vec3( 1, 1, 1 ) );
  
  // add the geode to the transform
  transform->addChild( geode );
  
  // add the transform to the root
  group_root->addChild( transform );
  
  // add the root to the transient data scene graph
  return group_root;
}

osg::Group* visualize_sphere(double r, const Ravelin::Pose3d& pose){
  double R = fRand(0,0.25) * VIBRANCY;
  double G = fRand(0,0.25) * VIBRANCY;
  double B = fRand(0,0.25) * VIBRANCY;
  osg::Vec4 color = osg::Vec4(R,G,B, 1.0 );
  
  // the osg node this event visualization will attach to
  osg::Group* group_root = new osg::Group();
  
  // turn off lighting for this node
  osg::StateSet *lighting = group_root->getOrCreateStateSet();
  lighting->setMode( GL_LIGHTING, osg::StateAttribute::PROTECTED | osg::StateAttribute::OFF );
  
  // a geode for the visualization geometry
  osg::ref_ptr<osg::Geode> geode = buildSphere(r,10,10);
  
  // add some hints to reduce the polygonal complexity of the visualization
  osg::TessellationHints *hints = new osg::TessellationHints();
  hints->setTessellationMode( osg::TessellationHints::USE_TARGET_NUM_FACES );
  hints->setCreateNormals( true );
  hints->setDetailRatio( 0.1 );
  
//  geode->setColor( color );
  
  osg::PositionAttitudeTransform* transform;
  transform = new osg::PositionAttitudeTransform();
  transform->setPosition( osg::Vec3( pose.x[0], pose.x[1], pose.x[2] ) );
  transform->setAttitude( osg::Quat( pose.q.x,pose.q.y,pose.q.z,pose.q.w ) );
  transform->setScale( osg::Vec3( 1, 1, 1 ) );
  
  // add the geode to the transform
  transform->addChild( geode );
  
  // add the transform to the root
  group_root->addChild( transform );
  
  // add the root to the transient data scene graph
  return group_root;
}

osg::Group* visualize_cylinder(double r, double h,const Ravelin::Pose3d& pose){
  
  // the osg node this event visualization will attach to
  osg::Group* group_root = new osg::Group();
  
  // turn off lighting for this node
  osg::StateSet *lighting = group_root->getOrCreateStateSet();
  lighting->setMode( GL_LIGHTING, osg::StateAttribute::PROTECTED | osg::StateAttribute::OFF );
  
  // a geode for the visualization geometry
  osg::ref_ptr<osg::Geode> geode = buildCylinder(r,h);
  
  // add some hints to reduce the polygonal complexity of the visualization
  osg::TessellationHints *hints = new osg::TessellationHints();
  hints->setTessellationMode( osg::TessellationHints::USE_TARGET_NUM_FACES );
  hints->setCreateNormals( true );
  hints->setDetailRatio( 0.1 );
  
  osg::PositionAttitudeTransform* transform;
  transform = new osg::PositionAttitudeTransform();
  transform->setPosition( osg::Vec3( pose.x[0], pose.x[1], pose.x[2] ) );
  transform->setAttitude( osg::Quat( pose.q.x,pose.q.y,pose.q.z,pose.q.w ) );
  transform->setScale( osg::Vec3( 1, 1, 1 ) );
  
  // add the geode to the transform
  transform->addChild( geode );
  
  // add the transform to the root
  group_root->addChild( transform );
  
  // add the root to the transient data scene graph
  return group_root;
}

void visualize_primitive(Moby::PrimitivePtr& primitive, boost::shared_ptr<const Ravelin::Pose3d>& pose_ptr, boost::shared_ptr<Simulator>& sim ){
  Ravelin::Pose3d pose(pose_ptr->q,pose_ptr->x,pose_ptr->rpose);
  pose.update_relative_pose(GLOBAL);
  
  boost::shared_ptr<Moby::BoxPrimitive> box = boost::dynamic_pointer_cast<Moby::BoxPrimitive>(primitive);
  if(box){
    double x = box->get_x_len(),
    y = box->get_y_len(),
    z = box->get_z_len();
    osg::Node * node = visualize_box(x,y,z,pose);
    sim->add_transient_vdata( node );
    return;
  }
  
  boost::shared_ptr<Moby::SpherePrimitive> sphere = boost::dynamic_pointer_cast<Moby::SpherePrimitive>(primitive);
  if (sphere){
    double r = sphere->get_radius();
    osg::Node * node = visualize_sphere(r,pose);
    sim->add_transient_vdata( node );
    return;
  }
  
  boost::shared_ptr<Moby::CylinderPrimitive> cylinder = boost::dynamic_pointer_cast<Moby::CylinderPrimitive>(primitive);
  if (cylinder){
    double r = cylinder->get_radius(),
           h = cylinder->get_height();
    osg::Node * node = visualize_cylinder(r,h,pose);
    sim->add_transient_vdata( node );
    return;
  }
  OUT_LOG(logDEBUG1) << "Body has unknown collision data, can't visualize!";
}

/// Draws a ray directed from a contact point along the contact normal
void visualize_ray( const Ravelin::Vector3d& point, const Ravelin::Vector3d& vec, const Ravelin::Vector3d& c,double point_radius, boost::shared_ptr<Simulator> sim ) {
  
  // random color for this contact visualization
  double r = c[0] * VIBRANCY;
  double g = c[1] * VIBRANCY;
  double b = c[2] * VIBRANCY;
  osg::Vec4 color = osg::Vec4( r, g, b, 1.0 );
  
  const double point_scale = point_radius;
  
  // the osg node this event visualization will attach to
  osg::Group* group_root = new osg::Group();
  
  // turn off lighting for this node
  osg::StateSet *point_state = group_root->getOrCreateStateSet();
  point_state->setMode( GL_LIGHTING, osg::StateAttribute::PROTECTED | osg::StateAttribute::OFF );
  
  // a geode for the visualization geometry
  osg::Geode* point_geode = new osg::Geode();
  
  // add some hints to reduce the polygonal complexity of the visualization
  osg::TessellationHints *hints = new osg::TessellationHints();
  hints->setTessellationMode( osg::TessellationHints::USE_TARGET_NUM_FACES );
  hints->setCreateNormals( true );
  hints->setDetailRatio( 0.1 );
  
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
  group_root->addChild( point_transform );
  
  // add the root to the transient data scene graph
  sim->add_transient_vdata( group_root );
  
  // ----- LINE -------
  
  //  osg::Group* vec_root = new osg::Group();
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
  group_root->addChild( vec_transform );
  
  // add the root to the transient data scene graph
  sim->add_transient_vdata( group_root );
}

void visualize_ray( const Ravelin::Vector3d& point, const Ravelin::Vector3d& vec, const Ravelin::Vector3d& c, boost::shared_ptr<Simulator> sim ) {
  visualize_ray(point,vec,c,0.1,sim);
}

void draw_pose(const Ravelin::Pose3d& p, boost::shared_ptr<Simulator> sim ,double lightness, double size, const Ravelin::Vector3d& c){
  Ravelin::Pose3d pose(p);
  assert(lightness >= 0.0 && lightness <= 2.0);
  pose.update_relative_pose(Pacer::GLOBAL);
  Ravelin::Matrix3d Rot(pose.q);
  Rot*= 0.3;
  double alpha = (lightness > 1.0)? 1.0 : lightness,
  beta = (lightness > 1.0)? lightness-1.0 : 0.0;
  
  if (c.norm() > 0) {
    visualize_ray(pose.x+Ravelin::Vector3d(Rot(0,0),Rot(1,0),Rot(2,0),Pacer::GLOBAL)*size,Ravelin::Vector3d(0,0,0)+pose.x,c,size,sim);
    visualize_ray(pose.x+Ravelin::Vector3d(Rot(0,1),Rot(1,1),Rot(2,1),Pacer::GLOBAL)*size,Ravelin::Vector3d(0,0,0)+pose.x,c*0.75,size,sim);
    visualize_ray(pose.x+Ravelin::Vector3d(Rot(0,2),Rot(1,2),Rot(2,2),Pacer::GLOBAL)*size,Ravelin::Vector3d(0,0,0)+pose.x,c*0.5,size,sim);
  } else {
  visualize_ray(pose.x+Ravelin::Vector3d(Rot(0,0),Rot(1,0),Rot(2,0),Pacer::GLOBAL)*size,Ravelin::Vector3d(0,0,0)+pose.x,Ravelin::Vector3d(alpha,beta,beta),size,sim);
  visualize_ray(pose.x+Ravelin::Vector3d(Rot(0,1),Rot(1,1),Rot(2,1),Pacer::GLOBAL)*size,Ravelin::Vector3d(0,0,0)+pose.x,Ravelin::Vector3d(beta,alpha,beta),size,sim);
  visualize_ray(pose.x+Ravelin::Vector3d(Rot(0,2),Rot(1,2),Rot(2,2),Pacer::GLOBAL)*size,Ravelin::Vector3d(0,0,0)+pose.x,Ravelin::Vector3d(beta,beta,alpha),size,sim);
}
}

void draw_text(std::string& text_str, const Ravelin::Vector3d& point, const Ravelin::Quatd& quat, const Ravelin::Vector3d& c, boost::shared_ptr<Simulator> sim, double size){
  
  osg::Group* 	group_root 	= new osg::Group();
  osg::Billboard* billboard = new osg::Billboard();
  osgText::Text* text   	= new osgText::Text();
  
  osgViewer::Viewer viewer;
  
  double r = c[0] * VIBRANCY;
  double g = c[1] * VIBRANCY;
  double b = c[2] * VIBRANCY;
  osg::Vec4 color = osg::Vec4( r, g, b, 1.0 );
  
//  osgText::Font font = new osgText::readFontFile( "fonts/arial.ttf" );
  
//  osg::Vec4 white( 1.f, 1.f, 1.f, 1.f );
  
  { //top right
    osg::ref_ptr<osgText::Text> text = new osgText::Text;
//    text->setFont( font.get() );
    text->setColor( color );
    text->setCharacterSize( .15f );
    text->setPosition( osg::Vec3( 0.f, 0.f, 0.f ) );
    text->setAxisAlignment( osgText::Text::SCREEN );
    text->setText( text_str.c_str() );
  }

  // a geode for the visualization geometry
  osg::Geode* point_geode = new osg::Geode();
  
  osg::PositionAttitudeTransform* point_transform;
  point_transform = new osg::PositionAttitudeTransform();
  point_transform->setPosition( osg::Vec3( point[0], point[1], point[2] ) );
  //point_transform->setScale( osg::Vec3( point_scale, point_scale, point_scale ) );
  
  
  // add the geode to the transform
  point_transform->addChild( point_geode );
  point_geode->addDrawable( text );
  
  // add the transform to the root
  group_root->addChild( point_transform );
  
  // add the root to the transient data scene graph
  sim->add_transient_vdata( group_root );
}

