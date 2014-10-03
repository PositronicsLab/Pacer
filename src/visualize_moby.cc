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
extern double SIMULATION_TIME;
const double VIBRANCY = 1;

void visualize_ray( const Ravelin::Vector3d& point, const Ravelin::Vector3d& vec, const Ravelin::Vector3d& c, boost::shared_ptr<EventDrivenSimulator> sim ) {
  visualize_ray(point,vec,c,0.1,sim);
}

/// Draws a ray directed from a contact point along the contact normal
void visualize_ray( const Ravelin::Vector3d& point, const Ravelin::Vector3d& vec, const Ravelin::Vector3d& c,double point_radius, boost::shared_ptr<EventDrivenSimulator> sim ) {

  // random color for this contact visualization
  double r = c[0] * VIBRANCY;
  double g = c[1] * VIBRANCY;
  double b = c[2] * VIBRANCY;
  osg::Vec4 color = osg::Vec4( r, g, b, 1.0/point_radius );

  const double point_scale = 0.01;

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

void draw_pose(const Ravelin::Pose3d& p, boost::shared_ptr<EventDrivenSimulator> sim ,double lightness){
  Ravelin::Pose3d pose(p);
  assert(lightness >= 0.0 && lightness <= 2.0);
  pose.update_relative_pose(Moby::GLOBAL);
  Ravelin::Matrix3d Rot(pose.q);
  Rot*= 0.3;
  double alpha = (lightness > 1.0)? 1.0 : lightness,
         beta = (lightness > 1.0)? lightness-1.0 : 0.0;

  visualize_ray(pose.x+Ravelin::Vector3d(Rot(0,0),Rot(1,0),Rot(2,0),Moby::GLOBAL)/10,Ravelin::Vector3d(0,0,0)+pose.x,Ravelin::Vector3d(alpha,beta,beta),sim);
  visualize_ray(pose.x+Ravelin::Vector3d(Rot(0,1),Rot(1,1),Rot(2,1),Moby::GLOBAL)/10,Ravelin::Vector3d(0,0,0)+pose.x,Ravelin::Vector3d(beta,alpha,beta),sim);
  visualize_ray(pose.x+Ravelin::Vector3d(Rot(0,2),Rot(1,2),Rot(2,2),Moby::GLOBAL)/10,Ravelin::Vector3d(0,0,0)+pose.x,Ravelin::Vector3d(beta,beta,alpha),sim);
}


// ============================================================================
/**
 * What to draw each refresh
 * Called by GLUT
 */
#if defined(VISUALIZE_MOBY) && defined(USE_GLCONSOLE)
// Get these from https://github.com/arpg/GLConsole.git
//include this header for CVars and GLConsole
#include <GLConsole/GLConsole.h>
// Single global instance so glut can get access
extern GLConsole theConsole;

using namespace std;

void display()
{
  //set up the scene
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();
  glTranslatef(0,0.0f,-2.0f);

  //draw the console. always call it last so it is drawn on top of everything
  theConsole.RenderConsole();

  glutSwapBuffers();
}

/**
 * resize the window to the desired size
 * @param w Window width
 * @param h Window height
 */
void reshape (int w, int h)
{
  glViewport     ( 0, 0, w, h );
  glMatrixMode   ( GL_PROJECTION );
  glLoadIdentity ( );

  if ( h == 0 )
    gluPerspective ( 80, ( float ) w, 1.0, 5000.0 );
  else
    gluPerspective ( 80, ( float ) w / ( float ) h, 1.0, 5000.0 );

  glMatrixMode   ( GL_MODELVIEW );
  glLoadIdentity ( );
}

/**
 * just redisplay constantly to let the console update...
 */
void idle()
{
#ifdef WIN32
  Sleep( 1 );
#else
  usleep( (int)1e4 );
#endif
    glutPostRedisplay(); // we have
}

/**
 * Pass keboard events to the console.
 * Up and down arrows to scroll through console history.
 * Shift+up or shift+down to scroll the console window.
 * Pageup or pagedown to scroll the console window as well.
 */
void special(int key, int, int )
{
  if( theConsole.IsOpen() )
  {
    //pass all key strokes to the console
    theConsole.SpecialFunc( key );
  }
  else
  {
    //do your own thing with the keys
  }
}

/**
 * handle incoming key events and send to console
 */
void keyfunc( unsigned char key, int, int )
{
  switch( key ) {

    case 27:  //escape
      exit ( 0 );
      break;

    case GLCONSOLE_KEY: //~ key opens console on US keyboards.
                                  //On UK keyboards it is the ` key (the one above the Tab and left of the 1 keys)
      theConsole.ToggleConsole();
      break;

    default:
      if( theConsole.IsOpen() ) {
        //send keystroke to console
        theConsole.KeyboardFunc( key );
      }
      else {
        //do your own thing with the keys
      }
      break;
  }
}

void init_glconsole(){
  int argc = 0;
  const char * argv[] = {};
  //Initialise GLUT
  glutInit(&argc,(char **)argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowPosition (800, 50);
  glutInitWindowSize (600, 200);
  glutCreateWindow("Locomotion Console");
//  glutHideWindow();

//   standard GL init
//  glShadeModel(GL_SMOOTH);
//  glClearColor(0.0f, 0.0f, 1.0f, 0.5f);
//  glClearDepth(1.0f);
//  glEnable(GL_DEPTH_TEST);
//  glDepthFunc(GL_LEQUAL);
//  glEnable ( GL_COLOR_MATERIAL );
//  glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

  glutReshapeFunc (reshape);
  glutDisplayFunc (display);
  glutKeyboardFunc (keyfunc);
  glutSpecialFunc (special);
  glutIdleFunc(idle);

  glutMainLoop();
}
#endif
