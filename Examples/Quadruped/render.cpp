// viewer.cpp
#include <osgDB/WriteFile>
#include <osg/CameraNode>
#include <osgViewer/Viewer>
#include <osg/Node>
#include <osg/PositionAttitudeTransform>
#include <osgDB/ReadFile>
#include <osgGA/TrackballManipulator>


class SnapImageDrawCallback : public ::osg::CameraNode::DrawCallback
{
       public:
       SnapImageDrawCallback()
       {
       _snapImageOnNextFrame = false;
       }

       void setFileName(const std::string& filename) { _filename = filename; }
       const std::string& getFileName() const { return _filename; }
       void setSnapImageOnNextFrame(bool flag) { _snapImageOnNextFrame = flag; }
       bool getSnapImageOnNextFrame() const { return _snapImageOnNextFrame; }
       virtual void operator () (const ::osg::CameraNode& camera) const
       {
               ::osg::notify(::osg::NOTICE) << "Saving screen image to '"<<_filename<<"'"<< std::endl;
               if (!_snapImageOnNextFrame) return;
               int x,y,width,height;
               x = camera.getViewport()->x();
               y = camera.getViewport()->y();
               width = camera.getViewport()->width();
               height = camera.getViewport()->height();
               ::osg::ref_ptr< ::osg::Image> image = new ::osg::Image();
               image->readPixels(x,y,width,height,GL_RGB,GL_UNSIGNED_BYTE);

               if (::osgDB::writeImageFile(*image,_filename))

               {
                       std::cout << "Saved screen image to '"<<_filename<<"'"<< std::endl;
               }
               _snapImageOnNextFrame = false;
       }

       protected:

       ::std::string _filename;
       mutable bool _snapImageOnNextFrame;

};


void
renderSceneToImage(::osg::Node* pRoot, const ::std::string& sFileName_)
{
       int nWidth = 640, nHeight = 480;

       ::osgViewer::Viewer * viewer = new ::osgViewer::Viewer();
       viewer->setSceneData(pRoot);
       viewer->getCamera()->setRenderTargetImplementation(::osg::CameraNode::FRAME_BUFFER_OBJECT);

       ::osg::ref_ptr<SnapImageDrawCallback> snapImageDrawCallback = new SnapImageDrawCallback();
       viewer->getCamera()->setPostDrawCallback (snapImageDrawCallback.get());

       snapImageDrawCallback->setFileName(sFileName_);
       snapImageDrawCallback->setSnapImageOnNextFrame(true);


       ::osg::ref_ptr< ::osg::GraphicsContext> pbuffer;

       ::osg::ref_ptr< ::osg::GraphicsContext::Traits> traits = new ::osg::GraphicsContext::Traits;
       traits->x = 0;
       traits->y = 0;
       traits->width = nWidth;
       traits->height = nHeight;
       traits->red = 8;
       traits->green = 8;
       traits->blue = 8;
       traits->alpha = 8;
       traits->windowDecoration = false;
       traits->pbuffer = true;
       traits->doubleBuffer = true;
       traits->sharedContext = 0;

       pbuffer = ::osg::GraphicsContext::createGraphicsContext(traits.get());
       if (pbuffer.valid())
       {
               ::osg::ref_ptr< ::osg::Camera> camera2 = new ::osg::Camera();
               camera2->setGraphicsContext(pbuffer.get());
               GLenum buffer = pbuffer->getTraits()->doubleBuffer ? GL_BACK : GL_FRONT;
               camera2->setDrawBuffer(buffer);
               camera2->setReadBuffer(buffer);
               viewer->addSlave(camera2.get(), ::osg::Matrixd(), ::osg::Matrixd());
       }

       viewer->realize();
       viewer->frame();
}

void render(::osg::Node* node, const ::std::string& sFileName_){
  // Declare a node which will serve as the root node
  // for the scene graph. Since we will be adding nodes
  // as 'children' of this node we need to make it a 'group'
  // instance.
  // The 'node' class represents the most generic version of nodes.
  // This includes nodes that do not have children (leaf nodes.)
  // The 'group' class is a specialized version of the node class.
  // It adds functions associated with adding and manipulating
  // children.

  osg::Group* root = new osg::Group();
  root->addChild(node);

  // Declare transform, initialize with defaults.

  osg::PositionAttitudeTransform* nodeXform =
     new osg::PositionAttitudeTransform();

  // Use the 'addChild' method of the osg::Group class to
  // add the transform as a child of the root node and the
  // node node as a child of the transform.

  root->addChild(nodeXform);

  nodeXform->addChild(node);

  // Declare and initialize a Vec3 instance to change the
  // position of the node model in the scene
  osg::Vec3 nodePosit(5,0,0);
  nodeXform->setPosition( nodePosit );

  // Declare a 'viewer'
  osgViewer::Viewer viewer;

  // Next we will need to assign the scene graph we created
  // above to this viewer:
  viewer.setSceneData( root );

  // attach a trackball manipulator to all user control of the view
  viewer.setCameraManipulator(new osgGA::TrackballManipulator);

  // create the windows and start the required threads.
  viewer.realize();

  // Enter the simulation loop. viewer.done() returns false
  // until the user presses the 'esc' key.
  // (This can be changed by adding your own keyboard/mouse
  // event handler or by changing the settings of the default
  // keyboard/mouse event handler)

  while( !viewer.done() )
  {
     // dispatch the new frame, this wraps up the follow Viewer operations:
     //   advance() to the new frame
     //   eventTraversal() that collects events and passes them on to the event handlers and event callbacks
     //   updateTraversal() to calls the update callbacks
     //   renderingTraversals() that runs syncronizes all the rendering threads (if any) and dispatch cull, draw and swap buffers
     viewer.frame();
  }
}

int main(int argc, char** argv)
{
  ::osg::Node* pRoot = osgDB::readNodeFile(argv[0]);
  std::string sFileName(argv[1]);
//  renderSceneToImage(pRoot,sFileName);
  render(pRoot,sFileName);
}
