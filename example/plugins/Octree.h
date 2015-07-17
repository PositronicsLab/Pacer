//
//  Octree.h
//  
//
//  Created by samzapo on 7/14/15.
//
//

#ifndef _Octree_h
#define _Octree_h
class Object {
  
};

class Node {
  virtual Node(){
    if(stopDepth < 0)
  }
  virtual ~Node(){}
  
private:
  double center[3];
  double radius;
  std::vector< boost::weak_ptr<Node> > children;
  std::vector< boost::shared_ptr<Object> > objects;
};




#endif
