#ifndef VISUALIZABLE_H
#define VISUALIZABLE_H

#include <Ravelin/Vector3d.h>
#include <Ravelin/Pose3d.h>
#include <vector>
#include <boost/shared_ptr.hpp>


#ifdef USE_DISPLAY
#define VISUALIZE(x) \
Utility::visualize.push_back \
(Pacer::VisualizablePtr \
(x))
#else
#define VISUALIZE(x) if(0){}
#endif
#define RAY(p,t,c,s) \
new Pacer::Ray(p,t,c,s)

#define POINT(p,c,s) \
new Pacer::Point(p,c,s)

#define POSE(p,s) \
new Pacer::Pose(p,s)

#define TEXT(t,p,q,c,s) \
new Pacer::Text(t,p,q,c,s)


namespace Pacer{

class Visualizable : public boost::enable_shared_from_this<Visualizable>{
public:
  boost::shared_ptr<Visualizable> ptr(){
    return shared_from_this();
  }

  enum Type{
    eUnknown = 0,
    ePoint = 1,
    eRay = 2,
    ePose = 3,
    eText = 4
  } eType;
  double size;
  Ravelin::Vector3d color;
};

class Ray : public Visualizable{
public:
  Ravelin::Vector3d point1,point2;
  Ray(Ravelin::Vector3d p1,Ravelin::Vector3d p2,Ravelin::Vector3d c = Ravelin::Vector3d(1.0,1.0,1.0),double s = 0.01){
    eType = eRay;
    size = s;
    point2 = p2;
    point1 = p1;
    color = c;
  }
};

  class Text : public Visualizable{
  public:
    Ravelin::Vector3d point;
    Ravelin::Quatd quat;
    std::string text_string;
    Text(std::string t, Ravelin::Vector3d p,Ravelin::Quatd q, Ravelin::Vector3d c = Ravelin::Vector3d(1.0,1.0,1.0),double s = 24){
      eType = eText;
      text_string = t;
      size = s;
      point = p;
      quat = q;
      color = c;
    }
  };

  
class Point : public Visualizable{
public:
  Ravelin::Vector3d point;
  Point(Ravelin::Vector3d p,Ravelin::Vector3d c = Ravelin::Vector3d(1.0,1.0,1.0),double s = 0.01){
    eType = ePoint;
    size = s;
    point = p;
    color = c;
  }
};

class Pose : public Visualizable{
public:
  double shade;
  Ravelin::Pose3d pose;
  Pose(const Ravelin::Pose3d& p,double sd = 0.5,double s = 0.1){
    pose = Ravelin::Pose3d();
    eType = ePose;
    shade = sd;
    size = s;
    pose = p;
  }
};

typedef boost::shared_ptr<Pacer::Visualizable> VisualizablePtr;

}
#endif // VISUALIZABLE_H
