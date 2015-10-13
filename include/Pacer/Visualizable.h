#ifndef VISUALIZABLE_H
#define VISUALIZABLE_H

#include <Ravelin/Vector3d.h>
#include <Ravelin/Pose3d.h>
#include <vector>
#include <boost/shared_ptr.hpp>

namespace Pacer{

class Visualizable{
public:
  typedef int viz_Type; 
  static const viz_Type 
    eUnknown = 0,
    ePoint = 1,
    eRay = 2,
    ePose = 3;
  viz_Type eType;
  double size;
  Ravelin::Vector3d color;
};

class Ray : public Visualizable{
public:
  Ravelin::Vector3d point1,point2;
  Ray(Ravelin::Vector3d p1,Ravelin::Vector3d p2,Ravelin::Vector3d c = Ravelin::Vector3d(1.0,1.0,1.0),double s = 0.001){
    eType = eRay;
    size = s;
    point2 = p2;
    point1 = p1;
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
  Pose(const Ravelin::Pose3d& p,double sd = 0.5,double s = 0.001){
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
