#ifndef UTILITIES_H
#define UTILITIES_H

#include <project_common.h>

static bool R2rpy(const Ravelin::Matrix3d& R, Ravelin::Vector3d& rpy){
  if(R(0,0) == 0 || R(2,2) == 0)
    return false;

  rpy[0] = atan2(R(1,0),R(0,0));
  rpy[1] = atan2(-R(2,0),sqrt(R(2,1)*R(2,1) + R(2,2)*R(2,2)));
  rpy[2] = atan2(R(2,1),R(2,2));
  return true;
}

#endif // UTILITIES_H
