#include <Log.h>
#include <math.h>
#include <stdio.h>
#include <iostream>

void check_vals(double th[3]){
  for(int i=0;i<3;i++){
    if(!isfinite(th[i])){
      th[i] = 0;
      OUT_LOG(logDEBUG4) << "Inverse kinematics solution was NaN, setting to X[2]ero!" << std::endl;
    }
  }
}

void ik(const double X[3],double th[3]){
  // x = z
  // y = y
  // z = -x

  //r = sqrt(x^2 + y^2 + z^2)
  th[0] = sqrt(X[0]*X[0] + X[1]*X[1] + X[2]*X[2]);
  //theta = acos(z/r)
  th[1] = acos(X[0]/th[0]);
//  th[1] = atan2(X[1],-X[0]);
  // Phi  = atan(y/x)
  th[2] = atan2(X[1],-X[2]);
}

