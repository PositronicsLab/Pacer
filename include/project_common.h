#ifndef PROJECT_COMMON_H
#define PROJECT_COMMON_H

#include <boost/shared_ptr.hpp>

#include <Ravelin/MatrixNd.h>
#include <Ravelin/VectorNd.h>
#include <Ravelin/Vector3d.h>
#include <Ravelin/Vector2d.h>
#include <Ravelin/LinAlgd.h>
#include <Ravelin/AAngled.h>
#include <Ravelin/SForced.h>
#include <Ravelin/Pose3d.h>

#include <Moby/Simulator.h>
#include <Moby/EventDrivenSimulator.h>
#include <Moby/RCArticulatedBody.h>
#include <Moby/DynamicBody.h>
#include <Moby/RNEAlgorithm.h>
#include <Moby/LCP.h>
#include <math.h>

#include <cmath>
#include <sys/types.h>
#include <sys/times.h>
#include <fstream>
#include <string>
#include <iostream>     // std::cout, std::fixed
#include <iomanip>      // std::setprecision
#include <math.h>

void outlog(const Ravelin::VectorNd& M, std::string name);
void outlog(const Ravelin::SharedVectorNd& M, std::string name);
void outlog(const Ravelin::MatrixNd& z, std::string name);

#include <Log.h>

#endif // PROJECT_COMMON_H
