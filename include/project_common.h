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
#include <numeric>

//#define NDEBUG
void OUTLOG(const Ravelin::VectorNd& M, std::string name);
void OUTLOG(const Ravelin::SharedVectorNd& M, std::string name);
void OUTLOG(const Ravelin::MatrixNd& z, std::string name);

const double grav = 9.8; // M/s.s
const double M_PI_16 = 0.19634954084;
const double M_PI_8 = 0.39269908169;

#include <Log.h>

#endif // PROJECT_COMMON_H
