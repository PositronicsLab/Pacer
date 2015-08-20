/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#ifndef PROJECT_COMMON_H
#define PROJECT_COMMON_H

#include <boost/shared_ptr.hpp>

#include <Ravelin/MatrixNd.h>
#include <Ravelin/Transform3d.h>
#include <Ravelin/VectorNd.h>
#include <Ravelin/SVector6d.h>
#include <Ravelin/Vector3d.h>
#include <Ravelin/Vector2d.h>
#include <Ravelin/LinAlgd.h>
#include <Ravelin/AAngled.h>
#include <Ravelin/SForced.h>
#include <Ravelin/Pose3d.h>

// Dynamics
#include <Ravelin/RigidBodyd.h>
#include <Ravelin/DynamicBodyd.h>
#include <Ravelin/RCArticulatedBodyd.h>

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
#include <Pacer/utilities.h>

#include <boost/assign/std/vector.hpp>
#include <boost/assign/list_of.hpp>

#ifdef TIMING
#include <ctime>
#endif

const double grav     = 9.81; // m / s*s
const double M_PI_8   = 0.39269908169;
const double M_PI_16  = 0.19634954084;

#include <Pacer/Log.h>

static Ravelin::LinAlgd LA_;
#endif // PROJECT_COMMON_H
