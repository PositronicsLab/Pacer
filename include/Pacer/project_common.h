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
#include <Pacer/utilities.h>

#include <boost/assign/std/vector.hpp>
#include <boost/assign/list_of.hpp>

#include <CVars/CVar.h>

#ifdef TIMING
#include <ctime>
#endif

// ----------------------------------------------------------------------------

const double grav     = 9.81; // M/s.s
const double M_PI_8   = 0.39269908169;
const double M_PI_16  = 0.19634954084;

// ----------------------------------------------------------------------------
#include <Pacer/Log.h>
 void OUTLOG(const Ravelin::VectorNd& M, std::string name,TLogLevel LL);
 void OUTLOG(const Ravelin::SharedVectorNd& M, std::string name,TLogLevel LL);
 void OUTLOG(const Ravelin::MatrixNd& z, std::string name,TLogLevel LL);
 void OUTLOG(const Ravelin::SharedConstMatrixNd& z, std::string name,TLogLevel LL);
 void OUTLOG(const Ravelin::Matrix3d& z, std::string name,TLogLevel LL);
 void OUTLOG(const Ravelin::Pose3d& P, std::string name,TLogLevel LL);
 void OUTLOG(const Ravelin::Origin3d& z, std::string name,TLogLevel LL);
 void OUTLOG(const Ravelin::Vector3d& z, std::string name,TLogLevel LL);
 void OUTLOG(const Ravelin::Vector2d& z, std::string name,TLogLevel LL);
 void OUTLOG(const Ravelin::SVector6d& z, std::string name,TLogLevel LL);
 void OUTLOG(const Ravelin::AAngled& z, std::string name,TLogLevel LL);
 void OUTLOG(const std::vector<double>& z, std::string name,TLogLevel LL);
 void OUTLOG(const std::vector<int>& z, std::string name,TLogLevel LL);
 void OUTLOG(double x, std::string name,TLogLevel LL);
 void OUTLOG(const std::vector<std::string>& z, std::string name,TLogLevel LL);
extern std::string LOG_TYPE;
// ----------------------------------------------------------------------------

// STATIC declarations to be used in each src file

static Ravelin::VectorNd workv_;
static Ravelin::Vector3d workv3_;
static Ravelin::MatrixNd workM_;
static Ravelin::LinAlgd LA_;
#endif // PROJECT_COMMON_H
