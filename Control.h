#ifndef CONTROL_H
#define CONTROL_H

#include <cmath>
#include <sys/types.h>
#include <sys/times.h>
#include <fstream>

#include <boost/shared_ptr.hpp>
#include <Ravelin/MatrixNd.h>
#include <Ravelin/VectorNd.h>
#include <Ravelin/Vector3d.h>
#include <Ravelin/LinAlgd.h>
#include <Ravelin/AAngled.h>
#include <Ravelin/SForced.h>

#include <Moby/Simulator.h>
#include <Moby/EventDrivenSimulator.h>
#include <Moby/RCArticulatedBody.h>
#include <Moby/DeformableBody.h>
#include <Moby/DynamicBody.h>
#include <Moby/RevoluteJoint.h>
#include <Moby/GravityForce.h>
#include <Moby/Constants.h>
#include <Moby/RNEAlgorithm.h>
#include <Moby/LCP.h>
//#include <Opt/LCP.h>

typedef Ravelin::MatrixNd Mat;
typedef Ravelin::VectorNd Vec;

void outlog(const Mat& M, std::string name);
void outlog(const Vec& z, std::string name);
void outlog2(const Vec& M, std::string name);
void outlog2(const Ravelin::SharedVectorNd& M, std::string name);
void outlog2(const Mat& z, std::string name);
//double friction_estimation(const Vec& v, const Vec& fext,
//                           double dt, const Mat& N,
//                           const Mat& ST, const Mat& M,
//                           bool post_event, Mat& MU, Vec& cf);

void idyn(const Vec& v, const Vec& qdd, const Mat& M,
          const  Mat& N, const Mat& ST, const Vec& fext,
          double h, const Mat& MU, Vec& uff);


struct ContactData
{
  Ravelin::Vector3d point;  // contact point
  Ravelin::Vector3d normal; // contact normal (pointing away from the ground)
  std::string name;
};

using boost::shared_ptr;
using boost::dynamic_pointer_cast;
using std::string;
using std::map;
using std::pair;
using std::vector;

enum RobotDOFs {
  LF_HFE = 0,
  LF_HAA,
  LF_KFE,

  LH_HFE,
  LH_HAA,
  LH_KFE,

  RF_HFE,
  RF_HAA,
  RF_KFE,

  RH_HFE,
  RH_HAA,
  RH_KFE,

  NJOINT
};

const unsigned NUM_EEFS = 4,
               NDOFS = NJOINT+6,
               N_FIXED_JOINTS = 4,
               NSPATIAL = 6,
               nk = 4;

#endif // CONTROL_H
