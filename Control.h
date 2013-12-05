#ifndef CONTROL_H
#define CONTROL_H

#include <cmath>
#include <sys/types.h>
#include <sys/times.h>
#include <fstream>
#include <iostream>     // std::cout, std::fixed
#include <iomanip>      // std::setprecision

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

typedef Ravelin::MatrixNd Mat;
typedef Ravelin::VectorNd Vec;

void outlog(const Mat& M, std::string name);
void outlog(const Vec& z, std::string name);
void outlog2(const Vec& M, std::string name);
void outlog2(const Ravelin::SharedVectorNd& M, std::string name);
void outlog2(const Mat& z, std::string name);

double friction_estimation(const Vec& v, const Vec& fext,
                           double dt, const Mat& N,
                           const Mat& ST, const Mat& M,
                           Mat& MU, Vec& cf);

void visualize_contact( const Moby::Event& e,
                        boost::shared_ptr<Moby::EventDrivenSimulator> sim );
void visualize_polygon( const Mat& verts,
                        boost::shared_ptr<Moby::EventDrivenSimulator> sim );
void visualize_ray( const Ravelin::Vector3d& point, const Ravelin::Vector3d& vec, boost::shared_ptr<Moby::EventDrivenSimulator> sim ) ;

void idyn(const Vec& v, const Vec& qdd, const Mat& M,
          const  Mat& N, const Mat& ST, const Vec& fext,
          double h, const Mat& MU, Vec& uff);

std::vector<Ravelin::Vector3d>& stepTrajectory(const std::vector<Ravelin::Vector3d>& control_points, std::vector<Ravelin::Vector3d>& trajectory);

/// 4 foot (body-fixed) state-space traj to joint-space traj
std::vector<Ravelin::Vector3d>& trajectoryIK(
        const std::vector<std::vector<Ravelin::Vector3d> >& feet_positions,
        std::vector<std::vector<Ravelin::Vector3d> >& joint_positions);

struct ContactData
{
  Ravelin::Vector3d point;  // contact point
  Ravelin::Vector3d normal; // contact normal (pointing away from the ground)
  std::string name;
};
void determine_N_D(std::vector<ContactData>& contacts, Mat& N, Mat& D);
void determine_N_D2(std::vector<ContactData>& contacts, Mat& N, Mat& ST);

using boost::shared_ptr;
using boost::dynamic_pointer_cast;
using std::string;
using std::map;
using std::pair;
using std::vector;

enum RobotDOFs {
  BODY_JOINT = 0,
  LF_HFE = 3,
  LF_HAA = 7,
  LF_KFE = 11,

  LH_HFE = 1,
  LH_HAA = 5,
  LH_KFE = 9,

  RF_HFE = 4,
  RF_HAA = 8,
  RF_KFE = 12,

  RH_HFE = 2,
  RH_HAA = 6,
  RH_KFE = 10,

  NJOINT = 13
};

const unsigned NUM_EEFS = 4,
               N_JOINTS = NJOINT,
               NDOFS = N_JOINTS+6, // NDFOFS for forces, accel, & velocities
               N_FIXED_JOINTS = 4,
               NSPATIAL = 6,
               nk = 4;

// robot pointer
extern Moby::RCArticulatedBodyPtr abrobot;
extern Moby::DynamicBodyPtr dbrobot;
extern std::vector<Moby::JointPtr> joints_;
extern std::vector<Moby::RigidBodyPtr> links_,eefs_;

#endif // CONTROL_H
