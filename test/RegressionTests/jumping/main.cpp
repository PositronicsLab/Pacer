#include <Moby/TimeSteppingSimulator.h>
#include <Moby/RCArticulatedBody.h>
#include <Moby/ArticulatedBody.h>
#include <stdlib.h>
#include <Ravelin/RCArticulatedBodyd.h>
#include <Ravelin/ArticulatedBodyd.h>
#include <Ravelin/Jointd.h>
#include <Ravelin/Transform3d.h>
#include <Ravelin/Vector3d.h>
#include <boost/shared_ptr.hpp>
#include <set>
#include <Moby/ControlledBody.h>
#include <Pacer/utilities.h>
#include <Pacer/controller.h>

using Moby::Simulator;
using Ravelin::RigidBodyd;
using Ravelin::Jointd;
using Ravelin::RCArticulatedBodyd;
using Ravelin::Pose3d;
using Ravelin::DynamicBodyd;

using namespace Pacer;

Ravelin::VectorNd q_f,qd_f;
Ravelin::VectorNd q_0,qd_0;
// Check functions
#include "checks.cpp"

#define TEST_TYPE RegressionTest
#define TEST_NAME Walking

#include "../main.cpp"

#undef TEST_TYPE
#undef TEST_NAME
