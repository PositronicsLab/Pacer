#include <quadruped.h>

typedef Ravelin::MatrixNd Mat;
typedef Ravelin::VectorNd Vec;

using namespace Ravelin;
using namespace Moby;

void lf(const double X[3],double th[3]);
void rf(const double X[3],double th[3]);
void lh(const double X[3],double th[3]);
void rh(const double X[3],double th[3]);

// inverse kinematics solver conversion

std::vector<std::vector<Ravelin::Vector3d> >& trajectoryIK(
        const std::vector<std::vector<Ravelin::Vector3d> >& feet_positions,
        std::vector<std::vector<Ravelin::Vector3d> >& joint_positions){

  int num_steps = joint_positions.size();
  int num_feet = feet_positions.size();
    for(int i=0;i<num_steps;i++){
        joint_positions[i].resize(num_feet);
        lf(feet_positions[0][i].data(),joint_positions[i][0].data());
        rf(feet_positions[1][i].data(),joint_positions[i][1].data());
        lh(feet_positions[2][i].data(),joint_positions[i][2].data());
        rh(feet_positions[3][i].data(),joint_positions[i][3].data());
    }

}

std::vector<Ravelin::Vector3d>& feetIK(
        const std::vector<Ravelin::Vector3d>& feet_positions,
        std::vector<Ravelin::Vector3d>& joint_positions){
  int num_feet = feet_positions.size();
      joint_positions.resize(num_feet);
      lf(feet_positions[0].data(),joint_positions[0].data());
      rf(feet_positions[1].data(),joint_positions[1].data());
      lh(feet_positions[2].data(),joint_positions[2].data());
      rh(feet_positions[3].data(),joint_positions[3].data());

      return joint_positions;
}


/* Use this to convert from MATHEMATICA
:%s/List(/   /g
:%s/)))),/)));/g
:%s/Power(z,2))))/Power(z,2));/g
:%s/Rule(th1,/th[0] = /g
:%s/Rule(th2,/th[1] = /g
:%s/Rule(th3,/th[2] = /g
:%s/Power/pow/g
:%s/Sqrt/sqrt/g
:%s/ArcCos/acos/g
:%s/x/X[0]/gI
:%s/y/X[1]/g
:%s/z/X[2]/g
*/
