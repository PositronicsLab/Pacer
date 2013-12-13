#include <quadruped_control.h>

typedef Ravelin::MatrixNd Mat;
typedef Ravelin::VectorNd Vec;

using namespace Ravelin;
using namespace Moby;

void determine_N_D(std::vector<ContactData>& contacts, Mat& N, Mat& D)
{
  int nc = contacts.size();

  for(unsigned i=0;i<links_.size();i++)
    for(unsigned j=0;j<nc;j++)
      if(contacts[j].name.compare(links_[i]->id) == 0)
        eefs_[j] = links_[i];

  static Mat J, Jsub;
  static Vec col;

  // resize temporary N and ST
  N.resize(NJOINT+6,nc);
  D.resize(NJOINT+6,nc*nk);

  J.resize(NSPATIAL, NJOINT);

  // loop over all contacts
  for(int i = 0; i < nc; i++){
    // get the contact point and normal
    ContactData& c = contacts[i];

    // generate the contact tangents here...
    Vector3d tan1, tan2;
    Vector3d::determine_orthonormal_basis(c.normal, tan1, tan2);
    Vector3d torque;
    torque.set_zero();

    RigidBodyPtr sbfoot = eefs_[i];

    Vec col(NSPATIAL);
    AAngled aa(0,0,1,0);
    Origin3d o(c.point);
    boost::shared_ptr<const Ravelin::Pose3d> pose(new Pose3d(aa,o));
    SForced sfn(c.normal,torque,pose);
    abrobot->convert_to_generalized_force(sbfoot,sfn, c.point, col);
    N.set_column(i,col);
    for(int k=0;k<nk;k++){
      if(k%2 == 0) {
        SForced sfs(tan1,torque,pose);
        abrobot->convert_to_generalized_force(sbfoot,sfs, c.point, col);
      } else {
        SForced sft(tan2,torque,pose);
        abrobot->convert_to_generalized_force(sbfoot,sft, c.point, col);
      }
      if(k>=2) col.negate();
      D.set_column(i*nk + k,col);
    }
  }
}

void determine_N_D2(std::vector<ContactData>& contacts, Mat& N, Mat& ST)
{
  int nc = contacts.size(),nk = 2;

  for(unsigned i=0;i<links_.size();i++)
    for(unsigned j=0;j<nc;j++)
      if(contacts[j].name.compare(links_[i]->id) == 0)
        eefs_[j] = links_[i];

  static Mat J, Jsub;
  static Vec col;

  // resize temporary N and ST
  N.resize(NJOINT+6,nc);
  ST.resize(NJOINT+6,nc*nk);

  J.resize(NSPATIAL, NJOINT);

  // loop over all contacts
  for(int i = 0; i < nc; i++){
    // get the contact point and normal
    ContactData& c = contacts[i];

    // generate the contact tangents here...
    Vector3d tan1, tan2;
    Vector3d::determine_orthonormal_basis(c.normal, tan1, tan2);
    Vector3d torque;
    torque.set_zero();

    RigidBodyPtr sbfoot = eefs_[i];

    Vec col(NSPATIAL);
    AAngled aa(0,0,1,0);
    Origin3d o(c.point);
    boost::shared_ptr<const Ravelin::Pose3d> pose(new Pose3d(aa,o));
    SForced sfn(c.normal,torque,pose);
    abrobot->convert_to_generalized_force(sbfoot,sfn, c.point, col);
    N.set_column(i,col);

    SForced sfs(tan1,torque,pose);
    abrobot->convert_to_generalized_force(sbfoot,sfs, c.point, col);
    ST.set_column(i,col);

    SForced sft = SForced(tan2,torque,pose);
    abrobot->convert_to_generalized_force(sbfoot,sft, c.point, col);
    ST.set_column(i+nc,col);
  }
}

void lf(const double X[3],double th[3]);
void rf(const double X[3],double th[3]);
void lh(const double X[3],double th[3]);
void rh(const double X[3],double th[3]);

// inverse kinematics solver conversion

std::vector<Ravelin::Vector3d>& trajectoryIK(
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
