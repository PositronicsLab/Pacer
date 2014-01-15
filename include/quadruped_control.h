#ifndef CONTROL_H
#define CONTROL_H

#include <project_common.h>

typedef Ravelin::MatrixNd Mat;
typedef Ravelin::VectorNd Vec;

double friction_estimation(const Vec& v, const Vec& fext,
                           double dt, const Mat& N,
                           const Mat& ST, const Mat& M,
                           Mat& MU, Vec& cf);

void visualize_contact( const Moby::Event& e,
                        boost::shared_ptr<Moby::EventDrivenSimulator> sim );

void visualize_polygon( const Mat& verts,
                        boost::shared_ptr<Moby::EventDrivenSimulator> sim );

void visualize_ray(   const Ravelin::Vector3d& point, const Ravelin::Vector3d& vec, boost::shared_ptr<Moby::EventDrivenSimulator> sim ) ;

void inverse_dynamics(const Vec& v, const Vec& qdd, const Mat& M,
                      const  Mat& N, const Mat& ST, const Vec& fext,
                      double h, const Mat& MU, Vec& uff);

std::vector<Ravelin::Vector3d>& foot_oscilator(
  const std::vector<Ravelin::Vector3d>& x0,  const std::vector<Ravelin::Vector3d>& x, const Mat& C,
  double Ls,const Vec& Hs,double Df,double Vf,double bp,std::vector<Ravelin::Vector3d>& xd);

std::vector<Ravelin::Vector3d>& stepTrajectory(
    const std::vector<Ravelin::Vector3d>& control_points,
    int num_segments, std::vector<Ravelin::Vector3d>& trajectory);

/// 4 foot (body-fixed) state-space traj to joint-space traj
std::vector<std::vector<Ravelin::Vector3d> > &trajectoryIK(
        const std::vector<std::vector<Ravelin::Vector3d> >& feet_positions,
        std::vector<std::vector<Ravelin::Vector3d> >& joint_positions);

std::vector<Ravelin::Vector3d>& feetIK(
        const std::vector<Ravelin::Vector3d>& feet_positions,
        std::vector<Ravelin::Vector3d>& joint_positions);

class EndEffector
{
public:
  EndEffector(){
    init();
  }
  EndEffector(Moby::RigidBodyPtr l,Ravelin::Vector3d o){
    link = l;
    id = link->id;
    origin = o;
    init();
  }

  // permanent data
  std::string           id;
  Ravelin::Vector3d     origin;
  // Rigid Body (moby data)
  Moby::RigidBodyPtr    link;
  // kinematic chain indexing data
  std::vector<unsigned> chain;
  // Contact Data

  Ravelin::Vector3d     point;
  Ravelin::Vector3d     normal; //(pointing away from the ground)
  Moby::SingleBodyPtr   contacted_body;
  bool                  active;

private:
  void init();
};

//void determine_contact_jacobians(std::vector<ContactData>& contacts,
//                                 Mat& N, Mat& D, Mat& ST);
//void determine_N_D(std::vector<ContactData>& contacts, Mat& N, Mat& D);
//void determine_N_ST(std::vector<ContactData>& contacts, Mat& N, Mat& ST);

void get_q_qd_qdd(const std::vector<std::vector<Ravelin::Vector3d> >& joint_trajectory, unsigned TIME, std::map<std::string, double>& q_des,std::map<std::string, double>& qd_des,std::map<std::string, double>& qdd_des);
void get_q_qd_qdd(const std::vector<Ravelin::Vector3d>& joint_position, std::map<std::string, double>& q_des,std::map<std::string, double>& qd_des,std::map<std::string, double>& qdd_des);

using boost::shared_ptr;
using boost::dynamic_pointer_cast;
using std::string;
using std::map;
using std::pair;
using std::vector;

extern unsigned NUM_EEFS,
                NUM_JOINTS,
                NUM_LINKS,
                NDOFS, // NDFOFS for forces, accel, & velocities
                NSPATIAL,
                NEULER,
                NK;

extern Moby::RCArticulatedBodyPtr abrobot;
extern Moby::DynamicBodyPtr dbrobot;
extern std::vector<Moby::JointPtr> joints_;
extern std::vector<Moby::RigidBodyPtr> links_;

#endif // CONTROL_H
