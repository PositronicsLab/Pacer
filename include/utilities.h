#ifndef UTILITIES_H
#define UTILITIES_H

#include <project_common.h>

Ravelin::Vector3d& R2rpy(const Ravelin::Matrix3d& R, Ravelin::Vector3d& rpy);
Ravelin::Vector3d& quat2rpy(const Ravelin::Quatd& q, Ravelin::Vector3d& rpy);
Ravelin::Quatd& aa2quat(const Ravelin::VectorNd& aa,Ravelin::Quatd& q);
Ravelin::Matrix3d& Rz(double x,Ravelin::Matrix3d& R);
Ravelin::Matrix3d& Ry(double x,Ravelin::Matrix3d& R);
Ravelin::Matrix3d& Rx(double x,Ravelin::Matrix3d& R);

/// Calculates The null pace for matrix M and places it in Vk
/// returns the number of columns in Vk
unsigned kernal( Ravelin::MatrixNd& M,Ravelin::MatrixNd& Vk);
void check_finite(Ravelin::VectorNd& v);

double distance_from_plane(const Ravelin::Vector3d& normal,const Ravelin::Vector3d& point, const Ravelin::Vector3d& x);

/// 4 Control Point Bezier curve EEF calculation
// Calculate the next bezier point.
void evalBernstein(const Ravelin::Vector3d& A, const Ravelin::Vector3d& B, const Ravelin::Vector3d& C, const Ravelin::Vector3d& D, double t,
                   Ravelin::Vector3d& P,Ravelin::Vector3d& dP,Ravelin::Vector3d& ddP);

void evalBernstein(const Ravelin::Vector3d& A, const Ravelin::Vector3d& B, const Ravelin::Vector3d& C, const Ravelin::Vector3d& D, double t,Ravelin::Vector3d& P);
Ravelin::Vector3d logistic(double x,double a, double b);
void bezierCurve(const std::vector<Ravelin::Vector3d>& control_points, int num_segments,
                 std::vector<Ravelin::Vector3d>& trajectory,
                 std::vector<Ravelin::Vector3d> & dtrajectory,
                 std::vector<Ravelin::Vector3d> & ddtrajectory);

Ravelin::Vector3d slerp( const Ravelin::Vector3d& v0,const Ravelin::Vector3d& v1,double t);
Ravelin::Vector3d lerp( const Ravelin::Vector3d& v0,const Ravelin::Vector3d& v1,double t);

void calc_cubic_spline_coefs(const Ravelin::VectorNd& T,const Ravelin::VectorNd& X,
                                           const Ravelin::Vector2d& Xd,const Ravelin::Vector2d& Xdd,
                                           Ravelin::VectorNd& B);
  void eval_cubic_spline(const Ravelin::VectorNd& coefs,const Ravelin::VectorNd& t_limits,int num_segments,
                         Ravelin::VectorNd& X, Ravelin::VectorNd& Xd, Ravelin::VectorNd& Xdd);

  double linesearch(double (*f)(const Ravelin::VectorNd& x, Ravelin::VectorNd& fk, Ravelin::VectorNd& gk) ,
                    const Ravelin::VectorNd& d,const Ravelin::VectorNd& x,double rho = 0.5,double c = 1e-4);

#endif // UTILITIES_H
