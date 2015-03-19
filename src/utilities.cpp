/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/project_common.h>
#include <Pacer/utilities.h>

static Ravelin::VectorNd workv_;
static Ravelin::Vector3d workv3_;
static Ravelin::MatrixNd workM_;

    //static std::vector<Pacer::VisualizablePtr> visualize;
std::vector<Pacer::VisualizablePtr> Utility::visualize;

/// Calculates The null pace for matrix M and places it in Vk
/// returns the number of columns in Vk
unsigned Utility::kernal( Ravelin::MatrixNd& M,Ravelin::MatrixNd& null_M){
  unsigned size_null_space = 0;
  Ravelin::MatrixNd U,V;
  Ravelin::VectorNd S;

  // SVD decomp to retrieve nullspace of Z'AZ
  LA_.svd(M,U,S,V);
  if(S.rows() != 0){
    // Calculate the tolerance for ruling that a singular value is supposed to be zero
    double ZERO_TOL = std::numeric_limits<double>::epsilon() * M.rows() * S[0];
    // Count Zero singular values
    for(int i = S.rows()-1;i>=0;i--,size_null_space++)
      if(S[i] > ZERO_TOL)
        break;
    // get the nullspace
    null_M.set_zero(V.rows(),size_null_space);
    V.get_sub_mat(0,V.rows(),V.columns()-size_null_space,V.columns(),null_M);
  }
  return size_null_space;
}

void Utility::check_finite(Ravelin::VectorNd& v){
  for(int i=0;i<v.rows();i++)
    if(!std::isfinite(v[i]))
      v[i] = 0;
}

double Utility::distance_from_plane(const Ravelin::Vector3d& normal,const Ravelin::Vector3d& point, const Ravelin::Vector3d& x){
  Ravelin::Vector3d v;
  v = x - point;
  return normal.dot(v);
}

/// 4 Control Point Bezier curve EEF calculation
// Calculate the next bezier point.

Ravelin::Vector3d Utility::logistic(double x,double a, double b){
  return Ravelin::Vector3d( 1/(1+exp(-a*(x-b))),
                           (a*exp(a*(b+x)))/((exp(a*b)+exp(a*x))*(exp(a*b)+exp(a*x))),
                           (a*a*exp(a*(b+x))*(exp(a*b)-exp(a*x)))/((exp(a*b)+exp(a*x))*(exp(a*b)+exp(a*x))*(exp(a*b)+exp(a*x)))
                            );
}

Ravelin::Vector3d Utility::slerp( const Ravelin::Vector3d& v0,const Ravelin::Vector3d& v1,double t){
  double a = v0.dot(v1);
  return (v0*sin((1-t)*a)/sin(a) + v1*sin(t*a)/sin(a));
}

Ravelin::Vector3d Utility::lerp( const Ravelin::Vector3d& v0,const Ravelin::Vector3d& v1,double t){
  return (v0*(1-t) + v1*t);
}

void Utility::solve(Ravelin::MatrixNd& M,Ravelin::VectorNd& bx){
  if(M.rows() == M.columns())
    LA_.solve_fast(M,bx);
  else{
    LA_.pseudo_invert(workM_ = M);
    workM_.mult(workv_ = bx,bx);
  }
}
