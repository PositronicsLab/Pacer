#ifndef UTILITIES_H
#define UTILITIES_H

#include <project_common.h>

static bool R2rpy(const Ravelin::Matrix3d& R, Ravelin::Vector3d& rpy){
  if(R(0,0) == 0 || R(2,2) == 0)
    return false;

  rpy[2] = atan2(R(1,0),R(0,0));
  rpy[1] = atan2(-R(2,0),sqrt(R(2,1)*R(2,1) + R(2,2)*R(2,2)));
  rpy[0] = atan2(R(2,1),R(2,2));
  return true;
}

static bool quat2rpy(const Ravelin::Quatd& q, Ravelin::Vector3d& rpy){
  rpy[0] = atan2(2.0*(q[0]*q[1] + q[2]*q[3]),1.0 - 2.0*(q[1]*q[1] + q[2]*q[2]));
  rpy[1] = asin(2.0*(q[0]*q[2] - q[3]*q[1]));
  rpy[2] = atan2(2.0*(q[0]*q[3] + q[1]*q[2]),1.0 - 2.0*(q[2]*q[2] + q[3]*q[3]));
  return true;
}
static bool aa2quat(const Ravelin::VectorNd& aa,Ravelin::Quatd& q){
  q[0] = cos(aa[3]*0.5);
  q[1] = sin(aa[3]*0.5)*cos(aa[0]);
  q[2] = sin(aa[3]*0.5)*cos(aa[1]);
  q[3] = sin(aa[3]*0.5)*cos(aa[2]);
  return true;
}
static bool Rz(double x,Ravelin::Matrix3d& R){
  R = Ravelin::Matrix3d( cos(x), sin(x), 0,
                        -sin(x), cos(x), 0,
                              0,      0, 1);
}

static bool Ry(double x,Ravelin::Matrix3d& R){
  R = Ravelin::Matrix3d( cos(x), 0,-sin(x),
                              0, 1,      0,
                         sin(x), 0, cos(x));
}

static bool Rx(double x,Ravelin::Matrix3d& R){
  R = Ravelin::Matrix3d(1,      0,      0,
                        0, cos(x), sin(x),
                        0,-sin(x), cos(x));
}

/// Calculates The null pace for matrix M and places it in Vk
/// returns the number of columns in Vk
static unsigned kernal( Ravelin::MatrixNd& M,Ravelin::MatrixNd& Vk){
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
    Vk.set_zero(V.rows(),size_null_space);
    V.get_sub_mat(0,V.rows(),V.columns()-size_null_space,V.columns(),Vk);
  }
  return size_null_space;
}

static void check_finite(Ravelin::VectorNd& v){
  for(int i=0;i<v.rows();i++)
    if(!isfinite(v[i]))
      v[i] = 0;
}

static double distance_from_plane(const Ravelin::Vector3d& normal,const Ravelin::Vector3d& point, const Ravelin::Vector3d& x){
  static Ravelin::Vector3d v;
  v = x - point;
  return normal.dot(v);
}

#endif // UTILITIES_H
