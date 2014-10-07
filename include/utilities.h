#ifndef UTILITIES_H
#define UTILITIES_H

#include <project_common.h>

class Utility{
  public:
  static Ravelin::Vector3d& R2rpy(const Ravelin::Matrix3d& R, Ravelin::Vector3d& rpy);
  static Ravelin::Vector3d& quat2TaitBryanZ(const Ravelin::Quatd& q_, Ravelin::Vector3d& ypr);
  static Ravelin::Vector3d& quat2TaitBryanX(const Ravelin::Quatd& q_, Ravelin::Vector3d& ypr);
  static Ravelin::Vector3d& quat2rpy(const Ravelin::Quatd& q, Ravelin::Vector3d& rpy);
  static Ravelin::Quatd& aa2quat(const Ravelin::VectorNd& aa,Ravelin::Quatd& q);
  static Ravelin::Matrix3d& Rz(double x,Ravelin::Matrix3d& R);
  static Ravelin::Matrix3d& Ry(double x,Ravelin::Matrix3d& R);
  static Ravelin::Matrix3d& Rx(double x,Ravelin::Matrix3d& R);

  /// Calculates The null pace for matrix M and places it in Vk
  /// returns the number of columns in Vk
  static unsigned kernal( Ravelin::MatrixNd& M,Ravelin::MatrixNd& null_M);
  static void check_finite(Ravelin::VectorNd& v);

  static double distance_from_plane(const Ravelin::Vector3d& normal,const Ravelin::Vector3d& point, const Ravelin::Vector3d& x);

  /// 4 Control Point Bezier curve EEF calculation
  // Calculate the next bezier point.
  static void evalBernstein(const Ravelin::Vector3d& A, const Ravelin::Vector3d& B, const Ravelin::Vector3d& C, const Ravelin::Vector3d& D, double t,
                     Ravelin::Vector3d& P,Ravelin::Vector3d& dP,Ravelin::Vector3d& ddP);

  static void evalBernstein(const Ravelin::Vector3d& A, const Ravelin::Vector3d& B, const Ravelin::Vector3d& C, const Ravelin::Vector3d& D, double t,Ravelin::Vector3d& P);
  static Ravelin::Vector3d logistic(double x,double a, double b);
  static void bezierCurve(const std::vector<Ravelin::Vector3d>& control_points, int num_segments,
                   std::vector<Ravelin::Vector3d>& trajectory,
                   std::vector<Ravelin::Vector3d> & dtrajectory,
                   std::vector<Ravelin::Vector3d> & ddtrajectory);

  static double get_z_plane(double x, double y,const Ravelin::Vector3d& N, const Ravelin::Vector3d& P){
    static std::vector<double> gp(4);
    // N(X - P) = a(x - px) + b(y - py) + c(z - pz) = 0
    // ax + by + cz + N.-P = 0
    // ax + by + cz = d
    // N = [a b c]
    gp[0] = N[0];
    gp[1] = N[1];
    gp[2] = N[2];
    // d = N.P
    gp[3] = N.dot(P);

    // gp is a,b,c,d
    // z = d/c + a/-c X + b/-c Y
    return (gp[3]/gp[2] + x*gp[0]/-gp[2] + y*gp[1]/-gp[2]);
  }

  static Ravelin::Vector3d slerp( const Ravelin::Vector3d& v0,const Ravelin::Vector3d& v1,double t);
  static Ravelin::Vector3d lerp( const Ravelin::Vector3d& v0,const Ravelin::Vector3d& v1,double t);

  static void calc_cubic_spline_coefs(const Ravelin::VectorNd& T,const Ravelin::VectorNd& X,
                                             const Ravelin::Vector2d& Xd,const Ravelin::Vector2d& Xdd,
                                             Ravelin::VectorNd& B);
  static void calc_cubic_spline_coefs(const Ravelin::VectorNd &T, const Ravelin::VectorNd &X,  const Ravelin::Vector2d &Xd, Ravelin::VectorNd &B);

  static void eval_cubic_spline(const Ravelin::VectorNd& coefs,const Ravelin::VectorNd& t_limits,int num_segments,
                           Ravelin::VectorNd& X, Ravelin::VectorNd& Xd, Ravelin::VectorNd& Xdd);
  static bool eval_cubic_spline(const std::vector<Ravelin::VectorNd>& coefs,const std::vector<Ravelin::VectorNd>& t_limits,double t,
                           double& X, double& Xd, double& Xdd);

  static double linesearch(double (*f)(const Ravelin::VectorNd& x, Ravelin::VectorNd& fk, Ravelin::VectorNd& gk) ,
                      const Ravelin::VectorNd& d,const Ravelin::VectorNd& x,double rho = 0.5,double c = 1e-4);

  static int gcd(int a, int b)
  {
      for (;;)
      {
          if (a == 0) return b;
          b %= a;
          if (b == 0) return a;
          a %= b;
      }
  }

  static int lcm(int a, int b)
  {
      int temp = gcd(a, b);

      return temp ? (a / temp * b) : 0;
  }

  static int sign(double x){
    if(x>0)
      return 1;
    if(x<0)
      return -1;
    else
      return 0;
  }
};
#endif // UTILITIES_H
