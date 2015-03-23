/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#ifndef UTILITIES_H
#define UTILITIES_H

    static const double     _PI= 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348;
    static const double _TWO_PI= 6.2831853071795864769252867665590057683943387987502116419498891846156328125724179972560696;
#include <Ravelin/MatrixNd.h>
#include <Ravelin/Transform3d.h>
#include <Ravelin/VectorNd.h>
#include <Ravelin/SVector6d.h>
#include <Ravelin/Vector3d.h>
#include <mutex>
#include <Pacer/Log.h>
#include <boost/assign/std/vector.hpp>
#include <boost/assign/list_of.hpp>

#include <boost/shared_ptr.hpp>
#include <math.h>
#include <cmath>
#include <sys/types.h>
#include <sys/times.h>
#include <fstream>
#include <string>
#include <iostream>     // std::cout, std::fixed
#include <iomanip>      // std::setprecision
#include <math.h>
#include <numeric>
#include <Pacer/Visualizable.h>

class Utility{

    /// Data storage
public:
    static std::vector<Pacer::VisualizablePtr> visualize;

    // Floating-point modulo
    // The result (the remainder) has same sign as the divisor.
    // Similar to matlab's mod(); Not similar to fmod() -   Mod(-3,4)= 1   fmod(-3,4)= -3
    template<typename T>
    static T Mod(T x, T y)
    {
        if (0. == y)
            return x;

        double m= x - y * floor(x/y);

        // handle boundary cases resulted from floating-point cut off:

        if (y > 0)              // modulo range: [0..y)
        {
            if (m>=y)           // Mod(-1e-16             , 360.    ): m= 360.
                return 0;

            if (m<0 )
            {
                if (y+m == y)
                    return 0  ; // just in case...
                else
                    return y+m; // Mod(106.81415022205296 , _TWO_PI ): m= -1.421e-14
            }
        }
        else                    // modulo range: (y..0]
        {
            if (m<=y)           // Mod(1e-16              , -360.   ): m= -360.
                return 0;

            if (m>0 )
            {
                if (y+m == y)
                    return 0  ; // just in case...
                else
                    return y+m; // Mod(-106.81415022205296, -_TWO_PI): m= 1.421e-14
            }
        }

        return m;
    }

    // wrap [rad] angle to [-PI..PI)
    static double WrapPosNegPI(double fAng)
    {
        return Mod(fAng + _PI, _TWO_PI) - _PI;
    }

    // wrap [rad] angle to [0..TWO_PI)
    static double WrapTwoPI(double fAng)
    {
        return Mod(fAng, _TWO_PI);
    }

    // wrap [deg] angle to [-180..180)
    static double WrapPosNeg180(double fAng)
    {
        return Mod(fAng + 180., 360.) - 180.;
    }

    // wrap [deg] angle to [0..360)
    static double Wrap360(double fAng)
    {
        return Mod(fAng ,360.);
    }

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

  template<class T>
  static T sign(T x){
    if(x>0)
      return (T) 1.0;
    if(x<0)
      return (T) -1.0;
    else
      return (T) 0.0;
  }

	static bool isvalid(const Ravelin::VectorNd& v){
	  if(!std::isfinite(v.norm()))
      return false;
	  return true;
	}

	// Solvers
  static void solve(Ravelin::MatrixNd& M,Ravelin::VectorNd& bx);
	static bool solve_qp_pos(const Ravelin::MatrixNd& Q, const Ravelin::VectorNd& c, const Ravelin::MatrixNd& A, const Ravelin::VectorNd& b, Ravelin::VectorNd& x);
	static bool solve_qp_pos(const Ravelin::MatrixNd& Q, const Ravelin::VectorNd& c, Ravelin::VectorNd& x);
	static bool solve_qp(const Ravelin::MatrixNd& Q, const Ravelin::VectorNd& c, const Ravelin::MatrixNd& A, const Ravelin::VectorNd& b, Ravelin::VectorNd& x);
	static bool lcp_symm_iter(const Ravelin::MatrixNd& M, const Ravelin::VectorNd& q, Ravelin::VectorNd& z, double lambda, double omega, unsigned MAX_ITER);
  static bool lcp_fast(const Ravelin::MatrixNd& M, const Ravelin::VectorNd& q, const std::vector<unsigned>& indices, Ravelin::VectorNd& z, double zero_tol);

  //////////////  Variables ///////////////
  static void load_variables(std::string fname);
  
  static void test_function(std::string fname){}
  
  template <typename T>
  static T& get_variable(std::string tag){
    try{
      T& v = get_variable_internal<T>(tag);
      OUTLOG(v,"Get: "+tag+" --> ",logDEBUG);
      return v;
    }catch(...){
      throw std::runtime_error("Parameter: \"" + tag + "\" not found in XML!");
    }
  }
  
  template <typename T>
  static T& get_variable_internal(std::string tag);

};
#endif // UTILITIES_H
