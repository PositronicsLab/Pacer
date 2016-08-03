/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#ifndef UTILITIES_H
#define UTILITIES_H

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

#include <boost/icl/type_traits/to_string.hpp>
//template<typename T>
//using toString = boost::icl::to_string<T>;

static const double _TWO_PI= M_PI*2.0;

#include <Pacer/Visualizable.h>
#include <Pacer/output.h>
#include <Pacer/Random.h>
#include <Pacer/controller.h>

#include <Ravelin/LinAlgd.h>

static Ravelin::LinAlgd LA_;

class Utility{

    /// Data storage
public:
    static std::vector<Pacer::VisualizablePtr> visualize;

  static Ravelin::VectorNd pose_to_vec(const boost::shared_ptr<Ravelin::Pose3d> T){
    Ravelin::VectorNd p(7);
//    Ravelin::Transform3d T = Ravelin::Pose3d::calc_relative_pose(pose,boost::shared_ptr<Ravelin::Pose3d>( new Ravelin::Pose3d()));
    p = pose_to_vec(*(T.get()));
    return p;
  }
  
  static Ravelin::VectorNd pose_to_vec(const Ravelin::Pose3d& T){
    Ravelin::VectorNd p(7);
    //    Ravelin::Transform3d T = Ravelin::Pose3d::calc_relative_pose(pose,boost::shared_ptr<Ravelin::Pose3d>( new Ravelin::Pose3d()));
    p[0] = T.x[0];
    p[1] = T.x[1];
    p[2] = T.x[2];
    
    p[3] = T.q.x;
    p[4] = T.q.y;
    p[5] = T.q.z;
    p[6] = T.q.w;
    return p;
  }
  
  static Ravelin::Pose3d vec_to_pose(const Ravelin::VectorNd& p){
    Ravelin::Pose3d T;
    //    Ravelin::Transform3d T = Ravelin::Pose3d::calc_relative_pose(pose,boost::shared_ptr<Ravelin::Pose3d>( new Ravelin::Pose3d()));
    T.x[0] = p[0];
    T.x[1] = p[1];
    T.x[2] = p[2];
                 
    T.q.x  = p[3];
    T.q.y  = p[4];
    T.q.z  = p[5];
    T.q.w  = p[6];
    return T;
  }
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
        return Mod(fAng + M_PI, _TWO_PI) - M_PI;
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

  static bool eval_cubic_spline(const Ravelin::VectorNd& coefs,const Ravelin::VectorNd& t_limits,double t,
                           double& X, double& Xd, double& Xdd);
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
  static bool solve_qp_pos(const Ravelin::MatrixNd& Q, const Ravelin::VectorNd& c, const Ravelin::MatrixNd& A, const Ravelin::VectorNd& b, Ravelin::VectorNd& x, Ravelin::VectorNd& v, bool warm_start = false,bool regularize = true);
  static bool solve_qp_pos(const Ravelin::MatrixNd& Q, const Ravelin::VectorNd& c, Ravelin::VectorNd& x, bool warm_start = false);
  static bool solve_qp(const Ravelin::MatrixNd& Q, const Ravelin::VectorNd& c, const Ravelin::MatrixNd& A, const Ravelin::VectorNd& b, Ravelin::VectorNd& x);
	static bool lcp_symm_iter(const Ravelin::MatrixNd& M, const Ravelin::VectorNd& q, Ravelin::VectorNd& z, double lambda, double omega, unsigned MAX_ITER);
  static bool lcp_fast(const Ravelin::MatrixNd& M, const Ravelin::VectorNd& q, const std::vector<unsigned>& indices, Ravelin::VectorNd& z, double zero_tol);
};

static double sigmoid(double x){
  return (1.0 / (1.0 + exp(-x)));
}

static double sigmoid_interp(double v0, double vF, double alpha){
  // sigmoid curve interpolates wrt to alpha \in {0..1}
  double diff = vF - v0;
  return v0 + diff*sigmoid(alpha*10.0 - 5.0);
}

template <typename T>
static int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

template <typename T>
static T sqr(T val) {
  return val*val;
}

static double decimal_part(double val){
  return val - (double) ((int) val);
  
}

namespace Random{

  typedef std::pair<std::vector<boost::shared_ptr<Random::Generator> >, std::vector<double> > ParamDefaultPair;
  typedef std::map<std::string, ParamDefaultPair > ParamMap;
  typedef std::map<std::string, std::vector<double> > ParamValueMap;
  
  static void parse_distribution(const std::string& parameters,const boost::shared_ptr<Pacer::Controller> ctrl,std::vector<boost::shared_ptr<Random::Generator> >& generator){
    std::vector<double> min, max, mu, sigma;
    int N = 0;
    bool has_min = false, has_max = false, has_mu = false, has_sigma = false;
    
    std::string distribution_type;
    if(!ctrl->get_data<std::string>(parameters+".distribution", distribution_type))
      throw std::runtime_error("there is no default value OR distribution params for this!");
    
    OUT_LOG(logDEBUG) << parameters << " has a distribution of type: " << distribution_type;
    
    if ((has_min = ctrl->get_data<std::vector<double> >(parameters+".min", min))){
      OUT_LOG(logDEBUG) << "min = " << min;
      N = min.size();
    }
    if ((has_max = ctrl->get_data<std::vector<double> >(parameters+".max", max))){
      OUT_LOG(logDEBUG) << "max = " << max;
      N = max.size();
    }
    if ((has_mu = ctrl->get_data<std::vector<double> >(parameters+".mu", mu))){
      OUT_LOG(logDEBUG) << "mu = " << mu;
      N = mu.size();
    }
    if ((has_sigma = ctrl->get_data<std::vector<double> >(parameters+".sigma", sigma))){
      OUT_LOG(logDEBUG) << "sigma = " << sigma;
      N = sigma.size();
    }
    
    generator.resize(N);
    for (int i=0;i<N;i++) {
      generator[i] = boost::shared_ptr<Random::Generator>(new Random::Generator());
      if(distribution_type.compare("gaussian") == 0){
        if (has_max && has_min && has_mu && has_sigma) {
          generator[i]->set_gaussian(mu[i],sigma[i],min[i],max[i]);
        } else if (has_max && has_min && !has_mu && !has_sigma) {
          generator[i]->set_gaussian_from_limits(min[i],max[i]);
        } else if (has_max && !has_min && !has_mu && !has_sigma) {
          generator[i]->set_gaussian_from_limits(-max[i],max[i]);
        } else if (!has_max && !has_min && has_mu && has_sigma) {
          generator[i]->set_gaussian(mu[i],sigma[i]);
        } else {
          throw std::runtime_error("Not a valid set of params for a GAUSSIAN distribution!");
        }
      } else if(distribution_type.compare("uniform") == 0){
        if (has_max && has_min) {
          generator[i]->set_uniform(min[i],max[i]);
        } else if (has_max && !has_min){
          generator[i]->set_uniform(-max[i],max[i]);
        } else {
          throw std::runtime_error("Not a valid set of params for a UNIFORM distribution!");
        }
      } else {
        throw std::runtime_error("Not a valid distribution!");
      }
    }
  }
  
  // create distribution map
  static void create_distributions(std::string name,const boost::shared_ptr<Pacer::Controller> ctrl,ParamMap& parameter_generator, bool verbose_name = false){
    // Initialize Parameter distributions
    
    // FOR EACH UNCERTAINTY (manufacturing ,state)
    std::vector<std::string> uncertainty_names;
    if(ctrl->get_data<std::vector<std::string> >(name+".id",uncertainty_names)){
      for (int i=0; i<uncertainty_names.size();i++) {
        std::string& uncertainty_name = uncertainty_names[i];
        
        // FOR EACH TYPE (joint, link)
        std::vector<std::string> type_names;
        if(ctrl->get_data<std::vector<std::string> >(name+"."+uncertainty_name+".id",type_names)){
          for (int j=0; j<type_names.size();j++) {
            std::string& type_name = type_names[j];
            
            // FOR EACH OBJECT (joint name, link name)
            std::vector<std::string> object_names;
            if(ctrl->get_data<std::vector<std::string> >(name+"."+uncertainty_name+"."+type_name+".id",object_names)){
              for (int k=0; k<object_names.size();k++) {
                std::string& object_name = object_names[k];
                
                std::vector<std::string> value_names;
                if(ctrl->get_data<std::vector<std::string> >(name+"."+uncertainty_name+"."+type_name+"."+object_name+".id",value_names)){
                  for (int l=0; l<value_names.size();l++) {
                    std::string& value_name = value_names[l];
                    
                    // Create vector of generators and default values
                    std::vector<boost::shared_ptr<Random::Generator> > generator;
                    std::vector<double>                        default_value;
                    // try to use default value
                    if(!ctrl->get_data<std::vector<double> >(name+"."+uncertainty_name+"."+type_name+"."+object_name+"."+value_name,default_value)){
                      // if we're here then there are sub-tags to this parameter (generator params)
                      parse_distribution(name+"."+uncertainty_name+"."+type_name+"."+object_name+"."+value_name,ctrl,generator);
                      
                      OUT_LOG(logDEBUG) << "Created generator for uncertain parameter: "<< object_name << "." << value_name;
                      OUT_LOG(logDEBUG) << "\t FROM: uncertainty."+uncertainty_name+"."+type_name+"."+object_name+"."+value_name;
                    } else {
                      OUT_LOG(logDEBUG) << "Used default for uncertain parameter: "<< object_name << "." << value_name;
                      OUT_LOG(logDEBUG) << "\t FROM: uncertainty."+uncertainty_name+"."+type_name+"."+object_name+"."+value_name;
                      OUT_LOG(logDEBUG) << "\t default: " << default_value;
                    }
                    
                    // error check
                    if( (generator.empty() && default_value.empty()) || (!generator.empty() && !default_value.empty()))
                      throw std::runtime_error("there are default values AND distribution params for this value!");
                    
                    OUT_LOG(logDEBUG) << "parameter: "<< object_name << "." << value_name << " pushed to map.";
                    if(verbose_name){
                      parameter_generator[uncertainty_name+"."+type_name+"."+object_name+"."+value_name] = ParamDefaultPair(generator,default_value);
                    } else {
                      parameter_generator[object_name+"."+value_name] = ParamDefaultPair(generator,default_value);
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
  }
  
  static void generate_parameters(const ParamMap& parameter_generator,const boost::shared_ptr<Pacer::Controller> ctrl, ParamValueMap& generated_params){
    for(ParamMap::const_iterator it = parameter_generator.begin(); it != parameter_generator.end(); it++){
      if (it->second.first.empty()) {
        for (int i=0;i<it->second.second.size(); i++) {
          double value = it->second.second[i];
          OUT_LOG(logDEBUG) << " " << value;
          generated_params[it->first].push_back(value);
        }
      } else { // Generators created for variable
        for (int i=0;i<it->second.first.size(); i++) {
          double value = it->second.first[i]->generate();
          generated_params[it->first].push_back(value);
        }
      }
    }
  }

}
#endif
