
#include<iostream>
#include<vector>
#include<algorithm>
#include<cmath>
#include <project_common.h>
#include <utilities.h>

void Utility::evalBernstein(const Ravelin::Vector3d& A, const Ravelin::Vector3d& B, const Ravelin::Vector3d& C, const Ravelin::Vector3d& D, double t,Ravelin::Vector3d& P,Ravelin::Vector3d& dP,Ravelin::Vector3d& ddP) {

  // Position
    P[0] =A[0]- 3*A[0]*t + 3*A[0]*t*t -A[0]*t*t*t + 3*B[0]*t - 6*B[0]*t*t + 3*B[0]*t*t*t + 3*C[0]*t*t - 3*C[0]*t*t*t + t*t*t*D[0];
    P[1] =A[1]- 3*A[1]*t + 3*A[1]*t*t -A[1]*t*t*t + 3*B[1]*t - 6*B[1]*t*t + 3*B[1]*t*t*t + 3*C[1]*t*t - 3*C[1]*t*t*t + t*t*t*D[1];
    P[2] =A[2]- 3*A[2]*t + 3*A[2]*t*t -A[2]*t*t*t + 3*B[2]*t - 6*B[2]*t*t + 3*B[2]*t*t*t + 3*C[2]*t*t - 3*C[2]*t*t*t + t*t*t*D[2];

   // Velocity
    dP[0] = -3*(A[0]*(-1+t)*(-1+t)+t*(-2*C[0]+3*C[0]*t-D[0]*t)+B[0]*(-1+4*t-3*t*t));
    dP[1] = -3*(A[1]*(-1+t)*(-1+t)+t*(-2*C[1]+3*C[1]*t-D[1]*t)+B[1]*(-1+4*t-3*t*t));
    dP[2] = -3*(A[2]*(-1+t)*(-1+t)+t*(-2*C[2]+3*C[2]*t-D[2]*t)+B[2]*(-1+4*t-3*t*t));

   // Acceleration
    ddP[0] =-6*(-C[0]+B[0]*(2-3*t)+A[0]*(-1+t)+3*C[0]*t-D[0]*t);
    ddP[1] =-6*(-C[1]+B[1]*(2-3*t)+A[1]*(-1+t)+3*C[1]*t-D[1]*t);
    ddP[2] =-6*(-C[2]+B[2]*(2-3*t)+A[2]*(-1+t)+3*C[2]*t-D[2]*t);
}

void Utility::evalBernstein(const Ravelin::Vector3d& A, const Ravelin::Vector3d& B, const Ravelin::Vector3d& C, const Ravelin::Vector3d& D, double t,Ravelin::Vector3d& P) {

  // Position
    P[0] =A[0]- 3*A[0]*t + 3*A[0]*t*t -A[0]*t*t*t + 3*B[0]*t - 6*B[0]*t*t + 3*B[0]*t*t*t + 3*C[0]*t*t - 3*C[0]*t*t*t + t*t*t*D[0];
    P[1] =A[1]- 3*A[1]*t + 3*A[1]*t*t -A[1]*t*t*t + 3*B[1]*t - 6*B[1]*t*t + 3*B[1]*t*t*t + 3*C[1]*t*t - 3*C[1]*t*t*t + t*t*t*D[1];
    P[2] =A[2]- 3*A[2]*t + 3*A[2]*t*t -A[2]*t*t*t + 3*B[2]*t - 6*B[2]*t*t + 3*B[2]*t*t*t + 3*C[2]*t*t - 3*C[2]*t*t*t + t*t*t*D[2];
}

void Utility::bezierCurve(const std::vector<Ravelin::Vector3d>& control_points, int num_segments,
                 std::vector<Ravelin::Vector3d>& trajectory,
                 std::vector<Ravelin::Vector3d> & dtrajectory,
                 std::vector<Ravelin::Vector3d> & ddtrajectory){
  trajectory.resize(num_segments);
  dtrajectory.resize(num_segments);
  ddtrajectory.resize(num_segments);
  double u_last,t_last;
  for(int t = 0;t < num_segments; t++) {
    Ravelin::Vector3d u = logistic((double)t/(double)num_segments,10,0.5);
    evalBernstein(control_points[0], control_points[1], control_points[2], control_points[3], u[0],trajectory[t],dtrajectory[t],ddtrajectory[t]);
    for(int i=0;i<3;i++){
      if(!std::isfinite(trajectory[t][i]))
        trajectory[t][i] = trajectory[t-1][i];
      if(!std::isfinite(dtrajectory[t][i]))
        dtrajectory[t][i] = dtrajectory[t-1][i];
      if(!std::isfinite(ddtrajectory[t][i]))
        ddtrajectory[t][i] = ddtrajectory[t-1][i];
    }
    ddtrajectory[t] = ddtrajectory[t]*u[1]*u[1] + dtrajectory[t]*u[2];
    dtrajectory[t] *= u[1];
  }
}

bool Utility::eval_cubic_spline(const std::vector<Ravelin::VectorNd>& coefs,const std::vector<Ravelin::VectorNd>& t_limits,double t,
                       double& X, double& Xd, double& Xdd){
  // Find Spline in which to evaluate t
  int j=0,k=0;
  while(t >= t_limits[j][k+1]){
    k++;
    if(k == t_limits[j].rows()-1){
      j++;
      k=0;
      if(j == t_limits.size())
        // t is not valid in any of the spline intervals
        return false;
    }
  }
  // Spline always evaluates from t[0] = 0 in interval
  // offset t to start of interval to find t in spline
  t -= t_limits[j][0];

  X    = t*t*t*coefs[j][k*4] + t*t*coefs[j][k*4+1] + t*coefs[j][k*4+2] + coefs[j][k*4+3];
  Xd   = 3*t*t*coefs[j][k*4] + 2*t*coefs[j][k*4+1] +   coefs[j][k*4+2];
  Xdd  =   6*t*coefs[j][k*4] +   2*coefs[j][k*4+1] ;

  return true;
}


void Utility::calc_cubic_spline_coefs(const Ravelin::VectorNd& T_,const Ravelin::VectorNd& X,
                                           const Ravelin::Vector2d& Xd, Ravelin::VectorNd& B){
  static Ravelin::MatrixNd A;

  // Spline always solves from t[0] = 0 in interval
  static Ravelin::VectorNd T;
  T = T_;
  for(int i=0;i<T.rows();i++)
    T[i] -= T_[0];

  assert(T.rows() == X.rows());

  int N = X.rows(); //n_control_points
  // N_VIRTUAL_KNOTS= 2
  // N_SPLINES      = num_knots+N_VIRTUAL_KNOTS - 1
  // N_CONSTRAINTS  = num_knots*3 + num_knots + 2 + 2
  // N_COEFS        = 4*N_SPLINES
  B.set_zero(4*(N+1));
  A.set_zero(4*(N+1),4*(N+1));

  // ---------- start constraint ----------
  B[0] =     0;
  B[1] =  Xd[0]; // Velocity clamped spline
  B[2] =   X[0];
  // xdd
  A(0,0) = 6*T[0];
  A(0,1) = 2;
  // xd
  A(1,0) = 3*T[0]*T[0];
  A(1,1) = 2*T[0];
  A(1,2) = 1;
  // x
  A(2,0) = T[0]*T[0]*T[0];
  A(2,1) = T[0]*T[0];
  A(2,2) = T[0];
  A(2,3) = 1;

  // ---------- start Virtual constraints ----------
  double tv0 = T[0] + (T[1]-T[0])/1000.0;
  // xdd
  A(3,0) = 6*tv0;
  A(3,1) = 2;
  A(3,4) = -6*tv0;
  A(3,5) = -2;
  // xd
  A(4,0) = 3*tv0*tv0;
  A(4,1) = 2*tv0;
  A(4,2) = 1;
  A(4,4) = -3*tv0*tv0;
  A(4,5) = -2*tv0;
  A(4,6) = -1;
  // x
  A(5,0) = tv0*tv0*tv0;
  A(5,1) = tv0*tv0;
  A(5,2) = tv0;
  A(5,3) = 1;
  A(5,4) = -tv0*tv0*tv0;
  A(5,5) = -tv0*tv0;
  A(5,6) = -tv0;
  A(5,7) = -1;

  // ---------- End Virtual Constraint ----------
  double tvN = T[N-1] - (T[N-1]-T[N-2])/1000.0;
  // xddd
//  A(A.rows()-6,A.columns()-8) = 6;
//  A(A.rows()-6,A.columns()-4) = -6;
  // xdd
  A(A.rows()-6,A.columns()-8) = 6*tvN;
  A(A.rows()-6,A.columns()-7) = 2;
  A(A.rows()-6,A.columns()-4) = -6*tvN;
  A(A.rows()-6,A.columns()-3) = -2;
  // xd
  A(A.rows()-5,A.columns()-8) = 3*tvN*tvN;
  A(A.rows()-5,A.columns()-7) = 2*tvN;
  A(A.rows()-5,A.columns()-6) = 1;
  A(A.rows()-5,A.columns()-4) = -3*tvN*tvN;
  A(A.rows()-5,A.columns()-3) = -2*tvN;
  A(A.rows()-5,A.columns()-2) = -1;
  // x
  A(A.rows()-4,A.columns()-8) = tvN*tvN*tvN;
  A(A.rows()-4,A.columns()-7) = tvN*tvN;
  A(A.rows()-4,A.columns()-6) = tvN;
  A(A.rows()-4,A.columns()-5) = 1;
  A(A.rows()-4,A.columns()-4) = -tvN*tvN*tvN;
  A(A.rows()-4,A.columns()-3) = -tvN*tvN;
  A(A.rows()-4,A.columns()-2) = -tvN;
  A(A.rows()-4,A.columns()-1) = -1;

  // ---------- End constraint ----------
  B[B.rows()-3] = 0;
  B[B.rows()-2] = Xd[1]; // Velocity clamped spline
  B[B.rows()-1] = X[N-1];
  // xdd
  A(A.rows()-3,A.columns()-4) = 6*T[N-1];
  A(A.rows()-3,A.columns()-3) = 2;
  // xddd
//  A(A.rows()-3,A.columns()-4) = 6;
  // xd
  A(A.rows()-2,A.columns()-4) = 3*T[N-1]*T[N-1];
  A(A.rows()-2,A.columns()-3) = 2*T[N-1];
  A(A.rows()-2,A.columns()-2) = 1;
  // x
  A(A.rows()-1,A.columns()-4) = T[N-1]*T[N-1]*T[N-1];
  A(A.rows()-1,A.columns()-3) = T[N-1]*T[N-1];
  A(A.rows()-1,A.columns()-2) = T[N-1];
  A(A.rows()-1,A.columns()-1) = 1;

  // ---------- Fill in continuity conponents at each of N-2 knots ----------
  for(int i=0;i<N-2;i++){
    // Xdd continuity
    // \ddot{P}_i - \ddot{P}_{i+1} = 0
    A(5 + 4*i + 1,3 + 4*i     + 1) = 6*T[i+1];
    A(5 + 4*i + 1,3 + 4*i     + 2) = 2;
    A(5 + 4*i + 1,3 + 4*(i+1) + 1) = -6*T[i+1];
    A(5 + 4*i + 1,3 + 4*(i+1) + 2) = -2;

    // Xd continuity
    // \dot{P}_i - \dot{P}_{i+1} = 0
    A(5 + 4*i + 2,3 + 4*i     + 1) = 3*T[i+1]*T[i+1];
    A(5 + 4*i + 2,3 + 4*i     + 2) = 2*T[i+1];
    A(5 + 4*i + 2,3 + 4*i     + 3) = 1;
    A(5 + 4*i + 2,3 + 4*(i+1) + 1) = -3*T[i+1]*T[i+1];
    A(5 + 4*i + 2,3 + 4*(i+1) + 2) = -2*T[i+1];
    A(5 + 4*i + 2,3 + 4*(i+1) + 3) = -1;

    // X continuity
    // P_i - P_{i+1} = 0
    A(5 + 4*i + 3,3 + 4*i     + 1) = T[i+1]*T[i+1]*T[i+1];
    A(5 + 4*i + 3,3 + 4*i     + 2) = T[i+1]*T[i+1];
    A(5 + 4*i + 3,3 + 4*i     + 3) = T[i+1];
    A(5 + 4*i + 3,3 + 4*i     + 4) = 1;
    A(5 + 4*i + 3,3 + 4*(i+1) + 1) = -T[i+1]*T[i+1]*T[i+1];
    A(5 + 4*i + 3,3 + 4*(i+1) + 2) = -T[i+1]*T[i+1];
    A(5 + 4*i + 3,3 + 4*(i+1) + 3) = -T[i+1];
    A(5 + 4*i + 3,3 + 4*(i+1) + 4) = -1;

    // P_i = X[i]
    B[5 + 4*i + 4] = X[i+1];
    A(5 + 4*i + 4,3 + 4*(i+1) + 1) = T[i+1]*T[i+1]*T[i+1];
    A(5 + 4*i + 4,3 + 4*(i+1) + 2) = T[i+1]*T[i+1];
    A(5 + 4*i + 4,3 + 4*(i+1) + 3) = T[i+1];
    A(5 + 4*i + 4,3 + 4*(i+1) + 4) = 1;
  }

  workv_ = B;

  // Solve linear system (A is corrupted and workv_ has result)
  LA_.solve_fast(A,workv_);

  // Exclude virtual points from returned spline coeficients
  workv_.get_sub_vec(4,workv_.size()-4,B);
}

bool eval_cubic_spline(const Ravelin::VectorNd& coefs,const Ravelin::VectorNd& t_limits,double t,
                       double& X, double& Xd, double& Xdd){
 int k=0;
  while(t >= t_limits[k+1]){
    k++;
    if(k == t_limits.rows()-1)
      return false;
  }
  // Spline always evaluates from t[0] = 0 in interval
  // offset t to start of interval to find t in spline
  t -= t_limits[0];

  X    = t*t*t*coefs[k*4] + t*t*coefs[k*4+1] + t*coefs[k*4+2] + coefs[k*4+3];
  Xd   = 3*t*t*coefs[k*4] + 2*t*coefs[k*4+1] +   coefs[k*4+2];
  Xdd  =   6*t*coefs[k*4] +   2*coefs[k*4+1] ;

  return true;
}

int test_spline(){

  // This is for ONE dimension,
  // for 3 dimensions just do this in each of the 3 spatial axis
  Ravelin::VectorNd coefs;
  //  we want a spline through x = {0,10,5} these are "knots";
  //  we want the spline to reach each knot at times t = {0,1,2};
  Ravelin::VectorNd X(3);
  Ravelin::VectorNd t_limits(3);
  X[0] = 0.0;
  X[1] = 10.0;
  X[2] = 5.0;

  t_limits[0] = 0.0;
  t_limits[1] = 1.0;
  t_limits[2] = 2.0;

  //  we also want the spaceship to be stationary in the x axis
  // at the 2 end points
  // This is a "Velocity clamped" cubic spline
  Ravelin::Vector2d Xd(0,0);

//  calc_cubic_spline_coefs(t_limits,X,Xd,coefs);

  // Now we can evaluate the spline at any t [0..2)
  double pos,vel,acc;
  for(double t=0;t<2;t += 0.01){
    eval_cubic_spline(coefs,t_limits,t,pos,vel,acc);
    printf("%f, %f, %f\n",pos,vel,acc);
    // now pos,vel,acc are your desired velocities on the trajectory at time t.
    // You may track these using a PD controller on pos & vel
    // or an inverse dynamics controller on acc
  }
  return 0;
}
