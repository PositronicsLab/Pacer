#include <quadruped_control.h>

Ravelin::Vector3d& foot_oscilator(
  const Ravelin::Vector3d& x0,  const Ravelin::Vector3d& x, const Mat& C,
  double Ls,const Vec& Hs,double Df,double Vf,double bp,Ravelin::Vector3d& xd){

  /* Tunable parameters
   * Ls    : length of step
   * Hs    : height of step
   * Df    : step duty factor
   * Vf    : forward velocity
   * bp    : the transition rate between phases
   * a/b/c : affect the convergence rate to the limit cycle
   */
  // alpha, beta, gamma
  double a = 1.0,
         b = 1.0,
         c = 1.0;

  Ravelin::Vector3d& xbar = x - x0;

  double Sp1 = 1.0/(exp(-bp*xbar[2]) + 1.0);
  double Sp2 = 1.0/(exp( bp*xbar[2]) + 1.0);

  double ws = M_PI * Vf/Ls * (Df/(1.0-Df) * Sp1 + Sp2);

  double Cp = 0;
  for(int i=0;i<Hs.rows();i++)
    for(int j=0;j<Hs.rows();j++)
      Cp += C(i,j)*Hs[i]/Hs[j];

  double dyc = 0;// xbar[1] - ;

  // \dot{x}
  xd[0] = a*(1.0 - xbar[0]*xbar[0]*4/(Ls*Ls) - xbar[2]*xbar[2]/(Hs*Hs) )*xbar[0]
          + ws*2*Ls*xbar[2]/(2*Hs);

  // \dot{y}
  xd[1] = b*(xbar[1] + dyc);

  // \dot{z}
  xd[2] = c*(1.0 - xbar[0]*xbar[0]*4/(Ls*Ls) - xbar[2]*xbar[2]/(Hs*Hs) )*xbar[2]
          + ws*2*Hs*xbar[0]/Ls + Cp;
}
