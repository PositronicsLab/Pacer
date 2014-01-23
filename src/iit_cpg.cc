#include <quadruped.h>

std::vector<Ravelin::Vector3d>& Quadruped::foot_oscilator(
  const std::vector<Ravelin::Vector3d>& x0,  const std::vector<Ravelin::Vector3d>& x, const Ravelin::MatrixNd& C,
  double Ls,const Ravelin::VectorNd& Hs,double Df,double Vf,double bp,std::vector<Ravelin::Vector3d>& xd){

  /* Tunable parameters
   * Ls    : length of step
   * Hs    : height of step
   * Df    : step duty factor
   * Vf    : forward velocity
   * bp    : the transition rate between phases
   * bf    : the transition rate between phases
   */
  // a/b/c : affect the convergence rate of the limit cycle
  double a = 10,
         b = -1,
         c = 10;

  // Stepping Filter params
  // depth of step phase where touchdown occurs (fraction of Hs)
  double ztd = 0.0;
  // speed at which behavior changes (arbitrary scale)
  double bf = 800;

  std::vector<Ravelin::Vector3d> xb(NUM_EEFS);
  for(int i=0;i<NUM_EEFS;i++)
    xb[i] = x[i] - x0[i];

  double Cp = 0;
  for(int i=0;i<NUM_EEFS;i++)
    for(int j=0;j<NUM_EEFS;j++)
      Cp += C(i,j)*Hs[i]*xb[j][2]/Hs[j];

  for(int i=0;i<NUM_EEFS;i++){

    Ravelin::Vector3d& xbar = xb[i];
    double Sp1 = 1.0/(exp(-bp*xbar[2]) + 1.0);
    double Sp2 = 1.0/(exp( bp*xbar[2]) + 1.0);

    double ws = M_PI * (Vf/Ls) * ((Df*Sp1)/(1.0-Df) + Sp2);

    // realtively thin gait [shoulder width]
    double dyc = 0;

    // \dot{x}
    double oscil = 1.0 - (4*xbar[0]*xbar[0])/(Ls*Ls) - xbar[2]*xbar[2]/(Hs[i]*Hs[i]) ;
    xd[i][0] = a*(oscil)*xbar[0] + ws*Ls*xbar[2]/(2*Hs[i]);
    xd[i][1] = b*(xbar[1] + dyc);
    xd[i][2] = c*(oscil)*xbar[2] - ws*2*Hs[i]*xbar[0]/Ls + Cp;

    double Sf1 = 1.0/(exp(-bf*(xbar[2] - ztd*Hs[i])) + 1.0);
    double Sf2 = 1.0/(exp( bf*(xbar[2] - ztd*Hs[i])) + 1.0);

    xd[i] = (xd[i])*Sf1 - Ravelin::Vector3d(Vf,0,0)*Sf2;

  }

  return xd;
}
