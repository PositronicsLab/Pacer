#include<robot.h>
#include <utilities.h>

using namespace Ravelin;

void Robot::contact_jacobian_null_stabilizer(const Ravelin::MatrixNd& R, Ravelin::VectorNd& uff){
  static Ravelin::VectorNd workv_;
  static Ravelin::MatrixNd workM_;

  if(NC == 0) return;

  std::vector<int> active_dofs;
  for(int i=0;i<NUM_EEFS;i++)
    if(eefs_[i].active)
      for(int j=0;j<eefs_[i].chain.size();j++)
        active_dofs.push_back(eefs_[i].chain[j]);

  if(active_dofs.size() == 0) return;
  OUTLOG(R,"R",logDEBUG1);
  // Select active rows in Jacobian Matrix
  // R -> Jh
  static Ravelin::MatrixNd Jh;
  for(int i=NUM_JOINTS;i<NDOFS;i++)
    active_dofs.push_back(i);

  std::sort(active_dofs.begin(),active_dofs.end());
  R.select_rows(active_dofs.begin(),active_dofs.end(),Jh);

//  Jh.transpose();
  OUTLOG(Jh,"Jh",logDEBUG1);

  // Generate Jh Nullspace
  Ravelin::MatrixNd NULL_Jh = MatrixNd::identity(Jh.columns()),
      Jh_plus;
//  Jh.mult_transpose(Jh,Jh_plus);
//  OUTLOG(Jh_plus,"(Jh Jh')");
  LA_.pseudo_invert(Jh);
  Jh_plus = Jh;
  OUTLOG(Jh,"Jh' (Jh Jh')^-1 == Jh+",logDEBUG1);

  LA_.nullspace(Jh,NULL_Jh);

  OUTLOG(NULL_Jh,"null(Jh)",logDEBUG1);
  // Use NULL(Jh) to stabilize trunk w/o altering gait

  // Trunk Stability gains
  Ravelin::Vector3d& rpy = roll_pitch_yaw;

  OUTLOG(rpy,"Roll, Pitch, Yaw",logDEBUG1);
  double Xp = 0, Xv = 0,
         Yp = 0, Yv = 0,
         Zp = 0, Zv = 0,
         Rp = 0, Rv = 1e-1,
         Pp = 0, Pv = 1e-1;

  static Ravelin::VectorNd Y,tY;
  Y.set_zero(NC*3+6);
  tY.set_zero(NC*3);

  Y[NC*3+2] = -(0)*Zp + -(vel[NUM_JOINTS+2])*Zv;
  Y[NC*3+3] = -rpy[0]*Rp + -vel[NUM_JOINTS+3]*Rv;
  Y[NC*3+4] = -rpy[1]*Pp + -vel[NUM_JOINTS+4]*Pv;
  Y.negate();
  OUTLOG(Y,"g_stabilization",logDEBUG1);
//  NULL_Jh.transpose_mult(Y,tY);
  Jh_plus.mult(Y,tY);
  OUTLOG(tY,"stabilization",logDEBUG1);
  // apply base stabilization forces
  for(int i=0;i<NC*3;i++)
    uff[active_dofs[i]] += tY[i];


}

