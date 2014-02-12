#include<quadruped.h>
#include <utilities.h>

using namespace Ravelin;

void Quadruped::contact_jacobian_null_stabilizer(const Ravelin::MatrixNd& R, Ravelin::VectorNd& uff){
  static Ravelin::VectorNd workv_;
  static Ravelin::MatrixNd workM_;

  std::vector<int> active_dofs;
  for(int i=0;i<NUM_EEFS;i++)
    if(eefs_[i].active)
      for(int j=0;j<eefs_[i].chain.size();j++)
        active_dofs.push_back(eefs_[i].chain[j]);

  if(active_dofs.size() == 0) return;
  OUTLOG(R,"R");
  // Select active rows in Jacobian Matrix
  // R -> Jh
  static Ravelin::MatrixNd Jh;
  for(int i=NUM_JOINTS;i<NDOFS;i++)
    active_dofs.push_back(i);

  std::sort(active_dofs.begin(),active_dofs.end());
  R.select_rows(active_dofs.begin(),active_dofs.end(),Jh);

  Jh.transpose();
  OUTLOG(Jh,"Jh");

  // Generate Jh Nullspace
  Ravelin::MatrixNd NULL_Jh = MatrixNd::identity(Jh.columns()),
      Jh_plus;
  workM_ = MatrixNd::identity(Jh.rows());
  Jh.mult_transpose(Jh,Jh_plus);
  OUTLOG(Jh_plus,"(Jh Jh')");

  try{
    LA_.solve_fast(Jh_plus,workM_);
    Jh.transpose_mult(workM_,Jh_plus);
    OUTLOG(Jh_plus,"Jh' (Jh Jh')^-1");
    OUTLOG(Jh,"Jh");
    NULL_Jh -= Jh.transpose_mult_transpose(Jh_plus,workM_);
    if(NULL_Jh.norm_inf() > 1e8)
      NULL_Jh.set_zero(Jh.rows(),Jh.rows());
  } catch(Ravelin::SingularException e) {
    OUT_LOG(logERROR) << "No trunk Stabilization" << std::endl;
    NULL_Jh.set_zero(Jh.rows(),Jh.rows());
  }

  OUTLOG(NULL_Jh,"null(Jh)");

  // Use NULL(Jh) to stabilize trunk w/o altering gait

  // Trunk Stability gains
  Ravelin::Vector3d rpy;
  Ravelin::Matrix3d Rot(base_frame->q);

  bool use_rpy =  R2rpy(Rot,rpy);
  std::cout << "rpy" << rpy << std::endl;
  double Xp = 0, Xv = 0,
         Yp = 0, Yv = 0,
         Zp = 1e-2, Zv = 0,
         Rp = 1e-2, Rv = 1e-2,
         Pp = 1e-2, Pv = 1e-2;

  static Ravelin::VectorNd Y,tY;
  Y.set_zero(NC*3+6);

  Y[NC*3+2] = (base_horizonal_frame->x[2]+eefs_[0].origin[2])*Zp + (vel[NUM_JOINTS+2]-0)*Zv;
  Y[NC*3+3] = ((use_rpy)? (rpy[0]-0) : 0) + (vel[NUM_JOINTS+3]-0)*Rv;
  Y[NC*3+4] = ((use_rpy)? (rpy[1]-0) : 0) +  (vel[NUM_JOINTS+4]-0)*Pv;

  NULL_Jh.mult(Y,tY);
  std::cout << "tS" << tY << std::endl;
  // apply base stabilization forces
  for(int i=0;i<NC*3;i++)
    uff[active_dofs[i]] += tY[i];

}
