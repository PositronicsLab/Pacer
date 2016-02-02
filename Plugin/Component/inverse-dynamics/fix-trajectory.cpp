bool fix_trajectory(const Ravelin::VectorNd& v, const Ravelin::MatrixNd& M,const  Ravelin::MatrixNd& N,const Ravelin::MatrixNd& S,const Ravelin::MatrixNd& T, const Ravelin::VectorNd& fext, double h, const Ravelin::MatrixNd& MU, Ravelin::VectorNd& qdd){
  
  //////////////////////// LP PHASE /////////////////////////
  /* OBJECTIVE
   * Min workspace deviation from v+
   * min_{x = [z,T,s]'}
   *   [0,0,1] x
   * s.t.
   *  -[A,",-I] x   >= -v+              // operational space goal l1-norm
   *   [A,",-I] x    >=  v+              // "
   *   N*iM*[R' F,0] x >= -N(h*iM*fext + v)   // Interpenetration
   *   [CF,0,0] x      >=  0                  // coulomb friction
   *   z >= 0                                 // compressive force
   *   T+ >= T >= T-                          // torque limits
   */
  
  Ravelin::MatrixNd M(WS_DOFS*2 + NC*2,nq+NC*5+WS_DOFS),
  A(0,M.columns());
  Ravelin::VectorNd q(M.rows()),
  c,
  x1,
  b(0);
  
  c.set_zero(nq+NC*5+WS_DOFS);
  c.set_sub_vec(nq+NC*5,Ravelin::VectorNd::one(WS_DOFS));
  c.set_sub_vec(nq+NC*5+WS_DOFS-6,(workv_ = Ravelin::VectorNd::one(6))*=0.1);
  
  // -- operational space goal l1-norm --
  // J iM [R' F] x == (v*_w - v_w - h J iM fext)
  //  [ M, I][x,s]' >=   (v*_w - v_w - h J iM fext)
  //  [-M, I][x,s]' >=  -(v*_w - v_w - h J iM fext)
  
  workM1.set_zero(JiMRTF.rows()*2,JiMRTF.columns()+WS_DOFS);
  workM1.set_sub_mat(0,0,JiMRTF);
  workM1.set_sub_mat(JiMRTF.rows(),0,(workM2 = JiMRTF).negate());
  workM1.set_sub_mat(0,JiMRTF.columns(),Ravelin::MatrixNd::identity(WS_DOFS));
  workM1.set_sub_mat(JiMRTF.rows(),JiMRTF.columns(),Ravelin::MatrixNd::identity(WS_DOFS));
  M.set_sub_mat(0,0,workM1);
  
  iM.mult((workv2 = fext),workv1,-h,0);
  Rw.mult(workv1,workv2);
  (workv2 += v_bar) -= vel_w;
  
  workv1.set_zero(WS_DOFS*2);
  workv1.set_sub_vec(0,workv2);
  workv1.set_sub_vec(WS_DOFS,workv2.negate());
  q.set_sub_vec(0,workv1);
  
  //  -- Interpenetration --
  // N*iM*[R' F] >= -N*( h*iM*fext + v )
  N.transpose_mult(iMRTF,workM2);
  M.set_sub_mat(WS_DOFS*2,0,workM2);
  
  iM.mult((workv2 = fext),workv1,h,0);
  workv1 += v;
  N.transpose_mult(workv1,workv2);
  workv2.negate();
  q.set_sub_vec(WS_DOFS*2,workv2);
  
  //  -- Coulomb Friction --
  M.set_sub_mat(WS_DOFS*2+NC,0,CF);
  q.set_sub_vec(WS_DOFS*2+NC,Ravelin::VectorNd::zero(NC));
  
  Ravelin::VectorNd ll(NVARS+WS_DOFS),ul(NVARS+WS_DOFS);
  // Lower Torque Limit
  std::fill(ll.begin(), ll.end(), -1e30);
  ll.set_sub_vec(0,Ravelin::VectorNd::zero(NC*5));
  ll.set_sub_vec(NC*5,torque_limits_l);
  
  // Upper Torque Limit
  std::fill(ul.begin(), ul.end(), 1e30);
  ul.set_sub_vec(NC*5,torque_limits_u);
  
  x1.set_zero(c.rows());
  OUTLOG(M,"M",logERROR);
  OUTLOG(q,"q",logERROR);
  OUTLOG(c,"c",logERROR);
  OUTLOG(ll ,"lpl" ,logERROR);
  OUTLOG(ul ,"lpu" ,logERROR);
  OUTLOG(x1 ,"lpx0"  ,logERROR);
  
  Opt::LPParams params;
  params.c = c;
  params.A = A;
  params.b = b;
  params.M = M;
  params.q = q;
  params.l = ll;
  params.u = ul;
  params.n = c.rows();
  unsigned LP_RESULT_FLAG = 0;
  if(!Opt::LP::lp_simplex(params,x1,LP_RESULT_FLAG)){
    OUT_LOG(logERROR)  << "ERROR: Unable to solve widyn LP! : " << LP_RESULT_FLAG ;
    return false;
  }
  
  OUTLOG(x1.get_sub_vec(NVARS,NVARS+WS_DOFS,workv1),"LP: [s]",logERROR);
  OUT_LOG(logERROR) << "LP: Objective: c' x = " << x1.dot(c);
  assert(NC != 4);
  
  // get
  (workv1 = x1).get_sub_vec(0,NVARS,x1);
  OUTLOG(x1,"LP: [z,T]",logERROR);
  x1.get_sub_vec(NC*3,NC*3+nq,x);
  
  iM.mult(fext,workv2,h,0);
  Rw.mult(workv2,workv1);
  OUTLOG(workv1,"dv_fext",logERROR);
  OUTLOG(JiMRTF,"JiMRTF",logERROR);
  
  JiMRTF.mult(x1,workv1,1,1);
  (workv2 = vel_w) += workv1;
  OUTLOG(workv2,"new_OS_GOAL",logERROR);
  v_bar = workv2;
  //    LA_.nullspace(JiMRTF,P);
  //    unsigned size_null_space = P.columns();
  //    if(size_null_space == 0) return true;
  
  // TODO: Add difference check
  (workv1 = qdd) -= new_qdd;
  bool new_traj = (workv1.norm() > NEAR_ZERO);
  if(new_traj){
    qdd = new_qdd;
    return true;
  }
  
  // Trajectory was not updated
  return false;
}
