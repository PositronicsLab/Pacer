
bool inverse_dynamics_ap(const Ravelin::VectorNd& vel, const Ravelin::VectorNd& qdd, const Ravelin::MatrixNd& M,const  Ravelin::MatrixNd& NT,
                         const Ravelin::MatrixNd& D_, const Ravelin::VectorNd& fext, double dt, const Ravelin::MatrixNd& MU, Ravelin::VectorNd& x, Ravelin::VectorNd& cf){
  Ravelin::MatrixNd _workM, _workM2;
  Ravelin::VectorNd _workv, _workv2;
  
  const double NEAR_ZERO = Moby::NEAR_ZERO;
  OUT_LOG(logDEBUG) << ">> inverse_dynamics_ap() entered" << std::endl;
  
  // get number of degrees of freedom and number of contact points
  int n = M.rows();
  int nq = n - 6;
  int nc = NT.columns();
  int nk = D_.columns()/nc;
  
  // setup R
  Ravelin::MatrixNd R(n, nc*5 ),DT = D_,D(n,nc*nk),N = NT;
  D = DT;
  D.transpose();
  N.transpose();
  R.block(0,n,0,nc) = NT;
  R.block(0,n,nc,nc*5) = DT;
  
  OUTLOG(NT,"N'",logDEBUG1);
  OUTLOG(DT,"D'",logDEBUG1);
  
  // compute D, E, and F
  Ravelin::MatrixNd iM_chol = M;
  bool pass = LA_.factor_chol(iM_chol);
  assert(pass);
  
  Ravelin::MatrixNd iM;
  iM = Ravelin::MatrixNd::identity(n);
  LA_.solve_chol_fast(iM_chol,iM);
  
  // | F E'|  =  inv(M)
  // | E D |
  Ravelin::MatrixNd E(6,nq);
  iM.get_sub_mat(nq,n,0,nq,E);
  Ravelin::MatrixNd F(nq,nq);
  iM.get_sub_mat(0,nq,0,nq,F);
  Ravelin::MatrixNd iF;
  iF = F;
  pass = LA_.factor_chol(iF);
  assert(pass);
  
  // compute j and k
  // [F,E']
  Ravelin::MatrixNd FET(F.rows(),F.columns()+E.rows()),
  ET = E;
  ET.transpose();
  FET.set_sub_mat(0,0,F);
  FET.set_sub_mat(0,F.columns(),ET);
  
  Ravelin::VectorNd vq(nq);
  vel.get_sub_vec(0,nq,vq);
  
  Ravelin::VectorNd v = vel;
  iM.mult(fext,v,dt,1);
  
  Ravelin::VectorNd vqstar;
  //  ((vqstar = qdd) *= dt) += vq;
  ((vqstar = qdd) *= dt) += v.segment(0,nq);
  //////////////////////////////////////////////////////////////
  Ravelin::MatrixNd P;
  P.set_zero(nq,n);
  P.set_sub_mat(0,0,Ravelin::MatrixNd::identity(nq));
  Ravelin::MatrixNd PT = P;
  PT.transpose();
  
  OUTLOG(PT,"P'",logDEBUG1);
  
  Ravelin::MatrixNd Cd_iM_CdT,   Cd_iM_CnT, Cd_iM_JxT,
  Cn_iM_CdT,   Cn_iM_CnT,   Cn_iM_JxT,
  /*  Jx_iM_CdT,  Jx_iM_CnT,*/ Jx_iM_JxT;
  
  // S
  LA_.solve_chol_fast(iM_chol,_workM = DT);
  D.mult(_workM,Cd_iM_CdT);
  
  LA_.solve_chol_fast(iM_chol,_workM =  NT);
  D.mult(_workM,Cd_iM_CnT);
  
  LA_.solve_chol_fast(iM_chol,_workM = PT);
  D.mult(_workM,Cd_iM_JxT);
  
  // N
  LA_.solve_chol_fast(iM_chol,_workM = DT);
  N.mult(_workM,Cn_iM_CdT);
  
  LA_.solve_chol_fast(iM_chol,_workM = NT);
  N.mult(_workM,Cn_iM_CnT);
  
  LA_.solve_chol_fast(iM_chol,_workM = PT);
  N.mult(_workM,Cn_iM_JxT);
  
  // P
  //  LA_.solve_chol_fast(iM_chol,_workM = DT);
  //  P.mult(_workM,Jx_iM_CdT);
  
  //  LA_.solve_chol_fast(iM_chol,_workM = NT);
  //  P.mult(_workM,Jx_iM_CnT);
  
  LA_.solve_chol_fast(iM_chol,_workM = PT);
  P.mult(_workM,Jx_iM_JxT);
  
  Ravelin::VectorNd Cd_v, Cn_v, Jx_v;
  D.mult(v,Cd_v);
  N.mult(v,Cn_v);
  P.mult(v,Jx_v);
  
  // Printouts
  OUTLOG(Cd_iM_CdT,"Cd_iM_CdT",logDEBUG1);
  //  OUTLOG(Cd_iM_CnT,"Cd_iM_CnT",logDEBUG1);
  OUTLOG(Cd_iM_JxT ,"Cd_iM_JxT",logDEBUG1);
  
  OUTLOG(Cn_iM_CdT,"Cn_iM_CdT",logDEBUG1);
  OUTLOG(Cn_iM_CnT,"Cn_iM_CnT",logDEBUG1);
  OUTLOG(Cn_iM_JxT ,"Cn_iM_JxT",logDEBUG1);
  
  //  OUTLOG( Jx_iM_CdT,"Jx_iM_CdT",logDEBUG1);
  //  OUTLOG( Jx_iM_CnT,"Jx_iM_CnT",logDEBUG1);
  OUTLOG( Jx_iM_JxT ,"Jx_iM_JxT",logDEBUG1);
  
  
  /////////////////////////////////////////////////////////////////////////////
  /// Solve System
  ///
  const unsigned J_IDX = 0;
  const unsigned N_IDX = 0;
  const unsigned D_IDX = nc;
  const unsigned E_IDX = D_IDX + nc*nk;
  
  // we do this by solving the MLCP:
  // |  A  C  | | x | + | g | = | 0 |
  // |  D  B  | | y |   | h |   | 0 |
  
  // g = [-M*v'; 0; 0; -vq_err]
  // h = [ 0 ];
  // A = [ M -P';
  //       P  0 ];
  
  // B = [ 0  0  0
  //         0  0  E
  //        mu -E' 0 ];
  
  // C = [ -N' -D' 0 ;
  //        0   0  0 ];
  // D = -C'
  // x = [ v^+; tau ]
  // y = [ cN, cD, lambda]
  
  Ravelin::MatrixNd B;
  // lower left & upper right block of matrix
  Ravelin::MatrixNd _LL, _UR;
  if (nk > 4)
    _LL.set_zero(nc*nk,nc+nc*nk);
  else
    _LL.set_zero(nc,nc+nc*nk);
  _UR.set_zero(_LL.columns(),_LL.rows());
  for(unsigned i=0,j=0,r=0;i<nc;i++)
  {
    unsigned NK = MU.columns();
    if (NK > 4)
    {
      assert(nk == NK);
      int nk4 = (NK+4)/4;
      for(unsigned k=0;k<nk4;k++)
      {
        // muK
        _LL(r+k,i)         = MU(i,k);
        // Xe
        _LL(r+k,nc+j)      = -cos((M_PI*k)/(2.0*nk4));
        _LL(r+k,nc+nc+j)   = -cos((M_PI*k)/(2.0*nk4));
        // Xf
        _LL(r+k,nc+nc*2+j) = -sin((M_PI*k)/(2.0*nk4));
        _LL(r+k,nc+nc*3+j) = -sin((M_PI*k)/(2.0*nk4));
        // XeT
        _UR(nc+j,r+k)      =  cos((M_PI*k)/(2.0*nk4));
        _UR(nc+nc+j,r+k)   =  cos((M_PI*k)/(2.0*nk4));
        // XfT
        _UR(nc+nc*2+j,r+k) =  sin((M_PI*k)/(2.0*nk4));
        _UR(nc+nc*3+j,r+k) =  sin((M_PI*k)/(2.0*nk4));
      }
      r+=nk4;
      j++;
    }
    else
    {
      //      assert(NK == 1);
      // muK
      _LL(r,i)         = MU(i,0);
      // Xe
      _LL(r,nc+j)      = -1.0;
      _LL(r,nc+nc+j)   = -1.0;
      // Xf
      _LL(r,nc+nc*2+j) = -1.0;
      _LL(r,nc+nc*3+j) = -1.0;
      // XeT
      _UR(nc+j,r)      =  1.0;
      _UR(nc+nc+j,r)   =  1.0;
      // XfT
      _UR(nc+nc*2+j,r) =  1.0;
      _UR(nc+nc*3+j,r) =  1.0;
      r += 1;
      j++;
    }
  }
  B.set_zero(_LL.rows()+_UR.rows(),_LL.columns()+_UR.columns());
  B.set_sub_mat(_UR.rows(),0,_LL);
  B.set_sub_mat(0,_LL.columns(),_UR);
  
  // Assuming that C is of full row rank (no dependent joint constraints)
  // A is invertible; then we just need to solve the LCP:
  
  // | B - D*inv(A)*C | | v | + | h - D*inv(A)*g | = | w |
  // and use the result to solve for u:
  // u = -inv(A)*(g + Cv)
  
  // A is the matrix | M X'|
  //                 | X 0 |  where X is [ S; T; P ]
  // blockwise inversion yields
  // inv(A) = [
  //    inv(M)-inv(M)*X'*Y*X*inv(M)   inv(M)*X'*Y ;
  //    Y*X*inv(M)                    -Y          ]
  // where Y = inv(X*inv(M)*X')
  
  // defining Q = C and using the result above yields following LCP:
  // matrix: Q*inv(A)*Q' = Q*inv(M)*Q' - Q*inv(M)*X'*Y*X*inv(M)*Q'
  // vector: Q*inv(A)*a  = -(-Q*v + Q*inv(M)*X'*Y*X*v - Q*inv(M)*X'*Y*[0;0;vq*])
  //                     = Q*v - Q*inv(M)*X'*Y*X*v + Q*inv(M)*X'*Y*[0;0;vq*])
  
  Ravelin::MatrixNd _MM,
  _Y = Jx_iM_JxT;
  
  // Invert (chol factorize) matrix _Y
  bool success = _LA.factor_chol(_Y);
  assert(success);
  
  Ravelin::MatrixNd _Q_iM_XT;
  Ravelin::VectorNd _qq;
  
  // inv(A) = [
  //    inv(M)-inv(M)*X'*Y*X*inv(M)   inv(M)*X'*Y ;
  //    Y*X*inv(M)                    -Y          ]
  // defining Y = inv(X*inv(M)*X') and Q = [Cn]
  // matrix:B-D*inv(A)*C = Q*inv(M)*Q' - Q*inv(M)*X'*Y*X*inv(M)*Q'
  //
  // vector:h-Q*inv(A)*a  = -(-Q*inv(M)*M*v - -Q*inv(M)*X'*Y*X*inv(M)*M*v + -Q*inv(M)*X'*Y*[0;0;vq*])
  //                     = -(-Q*v + Q*inv(M)*X'*Y*X*v - Q*inv(M)*X'*Y*[0;0;vq*])
  //                     =    Q*v - Q*inv(M)*X'*Y*X*v + Q*inv(M)*X'*Y*[0;0;vq*]
  
  // setup Q*inv(M)*Q'
  _MM.set_zero(nc+nc*nk+nc,nc+nc*nk+nc);
  _MM.set_sub_mat(0,0,Cn_iM_CnT);
  _MM.set_sub_mat(nc,0,Cd_iM_CnT);
  _MM.set_sub_mat(0,nc,Cn_iM_CdT);
  _MM.set_sub_mat(nc,nc,Cd_iM_CdT);
  
  OUTLOG(_MM,"Q_iM_QT",logDEBUG1);
  
  // setup Q*inv(M)*X'
  // NOTE: with will break if nk != 4
  _Q_iM_XT.resize(nc+nc*nk+nc,nq);
  _Q_iM_XT.set_sub_mat(N_IDX, J_IDX, Cn_iM_JxT);
  _Q_iM_XT.set_sub_mat(D_IDX, J_IDX, Cd_iM_JxT);
  _Q_iM_XT.set_sub_mat(E_IDX, J_IDX, Ravelin::MatrixNd::zero(nc,nq));
  
  OUTLOG(_Q_iM_XT,"Q_iM_XT",logDEBUG1);
  
  //  Ravelin::MatrixNd Y;
  //  Y = Ravelin::MatrixNd::identity(_Y.rows());
  //  _LA.solve_chol_fast(_Y, Y);
  
  // compute Y*X*inv(M)*Q'
  Ravelin::MatrixNd::transpose(_Q_iM_XT, _workM);
  _LA.solve_chol_fast(_Y, _workM);
  
  // compute Q*inv(M)*X'*Y*X*inv(M)*Q'
  _Q_iM_XT.mult(_workM, _workM2);
  _MM  -= _workM2;
  _MM += B;
  OUTLOG(_MM,"MM",logDEBUG1);
  
  // setup -Q*v
  _qq.set_zero(nc*(1+nk+1));
  _qq.set_sub_vec(0,Cn_v);
  _qq.set_sub_vec(nc,Cd_v);
  
  // setup X*v
  Ravelin::VectorNd _Xv,_YXv;
  _Xv.resize(nq);
  _Xv.set_sub_vec(J_IDX, Jx_v);
  
  // compute Y*X*v
  _YXv = _Xv;
  _LA.solve_chol_fast(_Y, _YXv);
  
  // compute Q*inv(M)*X' * Y*X*v
  // & subtract from LCP Vector
  _Q_iM_XT.mult(_YXv, _qq,-1.0,1.0);
  
  
  // compute Q*inv(M)*X'*Y*[vq*]
  Ravelin::VectorNd _Yvqstar;
  _Yvqstar.set_zero(nq);
  _Yvqstar.set_sub_vec(J_IDX, vqstar);
  _LA.solve_chol_fast(_Y, _Yvqstar);
  // & add to LCP Vector
  _Q_iM_XT.mult(_Yvqstar, _qq,1.0,1.0);
  
  OUTLOG(_qq,"qq",logDEBUG1);
  // setup remainder of LCP vector
  
  static Ravelin::VectorNd _v;
  
  OUT_LOG(logERROR) << "-- using: Moby::LCP::lcp_fast" << std::endl;
  
  if (!_lcp.lcp_fast(_MM, _qq, _v))
  {
    OUT_LOG(logERROR) << "Principal pivoting method LCP solver failed; falling back to regularized lemke solver" << std::endl;
    
    if (!_lcp.lcp_lemke_regularized(_MM, _qq, _v,-20,4,1))
      throw std::runtime_error("Unable to solve constraint LCP!");
  }
  OUTLOG(_v,"v",logDEBUG1);
  
  Ravelin::VectorNd tau;
  
  // compute the friction forces
  // u = -inv(A)*(a + Cv)
  // u = inv(A)*(Q'*[cn])  [b/c we don't care about new velocity]
  // recalling that inv(A) =
  // | inv(M)-inv(M)*X'*Y*X*inv(M)   inv(M)*X'*Y | ngc x ngc,    ngc x sz(x)
  // | Y*X*inv(M)                    -Y          | sz(x) x ngc,  sz(x) x sz(x)
  // Q is nlcp x (ngc + sz(x))
  // [cs; ct] = -Y*X*v - Y*X*inv(M)*Q'*[cn]
  tau = _YXv;
  tau -= _Yvqstar;
  _Q_iM_XT.transpose_mult(_v, _workv);
  _LA.solve_chol_fast(_Y, _workv);
  tau += _workv;
  tau.negate();
  
  Ravelin::VectorNd cn,cd;
  // setup impulses
  cn = _v.segment(0, nc);
  cd = _v.segment(nc, nc*(nk+1));
  
  OUTLOG(cn,"cN",logDEBUG);
  OUTLOG(cd,"cD",logDEBUG);
  
  cf.set_zero(nc*(nk+1));
  cf.set_sub_vec(0,cn);
  cf.set_sub_vec(nc,cd);
  
  //   IDYN MLCP
  //            A             C      x         g
  //       | M −S' -T' -P'   -N' | | v+ |   |-M v |    | 0 |
  //       | S  0   0   0     0  | | cS |   |  0  |  = | 0 |
  //       | T  0   0   0     0  | | cT |   |  0  |    | 0 |
  //       | P  0   0   0     0  | | a  |   |-vq* |    | 0 |
  //                              *       +
  //       | N  0   0   0     0  | | cN |   |  0  | >= | 0 |
  //            D            B      y         h
  
  //   Constraints
  //   [M −S' -T' -N' -P'  P' ].[ ]' - M*v  = 0
  //    S v+        = 0
  //    T v+        = 0
  //    P v+ - vq*  = 0
  //    N v+       >= 0
  
  //    M v+ - (S'cs + T'cT + N'cN) - a = M v
  //    M(v+ - v) = (S'cs + T'cT + N'cN)  +  a
  
  //   Therefore
  //    M(v+ - v)           = f_total
  //    S'cs + T'cT + N'cN  = contact_force
  //    f = cf + a
  
  // Using M(dv) - z = tau
  x = tau;
  x /= dt;
  
  OUTLOG(x,"tau",logDEBUG);
  
  OUT_LOG(logDEBUG) << "<< inverse_dynamics_ap() exited" << std::endl;
  return true;
}
