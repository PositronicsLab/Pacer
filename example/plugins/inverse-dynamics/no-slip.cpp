bool inverse_dynamics_no_slip_fast(const Ravelin::VectorNd& vel, const Ravelin::VectorNd& qdd, const Ravelin::MatrixNd& M,const  Ravelin::MatrixNd& nT,
                                   const Ravelin::MatrixNd& D, const Ravelin::VectorNd& fext, double dt, Ravelin::VectorNd& x, Ravelin::VectorNd& cf, bool frictionless, std::vector<unsigned>& indices, int active_eefs,bool SAME_AS_LAST_CONTACTS){
  Ravelin::MatrixNd _workM, _workM2;
  Ravelin::VectorNd _workv, _workv2;
  
  const double NEAR_ZERO = Moby::NEAR_ZERO;
  Ravelin::MatrixNd NT = nT;
  OUT_LOG(logDEBUG) << ">> inverse_dynamics_no_slip_fast() entered" << std::endl;
  
  // get number of degrees of freedom and number of contact points
  int n = M.rows();
  int nq = n - 6;
  int nc = NT.columns();
  int nk = 2;
  
  // setup R
  Ravelin::MatrixNd R(n, nc*5 ),ST(n,nc),TT(n,nc),N(nc,n),S(n,nc),T(n,nc);
  R.block(0,n,0,nc) = NT;
  R.block(0,n,nc,nc*5) = D;
  ST = D.block(0,n,0,nc);
  TT = D.block(0,n,nc,nc*2);
  N = NT;
  N.transpose();
  S = ST;
  T = TT;
  S.transpose();
  T.transpose();
  
  OUTLOG(N,"N",logDEBUG1);
  OUTLOG(S,"S",logDEBUG1);
  OUTLOG(T,"T",logDEBUG1);
  
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
  ((vqstar = qdd) *= dt) += v.segment(0,nq);
  
  //////////////////////////////////////////////////////////////
  Ravelin::MatrixNd P;
  P.set_zero(nq,n);
  P.set_sub_mat(0,0,Ravelin::MatrixNd::identity(nq));
  Ravelin::MatrixNd PT = P;
  PT.transpose();
  
  OUTLOG(P,"P",logDEBUG1);
  
  Ravelin::MatrixNd Cs_iM_CsT, Cs_iM_CtT, /*Cs_iM_CnT,*/ Cs_iM_JxT,
  Ct_iM_CsT, Ct_iM_CtT, /*Ct_iM_CnT,*/ Ct_iM_JxT,
  Cn_iM_CsT, Cn_iM_CtT,   Cn_iM_CnT,   Cn_iM_JxT,
  /*  Jx_iM_CsT,  Jx_iM_CtT,    Jx_iM_CnT,*/  Jx_iM_JxT;
  
  // S
  LA_.solve_chol_fast(iM_chol,_workM = ST);
  S.mult(_workM,Cs_iM_CsT);
  
  LA_.solve_chol_fast(iM_chol,_workM = TT);
  S.mult(_workM,Cs_iM_CtT);
  
  //  LA_.solve_chol_fast(iM_chol,_workM =  NT);
  //  S.mult(_workM,Cs_iM_CnT);
  
  LA_.solve_chol_fast(iM_chol,_workM = PT);
  S.mult(_workM,Cs_iM_JxT);
  
  // T
  LA_.solve_chol_fast(iM_chol,_workM = ST);
  T.mult(_workM,Ct_iM_CsT);
  
  LA_.solve_chol_fast(iM_chol,_workM = TT);
  T.mult(_workM,Ct_iM_CtT);
  
  //  LA_.solve_chol_fast(iM_chol,_workM = NT);
  //  T.mult(_workM,Ct_iM_CnT);
  
  LA_.solve_chol_fast(iM_chol,_workM = PT);
  T.mult(_workM,Ct_iM_JxT);
  
  // N
  LA_.solve_chol_fast(iM_chol,_workM = ST);
  N.mult(_workM,Cn_iM_CsT);
  
  LA_.solve_chol_fast(iM_chol,_workM = TT);
  N.mult(_workM,Cn_iM_CtT);
  
  LA_.solve_chol_fast(iM_chol,_workM = NT);
  N.mult(_workM,Cn_iM_CnT);
  
  LA_.solve_chol_fast(iM_chol,_workM = PT);
  N.mult(_workM,Cn_iM_JxT);
  
  // P
  //  LA_.solve_chol_fast(iM_chol,_workM = ST);
  //  P.mult(_workM,Jx_iM_CsT);
  
  //  LA_.solve_chol_fast(iM_chol,_workM = TT);
  //  P.mult(_workM,Jx_iM_CtT);
  
  //  LA_.solve_chol_fast(iM_chol,_workM = NT);
  //  P.mult(_workM,Jx_iM_CnT);
  
  LA_.solve_chol_fast(iM_chol,_workM = PT);
  P.mult(_workM,Jx_iM_JxT);
  
  Ravelin::VectorNd Cs_v, Ct_v, Cn_v, Jx_v;
  S.mult(v,Cs_v);
  T.mult(v,Ct_v);
  N.mult(v,Cn_v);
  P.mult(v,Jx_v);
  
  OUTLOG(Cn_v,"Cn_v",logDEBUG1);
  OUTLOG(Cs_v,"Cs_v",logDEBUG1);
  OUTLOG(Ct_v,"Ct_v",logDEBUG1);
  
  // Printouts
  OUTLOG(Cs_iM_CsT,"Cs_iM_CsT",logDEBUG1);
  OUTLOG(Cs_iM_CtT,"Cs_iM_CtT",logDEBUG1);
  //  OUTLOG(Cs_iM_CnT,"Cs_iM_CnT",logDEBUG1);
  OUTLOG(Cs_iM_JxT ,"Cs_iM_JxT",logDEBUG1);
  
  OUTLOG(Ct_iM_CsT,"Ct_iM_CsT",logDEBUG1);
  OUTLOG(Ct_iM_CtT,"Ct_iM_CtT",logDEBUG1);
  //  OUTLOG(Ct_iM_CnT,"Ct_iM_CnT",logDEBUG1);
  OUTLOG(Ct_iM_JxT ,"Ct_iM_JxT",logDEBUG1);
  
  OUTLOG(Cn_iM_CsT,"Cn_iM_CsT",logDEBUG1);
  OUTLOG(Cn_iM_CtT,"Cn_iM_CtT",logDEBUG1);
  OUTLOG(Cn_iM_CnT,"Cn_iM_CnT",logDEBUG1);
  OUTLOG(Cn_iM_JxT ,"Cn_iM_JxT",logDEBUG1);
  
  //  OUTLOG( Jx_iM_CsT,"Jx_iM_CsT",logDEBUG1);
  //  OUTLOG( Jx_iM_CtT,"Jx_iM_CtT",logDEBUG1);
  //  OUTLOG( Jx_iM_CnT,"Jx_iM_CnT",logDEBUG1);
  OUTLOG( Jx_iM_JxT ,"Jx_iM_JxT",logDEBUG1);
  
  /////////////////////////////////////////////////////////////////////////////
  /// Solve System
  ///
  std::vector<unsigned> S_indices,T_indices,J_indices;
  const unsigned N_IDX = 0;
  
  // we do this by solving the MLCP:
  // |  A  C  | | x | + | g | = | 0 |
  // |  D  B  | | y |   | h |   | 0 |
  
  // g = [-M*v'; 0; 0; -vq_err]
  // h = [ 0 ];
  // A = [ M -S' -T' -P';
  //       S  0   0   0 ;
  //       T  0   0   0 ;
  //       P  0   0   0 ];
  
  // B = [ 0 ];
  
  // C = [ -N';
  //        0 ;
  //        0 ];
  // D = -C'
  // x = [ v^+; cs; ct; a ]
  // y = [ cN]
  
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
  
  Ravelin::MatrixNd _MM,_Y;
  // ********************************************************
  // find largest non-singular set of J, S, and T indices
  // ********************************************************
  
  // loop through joint constraints, forming J*inv(M)*J' and checking condition
  for (unsigned i=0; i< nq; i++)
  {
    // add the index tentatively to the set
    J_indices.push_back(i);
    
    // This is an [I,0] matrix, it has full row rank
    continue;
  }
  
  // get the reduced Jx*iM*Jx' matrix
  Ravelin::MatrixNd _rJx_iM_JxT = Jx_iM_JxT;
  
  // loop through contacts, forming matrix below and checking its condition
  // | S*inv(M)*S'  S*inv(M)*T' S*inv(M)*J' |
  // | T*inv(M)*S'  T*inv(M)*T' T*inv(M)*J' |
  // | J*inv(M)*S'  J*inv(M)*T' J*inv(M)*J' |
  
  bool last_success = false;
  for (unsigned i=0; i< nc; i++)
  {
    // update S indices
    S_indices.push_back(i);
    
    // setup indices
    unsigned S_IDX;
    S_IDX = 0;
    unsigned T_IDX;
    T_IDX = S_indices.size();
    unsigned J_IDX;
    J_IDX = T_IDX + T_indices.size();
    _Y.resize(J_IDX + J_indices.size(), J_IDX + J_indices.size());
    
    // add S/S, T/T, J/J components to 'check' matrix
    Cs_iM_CsT.select_square(S_indices.begin(), S_indices.end(), _workM);
    _Y.set_sub_mat(S_IDX, S_IDX, _workM);
    Ct_iM_CtT.select_square(T_indices.begin(), T_indices.end(), _workM);
    _Y.set_sub_mat(T_IDX, T_IDX, _workM);
    _Y.set_sub_mat(J_IDX, J_IDX, _rJx_iM_JxT);
    
    // add S/T components to 'check' matrix
    Cs_iM_CtT.select(S_indices.begin(), S_indices.end(), T_indices.begin(), T_indices.end(), _workM);
    _Y.set_sub_mat(S_IDX, T_IDX, _workM);
    _Y.set_sub_mat(T_IDX, S_IDX, _workM, Ravelin::eTranspose);
    
    // add S/J components to check matrix
    Cs_iM_JxT.select(S_indices.begin(), S_indices.end(), J_indices.begin(), J_indices.end(), _workM);
    _Y.set_sub_mat(S_IDX, J_IDX, _workM);
    _Y.set_sub_mat(J_IDX, S_IDX, _workM, Ravelin::eTranspose);
    
    // add T/J components to check matrix
    Ct_iM_JxT.select(T_indices.begin(), T_indices.end(), J_indices.begin(), J_indices.end(), _workM);
    _Y.set_sub_mat(T_IDX, J_IDX, _workM);
    _Y.set_sub_mat(J_IDX, T_IDX, _workM, Ravelin::eTranspose);
    
    // skew the matrix away from positive definiteness
    for (unsigned j=0; j< _Y.rows(); j++)
      _Y(j,j) -= NEAR_ZERO;
    
    // see whether check matrix can be Cholesky factorized
    if (!_LA.factor_chol(_Y) || frictionless)
      S_indices.pop_back();
    
    // add index for T
    T_indices.push_back(i);
    
    // resize the check matrix
    T_IDX = S_indices.size();
    J_IDX = T_IDX + T_indices.size();
    _Y.resize(J_IDX + J_indices.size(), J_IDX + J_indices.size());
    
    // add S/S, T/T, J/J components to 'check' matrix
    Cs_iM_CsT.select_square(S_indices.begin(), S_indices.end(), _workM);
    _Y.set_sub_mat(S_IDX, S_IDX, _workM);
    Ct_iM_CtT.select_square(T_indices.begin(), T_indices.end(), _workM);
    _Y.set_sub_mat(T_IDX, T_IDX, _workM);
    _Y.set_sub_mat(J_IDX, J_IDX, _rJx_iM_JxT);
    
    // add S/T components to 'check' matrix
    Cs_iM_CtT.select(S_indices.begin(), S_indices.end(), T_indices.begin(), T_indices.end(), _workM);
    _Y.set_sub_mat(S_IDX, T_IDX, _workM);
    _Y.set_sub_mat(T_IDX, S_IDX, _workM, Ravelin::eTranspose);
    
    // add S/J components to check matrix
    Cs_iM_JxT.select(S_indices.begin(), S_indices.end(), J_indices.begin(), J_indices.end(), _workM);
    _Y.set_sub_mat(S_IDX, J_IDX, _workM);
    _Y.set_sub_mat(J_IDX, S_IDX, _workM, Ravelin::eTranspose);
    
    // add T/J components to check matrix
    Ct_iM_JxT.select(T_indices.begin(), T_indices.end(), J_indices.begin(), J_indices.end(), _workM);
    _Y.set_sub_mat(T_IDX, J_IDX, _workM);
    _Y.set_sub_mat(J_IDX, T_IDX, _workM, Ravelin::eTranspose);
    
    // skew the matrix away from positive definiteness
    for (unsigned j=0; j< _Y.rows(); j++)
      _Y(j,j) -= NEAR_ZERO;
    
    // see whether check matrix can be Cholesky factorized
    last_success = _LA.factor_chol(_Y);
    if (!last_success && !frictionless)
      T_indices.pop_back();
  }
  
  last_success = false;
  // output indices
  if (true)
  {
    std::ostringstream oss;
    oss << "s indices:";
    for (unsigned i=0; i< S_indices.size(); i++)
      oss << " " << S_indices[i];
    oss << "  t indices:";
    for (unsigned i=0; i< T_indices.size(); i++)
      oss << " " << T_indices[i];
    oss << "  j indices:";
    for (unsigned i=0; i< J_indices.size(); i++)
      oss << " " << J_indices[i];
    
    OUT_LOG(logDEBUG) << oss.str() << std::endl;
  }
  
  Ravelin::MatrixNd _X;
  
  // ********************************************************
  // reform Y if necessary
  // ********************************************************
  
  // setup indices
  unsigned S_IDX;
  S_IDX = 0;
  unsigned T_IDX;
  T_IDX = S_indices.size();
  unsigned J_IDX;
  J_IDX = T_IDX + T_indices.size();
  
  if (!last_success)
  {
    _Y.resize(J_IDX + J_indices.size(), J_IDX + J_indices.size());
    
    // add S/S, T/T, J/J components to X
    Cs_iM_CsT.select_square(S_indices.begin(), S_indices.end(), _workM);
    _Y.set_sub_mat(S_IDX, S_IDX, _workM);
    Ct_iM_CtT.select_square(T_indices.begin(), T_indices.end(), _workM);
    _Y.set_sub_mat(T_IDX, T_IDX, _workM);
    _Y.set_sub_mat(J_IDX, J_IDX, _rJx_iM_JxT);
    
    // add S/T components to X
    Cs_iM_CtT.select(S_indices.begin(), S_indices.end(), T_indices.begin(), T_indices.end(), _workM);
    _Y.set_sub_mat(S_IDX, T_IDX, _workM);
    _Y.set_sub_mat(T_IDX, S_IDX, _workM, Ravelin::eTranspose);
    
    // add S/J components to X
    Cs_iM_JxT.select(S_indices.begin(), S_indices.end(), J_indices.begin(), J_indices.end(), _workM);
    _Y.set_sub_mat(S_IDX, J_IDX, _workM);
    _Y.set_sub_mat(J_IDX, S_IDX, _workM, Ravelin::eTranspose);
    
    // add T/J components to X
    Ct_iM_JxT.select(T_indices.begin(), T_indices.end(), J_indices.begin(), J_indices.end(), _workM);
    _Y.set_sub_mat(T_IDX, J_IDX, _workM);
    _Y.set_sub_mat(J_IDX, T_IDX, _workM, Ravelin::eTranspose);
    
  }
  
  // do the Cholesky factorization (should not fail)
  
  // check the condition number on Y
  // TODO: remove this code when satisfied Cholesky factorization is not problem
  Ravelin::MatrixNd tmp = _Y;
  double cond = _LA.cond(tmp);
  if (cond > 1e6){
    OUT_LOG(logERROR) << "Condition number *may* be high (check!): " << cond << std::endl;
    bool success = _LA.factor_chol(_Y);
    assert(success);
  } else {
    bool success = _LA.factor_chol(_Y);
    assert(success);
  }
  
  
  Ravelin::MatrixNd _Q_iM_XT;
  Ravelin::VectorNd _qq;
  
  // inv(A) = [
  //    inv(M)-inv(M)*X'*Y*X*inv(M)   inv(M)*X'*Y ;
  //    Y*X*inv(M)                    -Y          ]
  // defining Y = inv(X*inv(M)*X') and Q = [Cn]
  // matrix:-D*inv(A)*C = Q*inv(M)*Q' - Q*inv(M)*X'*Y*X*inv(M)*Q'
  //
  // vector:-Q*inv(A)*a  = -(-Q*inv(M)*M*v - -Q*inv(M)*X'*Y*X*inv(M)*M*v + -Q*inv(M)*X'*Y*[0;0;vq*])
  //                     = -(-Q*v + Q*inv(M)*X'*Y*X*v - Q*inv(M)*X'*Y*[0;0;vq*])
  //                     =    Q*v - Q*inv(M)*X'*Y*X*v + Q*inv(M)*X'*Y*[0;0;vq*]
  
  // setup Q*inv(M)*Q'
  //  _MM = Cn_iM_CnT;
  
  OUTLOG(Cn_iM_CnT,"A",logDEBUG1);
  
  // setup Q*inv(M)*X'
  _Q_iM_XT.resize(nc, S_indices.size() + T_indices.size() + J_indices.size());
  Cn_iM_CsT.select_columns(S_indices.begin(), S_indices.end(), _workM);
  _Q_iM_XT.set_sub_mat(N_IDX, S_IDX, _workM);
  Cn_iM_CtT.select_columns(T_indices.begin(), T_indices.end(), _workM);
  _Q_iM_XT.set_sub_mat(N_IDX, T_IDX, _workM);
  Cn_iM_JxT.select_columns(J_indices.begin(), J_indices.end(), _workM);
  _Q_iM_XT.set_sub_mat(N_IDX, J_IDX, _workM);
  
  OUTLOG(_Q_iM_XT,"B",logDEBUG1);
  
  //  Ravelin::MatrixNd Y;
  //  Y = Ravelin::MatrixNd::identity(_Y.rows());
  //  _LA.solve_chol_fast(_Y, Y);
  
  // compute Y*X*inv(M)*Q'
  Ravelin::MatrixNd::transpose(_Q_iM_XT, _workM);
  _LA.solve_chol_fast(_Y, _workM);
  //  Y.mult_transpose(_Q_iM_XT,_workM);
  
  // compute Q*inv(M)*X'*Y*X*inv(M)*Q'
  _Q_iM_XT.mult(_workM, _workM2);
  (_MM = Cn_iM_CnT) -= _workM2;
  OUTLOG(_MM,"MM",logDEBUG1);
  
  // setup -Q*v
  _qq = Cn_v;
  
  // setup X*v
  Ravelin::VectorNd _Xv,_YXv;
  _Xv.resize(S_indices.size() + T_indices.size() + J_indices.size());
  Cs_v.select(S_indices.begin(), S_indices.end(), _workv);
  _Xv.set_sub_vec(S_IDX, _workv);
  Ct_v.select(T_indices.begin(), T_indices.end(), _workv);
  _Xv.set_sub_vec(T_IDX, _workv);
  Jx_v.select(J_indices.begin(), J_indices.end(), _workv);
  _Xv.set_sub_vec(J_IDX, _workv);
  
  // compute Y*X*v
  _YXv = _Xv;
  _LA.solve_chol_fast(_Y, _YXv);
  
  // compute Q*inv(M)*X' * Y*X*v
  _Q_iM_XT.mult(_YXv, _workv);
  
  // Add to LCP Vector
  _qq -= _workv;
  
  // compute Q*inv(M)*X'*Y*[0;0;vq*]
  Ravelin::VectorNd _Yvqstar;
  _Yvqstar.set_zero(S_indices.size() + T_indices.size() + J_indices.size());
  
  vqstar.select(J_indices.begin(), J_indices.end(), _workv);
  _Yvqstar.set_sub_vec(J_IDX, _workv);
  _LA.solve_chol_fast(_Y, _Yvqstar);
  _Q_iM_XT.mult(_Yvqstar,_workv);
  _qq += _workv;
  
  OUTLOG(_qq,"qq",logDEBUG1);
  // setup remainder of LCP vector
  
  static Ravelin::VectorNd _v;
  if(_v.size() != _qq.size() || !SAME_AS_LAST_CONTACTS)
    _v.resize(0);
  
  // attempt to solve the LCP using the fast method
  if(active_eefs > 2){
    OUT_LOG(logERROR) << "-- using: lcp_fast" << std::endl;
    OUTLOG(_v,"warm_start_v",logDEBUG1);
    
    if (!Utility::lcp_fast(_MM, _qq,indices, _v,Moby::NEAR_ZERO))
    {
      OUT_LOG(logERROR) << "-- Principal pivoting method LCP solver failed; falling back to regularized lemke solver" << std::endl;
      
      if (!_lcp.lcp_lemke_regularized(_MM, _qq, _v,-20,4,1))
        throw std::runtime_error("Unable to solve constraint LCP!");
    }
    
  } else {
    OUT_LOG(logERROR) << "-- using: Moby::LCP::lcp_fast" << std::endl;
    
    if (!_lcp.lcp_fast(_MM, _qq, _v))
    {
      OUT_LOG(logERROR) << "Principal pivoting method LCP solver failed; falling back to regularized lemke solver" << std::endl;
      
      if (!_lcp.lcp_lemke_regularized(_MM, _qq, _v,-20,4,1))
        throw std::runtime_error("Unable to solve constraint LCP!");
    }
  }
  OUTLOG(_v,"v",logERROR);
  
  Ravelin::VectorNd _cs_ct_tau, _v_plus;
  
  // u = inv(A)*(Q'*[cn])  [b/c we don't care about new velocity]
  _cs_ct_tau = _YXv;
  _cs_ct_tau -= _Yvqstar;
  _Q_iM_XT.transpose_mult(_v, _workv);
  LA_.solve_chol_fast(_Y, _workv);
  _cs_ct_tau += _workv;
  _cs_ct_tau.negate();
  
  OUTLOG(_cs_ct_tau,"cs_ct_tau",logDEBUG1);
  
  
  Ravelin::VectorNd cn,cs,ct,tau;
  // setup impulses
  cn = _v.segment(0, nc);
  cs.set_zero(nc);
  ct.set_zero(nc);
  tau.set_zero(nq);
  Ravelin::SharedConstVectorNd cs_vec = _cs_ct_tau.segment(S_IDX, T_IDX);
  Ravelin::SharedConstVectorNd ct_vec = _cs_ct_tau.segment(T_IDX, J_IDX);
  Ravelin::SharedConstVectorNd a_vec = _cs_ct_tau.segment(J_IDX, _cs_ct_tau.rows());
  cs.set(S_indices.begin(), S_indices.end(), cs_vec);
  ct.set(T_indices.begin(), T_indices.end(), ct_vec);
  tau.set(J_indices.begin(), J_indices.end(), a_vec);
  
  OUTLOG(cn,"cN",logDEBUG);
  OUTLOG(cs,"cS",logDEBUG);
  OUTLOG(ct,"cT",logDEBUG);
  
  cf.set_zero(nc*5);
  for(unsigned i=0;i< nc;i++){
    cf[i] = cn[i];
    if(ct[i] >= 0)
      cf[nc+i] = cs[i];
    else
      cf[nc+i+nc*2] = -cs[i];
    if(ct[i] >= 0)
      cf[nc+i+nc] = ct[i];
    else
      cf[nc+i+nc*3] = -ct[i];
  }
  
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
  
  
  OUT_LOG(logDEBUG) << "<< inverse_dynamics_no_slip_fast() exited" << std::endl;
  return true;
}