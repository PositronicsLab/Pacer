/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 *
 * This file implements a number of inverse dynamics controllers that leverage
 * contact force predictions to compute actuator forces for the robot.
 ****************************************************************************/

/** Maximal Dissipation Model (contact only)
 *  min{z}  v+' M v+
 *  such that:
 *  v+ = v- + inv(M)([ N, ST+, ST- ] z)
 *  N' v+ >= 0
 *  f_N >= 0
 *  mu * f_N{i} >= 1' [f_S{i}; f_T{i}] for i=1:nc
 **/
bool predict_contact_forces(const Ravelin::VectorNd& v, const Ravelin::MatrixNd& M,const  Ravelin::MatrixNd& N,
                            const Ravelin::MatrixNd& ST, const Ravelin::MatrixNd& MU, Ravelin::VectorNd& z){
  Ravelin::MatrixNd workM1,workM2;
  Ravelin::VectorNd workv1, workv2;
  
  OUT_LOG(logDEBUG) << ">> inverse_dynamics() entered" << std::endl;
  
  // get number of degrees of freedom and number of contact points
  int n = M.rows();
  int nq = n - 6;
  int nc = N.columns();
  
  // Invert M
  Ravelin::MatrixNd iM_chol;
  iM_chol = M;
  if(!LA_.factor_chol(iM_chol)){
    OUTLOG(M,"M",logDEBUG1);
    throw std::runtime_error("Chol Factorization Failed on M");
  }
  Ravelin::MatrixNd iM = Ravelin::MatrixNd::identity(n);
  LA_.solve_chol_fast(iM_chol,iM);
  
  // Compute Jacobians
  int nk = (nc == 0)? 0 : ST.columns()/nc;
  int nvars = nc + nc*(nk);
  // setup R
  Ravelin::MatrixNd R(n, nc + (nc*nk) );
  R.block(0,n,0,nc) = N;
  R.block(0,n,nc,nc*nk+nc) = ST;
  
  if(nc > 0){
    
    /////////////////////////////////////////////////////////////////////////////
    ///////////////// Stage 1 optimization:  IDYN Energy Min ////////////////////
    /////////////////////////////////////////////////////////////////////////////
    
    /////////////////////////////// OBJECTIVE ///////////////////////////////////
    // MIN: [z]' R' iM' M iM R [z] + v' M iM R [z]
    // Matrix: G = R' iM' M iM R
    //             --(iM' M iM  == iM)-->
    // G = R' M R
    Ravelin::MatrixNd G,iMR;
    R.transpose_mult(iM.mult(R,iMR),G);
    
    // c = v' M iM R [z]
    //    --( M iM R == R)-->
    // c = v' R [z]
    Ravelin::VectorNd c;
    R.transpose_mult(v,c);
    
    OUTLOG(G,"G",logDEBUG1);
    OUTLOG(c,"c",logDEBUG1);
    
    ////////////////////////////// CONSTRAINTS ///////////////////////////////////
    
    // setup linear inequality constraints -- noninterpenetration
    // N v+ >= 0
    // N Y R z + NY(Mv + hfext)>=0
    Ravelin::MatrixNd A1;
    Ravelin::VectorNd b1;
    N.transpose_mult(iMR,A1);
    N.transpose_mult(v,b1,-1,0);
    OUTLOG(A1,"A_noninterpen",logDEBUG1);
    OUTLOG(b1,"b_noninterpen",logDEBUG1);
    
    // setup linear inequality constraint -- coulomb friction
    Ravelin::MatrixNd A2;
    Ravelin::VectorNd b2;
    
    // inscribe friction polygon in friction cone (scale by cos(pi/nk))
    if(nk == 4){
      A2.set_zero(nc, nvars);
      b2.set_zero(nc);
      for (int ii=0;ii < nc;ii++){
        // normal force
        A2(ii,ii) = MU(ii,0);
        // tangent forces [polygonal]
        for(int kk=nc+ii;kk<nc+nk*nc;kk+=nc)
          A2(ii,kk) = -1.0;
      }
    } else {
      A2.set_zero(nc*nk/2, nvars);
      b2.set_zero(nc*nk/2);
      double polygon_rad = cos(M_PI/nk);
      // for each Contact
      for (int ii=0;ii < nc;ii++){
        // for each Friction Direction
        for(int k=0;k<nk/2;k++){
          // normal force
          A2(ii*nk/2+k,ii) = MU(ii,k)*polygon_rad;
          // tangent forces [polygonal]
          for(int kk=nc+ii+nc*k;kk<nc+nk*nc;kk+=nc*nk/2)
            A2(ii*nk/2+k,kk) = -1.0;
        }
      }
    }
    OUTLOG(b2,"b_friction",logDEBUG1);
    OUTLOG(A2,"A_friction",logDEBUG1);
    
    // combine all linear inequality constraints
    assert(A1.columns() == A2.columns());
    Ravelin::MatrixNd A(A1.rows()+A2.rows(),A1.columns());
    Ravelin::VectorNd b(b1.rows()+b2.rows());
    A.block(0,A1.rows(),0,A1.columns()) = A1;
    A.block(A1.rows(),A1.rows()+A2.rows(),0,A2.columns()) = A2;
    b.segment(0,b1.rows()) = b1;
    b.segment(b1.rows(),b1.rows()+b2.rows()) = b2;
    
    /// Stage 1 optimization energy minimization
    z.set_zero(nvars);
    
    static Ravelin::VectorNd _v;
    if(!Utility::solve_qp_pos(G,c,A,b,z,_v,false)){
      OUT_LOG(logERROR)  << "%ERROR: Unable to solve stage 1!";
      return false;
    }
    
    OUTLOG(z,"Z_OP1",logDEBUG1);
    // measure feasibility of solution
    // qM z - qq >= 0
    Ravelin::VectorNd feas;
    A.mult(z,feas) -= b;
    
    OUTLOG(feas,"feas_OP1 =[ % (A*z-b >= 0)",logDEBUG1);
    
    if(feas.rows() > 0){
      double min_elem = *std::min_element(feas.begin(), feas.end());
      if(min_elem < -NEAR_ZERO){
        OUT_LOG(logERROR)  << "ERROR: Optimization 1 produced an infeasible result!" << min_elem;
        return false;
      }
    }
    
    // DEBUGGING OUTPUT
    if (LOG(logDEBUG1)) {
      OUTLOG(v,"v- (pre-constraint)",logERROR);
      Ravelin::VectorNd v_plus = v;
      iM.mult(R.mult(z,workv1),v_plus,1,1);
      OUTLOG(v_plus,"v+ (post-constraint)",logERROR);
      OUTLOG(0.5*M.mult(v_plus,workv1).dot(v_plus),"KE ",logERROR);
      
      OUTLOG(c.dot(z) + 0.5*G.mult(z,workv1).dot(z),"KE dissipation (contact solve)",logERROR);
    }
  } else {
    z.set_zero(0);
  }
  return true;
}

/** Maximal Dissipation Model (contact + inverse dynamics) -- Simple setup
 *  min{z}  v+' M v+
 *  such that:
 *  v+ = v + inv(M)([ N, ST+, ST- ] z + h fext + P tau)
 *  N' v+ >= 0
 *  f_N >= 0
 *  mu * f_N{i} >= 1' [f_S{i}; f_T{i}] for i=1:nc
 *  P' v+ = v + h qdd
 **/
bool inverse_dynamics_two_stage_simple(const Ravelin::VectorNd& v, const Ravelin::VectorNd& qdd, const Ravelin::MatrixNd& M,const  Ravelin::MatrixNd& N, const Ravelin::MatrixNd& ST, const Ravelin::VectorNd& fext, double h, const Ravelin::MatrixNd& MU, Ravelin::VectorNd& x, Ravelin::VectorNd& cf_final){
  Ravelin::MatrixNd workM1,workM2;
  Ravelin::VectorNd workv1, workv2;
  
  OUT_LOG(logDEBUG) << ">> inverse_dynamics() entered" << std::endl;
  
  // get number of degrees of freedom and number of contact points
  int n = M.rows();
  int nq = n - 6;
  int nc = N.columns();
  
  Ravelin::VectorNd vq = v.segment(0,nq);
  Ravelin::VectorNd vb = v.segment(nq,n);
  
  Ravelin::VectorNd vqstar;
  ((vqstar = qdd) *= h) += vq;
  
  // Invert M
  Ravelin::MatrixNd iM_chol;
  iM_chol = M;
  if(!LA_.factor_chol(iM_chol)){
    OUTLOG(M,"M",logDEBUG1);
    throw std::runtime_error("Chol Factorization Failed on M");
  }
  Ravelin::MatrixNd iM = Ravelin::MatrixNd::identity(n);
  LA_.solve_chol_fast(iM_chol,iM);
  
  
  // P selection matrix
  Ravelin::MatrixNd P = Ravelin::MatrixNd::zero(nq,n);
  P.block(0,nq,0,nq) = Ravelin::MatrixNd::identity(nq);
  
  // X matrix
  // | M  P'|
  // | P  0 |
  Ravelin::MatrixNd X = Ravelin::MatrixNd::zero(n+nq,n+nq);
  X.set_sub_mat(0,0,M);
  X.set_sub_mat(n,0,P);
  X.set_sub_mat(0,n,P,Ravelin::eTranspose);
  
  // inv(X) matrix
  // | gamma        delta    |
  // | delta'       epsilon  |
  
  Ravelin::MatrixNd delta, gamma, iMPT,PiMPT,epsilon;
  // inv(M) P'
  workM1 = P;
  workM1.transpose();
  LA_.solve_chol_fast(iM_chol,iMPT = workM1);
  
  // P inv(M) P'
  P.mult(iMPT,PiMPT);
  
  // inv((P inv(M) P')')
  epsilon = Ravelin::MatrixNd::identity(nq);
  LA_.solve_fast(workM1 = PiMPT,epsilon);
  
  // delta = inv(M) P' inv((P inv(M) P')')
  iMPT.mult(epsilon,delta);
  
  // epsilon = -inv((P inv(M) P')')
  epsilon.negate();
  
  // gamma = inv(M) - delta P inv(M)
  delta.mult_transpose(iMPT,gamma=iM,-1,1);
  
  // Rebuild
  Ravelin::MatrixNd iX(n+nq,n+nq);
  iX.set_sub_mat(0,0,gamma);
  iX.set_sub_mat(n,0,delta,Ravelin::eTranspose);
  iX.set_sub_mat(0,n,delta);
  iX.set_sub_mat(n,n,epsilon);
#ifdef NDEBUG
  // DEBUGGING OUTPUT
  if (LOG(logDEBUG1)) {
    // Solve Inversion Method
    //  Ravelin::MatrixNd iX2 = Ravelin::MatrixNd::identity(n+nq);
    //  LA_.solve_fast(X,iX2);
    
    // Symmetric inversion method
    Ravelin::MatrixNd iX2 = X;
    LA_.inverse_symmetric(iX2);
    
    OUTLOG((iX2-=iX).norm_inf(),"inversion accuracy: iX",logDEBUG1);
    
    OUTLOG(gamma,"gamma",logDEBUG1);
    
    //  OUTLOG(gamma.mult(M.mult(gamma,workM1),workM2),"gamma == gamma*M*gamma",logERROR);
    OUTLOG(gamma.mult(M.mult(gamma,workM1),workM2 = gamma,-1,1).norm_inf(),"0 == gamma - gamma*M*gamma",logDEBUG1);
    OUTLOG(gamma.mult(M.mult(delta,workM1),workM2).norm_inf(),"0 == gamma*M*delta",logDEBUG1);
  }
#endif
  // Compute Jacobians
  int nk = (nc == 0)? 0 : ST.columns()/nc;
  int nvars = nc + nc*(nk);
  // setup R
  Ravelin::MatrixNd R(n, nc + (nc*nk) );
  R.block(0,n,0,nc) = N;
  R.block(0,n,nc,nc*nk+nc) = ST;
  
  // Calculate Mv + hFext
  Ravelin::VectorNd Mv_fext;
  M.mult(v,Mv_fext = fext,1,h);
  
  OUTLOG(Mv_fext,"v",logDEBUG1);
  OUTLOG(N,"N",logDEBUG1);
  OUTLOG(ST.block(0,n,0,nc*2),"D",logDEBUG1);
  OUTLOG(M,"M",logDEBUG1);
  OUTLOG(P,"P",logDEBUG1);
  
  if(nc > 0){
    
    /////////////////////////////////////////////////////////////////////////////
    ///////////////// Stage 1 optimization:  IDYN Energy Min ////////////////////
    /////////////////////////////////////////////////////////////////////////////
    
    /////////////////////////////// OBJECTIVE ///////////////////////////////////
    // MIN: [z]' R' gamma' M gamma R [z] + ( (v' M' + h fext) gamma')M gamma R [z]
    // Matrix: G = R' gamma' M gamma R
    //    --(gamma' M gamma  == gamma)-->
    // G = R' gamma R
    Ravelin::MatrixNd G, YR;
    R.transpose_mult(gamma.mult(R,YR),G);
    
    // Vector: c = ( (v' M' + h fext)' gamma' + (vq*)' delta' )M gamma R
    //    --(gamma' M gamma  == gamma)-->
    // c = ( (v' M' + h fext) gamma R
    
    Ravelin::VectorNd c, vqstar_delta_M;
    YR.transpose_mult(Mv_fext,c);
    //    --(delta' M gamma  == 0)-->
    //        R.transpose_mult( gamma.transpose_mult( M.mult(delta.mult(vqstar,vqstar_delta_M),workv1),workv2),c,1,1);
    OUTLOG(G,"G",logDEBUG1);
    OUTLOG(c,"c",logDEBUG1);
    
    ////////////////////////////// CONSTRAINTS ///////////////////////////////////
    
    // setup linear inequality constraints -- noninterpenetration
    // N v+ >= 0
    // N Y R z + NY(Mv + hfext)>=0
    Ravelin::MatrixNd A1;
    Ravelin::VectorNd b1;
    N.transpose_mult(YR,A1);
    N.transpose_mult(gamma.mult(Mv_fext,workv1),b1,-1,0);
    N.transpose_mult(delta.mult(vqstar,workv1),b1,-1,1);
    OUTLOG(A1,"A_noninterpen",logDEBUG1);
    OUTLOG(b1,"b_noninterpen",logDEBUG1);
    
    // setup linear inequality constraint -- coulomb friction
    Ravelin::MatrixNd A2;
    Ravelin::VectorNd b2;
    
    // inscribe friction polygon in friction cone (scale by cos(pi/nk))
    if(nk == 4){
      A2.set_zero(nc, nvars);
      b2.set_zero(nc);
      for (int ii=0;ii < nc;ii++){
        // normal force
        A2(ii,ii) = MU(ii,0);
        // tangent forces [polygonal]
        for(int kk=nc+ii;kk<nc+nk*nc;kk+=nc)
          A2(ii,kk) = -1.0;
      }
    } else {
      A2.set_zero(nc*nk/2, nvars);
      b2.set_zero(nc*nk/2);
      double polygon_rad = cos(M_PI/nk);
      // for each Contact
      for (int ii=0;ii < nc;ii++){
        // for each Friction Direction
        for(int k=0;k<nk/2;k++){
          // normal force
          A2(ii*nk/2+k,ii) = MU(ii,k)*polygon_rad;
          // tangent forces [polygonal]
          for(int kk=nc+ii+nc*k;kk<nc+nk*nc;kk+=nc*nk/2)
            A2(ii*nk/2+k,kk) = -1.0;
        }
      }
    }
    OUTLOG(b2,"b_friction",logDEBUG1);
    OUTLOG(A2,"A_friction",logDEBUG1);
    
    // combine all linear inequality constraints
    assert(A1.columns() == A2.columns());
    Ravelin::MatrixNd A(A1.rows()+A2.rows(),A1.columns());
    Ravelin::VectorNd b(b1.rows()+b2.rows());
    A.block(0,A1.rows(),0,A1.columns()) = A1;
    A.block(A1.rows(),A1.rows()+A2.rows(),0,A2.columns()) = A2;
    b.segment(0,b1.rows()) = b1;
    b.segment(b1.rows(),b1.rows()+b2.rows()) = b2;
    
    /// Stage 1 optimization energy minimization
    Ravelin::VectorNd z(nvars);
    
    static Ravelin::VectorNd _v;
    if(!Utility::solve_qp_pos(G,c,A,b,z,_v,false)){
      OUT_LOG(logERROR)  << "%ERROR: Unable to solve stage 1!";
      return false;
    }
    
    OUTLOG(z,"Z_OP1",logDEBUG1);
    // measure feasibility of solution
    // qM z - qq >= 0
    Ravelin::VectorNd feas;
    A.mult(z,feas) -= b;
    
    OUTLOG(feas,"feas_OP1 =[ % (A*z-b >= 0)",logDEBUG1);
    
    if(feas.rows() > 0){
      double min_elem = *std::min_element(feas.begin(), feas.end());
      if(min_elem < -NEAR_ZERO){
        OUT_LOG(logERROR)  << "ERROR: Optimization 1 produced an infeasible result!" << min_elem;
        return false;
      } else {
        cf_final = z;
      }
    }
    
    // DEBUGGING OUTPUT
    if (LOG(logDEBUG1)) {
      Ravelin::VectorNd f_vqstar(n+nq), vplus_tau;
      
      // [ Mv + Rz + hfext ; vq*]
      Ravelin::VectorNd f = Mv_fext;
      R.mult(cf_final,f,1,1);
      f_vqstar.segment(0,n) = f;
      f_vqstar.segment(n,n+nq) = vqstar;
      OUTLOG(vqstar,"vq*",logDEBUG1);
      
      // [v+ ; tau]
      iX.mult(f_vqstar,vplus_tau);
      Ravelin::VectorNd vplus = vplus_tau.segment(0,n);
      OUTLOG(vplus,"v+ (stage 1)",logERROR);
      OUTLOG(0.5*M.mult(vplus,workv1).dot(vplus),"KE (after stage 1)",logERROR);
      
      OUTLOG(c.dot(z) + 0.5*G.mult(z,workv1).dot(z),"KE dissipation (stage 1)",logERROR);
      
      x = vplus_tau.segment(n,n+nq);
      x /= -h;
      OUTLOG(x,"tau",logDEBUG1);
      OUTLOG(x.norm(),"||tau|| -- 1",logDEBUG1);
    }
    
    return true;
    
    /////////////////////////////////////////////////////////////////////////////
    ///////////////// Stage 2 optimization: command smoothing ///////////////////
    /////////////////////////////////////////////////////////////////////////////
    
    // Z = null(G)
    Ravelin::MatrixNd Z;
    LA_.nullspace(G,Z);
    unsigned size_null_space = Z.columns();
    if(size_null_space != 0)
    {
      OUT_LOG(logDEBUG1)  << "Nullspace exists! solving stage 2...";
      
      OUTLOG(Z,"null(G)",logDEBUG1);
      OUTLOG(R.mult(Z,workM1),"R*null(G)",logDEBUG1);
      
      OUTLOG(Z,"null(G)",logDEBUG1);
      
      /////////////////////////////// OBJECTIVE ///////////////////////////////////
      // w'Z'R'Y'M Y R Z w
      // Matrix: G = R' delta delta' R
      Ravelin::MatrixNd G, DTR,DTRZ;
      delta.transpose_mult(R,DTR);
      DTR.mult(Z,DTRZ);
      DTRZ.transpose_mult(DTRZ,G);
      
      Ravelin::MatrixNd Rz_Mv_fext;
      R.mult(z,Rz_Mv_fext = Mv_fext,1,1);
      
      // Vector: c = ((z' R' + v' M' + h fext)' delta + (vq*') epsilon) delta' R Z w
      Ravelin::VectorNd c;
      DTRZ.transpose_mult(delta.transpose_mult(Rz_Mv_fext,
                                               epsilon.mult(vqstar,workv1),1,1),c);
      if (LOG(logDEBUG1)) {
        OUTLOG(delta.mult(epsilon,workM1),"delta*epsilon",logDEBUG1);
        OUTLOG(DTRZ.transpose_mult_transpose(delta,workM1),"D D' R Z",logDEBUG1);
        OUTLOG(DTRZ.transpose_mult(epsilon,workM1),"E' D' R Z",logDEBUG1);
      }
      ////////////////////////////// CONSTRAINTS ///////////////////////////////////
      // Non-negativity Constraint
      // (Z w + z) >= 0:
      Ravelin::MatrixNd A1;
      Ravelin::VectorNd b1;
      // Z w
      A1 = Z;
      
      // z (contact impulses from Stage 1)
      b1 = z;
      b1.negate();
      OUTLOG(b1,"b_positive",logDEBUG1);
      OUTLOG(A1,"A_positive",logDEBUG1);
      
      // Non-Interpenetration:
      Ravelin::MatrixNd A2;
      Ravelin::VectorNd b2;
      
      // A = N gamma R Z
      N.transpose_mult(YR.mult(Z,workM1),A2);
      // b = -N gamma (Rz + Mv + h*fext)
      N.transpose_mult(gamma.mult(Rz_Mv_fext,workv1),b2,-1,0);
      N.transpose_mult(delta.mult(vqstar,workv1),b2,-1,1);
      
      OUTLOG(b2,"b_noninterpen",logDEBUG1);
      OUTLOG(A2,"A_noninterpen",logDEBUG1);
      
      // Coulomb Friction Polygon:
      nvars = Z.columns();
      Ravelin::MatrixNd A3;
      Ravelin::VectorNd b3;
      
      if(nk == 4){
        A3.set_zero(nc, nvars);
        b3.set_zero(nc);
        for (int ii=0;ii < nc;ii++){
          // normal direction
          //  A3(ii,:) = P(ii,:)
          //  b3(ii) = -z(ii)
          A3.row(ii) = ((workv1 = Z.row(ii))*=MU(ii,0));
          b3[ii] = -z[ii]*MU(ii,0);
          
          // tangent directions
          // kk indexes matrix, k refers to contact direction
          for(int kk=nc+ii;kk<nc+nk*nc;kk+=nc){
            A3.row(ii) -= Z.row(kk);
            b3[ii]     += z[kk];
          }
        }
      } else {
        A3.set_zero(nc*nk/2, nvars);
        b3.set_zero(nc*nk/2);
        for (int ii=0;ii < nc;ii++){
          // for each Friction Direction
          for(int k=0;k<nk/2;k++){
            // normal force
            A3.row(ii*nk/2+k) = ((workv1 = Z.row(ii))*=MU(ii,k));
            b3[ii*nk/2+k] = -z[ii]*MU(ii,k);
            // tangent forces [polygonal]
            for(int kk=nc+ii+nc*k;kk<nc+nk*nc;kk+=nc*nk/2){
              A3.row(ii*nk/2+k) -= Z.row(kk);
              b3[ii*nk/2+k]     += z[kk];
            }
          }
        }
      }
      OUTLOG(b3,"b_friction",logDEBUG1);
      OUTLOG(A3,"A_friction",logDEBUG1);
      
      // combine all linear inequality constraints
      //      assert(A1.columns() == A2.columns());
      //      assert(A2.columns() == A3.columns());
      Ravelin::MatrixNd A_OP2(A1.rows()+A2.rows()+A3.rows(),A1.columns());
      Ravelin::VectorNd b_OP2(b1.rows()+b2.rows()+b3.rows());
      A_OP2.block(0,A1.rows(),0,A1.columns()) = A1;
      A_OP2.block(A1.rows(),A1.rows()+A2.rows(),0,A2.columns()) = A2;
      A_OP2.block(A1.rows()+A2.rows(),A1.rows()+A2.rows()+A3.rows(),0,A3.columns()) = A3;
      b_OP2.segment(0,b1.rows()) = b1;
      b_OP2.segment(b1.rows(),b1.rows()+b2.rows()) = b2;
      b_OP2.segment(b1.rows()+b2.rows(),b1.rows()+b2.rows()+b3.rows()) = b3;
      
      Ravelin::VectorNd w(size_null_space);
      
      if(!Utility::solve_qp(G,c,A_OP2,b_OP2,w)){
        OUT_LOG(logERROR)  << "ERROR: Unable to solve stage 2!";
      } else {
        OUTLOG(w,"W_OP2",logDEBUG1);
        Ravelin::VectorNd feas;
        A_OP2.mult(w,feas) -= b_OP2;
        OUTLOG(feas,"feas_w_OP2 =[ % (A*z-b >= 0)",logDEBUG1);
        
        // Add to contact force vector
        Z.mult(w,z,1,1);
        // check feasibility
        OUTLOG(z,"Z_OP2",logDEBUG1);
        // measure feasibility of solution
        // qM z - qq >= 0
        A.mult(z,feas) -= b;
        OUTLOG(feas,"feas_z_OP2 =[ % (A*z-b >= 0)",logDEBUG1);
        if(feas.rows() > 0){
          double min_elem = *std::min_element(feas.begin(), feas.end());
          if(min_elem < -NEAR_ZERO){
            OUT_LOG(logERROR)  << "ERROR: Optimization 2 produced an infeasible result!" << min_elem;
          } else {
            cf_final = z;
          }
        }
      }
    }
  }
  
  // Retrieve inverse dynamics forces
  // M*v+ = M*v + P'*tau + R*z + h*fext
  //   x  = P*v + h*qdd
  
  Ravelin::VectorNd f_vqstar(n+nq), vplus_tau;
  
  // [ Mv + Rz + hfext ; vq*]
  Ravelin::VectorNd f = Mv_fext;
  if(nc > 0){
    R.mult(cf_final,f,1,1);
  }
  f_vqstar.segment(0,n) = f;
  f_vqstar.segment(n,n+nq) = vqstar;
  OUTLOG(vqstar,"vq*",logDEBUG1);
  
  // [v+ ; tau]
  iX.mult(f_vqstar,vplus_tau);
  Ravelin::VectorNd vplus = vplus_tau.segment(0,n);
  OUTLOG(vplus,"v+ (final)",logERROR);
  //  OUTLOG(G.mult(z,workv1 = c,1,1).dot(z),"KE dissipation (final)",logERROR);
  
  OUTLOG(M.mult(vplus,workv1).dot(vplus),"KE (final)",logERROR);
  
  x = vplus_tau.segment(n,n+nq);
  x /= -h;
  OUTLOG(x,"tau",logDEBUG1);
  OUTLOG(x.norm(),"||tau|| -- 2",logDEBUG1);
  
  
  
  // Some debugging dialogue
  OUT_LOG(logDEBUG) << "<< inverse_dynamics() exited" << std::endl;
  return true;
}

/** Maximal Dissipation Model (contact + inverse dynamics) -- Algebraic setup -- 1 stage
 *  min{z}  v+' M v+
 *  such that:
 *  v+ = v + inv(M)([ N, ST+, ST- ] z + h fext + P tau)
 *  N' v+ >= 0
 *  f_N >= 0
 *  mu * f_N{i} >= 1' [f_S{i}; f_T{i}] for i=1:nc
 *  P' v+ = v + h qdd
 **/
bool inverse_dynamics_one_stage(const Ravelin::VectorNd& v, const Ravelin::VectorNd& qdd, const Ravelin::MatrixNd& M,const  Ravelin::MatrixNd& N,
                               const Ravelin::MatrixNd& ST, const Ravelin::VectorNd& fext_, double h, const Ravelin::MatrixNd& MU, Ravelin::VectorNd& x, Ravelin::VectorNd& cf_final, std::vector<unsigned>& indices, int active_eefs,bool SAME_AS_LAST_CONTACTS){
  OUT_LOG(logDEBUG) << ">> inverse_dynamics() entered" << std::endl;
  
  Ravelin::VectorNd fext = fext_;
  // get number of degrees of freedom and number of contact points
  int n = M.rows();
  int nq = n - 6;
  int nc = N.columns();
  
  //  if (nc==0) {
  //    return false;
  //  }
  
  Ravelin::MatrixNd workM1,workM2;
  Ravelin::VectorNd workv1, workv2,fID;
  
  Ravelin::VectorNd vq(nq);
  v.get_sub_vec(0,nq,vq);
  
  Ravelin::VectorNd vb(6);
  v.get_sub_vec(nq,n,vb);
  
  Ravelin::VectorNd vqstar;
  ((vqstar = qdd) *= h) += vq;
  
  // Log these function variables
  
  // compute A, B, and C
  // | C B'| = M
  // | B A |
  Ravelin::MatrixNd C(nq,nq);
  M.get_sub_mat(0,nq,0,nq,C);
  
  Ravelin::MatrixNd B(6,nq);
  M.get_sub_mat(nq,n,0,nq,B);
  
  Ravelin::MatrixNd A(6,6);
  M.get_sub_mat(nq,n,nq,n,A);
  
  
  // compute D, E, and F
  Ravelin::MatrixNd iM_chol;
  iM_chol = M;
  bool pass = LA_.factor_chol(iM_chol);
  assert(pass);
  
  Ravelin::MatrixNd iM;
  iM = Ravelin::MatrixNd::identity(n);
  LA_.solve_chol_fast(iM_chol,iM);
  //  LA_.solve_fast(M,iM);
  
  // | F E'|  =  inv(M)
  // | E D |
  Ravelin::MatrixNd D(6,6);
  iM.get_sub_mat(nq,n,nq,n,D);
  Ravelin::MatrixNd E(6,nq);
  iM.get_sub_mat(nq,n,0,nq,E);
  Ravelin::MatrixNd F(nq,nq);
  iM.get_sub_mat(0,nq,0,nq,F);
  Ravelin::MatrixNd iF;
  iF = F;
  pass = LA_.factor_chol(iF);
  assert(pass);
  
  // if in mid-air only return ID forces solution
  // fID + fext = M qdd  ==> fID = M qdd - fext
  C.mult((workv1 = qdd),fID) -= fext.get_sub_vec(0,nq,workv2);
  
  int nk = (nc == 0)? 0 : ST.columns()/nc;
  int nvars = nc + nc*(nk);
  // setup R
  Ravelin::MatrixNd R(n, nc + (nc*nk) );
  R.block(0,n,0,nc) = N;
  R.block(0,n,nc,nc*nk+nc) = ST;
  
  // compute j and k
  // [E,D]
  Ravelin::MatrixNd ED(E.rows(),E.columns()+D.columns());
  ED.set_sub_mat(0,0,E);
  ED.set_sub_mat(0,E.columns(),D);
  // [F,E']
  Ravelin::MatrixNd FET(F.rows(),F.columns()+E.rows()),
  ET = E;
  ET.transpose();
  FET.set_sub_mat(0,0,F);
  FET.set_sub_mat(0,F.columns(),ET);
  
  //  OUTLOG( ED,"[E D]");
  //  OUTLOG(FET,"[F E']");
  
  // Predict Contact forces
  // Incorporate fID into acting forces on robot, then find contact forces
  fext.segment(0,nq) += fID;
  
  /// Stage 1 optimization energy minimization
  Ravelin::VectorNd z(nvars),cf(nvars);
  
  // j and k
  
  // j = [E,D]*fext*h + vb
  Ravelin::VectorNd j;
  // fext + [0;fID]
  ED.mult(fext,(j = vb),h,1);
  
  //  OUTLOG(j,"j = [ %= [E,D]*fext*h + vb");
  
  // k = [F,E']*fext*h  +  vq
  Ravelin::VectorNd k;
  FET.mult(fext,(k = vq),h,1);
  //  OUTLOG(k,"k = [ % = [F,E']*fext*h  +  vq");
  
  // compute Z and p
  // Z = ( [E,D] - E inv(F) [F,E'] ) R
  Ravelin::MatrixNd Z(ED.rows(), R.columns());
  workM1 = FET;
  //  OUTLOG(workM1,"workM1");
  LA_.solve_chol_fast(iF,workM1);
  E.mult(workM1,workM2);
  workM2 -= ED;
  workM2.mult(R,Z,-1,0);
  //  OUTLOG(Z,"Z = [ % = ( [E,D] - E inv(F) [F,E'] ) R");
  
  // p = j + E inv(F) (vq* - k)
  Ravelin::VectorNd p = j;
  workv1 = vqstar;
  workv1 -= k;
  LA_.solve_chol_fast(iF,workv1);
  E.mult(workv1,p,1,1);
  
  // H = Z'A Z
  Ravelin::MatrixNd H(Z.columns(),Z.columns());
  // compute objective function
  Z.transpose_mult(A,workM1);
  workM1.mult(Z,H);
  
  if(cf_final.rows() != 0){
    // return the inverse dynamics forces
    // x = iF*(vqstar - k - FET*R*(cf))/h
    (x = vqstar) -= k;
    FET.mult(R,workM1);
    workM1.mult(cf_final,x,-1,1);
    LA_.solve_chol_fast(iF,x);
    x /= h;
    x += fID;
    return true;
  }
  
  /////////////////////////////////////////////////////////////////////////////
  ///////////////// Stage 1 optimization:  IDYN Energy Min ////////////////////
  /////////////////////////////////////////////////////////////////////////////
  
  /////////////////////////////// OBJECTIVE ///////////////////////////////////
  // set Hessian:
  // qG = Z'A Z = [H]
  //  OUTLOG(H,"H = [ % = Z'A Z");
  Ravelin::MatrixNd qG = H;
  // set Gradient:
  // qc = Z'A p + Z'B vq*;
  Ravelin::VectorNd qc(Z.columns());
  // HINT: workM1 = Z'*A
  
  // qc = Z'A p
  workM1.mult(p,qc);
  
  Z.transpose_mult(B,workM2);
  // HINT: workM2 = Z'B
  
  // qc += Z'B vqstar
  workM2.mult(vqstar,qc,1,1);
  
  ////////////////////////////// CONSTRAINTS ///////////////////////////////////
  
  // setup linear inequality constraints -- noninterpenetration
  // N'[zeros(nq,:) ; Z] z + N'[vq* ; p] >= 0
  
  // [zeros(nq,:) ; Z]
  workM1.set_zero(n,Z.columns());
  workM1.set_sub_mat(nq,0,Z);
  // constraint Jacobain 1:
  // qM1 = N'[zeros(nq,:) ; Z]
  Ravelin::MatrixNd qM1(N.columns(),Z.columns());
  N.transpose_mult(workM1,qM1);
  OUTLOG(qM1,"M_IP",logDEBUG1);
  // [vq* ; p]
  Ravelin::VectorNd vqstar_p(n);
  vqstar_p.set_sub_vec(0,vqstar);
  vqstar_p.set_sub_vec(nq,p);
  
  // constraint vector 1
  // qq1 = -N'[vq* ; p]
  Ravelin::VectorNd qq1(N.columns());
  N.transpose_mult(vqstar_p,qq1);
  qq1.negate();
  OUTLOG(qq1,"q_IP",logDEBUG1);
  
  // setup linear inequality constraints -- coulomb friction
  // where : z = [cN  cS cT  -cS -cT]'
  // mu_i cN_i - cS_i - cT_i >= 0
  
  Ravelin::MatrixNd qM2;
  Ravelin::VectorNd qq2;
  // rhs ia zero
  
  
  // inscribe friction polygon in friction cone (scale by cos(pi/nk))
  if(nk == 4){
    qM2.set_zero(nc, nvars);
    qq2.set_zero(nc);
    for (int ii=0;ii < nc;ii++){
      // normal force
      qM2(ii,ii) = MU(ii,0);
      // tangent forces [polygonal]
      for(int kk=nc+ii;kk<nc+nk*nc;kk+=nc)
        qM2(ii,kk) = -1.0;
    }
  } else {
    qM2.set_zero(nc*nk/2, nvars);
    qq2.set_zero(nc*nk/2);
    double polygon_rad = cos(M_PI/nk);
    // for each Contact
    for (int ii=0;ii < nc;ii++){
      // for each Friction Direction
      for(int k=0;k<nk/2;k++){
        // normal force
        qM2(ii*nk/2+k,ii) = MU(ii,k)*polygon_rad;
        // tangent forces [polygonal]
        for(int kk=nc+ii+nc*k;kk<nc+nk*nc;kk+=nc*nk/2)
          qM2(ii*nk/2+k,kk) = -1.0;
      }
    }
  }
  //  OUTLOG(qM2,"CF");
  
  // combine all linear inequality constraints
  assert(qM1.columns() == qM2.columns());
  Ravelin::MatrixNd qM(qM1.rows()+qM2.rows(),qM1.columns());
  Ravelin::VectorNd qq(qq1.rows()+qq2.rows());
  qM.set_sub_mat(0,0,qM1);
  qM.set_sub_mat(qM1.rows(),0,qM2);
  qq.set_sub_vec(0,qq1);
  qq.set_sub_vec(qq1.rows(),qq2);
  
  static Ravelin::VectorNd _v;
  bool warm_start = true;
  if(_v.rows() != (qq.rows() + z.rows()) || !SAME_AS_LAST_CONTACTS)
    warm_start = false;
  
  if(!Utility::solve_qp_pos(qG,qc,qM,qq,z,_v,warm_start)){
    OUT_LOG(logERROR)  << "%ERROR: Unable to solve stage 1!";
    return false;
  }
  
  OUTLOG(z,"Z_OP1",logDEBUG1);
  // measure feasibility of solution
  // qM z - qq >= 0
  Ravelin::VectorNd feas;
  qM.mult(z,feas) -= qq;
  
  OUTLOG(feas,"feas_OP1 =[ % (A*z-b >= 0)",logDEBUG1);
  
  // push z into output vector
  cf_final = z;
  // return the inverse dynamics forces
  // x = iF*(vqstar - k - FET*R*(cf))/h
  (x = vqstar) -= k;
  FET.mult(R,workM1);
  workM1.mult(cf,x,-1,1);
  LA_.solve_chol_fast(iF,x);
  x /= h;
  x += fID;
  
  // Some debugging dialogue
  OUT_LOG(logDEBUG) << "<< inverse_dynamics() exited" << std::endl;
  return true;
}

/** Maximal Dissipation Model (contact + inverse dynamics) -- Algebraic setup -- 2 stage
 *  min{z}  v+' M v+
 *  such that:
 *  v+ = v + inv(M)([ N, ST+, ST- ] z + h fext + P tau)
 *  N' v+ >= 0
 *  f_N >= 0
 *  mu * f_N{i} >= 1' [f_S{i}; f_T{i}] for i=1:nc
 *  P' v+ = v + h qdd
 **/
bool inverse_dynamics_two_stage(const Ravelin::VectorNd& v, const Ravelin::VectorNd& qdd, const Ravelin::MatrixNd& M,const  Ravelin::MatrixNd& N,
                       const Ravelin::MatrixNd& ST, const Ravelin::VectorNd& fext_, double h, const Ravelin::MatrixNd& MU, Ravelin::VectorNd& x, Ravelin::VectorNd& cf_final, std::vector<unsigned>& indices, int active_eefs,bool SAME_AS_LAST_CONTACTS){
  OUT_LOG(logDEBUG) << ">> inverse_dynamics() entered" << std::endl;
  
  Ravelin::VectorNd fext = fext_;
  // get number of degrees of freedom and number of contact points
  int n = M.rows();
  int nq = n - 6;
  int nc = N.columns();
  
  //  if (nc==0) {
  //    return false;
  //  }
  
  Ravelin::MatrixNd workM1,workM2;
  Ravelin::VectorNd workv1, workv2,fID;
  
  Ravelin::VectorNd vq(nq);
  v.get_sub_vec(0,nq,vq);
  
  Ravelin::VectorNd vb(6);
  v.get_sub_vec(nq,n,vb);
  
  Ravelin::VectorNd vqstar;
  ((vqstar = qdd) *= h) += vq;
  
  // Log these function variables
  
  // compute A, B, and C
  // | C B'| = M
  // | B A |
  Ravelin::MatrixNd C(nq,nq);
  M.get_sub_mat(0,nq,0,nq,C);
  
  Ravelin::MatrixNd B(6,nq);
  M.get_sub_mat(nq,n,0,nq,B);
  
  Ravelin::MatrixNd A(6,6);
  M.get_sub_mat(nq,n,nq,n,A);
  
  
  // compute D, E, and F
  Ravelin::MatrixNd iM_chol;
  iM_chol = M;
  bool pass = LA_.factor_chol(iM_chol);
  assert(pass);
  
  Ravelin::MatrixNd iM;
  iM = Ravelin::MatrixNd::identity(n);
  LA_.solve_chol_fast(iM_chol,iM);
  //  LA_.solve_fast(M,iM);
  
  // | F E'|  =  inv(M)
  // | E D |
  Ravelin::MatrixNd D(6,6);
  iM.get_sub_mat(nq,n,nq,n,D);
  Ravelin::MatrixNd E(6,nq);
  iM.get_sub_mat(nq,n,0,nq,E);
  Ravelin::MatrixNd F(nq,nq);
  iM.get_sub_mat(0,nq,0,nq,F);
  Ravelin::MatrixNd iF;
  iF = F;
  pass = LA_.factor_chol(iF);
  assert(pass);
  
  // if in mid-air only return ID forces solution
  // fID + fext = M qdd  ==> fID = M qdd - fext
  C.mult((workv1 = qdd),fID) -= fext.get_sub_vec(0,nq,workv2);
  
  int nk = (nc == 0)? 0 : ST.columns()/nc;
  int nvars = nc + nc*(nk);
  
  // setup R
  Ravelin::MatrixNd R(n, nc + (nc*nk) );
  R.block(0,n,0,nc) = N;
  R.block(0,n,nc,nc*nk+nc) = ST;
  
  // compute j and k
  // [E,D]
  Ravelin::MatrixNd ED(E.rows(),E.columns()+D.columns());
  ED.set_sub_mat(0,0,E);
  ED.set_sub_mat(0,E.columns(),D);
  // [F,E']
  Ravelin::MatrixNd FET(F.rows(),F.columns()+E.rows()),
  ET = E;
  ET.transpose();
  FET.set_sub_mat(0,0,F);
  FET.set_sub_mat(0,F.columns(),ET);
  
  //  OUTLOG( ED,"[E D]");
  //  OUTLOG(FET,"[F E']");
  
  // Predict Contact forces
  // Incorporate fID into acting forces on robot, then find contact forces
  fext.segment(0,nq) += fID;
  
  /// Stage 1 optimization energy minimization
  Ravelin::VectorNd z(nvars),cf(nvars);
  
  // j and k
  
  // j = [E,D]*fext*h + vb
  Ravelin::VectorNd j;
  // fext + [0;fID]
  ED.mult(fext,(j = vb),h,1);
  
  //  OUTLOG(j,"j = [ %= [E,D]*fext*h + vb");
  
  // k = [F,E']*fext*h  +  vq
  Ravelin::VectorNd k;
  FET.mult(fext,(k = vq),h,1);
  //  OUTLOG(k,"k = [ % = [F,E']*fext*h  +  vq");
  
  // compute Z and p
  // Z = ( [E,D] - E inv(F) [F,E'] ) R
  Ravelin::MatrixNd Z(ED.rows(), R.columns());
  workM1 = FET;
  //  OUTLOG(workM1,"workM1");
  LA_.solve_chol_fast(iF,workM1);
  E.mult(workM1,workM2);
  workM2 -= ED;
  workM2.mult(R,Z,-1,0);
  //  OUTLOG(Z,"Z = [ % = ( [E,D] - E inv(F) [F,E'] ) R");
  
  // p = j + E inv(F) (vq* - k)
  Ravelin::VectorNd p = j;
  workv1 = vqstar;
  workv1 -= k;
  LA_.solve_chol_fast(iF,workv1);
  E.mult(workv1,p,1,1);
  
  // H = Z'A Z
  Ravelin::MatrixNd H(Z.columns(),Z.columns());
  // compute objective function
  Z.transpose_mult(A,workM1);
  workM1.mult(Z,H);
  
  if(cf_final.rows() != 0){
    // return the inverse dynamics forces
    // x = iF*(vqstar - k - FET*R*(cf))/h
    (x = vqstar) -= k;
    FET.mult(R,workM1);
    workM1.mult(cf_final,x,-1,1);
    LA_.solve_chol_fast(iF,x);
    x /= h;
    x += fID;
    return true;
  }
  
  /////////////////////////////////////////////////////////////////////////////
  ///////////////// Stage 1 optimization:  IDYN Energy Min ////////////////////
  /////////////////////////////////////////////////////////////////////////////
  
  /////////////////////////////// OBJECTIVE ///////////////////////////////////
  // set Hessian:
  // qG = Z'A Z = [H]
  //  OUTLOG(H,"H = [ % = Z'A Z");
  Ravelin::MatrixNd qG = H;
  // set Gradient:
  // qc = Z'A p + Z'B vq*;
  Ravelin::VectorNd qc(Z.columns());
  // HINT: workM1 = Z'*A
  
  // qc = Z'A p
  workM1.mult(p,qc);
  
  Z.transpose_mult(B,workM2);
  // HINT: workM2 = Z'B
  
  // qc += Z'B vqstar
  workM2.mult(vqstar,qc,1,1);
  
  ////////////////////////////// CONSTRAINTS ///////////////////////////////////
  
  // setup linear inequality constraints -- noninterpenetration
  // N'[zeros(nq,:) ; Z] z + N'[vq* ; p] >= 0
  
  // [zeros(nq,:) ; Z]
  workM1.set_zero(n,Z.columns());
  workM1.set_sub_mat(nq,0,Z);
  // constraint Jacobain 1:
  // qM1 = N'[zeros(nq,:) ; Z]
  Ravelin::MatrixNd qM1(N.columns(),Z.columns());
  N.transpose_mult(workM1,qM1);
  OUTLOG(qM1,"M_IP",logDEBUG1);
  // [vq* ; p]
  Ravelin::VectorNd vqstar_p(n);
  vqstar_p.set_sub_vec(0,vqstar);
  vqstar_p.set_sub_vec(nq,p);
  
  // constraint vector 1
  // qq1 = -N'[vq* ; p]
  Ravelin::VectorNd qq1(N.columns());
  N.transpose_mult(vqstar_p,qq1);
  qq1.negate();
  OUTLOG(qq1,"q_IP",logDEBUG1);
  
  // setup linear inequality constraints -- coulomb friction
  // where : z = [cN  cS cT  -cS -cT]'
  // mu_i cN_i - cS_i - cT_i >= 0
  
  Ravelin::MatrixNd qM2;
  Ravelin::VectorNd qq2;
  // rhs ia zero
  
  
  // inscribe friction polygon in friction cone (scale by cos(pi/nk))
  if(nk == 4){
    qM2.set_zero(nc, nvars);
    qq2.set_zero(nc);
    for (int ii=0;ii < nc;ii++){
      // normal force
      qM2(ii,ii) = MU(ii,0);
      // tangent forces [polygonal]
      for(int kk=nc+ii;kk<nc+nk*nc;kk+=nc)
        qM2(ii,kk) = -1.0;
    }
  } else {
    qM2.set_zero(nc*nk/2, nvars);
    qq2.set_zero(nc*nk/2);
    double polygon_rad = cos(M_PI/nk);
    // for each Contact
    for (int ii=0;ii < nc;ii++){
      // for each Friction Direction
      for(int k=0;k<nk/2;k++){
        // normal force
        qM2(ii*nk/2+k,ii) = MU(ii,k)*polygon_rad;
        // tangent forces [polygonal]
        for(int kk=nc+ii+nc*k;kk<nc+nk*nc;kk+=nc*nk/2)
          qM2(ii*nk/2+k,kk) = -1.0;
      }
    }
  }
  //  OUTLOG(qM2,"CF");
  
  // combine all linear inequality constraints
  assert(qM1.columns() == qM2.columns());
  Ravelin::MatrixNd qM(qM1.rows()+qM2.rows(),qM1.columns());
  Ravelin::VectorNd qq(qq1.rows()+qq2.rows());
  qM.set_sub_mat(0,0,qM1);
  qM.set_sub_mat(qM1.rows(),0,qM2);
  qq.set_sub_vec(0,qq1);
  qq.set_sub_vec(qq1.rows(),qq2);
  
  static Ravelin::VectorNd _v;
  bool warm_start = true;
  if(_v.rows() != (qq.rows() + z.rows()) || !SAME_AS_LAST_CONTACTS)
    warm_start = false;
  
  if(!Utility::solve_qp_pos(qG,qc,qM,qq,z,_v,warm_start)){
    OUT_LOG(logERROR)  << "%ERROR: Unable to solve stage 1!";
    return false;
  }
  
  OUTLOG(z,"Z_OP1",logDEBUG1);
  // measure feasibility of solution
  // qM z - qq >= 0
  Ravelin::VectorNd feas;
  qM.mult(z,feas) -= qq;
  
  OUTLOG(feas,"feas_OP1 =[ % (A*z-b >= 0)",logDEBUG1);
  
  // push z into output vector
  cf = z;
  
  /////////////////////////////////////////////////////////////////////////////
  ///////////////// Stage 2 optimization: command smoothing ///////////////////
  /////////////////////////////////////////////////////////////////////////////
  
  // H = Z'AZ
  Ravelin::MatrixNd P;
  LA_.nullspace(H,P);
  unsigned size_null_space = P.columns();
  if(size_null_space != 0)
  {
    // second optimization is necessary if the previous Hessian was PSD:
    // size_null_space > 0
    
    OUTLOG(P,"Null Space(P)",logDEBUG1);
    
    // compute U = [F,E']*R
    Ravelin::MatrixNd U;
    FET.mult(R,U);
    
    /////////////////////////////// OBJECTIVE //////////////////////////////////
    
    // Objective Hessian:
    // qG = P'*U'*iF'*iF*U*P;
    workM1 = U;
    LA_.solve_chol_fast(iF,workM1);
    workM1.mult(P,workM2);
    workM2.transpose_mult(workM2,qG);
    
    // HINT: workM2 = iF*U*P
    // HINT: workM1 = iF*U
    
    // Objective Gradient:
    // qc = z'*U'*iF'*iF*U*P - vqstar'*iF'*iF*U*P + k'*iF'*iF*U*P;
    
    // qc = (iF*U*P)'*iF*U*z
    workM2.transpose_mult(workM1.mult(z,workv1),workv2);
    qc = workv2;
    
    // qc -= (iF*U*P)'* iF*vqstar
    workv1 = vqstar;
    workM2.transpose_mult(LA_.solve_chol_fast(iF,workv1),workv2);
    qc -= workv2;
    
    // qc += (iF*U*P)'* iF*k
    workv1 = k;
    workM2.transpose_mult(LA_.solve_chol_fast(iF,workv1),workv2);
    qc += workv2;
    
    ////////////////////////////// CONSTRAINTS /////////////////////////////////
    
    // Linear Inequality Constraints:
    
    // Compressive force constraint (& polygonal tangential forces):
    // z + Pw >= 0 || P*w >= -z
    
    // Constraint Jacobian 1:
    // qM1 = null(H) = P
    qM1 = P;
    
    // Constraint Vector 1:
    // qq1 = z (contact impulses from Stage 1)
    qq1 = z;
    qq1.negate();
    
    // Non-Interpenetration:
    // SRZ: P = null( Z'*H*Z ) --> P = null(Z) this means:
    //       noninterpenetration & linear energy constraints always = 0
    
    // qM2 = N'*[zeros(nq,nvars_null);Z*P];
    // qq2 = N'*[Z*z + p];
    
    // Coulomb Friction Polygon:
    nvars = P.columns();
    Ravelin::MatrixNd qM3;
    Ravelin::VectorNd qq3;
    //    OUTLOG()
    if(nk == 4){
      qM3.set_zero(nc, nvars);
      qq3.set_zero(nc);
      for (int ii=0;ii < nc;ii++){
        // normal direction
        //  qM3(ii,:) = P(ii,:)
        //  qq3(ii) = -z(ii)
        qM3.row(ii) = ((workv1 = P.row(ii))*=MU(ii,0));
        qq3[ii] = -z[ii]*MU(ii,0);
        
        // tangent directions
        // kk indexes matrix, k refers to contact direction
        for(int kk=nc+ii;kk<nc+nk*nc;kk+=nc){
          qM3.row(ii) -= P.row(kk);
          qq3[ii]     += z[kk];
        }
      }
    } else {
      qM3.set_zero(nc*nk/2, nvars);
      qq3.set_zero(nc*nk/2);
      for (int ii=0;ii < nc;ii++){
        // for each Friction Direction
        for(int k=0;k<nk/2;k++){
          // normal force
          qM3.row(ii*nk/2+k) = ((workv1 = P.row(ii))*=MU(ii,k));
          qq3[ii*nk/2+k] = -z[ii]*MU(ii,k);
          // tangent forces [polygonal]
          for(int kk=nc+ii+nc*k;kk<nc+nk*nc;kk+=nc*nk/2){
            qM3.row(ii*nk/2+k) -= P.row(kk);
            qq3[ii*nk/2+k]     += z[kk];
          }
        }
      }
    }
    
    // Set up constarint matrix
    // SRZ: constraint 2 (interpenetration) is not here
    qM.set_zero(qM1.rows()+qM3.rows(),qM1.columns());
    qq.set_zero(qM1.rows()+qM3.rows());
    qM.set_sub_mat(0,0,qM1);
    qM.set_sub_mat(qM1.rows(),0,qM3);
    qq.set_sub_vec(0,qq1);
    qq.set_sub_vec(qq1.rows(),qq3);
    
    // optimize system
    Ravelin::VectorNd w(size_null_space);
    if(!Utility::solve_qp(qG,qc,qM,qq,w)){
      OUT_LOG(logERROR)  << "ERROR: Unable to solve stage 2!";
      return false;
      // then skip to calculating x from stage 1 solution
    } else {
      OUTLOG(w,"W_OP2",logDEBUG1);
      
      // measure feasibility of solution
      // qM w - qq >= 0
      feas = qq;
      qM.mult(w,feas,1,-1);
      
      OUTLOG(feas,"feas_OP2 =[ % (A*w - b >= 0)",logDEBUG1);
      
      // return the solution (contact forces)
      // cf = z + P*w;
      
      P.mult(w,cf,1,1);
      
      OUTLOG(cf,"z_OP2 =[ % (P*w + z)",logDEBUG1);
      
    }
  }
  
  //  OUTLOG(cf,"final_contact_force");
  //  Note compare contact force prediction to Moby contact force
  
  cf_final = cf;
  // return the inverse dynamics forces
  // x = iF*(vqstar - k - FET*R*(cf))/h
  (x = vqstar) -= k;
  FET.mult(R,workM1);
  workM1.mult(cf,x,-1,1);
  LA_.solve_chol_fast(iF,x);
  x /= h;
  x += fID;
  
  // Some debugging dialogue
  OUT_LOG(logDEBUG) << "<< inverse_dynamics() exited" << std::endl;
  return true;
}


/** Maximal Dissipation Model (contact + inverse dynamics) -- no-slip -- 2 stage
 *  min{z}  v+' M v+
 *  such that:
 *  v+ = v + inv(M)([ N, ST+, ST- ] z + h fext + P tau)
 *  N' v+ >= 0
 *  f_N >= 0
 *  P' v+ = v + h qdd
 **/
bool inverse_dynamics_no_slip(const Ravelin::VectorNd& v, const Ravelin::VectorNd& qdd, const Ravelin::MatrixNd& M,const  Ravelin::MatrixNd& N,
                              const Ravelin::MatrixNd& ST, const Ravelin::VectorNd& fext_, double h, Ravelin::VectorNd& x, Ravelin::VectorNd& cf_final, std::vector<unsigned>& indices, int active_eefs,bool SAME_AS_LAST_CONTACTS){
  OUT_LOG(logDEBUG) << ">> inverse_dynamics_no_slip() entered" << std::endl;
  
  Ravelin::VectorNd fext = fext_;
  // get number of degrees of freedom and number of contact points
  int n = M.rows();
  int nq = n - 6;
  int nc = N.columns();
  
  if (nc==0) {
    return false;
  }
  
  Ravelin::MatrixNd workM1,workM2;
  Ravelin::VectorNd workv1, workv2,fID;
  
  Ravelin::VectorNd vq(nq);
  v.get_sub_vec(0,nq,vq);
  
  Ravelin::VectorNd vb(6);
  v.get_sub_vec(nq,n,vb);
  
  Ravelin::VectorNd vqstar;
  ((vqstar = qdd) *= h) += vq;
  
  // Log these function variables
  
  // compute A, B, and C
  // | C B'| = M
  // | B A |
  Ravelin::MatrixNd C(nq,nq);
  M.get_sub_mat(0,nq,0,nq,C);
  
  Ravelin::MatrixNd B(6,nq);
  M.get_sub_mat(nq,n,0,nq,B);
  
  Ravelin::MatrixNd A(6,6);
  M.get_sub_mat(nq,n,nq,n,A);
  
  
  // compute D, E, and F
  Ravelin::MatrixNd iM_chol;
  iM_chol = M;
  bool pass = LA_.factor_chol(iM_chol);
  assert(pass);
  
  Ravelin::MatrixNd iM;
  iM = Ravelin::MatrixNd::identity(n);
  LA_.solve_chol_fast(iM_chol,iM);
  //  LA_.solve_fast(M,iM);
  
  // | F E'|  =  inv(M)
  // | E D |
  Ravelin::MatrixNd D(6,6);
  iM.get_sub_mat(nq,n,nq,n,D);
  Ravelin::MatrixNd E(6,nq);
  iM.get_sub_mat(nq,n,0,nq,E);
  Ravelin::MatrixNd F(nq,nq);
  iM.get_sub_mat(0,nq,0,nq,F);
  Ravelin::MatrixNd iF;
  iF = F;
  pass = LA_.factor_chol(iF);
  assert(pass);
  
#ifndef NDEBUG
  OUTLOG(N,"N",logDEBUG);
  //  OUTLOG(F,"",logDEBUG);
#endif
  
  // if in mid-air only return ID forces solution
  // fID + fext = M qdd  ==> fID = M qdd - fext
  C.mult((workv1 = qdd),fID) -= fext.get_sub_vec(0,nq,workv2);
  
  int nk = (nc == 0)? 0 : ST.columns()/nc;
  int nvars = nc + nc*(nk);
  // setup R
  Ravelin::MatrixNd R(n, nc + (nc*nk) );
  R.block(0,n,0,nc) = N;
  R.block(0,n,nc,nc*nk+nc) = ST;
  
  // compute j and k
  // [E,D]
  Ravelin::MatrixNd ED(E.rows(),E.columns()+D.columns());
  ED.set_sub_mat(0,0,E);
  ED.set_sub_mat(0,E.columns(),D);
  // [F,E']
  Ravelin::MatrixNd FET(F.rows(),F.columns()+E.rows()),
  ET = E;
  ET.transpose();
  FET.set_sub_mat(0,0,F);
  FET.set_sub_mat(0,F.columns(),ET);
  
  //  OUTLOG( ED,"[E D]");
  //  OUTLOG(FET,"[F E']");
  
  // Predict Contact forces
  // Incorporate fID into acting forces on robot, then find contact forces
  fext.segment(0,nq) += fID;
  
  /// Stage 1 optimization energy minimization
  Ravelin::VectorNd z(nvars),cf(nvars);
  
  // j and k
  
  // j = [E,D]*fext*h + vb
  Ravelin::VectorNd j;
  // fext + [0;fID]
  ED.mult(fext,(j = vb),h,1);
  
  //  OUTLOG(j,"j = [ %= [E,D]*fext*h + vb");
  
  // k = [F,E']*fext*h  +  vq
  Ravelin::VectorNd k;
  FET.mult(fext,(k = vq),h,1);
  //  OUTLOG(k,"k = [ % = [F,E']*fext*h  +  vq");
  
  // Use Sensed contact forces
  if(cf_final.rows() != 0){
    
    (x = vqstar) -= k;
    FET.mult(R,workM1);
    workM1.mult(cf_final,x,-1,1);
    LA_.solve_chol_fast(iF,x);
    x /= h;
    x += fID;
    
    return true;
  }
  
  // compute Z and p
  // Z = ( [E,D] - E inv(F) [F,E'] ) R
  Ravelin::MatrixNd Z(ED.rows(), R.columns());
  workM1 = FET;
  //  OUTLOG(workM1,"workM1");
  LA_.solve_chol_fast(iF,workM1);
  E.mult(workM1,workM2);
  workM2 -= ED;
  workM2.mult(R,Z,-1,0);
  //  OUTLOG(Z,"Z = [ % = ( [E,D] - E inv(F) [F,E'] ) R");
  
  // p = j + E inv(F) (vq* - k)
  Ravelin::VectorNd p = j;
  workv1 = vqstar;
  workv1 -= k;
  LA_.solve_chol_fast(iF,workv1);
  E.mult(workv1,p,1,1);
  
  // H = Z'A Z
  Ravelin::MatrixNd H(Z.columns(),Z.columns());
  // compute objective function
  Z.transpose_mult(A,workM1);
  workM1.mult(Z,H);
  
  /////////////////////////////////////////////////////////////////////////////
  ///////////////// Stage 1 optimization:  IDYN Energy Min ////////////////////
  /////////////////////////////////////////////////////////////////////////////
  
  /////////////////////////////// OBJECTIVE ///////////////////////////////////
  // set Hessian:
  // qG = Z'A Z = [H]
  //  OUTLOG(H,"H = [ % = Z'A Z");
  Ravelin::MatrixNd qG = H;
  // set Gradient:
  // qc = Z'A p + Z'B vq*;
  Ravelin::VectorNd qc(Z.columns());
  // HINT: workM1 = Z'*A
  
  // qc = Z'A p
  workM1.mult(p,qc);
  
  Z.transpose_mult(B,workM2);
  // HINT: workM2 = Z'B
  
  // qc += Z'B vqstar
  workM2.mult(vqstar,qc,1,1);
  
  ////////////////////////////// CONSTRAINTS ///////////////////////////////////
  
  // setup linear inequality constraints -- noninterpenetration
  // N'[zeros(nq,:) ; Z] z + N'[vq* ; p] >= 0
  
  // [zeros(nq,:) ; Z]
  workM1.set_zero(n,Z.columns());
  workM1.set_sub_mat(nq,0,Z);
  // constraint Jacobain 1:
  // qM1 = N'[zeros(nq,:) ; Z]
  Ravelin::MatrixNd qM1(N.columns(),Z.columns());
  N.transpose_mult(workM1,qM1);
  OUTLOG(qM1,"M_IP",logDEBUG);
  // [vq* ; p]
  Ravelin::VectorNd vqstar_p(n);
  vqstar_p.set_sub_vec(0,vqstar);
  vqstar_p.set_sub_vec(nq,p);
  
  // constraint vector 1
  // qq1 = -N'[vq* ; p]
  Ravelin::VectorNd qq1(N.columns());
  N.transpose_mult(vqstar_p,qq1);
  qq1.negate();
  OUTLOG(qq1,"q_IP",logDEBUG);
  
  Ravelin::MatrixNd qM(qM1.rows(),qM1.columns());
  Ravelin::VectorNd qq(qq1.rows());
  qM.set_sub_mat(0,0,qM1);
  qq.set_sub_vec(0,qq1);
  
  static Ravelin::VectorNd _v;
  bool warm_start = true;
  if(_v.rows() != (qq.rows() + z.rows()) || !SAME_AS_LAST_CONTACTS)
    warm_start = false;
  
  if(!Utility::solve_qp_pos(qG,qc,qM,qq,z,_v,warm_start)){
    OUT_LOG(logERROR)  << "%ERROR: Unable to solve stage 1!";
    return false;
  }
  
  OUTLOG(z,"Z_OP1",logDEBUG);
  // measure feasibility of solution
  // qM z - qq >= 0
  Ravelin::VectorNd feas;
  qM.mult(z,feas) -= qq;
  
  OUTLOG(feas,"feas_OP1 =[ % (A*z-b >= 0)",logDEBUG);
  
  // push z into output vector
  cf = z;
  
  /////////////////////////////////////////////////////////////////////////////
  ///////////////// Stage 2 optimization: command smoothing ///////////////////
  /////////////////////////////////////////////////////////////////////////////
  
  // H = Z'AZ
  Ravelin::MatrixNd P;
  LA_.nullspace(H,P);
  unsigned size_null_space = P.columns();
  if(size_null_space != 0)
  {
    // second optimization is necessary if the previous Hessian was PSD:
    // size_null_space > 0
    
    OUTLOG(P,"Null Space(P)",logDEBUG1);
    
    // compute U = [F,E']*R
    Ravelin::MatrixNd U;
    FET.mult(R,U);
    
    /////////////////////////////// OBJECTIVE //////////////////////////////////
    
    // Objective Hessian:
    // qG = P'*U'*iF'*iF*U*P;
    workM1 = U;
    LA_.solve_chol_fast(iF,workM1);
    workM1.mult(P,workM2);
    workM2.transpose_mult(workM2,qG);
    
    // HINT: workM2 = iF*U*P
    // HINT: workM1 = iF*U
    
    // Objective Gradient:
    // qc = z'*U'*iF'*iF*U*P - vqstar'*iF'*iF*U*P + k'*iF'*iF*U*P;
    
    // qc = (iF*U*P)'*iF*U*z
    workM2.transpose_mult(workM1.mult(z,workv1),workv2);
    qc = workv2;
    
    // qc -= (iF*U*P)'* iF*vqstar
    workv1 = vqstar;
    workM2.transpose_mult(LA_.solve_chol_fast(iF,workv1),workv2);
    qc -= workv2;
    
    // qc += (iF*U*P)'* iF*k
    workv1 = k;
    workM2.transpose_mult(LA_.solve_chol_fast(iF,workv1),workv2);
    qc += workv2;
    
    ////////////////////////////// CONSTRAINTS /////////////////////////////////
    
    // Linear Inequality Constraints:
    
    // Compressive force constraint (& polygonal tangential forces):
    // z + Pw >= 0 || P*w >= -z
    
    // Constraint Jacobian 1:
    // qM1 = null(H) = P
    qM1 = P;
    
    // Constraint Vector 1:
    // qq1 = z (contact impulses from Stage 1)
    qq1 = z;
    qq1.negate();
    
    // Non-Interpenetration:
    // SRZ: P = null( Z'*H*Z ) --> P = null(Z) this means:
    //       noninterpenetration & linear energy constraints always = 0
    
    // qM2 = N'*[zeros(nq,nvars_null);Z*P];
    // qq2 = N'*[Z*z + p];
    
    // Set up constarint matrix
    // SRZ: constraint 2 (interpenetration) is not here
    qM.set_zero(qM1.rows(),qM1.columns());
    qq.set_zero(qM1.rows());
    qM.set_sub_mat(0,0,qM1);
    qq.set_sub_vec(0,qq1);
    
    // optimize system
    Ravelin::VectorNd w(size_null_space);
    if(!Utility::solve_qp(qG,qc,qM,qq,w)){
      OUT_LOG(logERROR)  << "ERROR: Unable to solve stage 2!";
      // calculate x from stage 1 solution
      cf_final = cf;
      (x = vqstar) -= k;
      FET.mult(R,workM1);
      workM1.mult(cf_final,x,-1,1);
      LA_.solve_chol_fast(iF,x);
      x /= h;
      return true;
    } else {
      OUTLOG(w,"W_OP2",logDEBUG);
      
      // measure feasibility of solution
      // qM w - qq >= 0
      feas = qq;
      qM.mult(w,feas,1,-1);
      
      OUTLOG(feas,"feas_OP2 =[ % (A*w - b >= 0)",logDEBUG);
      
      // return the solution (contact forces)
      // cf = z + P*w;
      
      P.mult(w,cf,1,1);
      
      OUTLOG(cf,"z_OP2 =[ % (P*w + z)",logDEBUG);
      
    }
  } else {
    cf_final = cf;
    (x = vqstar) -= k;
    FET.mult(R,workM1);
    workM1.mult(cf_final,x,-1,1);
    LA_.solve_chol_fast(iF,x);
    x /= h;
  }
  
  
  //  OUTLOG(cf,"final_contact_force");
  //  Note compare contact force prediction to Moby contact force
  
  cf_final = cf;
  // return the inverse dynamics forces
  // x = iF*(vqstar - k - FET*R*(cf))/h
  (x = vqstar) -= k;
  FET.mult(R,workM1);
  workM1.mult(cf_final,x,-1,1);
  LA_.solve_chol_fast(iF,x);
  x /= h;
  x += fID;
  
  // Some debugging dialogue
  OUT_LOG(logDEBUG) << "<< inverse_dynamics_no_slip() exited" << std::endl;
  return true;
}
