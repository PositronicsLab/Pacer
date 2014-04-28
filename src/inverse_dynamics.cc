#include <robot.h>
#include <utilities.h>

int N_SYSTEMS = 0;

extern bool solve_qp_pos(const Ravelin::MatrixNd& Q, const Ravelin::VectorNd& c, const Ravelin::MatrixNd& A, const Ravelin::VectorNd& b, Ravelin::VectorNd& x);
extern bool solve_qp_pos(const Ravelin::MatrixNd& Q, const Ravelin::VectorNd& c, Ravelin::VectorNd& x);
extern bool solve_qp(const Ravelin::MatrixNd& Q, const Ravelin::VectorNd& c, const Ravelin::MatrixNd& A, const Ravelin::VectorNd& b, Ravelin::VectorNd& x);
extern bool solve_qp(const Ravelin::MatrixNd& Q, const Ravelin::VectorNd& c, const Ravelin::VectorNd& lb, const Ravelin::VectorNd& ub, const Ravelin::MatrixNd& A, const Ravelin::VectorNd& b, Ravelin::VectorNd& x);

Ravelin::VectorNd STAGE1, STAGE2;

//bool Robot::inverse_dynamics(const Ravelin::VectorNd& qdd, const Ravelin::MatrixNd& M, const Ravelin::VectorNd& fext, Ravelin::VectorNd& x){
//  M.mult(qdd,x) -= fext;
//}

bool Robot::inverse_dynamics(const Ravelin::VectorNd& v, const Ravelin::VectorNd& qdd, const Ravelin::MatrixNd& M,const  Ravelin::MatrixNd& N,
                         const Ravelin::MatrixNd& ST, const Ravelin::VectorNd& fext, double h, const Ravelin::MatrixNd& MU, Ravelin::VectorNd& x, Ravelin::VectorNd& cf_final){

#ifdef COLLECT_DATA   // record all input vars
  { // TODO: REMOVE THIS, FOR IROS PAPER
    STAGE1.set_zero(NUM_JOINTS);
    STAGE2.set_zero(NUM_JOINTS);
    STAGE1 *= NAN;
    STAGE2 *= NAN;

  }
#endif
  // get number of degrees of freedom and number of contact points
  int n = M.rows();
  int nq = n - 6;
  int nc = N.columns();

  static Ravelin::MatrixNd workM1,workM2;
  static Ravelin::VectorNd workv1, workv2;

  static Ravelin::VectorNd vq(nq);
  v.get_sub_vec(0,nq,vq);

  static Ravelin::VectorNd vb(6);
  v.get_sub_vec(nq,n,vb);

  static Ravelin::VectorNd vqstar;
  ((vqstar = qdd)*= h) += vq;

  // Log these function variables

  // compute A, B, and C
  // | C B'| = M
  // | B A |
  static Ravelin::MatrixNd C(nq,nq);
  M.get_sub_mat(0,nq,0,nq,C);

  static Ravelin::MatrixNd B(6,nq);
  M.get_sub_mat(nq,n,0,nq,B);

  static Ravelin::MatrixNd A(6,6);
  M.get_sub_mat(nq,n,nq,n,A);


  // compute D, E, and F
  static Ravelin::MatrixNd iM_chol;
  iM_chol = M;
  LA_.factor_chol(iM_chol);

  static Ravelin::MatrixNd iM;
  iM = Ravelin::MatrixNd::identity(n);
  LA_.solve_chol_fast(iM_chol,iM);
//  LA_.solve_fast(M,iM);

  // | F E'|  =  inv(M)
  // | E D |
  static Ravelin::MatrixNd D(6,6);
  iM.get_sub_mat(nq,n,nq,n,D);
  static Ravelin::MatrixNd E(6,nq);
  iM.get_sub_mat(nq,n,0,nq,E);
  static Ravelin::MatrixNd F(nq,nq);
  iM.get_sub_mat(0,nq,0,nq,F);
  static Ravelin::MatrixNd iF;
  iF = F;
  assert(LA_.factor_chol(iF));

  // if in mid-air only return ID forces solution
  if(nc == 0) return false;


  int nk = ST.columns()/nc;
  int nvars = nc + nc*(nk);
  // setup R
  Ravelin::MatrixNd R(n, nc + (nc*nk) );
  R.set_sub_mat(0,0,N);
  R.set_sub_mat(0,nc,ST);

  /// Stage 1 optimization energy minimization
  Ravelin::VectorNd z(nvars),cf(nvars);

#ifdef COLLECT_DATA   // record all input vars
    // generate a unique filename
    std::ostringstream fname;
    fname << "data/idyn_system" << (N_SYSTEMS) << ".m";

    // open the file
    std::ofstream out(fname.str().c_str());

    out  << "nc = " << nc << std::endl;
    out  << "nq = " << nq << std::endl;
    out  << "n = " << n << std::endl;
    out  << "h = " << h << std::endl;

    out  << "v = " <<v << std::endl;
    out  << "v = v';" << std::endl;
    out  << "a = " << qdd << std::endl;
    out  << "a = [a';zeros(6,1)]" << std::endl;

    out  << "vqstar = " <<vqstar << std::endl;
    out  << "vqstar = vqstar';" << std::endl;
    out  << "fext = " <<fext << std::endl;
    out  << "fext = fext';" << std::endl;

    out  <<  "N = [" <<N << "]"<< std::endl;
    out  << "ST = [" <<ST<< "]" << std::endl;
    out  << "S = ST(:,1:nc)" << std::endl;
    out  << "T = ST(:,nc+1:nc*2)" << std::endl;
    out  <<  "A = [" <<A << "]"<< std::endl;
    out  <<  "B = [" <<B << "]"<< std::endl;
    out  <<  "C = [" <<C << "]"<< std::endl;
    out  << "M = [C B';B A];" << std::endl;
    out  <<  "D = [" <<D << "]"<< std::endl;
    out  <<  "E = [" <<E << "]"<< std::endl;
    out  <<  "F = [" <<F << "]"<< std::endl;
    out  << "iM = [F E';E D];" << std::endl;

    out  << "MU = [" <<MU<< "]" << std::endl;
    out  <<  "R = [" <<R << "]"<< std::endl;

    out  << "CnT_invM_Cn = N'*iM*N" << std::endl;
    out  << "CnT_invM_Cs = N'*iM*S" << std::endl;
    out  << "CnT_invM_Ct = N'*iM*T" << std::endl;
    out  << "CsT_invM_Cs = S'*iM*S" << std::endl;
    out  << "CsT_invM_Ct = S'*iM*T" << std::endl;
    out  << "CtT_invM_Ct = T'*iM*T" << std::endl;
    out.close();
#endif
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
  // j and k

  // j = [E,D]*fext*h + vb
  Ravelin::VectorNd j;
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
//  OUTLOG(p,"p = [ % = j + E inv(F) (vq* - k)");

  // H = Z'A Z
  Ravelin::MatrixNd H(Z.columns(),Z.columns());
  // compute objective function
  Z.transpose_mult(A,workM1);
  workM1.mult(Z,H);

  if(cf_final.rows() != 0){
    (x = vqstar) -= k;
    FET.mult(R,workM1);
    workM1.mult(cf_final,x,-1,1);
    LA_.solve_chol_fast(iF,x);
    x /= h;
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
//  OUTLOG(qM1,"IP");
  // [vq* ; p]
  Ravelin::VectorNd vqstar_p(n);
  vqstar_p.set_sub_vec(0,vqstar);
  vqstar_p.set_sub_vec(nq,p);

  // constraint vector 1
  // qq1 = -N'[vq* ; p]
  Ravelin::VectorNd qq1(N.columns());
  N.transpose_mult(vqstar_p,qq1);
  qq1.negate();

  // setup linear inequality constraints -- coulomb friction
  // where : z = [{cN_i .. cN_nc}  {[cS -cS  cT -cT]_i .. [cS -cS  cT -cT]_nc}]'
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

  if(!solve_qp_pos(qG,qc,qM,qq,z)){
    OUT_LOG(logERROR)  << "%ERROR: Unable to solve stage 1!";
    return false;
  }

//  OUTLOG(z,"Z_OP1");
  // measure feasibility of solution
  // qM z - qq >= 0
  Ravelin::VectorNd feas;
  qM.mult(z,feas) -= qq;

//  OUTLOG(feas,"feas_OP1 =[ % (A*z-b >= 0)");

  // push z into output vector
  cf = z;
#ifdef COLLECT_DATA   // record all input vars
  { // TODO: REMOVE THIS, FOR IROS PAPER
    // return the inverse dynamics forces
    // x = iF*(vqstar - k - FET*R*(cf))/h
    x = vqstar;
    x -= k;
    FET.mult(R,workM1);
    workM1.mult(cf,x,-1,1);
    LA_.solve_chol_fast(iF,x);
    x /= h;
    STAGE1 = x;
  }
#endif

  /////////////////////////////////////////////////////////////////////////////
  ///////////////// Stage 2 optimization: command smoothing ///////////////////
  /////////////////////////////////////////////////////////////////////////////

  // H = Z'AZ
  Ravelin::MatrixNd P;
  unsigned size_null_space = Utility::kernal(H,P);
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
//    Ravelin::MatrixNd ZP(6+nq,size_null_space);
//    ZP.set_zero();
//    ZP.set_sub_mat(nq,0,Z.mult(P,workM1));

//    workv2.set_zero(n);
//    workv1 = p;
//    Z.mult(z,workv1,1,1);
//    workv2.set_sub_vec(nq,workv1);
//    workv2.set_sub_vec(0,vqstar);

//    N.transpose_mult(ZP,qM2);
//    N.transpose_mult(workv2,qq2);
//    qq2.negate(); // TODO: Fix This


    // Coulomb Friction Polygon:
    nvars = P.columns();
    Ravelin::MatrixNd qM3;
    Ravelin::VectorNd qq3;
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

//    qM.set_zero(qM1.rows()+qM2.rows()+qM3.rows(),qM1.columns());
//    qq.set_zero(qM1.rows()+qM2.rows()+qM3.rows());
//    qM.set_sub_mat(0,0,qM1);
//    qM.set_sub_mat(qM1.rows(),0,qM2);
//    qM.set_sub_mat(qM1.rows()+qM2.rows(),0,qM3);
//    qq.set_sub_vec(0,qq1);
//    qq.set_sub_vec(qM1.rows(),qq2);
//    qq.set_sub_vec(qq1.rows()+qM2.rows(),qq3);

    // optimize system
    Ravelin::VectorNd w(size_null_space);
    if(!solve_qp(qG,qc,qM,qq,w)){
      OUT_LOG(logERROR)  << "ERROR: Unable to solve stage 2!";
      return false;
      // then skip to calculating x from stage 1 solution
    } else {
//      OUTLOG(w,"W_OP2");

      // measure feasibility of solution
      // qM w - qq >= 0
      feas = qq;
      qM.mult(w,feas,1,-1);

//      OUTLOG(feas,"feas_OP2 =[ % (A*w - b >= 0)");

      // return the solution (contact forces)
      // cf = z + P*w;

      P.mult(w,cf,1,1);

//      OUTLOG(cf,"z_OP2 =[ % (P*w + z)");

#ifdef COLLECT_DATA   // record all input vars
      { // TODO: REMOVE THIS, FOR IROS PAPER
        // return the inverse dynamics forces
        // x = iF*(vqstar - k - FET*R*(cf))/h
        x = vqstar;
        x -= k;
        FET.mult(R,workM1);
        workM1.mult(cf,x,-1,1);
        LA_.solve_chol_fast(iF,x);
        x /= h;
        STAGE2 = x;
      }
#endif
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

  // Some debugging dialogue
#ifdef OUTPUT
  {
    // Zz + p == v + inv(M)(fext*h + R*z)
    Z.mult(z,workv1);
    workv1 += p;
    OUTLOG(workv1,"Z*z + p");

    (workv1 = fext)*=h;
    R.mult(z,workv1,1,1);
    iM.mult(workv1,workv2) += v;
    OUTLOG(workv2," == v + inv(M)(fext*h + R*z)");

    // N*(v + inv(M)fext*h == N*p
    workv1.set_zero(fext.rows());
    workv1.set_sub_vec(nq,p);
    N.transpose_mult(workv1,workv2);
    OUTLOG(workv2,"N'*p");

    (workv1 = fext)*=h;
    iM.mult(workv1,workv2) += v;
    N.transpose_mult(workv2,workv1);
    OUTLOG(workv1," == N'*(v + inv(M)*fext*h");

    // vq* == v + inv(M)(fext + [x; 0])*h
    OUTLOG(vqstar,"vq*");

    workv1.set_zero(fext.rows());
    workv1.set_sub_vec(0,x);
    (workv1 += fext);
    OUTLOG(workv1," (fext + [x; 0])");
    iM.mult(workv1,workv2);
    OUTLOG(workv2," inv(M)(fext + [x; 0])");
    (workv2*=h) += v;
    OUTLOG(workv2," v + inv(M)(fext + [x; 0])*h");

    workv1.set_zero(fext.rows());
    workv1.set_sub_vec(0,x);
    (workv1 += fext)*=h;
    R.mult(cf,workv1,1,1);
    iM.mult(workv1,workv2) += v;
    OUTLOG(workv1,"qdd == inv(M)(fext + R*z + [x; 0])*h)");

    Ravelin::MatrixNd S,T;
    ST.get_sub_mat(0,NDOFS,0,nc,S);
    ST.get_sub_mat(0,NDOFS,nc,nc*2,T);
    N.transpose_mult(iM,workM2);
    workM2.mult(N,workM1);
    OUTLOG(workM1,"N' * inv(M) * N");

    workM2.mult(S,workM1);
    OUTLOG(workM1,"N' * inv(M) * S");

    workM2.mult(T,workM1);
    OUTLOG(workM1,"N' * inv(M) * T");

    S.transpose_mult(iM,workM2);
    workM2.mult(S,workM1);
    OUTLOG(workM1,"S' * inv(M) * S");

    workM2.mult(T,workM1);
    OUTLOG(workM1,"S' * inv(M) * T");

    T.transpose_mult(iM,workM2);
    workM2.mult(T,workM1);
    OUTLOG(workM1,"T' * inv(M) * T");
  }
#endif
  return true;
}

bool Robot::inverse_dynamics(const Ravelin::VectorNd& v, const Ravelin::VectorNd& v_bar, const Ravelin::MatrixNd& M, const Ravelin::VectorNd& fext, double h, const Ravelin::MatrixNd& MU, Ravelin::VectorNd& x, Ravelin::VectorNd& z){

  // get number of degrees of freedom and number of contact points
  int n = M.rows();
  int nq = n - 6;

  // SRZ: Ground reaction forces are set to zero
  // (they destabilize robot system, why?)
//  z.set_zero();

  Ravelin::MatrixNd W;
  if(W.rows() == 0)
    (W = Ravelin::MatrixNd::identity(NUM_EEFS*3 + 6)) *= 1;

  // no base control
  for(int i=NUM_JOINTS;i<NUM_JOINTS+6;i++)
    W(i,i) = 100;

//  W(NUM_JOINTS,NUM_JOINTS) = 0;
//  W(NUM_JOINTS+1,NUM_JOINTS+1) = 0;
//  W(NUM_JOINTS+2,NUM_JOINTS+2) = 0;
//  W(NUM_JOINTS+5,NUM_JOINTS+5) = 0;

  OUTLOG(W,"W",logDEBUG1);
  static Ravelin::MatrixNd workM1,workM2;
  static Ravelin::VectorNd workv1, workv2;

  // Invert M -> iM
  static Ravelin::MatrixNd iM_chol;
  iM_chol = M;
  LA_.factor_chol(iM_chol);

  static Ravelin::MatrixNd iM;
  iM = Ravelin::MatrixNd::identity(n);
  LA_.solve_chol_fast(iM_chol,iM);

  // Workspace Jacobian
  Ravelin::MatrixNd Rb;
  calc_base_jacobian(Rb);

  Ravelin::MatrixNd F,U;
  OUTLOG(Rw,"Rw",logDEBUG1);

  // Actuated joint selection matrix
  F.set_zero(n,nq);
  F.set_sub_mat(0,0,Ravelin::MatrixNd::identity(nq));

  // v_err
  Ravelin::VectorNd v_err,V;
  (v_err = v) -= v_bar;
  OUTLOG(v_bar,"v_bar",logDEBUG1);
  OUTLOG(v,"v",logDEBUG1);
  OUTLOG(v_err,"v_err",logDEBUG1);
  OUTLOG(v_err.get_sub_vec(NUM_EEFS*3,NUM_EEFS*3+6,workv_),"BASE_ERR",logERROR);
  for (int i=0;i<NUM_EEFS;i++)
    OUTLOG(v_err.get_sub_vec(i*3,(i+1)*3,workv_),eefs_[i].id + "_ERR",logERROR);

  // W Rw iM
  Rw.mult(iM,workM1);
  W.mult(workM1,workM2);

  // V = (v_err)' W Rw iM
  workM2.transpose_mult(v_err,V);

  // U = (Rw iM)' W Rw iM
  workM1.transpose_mult(workM2,U);

  //
  Ravelin::MatrixNd qpQ,qpA;
  Ravelin::VectorNd qpc,qpb;
  if(z.rows() != 0){
    // do widyn for torque only
    // OBJECTIVE
    // Min workspace deviation from v_bar with weights W

    // Q = 1/2 h^2 [F' U F]
    F.transpose_mult(U,workM1);
    workM1.mult(F,qpQ);
    qpQ *= 0.5*h*h;

    // c = h[h fext' U + V + z'R U ] F
    // + h fext' U
    U.transpose_mult(fext,qpc,h,0);
    // + V
    qpc += V;
    // + z'R U
    qpc += U.transpose_mult(R.mult(z,workv1),workv2);
    // h*[^]*F
    F.transpose_mult((workv2 = qpc),qpc,h,0);
    // workv1 is R'z

    // CONSTRAINTS
    qpA.set_zero(nq*2,nq);
    qpb.set_zero(qpA.rows());

    // Interpenetration
    // A = N iM F
//    N.transpose_mult(iM.mult(F,workM1),workM2);
//    qpA.set_sub_mat(nq*2,0,workM2);
//    // b = -N(v + iM(h fext + R'z))
//    ((workv2 = fext) *= h) += workv1;
//    iM.mult(workv2,workv1) += v;
//    N.transpose_mult(workv1,workv2);
//    workv2.negate();
//    qpb.set_sub_vec(nq*2,workv2);

//    std::fill_n(torque_limits_u.begin(), torque_limits_u.size(), 1e+30);
//    std::fill_n(torque_limits_l.begin(), torque_limits_l.size(),-1e+30);

    // Torque Limits
    // Lower Torque Limit
    // A
    qpA.set_sub_mat(0,0,Ravelin::MatrixNd::identity(nq));
    // b
    qpb.set_sub_vec(0,(workv2 = torque_limits_l));

    // Upper Torque Limit
    // A
    qpA.set_sub_mat(nq,0,Ravelin::MatrixNd::identity(nq).negate());
    // b
    qpb.set_sub_vec(nq,(workv2 = torque_limits_u).negate());
    OUTLOG(D,"D",logERROR);
    x.resize(qpQ.rows());

    // SRZ: This is a QP with Symmetric PD hessian and box constraints
    // we can solve it faster than this!
    if(!solve_qp(qpQ,qpc,qpA,qpb,x)){
//    if(!solve_qp(qpQ,qpc,torque_limits_l,torque_limits_u,qpA,qpb,x)){
      OUT_LOG(logERROR)  << "ERROR: Unable to solve widyn!";
      assert(false);
    }

    OUTLOG(x,"x",logERROR);

    qpA.mult(x,workv1);
    workv1 -= qpb;
    OUTLOG(workv1,"feas >= 0",logERROR);
    qpQ.transpose_mult(x,workv1) += qpc;
    OUTLOG(workv1,"x'Q + c",logERROR);
    OUT_LOG(logERROR) << "(x'Q + c) x = "<< workv1.dot(x);

    return true;
  } else {
    // Do larger "Coupled" optimizations
    // for torque and cfs

    qpA.set_zero(NC+NC+nq*2,NC*3+nq);
    qpb.set_zero(qpA.rows());

    // CONSTARINTS
    // Interpenetration
    // K = N*iM*[R' F]
    workM2.set_zero(NDOFS,NC*5+nq);
    workM2.set_sub_mat(0,0,R);
    workM2.set_sub_mat(NC*5,0,F);
    iM.mult(workM2,workM1);
    N.transpose_mult(workM1,workM2);

    qpA.set_sub_mat(0,0,workM2);

    // h*iM*fext
    iM.mult((workv2 = fext)*=h,workv1);
    // + v
    workv1 += v;
    // -N*[]
    N.transpose_mult(workv1,workv2);
    workv2.negate();
    qpb.set_sub_vec(0,workv2);

    // Coulomb Friction
    // D goes 1_t1 2_t1 ... 1_t2 2_t2.. etc
    workM2.set_zero(NC,NC*5+nq);
    for (int ii=0;ii < NC;ii++){
      // normal force
      workM2(ii,ii) = MU(ii,0);
      // tangent forces [polygonal]
      for(int kk=NC+ii;kk<NC+4*NC;kk+=NC)
        workM2(ii,kk) = -1.0;
    }

    qpA.set_sub_mat(NC,0,workM2);
//    qpb.set_sub_vec(NC,/*zero*/);

    // Torque Limits
    // Lower Torque Limit
    // A
    qpA.set_sub_mat(NC*2,NC*5,Ravelin::MatrixNd::identity(nq));
    // b
    qpb.set_sub_vec(NC*2,(workv2 = torque_limits_l));

    // Upper Torque Limit
    // A
    qpA.set_sub_mat(NC*2+nq,0,Ravelin::MatrixNd::identity(nq).negate());
    // b
    qpb.set_sub_vec(NC*2+nq,(workv2 = torque_limits_u).negate());

  }

  return true;
}
