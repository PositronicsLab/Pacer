#include <robot.h>
#include <utilities.h>
#include <Opt/LP.h>

int N_SYSTEMS = 0;

extern bool solve_qp_pos(const Ravelin::MatrixNd& Q, const Ravelin::VectorNd& c, const Ravelin::MatrixNd& A, const Ravelin::VectorNd& b, Ravelin::VectorNd& x);
extern bool solve_qp_pos(const Ravelin::MatrixNd& Q, const Ravelin::VectorNd& c, Ravelin::VectorNd& x);
extern bool solve_qp(const Ravelin::MatrixNd& Q, const Ravelin::VectorNd& c, const Ravelin::MatrixNd& A, const Ravelin::VectorNd& b, Ravelin::VectorNd& x);
extern bool solve_qp(const Ravelin::MatrixNd& Q, const Ravelin::VectorNd& c, const Ravelin::VectorNd& lb, const Ravelin::VectorNd& ub, const Ravelin::MatrixNd& A, const Ravelin::VectorNd& b, Ravelin::VectorNd& x);
extern bool solve_qp(const Ravelin::MatrixNd& Q, const Ravelin::VectorNd& c, const Ravelin::VectorNd& lb, const Ravelin::VectorNd& ub, const Ravelin::MatrixNd& A, const Ravelin::VectorNd& b, const Ravelin::MatrixNd& M, const Ravelin::VectorNd& q, Ravelin::VectorNd& x);

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
  if(nc == 0){
    // Inverse dynamics for a floating base w/ no contact
//    goal accel
    Ravelin::VectorNd qdd_ext;
    F.mult(fext.get_sub_vec(0,nq,workv1),workv2,-1,0);
    C.mult(workv2 += qdd,x);
    return false;
  }

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

  if(!solve_qp_pos(qG,qc,qM,qq,z)){
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
//#define OUTPUT
#ifdef OUTPUT
  {
    // Zz + p == v + inv(M)(fext*h + R*z)
//    Z.mult(z,workv1);
//    workv1 += p;
//    OUTLOG(workv1,"Z*z + p",logDEBUG);

    // Idyn Result
    workv1.set_zero(fext.rows());
    workv1.set_sub_vec(0,x);
    (workv1 += fext) *= h;
    R.mult(z,workv1,1,1);
    iM.mult(workv1,workv2);
//    OUTLOG(workv2.get_sub_vec(0,nq,workv1),"qdd2 == inv(M)( (fext + [x;0])*h + R*z)",logDEBUG);
    OUTLOG(workv2.get_sub_vec(0,nq,workv1),"qdd_idyn",logDEBUG);
    OUTLOG(qdd,"qdd_des_idyn",logDEBUG);
    workv2 += v;
    OUTLOG(workv2,"v* == v + inv(M)( (fext + [x;0])*h + R*z)",logDEBUG);

    // N*(v + inv(M)fext*h == N*p
//    workv1.set_zero(fext.rows());
//    workv1.set_sub_vec(nq,p);
//    N.transpose_mult(workv1,workv2);
//    OUTLOG(workv2,"N'*p",logDEBUG);

//    (workv1 = fext)*=h;
//    iM.mult(workv1,workv2) += v;
//    N.transpose_mult(workv2,workv1);
//    OUTLOG(workv1," == N'*(v + inv(M)*fext*h",logDEBUG);

    // vq* == v + inv(M)(fext + [x; 0])*h
    OUTLOG(vqstar,"vq*",logDEBUG);

//    workv1.set_zero(fext.rows());
//    workv1.set_sub_vec(0,x);
//    (workv1 += fext);
//    OUTLOG(workv1," (fext + [x; 0])",logDEBUG);
//    iM.mult(workv1,workv2);
//    OUTLOG(workv2," inv(M)(fext + [x; 0])",logDEBUG);
//    (workv2*=h) += v;
//    OUTLOG(workv2," v + inv(M)(fext + [x; 0])*h",logDEBUG);

//    workv1.set_zero(fext.rows());
//    workv1.set_sub_vec(0,x);
//    (workv1 += fext)*=h;
//    R.mult(cf,workv1,1,1);
//    iM.mult(workv1,workv2) += v;
//    OUTLOG(workv1,"qdd == inv(M)(fext + R*z + [x; 0])*h)",logDEBUG);

//    Ravelin::MatrixNd S,T;
//    ST.get_sub_mat(0,NDOFS,0,nc,S);
//    ST.get_sub_mat(0,NDOFS,nc,nc*2,T);
//    N.transpose_mult(iM,workM2);
//    workM2.mult(N,workM1);
//    OUTLOG(workM1,"N' * inv(M) * N",logDEBUG);

//    workM2.mult(S,workM1);
//    OUTLOG(workM1,"N' * inv(M) * S",logDEBUG);

//    workM2.mult(T,workM1);
//    OUTLOG(workM1,"N' * inv(M) * T",logDEBUG);

//    S.transpose_mult(iM,workM2);
//    workM2.mult(S,workM1);
//    OUTLOG(workM1,"S' * inv(M) * S",logDEBUG);

//    workM2.mult(T,workM1);
//    OUTLOG(workM1,"S' * inv(M) * T",logDEBUG);

//    T.transpose_mult(iM,workM2);
//    workM2.mult(T,workM1);
//    OUTLOG(workM1,"T' * inv(M) * T",logDEBUG);
  }
#endif
  return true;
}
//#define LCP_METHOD
bool Robot::workspace_inverse_dynamics(const Ravelin::VectorNd& v, const Ravelin::VectorNd& v_bar_, const Ravelin::MatrixNd& M, const Ravelin::VectorNd& fext_, double h, const Ravelin::MatrixNd& MU, Ravelin::VectorNd& x, Ravelin::VectorNd& z){
  static Ravelin::VectorNd tau;
  Ravelin::VectorNd fext = fext_;
  Ravelin::VectorNd v_bar = v_bar_;
//  fext.negate();
  // get number of degrees of freedom and number of contact points
  int n = M.rows();
  int nq = n - 6;

  // SRZ: Ground reaction forces are set to zero
  // (they destabilize robot system, why?)

//  OUTLOG(R,"R",logERROR);
  static Ravelin::MatrixNd workM1,workM2;
  static Ravelin::VectorNd workv1, workv2;

  // Invert M -> iM
  static Ravelin::MatrixNd iM_chol;
  iM_chol = M;
  LA_.factor_chol(iM_chol);

  static Ravelin::MatrixNd iM;
  iM = Ravelin::MatrixNd::identity(n);
  LA_.solve_chol_fast(iM_chol,iM);

  Ravelin::MatrixNd F;

  // Actuated joint selection matrix
  F.set_zero(n,nq);
  F.set_sub_mat(0,0,Ravelin::MatrixNd::identity(nq));
  F *= h;

//#define OUTPUT
#ifdef OUTPUT
  OUTLOG(iM,"iM",logERROR);
  OUTLOG(M,"M",logERROR);
  OUTLOG(Rw,"J",logERROR);
  OUTLOG(v,"v",logERROR);
  OUTLOG(v_bar,"vw",logERROR);
  OUTLOG(fext,"f",logERROR);
  OUTLOG(h,"h",logERROR);
  OUTLOG(MU,"MU",logERROR);
  OUTLOG(NC,"nc",logERROR);
  OUTLOG(NUM_EEFS,"ne",logERROR);
  OUTLOG(NUM_JOINTS,"nq",logERROR);
  OUTLOG(n,"n",logERROR);
  OUTLOG(Rw.rows(),"nw",logERROR);
  OUTLOG(F,"F",logERROR);
  OUTLOG(R,"R",logERROR);
  OUTLOG(N,"N",logERROR);
  assert(NC == 0);
#endif

  // v_err
  Ravelin::VectorNd dvw = v_bar;
  // dvw = v_bar - vel_w
  Rw.mult(v,dvw,-1,1);

  OUTLOG(v_bar,"v_bar",logERROR);

  Ravelin::MatrixNd qpQ,qpA(NC*2,NC*5+nq);
  Ravelin::VectorNd qpc,qpb(NC*2);
  {
    const int NVARS = NC*5+nq;
    int WS_DOFS = Rw.rows();

    // Do larger "Coupled" optimizations
    // for torque and cfs

    //////////////////////// COEFS /////////////////////////
    // iM [R' F]: [z,\tau] --> [generalized_xdd]
    Ravelin::MatrixNd RTF;
    Ravelin::MatrixNd iMRTF;
    RTF.set_zero(NDOFS,NVARS);
    RTF.set_sub_mat(0,0,R);
    RTF.set_sub_mat(0,NC*5,F);
    iM.mult(RTF,iMRTF);

    // coulomb friction pyramid matrix
    Ravelin::MatrixNd CF;
    CF.set_zero(NC,NVARS);
    for (int ii=0;ii < NC;ii++){
      // normal force
      CF(ii,ii) = MU(ii,0);
      // tangent forces [polygonal]
      for(int kk=NC+ii;kk<NC+4*NC;kk+=NC)
        CF(ii,kk) = -1.0;
    }

    // QP Nullspace
    Ravelin::MatrixNd P;
    Ravelin::MatrixNd JiMRTF;
    Rw.mult(iMRTF,JiMRTF);

    //////////////////////// LP PHASE /////////////////////////
    /* OBJECTIVE
     * Min workspace deviation from v_bar
     * min_{x = [z,T,s]'}
     *   [0,0,1] x
     * s.t.
     *   -[A,",-I] x   >= -v_bar              // operational space goal l1-norm
     *   [A,",-I] x    >=  v_bar              // "
     *   N*iM*[R' F,0] x >= -N(h*iM*fext + v)   // Interpenetration
     *   [CF,0,0] x      >=  0                  // coulomb friction
     *   z >= 0                                 // compressive force
     *   T+ >= T >= T-                          // torque limits
     */

    Ravelin::MatrixNd lpA(WS_DOFS*2 + NC*2,nq+NC*5+WS_DOFS),
                      Aeq(0,lpA.columns());
    Ravelin::VectorNd lpb(lpA.rows()),
                      lpc,
                      x1,
                      beq(0);

    lpA.set_zero();
    lpb.set_zero();

    lpc.set_zero(nq+NC*5+WS_DOFS);
    lpc.set_sub_vec(nq+NC*5,Ravelin::VectorNd::one(WS_DOFS));
    lpc.set_sub_vec(nq+NC*5+WS_DOFS-6,(workv_ = Ravelin::VectorNd::one(6)));

//    C = J*iM*[R F]
//    d = dvw-J*iM*f
//    A = [ -[eye(nc).*1 -eye(nc) -eye(nc) -eye(nc) -eye(nc)] zeros(nc,nq)
//          -N'*iM*[R F]
//        ]
//    b = [  zeros(nc,1)
//           N'*(v+iM*f)
//        ]

//      lpM*x <= lpq
//    lpM = [ C  -eye(nw)
//           -C  -eye(nw)  % -dv +s >=  v -vw +fext
//            A  zeros(size(A,1),nw)
//          ]
//    lpq = [ d % +dv +s >= -v +vw -fext
//           -d % -dv +s >=  v -vw +fext
//            b
//          ]

    // Special LP constraint
    // -- operational space goal l1-norm --
    //    C = J*iM*[R F]
    //    d = dvw-J*iM*f
    Ravelin::MatrixNd C = JiMRTF;
    assert(WS_DOFS == C.rows());

    lpA.set_sub_mat(0,0,C);
    lpA.set_sub_mat(C.rows(),0,(workM_ = C).negate());
    workM1 = Ravelin::MatrixNd::identity(C.rows());
    workM1.negate();
    lpA.set_sub_mat(       0,C.columns(),workM1);
    lpA.set_sub_mat(C.rows(),C.columns(),workM1);

    Ravelin::VectorNd d;

    iM.mult(fext,workv1,-h,0);
    Rw.mult(workv1,d);
    d += dvw;

    lpb.set_sub_vec(0,d);
    lpb.set_sub_vec(d.rows(),(workv_ = d).negate());

    // Linear Constraints
    // A*x <= b
    //    A = [ -[eye(nc).*mu -eye(nc) -eye(nc) -eye(nc) -eye(nc)] zeros(nc,nq)
    //          -N'*iM*[R F]
    //        ]
    //    b = [  zeros(nc,1)
    //           N'*(v+iM*f)
    //        ]
    qpA.set_zero();
    qpb.set_zero();
    //  -- Coulomb Friction --
    CF.negate();
    qpA.set_sub_mat(0,0,CF);
    qpb.set_sub_vec(0,Ravelin::VectorNd::zero(NC));

    //  -- Interpenetration --
    // -N'*iM*[R F]
    N.transpose_mult(iMRTF,workM2,-1,0);
    qpA.set_sub_mat(NC,0,workM2);

    // N'*(v+iM*f)
    iM.mult(fext,workv1,h,0);
    workv1 += v;
    N.transpose_mult(workv1,workv2);
    qpb.set_sub_vec(NC,workv2);

    lpA.set_sub_mat(WS_DOFS*2,0,qpA);
    lpb.set_sub_vec(WS_DOFS*2,qpb);

    Ravelin::VectorNd ll(NVARS+WS_DOFS),ul(NVARS+WS_DOFS);
    // Lower Torque Limit
    std::fill(ll.begin(), ll.end(), -1e30);
    ll.set_sub_vec(0,Ravelin::VectorNd::zero(NC*5));
    ll.set_sub_vec(NC*5,torque_limits_l);

    // Upper Torque Limit
    std::fill(ul.begin(), ul.end(), 1e30);
    ul.set_sub_vec(NC*5,torque_limits_u);

    x1.set_zero(lpc.rows());
    OUTLOG(lpA,"lpA",logDEBUG1);
    OUTLOG(lpb,"lpb",logDEBUG1);
    OUTLOG(lpc,"lpc",logDEBUG1);
    OUTLOG(ll ,"lpl" ,logDEBUG1);
    OUTLOG(ul ,"lpu" ,logDEBUG1);
    OUTLOG(x1 ,"lpx0"  ,logDEBUG1);

    // A*x >= b
    lpA.negate();
    lpb.negate();

    Opt::LPParams params(lpc,Aeq,beq,lpA,lpb,ll,ul);
//    params.n = lpc.rows();
    unsigned LP_RESULT_FLAG = 0;

//      Solves the linear program:  min c'x
//                     subject to:  Ax = b
//                                  Mx >= q
    if(!Opt::LP::lp_simplex(params,x1,LP_RESULT_FLAG)){
      OUT_LOG(logERROR)  << "ERROR: Unable to solve widyn LP! : " << LP_RESULT_FLAG ;
      x = tau;
      return false;
    }

    OUTLOG(x1.get_sub_vec(NVARS,NVARS+WS_DOFS,workv1),"LP: [s]",logERROR);
    OUT_LOG(logERROR) << "LP: Objective: c' x = " << x1.dot(lpc);
//    assert(NC != 4);

    lpA.mult(x1,workv_);
    workv_ -= lpb;
    OUTLOG(workv_,"feas_geq_0",logDEBUG);

    (workv1 = x1).get_sub_vec(0,NVARS,x1);
    OUTLOG(x1,"LP: [z,T]",logDEBUG);
    x1.get_sub_vec(NC*3,NC*3+nq,x);

    OUTLOG(dvw,"v_os__old",logINFO);

    // new dvw ( will be variable d = J*inv(M)*[R' F] x )
    iM.mult(fext,workv1,h,0);
    Rw.mult(workv1,d);

    // New OS goal (found by LP)
    // C*x + fext
    OUTLOG(JiMRTF.mult(x1,(workv1 = d),1,1),"dv_os__new",logINFO);

    (workM1 = C).get_sub_mat(0,NUM_EEFS*3,0,C.columns(),C);
    (workv1 = d).get_sub_vec(0,NUM_EEFS*3,d);

    LA_.nullspace(JiMRTF,P);
    unsigned size_null_space = P.columns();
////    if(size_null_space == 0) return true;

    //////////////////////// QP PHASE /////////////////////////
    // OBJECTIVE
    // Min energy gain wrt T & z
    /* OBJECTIVE
     * ARGMIN(x = [z,T]')
     *   v+' M v+ ==>
     *   x'[R' F]'iM [R' F] x + (v' + fext'iM) [R' F] x
     * SUCH THAT
     *   [A,",-I] x + vbar == s               // operational space goal l1-norm result from LP
     *   N*iM*[R' F] x >= -N(h*iM*fext + v)   // Interpenetration
     *   [CF,0] x      >=  0                  // coulomb friction
     * Variable Box Constraints:
     *   z >= [0]                             // compressive force
     *   z <= [inf]
     *   T+ >= T >= T-               // NOT USING : torque limits
     */

    //  -- Operational Space Goal --
    // J iM [R' F] x == (v*_w - v_w - h J iM fext)
    Aeq = C;
    beq = d;

    // qpQ = [R' F]'iM [R' F]
    RTF.transpose_mult(iMRTF,qpQ);

    // qpc = (v' + fext'iM)[R' F]
    workv1 = v;
    iM.transpose_mult(fext,workv1,h,1);
    RTF.transpose_mult(workv1,qpc);

    // -- Torque Limits --
    ll.set_zero(NVARS);
    ul.set_zero(NVARS);

    // Lower var Limits
    std::fill(ll.begin(), ll.end(),-1e29);
    ll.set_sub_vec(0,Ravelin::VectorNd::zero(NC*5));
//    ll.set_sub_vec(NC*5,torque_limits_l);

    // Upper var Limits
    std::fill(ul.begin(), ul.end(), 1e29);
//    ul.set_sub_vec(NC*5,torque_limits_u);
    OUTLOG(ll,"qpl",logDEBUG1);
    OUTLOG(ul,"qpu",logDEBUG1);
    OUTLOG(qpA,"qpA",logDEBUG1);
    OUTLOG(qpb,"qpb",logDEBUG1);
    OUTLOG(qpQ,"qpQ",logDEBUG1);
    OUTLOG(qpc,"qpc",logDEBUG1);
    OUTLOG(Aeq,"Aeq",logDEBUG1);
    OUTLOG(beq,"beq",logDEBUG1);

    // A*x <= b
    qpA.set_zero();
    qpb.set_zero();

    x.set_zero(NVARS);
    bool RESULT_FLAG = false;
#define LCP_METHOD
#ifdef LCP_METHOD
    Ravelin::MatrixNd lcpA(Aeq.rows()*2 + qpA.rows() + NC*5,NVARS);
    lcpA.set_zero();

    lcpA.set_sub_mat(0,0,Aeq);

    lcpA.set_sub_mat(Aeq.rows(),0,(workM_ = Aeq).negate());

    lcpA.set_sub_mat(Aeq.rows()*2,0,qpA);

    lcpA.set_sub_mat(Aeq.rows()*2 + qpA.rows(),0,Ravelin::MatrixNd::identity(NC*5));


    Ravelin::VectorNd lcpb(beq.rows()*2 + qpA.rows() + NC*5);
    lcpb.set_zero();

    workv_.set_zero(beq.rows());
    std::fill(workv_.begin(), workv_.end(),-Moby::NEAR_ZERO);

    lcpb.set_sub_vec(0,workv_ += beq);

    lcpb.set_sub_vec(beq.rows(),(workv_ += beq).negate());

    lcpb.set_sub_vec(beq.rows()*2,qpb);

//    lcpb.set_sub_vec(beq.rows()*2 + qpA.rows(),zeros);

    if(!(RESULT_FLAG = solve_qp(qpQ,qpc,lcpA,lcpb,x))){
#else
    if(!(RESULT_FLAG = solve_qp(qpQ,qpc,ll,ul,Aeq,beq,qpA,qpb,x))){
#endif
      x1.get_sub_vec(NC*3,NC*3+nq,x);
      OUT_LOG(logERROR)  << "ERROR: Unable to solve widyn!";
      return false;
    }

    // ------------------------------------------------------------------------
    // ---------------- CHECK FEASIBILITY OF RESULT ---------------------------
    if(RESULT_FLAG){
      OUTLOG(x,"x = [z,tau]",logDEBUG);

      qpA.mult(x,workv_);
      workv_ -= qpb;
      OUTLOG(workv_,"feas_geq_0",logDEBUG);


      Aeq.mult(x,workv_);
      workv_ -= beq;
      OUTLOG(workv_,"feas_eq_0",logDEBUG);

      x.get_sub_vec(0,NC*5,workv2);
      OUTLOG(workv2,"cfs",logDEBUG);
      (workv1 = x).get_sub_vec(NC*5,NVARS,x);
    } else {
      x = tau;
      OUTLOG(x,"tau",logDEBUG);
    }
  }
  tau = x;
//  assert(NC == 0);

  return true;
}
