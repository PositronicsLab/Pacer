#include <quadruped.h>
#include <utilities.h>

//#define COLLECT_DATA
  static int N_SYSTEMS = 1;

extern bool solve_qp_pos(const Ravelin::MatrixNd& Q, const Ravelin::VectorNd& c, const Ravelin::MatrixNd& A, const Ravelin::VectorNd& b, Ravelin::VectorNd& x);
extern bool solve_qp_pos(const Ravelin::MatrixNd& Q, const Ravelin::VectorNd& c, Ravelin::VectorNd& x);
extern bool solve_qp(const Ravelin::MatrixNd& Q, const Ravelin::VectorNd& c, const Ravelin::MatrixNd& A, const Ravelin::VectorNd& b, Ravelin::VectorNd& x);

Ravelin::VectorNd STAGE1, STAGE2;

void Quadruped::inverse_dynamics(const Ravelin::VectorNd& v, const Ravelin::VectorNd& qdd, const Ravelin::MatrixNd& M,const  Ravelin::MatrixNd& N,
                         const Ravelin::MatrixNd& ST, const Ravelin::VectorNd& fext, double h, const Ravelin::MatrixNd& MU, Ravelin::VectorNd& uff){
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
  vqstar = qdd;
  vqstar *= h;
  vqstar += vq;

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
  // | F E'|  =  inv(M)
  // | E D |
  iM = Ravelin::MatrixNd::identity(n);
  LA_.solve_chol_fast(iM_chol,iM);
  static Ravelin::MatrixNd D(6,6);
  iM.get_sub_mat(nq,n,nq,n,D);
  static Ravelin::MatrixNd E(6,nq);
  iM.get_sub_mat(nq,n,0,nq,E);
  static Ravelin::MatrixNd F(nq,nq);
  iM.get_sub_mat(0,nq,0,nq,F);
  static Ravelin::MatrixNd iF;
  iF = F;
  LA_.factor_chol(iF);


  // determine vbstar, vqstar

  static Ravelin::VectorNd fID;
  workv1.set_zero(nq);
  fID.set_zero(nq);
  // compute fID
  // fID = (vq* - vq)/h
  workv1 = vqstar;
  workv1 -= vq;
  workv1 /= h;
  fext.get_sub_vec(0,nq,fID);
  F.mult(workv1,fID,1,-1);

  // in case of optimization error, this fn will return ID forces
//  uff = fID;
//  OUTLOG(fID,"fID");

  // if in mid-air only return ID forces solution
  if(nc == 0) return;


  int nk = ST.columns()/nc;
  int nvars = nc + nc*(nk);
  // setup R
  Ravelin::MatrixNd R(n, nc + (nc*nk) );
  R.set_sub_mat(0,0,N);
  R.set_sub_mat(0,nc,ST);

  /// Stage 1 optimization energy minimization
  Ravelin::VectorNd z(nvars),cf(nvars);

  static Ravelin::VectorNd zuff(n);
  zuff.set_zero();
  zuff.set_sub_vec(0,fID);

#ifdef COLLECT_DATA   // record all input vars
    // generate a unique filename
    std::ostringstream fname;
    fname << "idyn_system" << (N_SYSTEMS++) << ".m";

    // open the file
    std::ofstream out(fname.str().c_str());

    out  << "nc = " << nc << std::endl;
    out  << "nq = " << nq << std::endl;
    out  << "n = " << n << std::endl;
    out  << "h = " << h << std::endl;

//    out  << "M = [" <<M << "]"<< std::endl;
    out  << "v = " <<v << std::endl;
    out  << "v = v';" << std::endl;

//    out  << "vq = " <<vq << std::endl;
//    out  << "vb = " <<vb << std::endl;
    out  << "vqstar = " <<vqstar << std::endl;
    out  << "vqstar = vqstar';" << std::endl;
    out  << "fext = " <<fext << std::endl;
    out  << "fext = fext';" << std::endl;
    out  << "zuff = " <<zuff << std::endl;
    out  << "zuff = zuff';" << std::endl;
    out  <<  "A = [" <<A << "]"<< std::endl;
    out  <<  "B = [" <<B << "]"<< std::endl;
    out  <<  "C = [" <<C << "]"<< std::endl;
    out  << "M = [C B';B A];" << std::endl;
    out  <<  "D = [" <<D << "]"<< std::endl;
    out  <<  "E = [" <<E << "]"<< std::endl;
    out  <<  "F = [" <<F << "]"<< std::endl;
    out  << "iM = [F E';E D];" << std::endl;

    out  <<  "N = [" <<N << "]"<< std::endl;
    out  << "ST = [" <<ST<< "]" << std::endl;
    out  << "MU = [" <<MU<< "]" << std::endl;
    out  <<  "R = [" <<R << "]"<< std::endl;

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

  // j and k
  workv1 = fext;
  workv1 += zuff;
  workv1 *= h;
  // HINT: workv1 = (fext + zuff)*h

  // j = [E,D](fext + zuff)h + vb
  Ravelin::VectorNd j = vb;
  ED.mult(workv1,j,1,1);
//  OUTLOG(j,"j = [ %= [E,D](fext + zuff)h + vb");

  // k = [F,E'](fext + zuff)h  +  vq
  Ravelin::VectorNd k = vq;
  FET.mult(workv1,k,1,1);
//  OUTLOG(k,"k = [ % = [F,E'](fext + zuff)h  +  vq");

  // compute Z and p
  // Z = ( [E,D] - E inv(F) [F,E'] ) R
  Ravelin::MatrixNd Z(ED.rows(), R.columns());
  workM1 = FET;
  LA_.solve_chol_fast(iF,workM1);
  E.mult(workM1,workM2);
  workM2.negate();
  workM2 += ED;
  workM2.mult(R,Z);
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
  Ravelin::MatrixNd Z1(n,Z.columns());
  Z1.set_zero();
  Z1.set_sub_mat(nq,0,Z);

  // constraint Jacobain 1:
  // qM1 = N'[zeros(nq,:) ; Z]
  Ravelin::MatrixNd qM1(N.columns(),Z1.columns());
  N.transpose_mult(Z1,qM1);

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

  Ravelin::MatrixNd qM2 = Ravelin::MatrixNd::zero(nc, nvars);
  Ravelin::VectorNd qq2(nc);
  // rhs ia zero
  qq2.set_zero();

  // inscribe friction polygon in friction cone (scale by cos(pi/nk))
  double polygon_rad = cos(M_PI/nk);
  for (int ii=0;ii < nc;ii++){
    // normal force
    qM2(ii,ii) = 1;

    // tangent forces [polygonal]
    for(int k=0,kk=nc+nk*ii;kk<nc+nk*ii+nk;kk++,k++){
      double friction = MU(ii,k%(nk/2))*polygon_rad;
      qM2(ii,kk) = -1.0/friction;
    }
  }

  // combine all linear inequality constraints
  assert(qM1.columns() == qM2.columns());
  Ravelin::MatrixNd qM(qM1.rows()+qM2.rows(),qM1.columns());
  Ravelin::VectorNd qq(qq1.rows()+qq2.rows());
  qM.set_sub_mat(0,0,qM1);
  qM.set_sub_mat(qM1.rows(),0,qM2);
  qq.set_sub_vec(0,qq1);
  qq.set_sub_vec(qq1.rows(),qq2);

  if(!solve_qp_pos(qG*=0.5,qc,qM,qq,z)){
    OUT_LOG(logERROR)  << "%ERROR: Unable to solve stage 1!" << std::endl;
    return;
  }

  // measure feasibility of solution
  // qM z - qq >= 0
  Ravelin::VectorNd feas = qq;
  qM.mult(z,feas,1,-1);

//  OUTLOG(feas,"feas_OP1 =[ % (A*z-b >= 0)");
//  OUTLOG(z,"Z_OP1");

  // push z into output vector
  cf = z;
#ifdef COLLECT_DATA   // record all input vars
  { // TODO: REMOVE THIS, FOR IROS PAPER
    // return the inverse dynamics forces
    // uff = fID + iF*(vqstar - k - FET*R*(cf))/h
    uff = vqstar;
    uff -= k;
    FET.mult(R,workM1);
    workM1.mult(cf,uff,-1,1);
    LA_.solve_chol_fast(iF,uff);
    uff /= h;
    uff += fID;
    STAGE1 = uff;
  }
#endif

  /////////////////////////////////////////////////////////////////////////////
  ///////////////// Stage 2 optimization: command smoothing ///////////////////
  /////////////////////////////////////////////////////////////////////////////

  // H = Z'AZ
  Ravelin::MatrixNd P;
  unsigned size_null_space = kernal(H,P);
  if(size_null_space != 0)
  {
    // second optimization is necessary if the previous Hessian was PSD:
    // size_null_space > 0

//    OUTLOG(S,"Singular Values");
//    OUTLOG(P,"Null Space(P)");

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
    Ravelin::MatrixNd ZP(6+nq,size_null_space);
    ZP.set_zero();
    ZP.set_sub_mat(nq,0,Z.mult(P,workM1));

    workv2.set_zero(n);
    workv1 = p;
    Z.mult(z,workv1,1,1);
    workv2.set_sub_vec(nq,workv1);
    workv2.set_sub_vec(0,vqstar);

    N.transpose_mult(ZP,qM2);
    N.transpose_mult(workv2,qq2);
    qq2.negate();

    // Coulomb Friction Polygon:
    nvars = P.columns();
    Ravelin::MatrixNd qM3(nc, nvars);
    qM3.set_zero();
    Ravelin::VectorNd qq3(nc);
    qq3.set_zero();

    for (int ii=0;ii < nc;ii++){
      // normal direction
      //  qM3(ii,:) = P(ii,:)
      //  qq3(ii) = -z(ii)
      qM3.row(ii) = P.row(ii);
      qq3[ii] = -z[ii];

      // tangent directions
      // kk indexes matrix, k refers to contact direction
      for (int k=0,kk=nc+nk*ii;kk<nc+nk*ii+nk;kk++,k++){
        double friction = MU(ii,k%(nk/2))*polygon_rad;
        //  qM3(ii,:) -= P(kk,:) / friction
        //  qq3(ii)   +=   z(kk) / friction
        qM3.row(ii) -= ((workv1 = P.row(kk)) /= friction);
        qq3[ii]     += z[kk]/friction;
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
      OUT_LOG(logERROR)  << "ERROR: Unable to solve stage 2!" << std::endl;
//      assert(false);
      return;
      // then skip to calculating uff from stage 1 solution
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
#ifdef COLLECT_DATA   // record all input vars
      { // TODO: REMOVE THIS, FOR IROS PAPER
        // return the inverse dynamics forces
        // uff = fID + iF*(vqstar - k - FET*R*(cf))/h
        uff = vqstar;
        uff -= k;
        FET.mult(R,workM1);
        workM1.mult(cf,uff,-1,1);
        LA_.solve_chol_fast(iF,uff);
        uff /= h;
        uff += fID;
        STAGE2 = uff;
      }
#endif
    }
  }

  OUTLOG(cf,"final_contact_force");
  //  Note compare contact force prediction to Moby contact force
#ifndef NDEBUG
//  OUT_LOG(logINFO)  << "%cf = [cN cS cT] -> [z x y]"<< std::endl;
//  for(int i=0;i<nc;i++)
//    OUT_LOG(logINFO)  << "%["<< cf[i] << " "
//              << cf[i*nk+nc]-cf[i*nk+nc+nk/2] << " "
//              << cf[i*nk+nc+1]-cf[i*nk+nc+nk/2+1] << "] "<< std::endl;
//  OUT_LOG(logINFO)  << std::endl;
#endif
  // return the inverse dynamics forces
  // uff = fID + iF*(vqstar - k - FET*R*(cf))/h
  uff = vqstar;
  uff -= k;
  FET.mult(R,workM1);
  workM1.mult(cf,uff,-1,1);
  LA_.solve_chol_fast(iF,uff);
  uff /= h;
  uff += fID;
//  OUTLOG(uff,"final_joint_torque");

}
