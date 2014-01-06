#include <project_common.h>

static Ravelin::LinAlgd LA_;

typedef Ravelin::MatrixNd Mat;
typedef Ravelin::VectorNd Vec;

extern bool solve_qp_pos(const Mat& Q, const Vec& c, const Mat& A, const Vec& b, Vec& x);
extern bool solve_qp_pos(const Mat& Q, const Vec& c, Vec& x);
extern bool solve_qp(const Mat& Q, const Vec& c, const Mat& A, const Vec& b, Vec& x);

void inverse_dynamics(const Vec& v, const Vec& qdd, const Mat& M,const  Mat& N,
                         const Mat& ST, const Vec& fext, double h, const Mat& MU, Vec& uff){


  // get number of degrees of freedom and number of contact points
  int n = M.rows();
  int nq = n - 6;
  int nc = N.columns();

  static Mat workM1,workM2;
  static Vec workv1, workv2;

  static Vec vq(nq);
  v.get_sub_vec(0,nq,vq);

  static Vec vb(6);
  v.get_sub_vec(nq,n,vb);


  static Vec vqstar;
  vqstar = qdd;
  vqstar *= h;
  vqstar += vq;

  // Log these function variables
  OUTLOG(M,"M");
  OUTLOG(v,"v");
  OUTLOG(vqstar,"vqstar");
  OUTLOG(fext,"fext");

  // compute A, B, and C
  // | C B'| = M
  // | B A |
  static Mat C(nq,nq);
  M.get_sub_mat(0,nq,0,nq,C);

  static Mat B(6,nq);
  M.get_sub_mat(nq,n,0,nq,B);

  static Mat A(6,6);
  M.get_sub_mat(nq,n,nq,n,A);


  // compute D, E, and F
  static Mat iM_chol;
  iM_chol = M;
  LA_.factor_chol(iM_chol);

  static Mat iM;
  // | F E'|  =  inv(M)
  // | E D |
  iM = Mat::identity(n);
  LA_.solve_chol_fast(iM_chol,iM);
  static Mat D(6,6);
  iM.get_sub_mat(nq,n,nq,n,D);
  static Mat E(6,nq);
  iM.get_sub_mat(nq,n,0,nq,E);
  static Mat F(nq,nq);
  iM.get_sub_mat(0,nq,0,nq,F);
  static Mat iF;
  iF = F;
  LA_.factor_chol(iF);

  // determine vbstar, vqstar

  static Vec fID;
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
  uff = fID;

  // if in mid-air only return ID forces solution
  if(nc == 0) return;

  OUTLOG(N,"N");
  OUTLOG(ST,"ST");
  OUTLOG(MU,"MU");
  int nk = ST.columns()/nc;
  int nvars = nc + nc*(nk);
  // setup R
  Mat R(n, nc + (nc*nk) );
  R.set_sub_mat(0,0,N);
  R.set_sub_mat(0,nc,ST);
  OUTLOG(R,"R");

  /// Stage 1 optimization energy minimization
  Vec z(nvars),cf(nvars);

  static Vec zuff(n);
  zuff.set_zero();
  zuff.set_sub_vec(0,fID);

  // compute j and k
  // [E,D]
  Mat ED(E.rows(),E.columns()+D.columns());
  ED.set_sub_mat(0,0,E);
  ED.set_sub_mat(0,E.columns(),D);
  // [F,E']
  Mat FET(F.rows(),F.columns()+E.rows()),
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
  Vec j = vb;
  ED.mult(workv1,j,1,1);
  // k = [F,E'](fext + zuff)h  +  vq
  Vec k = vq;
  FET.mult(workv1,k,1,1);

  // compute Z and p
  // Z = ( [E,D] - E inv(F) [F,E'] ) R
  Mat Z(ED.rows(), R.columns());
  workM1 = FET;
  LA_.solve_chol_fast(iF,workM1);
  E.mult(workM1,workM2);
  workM2.negate();
  workM2 += ED;
  workM2.mult(R,Z);

  // p = j + E inv(F) (vq* - k)
  Vec p = j;
  workv1 = vqstar;
  workv1 -= k;
  LA_.solve_chol_fast(iF,workv1);
  E.mult(workv1,p,1,1);

  // H = Z'A Z
  Mat H(Z.columns(),Z.columns());
  // compute objective function
  Z.transpose_mult(A,workM1);
  workM1.mult(Z,H);

  /////////////////////////////////////////////////////////////////////////////
  ///////////////// Stage 1 optimization:  IDYN Energy Min ////////////////////
  /////////////////////////////////////////////////////////////////////////////

  /////////////////////////////// OBJECTIVE ///////////////////////////////////
  // set Hessian:
  // qG = Z'A Z = [H]
  Mat qG = H;
  // set Gradient:
  // qc = Z'A p + Z'B vq*;
  Vec qc(Z.columns());
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
  Mat Z1(n,Z.columns());
  Z1.set_zero();
  Z1.set_sub_mat(nq,0,Z);

  // constraint Jacobain 1:
  // qM1 = N'[zeros(nq,:) ; Z]
  Mat qM1(N.columns(),Z1.columns());
  N.transpose_mult(Z1,qM1);

  // [vq* ; p]
  Vec pvqstar(n);
  pvqstar.set_sub_vec(0,vqstar);
  pvqstar.set_sub_vec(nq,p);

  // constraint vector 1
  // qq1 = -N'[vq* ; p]
  Vec qq1(N.columns());
  N.transpose_mult(pvqstar,qq1);
  qq1.negate();

  // setup linear inequality constraints -- coulomb friction
  // where : z = [{cN_i .. cN_nc}  {[cS -cS  cT -cT]_i .. [cS -cS  cT -cT]_nc}]'
  // mu_i cN_i - cS_i - cT_i >= 0

  Mat qM2 = Mat::zero(nc, nvars);
  Vec qq2(nc);
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
  Mat qM(qM1.rows()+qM2.rows(),qM1.columns());
  Vec qq(qq1.rows()+qq2.rows());
  qM.set_sub_mat(0,0,qM1);
  qM.set_sub_mat(qM1.rows(),0,qM2);
  qq.set_sub_vec(0,qq1);
  qq.set_sub_vec(qq1.rows(),qq2);

  if(!solve_qp_pos(qG,qc,qM,qq,z)){
    OUT_LOG(logERROR)  << "ERROR: Unable to solve stage 1!" << std::endl;
//    assert(false);
    return;
  }

  // measure feasibility of solution
  // qM z - qq >= 0
  Vec feas = qq;
  qM.mult(z,feas,1,-1);

  OUTLOG(feas,"feas_OP1 =[ % (A*z-b >= 0)");
  OUTLOG(z,"Z_OP1");

  // push z into output vector
  cf = z;

  /////////////////////////////////////////////////////////////////////////////
  ///////////////// Stage 2 optimization: command smoothing ///////////////////
  /////////////////////////////////////////////////////////////////////////////

  // H = Z'AZ
  unsigned size_null_space = 0;
  Mat U,V,P;
  Vec S;

  // SVD decomp to retrieve nullspace of Z'AZ
  LA_.svd(H,U,S,V);

  if(S.rows() != 0){
    // Calculate the tolerance for ruling that a singular value is supposed to be zero
    double ZERO_TOL = std::numeric_limits<double>::epsilon() * H.rows() * S[0];
    // Count Zero singular values
    for(int i = S.rows()-1;i>=0;i--,size_null_space++)
      if(S[i] > ZERO_TOL)
        break;

    // get the nullspace
    P.set_zero(nvars,size_null_space);
    V.get_sub_mat(0,V.rows(),V.columns()-size_null_space,V.columns(),P);
  }

  if(size_null_space != 0)
  {
    // second optimization is necessary if the previous Hessian was PSD:
    // size_null_space > 0

//    OUTLOG(S,"Singular Values");
//    OUTLOG(P,"Null Space(P)");

    // compute U = [F,E']*R
    Mat U;
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
    // SRZ: P = null( Z'*[PD]*Z ) --> P = null(Z) this means:
    //       noninterpenetration & linear energy constraints always = 0
    Mat ZP(6+nq,size_null_space);
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
    Mat qM3(nc, nvars);
    qM3.set_zero();
    Vec qq3(nc);
    qq3.set_zero();

    for (int ii=0;ii < nc;ii++){
      // normal direction
      //  qM3(ii,:) = P(ii,:)
      //  qq3(ii,1) = -z(ii)
      qM3.row(ii) = P.row(ii);
      qq3[ii] = -z[ii];

      // tangent directions
      // kk indexes matrix, k refers to contact direction
      for (int k=0,kk=nc+nk*ii;kk<nc+nk*ii+nk;kk++,k++){
        double friction = MU(ii,k%(nk/2))*polygon_rad;
        //  qM3(ii,:) -= P(kk,:) / ( MU_{ii,k%(nk/2)} * polygon_rad )
        //  qq3(ii)   +=   z(kk) / ( MU_{ii,k%(nk/2)} * polygon_rad )
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
    Vec w(size_null_space);
    if(!solve_qp(qG,qc,qM,qq,w)){
      OUT_LOG(logERROR)  << "ERROR: Unable to solve stage 2!" << std::endl;
//      assert(false);
      return;
      // then skip to calculating uff from stage 1 solution
    } else {
      OUTLOG(w,"W_OP2");

      // measure feasibility of solution
      // qM w - qq >= 0
      feas = qq;
      qM.mult(w,feas,1,-1);

      OUTLOG(feas,"feas_OP2 =[ % (A*w - b >= 0)");

      // return the solution (contact forces)
      // cf = z + P*w;

      P.mult(w,cf,1,1);
    }
  }

  OUTLOG(fID,"fID");
  OUTLOG(cf,"final_contact_force");
  //  Note compare contact force prediction to Moby contact force

  // return the inverse dynamics forces
  // uff = fID + iF*(vqstar - k - FET*R*(cf))/h
  uff = vqstar;
  uff -= k;
  FET.mult(R,workM1);
  workM1.mult(cf,uff,-1,1);
  LA_.solve_chol_fast(iF,uff);
  uff /= h;
  uff += fID;
  OUTLOG(uff,"final_joint_torque");

//    assert(false);
}
