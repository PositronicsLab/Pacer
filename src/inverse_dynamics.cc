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
  int nk = ST.columns()/nc;
  int nvars = nc*(nk+1);

  static Mat workM1,workM2;
  static Vec workv1, workv2;

  static Vec vb(6);
  v.get_sub_vec(nq,n,vb);
  static Vec vq(nq);
  v.get_sub_vec(0,nq,vq);

  static Vec vqstar;
  vqstar = qdd;
  vqstar *= h;
  vqstar += vq;

  // setup R
  Mat R(n, nc + (nc*nk) );
  R.set_sub_mat(0,0,N);
  R.set_sub_mat(0,nc,ST);

  // Log these function variables
  OUTLOG(M,"M");
  OUTLOG(N,"N");
  OUTLOG(ST,"ST");
  OUTLOG(v,"v");
  OUTLOG(vqstar,"vqstar");
  OUTLOG(fext,"fext");
  OUTLOG(MU,"MU");
  OUTLOG(R,"R");

  Vec z(nvars),cf(nvars);
  // compute A, B, and C
  static Mat A(6,6);
  M.get_sub_mat(nq,n,nq,n,A);
  static Mat B(6,nq);
  M.get_sub_mat(nq,n,0,nq,B);
  static Mat C(nq,nq);
  M.get_sub_mat(0,nq,0,nq,C);

  // compute D, E, and F
  static Mat iM_chol;
  iM_chol = M;
  LA_.factor_chol(iM_chol);

  static Mat iM;
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

  /// Stage 1 optimization energy minimization

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

//  return;
  static Vec zuff(n);
  zuff.set_zero();
  zuff.set_sub_vec(0,fID);

  // compute j and k
  // [D,E]
  Mat DE(D.rows(),D.columns()+E.columns());
  DE.set_sub_mat(0,0,D);
  DE.set_sub_mat(0,D.columns(),E);
  // [E',F]
  Mat ETF(E.columns(),E.rows()+F.columns()),
      ET = E;
  ET.transpose();
  ETF.set_sub_mat(0,0,ET);
  ETF.set_sub_mat(0,ET.columns(),F);

  // j and k
  workv1 = fext;
  workv1 += zuff;
  workv1 *= h;
  // j = [D,E]*(fext + zuff)*h
  Vec j(6);
  DE.mult(workv1,j);
  j += vb;
  // k = [E',F]*(fext + zuff)*h  +  vq
  Vec k = vq;
  ETF.mult(workv1,k,1,1);

  // compute Z and p
  Mat Z(DE.rows(), R.columns());
  workM1 = ETF;
  LA_.solve_chol_fast(iF,workM1);
  E.mult(workM1,workM2);
  workM2.negate();
  workM2 += DE;
  workM2.mult(R,Z);

  Vec p = j;
  workv1 = vqstar;
  workv1 -= k;
  LA_.solve_chol_fast(iF,workv1);
  E.mult(workv1,p,1,1);

  Mat H(Z.columns(),Z.columns());
  // compute objective function
  Z.transpose_mult(A,workM1);
  workM1.mult(Z,H);

  /////////////////////////////////////////////////////////
  ////////////// Objective //////////////////////////////

  Mat qG = H;
  // qc = Z'*A*p + Z'*B*vqstar;
  Vec qc(Z.columns());
  // HINT: workM1 = Z'*A
  // qc = Z'*A*p
  workM1.mult(p,qc);
  Z.transpose_mult(B,workM2);
  // qc += Z'*B*vqstar
  workM2.mult(vqstar,qc,1,1);

  // setup linear inequality constraints -- noninterpenetration
  Mat Z1(n,Z.columns());
  Z1.set_zero();
  Z1.set_sub_mat(0,0,Z);

  Mat qM1(N.columns(),Z1.columns());
  N.transpose_mult(Z1,qM1);

  Vec pvqstar(n);
  pvqstar.set_sub_vec(nq,p);
  pvqstar.set_sub_vec(0,vqstar);
  pvqstar.negate();

  Vec qq1(N.columns());
  N.transpose_mult(pvqstar,qq1);

  // setup linear inequality constraints -- friction
  Mat qM2 = Mat::zero(nc, nvars);
  Vec qq2(nc);
  double polygon_rad = cos(M_PI/nk);
  for (int ii=0;ii < nc;ii++){
      double friction = polygon_rad*MU(ii,0);
    // normal force
    qM2(ii,ii) = friction;

    // tangent forces
    for(int kk=nc+nk*ii;kk<nc+nk*ii+nk/2;kk++)
      qM2(ii,kk) = -1.0;

    // negative tangent force
    for(int kk=nc+nk*ii+nk/2;kk<nc+nk*ii+nk;kk++)
      qM2(ii,kk) = -1.0;

    // rhs is zer
    qq2[ii] = 0.0;
  }

  /////////////////////////////////////////////////////////
  ////////////// CONSTRAINTS //////////////////////////////

  // combine all linear inequality constraints
  assert(qM1.columns() == qM2.columns());
  Mat qM(qM1.rows()+qM2.rows(),qM1.columns());
  Vec qq(qq1.rows()+qq2.rows());
  qM.set_sub_mat(0,0,qM1);
  qM.set_sub_mat(qM1.rows(),0,qM2);
  qq.set_sub_vec(0,qq1);
  qq.set_sub_vec(qq1.rows(),qq2);

  if(!solve_qp_pos(qG,qc,qM,qq,z)){
    std::cout  << "ERROR: Unable to solve stage 1!" << std::endl;
    return;
  }

  Vec feas = qq;
  qM.mult(z,feas,1,-1);

  OUTLOG(feas,"feas_OP1 =[ % (A*z-b >= 0)");
  OUTLOG(z,"Z_OP1");

  /// Stage 2 optimization command smoothing

  // H = Z'AZ
  int m = 0;
  Mat U,V,P;
  Vec S;

  // SVD decomp to retrieve nullspace of Z'AZ
  LA_.svd(H,U,S,V);

  if(S.rows() != 0){
    // Calculate the tolerance for ruling that a singular value is supposed to be zero
    double ZERO_TOL = std::numeric_limits<double>::epsilon() * H.rows() * S[0];
    // Count Zero singular values
    for(int i = S.rows()-1;i>=0;i--,m++)
      if(S[i] > ZERO_TOL)
        break;

    // get the nullspace
    P.set_zero(nvars,m);
    V.get_sub_mat(0,V.rows(),V.columns()-m,V.columns(),P);
  }

  OUTLOG(S,"Singular Values");
  OUTLOG(P,"Null Space(P)");

  if(m != 0 && false)
  {
    // compute U
    Mat U;
    ETF.mult(R,U);

    // second optimization necessary
    // compute objective function
    // qG = P'*U'*iF'*iF*U*P;
    workM1 = U;
    LA_.solve_chol_fast(iF,workM1);
    workM1.mult(P,workM2);
    workM2.transpose_mult(workM2,qG);

    // HINT: workM2 = iF*U*P
    // HINT: workM1 = iF*U

    //  qc = z'*U'*iF'*iF*U*P - vqstar'*iF'*iF*U*P + k'*iF'*iF*U*P;
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

    // compute new linear inequality constraints -- z + Pw >= 0
    qM1 = P;
    qq1 = z;
    qq1.negate();

    // compute new linear inequality constraints -- noninterpenetration
    Mat ZP(6+nq,m);
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

    // compute new friction constraints
    //
    nvars = P.columns();
    Mat qM3(nc, nvars);
    qM3.set_zero();
    Vec qq3(nc);
    qq3.set_zero();

//    double polygon_rad = cos(M_PI/nk);
    for (int ii=0;ii < nc;ii++){
        double friction = polygon_rad*MU(ii,0);
      // normal force
      //  qM3(ii,:) = P(ii,:)*MU(ii,0)
      //  qq3(ii,1) = -z(ii)*MU(ii,0)
      (workv1 = P.row(ii)) *= friction;
      qM3.set_row(ii,workv1);
      qq3[ii] = -z[ii]*friction;

      // first direction tangent forces
      for (int kk=nc+nk*ii;kk<nc+nk*ii+nk/2;kk++){
        //  qM3(ii,:) -= P(kk,:);
        //  qq3(ii,1) +=  z(kk);
        qM3.row(ii) -= P.row(kk);
        qq3[ii] += z[kk];
      }

      // second direction tangent forces
      for (int kk=nc+nk*ii+nk/2;kk<nc+nk*ii+nk;kk++){
        //  qM3(ii,:) -= P(kk,:);
        //  qq3(ii,1) +=  z(kk);
        qM3.row(ii) -= P.row(kk);
        qq3[ii] += z[kk];
      }
    }

    // setup inequalities
    assert(qM1.columns() == qM2.columns() && qM1.columns() == qM3.columns());

        qM.set_zero(qM1.rows()+qM2.rows()+qM3.rows(),qM1.columns());
        qq.set_zero(qM1.rows()+qM2.rows()+qM3.rows());
        qM.set_sub_mat(0,0,qM1);
        qM.set_sub_mat(qM1.rows(),0,qM2);
        qM.set_sub_mat(qM1.rows()+qM2.rows(),0,qM3);
        qq.set_sub_vec(0,qq1);
        qq.set_sub_vec(qq1.rows(),qq2);
        qq.set_sub_vec(qq1.rows()+qq2.rows(),qq3);
        // optimize

        Vec w(m);
        if(!solve_qp(qG,qc,qM,qq,w)){
          std::cout << "ERROR: Unable to solve stage 2!" << std::endl;
          return;
        }

        feas = qq;
        qM.mult(w,feas,1,1);

      OUTLOG(feas,"feas_OP2 =[ % (A*w-b >= 0)");
      OUTLOG(w,"W_OP2");

      // return the solution (contact forces)
        // cf = z + P*w;
        cf = z;
        P.mult(w,cf,1,1);

        // return the inverse dynamics forces
      } else {
        cf = z;
      }

    OUTLOG(cf,"contact_force");

  // return the inverse dynamics forces
  // uff = fID + iF*(vqstar - k - ETF*R*(cf))/h
    OUTLOG(fID,"fID");
    uff = vqstar;
    uff -= k;
    ETF.mult(R,workM1);
    workM1.mult(cf,uff,-1,1);
    LA_.solve_chol_fast(iF,uff);
    uff /= h;

    OUTLOG(uff,"zuff");

    uff += fID;

    OUTLOG(uff,"feed_forward_force");
}
