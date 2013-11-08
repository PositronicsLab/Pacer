#include <Control.h>

//#define NDEBUG

Moby::LCP lcp_;
Ravelin::LinAlgd LA_;

 static bool solve_qp_pos(const Mat& Q, const Vec& c, const Mat& A, const Vec& b, Vec& x)
{
     const int n = Q.rows();
     const int m = A.rows();

     Mat MMM;
     Vec zzz(n + m),qqq(n + m);
     // init and setup MMM
     MMM.set_zero(n + m,n + m);
     Mat nAT = A;
     nAT.transpose();
     nAT.negate();
     MMM.set_sub_mat(0,0,Q);
     MMM.set_sub_mat(0,n,nAT);
     MMM.set_sub_mat(n,0,A);

  // setup qqq
  qqq.set_sub_vec(0,c);
  Vec nb = b;
  nb.negate();
  qqq.set_sub_vec(n,nb);

  // solve the LCP
  zzz.set_zero();
  bool SOLVE_FLAG = true;

#ifndef NDEBUG
  std::cout << " >> solve qp positive" << std::endl;
  std::cout << "QP variables" << std::endl;
  outlog(Q,"G");
  outlog(c,"c");
  outlog(A,"A");
  outlog(b,"b");
  std::cout << "LCP variables" << std::endl;
  outlog(MMM,"M");
  outlog(qqq,"q");
#endif

  if(!lcp_.lcp_lemke_regularized(MMM,qqq,zzz))
//    if(!lcp_.lcp_lemke(MMM,qqq,zzz))
      SOLVE_FLAG = false;

  // extract x
  for(int i=0;i<n;i++)
      x[i] = zzz[i];
#ifndef NDEBUG
  std::cout << "Solutions" << std::endl;
  outlog(zzz,"LCPz");
  outlog(x,"QPx");
  std::cout << " << solve qp positive" << std::endl;
#endif
  return SOLVE_FLAG;
}

 static bool solve_qp_pos(const Mat& Q, const Vec& c, Vec& x)
{
     const int n = Q.rows();
     const int m = 0;

     // solve the LCP
  x.set_zero();
  bool SOLVE_FLAG = true;

#ifndef NDEBUG
  std::cout << " >> solve qp positive" << std::endl;
  std::cout << "QP variables" << std::endl;
  std::cout << "LCP variables" << std::endl;
  outlog(Q,"G");
  outlog(c,"c");
#endif

  if(!lcp_.lcp_lemke_regularized(Q,c,x))
//    if(!lcp_.lcp_lemke(Q,c,x))
      SOLVE_FLAG = false;
#ifndef NDEBUG
  std::cout << "Solutions" << std::endl;
  outlog(x,"QPx");
  std::cout << " << solve qp positive" << std::endl;
#endif
  return SOLVE_FLAG;
}


static bool solve_qp(const Mat& Q, const Vec& c, const Mat& A, const Vec& b, Vec& x)
{
  const int n = Q.rows();
  const int m = A.rows();

  // setup the LCP matrix
  // MMM = |  Q -Q -A' |
  //       | -Q  Q  A' |
  //       |  A -A  0  |
  Mat MMM,AT = A, nA = A, nQ = Q;
  nA.negate();
  AT.transpose();
  nQ.negate();
  Vec zzz,qqq, nc=c, nb=b;
  nc.negate();
  nb.negate();
  MMM.set_zero(Q.rows()*2 + A.rows(), Q.rows()*2 + A.rows());
  MMM.set_sub_mat(0,0,Q);
  MMM.set_sub_mat(n,n,Q);
  MMM.set_sub_mat(0,n,nQ);
  MMM.set_sub_mat(n,0,nQ);

  // setup linear inequality constraints in LCP matrix
  MMM.set_sub_mat(n,n*2,AT);
  AT.negate();
  MMM.set_sub_mat(0,n*2,AT);
  MMM.set_sub_mat(n*2,0,A);
  MMM.set_sub_mat(n*2,n,nA);

  // setup LCP vector
  qqq.resize(MMM.rows());
  qqq.set_sub_vec(0,c);
  qqq.set_sub_vec(n,nc);
  qqq.set_sub_vec(2*n,nb);

#ifndef NDEBUG
  std::cout << " >> solve qp" << std::endl;
  std::cout << "QP variables" << std::endl;
  outlog(Q,"G");
  outlog(c,"c");
  outlog(A,"A");
  outlog(b,"b");
  std::cout << "LCP variables" << std::endl;
  outlog(MMM,"M");
  outlog(qqq,"q");
#endif

  // solve the LCP
  zzz.set_zero(qqq.size());

  bool SOLVE_FLAG = true;
  if(!lcp_.lcp_lemke_regularized(MMM,qqq,zzz))
//    if(!lcp_.lcp_lemke(MMM,qqq,zzz))
      SOLVE_FLAG = false;

  // extract x
  for(int i=0;i<n;i++)
      x[i] = zzz[i] - zzz[n+i];

#ifndef NDEBUG
  std::cout << "Solutions" << std::endl;
  outlog(zzz,"LCPz");
  outlog(x,"QPx");
  std::cout << " << solve qp positive" << std::endl;
#endif
  return SOLVE_FLAG;
}

void idyn(const Vec& v, const Vec& qdd, const Mat& M,const  Mat& N,
                         const Mat& ST, const Vec& fext, double h, const Mat& MU, Vec& uff){


  // get number of degrees of freedom and number of contact points
  int n = M.rows();
  int nq = n - 6;
  int nc = N.columns();
  int nk = ST.columns()/nc;
  int nvars = nc*(nk+1);

  Mat workM1,workM2;
  Vec workv1, workv2;

  Vec vqstar = qdd;
  vqstar *= h;
  vqstar += v.get_sub_vec(6,n,workv1);

  // setup R
  Mat R(n, nc + (nc*nk) );
  R.set_sub_mat(0,0,N);
  R.set_sub_mat(0,nc,ST);

#ifndef NDEBUG
  outlog2(M,"M");
  outlog2(N,"N");
  outlog2(ST,"ST");
  outlog2(v,"v");
  outlog2(vqstar,"vqstar");
  outlog2(fext,"fext");
  outlog2(MU,"MU");
  outlog2(R,"R");
#endif



  Vec z(nvars),cf(nvars);
  // compute A, B, and C
  Mat A(6,6);
  M.get_sub_mat(nq,n,nq,n,A);
  Mat B(6,nq);
  M.get_sub_mat(nq,n,0,nq,B);
  Mat C(nq,nq);
  M.get_sub_mat(0,nq,0,nq,C);

  // compute D, E, and F
  Mat iM_chol = M;
  LA_.factor_chol(iM_chol);

  Mat iM = Mat::identity(n);
  LA_.solve_chol_fast(iM_chol,iM);
  Mat D(6,6);
  iM.get_sub_mat(nq,n,nq,n,D);
  Mat E(6,nq);
  iM.get_sub_mat(nq,n,0,nq,E);
  Mat F(nq,nq);
  iM.get_sub_mat(0,nq,0,nq,F);
  Mat iF = F;
  LA_.factor_chol(iF);

  /// Stage 1 optimization energy minimization

  // determine vb, vq, vbstar, vqstar

  Vec vb(6);
  v.get_sub_vec(nq,n,vb);
  Vec vq(nq);
  v.get_sub_vec(0,nq,vq);

  workv1.set_zero(nq);
  uff.set_zero(nq);
  // compute tff
  workv1 = vqstar;
  workv1 -= vq;
  workv1 *= (1/h);
  fext.get_sub_vec(0,nq,uff);
  F.mult(workv1,uff,1,-1);

  Vec zuff;
  zuff.set_zero(n);
  zuff.set_sub_vec(6,uff);

  // compute j and k
  // [D E]
  Mat DE(D.rows(),D.columns()+E.columns());
  DE.set_sub_mat(0,0,D);
  DE.set_sub_mat(0,D.columns(),E);
  // [E'F]
  Mat ETF(E.columns(),E.rows()+F.columns()), ET = E;
  ET.transpose();
  ETF.set_sub_mat(0,0,ET);
  ETF.set_sub_mat(0,ET.columns(),F);

  // j and k
  workv1 = fext;
  workv1 += zuff;
  workv1 *= h;
  Vec j(6);
  DE.mult(workv1,j);
  j += vb;
  Vec k = vq;
  ETF.mult(workv1,k,1,1);
//      k += vq;

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
//      p += j;

  Mat H(Z.columns(),Z.columns());
  // compute objective function
  Z.transpose_mult(A,workM1);
  workM1.mult(Z,H);
  Mat qG = H;
  // qc = Z'*A*p + Z'*B*vqstar;
  Vec qc(Z.columns());
  workM1.mult(p,qc);
  Z.transpose_mult(B,workM2);
  workM2.mult(vqstar,qc,1,1);
//      workM2.mult(vqstar,workv1);
//      qc += workv1;

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
  for (int ii=0;ii<nc;ii++){
    // normal force
    qM2(ii,ii) = MU(ii,0);

    // tangent forces
    for(int kk=nc+nk*ii;kk<nc+nk*ii+nk/2;kk++)
      qM2(ii,kk) = -1;

    // negative tangent force
    for(int kk=nc+nk*ii+nk/2;kk<nc+nk*ii+nk;kk++)
      qM2(ii,kk) = -1;

    // rhs is zer
    qq2[ii] = 0;
  }
  // combine all linear inequality constraints
  assert(qM1.columns() == qM2.columns());
  Mat qM(qM1.rows()+qM2.rows(),qM1.columns());
  Vec qq(qq1.rows()+qq2.rows());
  qM.set_sub_mat(0,0,qM1);
  qM.set_sub_mat(qM1.rows(),0,qM2);
  qq.set_sub_vec(0,qq1);
  qq.set_sub_vec(qq1.rows(),qq2);

#ifndef NDEBUG
  outlog2(qG,"qG");
  outlog2(qM,"qM");
  outlog2(qc,"qc");
  outlog2(qq,"q");
#endif
  if(!solve_qp_pos(qG,qc,qM,qq,z)){
    std::cout  << "ERROR: Unable to solve stage 1!" << std::endl;
    return;
  }

  Vec feas = qq;
  qM.mult(z,feas,1,-1);
#ifndef NDEBUG
  outlog(feas,"feas1op");
  outlog(z,"z1op");
#endif

  /// Stage 2 optimization command smoothing

  // H = Z'AZ
  int m = 0;
  Mat U,V,P;
  Vec S;

  // SVD decomp to retrieve nullspace of Z'AZ
  LA_.svd(H,U,S,V);

//    outlog2(S,"S");

  if(S.rows() != 0){
    // Calculate the tolerance for ruling that a singular value is supposed to be zero
    double ZERO_TOL = std::numeric_limits<double>::epsilon() * H.rows() * S[0];
    // Count Zero singular values
    for(int i = S.rows()-1;i>=0;i--,m++)
      if(S[i] > ZERO_TOL)
        break;

//        std::cout  << "Null Vectors: " << m << std::endl;

    // get the nullspace
    P.set_zero(nvars,m);
    V.get_sub_mat(0,V.rows(),V.columns()-m,V.columns(),P);
  }

  if(m != 0 && false)
  {
    // compute U
    Mat U;
    ETF.mult(R,U);

    // second optimization necessary
    // compute objective function
//        qG = P'*U'*iF'*iF*U*P;
    workM1 = U;
    LA_.solve_chol_fast(iF,workM1);
    workM1.mult(P,workM2);
    workM2.transpose_mult(workM2,qG);

    //  qc = z'*U'*iF'*iF*U*P - vqstar'*iF'*iF*U*P + k'*iF'*iF*U*P;
    workM2.transpose_mult(workM1.mult(z,workv1),workv2);
    qc = workv2;

    workv1 = vqstar;
    workM2.transpose_mult(LA_.solve_chol_fast(iF,workv1),workv2);
    qc -= workv2;

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
    nvars = P.columns();
    Mat qM3(nc, nvars);
    qM3.set_zero();
    Vec qq3(nc);
    qq3.set_zero();
    for (int ii=0;ii < nc;ii++){
      // normal force
      //  qM3(ii,:) = P(ii,:)*MU(ii,0)
      //  qq3(ii,1) = -z(ii)*MU(ii,0)
      workv1 = P.row(ii);
      workv1 *= MU(ii,0);
        qM3.set_row(ii,workv1);
      qq3[ii] = -z[ii]*MU(ii,0);

      // first direction tangent forces
      for (int kk=nc+nk*ii;kk<nc+nk*ii+nk/2;kk++){
        //  qM3(ii,:) -= P(kk,:)*MU(ii,2)^2;
        //  qq3(ii,1) +=  z(kk)*MU(ii,2)^2;
        workv1 = qM3.row(ii);
        workv1 -= P.row(kk);
        qM3.set_row(ii,workv1);
        qq3[ii] = qq3[ii] + z[kk];
      }

      // second direction tangent forces
      for (int kk=nc+nk*ii+nk/2;kk<nc+nk*ii+nk;kk++){
        //  qM3(ii,:) -= P(kk,:)*MU(ii,1)^2;
        //  qq3(ii,1) +=  z(kk)*MU(ii,0)^2;
        workv1 = qM3.row(ii);
        workv1 -= P.row(kk);
        qM3.set_row(ii,workv1);
        qq3[ii] = qq3[ii] + z[kk];
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
        qM.mult(w,feas,1,-1);

#ifndef NDEBUG
      outlog(feas,"feas2op");
      outlog(w,"w2op");
#endif
        // return the solution (contact forces)
        // cf = z + P*w;
        cf = z;
        P.mult(w,cf,1,1);

        // return the inverse dynamics forces
      } else {
        cf = z;
      }
#ifndef NDEBUG
    outlog(cf,"finalCF");
#endif
       // return the inverse dynamics forces
//    uff += iF*(vqstar - k - ETF*R*(cf)) * (1/h);
    uff = vqstar;
    uff -= k;
    ETF.mult(R,workM1);
    workM1.mult(cf,uff,-1,1);
    uff *= (1/h);
    LA_.solve_chol_fast(iF,uff);
#ifndef NDEBUG
    outlog(uff,"finalUFF");
#endif
}
