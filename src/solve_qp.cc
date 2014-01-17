#include <project_common.h>
#include <Opt/LCP.h>


//#define NDEBUG

Moby::LCP lcp_;
//Opt::LCP lcp_;

typedef Ravelin::MatrixNd Mat;
typedef Ravelin::VectorNd Vec;

const double NEAR_ZERO = sqrt(std::numeric_limits<double>::epsilon()); //2e-12;
const double NEAR_INF = 1.0/NEAR_ZERO;

bool isvalid(const Vec& v){
  if(v.norm() > NEAR_INF || !isfinite(v.norm()))
    return false;
  return true;
}

bool solve_qp_pos(const Mat& Q, const Vec& c, const Mat& A, const Vec& b, Vec& x)
{
     const int n = Q.rows();
     const int m = A.rows();

     Mat MMM;
     MMM.set_zero(n + m,n + m);

     Vec zzz(n + m),qqq(n + m);
     qqq.set_zero();
     zzz.set_zero();
     // init and setup MMM
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
  bool SOLVE_FLAG = true;

#ifndef NDEBUG
  std::cout << "% >> solve qp positive" << std::endl;
  std::cout << "%QP variables" << std::endl;
  OUTLOG(Q,"Q");
  OUTLOG(c,"c");
  OUTLOG(A,"AA");
  OUTLOG(b,"bb");
//  std::cout << "LCP variables" << std::endl;
//  OUTLOG(MMM,"MM");
//  OUTLOG(qqq,"qq");
#endif

  if(!lcp_.lcp_lemke(MMM,qqq,zzz))
    SOLVE_FLAG = false;
  else
    SOLVE_FLAG = isvalid(zzz);

  // extract x
  for(int i=0;i<n;i++)
      x[i] = zzz[i];
#ifndef NDEBUG
  std::cout << "%Solutions" << std::endl;
//  OUTLOG(zzz,"zz");
  OUTLOG(x,"xx");
  std::cout << "% << solve qp positive" << std::endl;
#endif
  return SOLVE_FLAG;
}

bool solve_qp_pos(const Mat& Q, const Vec& c, Vec& x)
{
     const int n = Q.rows();
     const int m = 0;

     // solve the LCP
  x.set_zero();
  bool SOLVE_FLAG = true;

#ifndef NDEBUG
  std::cout << " >> solve qp positive" << std::endl;
//  std::cout << "LCP variables" << std::endl;
//  OUTLOG(Q,"MM");
//  OUTLOG(c,"qq");
#endif

  if(!lcp_.lcp_lemke_regularized(Q,c,x))
//    if(!lcp_.lcp_lemke(Q,c,x))
      SOLVE_FLAG = false;
  else
    SOLVE_FLAG = isvalid(x);

#ifndef NDEBUG
  std::cout << "Solutions" << std::endl;
  OUTLOG(x,"zz");
  std::cout << " << solve qp positive" << std::endl;
#endif
  return SOLVE_FLAG;
}

bool solve_qp(const Mat& Q, const Vec& c, const Mat& A, const Vec& b, Vec& x)
{
  const int n = Q.rows();
  const int m = A.rows();

  // setup the LCP matrix
  // MMM = |  Q -Q -A' |
  //       | -Q  Q  A' |
  //       |  A -A  0  |
  Mat MMM,AT = A, nA = A, nQ = Q;
  MMM.set_zero(Q.rows()*2 + A.rows(), Q.rows()*2 + A.rows());

  nA.negate();
  AT.transpose();
  nQ.negate();

  Vec zzz,qqq, nc=c, nb=b;
  qqq.set_zero(MMM.rows());
  zzz.set_zero(MMM.rows());

  nc.negate();
  nb.negate();

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
  qqq.set_sub_vec(0,c);
  qqq.set_sub_vec(n,nc);
  qqq.set_sub_vec(2*n,nb);

#ifndef NDEBUG
  std::cout << "% >> solve qp" << std::endl;
  std::cout << "%QP variables" << std::endl;
  OUTLOG(Q,"Q");
  OUTLOG(c,"c");
  OUTLOG(A,"AA");
  OUTLOG(b,"bb");
//  std::cout << "LCP variables" << std::endl;
//  OUTLOG(MMM,"MM");
//  OUTLOG(qqq,"qq");
#endif

  // solve the LCP
  bool SOLVE_FLAG = true;
  double zero_tol = MMM.norm_inf()*MMM.rows()*std::numeric_limits<double>::epsilon() * 1e4;
  if(!lcp_.lcp_lemke_regularized(MMM,qqq,zzz,-20,4,0,-1.0,zero_tol))
//  if(!lcp_.lcp_lemke(MMM,qqq,zzz))
    SOLVE_FLAG = false;
  else
    SOLVE_FLAG = isvalid(zzz);

  // extract x
  for(int i=0;i<n;i++)
    x[i] = zzz[i] - zzz[n+i];

#ifndef NDEBUG
  std::cout << "%Solutions" << std::endl;
//  OUTLOG(zzz,"zz");
  OUTLOG(x,"xx");
  std::cout << "% << solve qp" << std::endl;
#endif
  return SOLVE_FLAG;
}
