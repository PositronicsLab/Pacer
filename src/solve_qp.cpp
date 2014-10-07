#include <project_common.h>

extern bool lcp_symm_iter(const Ravelin::MatrixNd& M, const Ravelin::VectorNd& q, Ravelin::VectorNd& z, double lambda, double omega, unsigned MAX_ITER);
Moby::LCP lcp_;

const int MAX_ITER = 100;
const double NEAR_ZERO = sqrt(std::numeric_limits<double>::epsilon()); //2e-12;
const double NEAR_INF = 1.0/NEAR_ZERO;

bool isvalid(const Ravelin::VectorNd& v){
  if(v.norm() > NEAR_INF || !std::isfinite(v.norm()))
    return false;
  return true;
}

//#define SPLITTING_METHOD

bool solve_qp_pos(const Ravelin::MatrixNd& Q, const Ravelin::VectorNd& c, const Ravelin::MatrixNd& A, const Ravelin::VectorNd& b, Ravelin::VectorNd& x)
{
     const int n = Q.rows();
     const int m = A.rows();

     Ravelin::MatrixNd MMM;
     MMM.set_zero(n + m,n + m);

     Ravelin::VectorNd zzz(n + m),qqq(n + m);
     qqq.set_zero();
     zzz.set_zero();
     // init and setup MMM
     Ravelin::MatrixNd nAT = A;
     nAT.transpose();
     nAT.negate();
     MMM.set_sub_mat(0,0,Q);
     MMM.set_sub_mat(0,n,nAT);
     MMM.set_sub_mat(n,0,A);

  // setup qqq
  qqq.set_sub_vec(0,c);
  Ravelin::VectorNd nb = b;
  nb.negate();
  qqq.set_sub_vec(n,nb);

  // solve the LCP
  bool SOLVE_FLAG = true;

#ifndef NDEBUG
  OUT_LOG(logDEBUG1)  << "% >> solve qp positive" ;
  OUT_LOG(logDEBUG1)  << "%QP variables" ;
  OUTLOG(Q,"qp_Q",logDEBUG1);
  OUTLOG(c,"qp_c",logDEBUG1);
  OUTLOG(A,"qp_A",logDEBUG1);
  OUTLOG(b,"qp_b",logDEBUG1);
//  OUT_LOG(logINFO)  << "LCP variables" << std::endl;
//  OUTLOG(MMM,"MM");
//  OUTLOG(qqq,"qq");
#endif

#ifndef SPLITTING_METHOD
  double zero_tol = MMM.norm_inf()*MMM.rows()*std::numeric_limits<double>::epsilon() * 1e4;
  if(!lcp_.lcp_lemke_regularized(MMM,qqq,zzz,-20,4,0,-1.0,zero_tol))
//  if(!lcp_.lcp_lemke(MMM,qqq,zzz))
    SOLVE_FLAG = false;
  else
    SOLVE_FLAG = isvalid(zzz);
#else
  lcp_symm_iter(MMM, qqq, zzz, 0.5, 1.0, MAX_ITER);
#endif

  // extract x
  for(int i=0;i<n;i++)
      x[i] = zzz[i];
#ifndef NDEBUG
  OUT_LOG(logDEBUG1)  << "%Solutions" ;
//  OUTLOG(zzz,"zz");
  OUTLOG(x,"xx",logDEBUG1);
  OUT_LOG(logDEBUG1)  << "% << solve qp positive" ;
#endif
  return SOLVE_FLAG;
}

bool solve_qp_pos(const Ravelin::MatrixNd& Q, const Ravelin::VectorNd& c, Ravelin::VectorNd& x)
{
     const int n = Q.rows();
     const int m = 0;

     // solve the LCP
  x.set_zero();
  bool SOLVE_FLAG = true;

#ifndef NDEBUG
  OUT_LOG(logDEBUG1)  << " >> solve qp positive" ;
//  OUT_LOG(logINFO)  << "LCP variables" << std::endl;
//  OUTLOG(Q,"MM");
//  OUTLOG(c,"qq");
#endif

  if(!lcp_.lcp_lemke_regularized(Q,c,x))
//    if(!lcp_.lcp_lemke(Q,c,x))
      SOLVE_FLAG = false;
  else
    SOLVE_FLAG = isvalid(x);

#ifndef NDEBUG
  OUT_LOG(logDEBUG1)  << "Solutions" ;
  OUTLOG(x,"zz",logDEBUG1);
  OUT_LOG(logDEBUG1)  << " << solve qp positive" ;
#endif
  return SOLVE_FLAG;
}

bool solve_qp(const Ravelin::MatrixNd& Q, const Ravelin::VectorNd& c, const Ravelin::MatrixNd& A, const Ravelin::VectorNd& b, Ravelin::VectorNd& x)
{
  const int n = Q.rows();
  const int m = A.rows();

  // setup the LCP matrix
  // MMM = |  Q -Q -A' |
  //       | -Q  Q  A' |
  //       |  A -A  0  |
  Ravelin::MatrixNd MMM,AT = A, nA = A, nQ = Q;
  MMM.set_zero(Q.rows()*2 + A.rows(), Q.rows()*2 + A.rows());

  nA.negate();
  AT.transpose();
  nQ.negate();

  Ravelin::VectorNd zzz,qqq, nc=c, nb=b;
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

  // setup LCP vector qqq = [c;-c;-b]
  qqq.set_sub_vec(0,c);
  qqq.set_sub_vec(n,nc);
  qqq.set_sub_vec(2*n,nb);

#ifndef NDEBUG
  OUT_LOG(logDEBUG1)  << "% >> solve qp" << std::endl;
  OUT_LOG(logDEBUG1)  << "%QP variables" << std::endl;
  OUTLOG(Q,"Q",logDEBUG1);
  OUTLOG(c,"c",logDEBUG1);
  OUTLOG(A,"AA",logDEBUG1);
  OUTLOG(b,"bb",logDEBUG1);
  OUT_LOG(logDEBUG1)  << "LCP variables" << std::endl;
  OUTLOG(MMM,"MMM",logDEBUG1);
  OUTLOG(qqq,"qqq",logDEBUG1);
#endif

  // solve the LCP
  bool SOLVE_FLAG = true;
#ifndef SPLITTING_METHOD
  double zero_tol = MMM.norm_inf()*MMM.rows()*std::numeric_limits<double>::epsilon() * 1e4;
  if(!lcp_.lcp_lemke_regularized(MMM,qqq,zzz,-20,4,0,-1.0,zero_tol))
//  if(!lcp_.lcp_lemke(MMM,qqq,zzz))
    SOLVE_FLAG = false;
  else
    SOLVE_FLAG = isvalid(zzz);
#else
  lcp_symm_iter(MMM, qqq, zzz, 0.5, 1.0, MAX_ITER);
#endif

  // extract x
  for(int i=0;i<n;i++)
    x[i] = zzz[i] - zzz[n+i];

#ifndef NDEBUG
  OUT_LOG(logDEBUG1)  << "%Solutions" << std::endl;
  OUTLOG(zzz,"zzz",logDEBUG1);
  OUTLOG(x,"xx",logDEBUG1);
  OUT_LOG(logDEBUG1)  << "% << solve qp" << std::endl;
#endif
  return SOLVE_FLAG;
}

#include <Opt/QPActiveSet.h>

Opt::QPActiveSet as_;

bool solve_qp(const Ravelin::MatrixNd& Q, const Ravelin::VectorNd& c, const Ravelin::VectorNd& lb_, const Ravelin::VectorNd& ub_, const Ravelin::MatrixNd& A, const Ravelin::VectorNd& b, Ravelin::VectorNd& x)
{
  const int n = Q.rows();
  const int m = A.rows();
  Ravelin::VectorNd lb = lb_;
  Ravelin::VectorNd ub = ub_;
  if(lb.rows() == 0)
    lb.set_one(Q.columns()) *= -1e29;
  if(ub.rows() == 0)
    ub.set_one(Q.columns()) *= 1e29;

#ifndef NDEBUG
  OUT_LOG(logDEBUG1)  << "% >> solve qp" << std::endl;
  OUT_LOG(logDEBUG1)  << "%QP variables" << std::endl;
  OUTLOG(Q,"Q",logDEBUG1);
  OUTLOG(c,"c",logDEBUG1);
  OUTLOG(A,"AA",logDEBUG1);
  OUTLOG(b,"bb",logDEBUG1);
  OUTLOG(lb,"lb",logDEBUG1);
  OUTLOG(ub,"ub",logDEBUG1);
#endif
  bool SOLVE_FLAG = true;
  SOLVE_FLAG = as_.qp_activeset(Q,c,lb,ub,A,b,Ravelin::MatrixNd::zero(0,c.rows()),Ravelin::VectorNd::zero(0),x);

#ifndef NDEBUG
  OUT_LOG(logDEBUG1)  << "%Solutions" << std::endl;
  OUTLOG(x,"xx",logDEBUG1);
  OUT_LOG(logDEBUG1)  << "% << solve qp" << std::endl;
#endif
  return SOLVE_FLAG;
}

#include<Opt/OptParams.h>
#include<Opt/QP.h>
//#include<Opt/ConvexOpt.h>

extern void solve_qp_matlab(const Ravelin::MatrixNd& Q, const Ravelin::VectorNd& c, const Ravelin::VectorNd& lb_, const Ravelin::VectorNd& ub_, const Ravelin::MatrixNd& A, const Ravelin::VectorNd& b,const Ravelin::MatrixNd& M, const Ravelin::VectorNd& q, Ravelin::VectorNd& x);

bool solve_qp(const Ravelin::MatrixNd& Q, const Ravelin::VectorNd& c, const Ravelin::VectorNd& lb_, const Ravelin::VectorNd& ub_, const Ravelin::MatrixNd& A, const Ravelin::VectorNd& b,const Ravelin::MatrixNd& M, const Ravelin::VectorNd& q, Ravelin::VectorNd& x)
{
  const int n = Q.rows();
  const int m = A.rows();
  Ravelin::VectorNd lb = lb_;
  Ravelin::VectorNd ub = ub_;
  if(lb.rows() == 0)
    lb.set_one(Q.columns()) *= -1e29;
  if(ub.rows() == 0)
    ub.set_one(Q.columns()) *= 1e29;

#ifndef NDEBUG
  OUT_LOG(logDEBUG1)  << "% >> solve qp" << std::endl;
  OUT_LOG(logDEBUG1)  << "%QP variables" << std::endl;
  OUTLOG(Q,"Q",logDEBUG1);
  OUTLOG(c,"c",logDEBUG1);
  OUTLOG(A,"AA",logDEBUG1);
  OUTLOG(b,"bb",logDEBUG1);
  OUTLOG(lb,"lb",logDEBUG1);
  OUTLOG(ub,"ub",logDEBUG1);
#endif
  bool SOLVE_FLAG = true;
//  SOLVE_FLAG = as_.qp_activeset(Q,c,lb,ub,A,b,M,q,x);
  static Opt::QP* qp_ = new Opt::QP();
  static Opt::OptParams* op_ = new Opt::OptParams();
  op_->A = A;
  op_->b = b;
  op_->M = M;
  op_->q = q;
  op_->n = Q.rows();
  op_->m = 0;//M.rows();
  op_->r = 0;//A.rows();
  op_->lb = lb;
  op_->ub = ub;
  op_->max_iterations = 1000;

  qp_->qp_convex_activeset(Q,c,*op_,x);

//  static Opt::ConvexOpt* qpc_ = new Opt::ConvexOpt();
//  static Opt::CvxOptParams* cop_ = new Opt::CvxOptParams(*op_);
//  qpc_->optimize_convex_pd(*cop_, x);

//  if(qp_->make_feasible_qp(A,b,M,q,x))
//    SOLVE_FLAG = Opt::QP::qp_convex_ip(Q,c,*op_,x);


#ifndef NDEBUG
  OUT_LOG(logDEBUG1)  << "%Solutions" << std::endl;
  OUTLOG(x,"xx",logDEBUG1);
  OUT_LOG(logDEBUG1)  << "% << solve qp" << std::endl;
#endif
  return SOLVE_FLAG;
}

void solve(Ravelin::MatrixNd& M,Ravelin::VectorNd& bx){
  if(M.rows() == M.columns())
    LA_.solve_fast(M,bx);
  else{
    LA_.pseudo_invert(workM_ = M);
    workM_.mult(workv_ = bx,bx);
  }
}
