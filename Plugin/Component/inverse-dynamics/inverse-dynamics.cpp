/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 *
 *  Dynamics with contact
 *  v = generalized_qd
 *  --- Dynamics model ---
 *  v- = v + h inv(M)(P tau + fext)
 *  --- Contact model ---
 *  v+ = v- + inv(M)([ N, ST+, ST- ] z)
 *  --- Contact model + Dynamics model ---
 *  v+ = v + inv(M)([ N, ST+, ST- ] z + h P tau + h fext)
 ****************************************************************************/
#include <Pacer/utilities.h>
#include <Moby/LCP.h>

int N_SYSTEMS = 0;

using Ravelin::VectorNd;
using Ravelin::MatrixNd;
using namespace Ravelin;

const double NEAR_ZERO = 1e-16;

const double grav = 9.8;
Ravelin::LinAlgd _LA;
Moby::LCP _lcp;

Ravelin::Vector3d workv3_;

Ravelin::VectorNd STAGE1, STAGE2;

////////////////////////////////////////////////////////////////////////////////
////////////////////////// EXTERNAL DECLEARATIONS //////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////
/////// Maximal Dissipation Model ///////
/////////////////////////////////////////
#include "maximal-dissipation.cpp"

/** Maximal Dissipation Model (contact + inverse dynamics) -- no-slip -- 2 stage
 *  min{z}  v+' M v+
 *  such that:
 *  v+ = v + inv(M)([ N, ST+, ST- ] z + h fext + P tau)
 *  N' v+ >= 0
 *  f_N >= 0
 *  P' v+ = v + h qdd
 **/
//extern bool inverse_dynamics_no_slip(const Ravelin::VectorNd& v, const Ravelin::VectorNd& qdd, const Ravelin::MatrixNd& M,const  Ravelin::MatrixNd& N, const Ravelin::MatrixNd& ST, const Ravelin::VectorNd& fext_, double h, Ravelin::VectorNd& x, Ravelin::VectorNd& cf_final);

/** Maximal Dissipation Model (contact + inverse dynamics) -- Algebraic setup -- 2 stage
 *  OUTPUT: z2
 * --- STAGE I ---
 *  min{z} f(z1) = v+' M v+
 *  such that:
 *  v+ = v + inv(M)([ N, ST+, ST- ] z + h fext + P tau)
 *  N' v+ >= 0
 *  f_N >= 0
 *  mu * f_N{i} >= 1' [f_S{i}; f_T{i}] for i=1:nc
 *  P' v+ = v + h qdd
 * --- STAGE II ---
 *  min{z} g(z2) = tau' tau
 *  such that:
 *  v+ = v + inv(M)([ N, ST+, ST- ] z + h fext + P tau)
 *  N' v+ >= 0
 *  f_N >= 0
 *  mu * f_N{i} >= 1' [f_S{i}; f_T{i}] for i=1:nc
 *  P' v+ = v + h qdd
 *  f(z2) <= f(z1)
 **/
//extern bool inverse_dynamics_two_stage(const Ravelin::VectorNd& v, const Ravelin::VectorNd& qdd, const Ravelin::MatrixNd& M,const  Ravelin::MatrixNd& N, const Ravelin::MatrixNd& ST, const Ravelin::VectorNd& fext_, double h, const Ravelin::MatrixNd& MU, Ravelin::VectorNd& x, Ravelin::VectorNd& cf_final);

//extern bool inverse_dynamics_two_stage_simple(const Ravelin::VectorNd& v, const Ravelin::VectorNd& qdd, const Ravelin::MatrixNd& M,const  Ravelin::MatrixNd& N,const Ravelin::MatrixNd& ST, const Ravelin::VectorNd& fext, double h, const Ravelin::MatrixNd& MU, Ravelin::VectorNd& x, Ravelin::VectorNd& cf_final);

/** Maximal Dissipation Model (contact + inverse dynamics) -- Algebraic setup -- 1 stage
 *  min{z}  v+' M v+
 *  such that:
 *  v+ = v + inv(M)([ N, ST+, ST- ] z + h fext + P tau)
 *  N' v+ >= 0
 *  f_N >= 0
 *  mu * f_N{i} >= 1' [f_S{i}; f_T{i}] for i=1:nc
 *  P' v+ = v + h qdd
 **/
//extern bool inverse_dynamics_one_stage(const Ravelin::VectorNd& v, const Ravelin::VectorNd& qdd, const Ravelin::MatrixNd& M,const  Ravelin::MatrixNd& N,const Ravelin::MatrixNd& ST, const Ravelin::VectorNd& fext_, double h, const Ravelin::MatrixNd& MU, Ravelin::VectorNd& x, Ravelin::VectorNd& cf_final);

/** Maximal Dissipation Model (contact only)
 *  min{z}  v+' M v+
 *  such that:
 *  v+ = v- + inv(M)([ N, ST+, ST- ] z)
 *  N' v+ >= 0
 *  f_N >= 0
 *  mu * f_N{i} >= 1' [f_S{i}; f_T{i}] for i=1:nc
 **/
//extern bool predict_contact_forces(const Ravelin::VectorNd& v, const Ravelin::MatrixNd& M,const  Ravelin::MatrixNd& N, const Ravelin::MatrixNd& ST, const Ravelin::MatrixNd& MU, Ravelin::VectorNd& z);

/////////////////////////////////////////
/////////// LCP NO-SLIP Model ///////////
/////////////////////////////////////////
#include "no-slip.cpp"

//extern bool inverse_dynamics_no_slip_fast(const Ravelin::VectorNd& vel, const Ravelin::VectorNd& qdd, const Ravelin::MatrixNd& M,const  Ravelin::MatrixNd& nT, const Ravelin::MatrixNd& D, const Ravelin::VectorNd& fext, double dt, Ravelin::VectorNd& x, Ravelin::VectorNd& cf, bool frictionless, std::vector<unsigned>& indices, int active_eefs,bool SAME_AS_LAST_CONTACTS);

/////////////////////////////////////////
///////////// LCP AP Model //////////////
/////////////////////////////////////////
#include "anitescu-potra.cpp"

//extern bool inverse_dynamics_ap(const Ravelin::VectorNd& vel, const Ravelin::VectorNd& qdd, const Ravelin::MatrixNd& M,const  Ravelin::MatrixNd& NT, const Ravelin::MatrixNd& D_, const Ravelin::VectorNd& fext, double dt, const Ravelin::MatrixNd& MU, Ravelin::VectorNd& x, Ravelin::VectorNd& cf);

/////////////////////////////////////////
/////////// MAKE TRAJ FEASIBLE //////////
/////////////////////////////////////////
//#include "fix-trajectory.cpp"
/** Rigid Contact Model (contact + inverse dynamics)
 *  min{v+, tau}  || P v+ - qdd ||
 *  such that:
 *  v+ = v + inv(M)([ N, ST+, ST- ] z + h fext + P tau)
 *  N' v+ >= 0
 *  f_N >= 0
 *  IF(mu < INF)
 *    mu * f_N{i} >= 1' [f_S{i}; f_T{i}] for i=1:nc
 *  ELSE
 *    S' v+ >= 0
 *    T' v+ >= 0
 *  P' v+ = v + h qdd
 **/
//extern bool fix_trajectory(const Ravelin::VectorNd& v, const Ravelin::VectorNd& qdd, const Ravelin::MatrixNd& M,const  Ravelin::MatrixNd& N,const Ravelin::MatrixNd& S,const Ravelin::MatrixNd& T, const Ravelin::VectorNd& fext, double h, const Ravelin::MatrixNd& MU);


////////////////////////////////////////////////////////////////////////////////
////////////////////////// END EXTERNAL DECLEARATIONS //////////////////////////
////////////////////////////////////////////////////////////////////////////////

void log_idyn_matrices(const Ravelin::VectorNd& v, const Ravelin::MatrixNd& M,const  Ravelin::MatrixNd& nT,
                                   const Ravelin::MatrixNd& D, const Ravelin::VectorNd& fext, double dt){
  Ravelin::MatrixNd _workM, _workM2;
  Ravelin::VectorNd _workv, _workv2;
  
  const double NEAR_ZERO = ::NEAR_ZERO;
  Ravelin::MatrixNd NT = nT;
  OUT_LOG(logDEBUG) << ">> log_idyn_matrices() entered" << std::endl;
  
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
  
  OUT_LOG(logDEBUG) << "<< log_idyn_matrices() exited" << std::endl;
}

/** Calculate forward dynamics
 *  --- Dynamics model ---
 *  v+ = v + h inv(M)(P tau + fext)
 **/
Ravelin::VectorNd forward_dynamics_no_contact(const Ravelin::MatrixNd& iM_chol,const Ravelin::VectorNd& fext, double h, Ravelin::VectorNd& x){
  OUT_LOG(logDEBUG) << ">> forward_dynamics() entered" << std::endl;
  
  // get number of degrees of freedom and number of contact points
  int n = fext.rows();
  int nq = n - 6;
  
  // add joint forces to FD
  Ravelin::VectorNd F_total = fext;
  F_total.segment(0,nq) += x;
  
  LA_.solve_chol_fast(iM_chol,F_total);
  
  OUT_LOG(logDEBUG) << "<< forward_dynamics() exited" << std::endl;
  return F_total;
}

/** Calculate forward dynamics
 *  --- Contact model + Dynamics model ---
 *  v+ = v + inv(M)([ N, ST+, ST- ] z + h P tau + h fext)
**/
Ravelin::VectorNd forward_dynamics(const Ravelin::MatrixNd& iM_chol,const  Ravelin::MatrixNd& N,
                      const Ravelin::MatrixNd& ST, const Ravelin::VectorNd& fext, double h, Ravelin::VectorNd& x, const Ravelin::VectorNd& cf){
  OUT_LOG(logDEBUG) << ">> forward_dynamics() entered" << std::endl;
  
  Ravelin::VectorNd qdd;
  // get number of degrees of freedom and number of contact points
  int n = fext.rows();
  int nq = n - 6;
  int nc = N.columns();
  
  qdd = forward_dynamics_no_contact(iM_chol,fext,h,x);
  if(nc == 0)
    return qdd;
  
  int nk = ST.columns()/nc;
  int nvars = nc + nc*(nk);
  // setup R
  Ravelin::MatrixNd R(n, nc + (nc*nk) );
  R.block(0,n,0,nc) = N;
  R.block(0,n,nc,nc*nk+nc) = ST;
  
  Ravelin::VectorNd F_contact;

  // add contact forces to FD
  R.mult(cf,F_contact);
  
  LA_.solve_chol_fast(iM_chol,F_contact);
  
  qdd += F_contact;
  OUT_LOG(logDEBUG) << "<< forward_dynamics() exited" << std::endl;
  return qdd;
}