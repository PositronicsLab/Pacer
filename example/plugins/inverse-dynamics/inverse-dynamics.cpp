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

#include <Pacer/controller.h>
#include <Pacer/utilities.h>
#include "../plugin.h"

#include <Moby/LCP.h>

//#define TIMING

using namespace Pacer;

int N_SYSTEMS = 0;

using Ravelin::VectorNd;
using Ravelin::MatrixNd;
using namespace Ravelin;

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
  
  const double NEAR_ZERO = Pacer::NEAR_ZERO;
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
  R.mult(cf,F_contact,1.0/h,0);
  
  LA_.solve_chol_fast(iM_chol,F_contact);
  
  qdd += F_contact;
  OUT_LOG(logDEBUG) << "<< forward_dynamics() exited" << std::endl;
  return qdd;
}

void loop(){
boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  static double last_time = -0.001;
  double dt = t - last_time;
  last_time = t;
  
  OUT_LOG(logDEBUG) << "simulator_time = " << t;
  
  
  double dt_idyn = ctrl->get_data<double>(plugin_namespace+".dt");
  double alpha = ctrl->get_data<double>(plugin_namespace+".alpha");
  int USE_DES_CONTACT = ctrl->get_data<int>(plugin_namespace+".des-contact");
  int USE_LAST_CFS = ctrl->get_data<int>(plugin_namespace+".last-cfs");
  
  double DT = (dt_idyn == 0)? dt : dt_idyn;
  if(DT == 0) DT = 0.001;
  
  static std::vector<std::string>
  foot_names = ctrl->get_data<std::vector<std::string> >("init.end-effector.id");
  
  std::vector<std::string> active_feet;
  
  int num_feet = foot_names.size();
  

  if(USE_DES_CONTACT){
    for(int i=0;i<num_feet;i++){
      bool is_stance = false;
      if(ctrl->get_data<bool>(foot_names[i]+".stance",is_stance))
        if(is_stance)
          active_feet.push_back(foot_names[i]);
    }
  } else
    active_feet = foot_names;
  
  double mcpf = 1e2;
  ctrl->get_data<double>(plugin_namespace+".max-contacts-per-foot",mcpf);
  int MAX_CONTACTS_PER_FOOT = mcpf;
  
  // TODO: REMOVE
//  int MULTIPLIER = std::ceil(t*5);
  
//  static std::vector<int> num_added(num_feet);
  // TODO: REMOVE
//  num_added[(( (int) (t*5.0*4.0) ) % 4 )] = MULTIPLIER;
  
  std::vector<unsigned> indices;
  std::vector< boost::shared_ptr<Pacer::Robot::contact_t> > contacts;
  boost::shared_ptr<Pose3d> base_link_frame = boost::shared_ptr<Pose3d>( new Ravelin::Pose3d(ctrl->get_data<Ravelin::Pose3d>("base_link_frame")));

  
  for(int i=0;i<active_feet.size();i++){
    std::vector< boost::shared_ptr< const Pacer::Robot::contact_t> > c;
    ctrl->get_link_contacts(active_feet[i],c);
    Vector3d pos;
    if(!c.empty()){
      for(int j=0;j<c.size() && j<MAX_CONTACTS_PER_FOOT;j++){
        OUT_LOG(logDEBUG) << "compliant: " << c[j]->compliant;
        OUT_LOG(logDEBUG) << "normal: " << c[j]->normal;
        OUT_LOG(logDEBUG) << "tangent: " << c[j]->tangent;
        OUT_LOG(logDEBUG) << "point: " << c[j]->point;
        std::string id(c[j]->id);
        pos = c[j]->point;
        Ravelin::Vector3d cf(0,0,0);
        ctrl->get_data<Ravelin::Vector3d>(id+".contact-force",cf);
        // Normal force > tolerance
        if (cf[0] > 0.5) {
//        for (int k=0; k<num_added[i]; k++) {
          contacts.push_back(ctrl->create_contact(id,pos,c[j]->normal,c[j]->tangent,c[j]->impulse,c[j]->mu_coulomb,c[j]->mu_viscous,0,c[j]->compliant));
          indices.push_back((unsigned) i);
//        }
        }
      }
    }
  }
  
  int NC = contacts.size();
  
  // State
  
  Ravelin::VectorNd q,generalized_qd,qdd_des,generalized_fext;
  Ravelin::VectorNd outputv;
  
  ctrl->get_generalized_value(Pacer::Robot::position,q);
  OUTLOG(q,"generalized_q",logDEBUG);
  
  ctrl->get_generalized_value(Pacer::Robot::velocity,generalized_qd);
  OUTLOG(generalized_qd,"generalized_qd",logDEBUG);
  
  ctrl->get_generalized_value(Pacer::Robot::load,generalized_fext);
  OUTLOG(generalized_fext,"generalized_fext",logDEBUG);
  
  ctrl->get_generalized_value(Pacer::Robot::acceleration,outputv);
  OUTLOG(outputv,"generalized_qdd",logDEBUG);
  
  // OUTPUT
  ctrl->get_joint_generalized_value(Pacer::Robot::position_goal,outputv);
  OUTLOG(outputv,"q_des",logDEBUG);
  
  ctrl->get_joint_generalized_value(Pacer::Robot::velocity_goal,outputv);
  OUTLOG(outputv,"qd_des",logDEBUG);
  
  ctrl->get_joint_generalized_value(Pacer::Robot::acceleration_goal,qdd_des);
  OUTLOG(qdd_des,"qdd_des",logDEBUG);
  
  int NUM_JOINT_DOFS = q.size() - Pacer::NEULER;
  int NDOFS = NUM_JOINT_DOFS + Pacer::NSPATIAL;
  
  // Inertia
  static Ravelin::MatrixNd M;
  
  static Ravelin::VectorNd q_last = Ravelin::VectorNd::zero(q.size());
  // Consider increasing tolerance
  //    if((q_last-=q).norm() > Pacer::NEAR_ZERO)
  ctrl->calc_generalized_inertia(q, M);
  q_last = q;
  // Jacobian Calculation
  Ravelin::MatrixNd N,S,T,D;
  
  ctrl->calc_contact_jacobians(q,contacts,N,S,T);
  
  D.set_zero(NDOFS,NC*4);
  D.set_sub_mat(0,0,S);
  D.set_sub_mat(0,NC,T);
  S.negate();
  T.negate();
  D.set_sub_mat(0,NC*2,S);
  D.set_sub_mat(0,NC*3,T);
  
  // Index of foot check (continuity of contacts)
  static std::vector<unsigned> last_indices = indices;
  static int last_NC = NC;
  bool SAME_INDICES = false;
  if(NC == last_NC){
    SAME_INDICES = true;
    for (int i=0; i<indices.size(); i++) {
      if(last_indices[i] != indices[i]){
        SAME_INDICES = false;
        break;
      }
    }
  }
  last_NC = NC;
  last_indices = indices;
  
  bool inf_friction = false;
  Ravelin::MatrixNd MU;
  MU.set_zero(N.columns(),2);
  for(int i=0;i<MU.rows();i++){
      std::fill(MU.row(i).begin(),MU.row(i).end(),contacts[i]->mu_coulomb);
  }
  
  Ravelin::VectorNd cf_init = Ravelin::VectorNd::zero(NC*5);
  {
    for(unsigned i=0;i< contacts.size();i++){
      
      Ravelin::Vector3d tan1,tan2;
      Ravelin::Vector3d::determine_orthonormal_basis(contacts[i]->normal,tan1,tan2);
      Ravelin::Matrix3d R_foot( contacts[i]->normal[0], contacts[i]->normal[1], contacts[i]->normal[2],
                               tan1[0],                tan1[1],                tan1[2],
                               tan2[0],                tan2[1],                tan2[2]);
      
      Ravelin::Origin3d contact_impulse = Ravelin::Origin3d(R_foot.mult(contacts[i]->impulse,workv3_));
      OUT_LOG(logDEBUG) << "Contact " << i << " mu coulomb: " << contacts[i]->mu_coulomb;

      OUT_LOG(logDEBUG) << "compliant: " << contacts[i]->compliant;
      OUT_LOG(logDEBUG) << "point: " << contacts[i]->point;
      if( !contacts[i]->compliant ){
        contact_impulse/=dt;
        OUT_LOG(logDEBUG) << "Contact " << i << " is rigid: " << contact_impulse ;
      } else {
        OUT_LOG(logDEBUG) << "Contact " << i << " is compliant: " << contact_impulse ;
      }
      cf_init[i] = contact_impulse[0];
      if(contact_impulse[1] >= 0)
        cf_init[NC+i] = contact_impulse[1];
      else
        cf_init[NC+i+NC*2] = -contact_impulse[1];
      if(contact_impulse[2] >= 0)
        cf_init[NC+i+NC] = contact_impulse[2];
      else
        cf_init[NC+i+NC*3] = -contact_impulse[2];
    }
    Utility::check_finite(cf_init);
    
    //    OUTLOG(cf_init,"cf_0",logDEBUG);
    
    Ravelin::SharedConstVectorNd cf_normal = cf_init.segment(0,NC);
    double sum = std::accumulate(cf_normal.begin(),cf_normal.end(),0.0);
//    OUT_LOG(logDEBUG) << "0, Sum normal force: " << sum ;
    
    if(USE_LAST_CFS){
      static std::queue<Ravelin::VectorNd>
      cf_delay_queue;
      static std::queue<Ravelin::MatrixNd>
      N_delay_queue,
      D_delay_queue,
      MU_delay_queue;
      static int FILTER_CFS = ctrl->get_data<int>(plugin_namespace+".last-cfs-filter");
      
      if(FILTER_CFS && NC>0){
        
        cf_delay_queue.push(cf_init);
        N_delay_queue.push(N);
        D_delay_queue.push(D);
        MU_delay_queue.push(MU);
        cf_init = cf_delay_queue.front();
        N = N_delay_queue.front();
        D = D_delay_queue.front();
        MU = MU_delay_queue.front();
        if(cf_delay_queue.size() >= 2){
          cf_delay_queue.pop();
          N_delay_queue.pop();
          D_delay_queue.pop();
          MU_delay_queue.pop();
        }
      } else {
        cf_delay_queue = std::queue<Ravelin::VectorNd>();
        N_delay_queue = std::queue<Ravelin::MatrixNd>();
        D_delay_queue = std::queue<Ravelin::MatrixNd>();
        MU_delay_queue = std::queue<Ravelin::MatrixNd>();
      }
    }
  }
  
  OUTLOG(MU,"MU",logDEBUG);
  
  // IDYN MAXIMAL DISSIPATION MODEL
  std::vector<std::string> controller_name = ctrl->get_data<std::vector<std::string> >(plugin_namespace+".type");
  
  std::map<std::string,Ravelin::VectorNd> cf_map,uff_map;
  
  OUTLOG(NC,"idyn_NC",logDEBUG);
  
  ////////////////////////// simulator DT IDYN //////////////////////////////
  for (std::vector<std::string>::iterator it=controller_name.begin();
       it!=controller_name.end(); it++) {
    
    const double h_dt = DT/dt;
    bool solve_flag = false;
    
    Ravelin::VectorNd id = Ravelin::VectorNd::zero(NUM_JOINT_DOFS);
    Ravelin::VectorNd cf;// = Ravelin::VectorNd::zero(NC*5);
    
    std::string& name = (*it);
    
    OUT_LOG(logDEBUG) << "CONTROLLER: " << name;
    OUT_LOG(logDEBUG) << "USE_LAST_CFS: " << USE_LAST_CFS;
    //
    if (NC == 0) {
      solve_flag = inverse_dynamics_no_slip_fast(generalized_qd,qdd_des,M,N,D,generalized_fext,DT,id,cf,false,indices,active_feet.size(),SAME_INDICES);
    }
    else if(USE_LAST_CFS){
      cf = cf_init;
      cf *= dt;
      OUT_LOG(logDEBUG) << "USING LAST CFS: " << cf;
      //      solve_flag = inverse_dynamics(qdd_des,M,N,D,generalized_fext,DT,id,cf);
      solve_flag = inverse_dynamics_one_stage(generalized_qd,qdd_des,M,N,D,generalized_fext,DT,MU,id,cf,indices,active_feet.size(),SAME_INDICES);
      OUT_LOG(logDEBUG) << "SAME: " << cf;
    } else {
#ifdef TIMING
          struct timeval start_t;
          struct timeval end_t;
          gettimeofday(&start_t, NULL);
#endif
      try{
        if(name.compare("NSQP") == 0){
          solve_flag = inverse_dynamics_no_slip(generalized_qd,qdd_des,M,N,D,generalized_fext,DT,id,cf,indices,active_feet.size(),SAME_INDICES);
        } else if(name.compare("NSLCP") == 0){
          solve_flag = inverse_dynamics_no_slip_fast(generalized_qd,qdd_des,M,N,D,generalized_fext,DT,id,cf,false,indices,active_feet.size(),SAME_INDICES);
        } else if(name.compare("CFQP") == 0){    // IDYN QP
          solve_flag = inverse_dynamics_two_stage(generalized_qd,qdd_des,M,N,D,generalized_fext,DT,MU,id,cf,indices,active_feet.size(),SAME_INDICES);
        } else if(name.compare("CFQP1") == 0){    // IDYN QP
          solve_flag = inverse_dynamics_one_stage(generalized_qd,qdd_des,M,N,D,generalized_fext,DT,MU,id,cf,indices,active_feet.size(),SAME_INDICES);
        } else if(name.compare("CFLCP") == 0){
          solve_flag = inverse_dynamics_ap(generalized_qd,qdd_des,M,N,D,generalized_fext,DT,MU,id,cf);
        }  else if(name.compare("SCFQP") == 0){
          solve_flag = inverse_dynamics_two_stage_simple(generalized_qd,qdd_des,M,N,D,generalized_fext,DT,MU,id,cf);
        }
      } catch (std::exception e){
        solve_flag = false;
      }
  
    if(!std::isfinite(cf.norm()) || !std::isfinite(id.norm())){
      OUTLOG(DT,"DT",logDEBUG);
      OUTLOG(cf,"IDYN_CF",logDEBUG);
      OUTLOG(id,"IDYN_U",logDEBUG);
      
#ifndef TIMING
      throw std::runtime_error("IDYN forces are NaN or INF");
#else //NOT TIMING
      continue;
#endif
    }
#ifdef TIMING
    gettimeofday(&end_t, NULL);
    double duration = (end_t.tv_sec - start_t.tv_sec) + (end_t.tv_usec - start_t.tv_usec) * 1E-6;
      std::cout << ("timing_"+name+" ") << duration*1000.0 << " " << NC << " " << t << std::endl;
#endif
    }
    
    //// Check for Normal direction torque chatter
    if (NC > 0 && solve_flag){
      static std::map<std::string,Ravelin::VectorNd> last_cf;
      std::map<std::string,Ravelin::VectorNd>::iterator it = last_cf.find(name);
      if(it != last_cf.end()){
        if(last_cf[name].rows() == NC){
          Ravelin::VectorNd diff_cf  = (*it).second;
          diff_cf -= cf.segment(0,NC);
          if(diff_cf.norm() > 0.01)
            OUT_LOG(logDEBUG) << "-- Torque chatter detected " << t;
        }
      } else {
        last_cf[name] = cf.segment(0,NC);
      }
    }
    
    double normal_sum = 0;
    
    /*
#ifndef NDEBUG
    if (solve_flag) {
      for (int j=0;j<NC;j++){
        Ravelin::Vector3d impulse(cf[j],cf[j+NC]-cf[j+NC*3],cf[j+NC*2]-cf[j+NC*4]);
        normal_sum += cf[j];
        OUT_LOG(logDEBUG) << "IDYN CONTACT FORCE : " << name;
        OUT_LOG(logDEBUG) << "t: " << t << " " << contacts[j]->id;
        
        Ravelin::Vector3d tan2 = Ravelin::Vector3d::cross(contacts[j]->normal,contacts[j]->tangent);
        Ravelin::Matrix3d R_foot( contacts[j]->normal[0],  contacts[j]->normal[1], contacts[j]->normal[2],
                                 contacts[j]->tangent[0], contacts[j]->tangent[1],contacts[j]->tangent[2],
                                 tan2[0],                tan2[1],                tan2[2]);
        
        Ravelin::Vector3d contact_impulse = R_foot.transpose_mult(impulse,workv3_);
        if( !contacts[j]->compliant ){
          //        contact_impulse/=dt;
          OUT_LOG(logDEBUG) << "force: " << contact_impulse/DT ;
        } else {
          OUT_LOG(logDEBUG) << "force: " << contact_impulse/DT ;
        }
      }
    }
    OUT_LOG(logDEBUG) << name << ", Sum normal force: " << normal_sum/DT;

    log_idyn_matrices(generalized_qd,M,N,D,generalized_fext,DT);
    
    {
      Ravelin::MatrixNd workM1,workM2;
      Ravelin::VectorNd workv1, workv2;

      // Compute Jacobians
      int n = generalized_qd.rows();
      int nc = NC;
      int nk = (nc == 0)? 0 : D.columns()/nc;
      int nvars = nc + nc*(nk);
      // setup R
      Ravelin::MatrixNd R(n, nc + (nc*nk) );
      R.block(0,n,0,nc) = N;
      R.block(0,n,nc,nc*nk+nc) = D;
      
      // Invert M
      Ravelin::MatrixNd iM_chol = M;
      if(!LA_.factor_chol(iM_chol)){
        throw std::runtime_error("Chol Factorization Failed on M");
      }
      Ravelin::MatrixNd iM = Ravelin::MatrixNd::identity(n);
      LA_.solve_chol_fast(iM_chol,iM);
      
      Ravelin::VectorNd qdd_idyn = forward_dynamics_no_contact(iM_chol,generalized_fext,DT,id);
      qdd_idyn *= DT;
      Ravelin::VectorNd v_minus = qdd_idyn;
      v_minus += generalized_qd;
      OUTLOG(v_minus,"v-",logDEBUG);
      
      double ke_minus = M.mult(v_minus,workv1).dot(v_minus)*0.5;
      OUTLOG(ke_minus,"KE (pre-contact)",logDEBUG);
      
      OUT_LOG(logDEBUG) << "-- Checking optimization post contact velocity ";

      // Is N (v- + iM R' z1) >= 0 ?
      {
        Ravelin::VectorNd v_plus;
        iM.mult(R.mult(cf,workv1),v_plus = v_minus,1,1);
        double ke_plus = M.mult(v_plus,workv1).dot(v_plus)*0.5;
        OUTLOG(ke_plus,"KE (post-contact)",logDEBUG);
        
        OUTLOG(ke_plus-ke_minus,"KE difference",logDEBUG);
        
        OUTLOG(v_plus,"v+",logDEBUG);
        OUTLOG(N.transpose_mult(v_plus,workv1),"N*v+",logDEBUG);
      }
      qdd_idyn = forward_dynamics(M,N,D,generalized_fext,DT,id,cf).segment(0,n-6);
      OUTLOG(qdd_des,"IDYN_QDD_DES",logDEBUG);
      OUTLOG(qdd_idyn,"IDYN_QDD_RESULT",logDEBUG);
      qdd_idyn -= qdd_des;
      OUTLOG(qdd_idyn,"IDYN_QDD_ERR",logDEBUG);
      if (qdd_idyn.norm() > 1e-4 || !solve_flag) {
        OUT_LOG(logDEBUG) << "IDYN failed to follow qdd_des : " << name;
        std::cerr << "IDYN failed to follow qdd_des : " << name << std::endl;
        //        throw std::runtime_error("IDYN failed to follow qdd_des");
      }
      
      {
        OUT_LOG(logDEBUG) << "-- Attempting separate contact solve ";
        
        // Jacobian Calculation
        
        Ravelin::VectorNd s,t;
        S.transpose_mult(v_minus,s);
        T.transpose_mult(v_minus,t);
        
        for (int i=0; i<contacts.size(); i++) {
          Ravelin::Vector3d
            normal  = contacts[i]->normal,
            tan1 = contacts[i]->tangent,
            tan2 = Ravelin::Vector3d::cross(contacts[i]->normal,contacts[i]->tangent);
          tan2.normalize();

          contacts[i]->tangent = (tan1*s[i] + tan2*t[i]);
          contacts[i]->tangent.normalize();
          
          OUTLOG(contacts[i]->tangent,"tan1 (v-)",logDEBUG);
          OUTLOG(contacts[i]->normal,"normal (v-)",logDEBUG);


        }
        Ravelin::MatrixNd N,S,T,D;

        ctrl->calc_contact_jacobians(q,contacts,N,S,T);
        
        D.set_zero(NDOFS,NC*4);
        D.set_sub_mat(0,0,S);
        D.set_sub_mat(0,NC,T);
        S.negate();
        T.negate();
        D.set_sub_mat(0,NC*2,S);
        D.set_sub_mat(0,NC*3,T);
        
        Ravelin::VectorNd z;
        
        if(!predict_contact_forces(v_minus,M,N,D,MU,z)){
          throw std::runtime_error("Contact force prediction failed!");
        } else {
          Ravelin::VectorNd v_plus;
          iM.mult(R.mult(z,workv1),v_plus = v_minus,1,1);
          OUTLOG(v_plus,"v+ (2 step)",logDEBUG);
          OUTLOG(z,"z",logDEBUG);
          OUTLOG(cf,"cf",logDEBUG);
          double ke_plus = M.mult(v_plus,workv1).dot(v_plus)*0.5;
          OUTLOG(ke_plus,"KE (post-constraint)",logDEBUG);
          OUTLOG(ke_plus-ke_minus,"KE difference",logDEBUG);
        }
      }
    }
#endif
     */
    cf /= DT;
    cf_map[name] = cf;
    uff_map[name] = id;
  }
  
  OUTLOG(controller_name,"controller_name",logDEBUG);
  //std::vector<std::string>::iterator it=controller_name.begin();
  //for (;it!=controller_name.end(); it++) {
  //;it!=controller_name.end(); it++) {
  for (int i=0;i<controller_name.size();i++){
    std::string& name = controller_name[i];//(*it);
    Ravelin::VectorNd& cf = cf_map[name];
    OUTLOG(uff_map[name],"uff_"+boost::icl::to_string<double>::apply(i+1),logDEBUG);
    OUTLOG(cf,"cf_"+boost::icl::to_string<double>::apply(i+1),logDEBUG);
  }
  
  Ravelin::VectorNd uff = uff_map[controller_name.front()];
  uff*=alpha;
  
  Ravelin::VectorNd u = ctrl->get_joint_generalized_value(Pacer::Controller::load_goal);
  OUTLOG(uff,"idyn_U",logDEBUG);
  if(u.rows() == uff.rows()){
    u += uff;
    ctrl->set_joint_generalized_value(Pacer::Controller::load_goal,u);
    OUTLOG(u,"FINAL_U",logDEBUG);
  }
}


void setup(){
  
}