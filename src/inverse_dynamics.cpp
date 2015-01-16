/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/controller.h>
#include <Pacer/utilities.h>
#include <Opt/LP.h>
using namespace Pacer;

int N_SYSTEMS = 0;

using Ravelin::VectorNd;
using Ravelin::MatrixNd;

Ravelin::LinAlgd _LA;
Moby::LCP _lcp;

extern bool solve_qp_pos(const Ravelin::MatrixNd& Q, const Ravelin::VectorNd& c, const Ravelin::MatrixNd& A, const Ravelin::VectorNd& b, Ravelin::VectorNd& x);
extern bool solve_qp_pos(const Ravelin::MatrixNd& Q, const Ravelin::VectorNd& c, Ravelin::VectorNd& x);
extern bool solve_qp(const Ravelin::MatrixNd& Q, const Ravelin::VectorNd& c, const Ravelin::MatrixNd& A, const Ravelin::VectorNd& b, Ravelin::VectorNd& x);
extern bool solve_qp(const Ravelin::MatrixNd& Q, const Ravelin::VectorNd& c, const Ravelin::VectorNd& lb, const Ravelin::VectorNd& ub, const Ravelin::MatrixNd& A, const Ravelin::VectorNd& b, Ravelin::VectorNd& x);
extern bool solve_qp(const Ravelin::MatrixNd& Q, const Ravelin::VectorNd& c, const Ravelin::VectorNd& lb, const Ravelin::VectorNd& ub, const Ravelin::MatrixNd& A, const Ravelin::VectorNd& b, const Ravelin::MatrixNd& M, const Ravelin::VectorNd& q, Ravelin::VectorNd& x);

Ravelin::VectorNd STAGE1, STAGE2;

//bool Robot::inverse_dynamics(const Ravelin::VectorNd& qdd, const Ravelin::MatrixNd& M, const Ravelin::VectorNd& fext, Ravelin::VectorNd& x){
//  M.mult(qdd,x) -= fext;
//}

bool Controller::inverse_dynamics(const Ravelin::VectorNd& v, const Ravelin::VectorNd& qdd, const Ravelin::MatrixNd& M,const  Ravelin::MatrixNd& N,
                         const Ravelin::MatrixNd& ST, const Ravelin::VectorNd& fext_, double h, const Ravelin::MatrixNd& MU, Ravelin::VectorNd& x, Ravelin::VectorNd& cf_final){

  OUT_LOG(logDEBUG) << ">> inverse_dynamics() entered" << std::endl;

  Ravelin::VectorNd fext = fext_;
  // get number of degrees of freedom and number of contact points
  int n = M.rows();
  int nq = n - 6;
  int nc = N.columns();

  Ravelin::MatrixNd workM1,workM2;
  Ravelin::VectorNd workv1, workv2,fID;

  Ravelin::VectorNd vq(nq);
  v.get_sub_vec(0,nq,vq);

  Ravelin::VectorNd vb(6);
  v.get_sub_vec(nq,n,vb);

  Ravelin::VectorNd vqstar;
  ((vqstar = qdd) *= h) += vq;

  // Log these function variables

  // compute A, B, and C
  // | C B'| = M
  // | B A |
  Ravelin::MatrixNd C(nq,nq);
  M.get_sub_mat(0,nq,0,nq,C);

  Ravelin::MatrixNd B(6,nq);
  M.get_sub_mat(nq,n,0,nq,B);

  Ravelin::MatrixNd A(6,6);
  M.get_sub_mat(nq,n,nq,n,A);


  // compute D, E, and F
  Ravelin::MatrixNd iM_chol;
  iM_chol = M;
  assert(LA_.factor_chol(iM_chol));

  Ravelin::MatrixNd iM;
  iM = Ravelin::MatrixNd::identity(n);
  LA_.solve_chol_fast(iM_chol,iM);
//  LA_.solve_fast(M,iM);

  // | F E'|  =  inv(M)
  // | E D |
  Ravelin::MatrixNd D(6,6);
  iM.get_sub_mat(nq,n,nq,n,D);
  Ravelin::MatrixNd E(6,nq);
  iM.get_sub_mat(nq,n,0,nq,E);
  Ravelin::MatrixNd F(nq,nq);
  iM.get_sub_mat(0,nq,0,nq,F);
  Ravelin::MatrixNd iF;
  iF = F;
  assert(LA_.factor_chol(iF));

  // if in mid-air only return ID forces solution
  // fID + fext = M qdd  ==> fID = M qdd - fext
  C.mult((workv1 = qdd),fID) -= fext.get_sub_vec(0,nq,workv2);

  if(nc == 0){
    // Inverse dynamics for a floating base w/ no contact
//    goal accel
    x = fID;
    return true;
  }

  int nk = ST.columns()/nc;
  int nvars = nc + nc*(nk);
  // setup R
  Ravelin::MatrixNd R(n, nc + (nc*nk) );
  R.block(0,n,0,nc) = N;
  R.block(0,n,nc,nc*nk+nc) = ST;

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

  // Predict Contact forces
  // Incorporate fID into acting forces on robot, then find contact forces
  workv1.set_zero(n);
  workv1.set_sub_vec(0,fID);
  fext += workv1;

  /// Stage 1 optimization energy minimization
  Ravelin::VectorNd z(nvars),cf(nvars);

  // j and k

  // j = [E,D]*fext*h + vb
  Ravelin::VectorNd j;
  // fext + [0;fID]
  ED.mult(fext,(j = vb),h,1);

//  OUTLOG(j,"j = [ %= [E,D]*fext*h + vb");

  // k = [F,E']*fext*h  +  vq
  Ravelin::VectorNd k;
  FET.mult(fext,(k = vq),h,1);
//  OUTLOG(k,"k = [ % = [F,E']*fext*h  +  vq");

  // Use Sensed contact forces
  if(cf_final.rows() != 0){
    Ravelin::VectorNd fext_cf;
    // conventional way of incorporating contact forces into ID problem
    R.mult(cf_final,fext_cf = fext_,1.0/h,1);
    C.mult((workv1 = qdd) -= F.mult(fext_cf.get_sub_vec(0,nq,workv_),workv2),x);
    workv1 = x;

    // This produces a very good sensed contact controller
    (x = vqstar) -= k;
    FET.mult(R,workM1);
    workM1.mult(cf_final,x,-1,1);
    LA_.solve_chol_fast(iF,x);
    x /= h;

    OUTLOG(fID,"f_ID",logDEBUG1);
    OUTLOG(workv1,"x_standard",logDEBUG1);
    OUTLOG(x,"x_eqn35",logDEBUG1);
    OUTLOG(workv1 -= x,"x_diff",logDEBUG1);

    return true;
  }

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
  workM1.set_zero(n,Z.columns());
  workM1.set_sub_mat(nq,0,Z);
  // constraint Jacobain 1:
  // qM1 = N'[zeros(nq,:) ; Z]
  Ravelin::MatrixNd qM1(N.columns(),Z.columns());
  N.transpose_mult(workM1,qM1);
  OUTLOG(qM1,"M_IP",logDEBUG1);
  // [vq* ; p]
  Ravelin::VectorNd vqstar_p(n);
  vqstar_p.set_sub_vec(0,vqstar);
  vqstar_p.set_sub_vec(nq,p);

  // constraint vector 1
  // qq1 = -N'[vq* ; p]
  Ravelin::VectorNd qq1(N.columns());
  N.transpose_mult(vqstar_p,qq1);
  qq1.negate();
  OUTLOG(qq1,"q_IP",logDEBUG1);

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

  OUTLOG(z,"Z_OP1",logDEBUG1);
  // measure feasibility of solution
  // qM z - qq >= 0
  Ravelin::VectorNd feas;
  qM.mult(z,feas) -= qq;

  OUTLOG(feas,"feas_OP1 =[ % (A*z-b >= 0)",logDEBUG1);

  // push z into output vector
  cf = z;

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

    // optimize system
    Ravelin::VectorNd w(size_null_space);
    if(!solve_qp(qG,qc,qM,qq,w)){
      OUT_LOG(logERROR)  << "ERROR: Unable to solve stage 2!";
      return false;
      // then skip to calculating x from stage 1 solution
    } else {
      OUTLOG(w,"W_OP2",logDEBUG1);

      // measure feasibility of solution
      // qM w - qq >= 0
      feas = qq;
      qM.mult(w,feas,1,-1);

      OUTLOG(feas,"feas_OP2 =[ % (A*w - b >= 0)",logDEBUG1);

      // return the solution (contact forces)
      // cf = z + P*w;

      P.mult(w,cf,1,1);

      OUTLOG(cf,"z_OP2 =[ % (P*w + z)",logDEBUG1);

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
  OUT_LOG(logDEBUG) << "<< inverse_dynamics() exited" << std::endl;
  return true;
}

bool Controller::inverse_dynamics_no_slip(const Ravelin::VectorNd& v, const Ravelin::VectorNd& qdd, const Ravelin::MatrixNd& M,const  Ravelin::MatrixNd& N,
                         const Ravelin::MatrixNd& ST, const Ravelin::VectorNd& fext_, double h, Ravelin::VectorNd& x, Ravelin::VectorNd& cf_final){
  OUT_LOG(logDEBUG) << ">> inverse_dynamics_no_slip() entered" << std::endl;

  Ravelin::VectorNd fext = fext_;
  // get number of degrees of freedom and number of contact points
  int n = M.rows();
  int nq = n - 6;
  int nc = N.columns();

  Ravelin::MatrixNd workM1,workM2;
  Ravelin::VectorNd workv1, workv2,fID;

  Ravelin::VectorNd vq(nq);
  v.get_sub_vec(0,nq,vq);

  Ravelin::VectorNd vb(6);
  v.get_sub_vec(nq,n,vb);

  Ravelin::VectorNd vqstar;
  ((vqstar = qdd) *= h) += vq;

  // Log these function variables

  // compute A, B, and C
  // | C B'| = M
  // | B A |
  Ravelin::MatrixNd C(nq,nq);
  M.get_sub_mat(0,nq,0,nq,C);

  Ravelin::MatrixNd B(6,nq);
  M.get_sub_mat(nq,n,0,nq,B);

  Ravelin::MatrixNd A(6,6);
  M.get_sub_mat(nq,n,nq,n,A);


  // compute D, E, and F
  Ravelin::MatrixNd iM_chol;
  iM_chol = M;
  assert(LA_.factor_chol(iM_chol));

  Ravelin::MatrixNd iM;
  iM = Ravelin::MatrixNd::identity(n);
  LA_.solve_chol_fast(iM_chol,iM);
//  LA_.solve_fast(M,iM);

  // | F E'|  =  inv(M)
  // | E D |
  Ravelin::MatrixNd D(6,6);
  iM.get_sub_mat(nq,n,nq,n,D);
  Ravelin::MatrixNd E(6,nq);
  iM.get_sub_mat(nq,n,0,nq,E);
  Ravelin::MatrixNd F(nq,nq);
  iM.get_sub_mat(0,nq,0,nq,F);
  Ravelin::MatrixNd iF;
  iF = F;
  assert(LA_.factor_chol(iF));

#ifndef NDEBUG
  OUTLOG(N,"N",logDEBUG);
//  OUTLOG(F,"",logDEBUG);
#endif

  // if in mid-air only return ID forces solution
  // fID + fext = M qdd  ==> fID = M qdd - fext
  C.mult((workv1 = qdd),fID) -= fext.get_sub_vec(0,nq,workv2);

  if(nc == 0){
    // Inverse dynamics for a floating base w/ no contact
//    goal accel
    x = fID;
    return true;
  }

  int nk = ST.columns()/nc;
  int nvars = nc + nc*(nk);
  // setup R
  Ravelin::MatrixNd R(n, nc + (nc*nk) );
  R.block(0,n,0,nc) = N;
  R.block(0,n,nc,nc*nk+nc) = ST;

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

  // Predict Contact forces
  // Incorporate fID into acting forces on robot, then find contact forces
  workv1.set_zero(n);
  workv1.set_sub_vec(0,fID);
  fext += workv1;

  /// Stage 1 optimization energy minimization
  Ravelin::VectorNd z(nvars),cf(nvars);

  // j and k

  // j = [E,D]*fext*h + vb
  Ravelin::VectorNd j;
  // fext + [0;fID]
  ED.mult(fext,(j = vb),h,1);

//  OUTLOG(j,"j = [ %= [E,D]*fext*h + vb");

  // k = [F,E']*fext*h  +  vq
  Ravelin::VectorNd k;
  FET.mult(fext,(k = vq),h,1);
//  OUTLOG(k,"k = [ % = [F,E']*fext*h  +  vq");

  // Use Sensed contact forces
  if(cf_final.rows() != 0){
    Ravelin::VectorNd fext_cf;
    // conventional way of incorporating contact forces into ID problem
    R.mult(cf_final,fext_cf = fext_,1.0/h,1);
    C.mult((workv1 = qdd) -= F.mult(fext_cf.get_sub_vec(0,nq,workv_),workv2),x);
    workv1 = x;

    // This produces a very good sensed contact controller
    (x = vqstar) -= k;
    FET.mult(R,workM1);
    workM1.mult(cf_final,x,-1,1);
    LA_.solve_chol_fast(iF,x);
    x /= h;

    OUTLOG(fID,"f_ID",logDEBUG1);
    OUTLOG(workv1,"x_standard",logDEBUG1);
    OUTLOG(x,"x_eqn35",logDEBUG1);
    OUTLOG(workv1 -= x,"x_diff",logDEBUG1);

    return true;
  }

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

  Ravelin::MatrixNd qM(qM1.rows(),qM1.columns());
  Ravelin::VectorNd qq(qq1.rows());
  qM.set_sub_mat(0,0,qM1);
  qq.set_sub_vec(0,qq1);

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

    // Set up constarint matrix
    // SRZ: constraint 2 (interpenetration) is not here
    qM.set_zero(qM1.rows(),qM1.columns());
    qq.set_zero(qM1.rows());
    qM.set_sub_mat(0,0,qM1);
    qq.set_sub_vec(0,qq1);

    // optimize system
    Ravelin::VectorNd w(size_null_space);
    if(!solve_qp(qG,qc,qM,qq,w)){
      OUT_LOG(logERROR)  << "ERROR: Unable to solve stage 2!";
      // calculate x from stage 1 solution
      cf_final = cf;
      (x = vqstar) -= k;
      FET.mult(R,workM1);
      workM1.mult(cf_final,x,-1,1);
      LA_.solve_chol_fast(iF,x);
      x /= h;
      return true;
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

    }
  } else {
    cf_final = cf;
    (x = vqstar) -= k;
    FET.mult(R,workM1);
    workM1.mult(cf_final,x,-1,1);
    LA_.solve_chol_fast(iF,x);
    x /= h;
  }


  //  OUTLOG(cf,"final_contact_force");
  //  Note compare contact force prediction to Moby contact force

  cf_final = cf;
  // return the inverse dynamics forces
  // x = iF*(vqstar - k - FET*R*(cf))/h
  (x = vqstar) -= k;
  FET.mult(R,workM1);
  workM1.mult(cf_final,x,-1,1);
  LA_.solve_chol_fast(iF,x);
  x /= h;

  // Some debugging dialogue
  OUT_LOG(logDEBUG) << "<< inverse_dynamics_no_slip() exited" << std::endl;
  return true;
}

extern bool lcp_fast(const MatrixNd& M, const VectorNd& q, const std::vector<unsigned>& indices, VectorNd& z, double zero_tol);

bool Controller::inverse_dynamics_no_slip_fast(const Ravelin::VectorNd& vel, const Ravelin::VectorNd& qdd, const Ravelin::MatrixNd& M,const  Ravelin::MatrixNd& nT,
                         const Ravelin::MatrixNd& D, const Ravelin::VectorNd& fext, double dt, Ravelin::VectorNd& x, Ravelin::VectorNd& cf, bool frictionless){
  Ravelin::MatrixNd _workM, _workM2;
  Ravelin::VectorNd _workv, _workv2;

  const double CHECK_ZERO = sqrt(Moby::NEAR_ZERO);
  Ravelin::MatrixNd NT = nT;
  OUT_LOG(logDEBUG) << ">> inverse_dynamics_no_slip_fast() entered" << std::endl;

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

  OUTLOG(NT,"N'",logDEBUG1);
  OUTLOG(ST,"S'",logDEBUG1);
  OUTLOG(TT,"T'",logDEBUG1);

  // compute D, E, and F
  Ravelin::MatrixNd iM_chol = M;
  assert(LA_.factor_chol(iM_chol));

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
  assert(LA_.factor_chol(iF));

  // compute j and k
  // [F,E']
  Ravelin::MatrixNd FET(F.rows(),F.columns()+E.rows()),
      ET = E;
  ET.transpose();
  FET.set_sub_mat(0,0,F);
  FET.set_sub_mat(0,F.columns(),ET);

  Ravelin::VectorNd vq(nq);
  vel.get_sub_vec(0,nq,vq);

  Ravelin::VectorNd v = vel;
  iM.mult(fext,v,dt,1);

  Ravelin::VectorNd vqstar;
//  ((vqstar = qdd) *= dt) += vq;
  ((vqstar = qdd) *= dt) += v.segment(0,nq);
  //////////////////////////////////////////////////////////////
  Ravelin::MatrixNd P;
  P.set_zero(nq,n);
  P.set_sub_mat(0,0,Ravelin::MatrixNd::identity(nq));
  Ravelin::MatrixNd PT = P;
  PT.transpose();

  OUTLOG(PT,"P'",logDEBUG1);

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


  /////////////////////////////////////////////////////////////////////////////
  /// Solve System
  ///
  std::vector<unsigned> S_indices,T_indices,J_indices;
  const unsigned N_IDX = 0;

  // we do this by solving the MLCP:
  // |  A  C  | | x | + | g | = | 0 |
  // |  D  B  | | y |   | h |   | 0 |

  // g = [-M*v'; 0; 0; -vq_err]
  // h = [ 0 ];
  // A = [ M -S' -T' -P';
  //       S  0   0   0 ;
  //       T  0   0   0 ;
  //       P  0   0   0 ];

  // B = [ 0 ];

  // C = [ -N';
  //        0 ;
  //        0 ];
  // D = -C'
  // x = [ v^+; cs; ct; a ]
  // y = [ cN]

  // Assuming that C is of full row rank (no dependent joint constraints)
  // A is invertible; then we just need to solve the LCP:

  // | B - D*inv(A)*C | | v | + | h - D*inv(A)*g | = | w |
  // and use the result to solve for u:
  // u = -inv(A)*(g + Cv)

  // A is the matrix | M X'|
  //                 | X 0 |  where X is [ S; T; P ]
  // blockwise inversion yields
  // inv(A) = [
  //    inv(M)-inv(M)*X'*Y*X*inv(M)   inv(M)*X'*Y ;
  //    Y*X*inv(M)                    -Y          ]
  // where Y = inv(X*inv(M)*X')

  // defining Q = C and using the result above yields following LCP:
  // matrix: Q*inv(A)*Q' = Q*inv(M)*Q' - Q*inv(M)*X'*Y*X*inv(M)*Q'
  // vector: Q*inv(A)*a  = -(-Q*v + Q*inv(M)*X'*Y*X*v - Q*inv(M)*X'*Y*[0;0;vq*])
  //                     = Q*v - Q*inv(M)*X'*Y*X*v + Q*inv(M)*X'*Y*[0;0;vq*])

  Ravelin::MatrixNd _MM,_Y;
  // ********************************************************
  // find largest non-singular set of J, S, and T indices
  // ********************************************************

  // loop through joint constraints, forming J*inv(M)*J' and checking condition
  for (unsigned i=0; i< nq; i++)
  {
    // add the index tentatively to the set
    J_indices.push_back(i);

    // This is an [I,0] matrix, it has full row rank
    continue;
    // select the rows and columns
    Jx_iM_JxT.select_square(J_indices.begin(), J_indices.end(), _Y);

    // skew the matrix away from positive definiteness
    for (unsigned j=0; j< J_indices.size(); j++)
      _Y(j,j) -= CHECK_ZERO;

    // attempt Cholesky factorization
    if (!_LA.factor_chol(_Y))
      J_indices.pop_back();
  }

  Ravelin::MatrixNd _rJx_iM_JxT;
  // get the reduced Jx*iM*Jx' matrix
  Jx_iM_JxT.select_square(J_indices.begin(), J_indices.end(), _rJx_iM_JxT);

  // loop through contacts, forming matrix below and checking its condition
  // | S*inv(M)*S'  S*inv(M)*T' S*inv(M)*J' |
  // | T*inv(M)*S'  T*inv(M)*T' T*inv(M)*J' |
  // | J*inv(M)*S'  J*inv(M)*T' J*inv(M)*J' |
  bool last_success = false;
  for (unsigned i=0; i< nc; i++)
  {
    // update S indices
    S_indices.push_back(i);

    // setup indices
    unsigned S_IDX = 0;
    unsigned T_IDX = S_indices.size();
    unsigned J_IDX = T_IDX + T_indices.size();
    _Y.resize(J_IDX + J_indices.size(), J_IDX + J_indices.size());

    // add S/S, T/T, J/J components to 'check' matrix
    Cs_iM_CsT.select_square(S_indices.begin(), S_indices.end(), _workM);
    _Y.set_sub_mat(S_IDX, S_IDX, _workM);
    Ct_iM_CtT.select_square(T_indices.begin(), T_indices.end(), _workM);
    _Y.set_sub_mat(T_IDX, T_IDX, _workM);
    _Y.set_sub_mat(J_IDX, J_IDX, _rJx_iM_JxT);

    // add S/T components to 'check' matrix
    Cs_iM_CtT.select(S_indices.begin(), S_indices.end(), T_indices.begin(), T_indices.end(), _workM);
    _Y.set_sub_mat(S_IDX, T_IDX, _workM);
    _Y.set_sub_mat(T_IDX, S_IDX, _workM, Ravelin::eTranspose);

    // add S/J components to check matrix
    Cs_iM_JxT.select(S_indices.begin(), S_indices.end(), J_indices.begin(), J_indices.end(), _workM);
    _Y.set_sub_mat(S_IDX, J_IDX, _workM);
    _Y.set_sub_mat(J_IDX, S_IDX, _workM, Ravelin::eTranspose);

    // add T/J components to check matrix
    Ct_iM_JxT.select(T_indices.begin(), T_indices.end(), J_indices.begin(), J_indices.end(), _workM);
    _Y.set_sub_mat(T_IDX, J_IDX, _workM);
    _Y.set_sub_mat(J_IDX, T_IDX, _workM, Ravelin::eTranspose);

    // skew the matrix away from positive definiteness
    for (unsigned j=0; j< _Y.rows(); j++)
      _Y(j,j) -= CHECK_ZERO;

    // see whether check matrix can be Cholesky factorized
    if (!_LA.factor_chol(_Y) || frictionless)
      S_indices.pop_back();

    // add index for T
    T_indices.push_back(i);

    // resize the check matrix
    T_IDX = S_indices.size();
    J_IDX = T_IDX + T_indices.size();
    _Y.resize(J_IDX + J_indices.size(), J_IDX + J_indices.size());

    // add S/S, T/T, J/J components to 'check' matrix
    Cs_iM_CsT.select_square(S_indices.begin(), S_indices.end(), _workM);
    _Y.set_sub_mat(S_IDX, S_IDX, _workM);
    Ct_iM_CtT.select_square(T_indices.begin(), T_indices.end(), _workM);
    _Y.set_sub_mat(T_IDX, T_IDX, _workM);
    _Y.set_sub_mat(J_IDX, J_IDX, _rJx_iM_JxT);

    // add S/T components to 'check' matrix
    Cs_iM_CtT.select(S_indices.begin(), S_indices.end(), T_indices.begin(), T_indices.end(), _workM);
    _Y.set_sub_mat(S_IDX, T_IDX, _workM);
    _Y.set_sub_mat(T_IDX, S_IDX, _workM, Ravelin::eTranspose);

    // add S/J components to check matrix
    Cs_iM_JxT.select(S_indices.begin(), S_indices.end(), J_indices.begin(), J_indices.end(), _workM);
    _Y.set_sub_mat(S_IDX, J_IDX, _workM);
    _Y.set_sub_mat(J_IDX, S_IDX, _workM, Ravelin::eTranspose);

    // add T/J components to check matrix
    Ct_iM_JxT.select(T_indices.begin(), T_indices.end(), J_indices.begin(), J_indices.end(), _workM);
    _Y.set_sub_mat(T_IDX, J_IDX, _workM);
    _Y.set_sub_mat(J_IDX, T_IDX, _workM, Ravelin::eTranspose);

    // skew the matrix away from positive definiteness
    for (unsigned j=0; j< _Y.rows(); j++)
      _Y(j,j) -= CHECK_ZERO;

    // see whether check matrix can be Cholesky factorized
    last_success = _LA.factor_chol(_Y);
    if (!last_success && !frictionless)
      T_indices.pop_back();
  }

  last_success = false;
  // output indices
  if (true)
  {
    std::ostringstream oss;
    oss << "s indices:";
    for (unsigned i=0; i< S_indices.size(); i++)
      oss << " " << S_indices[i];
    oss << "  t indices:";
    for (unsigned i=0; i< T_indices.size(); i++)
      oss << " " << T_indices[i];
    oss << "  j indices:";
    for (unsigned i=0; i< J_indices.size(); i++)
      oss << " " << J_indices[i];

    OUT_LOG(logDEBUG) << oss.str() << std::endl;
  }

  // ********************************************************
  // reform Y if necessary
  // ********************************************************

  // setup indices
  const unsigned S_IDX = 0;
  const unsigned T_IDX = S_indices.size();
  const unsigned J_IDX = T_IDX + T_indices.size();
  if (!last_success)
  {
    _Y.resize(J_IDX + J_indices.size(), J_IDX + J_indices.size());

    // add S/S, T/T, J/J components to X
    Cs_iM_CsT.select_square(S_indices.begin(), S_indices.end(), _workM);
    _Y.set_sub_mat(S_IDX, S_IDX, _workM);
    Ct_iM_CtT.select_square(T_indices.begin(), T_indices.end(), _workM);
    _Y.set_sub_mat(T_IDX, T_IDX, _workM);
    _Y.set_sub_mat(J_IDX, J_IDX, _rJx_iM_JxT);

    // add S/T components to X
    Cs_iM_CtT.select(S_indices.begin(), S_indices.end(), T_indices.begin(), T_indices.end(), _workM);
    _Y.set_sub_mat(S_IDX, T_IDX, _workM);
    _Y.set_sub_mat(T_IDX, S_IDX, _workM, Ravelin::eTranspose);

    // add S/J components to X
    Cs_iM_JxT.select(S_indices.begin(), S_indices.end(), J_indices.begin(), J_indices.end(), _workM);
    _Y.set_sub_mat(S_IDX, J_IDX, _workM);
    _Y.set_sub_mat(J_IDX, S_IDX, _workM, Ravelin::eTranspose);

    // add T/J components to X
    Ct_iM_JxT.select(T_indices.begin(), T_indices.end(), J_indices.begin(), J_indices.end(), _workM);
    _Y.set_sub_mat(T_IDX, J_IDX, _workM);
    _Y.set_sub_mat(J_IDX, T_IDX, _workM, Ravelin::eTranspose);

    // do the Cholesky factorization (should not fail)
    bool success = _LA.factor_chol(_Y);
    assert(success);
  }

  Ravelin::MatrixNd _Q_iM_XT;
  Ravelin::VectorNd _qq;

  // inv(A) = [
  //    inv(M)-inv(M)*X'*Y*X*inv(M)   inv(M)*X'*Y ;
  //    Y*X*inv(M)                    -Y          ]
  // defining Y = inv(X*inv(M)*X') and Q = [Cn]
  // matrix:-D*inv(A)*C = Q*inv(M)*Q' - Q*inv(M)*X'*Y*X*inv(M)*Q'
  //
  // vector:-Q*inv(A)*a  = -(-Q*inv(M)*M*v - -Q*inv(M)*X'*Y*X*inv(M)*M*v + -Q*inv(M)*X'*Y*[0;0;vq*])
  //                     = -(-Q*v + Q*inv(M)*X'*Y*X*v - Q*inv(M)*X'*Y*[0;0;vq*])
  //                     =    Q*v - Q*inv(M)*X'*Y*X*v + Q*inv(M)*X'*Y*[0;0;vq*]

  // setup Q*inv(M)*Q'
//  _MM = Cn_iM_CnT;

  OUTLOG(Cn_iM_CnT,"A",logDEBUG1);

  // setup Q*inv(M)*X'
  _Q_iM_XT.resize(nc, S_indices.size() + T_indices.size() + J_indices.size());
  Cn_iM_CsT.select_columns(S_indices.begin(), S_indices.end(), _workM);
  _Q_iM_XT.set_sub_mat(N_IDX, S_IDX, _workM);
  Cn_iM_CtT.select_columns(T_indices.begin(), T_indices.end(), _workM);
  _Q_iM_XT.set_sub_mat(N_IDX, T_IDX, _workM);
  Cn_iM_JxT.select_columns(J_indices.begin(), J_indices.end(), _workM);
  _Q_iM_XT.set_sub_mat(N_IDX, J_IDX, _workM);

  OUTLOG(_Q_iM_XT,"B",logDEBUG1);

//  Ravelin::MatrixNd Y;
//  Y = Ravelin::MatrixNd::identity(_Y.rows());
//  _LA.solve_chol_fast(_Y, Y);

  // compute Y*X*inv(M)*Q'
  Ravelin::MatrixNd::transpose(_Q_iM_XT, _workM);
  _LA.solve_chol_fast(_Y, _workM);
//  Y.mult_transpose(_Q_iM_XT,_workM);

  // compute Q*inv(M)*X'*Y*X*inv(M)*Q'
  _Q_iM_XT.mult(_workM, _workM2);
  (_MM = Cn_iM_CnT) -= _workM2;
  OUTLOG(_MM,"MM",logDEBUG1);

  // setup -Q*v
  _qq = Cn_v;

  // setup X*v
  Ravelin::VectorNd _Xv,_YXv;
  _Xv.resize(S_indices.size() + T_indices.size() + J_indices.size());
  Cs_v.select(S_indices.begin(), S_indices.end(), _workv);
  _Xv.set_sub_vec(S_IDX, _workv);
  Ct_v.select(T_indices.begin(), T_indices.end(), _workv);
  _Xv.set_sub_vec(T_IDX, _workv);
  Jx_v.select(J_indices.begin(), J_indices.end(), _workv);
  _Xv.set_sub_vec(J_IDX, _workv);

  // compute Y*X*v
  _YXv = _Xv;
  _LA.solve_chol_fast(_Y, _YXv);

  // compute Q*inv(M)*X' * Y*X*v
  _Q_iM_XT.mult(_YXv, _workv);

  // Add to LCP Vector
  _qq -= _workv;

  // compute Q*inv(M)*X'*Y*[0;0;vq*]
  Ravelin::VectorNd _Yvqstar;
  _Yvqstar.set_zero(S_indices.size() + T_indices.size() + J_indices.size());

  vqstar.select(J_indices.begin(), J_indices.end(), _workv);
  _Yvqstar.set_sub_vec(J_IDX, _workv);
  _LA.solve_chol_fast(_Y, _Yvqstar);
  _Q_iM_XT.mult(_Yvqstar,_workv);
  _qq += _workv;

  OUTLOG(_qq,"qq",logDEBUG1);
  // setup remainder of LCP vector

  // Setup Indices vector
  std::vector<unsigned> indices;
  unsigned active_eefs = 0;
  for(int i=0;i<eefs_.size();i++){
    if(!eefs_[i].active){
      continue;
    }
    active_eefs++;
    for(int j=0;j<eefs_[i].point.size();j++)
      indices.push_back(i);
  }

  static Ravelin::VectorNd _v;
  if(_v.size() != _qq.size())
    _v.resize(0);


  // attempt to solve the LCP using the fast method
  if(active_eefs > 2){
    OUT_LOG(logERROR) << "-- using: lcp_fast" << std::endl;

    if (!lcp_fast(_MM, _qq,indices, _v,Moby::NEAR_ZERO))
    {
      OUT_LOG(logERROR) << "-- Principal pivoting method LCP solver failed; falling back to regularized lemke solver" << std::endl;

      if (!_lcp.lcp_lemke_regularized(_MM, _qq, _v,-20,4,1))
        throw std::runtime_error("Unable to solve constraint LCP!");
    }
  } else {
    OUT_LOG(logERROR) << "-- using: Moby::LCP::lcp_fast" << std::endl;

    if (!_lcp.lcp_fast(_MM, _qq, _v))
    {
      OUT_LOG(logERROR) << "Principal pivoting method LCP solver failed; falling back to regularized lemke solver" << std::endl;

      if (!_lcp.lcp_lemke_regularized(_MM, _qq, _v,-20,4,1))
        throw std::runtime_error("Unable to solve constraint LCP!");
    }
  }
  OUTLOG(_v,"v",logDEBUG1);

  Ravelin::VectorNd _cs_ct_tau;

  // compute the friction forces
  // u = -inv(A)*(a + Cv)
  // u = inv(A)*(Q'*[cn])  [b/c we don't care about new velocity]
  // recalling that inv(A) =
  // | inv(M)-inv(M)*X'*Y*X*inv(M)   inv(M)*X'*Y | ngc x ngc,    ngc x sz(x)
  // | Y*X*inv(M)                    -Y          | sz(x) x ngc,  sz(x) x sz(x)
  // Q is nlcp x (ngc + sz(x))
  // [cs; ct] = -Y*X*v - Y*X*inv(M)*Q'*[cn]
  _cs_ct_tau = _YXv;
  _Q_iM_XT.transpose_mult(_v, _workv);
  _LA.solve_chol_fast(_Y, _workv);
  _cs_ct_tau += _workv;
  _cs_ct_tau.negate();

  Ravelin::VectorNd cn,cs,ct,tau;
  // setup impulses
  cn = _v.segment(0, nc);
  cs.set_zero(nc);
  ct.set_zero(nc);
  tau.set_zero(nq);
  Ravelin::SharedConstVectorNd cs_vec = _cs_ct_tau.segment(S_IDX, T_IDX);
  Ravelin::SharedConstVectorNd ct_vec = _cs_ct_tau.segment(T_IDX, J_IDX);
  Ravelin::SharedConstVectorNd a_vec = _cs_ct_tau.segment(J_IDX, _cs_ct_tau.rows());
  cs.set(S_indices.begin(), S_indices.end(), cs_vec);
  ct.set(T_indices.begin(), T_indices.end(), ct_vec);
  tau.set(J_indices.begin(), J_indices.end(), a_vec);

  OUTLOG(cn,"cN",logDEBUG);
  OUTLOG(cs,"cS",logDEBUG);
  OUTLOG(ct,"cT",logDEBUG);

  cf.set_zero(nc*5);
  for(unsigned i=0;i< nc;i++){
    cf[i] = cn[i];
    if(ct[i] >= 0)
      cf[nc+i] = cs[i];
    else
      cf[nc+i+nc*2] = -cs[i];
    if(ct[i] >= 0)
      cf[nc+i+nc] = ct[i];
    else
      cf[nc+i+nc*3] = -ct[i];
  }

//   IDYN MLCP
//            A             C      x         g
//       | M −S' -T' -P'   -N' | | v+ |   |-M v |    | 0 |
//       | S  0   0   0     0  | | cS |   |  0  |  = | 0 |
//       | T  0   0   0     0  | | cT |   |  0  |    | 0 |
//       | P  0   0   0     0  | | a  |   |-vq* |    | 0 |
//                              *       +
//       | N  0   0   0     0  | | cN |   |  0  | >= | 0 |
//            D            B      y         h

//   Constraints
//   [M −S' -T' -N' -P'  P' ].[ ]' - M*v  = 0
//    S v+        = 0
//    T v+        = 0
//    P v+ - vq*  = 0
//    N v+       >= 0

//    M v+ - (S'cs + T'cT + N'cN) - a = M v
//    M(v+ - v) = (S'cs + T'cT + N'cN)  +  a

//   Therefore
//    M(v+ - v)           = f_total
//    S'cs + T'cT + N'cN  = contact_force
//    f = cf + a

  // Using M(dv) - z = tau
  x = tau;
  if(nc>0)
    x -= R.mult(cf,_workv).segment(0,nq);
  x /= dt;

  OUTLOG(x,"tau",logDEBUG);

  OUT_LOG(logDEBUG) << "<< inverse_dynamics_no_slip_fast() exited" << std::endl;
  return true;
}

bool Controller::inverse_dynamics_ap(const Ravelin::VectorNd& vel, const Ravelin::VectorNd& qdd, const Ravelin::MatrixNd& M,const  Ravelin::MatrixNd& NT,
                         const Ravelin::MatrixNd& D_, const Ravelin::VectorNd& fext, double dt, const Ravelin::MatrixNd& MU, Ravelin::VectorNd& x, Ravelin::VectorNd& cf){
  Ravelin::MatrixNd _workM, _workM2;
  Ravelin::VectorNd _workv, _workv2;

  const double CHECK_ZERO = Moby::NEAR_ZERO;
  OUT_LOG(logDEBUG) << ">> inverse_dynamics_ap() entered" << std::endl;

  // get number of degrees of freedom and number of contact points
  int n = M.rows();
  int nq = n - 6;
  int nc = NT.columns();
  int nk = D_.columns()/nc;

  // setup R
  Ravelin::MatrixNd R(n, nc*5 ),DT = D_,D(n,nc*nk),N = NT;
  D = DT;
  D.transpose();
  N.transpose();
  R.block(0,n,0,nc) = NT;
  R.block(0,n,nc,nc*5) = DT;

  OUTLOG(NT,"N'",logDEBUG1);
  OUTLOG(DT,"D'",logDEBUG1);

  // compute D, E, and F
  Ravelin::MatrixNd iM_chol = M;
  assert(LA_.factor_chol(iM_chol));

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
  assert(LA_.factor_chol(iF));

  // compute j and k
  // [F,E']
  Ravelin::MatrixNd FET(F.rows(),F.columns()+E.rows()),
      ET = E;
  ET.transpose();
  FET.set_sub_mat(0,0,F);
  FET.set_sub_mat(0,F.columns(),ET);

  Ravelin::VectorNd vq(nq);
  vel.get_sub_vec(0,nq,vq);

  Ravelin::VectorNd v = vel;
  iM.mult(fext,v,dt,1);

  Ravelin::VectorNd vqstar;
//  ((vqstar = qdd) *= dt) += vq;
  ((vqstar = qdd) *= dt) += v.segment(0,nq);
  //////////////////////////////////////////////////////////////
  Ravelin::MatrixNd P;
  P.set_zero(nq,n);
  P.set_sub_mat(0,0,Ravelin::MatrixNd::identity(nq));
  Ravelin::MatrixNd PT = P;
  PT.transpose();

  OUTLOG(PT,"P'",logDEBUG1);

  Ravelin::MatrixNd Cd_iM_CdT,   Cd_iM_CnT, Cd_iM_JxT,
                    Cn_iM_CdT,   Cn_iM_CnT,   Cn_iM_JxT,
                 /*  Jx_iM_CdT,  Jx_iM_CnT,*/ Jx_iM_JxT;

  // S
  LA_.solve_chol_fast(iM_chol,_workM = DT);
  D.mult(_workM,Cd_iM_CdT);

  LA_.solve_chol_fast(iM_chol,_workM =  NT);
  D.mult(_workM,Cd_iM_CnT);

  LA_.solve_chol_fast(iM_chol,_workM = PT);
  D.mult(_workM,Cd_iM_JxT);

  // N
  LA_.solve_chol_fast(iM_chol,_workM = DT);
  N.mult(_workM,Cn_iM_CdT);

  LA_.solve_chol_fast(iM_chol,_workM = NT);
  N.mult(_workM,Cn_iM_CnT);

  LA_.solve_chol_fast(iM_chol,_workM = PT);
  N.mult(_workM,Cn_iM_JxT);

  // P
//  LA_.solve_chol_fast(iM_chol,_workM = DT);
//  P.mult(_workM,Jx_iM_CdT);

//  LA_.solve_chol_fast(iM_chol,_workM = NT);
//  P.mult(_workM,Jx_iM_CnT);

  LA_.solve_chol_fast(iM_chol,_workM = PT);
  P.mult(_workM,Jx_iM_JxT);

  Ravelin::VectorNd Cd_v, Cn_v, Jx_v;
  D.mult(v,Cd_v);
  N.mult(v,Cn_v);
  P.mult(v,Jx_v);

  // Printouts
  OUTLOG(Cd_iM_CdT,"Cd_iM_CdT",logDEBUG1);
//  OUTLOG(Cd_iM_CnT,"Cd_iM_CnT",logDEBUG1);
  OUTLOG(Cd_iM_JxT ,"Cd_iM_JxT",logDEBUG1);

  OUTLOG(Cn_iM_CdT,"Cn_iM_CdT",logDEBUG1);
  OUTLOG(Cn_iM_CnT,"Cn_iM_CnT",logDEBUG1);
  OUTLOG(Cn_iM_JxT ,"Cn_iM_JxT",logDEBUG1);

//  OUTLOG( Jx_iM_CdT,"Jx_iM_CdT",logDEBUG1);
//  OUTLOG( Jx_iM_CnT,"Jx_iM_CnT",logDEBUG1);
  OUTLOG( Jx_iM_JxT ,"Jx_iM_JxT",logDEBUG1);


  /////////////////////////////////////////////////////////////////////////////
  /// Solve System
  ///
  const unsigned J_IDX = 0;
  const unsigned N_IDX = 0;
  const unsigned D_IDX = nc;
  const unsigned E_IDX = D_IDX + nc*nk;

  // we do this by solving the MLCP:
  // |  A  C  | | x | + | g | = | 0 |
  // |  D  B  | | y |   | h |   | 0 |

  // g = [-M*v'; 0; 0; -vq_err]
  // h = [ 0 ];
  // A = [ M -P';
  //       P  0 ];

  // B = [ 0  0  0
//         0  0  E
//        mu -E' 0 ];

  // C = [ -N' -D' 0 ;
  //        0   0  0 ];
  // D = -C'
  // x = [ v^+; tau ]
  // y = [ cN, cD, lambda]

  Ravelin::MatrixNd B;
  // lower left & upper right block of matrix
  Ravelin::MatrixNd _LL, _UR;
  if (nk > 4)
    _LL.set_zero(nc*nk,nc+nc*nk);
  else
    _LL.set_zero(nc,nc+nc*nk);
  _UR.set_zero(_LL.columns(),_LL.rows());
  for(unsigned i=0,j=0,r=0;i<nc;i++)
  {
    unsigned NK = MU.columns();
    if (NK > 4)
    {
      assert(nk == NK);
      int nk4 = (NK+4)/4;
      for(unsigned k=0;k<nk4;k++)
      {
        // muK
        _LL(r+k,i)         = MU(i,k);
        // Xe
        _LL(r+k,nc+j)      = -cos((M_PI*k)/(2.0*nk4));
        _LL(r+k,nc+nc+j)   = -cos((M_PI*k)/(2.0*nk4));
        // Xf
        _LL(r+k,nc+nc*2+j) = -sin((M_PI*k)/(2.0*nk4));
        _LL(r+k,nc+nc*3+j) = -sin((M_PI*k)/(2.0*nk4));
        // XeT
        _UR(nc+j,r+k)      =  cos((M_PI*k)/(2.0*nk4));
        _UR(nc+nc+j,r+k)   =  cos((M_PI*k)/(2.0*nk4));
        // XfT
        _UR(nc+nc*2+j,r+k) =  sin((M_PI*k)/(2.0*nk4));
        _UR(nc+nc*3+j,r+k) =  sin((M_PI*k)/(2.0*nk4));
      }
      r+=nk4;
      j++;
    }
    else
    {
//      assert(NK == 1);
      // muK
      _LL(r,i)         = MU(i,0);
      // Xe
      _LL(r,nc+j)      = -1.0;
      _LL(r,nc+nc+j)   = -1.0;
      // Xf
      _LL(r,nc+nc*2+j) = -1.0;
      _LL(r,nc+nc*3+j) = -1.0;
      // XeT
      _UR(nc+j,r)      =  1.0;
      _UR(nc+nc+j,r)   =  1.0;
      // XfT
      _UR(nc+nc*2+j,r) =  1.0;
      _UR(nc+nc*3+j,r) =  1.0;
      r += 1;
      j++;
    }
  }
  B.set_zero(_LL.rows()+_UR.rows(),_LL.columns()+_UR.columns());
  B.set_sub_mat(_UR.rows(),0,_LL);
  B.set_sub_mat(0,_LL.columns(),_UR);

  // Assuming that C is of full row rank (no dependent joint constraints)
  // A is invertible; then we just need to solve the LCP:

  // | B - D*inv(A)*C | | v | + | h - D*inv(A)*g | = | w |
  // and use the result to solve for u:
  // u = -inv(A)*(g + Cv)

  // A is the matrix | M X'|
  //                 | X 0 |  where X is [ S; T; P ]
  // blockwise inversion yields
  // inv(A) = [
  //    inv(M)-inv(M)*X'*Y*X*inv(M)   inv(M)*X'*Y ;
  //    Y*X*inv(M)                    -Y          ]
  // where Y = inv(X*inv(M)*X')

  // defining Q = C and using the result above yields following LCP:
  // matrix: Q*inv(A)*Q' = Q*inv(M)*Q' - Q*inv(M)*X'*Y*X*inv(M)*Q'
  // vector: Q*inv(A)*a  = -(-Q*v + Q*inv(M)*X'*Y*X*v - Q*inv(M)*X'*Y*[0;0;vq*])
  //                     = Q*v - Q*inv(M)*X'*Y*X*v + Q*inv(M)*X'*Y*[0;0;vq*])

  Ravelin::MatrixNd _MM,
      _Y = Jx_iM_JxT;

  // Invert (chol factorize) matrix _Y
  bool success = _LA.factor_chol(_Y);
  assert(success);

  Ravelin::MatrixNd _Q_iM_XT;
  Ravelin::VectorNd _qq,_v;

  // inv(A) = [
  //    inv(M)-inv(M)*X'*Y*X*inv(M)   inv(M)*X'*Y ;
  //    Y*X*inv(M)                    -Y          ]
  // defining Y = inv(X*inv(M)*X') and Q = [Cn]
  // matrix:B-D*inv(A)*C = Q*inv(M)*Q' - Q*inv(M)*X'*Y*X*inv(M)*Q'
  //
  // vector:h-Q*inv(A)*a  = -(-Q*inv(M)*M*v - -Q*inv(M)*X'*Y*X*inv(M)*M*v + -Q*inv(M)*X'*Y*[0;0;vq*])
  //                     = -(-Q*v + Q*inv(M)*X'*Y*X*v - Q*inv(M)*X'*Y*[0;0;vq*])
  //                     =    Q*v - Q*inv(M)*X'*Y*X*v + Q*inv(M)*X'*Y*[0;0;vq*]

  // setup Q*inv(M)*Q'
  _MM.set_zero(nc+nc*nk+nc,nc+nc*nk+nc);
  _MM.set_sub_mat(0,0,Cn_iM_CnT);
  _MM.set_sub_mat(nc,0,Cd_iM_CnT);
  _MM.set_sub_mat(0,nc,Cn_iM_CdT);
  _MM.set_sub_mat(nc,nc,Cd_iM_CdT);

  OUTLOG(_MM,"Q_iM_QT",logDEBUG1);

  // setup Q*inv(M)*X'
  // NOTE: with will break if nk != 4
  _Q_iM_XT.resize(nc+nc*nk+nc,nq);
  _Q_iM_XT.set_sub_mat(N_IDX, J_IDX, Cn_iM_JxT);
  _Q_iM_XT.set_sub_mat(D_IDX, J_IDX, Cd_iM_JxT);
  _Q_iM_XT.set_sub_mat(E_IDX, J_IDX, Ravelin::MatrixNd::zero(nc,nq));

  OUTLOG(_Q_iM_XT,"Q_iM_XT",logDEBUG1);

//  Ravelin::MatrixNd Y;
//  Y = Ravelin::MatrixNd::identity(_Y.rows());
//  _LA.solve_chol_fast(_Y, Y);

  // compute Y*X*inv(M)*Q'
  Ravelin::MatrixNd::transpose(_Q_iM_XT, _workM);
  _LA.solve_chol_fast(_Y, _workM);

  // compute Q*inv(M)*X'*Y*X*inv(M)*Q'
  _Q_iM_XT.mult(_workM, _workM2);
  _MM  -= _workM2;
  _MM += B;
  OUTLOG(_MM,"MM",logDEBUG1);

  // setup -Q*v
  _qq.set_zero(nc*(1+nk+1));
  _qq.set_sub_vec(0,Cn_v);
  _qq.set_sub_vec(nc,Cd_v);

  // setup X*v
  Ravelin::VectorNd _Xv,_YXv;
  _Xv.resize(nq);
  _Xv.set_sub_vec(J_IDX, Jx_v);

  // compute Y*X*v
  _YXv = _Xv;
  _LA.solve_chol_fast(_Y, _YXv);

  // compute Q*inv(M)*X' * Y*X*v
  // & subtract from LCP Vector
  _Q_iM_XT.mult(_YXv, _qq,-1.0,1.0);


  // compute Q*inv(M)*X'*Y*[vq*]
  Ravelin::VectorNd _Yvqstar;
  _Yvqstar.set_zero(nq);
  _Yvqstar.set_sub_vec(J_IDX, vqstar);
  _LA.solve_chol_fast(_Y, _Yvqstar);
  // & add to LCP Vector
  _Q_iM_XT.mult(_Yvqstar, _qq,1.0,1.0);

  OUTLOG(_qq,"qq",logDEBUG1);
  // setup remainder of LCP vector

  // Setup Indices vector
  std::vector<unsigned> indices;
  unsigned active_eefs = 0;
  for(int i=0;i<eefs_.size();i++){
    if(!eefs_[i].active){
      continue;
    }
    active_eefs++;
    for(int j=0;j<eefs_[i].point.size();j++)
      indices.push_back(i);
  }

  // D
  for(int i=0;i<nk;i++){
    for(int i=0;i<eefs_.size();i++){
      if(!eefs_[i].active){
        continue;
      }
      for(int j=0;j<eefs_[i].point.size();j++)
        indices.push_back(i);
    }
  }

  // E
  for(int i=0;i<eefs_.size();i++){
    if(!eefs_[i].active){
      continue;
    }
    for(int j=0;j<eefs_[i].point.size();j++)
      indices.push_back(i);
  }

  // attempt to solve the LCP using the fast method
  if(active_eefs > 2){
    OUT_LOG(logERROR) << "-- using: lcp_fast" << std::endl;

    if (!lcp_fast(_MM, _qq,indices, _v,CHECK_ZERO))
    {
      OUT_LOG(logERROR) << "-- Principal pivoting method LCP solver failed; falling back to regularized lemke solver" << std::endl;

      if (!_lcp.lcp_lemke_regularized(_MM, _qq, _v,-20,4,1))
        throw std::runtime_error("Unable to solve constraint LCP!");
    }
  } else {
    OUT_LOG(logERROR) << "-- using: Moby::LCP::lcp_fast" << std::endl;

    if (!_lcp.lcp_fast(_MM, _qq, _v))
    {
      OUT_LOG(logERROR) << "Principal pivoting method LCP solver failed; falling back to regularized lemke solver" << std::endl;

      if (!_lcp.lcp_lemke_regularized(_MM, _qq, _v,-20,4,1))
        throw std::runtime_error("Unable to solve constraint LCP!");
    }
  }
  OUTLOG(_v,"v",logDEBUG1);

  Ravelin::VectorNd tau;

  // compute the friction forces
  // u = -inv(A)*(a + Cv)
  // u = inv(A)*(Q'*[cn])  [b/c we don't care about new velocity]
  // recalling that inv(A) =
  // | inv(M)-inv(M)*X'*Y*X*inv(M)   inv(M)*X'*Y | ngc x ngc,    ngc x sz(x)
  // | Y*X*inv(M)                    -Y          | sz(x) x ngc,  sz(x) x sz(x)
  // Q is nlcp x (ngc + sz(x))
  // [cs; ct] = -Y*X*v - Y*X*inv(M)*Q'*[cn]
  tau = _YXv;
  _Q_iM_XT.transpose_mult(_v, _workv);
  _LA.solve_chol_fast(_Y, _workv);
  tau += _workv;
  tau.negate();

  Ravelin::VectorNd cn,cd;
  // setup impulses
  cn = _v.segment(0, nc);
  cd = _v.segment(nc, nc*(nk+1));

  OUTLOG(cn,"cN",logDEBUG);
  OUTLOG(cd,"cD",logDEBUG);

  cf.set_zero(nc*(nk+1));
  cf.set_sub_vec(0,cn);
  cf.set_sub_vec(nc,cd);

//   IDYN MLCP
//            A             C      x         g
//       | M −S' -T' -P'   -N' | | v+ |   |-M v |    | 0 |
//       | S  0   0   0     0  | | cS |   |  0  |  = | 0 |
//       | T  0   0   0     0  | | cT |   |  0  |    | 0 |
//       | P  0   0   0     0  | | a  |   |-vq* |    | 0 |
//                              *       +
//       | N  0   0   0     0  | | cN |   |  0  | >= | 0 |
//            D            B      y         h

//   Constraints
//   [M −S' -T' -N' -P'  P' ].[ ]' - M*v  = 0
//    S v+        = 0
//    T v+        = 0
//    P v+ - vq*  = 0
//    N v+       >= 0

//    M v+ - (S'cs + T'cT + N'cN) - a = M v
//    M(v+ - v) = (S'cs + T'cT + N'cN)  +  a

//   Therefore
//    M(v+ - v)           = f_total
//    S'cs + T'cT + N'cN  = contact_force
//    f = cf + a

  // Using M(dv) - z = tau
  x = tau;
  if(nc>0)
    x -= R.mult(cf,_workv).segment(0,nq);
  x /= dt;

  OUTLOG(x,"tau",logDEBUG);

  OUT_LOG(logDEBUG) << "<< inverse_dynamics_ap() exited" << std::endl;
  return true;
}
