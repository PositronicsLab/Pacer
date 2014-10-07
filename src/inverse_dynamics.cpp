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
  ((vqstar = qdd) *= h) += vq;

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
  assert(LA_.factor_chol(iM_chol));

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

#ifndef NDEBUG
  OUTLOG(N,"N",logDEBUG);
//  OUTLOG(F,"",logDEBUG);
#endif

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

  OUTLOG(R,"R",logDEBUG1);

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
  return true;
}



//#define LCP_METHOD
bool Robot::workspace_inverse_dynamics(const Ravelin::VectorNd& v, const Ravelin::VectorNd& vw_bar_, const Ravelin::MatrixNd& M, const Ravelin::VectorNd& fext, double h, const Ravelin::MatrixNd& MU, Ravelin::VectorNd& x, Ravelin::VectorNd& z){
  static Ravelin::VectorNd tau = x;
  // get number of degrees of freedom and number of contact points
  int n = M.rows();
  int nq = n - 6;
  Ravelin::VectorNd vw_bar = vw_bar_;

//   (workM_ = Rw).get_sub_mat(0,NUM_EEFS*3,0,NDOFS,Rw);
//   (workv_ = vw_bar).get_sub_vec(0,NUM_EEFS*3,vw_bar);
  OUTLOG(Rw,"Rw",logDEBUG1);
  OUTLOG(vw_bar,"vw_bar",logDEBUG1);


  // SRZ: Ground reaction forces are set to zero
  // (they destabilize robot system, why?)

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

  OUTLOG(vw_bar,"vw_bar",logERROR);
  Ravelin::VectorNd dvw;
  // dvw = vw_bar - vw
  // vw = Rw*v
  Rw.mult(v,dvw,-1,0);
  OUTLOG(dvw,"vw",logERROR);
  dvw += vw_bar;
  OUTLOG(dvw,"dvw",logERROR);

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

    ///////////////////////////////////////////////////////////
    //////////////////////// LP PHASE /////////////////////////
    /* OBJECTIVE
     * Min workspace deviation from dvw
     * min_{x = [z,T,s]'}
     *   [0,0,1] x
     * s.t.
     *   -[ C ,-I] x   >= -dvw              // operational space goal l1-norm
     *   [ C ,-I] x    >=  dvw              //
     *   [ N'*A ,0] x >= -N(h*iM*fext + v)   // Interpenetration
     *   [CF,0,0] x      >=  0                  // coulomb friction
     *   z >= 0                                 // compressive force
     *   T+ >= T >= T-                          // torque limits
     */

    Ravelin::MatrixNd lpA(WS_DOFS*2 + NC*2,nq+NC*5+WS_DOFS),
                      Aeq(0,lpA.columns());
    Ravelin::VectorNd lpb(lpA.rows()),
                      lpc,
                      x1,x2,
                      beq(0);

    lpA.set_zero();
    x1.set_zero(NVARS);
    x2.set_zero(NVARS);
    lpb.set_zero();

    lpc.set_zero(nq+NC*5+WS_DOFS);
    lpc.set_sub_vec(nq+NC*5,Ravelin::VectorNd::one(WS_DOFS));

//    C = J*iM*[R F]
//    d = dvw-J*iM*f
//    A = [ -[eye(nc).*mu -eye(nc) -eye(nc) -eye(nc) -eye(nc)] zeros(nc,nq)
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

    // Linear Constraints FOR BOTH OPTIMIZATIONS
    // A*x >= b
    //    A = [ [eye(nc).*mu -eye(nc) -eye(nc) -eye(nc) -eye(nc)] zeros(nc,nq)
    //          N'*iM*[R F]
    //        ]
    //    b = [  zeros(nc,1)
    //           -N'*(v)
    //        ]
    qpA.set_zero();
    qpb.set_zero();
    //  -- Coulomb Friction --
    qpA.set_sub_mat(0,0,CF);
    qpb.set_sub_vec(0,Ravelin::VectorNd::zero(NC));

    //  -- Interpenetration --
    // N'*iM*[R F]
    N.transpose_mult(iMRTF,workM2,1,0);
    qpA.set_sub_mat(NC,0,workM2);

    // external forces are reduced to 0 as dt->0 --< N'*(v+iM*f) >--
//    iM.mult(fext,workv1,h,0);
//    workv1 += v;
//    N.transpose_mult(workv1,workv2);
    // -N'*(v)
    N.transpose_mult(v,workv2,1,0);
    qpb.set_sub_vec(NC,workv2);

    lpA.set_sub_mat(WS_DOFS*2,0,qpA);
    lpb.set_sub_vec(WS_DOFS*2,qpb);

    ///////  Special LP constraint
    // -- operational space goal l1-norm --
    //    C = J*iM*[R F]
    //    d = dvw-J*iM*f
    Ravelin::MatrixNd C = JiMRTF;
    assert(WS_DOFS == C.rows());

    // -C x + d >= -s  <--  C x - d <=  s
    //  C x - d >= -s

    // -C x + s >= -d
    //  C x + s >=  d

    //
    //
    // A =    [-C  I;
    //          C  I]
    lpA.set_sub_mat(0,0,(workM_ = C).negate());
    lpA.set_sub_mat(C.rows(),0,C);
    lpA.set_sub_mat(       0,C.columns(),Ravelin::MatrixNd::identity(C.rows()));
    lpA.set_sub_mat(C.rows(),C.columns(),Ravelin::MatrixNd::identity(C.rows()));

    Ravelin::VectorNd d = dvw;
    Ravelin::VectorNd ll(NVARS+WS_DOFS),ul(NVARS+WS_DOFS);
#define NO_LP
#ifndef NO_LP
    // NOTE: external forces are reduced to 0 as dt->0
    //    iM.mult(fext,workv1,-h,0);
    //    Rw.mult(workv1,d);
    //    d += dvw;

    lpb.set_sub_vec(0,(workv_ = d).negate());
    lpb.set_sub_vec(d.rows(),d);

    // Lower Torque Limit
    std::fill(ll.begin(), ll.end(), -1e30);
    ll.set_sub_vec(0,Ravelin::VectorNd::zero(NC*5));
    //ll.set_sub_vec(NC*5,torque_limits_l);

    // Upper Torque Limit
    std::fill(ul.begin(), ul.end(), 1e30);
    //ul.set_sub_vec(NC*5,torque_limits_u);

    x1.set_zero(lpc.rows());
    OUTLOG(lpA,"lpA",logDEBUG1);
    OUTLOG(lpb,"lpb",logDEBUG1);
    OUTLOG(lpc,"lpc",logDEBUG1);
    OUTLOG(ll ,"lpl" ,logDEBUG1);
    OUTLOG(ul ,"lpu" ,logDEBUG1);
    OUTLOG(x1 ,"lpx0"  ,logDEBUG1);

    Opt::LPParams params(lpc,Aeq,beq,lpA,lpb,ll,ul);
//    params.n = lpc.rows();
    unsigned LP_RESULT_FLAG = 0;

    /// Solves a linear program using the simplex method
    /**
     *  Solves the linear program:  min c'x
     *                 subject to:  Ax = b
     *                              Mx >= q
     * \param the optimal point (if any) on return
     * \return <b>false</b> if problem infeasible; <b>true</b> otherwise
     */
    if(!Opt::LP::lp_simplex(params,x1,LP_RESULT_FLAG)){
      OUT_LOG(logERROR)  << "ERROR: Unable to solve widyn LP! : " << LP_RESULT_FLAG ;
      x = tau;
      return false;
    }

    OUTLOG(x1.segment(NVARS,NVARS+WS_DOFS),"LP: [s]",logERROR);
    OUT_LOG(logERROR) << "LP: Objective: c' x = " << x1.dot(lpc);
//    assert(NC != 4);

    lpA.mult(x1,workv_);
    workv_ -= lpb;
    OUTLOG(workv_,"feas_geq_0",logDEBUG);

    (workv1 = x1).get_sub_vec(0,NVARS,x1);
    OUTLOG(x1,"LP: [z,T]",logDEBUG);
    x1.get_sub_vec(NC*3,NC*3+nq,x);

    // new dvw ( will be variable d = J*inv(M)*[R' F] x )
//    iM.mult(fext,workv1,h,0);
//    Rw.mult(workv1,d);
    OUTLOG(d,"old_dvw",logINFO);
    C.mult(x1,d);
#endif
    // New OS goal (found by LP)
    // C*x + fext
    OUTLOG(d,"new_dvw",logINFO);

//    (workM1 = C).get_sub_mat(0,NUM_EEFS*3,0,C.columns(),C);
//    (workv1 = d).get_sub_vec(0,NUM_EEFS*3,d);

    LA_.nullspace(JiMRTF,P);
    unsigned size_null_space = P.columns();
    if(size_null_space == 0) return true;

    ///////////////////////////////////////////////////////////
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
    ll.resize(NVARS);
    ul.resize(NVARS);

    double clamp_val = 1e30;
    // Lower var Limits
    std::fill(ll.begin(), ll.end(),-clamp_val);
    ll.set_sub_vec(0,Ravelin::VectorNd::zero(NC*5));
//    ll.set_sub_vec(NC*5,torque_limits_l);

    // Upper var Limits
    std::fill(ul.begin(), ul.end(), clamp_val);
//    ul.set_sub_vec(NC*5,torque_limits_u);
    x2 = x1;

    OUTLOG(ll,"qpl",logDEBUG1);
    OUTLOG(ul,"qpu",logDEBUG1);
    OUTLOG(qpA,"qpA",logDEBUG1);
    OUTLOG(qpb,"qpb",logDEBUG1);
    OUTLOG(qpQ,"qpQ",logDEBUG1);
    OUTLOG(qpc,"qpc",logDEBUG1);
    OUTLOG(Aeq,"Aeq",logDEBUG1);
    OUTLOG(beq,"beq",logDEBUG1);
    OUTLOG(x2,"x0",logDEBUG1);

    bool RESULT_FLAG = false;
//#define LCP_METHOD
#ifdef LCP_METHOD
    Ravelin::MatrixNd lcpA;
    lcpA.set_zero(Aeq.rows()*2 + qpA.rows() + NC*5,NVARS);

    lcpA.set_sub_mat(0,0,Aeq);

    lcpA.set_sub_mat(Aeq.rows(),0,(workM_ = Aeq).negate());

    lcpA.set_sub_mat(Aeq.rows()*2,0,qpA);

    lcpA.set_sub_mat(Aeq.rows()*2 + qpA.rows(),0,Ravelin::MatrixNd::identity(NC*5));

    Ravelin::VectorNd lcpb;

    lcpb.set_zero(beq.rows()*2 + qpA.rows() + NC*5);

    workv_.set_zero(beq.rows());
    std::fill(workv_.begin(), workv_.end(),-Moby::NEAR_ZERO);

    lcpb.set_sub_vec(0,workv_ += beq);

    lcpb.set_sub_vec(beq.rows(),(workv_ += beq).negate());

    lcpb.set_sub_vec(beq.rows()*2,qpb);

//    lcpb.set_sub_vec(beq.rows()*2 + qpA.rows(),Ravelin::VectorNd::zero(NC*5));

    solve_qp(qpQ,qpc,lcpA,lcpb,x2);
#else
    /// Active-set QP algorithm for solving QPs
    /**
     * \param H the quadratic term
     * \param c the linear term
     * \param lb the lower bound
     * \param ub the lower bound
     * \param M the linear inequality constraints (M*z >= q)
     * \param q the linear inequality bound (M*z >= q)
     * \param A the linear equality constraint (A*z = b)
     * \param b the linear equality constraint (A*z = b)
     * \param z a vector "close" to the solution on input (optional); contains
     *        the solution on output
     */
    solve_qp(qpQ,qpc,ll,ul,Aeq,beq,qpA,qpb,x2);
#endif
    // ------------------------------------------------------------------------
    // ---------------- CHECK FEASIBILITY OF RESULT ---------------------------
      OUTLOG(x2,"x = [z,tau]",logDEBUG);

      qpA.mult(x2,workv_);
      workv_ -= qpb;
      OUTLOG(workv_,"feas_geq_0",logDEBUG);


      Aeq.mult(x2,workv_);
      workv_ -= beq;
      OUTLOG(workv_,"feas_eq_0",logDEBUG);

      x2.get_sub_vec(0,NC*5,z);
      OUTLOG(z,"cfs",logDEBUG);
      (workv1 = x2).get_sub_vec(NC*5,NVARS,x);
  }
  tau = x;
  return true;
}

bool Robot::workspace_inverse_dynamics(const Ravelin::VectorNd& v, const Ravelin::VectorNd& vw_bar_, const Ravelin::MatrixNd& M, const Ravelin::VectorNd& fext, double h, const Ravelin::MatrixNd& MU, Ravelin::VectorNd& x){
  OUT_LOG(logDEBUG) <<  ">> workspace_inverse_dynamics (feedback CFs) entered ";
  // get number of degrees of freedom and number of contact points
  int n = M.rows();
  int nq = n - 6;
  Ravelin::VectorNd vw_bar = vw_bar_;

  OUTLOG(Rw,"Rw",logDEBUG1);
  OUTLOG(vw_bar,"vw_bar",logDEBUG1);

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

  OUTLOG(vw_bar,"vw_bar",logERROR);
  Ravelin::VectorNd dvw;
  // dvw = vw_bar - vw
  // vw = Rw*v
  Rw.mult(v,dvw,-1,0);
  OUTLOG(dvw,"vw",logERROR);
  dvw += vw_bar;
  OUTLOG(dvw,"dvw",logERROR);

    const int NVARS = nq;
    int WS_DOFS = Rw.rows();

    // Do larger "Coupled" optimizations
    // for torque and cfs

    //////////////////////// COEFS /////////////////////////
    // iM [F]: [\tau] --> [generalized_xdd]
    Ravelin::MatrixNd iMF;
    iM.mult(F,iMF);

    // QP Nullspace
    Ravelin::MatrixNd P;
    Ravelin::MatrixNd JiMF;
    Rw.mult(iMF,JiMF);

    ///////////////////////////////////////////////////////////
    //////////////////////// LP PHASE /////////////////////////
    /* OBJECTIVE
     * Min workspace deviation from dvw
     * min_{x = [T,s]'}
     *   [0,1] x
     * s.t.
     *   -[ 0 ,-I] x   >= -dvw              // operational space goal l1-norm
     *   [ 0 ,-I] x    >=  dvw              //
     *   T+ >= T >= T-                          // torque limits
     */

    Ravelin::MatrixNd lpA(WS_DOFS*2,nq+WS_DOFS),
                      Aeq(0,lpA.columns());
    Ravelin::VectorNd lpb(lpA.rows()),
                      lpc,
                      x1,
                      beq(Aeq.rows());


    lpA.set_zero();
    x1.set_zero(NVARS);
    lpb.set_zero();

    lpc.set_zero(nq+WS_DOFS);
    lpc.set_sub_vec(nq,Ravelin::VectorNd::one(WS_DOFS));

//    C = J*iM*[R F]
//    d = dvw-J*iM*f

//      lpM*x <= lpq
//    lpM = [ C  -eye(nw)
//           -C  -eye(nw)  % -dv +s >=  v -vw +fext
//          ]
//    lpq = [ d % +dv +s >= -v +vw -fext
//           -d % -dv +s >=  v -vw +fext
//          ]

    ///////  Special LP constraint
    // -- operational space goal l1-norm --
    //    C = J*iM*[R F]
    //    d = dvw-J*iM*f
    Ravelin::MatrixNd C = JiMF;
    assert(WS_DOFS == C.rows());

    // -C x + d >= -s  <--  C x - d <=  s
    //  C x - d >= -s

    // -C x + s >= -d
    //  C x + s >=  d

    //
    //
    // A =    [-C  I;
    //          C  I]
    lpA.set_sub_mat(0,0,(workM_ = C).negate());
    lpA.set_sub_mat(C.rows(),0,C);
    lpA.set_sub_mat(       0,C.columns(),Ravelin::MatrixNd::identity(C.rows()));
    lpA.set_sub_mat(C.rows(),C.columns(),Ravelin::MatrixNd::identity(C.rows()));

    Ravelin::VectorNd d = dvw;
    Ravelin::VectorNd ll(NVARS+WS_DOFS),ul(NVARS+WS_DOFS);

    // NOTE: external forces are active on robot (includes contact forces)
    iM.mult(fext,workv1,h,0);
    Rw.mult(workv1,d,1,1);

    lpb.set_sub_vec(0,(workv_ = d).negate());
    lpb.set_sub_vec(d.rows(),d);

    double clamp_val = 1e30;
    // Lower Torque Limit
    std::fill(ll.begin(), ll.end(), -clamp_val);
    ll.set_sub_vec(0,torque_limits_l);

    // Upper Torque Limit
    std::fill(ul.begin(), ul.end(), clamp_val);
    ul.set_sub_vec(0,torque_limits_u);

    x1.set_zero(lpc.rows());
    OUTLOG(lpA,"lpA",logDEBUG1);
    OUTLOG(lpb,"lpb",logDEBUG1);
    OUTLOG(lpc,"lpc",logDEBUG1);
    OUTLOG(ll ,"lpl" ,logDEBUG1);
    OUTLOG(ul ,"lpu" ,logDEBUG1);
    OUTLOG(x1 ,"lpx0"  ,logDEBUG1);

    Opt::LPParams params(lpc,Aeq,beq,lpA,lpb,ll,ul);
//    params.n = lpc.rows();
    unsigned LP_RESULT_FLAG = 0;

    /// Solves a linear program using the simplex method
    /**
     *  Solves the linear program:  min c'x
     *                 subject to:  Ax = b
     *                              Mx >= q
     * \param the optimal point (if any) on return
     * \return <b>false</b> if problem infeasible; <b>true</b> otherwise
     */
    if(!Opt::LP::lp_simplex(params,x1,LP_RESULT_FLAG)){
      OUT_LOG(logERROR)  << "ERROR: Unable to solve widyn LP! : " << LP_RESULT_FLAG ;
      return false;
    }
    OUTLOG(x1.segment(NVARS,NVARS+WS_DOFS),"LP: [s]",logERROR);
    OUT_LOG(logERROR) << "LP: Objective: c' x = " << x1.dot(lpc);
//    assert(NC != 4);

    lpA.mult(x1,workv_);
    workv_ -= lpb;
    OUTLOG(workv_,"feas_geq_0",logDEBUG);

    (workv1 = x1).get_sub_vec(0,NVARS,x1);
    OUTLOG(x1,"LP: [T]",logDEBUG);
    x = x1.segment(0,nq);

    // new dvw ( will be variable d = J*inv(M)*[R' F] x )
//    iM.mult(fext,workv1,h,0);
//    Rw.mult(workv1,d);
    OUTLOG(d,"old_dvw",logINFO);
    C.mult(x1,d);
    iM.mult(fext,workv1,h,0);
    Rw.mult(workv1,d,1,1);

    // New OS goal (found by LP)
    // C*x + fext
    OUTLOG(d,"new_dvw",logINFO);

    LA_.nullspace(JiMF,P);
    unsigned size_null_space = P.columns();
//    if(size_null_space == 0) return true;

  ///// Now Find the minimal force to achieve that workspace task

  x = x1.segment(0,nq);

  return true;
}
