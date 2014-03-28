#include <robot.h>

const double NEAR_ZERO = sqrt(std::numeric_limits<double>::epsilon()); //2e-12;

extern bool solve_qp(const Ravelin::MatrixNd& Q, const Ravelin::VectorNd& c, const Ravelin::MatrixNd& A, const Ravelin::VectorNd& b, Ravelin::VectorNd& x);

//#define USE_D

/// Friction Estimation
/// Calculates Coulomb friction at end-effectors
double Robot::friction_estimation(const Ravelin::VectorNd& v, const Ravelin::VectorNd& f, double dt,
                         const Ravelin::MatrixNd& N,const Ravelin::MatrixNd& D, const Ravelin::MatrixNd& M, Ravelin::MatrixNd& MU, Ravelin::VectorNd& cf)
{
    std::cout << "entered friction_estimation()" << std::endl;
    static Ravelin::VectorNd v_(6), f_(6);  // previous values
    static int ITER = 0;
    double norm_error = -1;
    int nc = N.columns();

    if(nc > 0 && N.rows() == f_.rows()){
      std::cout << "started friction_estimation()" << std::endl;
      ITER++;
      std::cout << "************** Friction Estimation **************" << std::endl;
      std::cout << "ITER: " << ITER << std::endl;
      std::cout << "dt = " << dt << std::endl;
      OUTLOG(N,"N",logDEBUG1);
      OUTLOG(M,"M",logDEBUG1);
      OUTLOG(v,"post-event-vel",logDEBUG1);
      OUTLOG(v_,"pre-event-vel",logDEBUG1);
      OUTLOG(f_,"f_external",logDEBUG1);

      int ngc = f_.rows();
      int nq = ngc - 6, nk = D.columns()/nc;
      int nvars = nc+nk*nc;
      cf.resize(nc + nc*(nk/2));

     // dv = v_{t} - v_{}
      Ravelin::VectorNd dv =  v;
      dv -= v_;
//      OUTLOG(dv,"dv");

      // j_obs = M*dv
      Ravelin::VectorNd jstar(M.rows());
      M.mult(dv,jstar);
//      OUTLOG(jstar,"j_observed");

      // j_exp = f_{t-1}*dt
      f_ *= dt;

//      OUTLOG(f_,"j_expected");

      // j* = (j_obs - j_exp) = j_err
      jstar -= f_;
//      OUTLOG(jstar,"j_error");


      /// //////////////////////////////
      /// STAGE I
#ifdef USE_D
     OUTLOG(D,"D");
      int n = N.columns()+D.columns();

      Ravelin::MatrixNd R(ngc,n);
      R.set_sub_mat(0,0,N);
      R.set_sub_mat(0,N.columns(),D);

      /////////// OBJECTIVE ////////////
      // M
      Ravelin::MatrixNd Q(R.columns(),R.columns());
      R.transpose_mult(R,Q);
      // c
      Ravelin::VectorNd c(R.columns());
      R.transpose_mult(jstar,c);
      // Inequality constraint
      c.negate();

      Ravelin::VectorNd z(n);
      z.set_zero();
      if (!lcp2_.lcp_lemke_regularized(Q,c,z)){
          std::cout << "friction estimation failed" << std::endl;
      } else {

      OUTLOG(z,"z");

      Ravelin::VectorNd err(ngc);
      R.mult(z,err);
      OUTLOG(err,"gf");
      err -= jstar;

      OUTLOG(err,"err");
      std::cout << "norm err: " << err.norm() << std::endl;
#else // use ST
      Ravelin::MatrixNd ST;
      ST.resize(D.rows(),D.columns()/2);
      ST.set_zero();
      // remove negations from D to create ST
      // of the form [S T]

      for(int i=0;i<nc;i++){
        ST.set_column(i,D.column(i*nk));
        ST.set_column(nc+i,D.column(i*nk+1));
      }
//     OUTLOG(ST,"ST");
     int n = N.columns()+ST.columns();

     Ravelin::MatrixNd R(ngc,n);
     R.set_sub_mat(0,0,N);
     R.set_sub_mat(0,N.columns(),ST);
//     OUTLOG(R,"R");

     // Some additional Printouts
      Ravelin::VectorNd vprint = jstar,cvprint;
      Ravelin::MatrixNd iM = M;
      LA_.factor_chol(iM);
      LA_.solve_chol_fast(iM,vprint);
      R.transpose_mult(vprint,cvprint);
//      OUTLOG(vprint,"v_error");
//      OUTLOG(cvprint,"cv_error");
     /////////// OBJECTIVE ////////////
     // Q = R'R
     Ravelin::MatrixNd Q(R.columns(),R.columns());
     R.transpose_mult(R,Q);
     // c = R' M (v_{t} - v_{t-1})
     Ravelin::VectorNd c(R.columns());
     R.transpose_mult(jstar,c);
     c.negate();

     ////////// CONSTRAINT ///////////

     Ravelin::MatrixNd A(nc,n);
     A.set_zero();
     Ravelin::VectorNd b(nc);
     b.set_zero();
     for(int i=0;i<nc;i++)
         A(i,i) = 1;

     Ravelin::VectorNd z(n);
     z.set_zero(n);

     if (!solve_qp(Q,c,A,b,z)){
         std::cout << "friction estimation failed" << std::endl;
     }
     else {
//       OUTLOG(z,"cf");
//       OUTLOG(cf_moby,"cf_MOBY");

       Ravelin::VectorNd err(ngc);
       R.mult(z,err);
//       OUTLOG(err,"generalized force from cfs = [R*z]");
       err -= jstar;
       norm_error =  err.norm();

//       OUTLOG(err,"err = [R*z - j_error]");
//       std::cout << "norm err: " << err.norm() << std::endl;

#endif

         /// //////////////////////////////
         /// STAGE II

          // Q = R'R = RTR
          int m = 0;
          Ravelin::MatrixNd U,V;
          Ravelin::VectorNd S;
          LA_.svd(Q,U,S,V);

          Ravelin::MatrixNd P;
          // SVD decomp to retrieve nullspace of R'R
          if(S.rows() != 0)
          {
              // Calculate the tolerance for ruling that a singular value is supposed to be zero
              double ZERO_TOL = std::numeric_limits<double>::epsilon() * Q.rows() * S[0];
              // Count Zero singular values
              for(int i = S.rows()-1;i>=0;i--,m++)
                  if(S[i] > ZERO_TOL)
                      break;

              // get the nullspace
              P.resize(nvars,m);
              V.get_sub_mat(0,V.rows(),V.columns() - m,V.columns(),P);
          }

//          std::cout << "m: " << m << std::endl;

          // NOTE:: stage 2 is inactive
          if(m != 0 && false)
          {
//            OUTLOG(P,"P");

            Ravelin::VectorNd cN(nc);
            z.get_sub_vec(0,nc,cN);

            /// Objective
            Ravelin::MatrixNd Q2(m,m);
            Ravelin::VectorNd c2(m), w(m);

            /// min l2-norm(cN)
            Ravelin::MatrixNd P_nc(nc,m);
            P.get_sub_mat(0,nc,0,m,P_nc);
//                OUTLOG(P_nc,"P_nc");
//                P_nc.transpose_mult(P_nc,Q2);
            // c = P'cN
//                P_nc.transpose_mult(cN,c2);

            /// min l2-norm(cf)
            P.transpose_mult(P,Q2);
            // c = P'z
            P.transpose_mult(z,c2);

            /// min l2-norm(cST)
//                Ravelin::MatrixNd P_tc = P.get_sub_mat(nc,P.rows(),0,m);
//                P_tc.transpose_mult(P_tc,Q2);
            // c = P'z
//                P_tc.transpose_mult(z.get_sub_vec(nc,z.rows()),c2);
#ifdef USE_D
            Ravelin::MatrixNd A(nvars+1,m);
            Ravelin::VectorNd  b(nvars+1);
            //      A     w        b
            //  | R'jP || w | <= |+0 |
            //  |   P  || : | >= |-z |


            //      A        w        b
            //  |-R'jP |   | w |    |+0 |
            //  |   P  | * | . | >= |-z |
            //  |   .  |   | . |    | . |
            //  |   .  |            | . |
            //  |   .  |            | . |
            //  |   .  |            | . |

            A.set_zero();

            Ravelin::VectorNd cP(m);
            //-R'jP
            P.transpose_mult(c,cP);
            A.set_row(0,cP);

            b[0] = 0;//c.dot(z);
            //  |   P  || : | >= |-z |
            Ravelin::MatrixNd nP = P;
            nP.negate();
            A.set_sub_mat(1,0,nP);

            // b = z
            Ravelin::VectorNd cNST = z;
            cNST.negate();
            OUTLOG(cNST,"cNST");
            OUTLOG(z,"z");

            b.set_sub_vec(1,z);
#else // USE_ST
            Ravelin::MatrixNd A(nc+1,m);
            Ravelin::VectorNd  b(nc+1);
            A.set_zero();
            b.set_zero();
            //      A           w         b
            //  |    R'jP    || w | <= | +0 |
            //  |  P[1:nc,:] || : | >= |-cN |
            /// -------------------------------
            //  |   -R'jP   |   | w |    | +0 |
            //  | P[1:nc,:] | * | . | >= |-cN |
            //                  | . |

            //  |   -R'jP   |   | w |    | +0 |
            Ravelin::VectorNd cP(m);
            P.transpose_mult(c,cP);
      for(int i=0;i<m;i++)
A(0,i) = cP[i];
            //A.set_row(0,cP);
            b[0] = 0;

            //  | P[1:nc,:] | * | w | >= |-cN |
            A.set_sub_mat(1,0,P_nc);
            // b = cN
            cN.negate();
            b.set_sub_vec(1,cN);
#endif
            if(!solve_qp(Q2,c2,A,b,w)) {
                std::cout << "friction estimation 2 failed" << std::endl;
            } else {
//              OUTLOG(w,"w");

              Ravelin::VectorNd z2(nvars);
              P.mult(w,z2);

//              OUTLOG(z2," z2");
              z += z2;
//              OUTLOG(z,"z+z2");

              err.set_zero(ngc);
              R.mult(z,err);
//              OUTLOG(err,"generalized force from cfs = [R*(z+z2)]");
//              err -= jstar;

//              OUTLOG(err,"err = [R*(z+z2) - j_error]");
//              norm_error =  err.norm();

//              std::cout << "norm err2: " << err.norm() << std::endl;

//              // Print Moby Error
//err.set_zero(ngc);
////                        R.mult(cf_moby,err);
//              OUTLOG(err,"MOBY generalized force from cfs = [R*(z+z2)]");
//              err -= jstar;

//              OUTLOG(err,"MOBY err = [R*(z+z2) - j_error]");
//              norm_error =  err.norm();

//              std::cout << "MOBY norm err: " << err.norm() << std::endl;
          }
        }
      }
#ifdef USE_D
      for(int i = 0;i < nc;i++)
      {
          cf[i] = z[i];
          cf[nc+i] = z[nc+nk*i] - z[nc+nk*i+nk/2];
          cf[nc*2+i] = z[nc+nk*i+1] - z[nc+nk*i+1+nk/2];
      }
#else
      cf = z;
#endif
      for(int i = 0;i < nc;i++){
          if(cf[i] > 0){
            MU(i,1) = (MU(i,0) = sqrt(cf[nc+i]*cf[nc+i] + cf[nc*2+i]*cf[nc*2+i]) / cf[i]);
          } else {
            MU(i,0) = (MU(i,1) = sqrt(-1));
          }

          std::cout << "cf Estimate = [" << cf[i+nc] << " " << cf[i+nc*2] << " " << cf[i] << "]" << std::endl;
          std::cout << "MU_Estimate : " << MU(i,0) << std::endl;
      }
    }

    f_ = f;
    v_ = v;

    return norm_error;
}

