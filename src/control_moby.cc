/*****************************************************************************
 * Controller for LINKS robot
 ****************************************************************************/
#include <quadruped.h>

/// THESE DEFINES DETERMINE WHAT TYPE OF CONTROLLER THIS CODE USES
//    #define NDEBUG
//    #define USE_DUMMY_CONTACTS
//    #define FRICTION_EST
//    #define CONTROL_IDYN

    #define CONTROL_ZMP
    #define RENDER_CONTACT
//    #define USE_ROBOT
//    #define FOOT_TRAJ
//    #define TRUNK_STABILIZATION
//    #define CONTROL_KINEMATICS
//    #define FIXED_BASE

    std::string LOG_TYPE("INFO");
/// END USER DEFINITIONS

#ifdef USE_ROBOT
  #include <dxl/Dynamixel.h>
    Dynamixel* dxl_;
#endif

#define grav 9.8 // M/s.s
#define M_PI_16 0.19634954084
#define M_PI_8 0.39269908169
Vec BASE_ORIGIN(7);

using namespace Moby;
using namespace Ravelin;

// EEF names determines (permanently) the order of EEF data structure
std::vector<std::string> eef_names_;

// Contains eef and contact data
std::vector<EndEffector> eefs_;

static map<string, double> q0,u_max;
static Ravelin::LinAlgd LA_;

// simulator
boost::shared_ptr<Moby::EventDrivenSimulator> sim;
Moby::RCArticulatedBodyPtr abrobot;
Moby::DynamicBodyPtr dbrobot;

std::vector<Moby::JointPtr> joints_;
std::vector<Moby::RigidBodyPtr> links_;

static Vec workv_;
static Mat workM_;

 unsigned NUM_EEFS,
          N_FIXED_JOINTS,
          NUM_JOINTS,
          NUM_LINKS,
#ifdef FIXED_BASE
         NEULER = 0,// for generalized coords
         NSPATIAL = 0,
#else
          NEULER = 7,// for generalized coords
          NSPATIAL = 6,
#endif
          NDOFS, // for generalized velocity, forces. accel
          NK = 4,
          NC = 0;

///////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// PID ///////////////////////////////////////

struct Gains
{
  double kp;
  double kv;
  double ki;
};

// map of gains
map<string, Gains> gains;

// integration error
map<string, double> perr_sum;

template <typename T, typename U>
const U& get(const map<T, U>& m, const T& key)
{
  typename map<T, U>::const_iterator i = m.find(key);
  if (i == m.end())
    throw std::runtime_error("Unexpectedly couldn't find key in map!");
  return i->second;
}

/// Controls the robot
void control_PID(const map<string, double>& q_des,
                 const map<string, double>& qd_des,
                 const map<string, Gains>& gains,
                 double time,Mat& ufb)
{
  // clear and set motor torques
  for (unsigned m=0,i=0; m< NUM_JOINTS; m++)
  {
    // get the joint
    JointPtr j = joints_[m];
    if(j->q.rows() == 0) continue;

    // get the two gains
    const double KP = get(gains,j->id).kp;
    const double KV = get(gains,j->id).kv;
    const double KI = get(gains,j->id).ki;

    // add feedback torque to joints
    double perr = get(q_des,j->id) - j->q[0];
    perr_sum[j->id] += perr;
    double ierr = perr_sum[j->id];
    double derr = get(qd_des,j->id) - j->qd[0];

    Vec fb_torque(1);
    fb_torque[0] = perr*KP + derr*KV + ierr*KI;
    ufb.set_row(i,fb_torque);
    i++;
  }
}

///////////////////////////////////////////////////////////////////////////////
/////////////////////////////// Contact DATA //////////////////////////////////

void calculate_dyn_properties(Mat& M, Vec& fext){
    M.resize(NDOFS,NDOFS);
    fext.resize(NDOFS);
    abrobot->get_generalized_inertia(M);
    abrobot->get_generalized_forces(fext);
}

//////////////////////////////////////////////////////////////////////////////
/////////////////////////////// Controllers //////////////////////////////////

/// Event callback function for processing [contact] events
#ifndef USE_DUMMY_CONTACTS
void post_event_callback_fn(const vector<Event>& e,
                            boost::shared_ptr<void> empty)
{

  // PROCESS CONTACTS
  int nc = e.size();
  for(unsigned i=0;i<e.size();i++){
    if (e[i].event_type == Event::eContact)
    {
#ifdef RENDER_CONTACT
        visualize_contact(e[i],sim);
#endif
      SingleBodyPtr sb1 = e[i].contact_geom1->get_single_body();
      SingleBodyPtr sb2 = e[i].contact_geom2->get_single_body();

      std::vector<std::string>::iterator iter =
          std::find(eef_names_.begin(), eef_names_.end(), sb1->id);

      //if end effector doesnt exist
      if(iter  == eef_names_.end())
        continue;

      size_t index = std::distance(eef_names_.begin(), iter);

      OUTLOG(e[i].contact_impulse.get_linear(),sb1->id);
//      OUTLOG(e[i].contact_impulse.get_angular(),"angular");
      if (eefs_[index].active)
        continue;

      // Increment number of active contacts
      NC++;

      // Push Active contact info to EEF
      eefs_[index].active = true;
      eefs_[index].point = e[i].contact_point;
      eefs_[index].normal = e[i].contact_normal;
      eefs_[index].impulse = e[i].contact_impulse.get_linear();

      eefs_[index].contacted_body = sb2;
    }
  }

  //  SRZ compare contact force prediction to Moby contact force
}
#endif

/// Event callback function for setting friction vars pre-event
void pre_event_callback_fn(vector<Event>& e, boost::shared_ptr<void> empty){}

void apply_simulation_forces(const Mat& u){
    for(unsigned m=0,i=0;m< NUM_JOINTS;m++){
        if(joints_[m]->q.size() == 0) continue;
        // reset motor torque
        Vec row;
        joints_[m]->reset_force();
        joints_[m]->add_force(u.get_row(i,row));
        i++;
    }
}

double calc_energy(Vec& v, Mat& M){
  // Potential Energy
  double PE = 0;
  for(int i=0;i<links_.size();i++){
     RigidBody& link = *links_[i];
     double m = link.get_mass();
     Ravelin::Pose3d link_com = *link.get_inertial_pose();
     link_com.update_relative_pose(Moby::GLOBAL);
     PE += link_com.x[2] * m * grav;
  }
  M.mult(v, workv_);
  double KE = workv_.dot(v)*0.5;
#ifndef NDEBUG
  std::cout << "KE = " << KE << ", PE = " << PE << std::endl;
  std::cout << "Total Energy = " << (KE + PE) << std::endl;
#endif
  return (KE + PE);
  // Kinetic Energy
}

Vector3d& calc_com(Vector3d& weighted_com,Vector3d& com_acc){
  weighted_com.set_zero();
  const std::vector<RigidBodyPtr>& links = abrobot->get_links();
  const SAcceld& base_acc = links[0]->get_accel();
  double total_mass=0;
  for(int i=0;i<links.size();i++){

     RigidBody& link = *links[i];
     double m = link.get_mass();
     total_mass += m;
     Ravelin::Pose3d link_com = *link.get_inertial_pose();
     link_com.update_relative_pose(Moby::GLOBAL);
     weighted_com += (link_com.x *= m);
  }
  weighted_com /= total_mass;

  shared_ptr<Ravelin::Pose3d> base_com_w(new Ravelin::Pose3d());
  base_com_w->x = Ravelin::Origin3d(weighted_com);
  SAcceld com_xdd = Ravelin::Pose3d::transform(base_com_w, base_acc);
  com_acc = com_xdd.get_linear();

  return weighted_com;
}

void get_trajectory(double t,double dt, map<string, double>& q_des,
                    map<string, double>& qd_des, map<string, double>& qdd_des){
}

/// The main control loop

void controller(DynamicBodyPtr dbp, double t, void*)
{
    static int ITER = 0;
    static double last_time = 0;
    static Mat uff(NUM_JOINTS,1),
        ufb(NUM_JOINTS,1),
        u(NUM_JOINTS,1),
        q(NUM_JOINTS,1),
        qd(NUM_JOINTS,1);
    static Vec qdd = Vec::zero(NUM_JOINTS);

    uff.set_zero();
    ufb.set_zero();
    u.set_zero();

    double dt = t - last_time;
    if (dt == 0) return;

    ///  Record Robot State
    for(unsigned m=0;m< NUM_JOINTS;m++){
        if(joints_[m]->q.size() == 0) continue;
        q.set_row(m,joints_[m]->q);
        qd.set_row(m,joints_[m]->qd);
    }

    /// setup a steady state
    static map<string, double> q_des, qd_des, qdd_des;
    if (q_des.empty())
      for (unsigned m=0; m< NUM_JOINTS; m++)
      {
         q_des[joints_[m]->id] = q0[joints_[m]->id];
         qd_des[joints_[m]->id] = 0.0;
         qdd_des[joints_[m]->id] = 0.0;
      }

#ifdef USE_DUMMY_CONTACTS
    NC = eefs_.size();

    for(unsigned i=0;i<NC;i++){
      eefs_[i].active = true;
      Ravelin::Pose3d pose = *eefs_[i].link->get_pose();
      pose.update_relative_pose(Moby::GLOBAL);
      eefs_[i].normal = Ravelin::Vector3d(0,0,1);
      eefs_[i].point = pose.x;
    }
#endif

#ifdef RENDER_CONTACT
     Mat support_poly(3,NC);
     for(int c=0,cc=0;c<NUM_EEFS;c++)
       if(eefs_[c].active){
          support_poly.set_column(cc,eefs_[c].point);
         cc++;
       }

     if(NC > 0)
      visualize_polygon(support_poly,sim);
#endif

     uff.set_zero();
     ufb.set_zero();
     u.set_zero();

      /// Run friction estimation
      static Mat N(NDOFS,NUM_EEFS),D(NDOFS,NUM_EEFS*NK),M(NDOFS,NDOFS),ST(NDOFS,NUM_EEFS*2);
      static Vec fext(NDOFS);

      static Vec vel(NDOFS), gc(NDOFS+1), acc(NDOFS);

      // et robot state vectors
      dbrobot->get_generalized_acceleration(acc);
      dbrobot->get_generalized_velocity(DynamicBody::eSpatial,vel);
      dbrobot->get_generalized_coordinates(DynamicBody::eSpatial,gc);

      // Get base frame
      Ravelin::Pose3d base_frame = *links_[0]->get_pose();
      base_frame.update_relative_pose(Moby::GLOBAL);

#if defined CONTROL_IDYN || defined FRICTION_EST
      // Get robot dynamics state
      // SRZ: Very Heavy Computation
      calculate_dyn_properties(M,fext);
      calc_energy(vel,M);
#endif

      N.set_zero(NDOFS,NC);
      ST.set_zero(NDOFS,NC*2);
      D.set_zero(NDOFS,NC*NK);
      // Contact Jacobian [GLOBAL frame]
      Mat J(3,NDOFS);
      for(int i=0,ii=0;i<NUM_EEFS;i++){
        if(!eefs_[i].active)
          continue;

        shared_ptr<Ravelin::Pose3d> event_frame(new Ravelin::Pose3d(Moby::GLOBAL));
        event_frame->q = Ravelin::Quatd::identity();
        event_frame->x = eefs_[i].point;

        dbrobot->calc_jacobian(event_frame,eefs_[i].link,workM_);
 // THIS WAS THE WRONG WAY OF DOING IT:  dbrobot->calc_jacobian(Moby::GLOBAL,eefs_[i].link,workM_);

        workM_.get_sub_mat(0,3,0,NDOFS,J);

        Vector3d tan1, tan2;
        Vector3d::determine_orthonormal_basis(eefs_[i].normal, tan1, tan2);

        // Normal direction
        J.transpose_mult(eefs_[i].normal,workv_);
        N.set_column(ii,workv_);

        // 1st tangent
        J.transpose_mult(tan1,workv_);
        ST.set_column(ii,workv_);

        D.set_column(ii,workv_);
        workv_.negate();
        D.set_column(NC*2+ii,workv_);

        // 2nd tangent
        J.transpose_mult(tan2,workv_);
        ST.set_column(NC+ii,workv_);
        D.set_column(NC+ii,workv_);
        workv_.negate();
        D.set_column(NC*3+ii,workv_);

        ii++;
      }

      Mat R(NDOFS, NC + (NC*2) );
      R.set_sub_mat(0,0,N);
      R.set_sub_mat(0,NC,ST);

#ifdef FOOT_TRAJ

    // create base Horizontal-frame
    boost::shared_ptr<const Ravelin::Pose3d> base_horizonal_frame(new Ravelin::Pose3d(base_frame));
    // SRZ: convert to base-horizontal frame
//    Ravelin::AAngled = base_frame.
//    base_horizonal_frame.
    // convert jacobian to base horizontal frame

    {
      std::vector<Ravelin::Vector3d> foot_vel(NUM_EEFS),foot_origins(NUM_EEFS),foot_poses(NUM_EEFS);
      Vec Hs(NUM_EEFS);
      Mat C(NUM_EEFS,NUM_EEFS);

      // SRZ: this is the coupling matrix C
      // see: Pattern generators with sensory feedback for the control of quadruped locomotion
      C.set_zero();
      unsigned gait_pattern = 0;
      switch(gait_pattern){
        case 0: // Trotting gait
                       C(0,1) = -1; C(0,2) = -1; C(0,3) =  1;
          C(1,0) = -1;              C(1,2) =  1; C(1,3) = -1;
          C(2,0) = -1; C(2,1) =  1;              C(2,3) = -1;
          C(3,0) =  1; C(3,1) = -1; C(3,2) = -1;
          break;
        case 1: // Pacing gait
                       C(0,1) = -1; C(0,2) =  1; C(0,3) = -1;
          C(1,0) = -1;              C(1,2) = -1; C(1,3) =  1;
          C(2,0) =  1; C(2,1) = -1;              C(2,3) = -1;
          C(3,0) = -1; C(3,1) =  1; C(3,2) = -1;
        break;
        case 2: // Bounding gait
                       C(0,1) =  1; C(0,2) = -1; C(0,3) = -1;
          C(1,0) =  1;              C(1,2) = -1; C(1,3) = -1;
          C(2,0) = -1; C(2,1) = -1;              C(2,3) =  1;
          C(3,0) = -1; C(3,1) = -1; C(3,2) =  1;
        break;
        case 3: // Walking gait
                       C(0,1) = -1; C(0,2) =  1; C(0,3) = -1;
          C(1,0) = -1;              C(1,2) = -1; C(1,3) =  1;
          C(2,0) = -1; C(2,1) =  1;              C(2,3) = -1;
          C(3,0) =  1; C(3,1) = -1; C(3,2) = -1;
        break;
        case 4: // squatting
                       C(0,1) =  1; C(0,2) = -1; C(0,3) =  1;
          C(1,0) =  1;              C(1,2) =  1; C(1,3) = -1;
          C(2,0) = -1; C(2,1) =  1;              C(2,3) =  1;
          C(3,0) =  1; C(3,1) = -1; C(3,2) =  1;
        break;
      }

      double speed = 1;

      // Additional Aprameters for CPG
      double Ls = 0.02,
             Df = 0.55, // where 50% –_–_–_ 100% ======  *Duty factor
                        //           –_–_–_      ======
             Vf = 0.02,
             bp = 100;

      for(int f=0;f<NUM_EEFS;f++){
        // set height of gait (each foot)
        Hs[f] = 0.01;

        // set gait centers
        Ravelin::Pose3d link_pose = *eefs_[f].link->get_pose();
        link_pose.update_relative_pose(base_horizonal_frame);
        foot_origins[f] = eefs_[f].origin;
        foot_poses[f] = link_pose.x;
        foot_vel[f].set_zero();
      }

      // retrieve oscilator value
      foot_oscilator(foot_origins,foot_poses,C,Ls,Hs,Df,Vf,bp,foot_vel);

      // Foot goal position
      for(int f=0;f<NUM_EEFS;f++){
//        foot_vel[f] = Ravelin::Vector3d(0,0,0);
//        foot_poses[f] = foot_origins[f]+Ravelin::Vector3d(0,0,0.02*cos(t*2.0));//(foot_vel[f]*dt*speed);
        foot_poses[f] += (foot_vel[f]*dt*speed);
      }
      std::vector<Ravelin::Vector3d> joint_positions(NUM_EEFS);
      feetIK(foot_poses,joint_positions);

      // Foot goal Velocity

      if(false)
      for(int f=0;f<NUM_EEFS;f++){
        // Calc jacobian for AB at this EEF
        dbrobot->calc_jacobian(boost::shared_ptr<const Ravelin::Pose3d>(new Ravelin::Pose3d(base_frame)),eefs_[f].link,workM_);

        std::vector<unsigned>& joint_inds = eefs_[f].chain;

        // Populate EEF jacobian J:(qd -> xd)
        Mat iJ(3,3);
        for(int i=0;i<joint_inds.size();i++)
          for(int j=0;j<3;j++)
            iJ(j,i) = workM_(j,joint_inds[i]);

//        OUTLOG(iJ,"J");

        // PseudoInverse
        Ravelin::Vector3d foot_velocity = foot_vel[f];
        // (JJ')J
        // SOLVE [inverse] EEF jacobian iJ:(xd -> qd)
        LA_.solve_fast(iJ,foot_vel[f]);

        // Jacobian transpose method
//        iJ.transpose_mult(foot_velocity,foot_vel[f]);

        foot_vel[f] *= dt*speed;

        // Write into qd desired
        for(int i=0;i<joint_inds.size();i++){
          q_des[joints_[joint_inds[i]]->id] = joints_[joint_inds[i]]->q[0] + foot_vel[f][i];
          qd_des[joints_[joint_inds[i]]->id] = foot_vel[f][i];
        }
      }


      // Use Positional (numerical velocity) Control
      for(int i=0;i<NUM_EEFS;i++){
        for(int j=0;j<eefs_[i].chain.size();j++){
          qd_des[joints_[eefs_[i].chain[j]]->id] = (joint_positions[i][eefs_[i].chain.size()-j-1] - joints_[eefs_[i].chain[j]]->q[0])/dt;
          q_des[joints_[eefs_[i].chain[j]]->id] = joint_positions[i][eefs_[i].chain.size()-j-1];
        }
      }
      q_des["LF_HIP_AA"] = 0;
      q_des["RF_HIP_AA"] = 0;
      q_des["LH_HIP_AA"] = 0;
      q_des["RH_HIP_AA"] = 0;
      qd_des["LF_HIP_AA"] = 0;
      qd_des["RF_HIP_AA"] = 0;
      qd_des["LH_HIP_AA"] = 0;
      qd_des["RH_HIP_AA"] = 0;
    }
#endif

#ifdef TRUNK_STABILIZATION
    if(NC>0){
//      std::cout << "NC = " << NC << std::endl;

      OUTLOG(R,"R");
      // Select active rows in Jacobian Matrix
      // R -> Jh
      static std::vector<int> active_dofs;
      active_dofs.clear();
      static Mat Jh;
      for(int i=NUM_JOINTS;i<NDOFS;i++)
        active_dofs.push_back(i);

      for(int i=0;i<NUM_EEFS;i++)
        if(eefs_[i].active)
          for(int j=0;j<eefs_[i].chain.size();j++)
            active_dofs.push_back(eefs_[i].chain[j]);

      std::sort(active_dofs.begin(),active_dofs.end());
      R.select_rows(active_dofs.begin(),active_dofs.end(),Jh);

      OUTLOG(Jh,"Jh");

      // Generate Jh Nullspace
      Mat NULL_Jh = MatrixNd::identity(Jh.rows()),
          Jh_plus;
      workM_ = MatrixNd::identity(Jh.rows());
      Jh.mult_transpose(Jh,Jh_plus);
      try{
        LA_.solve_fast(Jh_plus,workM_);
        Jh.transpose_mult(workM_,Jh_plus);
        OUTLOG(Jh_plus,"Jh' (Jh Jh')^-1");
        NULL_Jh -= Jh.mult(Jh_plus,workM_);
        if(NULL_Jh.norm_inf() > 1e8)
          NULL_Jh.set_zero(Jh.rows(),Jh.rows());
      } catch(Ravelin::SingularException e) {
//        static std::vector<int> pivwork;
//        workM_ = Jh_plus;
//        Jh_plus = Jh;
//        LA_.factor_LU(workM_,pivwork);
//        LA_.solve_LU_fast(workM_,true,pivwork,Jh_plus);
//        Jh_plus.transpose();
//        OUTLOG(Jh_plus,"Jh' (Jh Jh')^-1 (LU decomp)");
        NULL_Jh.set_zero(Jh.rows(),Jh.rows());
      }




      OUTLOG(NULL_Jh,"null(Jh)");

      // Use NULL(Jh) to stabilize trunk w/o altering gait

      // Trunk Stability gains
      double Xp = 0, Xv = 0,
             Yp = 0, Yv = 0,
             Zp = 1e-2, Zv = 0,
             Rp = 1e-2, Rv = 1e-4,
             Pp = 1e-2, Pv = 1e-4;

      static Vec Y,tY;
      Y.set_zero(NC*3+6);

      Y[NC*3+2] = /*(base_horizonal_frame->x[2]-BASE_ORIGIN[2])*Zp +*/ (vel[NUM_JOINTS+2]-0)*Zv;
      Y[NC*3+3] =  (vel[NUM_JOINTS+3]-0)*Rv;
      Y[NC*3+4] =  (vel[NUM_JOINTS+4]-0)*Pv;

      NULL_Jh.mult(Y,tY);

      // apply base stabilization forces
      for(int i=0;i<NC*3;i++)
        uff(active_dofs[i],0) += tY[i];

    }
#endif


      static Mat MU;
      MU.set_zero(NC,NK/2);
#ifdef FRICTION_EST
      static Vec cf;
      double err = friction_estimation(vel,fext,dt,N,D,M,MU,cf);
      OUTLOG(MU,"MU");
      OUTLOG(cf,"contact_forces");
#else
      for(int i=0;i<NC;i++)
        for(int k=0;k<NK/2;k++)
          MU(i,k) = 0.5;
#endif

#ifdef CONTROL_ZMP
      Vector3d CoM,acc_CoM;
      calc_com(CoM,acc_CoM);

      Vector3d ZmP(CoM[0] - 10*(CoM[2]*acc_CoM[0])
                    /(acc_CoM[2]-9.8),
                   CoM[1] - 10*(CoM[2]*acc_CoM[1])
                    /(acc_CoM[2]-9.8),
                   0);

      Vector3d CoM_2D(CoM);
      CoM_2D[2] = 0;
      visualize_ray(CoM_2D,CoM,sim);
      visualize_ray(ZmP,CoM_2D,sim);
#endif
#ifdef CONTROL_IDYN
      for(int i=0;i<NUM_JOINTS; i++)
        qdd[i] = 0;// qdd_des[joints_[i]->id];
      static Vec ff(NUM_JOINTS);
      ff.set_zero();

      inverse_dynamics(vel,qdd,M,N,D,fext,0.1,MU,ff);
      uff.set_zero();
      uff.set_column(0,ff);
#endif
      ///  Determine FB forces
      control_PID(q_des, qd_des, gains,t,ufb);

      // combine ufb and uff
      u = ufb;
      u += uff;

      // Limit Torques
      for(unsigned m=0;m< NUM_JOINTS;m++){
        if(u(m,0) > u_max[joints_[m]->id])
          u(m,0) = u_max[joints_[m]->id];
        else if(u(m,0) < -u_max[joints_[m]->id])
          u(m,0) = -u_max[joints_[m]->id];
      }

#ifdef USE_ROBOT
# ifdef CONTROL_KINEMATICS
      Vec qdat = q.column(0);
      Vec qddat = qd.column(0);
//      dxl_->set_state(qdat.data(),qddat.data());
      dxl_->set_position(qdat.data());
# else
      Vec udat = u.column(0);
      dxl_->set_torque(udat.data());
# endif
#endif

      // send torque commands to robot
      apply_simulation_forces(u);

#ifndef NDEBUG
      std::cout <<  "@ time = "<< t << std::endl;
     std::cout << "JOINT\t: U\t| Q\t: des\t: err\t| "
                  "Qd\t: des\t: err\t|" << std::endl;
     for(unsigned m=0;m< NUM_JOINTS;m++)
       std::cout << joints_[m]->id
                 << "\t " <<  std::setprecision(4) << u(m,0)
                 << "\t| " << joints_[m]->q[0]
                 << "\t " << q_des[joints_[m]->id]
                 << "\t " << q(m,0) - q_des[joints_[m]->id]
                 << "\t| " << joints_[m]->qd[0]
                 << "\t " << qd_des[joints_[m]->id]
                 << "\t " <<  qd(m,0) - qd_des[joints_[m]->id]
                 << "\t| " << std::endl;
     std::cout <<"CoM : "<< CoM << std::endl;
     std::cout <<"ZmP : "<< ZmP << std::endl;
     std::cout << std::endl;
#endif
     last_time = t;

# ifdef CONTROL_KINEMATICS
        for(unsigned i=0;i< NUM_JOINTS;i++){
            if(joints_[i]->q.size() == 0) continue;
            joints_[i]->q[0] = q_des[joints_[i]->id];
        }

       abrobot->update_link_poses();
       abrobot->update_link_velocities();
#endif
       // Deactivate all contacts
       NC = 0;
       for(int i=0;i<eefs_.size();i++)
         eefs_[i].active = false;
       ITER++;
}

void PD_controller(DynamicBodyPtr dbp, double t, void*)
{
  static int ITER = 0;
  static double last_time = 0;
  static Mat u(NUM_JOINTS,1);
  static Vec qdd = Vec::zero(NUM_JOINTS);

  u.set_zero();

  double dt = t - last_time;
  static map<string, double> q_des, qd_des, qdd_des;
  if (q_des.empty())
    for (unsigned m=0; m< NUM_JOINTS; m++)
    {
       q_des[joints_[m]->id] = q0[joints_[m]->id];
       qd_des[joints_[m]->id] = 0.0;
       qdd_des[joints_[m]->id] = 0.0;
    }
  u.set_zero();

  control_PID(q_des, qd_des, gains,t,u);

  // send torque commands to robot
  apply_simulation_forces(u);

 last_time = t;

 ITER++;
}

/// plugin must be "extern C"
extern "C" {

void init(void* separator,
          const std::map<std::string, BasePtr>& read_map,
          double time)
{
  std::cout << LOG_TYPE << std::endl;
  FILELog::ReportingLevel() =
      FILELog::FromString( (!LOG_TYPE.empty()) ? LOG_TYPE : "INFO");

  // get a reference to the EventDrivenSimulator instance
  for (std::map<std::string, BasePtr>::const_iterator i = read_map.begin();
       i !=read_map.end(); i++)
  {
    // Find the simulator reference
    if (!sim)
      sim = dynamic_pointer_cast<EventDrivenSimulator>(i->second);

    // find the robot reference
    if (!abrobot)
    {
      abrobot = dynamic_pointer_cast<RCArticulatedBody>(i->second);
      dbrobot = dynamic_pointer_cast<DynamicBody>(i->second);
    }
  }

  // This will force us to updtae the robot state instead of Moby
#ifdef CONTROL_KINEMATICS
  abrobot->set_kinematic(true);
#endif

  // If using dummy contact ignore impact callback function
#ifndef USE_DUMMY_CONTACTS
  sim->event_post_impulse_callback_fn = &post_event_callback_fn;
#endif

  // setup the controller
  abrobot->controller = &controller;

  // Set up joint references
  std::vector<JointPtr> joints = abrobot->get_joints();
  joints_.resize(joints.size());

  N_FIXED_JOINTS = 0;
  for(unsigned i=0;i<joints.size();i++){
    if(joints[i]->q.rows() == 0){
      N_FIXED_JOINTS ++;
      continue;
    }
    joints_[joints[i]->get_coord_index()] = joints[i];
     std::cout << joints[i]->get_coord_index() << " "
               << joints_[joints[i]->get_coord_index()]->id << std::endl;
  }

  // Set up link references
  links_ = abrobot->get_links();
  for(unsigned i=0;i<NUM_LINKS;i++)
    std::cout << i << " " << links_[i]->id << std::endl;

  // Set up end effectors
  eef_names_.push_back("LF_FOOT");
  eef_names_.push_back("RF_FOOT");
  eef_names_.push_back("LH_FOOT");
  eef_names_.push_back("RH_FOOT");

  // z = -0.1305

  std::map<std::string, Ravelin::Vector3d> eef_origins_;
  eef_origins_["LF_FOOT"] = Ravelin::Vector3d(0.11564, 0.0575, -0.0922774);
  eef_origins_["RF_FOOT"] = Ravelin::Vector3d(0.11564, -0.0575, -0.0922774);
  eef_origins_["LH_FOOT"] = Ravelin::Vector3d(-0.0832403, 0.0575, -0.0922774);
  eef_origins_["RH_FOOT"] = Ravelin::Vector3d(-0.0832403, -0.0575, -0.0922774);

  NUM_JOINTS = joints_.size() - N_FIXED_JOINTS;
  NUM_LINKS = links_.size();
  NDOFS = NSPATIAL + NUM_JOINTS; // for generalized velocity, forces. accel

  for(unsigned i=0;i<links_.size();i++)
      for(unsigned j=0;j<eef_names_.size();j++)
          if(eef_names_[j].compare(links_[i]->id) == 0){
            eefs_.push_back(EndEffector(links_[i],eef_origins_[links_[i]->id]));
          }

  NUM_EEFS = eefs_.size();

  std::cout << "NUM_EEFS: " << NUM_EEFS << std::endl;
  std::cout << "N_FIXED_JOINTS: " << N_FIXED_JOINTS << std::endl;
  std::cout << "NUM_JOINTS: " << NUM_JOINTS << std::endl;
  std::cout << "NDOFS: " << NDOFS << std::endl;
  std::cout << "NSPATIAL: " << NSPATIAL << std::endl;
  std::cout << "NEULER: " << NEULER << std::endl;
  std::cout << "NK: " << NK << std::endl;

  /// LOCALLY SET VALUES
  // robot's initial (ZERO) configuration
  BASE_ORIGIN.set_zero();
  BASE_ORIGIN[2] = 0.1009;
//  BASE_ORIGIN[2] = 0.0922774;
  BASE_ORIGIN[6] = 1;

  q0["BODY_JOINT"] = 0;
  q0["LF_HIP_AA"] = 0;
  q0["LF_HIP_FE"] = M_PI_4;
  q0["LF_LEG_FE"] = M_PI_2;

  q0["RF_HIP_AA"] =  0;
  q0["RF_HIP_FE"] =  M_PI_4;
  q0["RF_LEG_FE"] =  M_PI_2;

  q0["LH_HIP_AA"] =  0;
  q0["LH_HIP_FE"] =  M_PI_4;
  q0["LH_LEG_FE"] =  M_PI_2;

  q0["RH_HIP_AA"] =  0;
  q0["RH_HIP_FE"] =  M_PI_4;
  q0["RH_LEG_FE"] =  M_PI_2;

  // Maximum torques
  u_max["BODY_JOINT"]=  2.60;
  u_max["LF_HIP_AA"] =  2.60;
  u_max["LF_HIP_FE"] =  2.60;
  u_max["LF_LEG_FE"] =  2.60;

  u_max["RF_HIP_AA"] =  2.60;
  u_max["RF_HIP_FE"] =  2.60;
  u_max["RF_LEG_FE"] =  2.60;

  u_max["LH_HIP_AA"] =  2.60;
  u_max["LH_HIP_FE"] =  6.00;
  u_max["LH_LEG_FE"] =  2.60;

  u_max["RH_HIP_AA"] =  2.60;
  u_max["RH_HIP_FE"] =  6.00;
  u_max["RH_LEG_FE"] =  2.60;

  // Setup gains
  for(unsigned i=0;i<NUM_JOINTS;i++){
    // pass gain values to respective joint
    gains[joints_[i]->id].kp = 1e1;
    gains[joints_[i]->id].kv = 5e-2;
    gains[joints_[i]->id].ki = 0;//1e-3;
  }

  // Set Initial State
  Vec q_start(NUM_JOINTS+NEULER),qd_start(NDOFS);

  abrobot->get_generalized_coordinates(DynamicBody::eEuler,q_start);
  qd_start.set_zero();
#ifndef FIXED_BASE
  q_start.set_sub_vec(NUM_JOINTS,BASE_ORIGIN);
#endif

#ifdef FOOT_TRAJ
  std::map<std::string, Ravelin::Vector3d> eef_origin_offset;
  eef_origin_offset["LF_FOOT"] = Ravelin::Vector3d( 0.0, 0.0, 0.0);
  eef_origin_offset["RF_FOOT"] = Ravelin::Vector3d(-0.0, 0.0, 0.0);
  eef_origin_offset["LH_FOOT"] = Ravelin::Vector3d(-0.0, 0.0, 0.0);
  eef_origin_offset["RH_FOOT"] = Ravelin::Vector3d( 0.0, 0.0, 0.0);

  std::vector<Ravelin::Vector3d> foot_origins(NUM_EEFS),joint_positions(NUM_EEFS);
  for(int i=0;i<NUM_EEFS;i++)
    foot_origins[i] = eef_origins_[eef_names_[i]] + eef_origin_offset[eef_names_[i]];
  feetIK(foot_origins,joint_positions);

  for(int i=0;i<NUM_EEFS;i++){
    for(int j=0;j<eefs_[i].chain.size();j++){
      q_start[eefs_[i].chain[j]] = joint_positions[i][eefs_[i].chain.size()-j-1];
      qd_start[eefs_[i].chain[j]] = 0;
    }
  }
#else
  for(int i=0;i<NUM_JOINTS;i++){
    q_start[i] = q0[joints_[i]->id];
    qd_start[i] = 0.0;
  }
#endif

  // Push initial state to robot
  abrobot->set_generalized_coordinates(DynamicBody::eEuler,q_start);
  abrobot->set_generalized_velocity(DynamicBody::eSpatial,qd_start);

  // If use robot is active also init dynamixel controllers
#ifdef USE_ROBOT
  dxl_ = new Dynamixel;
#endif
}

} // end extern C

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

void EndEffector::init(){
  Moby::JointPtr joint_ptr = link->get_inner_joint_explicit();
  Moby::RigidBodyPtr rb_ptr = link;
  while (!rb_ptr->is_base() && rb_ptr->id.compare("THORAX") != 0){
    rb_ptr = joint_ptr->get_inboard_link();
    for(int j=0;j<NUM_JOINTS;j++){
      if(joint_ptr->id.compare(joints_[j]->id) == 0){
        chain.push_back(j);
      }
    }
    joint_ptr = rb_ptr->get_inner_joint_explicit();
  }

  Ravelin::Pose3d pose = *link->get_pose();
  normal = Ravelin::Vector3d(0,0,1);
  point = pose.x;
  active = false;
}
