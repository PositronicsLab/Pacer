#include <quadruped.h>

using namespace Ravelin;
using namespace Moby;

static Ravelin::VectorNd workv_;
static Ravelin::MatrixNd workM_;

Ravelin::VectorNd& Quadruped::control(const Ravelin::VectorNd& q,
                                      const Ravelin::VectorNd& qd,
                                      Ravelin::VectorNd& q_des,
                                      Ravelin::VectorNd& qd_des,
                                      Ravelin::VectorNd& u){

  unsigned NC = 0;
  for (unsigned i=0; i< NUM_EEFS;i++)
    if(eefs_[i].active)
      NC++;

  static Ravelin::VectorNd uff(NUM_JOINTS),
                           ufb(NUM_JOINTS);
  uff.set_zero();
  ufb.set_zero();
  u.set_zero();

  static Ravelin::VectorNd qdd(NUM_JOINTS);

  for (unsigned m=0,i=0; m< NUM_JOINTS; m++)
  {
     if(joints_[m]->q.size() == 0) continue;
     q_des[i] = q0_[joints_[i]->id];
     qd_des[i] = 0.0;
     i++;
  }

  /// Run friction estimation
  static Ravelin::MatrixNd N(NDOFS,NUM_EEFS),D(NDOFS,NUM_EEFS*NK),M(NDOFS,NDOFS),ST(NDOFS,NUM_EEFS*2);
  static Ravelin::VectorNd fext(NDOFS);

  static Ravelin::VectorNd vel(NDOFS), gc(NDOFS+1), acc(NDOFS);

  // et robot state vectors
  dbrobot_->get_generalized_acceleration(acc);
  dbrobot_->get_generalized_velocity(DynamicBody::eSpatial,vel);
  dbrobot_->get_generalized_coordinates(DynamicBody::eSpatial,gc);

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
  Ravelin::MatrixNd J(3,NDOFS);
  for(int i=0,ii=0;i<NUM_EEFS;i++){
    if(!eefs_[i].active)
      continue;

    boost::shared_ptr<Ravelin::Pose3d> event_frame(new Ravelin::Pose3d(Moby::GLOBAL));
    event_frame->q = Ravelin::Quatd::identity();
    event_frame->x = eefs_[i].point;

    dbrobot_->calc_jacobian(event_frame,eefs_[i].link,workM_);
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

  Ravelin::MatrixNd R(NDOFS, NC + (NC*2) );
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
  Ravelin::VectorNd Hs(NUM_EEFS);
  Ravelin::MatrixNd C(NUM_EEFS,NUM_EEFS);

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
    Ravelin::MatrixNd iJ(3,3);
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
  static Ravelin::MatrixNd Jh;
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
  Ravelin::MatrixNd NULL_Jh = MatrixNd::identity(Jh.rows()),
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

  static Ravelin::VectorNd Y,tY;
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


  static Ravelin::MatrixNd MU;
  MU.set_zero(NC,NK/2);
#ifdef FRICTION_EST
  static Ravelin::VectorNd cf;
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
  static Ravelin::VectorNd ff(NUM_JOINTS);
  ff.set_zero();

  inverse_dynamics(vel,qdd,M,N,D,fext,0.1,MU,ff);
  uff.set_zero();
  uff.set_column(0,ff);
#endif
  ///  Determine FB forces
  control_PID(q_des, qd_des,q,qd,joint_names_, gains_,0,ufb);

  // combine ufb and uff
  u = ufb;
  u += uff;

  // Limit Torques
  for(unsigned m=0;m< NUM_JOINTS;m++){
    if(u[m] > torque_limits_[joints_[m]->id])
      u[m] = torque_limits_[joints_[m]->id];
    else if(u[m] < -torque_limits_[joints_[m]->id])
      u[m] = -torque_limits_[joints_[m]->id];
  }

#ifdef USE_ROBOT
# ifdef CONTROL_KINEMATICS
  Ravelin::VectorNd qdat = q.column(0);
  Ravelin::VectorNd qddat = qd.column(0);
//      dxl_->set_state(qdat.data(),qddat.data());
  dxl_->set_position(qdat.data());
# else
  Ravelin::VectorNd udat = u.column(0);
  dxl_->set_torque(udat.data());
# endif
#endif

#ifdef OUTPUT
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

     // Deactivate all contacts
     NC = 0;
     for(int i=0;i<eefs_.size();i++)
       eefs_[i].active = false;
}

void Quadruped::init(){
  // Set up joint references
  NSPATIAL = 6;
  NEULER = 7;
  unsigned BASE_DOFS  = 6;
  compile();

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

  NUM_JOINTS = joints_.size() - NUM_FIXED_JOINTS;
  NUM_LINKS = links_.size();
  NDOFS = NSPATIAL + NUM_JOINTS; // for generalized velocity, forces. accel

  EndEffector::joint_names_ = joint_names_;
  for(unsigned i=0;i<links_.size();i++)
      for(unsigned j=0;j<eef_names_.size();j++)
          if(eef_names_[j].compare(links_[i]->id) == 0){
            eefs_.push_back(EndEffector(links_[i],eef_origins_[links_[i]->id]));
          }

  NUM_EEFS = eefs_.size();

  std::cout << "NUM_EEFS: " << NUM_EEFS << std::endl;
  std::cout << "N_FIXED_JOINTS: " << NUM_FIXED_JOINTS << std::endl;
  std::cout << "NUM_JOINTS: " << NUM_JOINTS << std::endl;
  std::cout << "NDOFS: " << NDOFS << std::endl;
  std::cout << "NSPATIAL: " << NSPATIAL << std::endl;
  std::cout << "NEULER: " << NEULER << std::endl;
  std::cout << "NK: " << NK << std::endl;

  Ravelin::VectorNd BASE_ORIGIN(7);

  /// LOCALLY SET VALUES
  // robot's initial (ZERO) configuration
  BASE_ORIGIN.set_zero();
//  BASE_ORIGIN[2] = 0.1009;
  BASE_ORIGIN[2] = 0.0922774;
  BASE_ORIGIN[6] = 1;

  q0_["BODY_JOINT"] = 0;
  q0_["LF_HIP_AA"] = 0;
  q0_["LF_HIP_FE"] = M_PI_4;
  q0_["LF_LEG_FE"] = M_PI_2;

  q0_["RF_HIP_AA"] =  0;
  q0_["RF_HIP_FE"] =  M_PI_4;
  q0_["RF_LEG_FE"] =  M_PI_2;

  q0_["LH_HIP_AA"] =  0;
  q0_["LH_HIP_FE"] =  M_PI_4;
  q0_["LH_LEG_FE"] =  M_PI_2;

  q0_["RH_HIP_AA"] =  0;
  q0_["RH_HIP_FE"] =  M_PI_4;
  q0_["RH_LEG_FE"] =  M_PI_2;

  // Maximum torques
  torque_limits_["BODY_JOINT"]=  2.60;
  torque_limits_["LF_HIP_AA"] =  2.60;
  torque_limits_["LF_HIP_FE"] =  2.60;
  torque_limits_["LF_LEG_FE"] =  2.60;

  torque_limits_["RF_HIP_AA"] =  2.60;
  torque_limits_["RF_HIP_FE"] =  2.60;
  torque_limits_["RF_LEG_FE"] =  2.60;

  torque_limits_["LH_HIP_AA"] =  2.60;
  torque_limits_["LH_HIP_FE"] =  6.00;
  torque_limits_["LH_LEG_FE"] =  2.60;

  torque_limits_["RH_HIP_AA"] =  2.60;
  torque_limits_["RH_HIP_FE"] =  6.00;
  torque_limits_["RH_LEG_FE"] =  2.60;

  // Setup gains
  for(unsigned i=0;i<NUM_JOINTS;i++){
    // pass gain values to respective joint
    gains[joints_[i]->id].kp = 1e1;
    gains[joints_[i]->id].kv = 5e-2;
    gains[joints_[i]->id].ki = 0;//1e-3;
  }

  // Set Initial State
  Ravelin::VectorNd q_start(NUM_JOINTS+NEULER),qd_start(NDOFS);

  abrobot_->get_generalized_coordinates(DynamicBody::eEuler,q_start);
  qd_start.set_zero();
if(BASE_DOFS != 0)
  q_start.set_sub_vec(NUM_JOINTS,BASE_ORIGIN);


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
    q_start[i] = q0_[joints_[i]->id];
    qd_start[i] = 0.0;
  }
#endif

  // Push initial state to robot
  abrobot_->set_generalized_coordinates(DynamicBody::eEuler,q_start);
  abrobot_->set_generalized_velocity(DynamicBody::eSpatial,qd_start);
}

