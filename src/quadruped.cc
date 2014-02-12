#include <quadruped.h>
#include <utilities.h>

using namespace Ravelin;
//using namespace Moby;

static Ravelin::VectorNd workv_;
static Ravelin::MatrixNd workM_;

const bool FOOT_TRAJ = false;
const bool TRUNK_STABILIZATION = false;
const bool CONTROL_IDYN = true;
const bool FRICTION_EST = false;
const bool CONTROL_ZMP = true;
//    #define FIXED_BASE

extern Ravelin::VectorNd STAGE1, STAGE2;
///////////////////////////////////////////////////////////////////////////////
/////////////////////////////// VIZ DATA //////////////////////////////////

extern boost::shared_ptr<Moby::EventDrivenSimulator> sim;
extern void visualize_contact( const Moby::Event& e,
                        boost::shared_ptr<Moby::EventDrivenSimulator> sim );

extern void visualize_polygon( const Ravelin::MatrixNd& verts,
                        boost::shared_ptr<Moby::EventDrivenSimulator> sim );

extern void visualize_ray(   const Ravelin::Vector3d& point, const Ravelin::Vector3d& vec, boost::shared_ptr<Moby::EventDrivenSimulator> sim ) ;

///////////////////////////////////////////////////////////////////////////////


Ravelin::VectorNd& Quadruped::control(double dt,
                                      const Ravelin::VectorNd& q,
                                      const Ravelin::VectorNd& qd,
                                      Ravelin::VectorNd& q_des,
                                      Ravelin::VectorNd& qd_des,
                                      Ravelin::VectorNd& u){
  static double t = 0;
  t += dt;
  NC = 0;
  for (unsigned i=0; i< NUM_EEFS;i++)
    if(eefs_[i].active)
      NC++;

  uff.set_zero(NUM_JOINTS);
  ufb.set_zero(NUM_JOINTS);
  u.set_zero(NUM_JOINTS);

  qdd.set_zero(NUM_JOINTS);
  qd_des.set_zero(NUM_JOINTS);
  q_des.set_zero(NUM_JOINTS);

  std::vector<Ravelin::Vector3d> foot_origins(NUM_EEFS),joint_positions(NUM_EEFS);
  for(int i=0;i<NUM_EEFS;i++)
    foot_origins[i] = eef_origins_[eef_names_[i]];
  feetIK(foot_origins,joint_positions);

  for(int i=0;i<NUM_EEFS;i++){
    int chain_size = eefs_[i].chain.size();
    for(int j=0;j<chain_size;j++){
      q_des[eefs_[i].chain[j]] = joint_positions[i][chain_size-j-1];
      qd_des[eefs_[i].chain[j]] = 0;
    }
  }
  // fetch robot state vectors
  dbrobot_->get_generalized_acceleration(acc);
  dbrobot_->get_generalized_velocity(Moby::DynamicBody::eSpatial,vel);
  dbrobot_->get_generalized_coordinates(Moby::DynamicBody::eSpatial,gc);

  // Get base frame
  base_frame = boost::shared_ptr<Ravelin::Pose3d>( new Ravelin::Pose3d(*links_[0]->get_pose()));
  base_frame->update_relative_pose(Moby::GLOBAL);

  /////////////////////////////////////////////////////////////////////////////

  base_horizonal_frame = boost::shared_ptr<Ravelin::Pose3d>( new Ravelin::Pose3d(base_frame));
  // TODO preserve only YAW in this frame

  if(CONTROL_IDYN || FRICTION_EST){
    // Get robot dynamics state
    // SRZ: Very Heavy Computation
    calculate_dyn_properties(M,fext);
    calc_energy(vel,M);
  }

  calc_contact_jacobians(N,ST,D,R);


  for(int f=0;f<NUM_EEFS;f++){
    // set gait centers
    Ravelin::Pose3d link_pose = *eefs_[f].link->get_pose();
    link_pose.update_relative_pose(boost::shared_ptr<const Ravelin::Pose3d>(
                                     new Ravelin::Pose3d(base_horizonal_frame)));
    std::cout << eefs_[f].id << " " << link_pose.x << std::endl;
  }

  // Kinematically programmed trot
  sinusoidal_trot(q_des,qd_des,qdd,dt);

  // cpg programmed trot
//  cpg_trot(q_des,qd_des,qdd,dt);

  if(TRUNK_STABILIZATION){
    Ravelin::VectorNd id(NUM_JOINTS);
    contact_jacobian_null_stabilizer(R,id);
    uff += id;
  }

  static Ravelin::MatrixNd MU;
  MU.set_zero(NC,NK/2);

  if(FRICTION_EST){
    static Ravelin::VectorNd cf;
    double err = friction_estimation(vel,fext,dt,N,D,M,MU,cf);
    OUTLOG(MU,"MU");
    OUTLOG(cf,"contact_forces");
  } else {
    for(int i=0;i<NC;i++)
      for(int k=0;k<NK/2;k++)
        MU(i,k) = 0.1;
  }

  if(CONTROL_ZMP){
    Vector3d CoM,acc_CoM;
    calc_com(CoM,acc_CoM);

    Vector3d ZmP(CoM[0] - (CoM[2]*acc_CoM[0])/(acc_CoM[2]-9.8),
                 CoM[1] - (CoM[2]*acc_CoM[1])/(acc_CoM[2]-9.8),
                 0);

    Vector3d CoM_2D(CoM);
    CoM_2D[2] = 0;
    visualize_ray(CoM_2D,CoM,sim);
    visualize_ray(ZmP,CoM_2D,sim);
  }

  if(CONTROL_IDYN){
    Ravelin::VectorNd id(NUM_JOINTS);
    inverse_dynamics(vel,qdd,M,N,D,fext,dt,MU,id);
    uff += id;
  }
///  Determine FB forces
  control_PID(q_des, qd_des,q,qd,joint_names_, gains_,ufb);

  // combine ufb and uff
  u = ufb;
//  u += uff;

  // Limit Torques
  for(unsigned m=0;m< NUM_JOINTS;m++){
    if(u[m] > torque_limits_[joints_[m]->id])
      u[m] = torque_limits_[joints_[m]->id];
    else if(u[m] < -torque_limits_[joints_[m]->id])
      u[m] = -torque_limits_[joints_[m]->id];
  }

#ifdef OUTPUT
      std::cout <<  "@ time = "<< t << std::endl;
     std::cout << "JOINT\t: U\t| Q\t: des\t: err\t| "
                  "Qd\t: des\t: err\t|" << std::endl;
     for(unsigned m=0;m< NUM_JOINTS;m++)
       std::cout << joints_[m]->id
                 << "\t " <<  std::setprecision(4) << u[m]
                 << "\t| " << joints_[m]->q[0]
                 << "\t " << q_des[m]
                 << "\t " << q[m] - q_des[m]
                 << "\t| " << joints_[m]->qd[0]
                 << "\t " << qd_des[m]
                 << "\t " <<  qd[m] - qd_des[m]
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
#ifdef FIXED_BASE
  NSPATIAL = 0;
  NEULER = 0;
#else
  NSPATIAL = 6;
  NEULER = 7;
#endif
  compile();

  // Set up end effectors
  eef_names_.push_back("LF_FOOT");
  eef_names_.push_back("RF_FOOT");
  eef_names_.push_back("LH_FOOT");
  eef_names_.push_back("RH_FOOT");

  int num_leg_stance = 4;
  switch(num_leg_stance){
    case 4:
//      eef_origins_["LF_FOOT"] = Ravelin::Vector3d(0.18, 0.0675, -0.13);
//      eef_origins_["RF_FOOT"] = Ravelin::Vector3d(0.18, -0.0675, -0.13);
      eef_origins_["LH_FOOT"] = Ravelin::Vector3d(-0.10, 0.0495, -0.13);
      eef_origins_["RH_FOOT"] = Ravelin::Vector3d(-0.10, -0.0495, -0.13);
      eef_origins_["LF_FOOT"] = Ravelin::Vector3d(0.14, 0.056278, -0.13);
      eef_origins_["RF_FOOT"] = Ravelin::Vector3d(0.14, -0.056278, -0.13);
//      eef_origins_["LH_FOOT"] = Ravelin::Vector3d(-0.06, 0.0675, -0.13);
//      eef_origins_["RH_FOOT"] = Ravelin::Vector3d(-0.06, -0.0675, -0.13);
      break;
    case 3:
      // NOTE THIS IS A STABLE 3-leg stance
      eef_origins_["LF_FOOT"] = Ravelin::Vector3d(0.18, 0.1275, -0.13);
      eef_origins_["RF_FOOT"] = Ravelin::Vector3d(0.14, -0.1075, -0.13);
      eef_origins_["LH_FOOT"] = Ravelin::Vector3d(-0.10, 0.06, -0.13);
      eef_origins_["RH_FOOT"] = Ravelin::Vector3d(-0.06, -0.04, -0.08);
      break;
    case 2:
      // NOTE THIS IS AN UNSTABLE 2-leg stance
      eef_origins_["LF_FOOT"] = Ravelin::Vector3d(0.14, 0.0775, -0.11);
      eef_origins_["RF_FOOT"] = Ravelin::Vector3d(0.14, -0.0775, -0.13);
      eef_origins_["LH_FOOT"] = Ravelin::Vector3d(-0.06, 0.07, -0.13);
      eef_origins_["RH_FOOT"] = Ravelin::Vector3d(-0.06, -0.04, -0.08);
      break;
    default: break;
  }

  NUM_JOINTS = joints_.size() - NUM_FIXED_JOINTS;
  NUM_LINKS = links_.size();
  NDOFS = NSPATIAL + NUM_JOINTS; // for generalized velocity, forces. accel

  for(unsigned j=0;j<eef_names_.size();j++)
    for(unsigned i=0;i<links_.size();i++)
      if(eef_names_[j].compare(links_[i]->id) == 0)
        eefs_.push_back(EndEffector(links_[i],eef_origins_[links_[i]->id],joint_names_));

  NUM_EEFS = eefs_.size();
  std::cout << NUM_EEFS << " end effectors:" << std::endl;
  for(unsigned j=0;j<NUM_EEFS;j++)
    std::cout << eefs_[j].id << std::endl;

  NK = 4;

  std::cout << "NUM_EEFS: " << NUM_EEFS << std::endl;
  std::cout << "N_FIXED_JOINTS: " << NUM_FIXED_JOINTS << std::endl;
  std::cout << "NUM_JOINTS: " << NUM_JOINTS << std::endl;
  std::cout << "NDOFS: " << NDOFS << std::endl;
  std::cout << "NSPATIAL: " << NSPATIAL << std::endl;
  std::cout << "NEULER: " << NEULER << std::endl;
  std::cout << "NK: " << NK << std::endl;

  q0_["BODY_JOINT"] = 0;
  q0_["LF_HIP_AA"] = M_PI_8;
  q0_["LF_HIP_FE"] = M_PI_4;
  q0_["LF_LEG_FE"] = M_PI_2;

  q0_["RF_HIP_AA"] =  -M_PI_8;
  q0_["RF_HIP_FE"] =  -M_PI_4;
  q0_["RF_LEG_FE"] =  -M_PI_2;

  q0_["LH_HIP_AA"] =  -M_PI_8;
  q0_["LH_HIP_FE"] =  -M_PI_4;
  q0_["LH_LEG_FE"] =  -M_PI_2;

  q0_["RH_HIP_AA"] =  M_PI_8;
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
  {
    gains_["BODY_JOINT"].kp=  1e1;
    gains_["LF_HIP_AA"].kp =  10e1;
    gains_["LF_HIP_FE"].kp =  1e1;
    gains_["LF_LEG_FE"].kp =  1e1;
    gains_["RF_HIP_AA"].kp =  10e1;
    gains_["RF_HIP_FE"].kp =  1e1;
    gains_["RF_LEG_FE"].kp =  1e1;
    gains_["LH_HIP_AA"].kp =  10e1;
    gains_["LH_HIP_FE"].kp =  1e1;
    gains_["LH_LEG_FE"].kp =  1e1;
    gains_["RH_HIP_AA"].kp =  10e1;
    gains_["RH_HIP_FE"].kp =  1e1;
    gains_["RH_LEG_FE"].kp =  1e1;

    gains_["BODY_JOINT"].kv =  1e-1;
    gains_["LF_HIP_AA"].kv  =  5e-1;
    gains_["LF_HIP_FE"].kv  =  1e-1;
    gains_["LF_LEG_FE"].kv  =  1e-1;
    gains_["RF_HIP_AA"].kv  =  5e-1;
    gains_["RF_HIP_FE"].kv  =  1e-1;
    gains_["RF_LEG_FE"].kv  =  1e-1;
    gains_["LH_HIP_AA"].kv  =  5e-1;
    gains_["LH_HIP_FE"].kv  =  1e-1;
    gains_["LH_LEG_FE"].kv  =  1e-1;
    gains_["RH_HIP_AA"].kv  =  5e-1;
    gains_["RH_HIP_FE"].kv  =  1e-1;
    gains_["RH_LEG_FE"].kv  =  1e-1;

    gains_["BODY_JOINT"].ki=  0;
    gains_["LF_HIP_AA"].ki =  0;
    gains_["LF_HIP_FE"].ki =  0;
    gains_["LF_LEG_FE"].ki =  0;
    gains_["RF_HIP_AA"].ki =  0;
    gains_["RF_HIP_FE"].ki =  0;
    gains_["RF_LEG_FE"].ki =  0;
    gains_["LH_HIP_AA"].ki =  0;
    gains_["LH_HIP_FE"].ki =  0;
    gains_["LH_LEG_FE"].ki =  0;
    gains_["RH_HIP_AA"].ki =  0;
    gains_["RH_HIP_FE"].ki =  0;
    gains_["RH_LEG_FE"].ki =  0;

    for(int i=0;i<NUM_JOINTS;i++)
      gains_[joints_[i]->id].perr_sum=  0;
  }

  // Set Initial State
  Ravelin::VectorNd q_start(NUM_JOINTS+NEULER),
                    qd_start(NUM_JOINTS+NSPATIAL);

  abrobot_->get_generalized_coordinates(Moby::DynamicBody::eEuler,q_start);
  qd_start.set_zero();

std::vector<Ravelin::Vector3d> foot_origins(NUM_EEFS),joint_positions(NUM_EEFS);
if(FOOT_TRAJ){
  std::map<std::string, Ravelin::Vector3d> eef_origin_offset;
  eef_origin_offset["LF_FOOT"] = Ravelin::Vector3d( 0.01, 0.0, 0.0);
  eef_origin_offset["RF_FOOT"] = Ravelin::Vector3d(-0.01, 0.0, 0.0);
  eef_origin_offset["LH_FOOT"] = Ravelin::Vector3d(-0.01, 0.0, 0.0);
  eef_origin_offset["RH_FOOT"] = Ravelin::Vector3d( 0.01, 0.0, 0.0);

  for(int i=0;i<NUM_EEFS;i++)
    foot_origins[i] = eef_origins_[eef_names_[i]] + eef_origin_offset[eef_names_[i]];
  feetIK(foot_origins,joint_positions);

  for(int i=0;i<NUM_EEFS;i++){
    for(int j=0;j<eefs_[i].chain.size();j++){
      q_start[eefs_[i].chain[j]] = joint_positions[i][j];
      qd_start[eefs_[i].chain[j]] = 0;
    }
  }
} else {
  for(int i=0;i<NUM_EEFS;i++)
    foot_origins[i] = eef_origins_[eef_names_[i]];
  feetIK(foot_origins,joint_positions);

  for(int i=0;i<NUM_EEFS;i++){
    for(int j=0;j<eefs_[i].chain.size();j++){
      q_start[eefs_[i].chain[j]] = joint_positions[i][j];
      qd_start[eefs_[i].chain[j]] = 0;
    }
  }
}
  // Push initial state to robot
  abrobot_->set_generalized_coordinates(Moby::DynamicBody::eEuler,q_start);
  abrobot_->set_generalized_velocity(Moby::DynamicBody::eSpatial,qd_start);

}

