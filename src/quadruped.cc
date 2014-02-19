#include <quadruped.h>
#include <utilities.h>

using namespace Ravelin;
//using namespace Moby;

static Ravelin::VectorNd workv_;
static Ravelin::MatrixNd workM_;

const bool FOOT_TRAJ = true;
const bool TRUNK_STABILIZATION = false;
const bool CONTROL_IDYN = true;
const bool FRICTION_EST = false;
const bool CONTROL_ZMP = true;
//    #define FIXED_BASE
#define VISUALIZE_MOBY
#define OUTPUT

extern Ravelin::VectorNd STAGE1, STAGE2;

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
      q_des[eefs_[i].chain[j]] = joint_positions[i][j];
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

  base_horizonal_frame = boost::shared_ptr<Ravelin::Pose3d>( new Ravelin::Pose3d(Moby::GLOBAL));
  base_horizonal_frame->update_relative_pose(Moby::GLOBAL);
  Ravelin::Matrix3d Rot(base_frame->q);
  bool use_rpy =  R2rpy(Rot,roll_pitch_yaw);
  OUTLOG(roll_pitch_yaw,"roll_pitch_yaw");
  // remove roll and pitch -- preserve yaw
  Rz(roll_pitch_yaw[2],Rot);
  base_horizonal_frame->x = base_frame->x;
  base_horizonal_frame->q = base_frame->q;//Quatd(Rot);


  if(CONTROL_IDYN || FRICTION_EST){
    // Get robot dynamics state
    // SRZ: Very Heavy Computation
    calculate_dyn_properties(M,fext);
    calc_energy(vel,M);
  }

  calc_contact_jacobians(N,ST,D,R);
  center_of_contact.point.set_zero();
  center_of_contact.point.pose = Moby::GLOBAL;
  center_of_contact.normal.pose = Moby::GLOBAL;
  for(int f=0;f<NUM_EEFS;f++){
    // set gait centers
    if(eefs_[f].active){
      center_of_contact.point += eefs_[f].point/NC;
      center_of_contact.normal = eefs_[f].normal;
    }
  }
#ifdef VISUALIZE_MOBY
  visualize_ray(center_of_contact.point,
                center_of_contact.normal + center_of_contact.point,
                Ravelin::Vector3d(1,1,0),
                sim);
#endif
  center_of_contact.point = base_horizonal_frame->inverse_transform_point(center_of_contact.point);
  center_of_contact.normal = base_horizonal_frame->inverse_transform_vector(center_of_contact.normal);

  if(FOOT_TRAJ){
    // Kinematically programmed trot
//    sinusoidal_trot(q_des,qd_des,qdd,dt);

    // cpg programmed trot
    cpg_trot(q_des,qd_des,qdd,dt);
  }

  if(CONTROL_ZMP){
    Vector3d acc_CoM;
    calc_com(center_of_mass,acc_CoM);

    zero_moment_point = Ravelin::Vector3d(center_of_mass[0] - (center_of_mass[2]*acc_CoM[0])/(acc_CoM[2]-9.8),
                 center_of_mass[1] - (center_of_mass[2]*acc_CoM[1])/(acc_CoM[2]-9.8),
                 0);
//    fk_stance_adjustment(dt);
  }

  if(TRUNK_STABILIZATION){
    Ravelin::VectorNd id(NUM_JOINTS);
    id.set_zero();
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
  } else
    for(int i=0;i<NC;i++)
      for(int k=0;k<NK/2;k++)
        MU(i,k) = 0.5;


  if(CONTROL_IDYN){
    Ravelin::VectorNd id(NUM_JOINTS);
    id.set_zero();
    inverse_dynamics(vel,qdd,M,N,D,fext,0.01,MU,id);
    uff += id;
  }
///  Determine FB forces
  control_PID(q_des, qd_des,q,qd,joint_names_, gains_,ufb);

  check_finite(uff);
  check_finite(ufb);
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

#ifdef OUTPUT
      std::cout << "NC = " << NC << " @ time = "<< t << std::endl;
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
//     std::cout <<"CoM : "<< CoM << std::endl;
//     std::cout <<"ZmP : "<< ZmP << std::endl;
     std::cout << std::endl;
#ifdef VISUALIZE_MOBY
     {

       // CONTACTS
       if(NC != 0){
         std::vector<EndEffector> active_eefs;
         if(eefs_[0].active)
           active_eefs.push_back(eefs_[0]);
         if(eefs_[1].active)
           active_eefs.push_back(eefs_[1]);
         if(eefs_[3].active)
           active_eefs.push_back(eefs_[3]);
         if(eefs_[2].active)
           active_eefs.push_back(eefs_[2]);

         for(int i=0;i<NC;i++){
           OUTLOG(active_eefs[i].point,active_eefs[i].id);
           OUTLOG(active_eefs[(i+1)%NC].point,active_eefs[(i+1)%NC].id);
           visualize_ray(active_eefs[i].point,
                         active_eefs[(i+1)%NC].point,
                         Ravelin::Vector3d(1,1,1),
                         sim);
         }
         for(int i=0;i<NC;i++)
           for(int j=0;j<active_eefs[i].contacts.size();j++)
             visualize_ray(active_eefs[i].contacts[j],
                           active_eefs[i].point,
                           Ravelin::Vector3d(1,1,1),
                           sim);
       }

       // ZMP and COM
       Vector3d CoM_2D(center_of_mass);
       CoM_2D[2] = 0;
       visualize_ray(CoM_2D,center_of_mass,Ravelin::Vector3d(0,0,1),sim);
       visualize_ray(zero_moment_point,CoM_2D,Ravelin::Vector3d(0,0,1),sim);

       calc_eef_jacobians(workM_);
       Ravelin::VectorNd g_qd_des(NDOFS);
       g_qd_des.set_zero();
       g_qd_des.set_sub_vec(0,qd_des);
       workM_.transpose_mult(g_qd_des,workv_);
       for(int i=0;i<NUM_EEFS;i++){
         // EEF Velocities
         Ravelin::Pose3d foot_pose = *eefs_[i].link->get_pose();
         foot_pose.update_relative_pose(Moby::GLOBAL);
         visualize_ray(
               Ravelin::Vector3d(workv_[i+NUM_EEFS],workv_[i+NUM_EEFS*2],workv_[i])+Ravelin::Vector3d(foot_pose.x,Moby::GLOBAL),
               Ravelin::Vector3d(foot_pose.x,Moby::GLOBAL),
               Ravelin::Vector3d(1,0,0),
               sim);

         // EEF ORIGINS
         eef_origins_[eefs_[i].id].pose = base_horizonal_frame;
         Ravelin::Vector3d origin = base_horizonal_frame->transform_point(eef_origins_[eefs_[i].id]);
         visualize_ray(
               origin,
               Ravelin::Vector3d(foot_pose.x,Moby::GLOBAL),
               Ravelin::Vector3d(0,0,0),
               sim);
       }

     }
#endif
#endif

     // Deactivate all contacts
     NC = 0;
     for(int i=0;i<eefs_.size();i++)
       eefs_[i].active = false;
     return u;
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
//      eef_origins_["LF_FOOT"] = Ravelin::Vector3d(0.12, 0.056278, -0.13);
//      eef_origins_["RF_FOOT"] = Ravelin::Vector3d(0.12, -0.056278, -0.13);
//      eef_origins_["LH_FOOT"] = Ravelin::Vector3d(-0.08, 0.0495, -0.13);
//      eef_origins_["RH_FOOT"] = Ravelin::Vector3d(-0.08, -0.0495, -0.13);
      eef_origins_["LF_FOOT"] = Ravelin::Vector3d(0.115, 0.076278, -0.13);
      eef_origins_["RF_FOOT"] = Ravelin::Vector3d(0.115, -0.076278, -0.13);
      eef_origins_["LH_FOOT"] = Ravelin::Vector3d(-0.075, 0.076278, -0.13);
      eef_origins_["RH_FOOT"] = Ravelin::Vector3d(-0.075, -0.076278, -0.13);
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
  for(unsigned j=0;j<NUM_EEFS;j++){
    std::cout << eefs_[j].id << std::endl;
  }

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
  for(int i=0;i<NUM_JOINTS;i++){
    gains_[joints_[i]->id].perr_sum = 0;
    gains_[joints_[i]->id].kp = 2e1;
    gains_[joints_[i]->id].kv = 2e-1;
    gains_[joints_[i]->id].ki = 0;
  }

  // Set Initial State
  Ravelin::VectorNd q_start(NUM_JOINTS+NEULER),
                    qd_start(NUM_JOINTS+NSPATIAL);

  abrobot_->get_generalized_coordinates(Moby::DynamicBody::eEuler,q_start);
  qd_start.set_zero();

std::vector<Ravelin::Vector3d> foot_origins(NUM_EEFS),joint_positions(NUM_EEFS);
Ravelin::VectorNd q_des(NUM_JOINTS), qd_des(NUM_JOINTS),qdd(NUM_JOINTS);
if(FOOT_TRAJ){
  sinusoidal_trot(q_des,qd_des,qdd,0);
  q_start.set_sub_vec(0,q_des);
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

