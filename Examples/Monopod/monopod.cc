#include <monopod.h>
#include <utilities.h>

using namespace Ravelin;
//using namespace Moby;

const bool CONTROL_IDYN = false;
const bool FRICTION_EST = false;

extern bool new_sim_step;

Ravelin::VectorNd& Monopod::control(double dt,
                                      const Ravelin::VectorNd& q,
                                      const Ravelin::VectorNd& qd,
                                      Ravelin::VectorNd& q_des,
                                      Ravelin::VectorNd& qd_des,
                                      Ravelin::VectorNd& u){
  static double t = 0;
  t += dt;

  static Ravelin::VectorNd last_q_des,last_qd_des;

  if(!new_sim_step){
//    u.set_zero(NUM_JOINTS);
//    control_PID(last_q_des, last_qd_des,q,qd,joint_names_, gains_,u);
//    return u;
  }
  NC = 0;
  for (unsigned i=0; i< NUM_EEFS;i++)
    if(eefs_[i].active)
      NC++;

  qdd.set_zero(NUM_JOINTS);
  qd_des.set_zero(NUM_JOINTS);
  q_des.set_zero(NUM_JOINTS);

  for(unsigned i=0;i< NUM_JOINTS;i++){
    q_des[i] = q[i];
    qd_des[i] = qd[i];
  }
  // UPDATE ROBOT VARIABLES
  if(new_sim_step){
//    std::cout << "NC = " << NC << " @ time = "<< t << std::endl;
    update();
  }

  /////////////////////////////////////////////////////////////////////////////
  //////////////////////////// Run Various Controllers ////////////////////////

  Ravelin::Vector3d xd_des(0,0,0,Moby::GLOBAL);
  raibert_hopper(xd_des,dt,q_des,qd_des,ufb);

  ///  Determine FB forces
  last_q_des = q_des;
  last_qd_des = qd_des;
  PID::control(q_des, qd_des,q,qd,joint_names_, gains_,u);

  Utility::check_finite(u);
  // combine ufb and uff

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
     std::cout <<"CoM : "<< center_of_mass << std::endl;
     std::cout <<"ZmP : "<< zero_moment_point << std::endl;
     OUTLOG(J.transpose(),"J");
     J.transpose();
     std::cout << std::endl;
     for(int i=0;i<NUM_EEFS;i++){
       // EEF Velocities
       Ravelin::Pose3d foot_pose = *eefs_[i].link->get_pose();
       foot_pose.update_relative_pose(Moby::GLOBAL);
       std::cout << eefs_[i].id << foot_pose.x << std::endl;
     }
     std::cout << "KE = " << KE << ", PE = " << PE << std::endl;
     std::cout << "Total Energy = " << (KE + PE) << std::endl;
     OUTLOG(roll_pitch_yaw,"roll_pitch_yaw");
#endif
#ifdef VISUALIZE_MOBY
     {

       // CONTACTS
       if(NC != 0){
         std::vector<EndEffector> active_eefs;
         if(eefs_[0].active)
           active_eefs.push_back(eefs_[0]);

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

       Ravelin::VectorNd g_qd_des(NDOFS);
       g_qd_des.set_zero();
       g_qd_des.set_sub_vec(0,qd_des);
       J.transpose_mult(g_qd_des,workv_);
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
     // Deactivate all contacts
     NC = 0;
     for(int i=0;i<eefs_.size();i++)
       eefs_[i].active = false;

     return u;
}

void Monopod::init(){
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
  eef_names_.push_back("FOOT");

  eef_origins_["FOOT"] = Ravelin::Vector3d(0,0,-0.75);

  NUM_JOINTS = joints_.size() - NUM_FIXED_JOINTS;
  NUM_LINKS = links_.size();
  NDOFS = NSPATIAL + NUM_JOINTS; // for generalized velocity, forces. accel

  for(unsigned j=0;j<eef_names_.size();j++)
    for(unsigned i=0;i<links_.size();i++)
      if(eef_names_[j].compare(links_[i]->id) == 0)
        eefs_.push_back(EndEffector(links_[i],eef_origins_[links_[i]->id],joint_names_));

  NUM_EEFS = eefs_.size();

  NK = 4;
#ifdef OUTPUT
  std::cout << NUM_EEFS << " end effectors:" << std::endl;
  for(unsigned j=0;j<NUM_EEFS;j++){
    std::cout << eefs_[j].id << std::endl;
  }

  std::cout << "NUM_EEFS: " << NUM_EEFS << std::endl;
  std::cout << "N_FIXED_JOINTS: " << NUM_FIXED_JOINTS << std::endl;
  std::cout << "NUM_JOINTS: " << NUM_JOINTS << std::endl;
  std::cout << "NDOFS: " << NDOFS << std::endl;
  std::cout << "NSPATIAL: " << NSPATIAL << std::endl;
  std::cout << "NEULER: " << NEULER << std::endl;
  std::cout << "NK: " << NK << std::endl;
#endif

  q0_["ROLL_JOINT"] = 0;
  q0_["PITCH_JOINT"] = M_PI_2;
  q0_["PISTON_JOINT"] = 0;

  torque_limits_["ROLL_JOINT"] = 1000;
  torque_limits_["PITCH_JOINT"] = 1000;
  torque_limits_["PISTON_JOINT"] = 1000;
  // Maximum torques

  // Setup gains
  for(int i=0;i<NUM_JOINTS;i++){
    gains_[joints_[i]->id].perr_sum = 0;
    gains_[joints_[i]->id].ki = 0;
  }
  gains_["PISTON_JOINT"].kp = 2e4;
  gains_["PISTON_JOINT"].kv = 2e2;
  gains_["ROLL_JOINT"].kp = 2e4;
  gains_["ROLL_JOINT"].kv = 5e1;
  gains_["PITCH_JOINT"].kp = 2e4;
  gains_["PITCH_JOINT"].kv = 5e1;

  Ravelin::VectorNd q_start(NUM_JOINTS+NEULER);
  dbrobot_->get_generalized_coordinates(Moby::DynamicBody::eSpatial,q_start);
  q_start[0] = 0;
  q_start[1] = M_PI_2;
  q_start[2] = 1.0;
  dbrobot_->set_generalized_coordinates(Moby::DynamicBody::eSpatial,q_start);

  MODE = FLIGHT;
}

