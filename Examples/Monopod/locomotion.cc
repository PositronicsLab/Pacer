#include <monopod.h>
#include <utilities.h>

void ik(const double X[3],double th[3]);

void Monopod::raibert_hopper(const Ravelin::Vector3d& xd_des,double dt,
                    Ravelin::VectorNd& q_des,
                    Ravelin::VectorNd& qd_des,
                    Ravelin::VectorNd& u){

  EndEffector& eef = eefs_[0];
  static Ravelin::Vector3d foot_pos, joint_pos;
  static double t = 0,
                Vz = 0;
  t += dt;

  double AKv = 1e-1;

  Ravelin::Vector3d lin_vel,
                    down = Ravelin::Vector3d(0,0,1,Moby::GLOBAL);
  vel.get_sub_vec(NUM_JOINTS,NUM_JOINTS+3,lin_vel);

  // Set Controller mode
  if(MODE == FLIGHT && NC != 0 && lin_vel[2] < 0){
    //// FOOT CONTACTS GROUND ->
    MODE = LOADING;
  } else if(MODE == LOADING && q_des[2] < 0.97 ){
    //// LEG SHORTENS ->
    MODE = COMPRESSION;
    Vz = qd_des[2];
  } else if(MODE == COMPRESSION && (q_des[2] < 0.50  || qd_des[2] < 0 ) ){
    //// LEG LENGTHENS ->
    MODE = THRUST;
  } else if(MODE == THRUST && ((lin_vel.norm() >= -Vz) || q_des[2] > 1.1 )){
    //// LEG NEAR FULL LENGTH ->
    MODE = UNLOADING;
  } else if(MODE == UNLOADING && NC == 0){
    //// FOOT NOT IN CONTACT ->
    MODE = FLIGHT;
    t = 0;
  }
#ifdef OUTPUT
  std::cout << "CONTROLLER MODE: " << MODE << std::endl;
  std::cout << "Vz0 : "<< Vz << ", |V| : " << lin_vel.norm() << std::endl;
  OUTLOG(lin_vel,"lin_vel");
#endif
  lin_vel[2] = 0;
  lin_vel.pose = Moby::GLOBAL;

  // if the foot hops along these axes:
  // speed will change and heading will remain unaffected: neutral_heading_locus
  // heading will change and speed will remain unaffected: neutral_speed_locus
  Ravelin::Vector3d neutral_heading_locus, neutral_speed_locus = Ravelin::Vector3d::cross(lin_vel,down);
  if(neutral_speed_locus.norm() > sqrt(std::numeric_limits<double>::epsilon())){
    neutral_speed_locus.normalize();
    neutral_heading_locus = Ravelin::Vector3d::cross(down,neutral_speed_locus);
#ifdef OUTPUT
    OUTLOG(neutral_speed_locus,"neutral_speed_locus");
    OUTLOG(neutral_heading_locus,"neutral_heading_locus");
#endif
  }

  // Params of hop
  double duration_of_stance = 0.4,
         k_xd = 5e-2, // useless constant
         duration_of_swing = 0.5;

  Ravelin::Vector3d neutral_foot_position = Ravelin::Vector3d(base_frame->x[0],base_frame->x[1],0,Moby::GLOBAL) +
                                            lin_vel*duration_of_stance/2; // neutral_foot_position

#ifdef VISUALIZE_MOBY
    visualize_ray(neutral_foot_position,Ravelin::Vector3d(0,0,0),Ravelin::Vector3d(0,1,1),sim);
#endif

  // Perform controller action based on mode
  switch(MODE){
  case LOADING:
    //// STOP EXHAUSTING LEG & ZERO HIP TORQUE
    qd_des[0] = 0;
    qd_des[1] = 0;
    break;
  case COMPRESSION:
    //// UPPER LEG CHAMBER SEALED & SERVO BODY ATTITUDE WITH HIP
    qd_des[0] = AKv*roll_pitch_yaw[0]/dt;
    qd_des[1] = AKv*roll_pitch_yaw[1]/dt;
    qd_des[2] = 0;
    break;
  case THRUST:
    //// PRESSURIZE LEG & SERVO BODY ATTITUDE WITH HIP
    qd_des[0] = AKv*roll_pitch_yaw[0]/dt;
    qd_des[1] = AKv*roll_pitch_yaw[1]/dt;
    qd_des[2] = -Vz*1.1;
    q_des[2] += qd_des[2]*dt;
    break;
  case UNLOADING:
    qd_des[0] = 0;
    qd_des[1] = 0;
    qd_des[2] = 0;
    break;
  case FLIGHT:
    //// EXHAUST LEG TO LOW PRESSURE & POSITION LEG FOR LANDING
    Ravelin::Vector3d forward_foot_position = neutral_foot_position +
                                              k_xd*(lin_vel - xd_des);
#ifdef VISUALIZE_MOBY
    visualize_ray(forward_foot_position,Ravelin::Vector3d(0,0,0),Ravelin::Vector3d(0,1,1),sim);
#endif
    foot_pos = base_frame->inverse_transform_point(forward_foot_position);
    ik(foot_pos.data(),joint_pos.data());
    joint_pos[0] = 1.0;

#ifdef OUTPUT
    OUTLOG(foot_pos,"foot_pos");
    OUTLOG(joint_pos,"joint_pos");
#endif
    for(int i=0;i<eef.chain.size();i++){
      if( t< duration_of_swing)
        q_des[eef.chain[i]] = (t/duration_of_swing)*joint_pos[i] + (1 - t/duration_of_swing)*q_des[eef.chain[i]];
      else
        q_des[eef.chain[i]] = joint_pos[i];
      qd_des[eef.chain[i]] = 0;
    }

    break;
  }
}
