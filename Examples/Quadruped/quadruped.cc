#include <quadruped.h>
#include <utilities.h>

using namespace Ravelin;
//using namespace Moby;


const bool WALK = true,
           TRUNK_STABILIZATION = false,
           CONTROL_IDYN = true,
           FRICTION_EST = false,
           PARALLEL_STIFFNESS = false,
           USE_LAST_CFS = true;

extern Ravelin::VectorNd STAGE1, STAGE2;
extern int N_SYSTEMS;
std::vector<std::vector<int> > trot, walk,gallop;

extern Ravelin::VectorNd perturbation;

// TODO: This should be extern double to moby's (nominal) STEP_SIZE
double STEP_SIZE = 0.01;

extern bool new_sim_step;

Ravelin::VectorNd& Quadruped::control(double t,
                                      const Ravelin::VectorNd& q,
                                      const Ravelin::VectorNd& qd,
                                      Ravelin::VectorNd& q_des,
                                      Ravelin::VectorNd& qd_des,
                                      Ravelin::VectorNd& u){
  static Ravelin::VectorNd qd_last = qd;
  std::cerr << " -- Quadruped::control(.) entered" << std::endl;
  OUT_LOG(logINFO)<< "num_contacts = " << NC ;
  OUT_LOG(logINFO)<< "time = "<< t ;

  update();

  try{
    fext -= perturbation;
  }catch(Ravelin::MissizeException e){

  }

  ((qdd = qd)-=qd_last)/=STEP_SIZE;

#ifdef COLLECT_DATA   // record all input vars
  {
    // generate a unique filename
    std::ostringstream fname;
    fname << "data/true_data" << (N_SYSTEMS) << ".m";

    // open the file
    std::ofstream out(fname.str().c_str());

    out << "contacts = [";
    for(unsigned i=0;i< eefs_.size();i++){
      if(eefs_[i].active)
        out << 1 << " "<< std::endl;
      else
        out << 0 << " " << std::endl;
    }
    out << "];" << std::endl;

    for(unsigned i=0,ii=0;i< eefs_.size();i++){
      if(!eefs_[i].active){
        out << "impulse_" << i << " = [0,0,0];" << std::endl;
        out << "normal_"  << i << " = [0,0,0];" << std::endl;
        out << "point_"   << i << " = [0,0,0];" << std::endl;
        continue;
      }

      out << "impulse_" << i << " = "<< eefs_[i].contact_impulses[0] << ";" << std::endl;
      out << "normal_"  << i << " = "<< eefs_[i].normal << ";" << std::endl;
      out << "point_"   << i << " = "<< eefs_[i].point << ";" << std::endl;
      ii++;
    }

    out <<"impulse_true = [impulse_0' impulse_1' impulse_2' impulse_3'];" << std::endl;
    out <<"normal_true  = [normal_0' normal_1' normal_2' normal_3'];" << std::endl;
    out <<"point_true   = [point_0' point_1' point_2' point_3'];" << std::endl;

    out.close();
  }
#endif
  N_SYSTEMS++;

  uff.set_zero(NUM_JOINTS);
  ufb.set_zero(NUM_JOINTS);
  u.set_zero(NUM_JOINTS);

  qdd_des.set_zero(NUM_JOINTS);
  qd_des.set_zero(NUM_JOINTS);
  q_des.set_zero(NUM_JOINTS);

  for(unsigned i=0;i< NUM_JOINTS;i++){
    joints_[i]->q[0]  = q[i];
    joints_[i]->qd[0]  = qd[i];
  }
  abrobot_->update_link_poses();
  abrobot_->update_link_velocities();

  q_des = q;
  Ravelin::SVector6d go_to(0,0,0,0,0,0,base_frame);;
  Ravelin::VectorNd vb_w(NUM_EEFS*3 + 6);
  std::vector<Ravelin::Vector3d> foot_vel(NUM_EEFS), foot_pos(NUM_EEFS), foot_acc(NUM_EEFS);

  if(WALK){
    double interval_time = 0.1, step_height = 0.01;
    // Assign Gait to the locomotion controller
    std::vector<std::vector<int> > gait = trot;

    static std::vector<Ravelin::Vector3d> footholds;
    // Determine footholds (every 10th of a second)
//    if(((int)(t*1000) % 100) == 0){
    if((int)(t*1000) == 0){
      footholds.clear();
//      find_footholds(footholds,1000);
    }
    go_to = Ravelin::SVector6d(0.1,0,0,0,0,0,base_frame);
    walk_toward(go_to,gait,footholds,interval_time,step_height,t,q,qd,qdd,foot_pos,foot_vel, foot_acc);
    trajectory_ik(foot_pos,foot_vel, foot_acc,q_des,qd_des,qdd_des);
  }
  else {
    for(int i=0;i<NUM_EEFS;i++){
      foot_pos[i] = eefs_[i].origin;
      RRMC(eefs_[i],q,eefs_[i].origin,q_des);
      foot_vel[i].set_zero();
      foot_acc[i].set_zero();
      foot_vel[i].pose = foot_acc[i].pose = foot_pos[i].pose = base_frame;
    }
  }
  workspace_trajectory_goal(go_to,foot_pos,foot_vel,foot_acc,0.05,0.001,vb_w);

  static Ravelin::MatrixNd MU;
  MU.set_zero(NC,NK/2);

  if(FRICTION_EST){
    Ravelin::VectorNd cf;
    double err = friction_estimation(vel,fext,STEP_SIZE,N,D,M,MU,cf);
    OUT_LOG(logINFO)<< "err (friction estimation): " << err << std::endl;
    OUTLOG(MU,"MU",logDEBUG);
    OUTLOG(cf,"contact_forces",logDEBUG);
  } else
    for(int i=0;i<NC;i++)
      for(int k=0;k<NK/2;k++)
        MU(i,k) = 1;

  if(TRUNK_STABILIZATION){
    Ravelin::VectorNd id(NUM_JOINTS);
    Ravelin::MatrixNd J;
    calc_base_jacobian(J);
    zmp_stabilizer(J,Ravelin::Vector2d(0,0),id);
//    qdd_des += id;
  }

  //  Determine FB forces
  if(PARALLEL_STIFFNESS){
    // Determine accuracy of IDYN
    // Apply eef Compliance
    eef_stiffness_fb(q_des, qd_des,q,qd,ufb);
  }
  else
  {
    static std::map<std::string, Gains>      gains;

    // Setup gains
    if(gains.size() == 0)
      for(int i=0;i<NUM_JOINTS;i++){
        gains[joints_[i]->id].perr_sum = 0;
        gains[joints_[i]->id].kp = 1e5;
        gains[joints_[i]->id].kv = 5e2;
        gains[joints_[i]->id].ki = 0;
      }
    PID::control(q_des, qd_des,q,qd,joint_names_, gains,qdd_des);
  }

  if(CONTROL_IDYN){
    double dt = STEP_SIZE;
    double alpha = 1.0;
    Ravelin::VectorNd cf;
    Ravelin::VectorNd id(NUM_JOINTS);
    id.set_zero();
    if(USE_LAST_CFS || NC==0){
      cf.set_zero(NC*5);
      for(unsigned i=0,ii=0;i< eefs_.size();i++){
        if(!eefs_[i].active) continue;
          Ravelin::Matrix3d R_foot(             eefs_[i].normal[0],              eefs_[i].normal[1],              eefs_[i].normal[2],
                                 eefs_[i].event->contact_tan1[0], eefs_[i].event->contact_tan1[1], eefs_[i].event->contact_tan1[2],
                                 eefs_[i].event->contact_tan2[0], eefs_[i].event->contact_tan2[1], eefs_[i].event->contact_tan2[2]);
        Ravelin::Origin3d contact_impulse = Ravelin::Origin3d(R_foot.mult(eefs_[i].contact_impulses[0],workv3_)*(STEP_SIZE/0.001));
        cf[ii] = contact_impulse[0];
        if(contact_impulse[1] > 0)
          cf[NC+ii] = contact_impulse[1];
        else
          cf[NC+ii+NC*2] = contact_impulse[1];
        if(contact_impulse[2] > 0)
          cf[NC+ii+NC] = contact_impulse[2];
        else
          cf[NC+ii+NC*3] = contact_impulse[2];
        ii++;
      }
      Utility::check_finite(cf);
      OUTLOG(cf,"cf z",logERROR);
    }


//      inverse_dynamics(vel,qdd_des,M,N,D,fext,dt,MU,id,cf);
      Rw.mult(vel,vel_w);
      inverse_dynamics(vel_w,vb_w,M,fext,dt,MU,id,cf);

    // Parallel stiffness controller
#ifdef COLLECT_DATA   // record all input vars
    {
      // generate a unique filename
      std::ostringstream fname;
      fname << "data/observed_data" << (N_SYSTEMS) << ".m";

      // open the file
      std::ofstream out(fname.str().c_str());

      out << "contacts = [";
      for(unsigned i=0;i< eefs_.size();i++){
        if(eefs_[i].active)
          out << 1 << " "<< std::endl;
        else
          out << 0 << " " << std::endl;
      }
      out << "];" << std::endl;

      for(unsigned i=0,ii=0;i< eefs_.size();i++){
        if(!eefs_[i].active){
          out << "impulse_" << i << " = [0,0,0];" << std::endl;
          out << "normal_"  << i << " = [0,0,0];" << std::endl;
          out << "point_"   << i << " = [0,0,0];" << std::endl;
          continue;
        }
        Ravelin::Matrix3d R_foot(             eefs_[i].normal[0],              eefs_[i].normal[1],              eefs_[i].normal[2],
                                 eefs_[i].event->contact_tan1[0], eefs_[i].event->contact_tan1[1], eefs_[i].event->contact_tan1[2],
                                 eefs_[i].event->contact_tan2[0], eefs_[i].event->contact_tan2[1], eefs_[i].event->contact_tan2[2]);
        Ravelin::Origin3d contact_impulse(cf[ii],(cf[ii*NK+NC]-cf[ii*NK+NC+NK/2]),(cf[ii*NK+NC+1]-cf[ii*NK+NC+NK/2+1]));
        out << "impulse_" << i << " = "<< R_foot.transpose_mult(contact_impulse,workv3_)/(STEP_SIZE/0.001) << ";" << std::endl;
        out << "normal_"  << i << " = "<< eefs_[i].normal << ";" << std::endl;
        out << "point_"   << i << " = "<< eefs_[i].point << ";" << std::endl;
        ii++;
      }

      out <<"impulse_observed = [impulse_0' impulse_1' impulse_2' impulse_3'];" << std::endl;
      out <<"normal_observed  = [normal_0' normal_1' normal_2' normal_3'];" << std::endl;
      out <<"point_observed   = [point_0' point_1' point_2' point_3'];" << std::endl;

      out.close();
    }
#endif

    OUTLOG(STAGE1,"STAGE1",logDEBUG1);
    OUTLOG(STAGE2,"STAGE2",logDEBUG1);

    uff += (id*=alpha);
  }
  else
  {
    static std::map<std::string, Gains>      gains;

    // Setup gains
    if(gains.size() == 0)
      for(int i=0;i<NUM_JOINTS;i++){
        gains[joints_[i]->id].perr_sum = 0;
        gains[joints_[i]->id].kp = 1e2;
        gains[joints_[i]->id].kv = 1e-1;
        gains[joints_[i]->id].ki = 0;
      }

    PID::control(q_des, qd_des,q,qd,joint_names_, gains,ufb);
  }

  Utility::check_finite(ufb);
  Utility::check_finite(uff);
  // combine ufb and uff
  u = ufb;
  u += uff;

  for(unsigned i=0;i< NUM_JOINTS;i++){
    joints_[i]->q[0]  = q[i];
    joints_[i]->qd[0]  = qd[i];
  }
  abrobot_->update_link_poses();
  abrobot_->update_link_velocities();

     ((workv_ = qd)-=qd_last)/=STEP_SIZE;
     OUT_LOG(logINFO) <<"JOINT\t: U\t| Q\t: des\t: err\t|Qd\t: des\t: err\t|Qdd\t: des\t: err"<<std::endl;
     for(unsigned i=0;i< NUM_JOINTS;i++)
       OUT_LOG(logINFO)<< joints_[i]->id
                 << "\t " <<  std::setprecision(4) << u[i]
                 << "\t| " << joints_[i]->q[0]
                 << "\t " << q_des[i]
                 << "\t " << q[i] - q_des[i]
                 << "\t| " << joints_[i]->qd[0]
                 << "\t " << qd_des[i]
                 << "\t " <<  qd[i] - qd_des[i]
                 << "\t| " << qdd[i]
                 << "\t " << qdd_des[i]
                 << "\t " <<  (qdd[i] - qdd_des[i]);
     OUTLOG(roll_pitch_yaw,"roll_pitch_yaw",logINFO);
     OUTLOG(zero_moment_point,"ZmP",logINFO);
     OUTLOG(center_of_mass_x,"CoM_x",logINFO);
     OUTLOG(center_of_mass_xd,"CoM_xd",logINFO);
     OUTLOG(center_of_mass_xdd,"CoM_xdd",logINFO);
     OUTLOG(q_des,"q_des",logDEBUG);
     OUTLOG(qd_des,"qd_des",logDEBUG);
     OUTLOG(qdd_des,"qdd_des",logDEBUG);
     OUTLOG(q,"q",logDEBUG);
     OUTLOG(qd,"qd",logDEBUG);
     OUTLOG(qdd,"qdd",logDEBUG);
     OUTLOG(fext,"fext",logDEBUG);

     OUTLOG(uff,"uff",logDEBUG);
     OUTLOG(ufb,"ufb",logDEBUG);

#ifdef VISUALIZE_MOBY
     for(unsigned i=0;i< NUM_JOINTS;i++){
       joints_[i]->q[0]  = q[i];
       joints_[i]->qd[0]  = qd[i];
     }
     abrobot_->update_link_poses();
     abrobot_->update_link_velocities();
/*
       for(int i=0;i<NUM_EEFS;i++){
         Ravelin::Vector3d pos = Ravelin::Pose3d::transform_point(Moby::GLOBAL,Ravelin::Vector3d(0,0,0,eefs_[i].link->get_pose()));

         EndEffector& foot = eefs_[i];
         // Calc jacobian for AB at this EEF
         Ravelin::MatrixNd J;
         Ravelin::Origin3d x,xd,xdd;
         for(int k=0;k<foot.chain.size();k++){                // actuated joints
           OUT_LOG(logINFO)<< joints_[foot.chain[k]]->id;
           x[k] = q_des[foot.chain[k]];
           xd[k] = qd_des[foot.chain[k]];
           xdd[k] = qdd_des[foot.chain[k]];
         }
         OUTLOG(x,foot.id + "_q",logINFO);
         OUTLOG(xd,foot.id + "_qd",logINFO);
         OUTLOG(xdd,foot.id + "_qdd",logINFO);
         foot_jacobian(x,foot,J);

         Ravelin::Vector3d p = Ravelin::Pose3d::transform_point(Moby::GLOBAL,Ravelin::Vector3d(0,0,0,eefs_[i].link->get_pose()));
         OUTLOG(p,foot.id + "_x",logINFO);
         visualize_ray( p, pos, Ravelin::Vector3d(0,0,0), sim);

         J.mult((workv3_ = xd), xd) *= sqrt(STEP_SIZE);
         xd = Ravelin::Pose3d::transform_vector(Moby::GLOBAL,Ravelin::Vector3d(xd,base_frame));
         OUTLOG(xd,foot.id + "_xd",logINFO);
         visualize_ray(xd+p, p, Ravelin::Vector3d(1,0,0), sim);

         J.mult((workv3_ = xdd), xdd) *= STEP_SIZE;
         OUTLOG(xdd,eefs_[i].id + "_xdd",logINFO);
         visualize_ray(xdd+xd+p, xd+p, Ravelin::Vector3d(1,0.5,0), sim);
       }
*/
       for(unsigned i=0;i< NUM_JOINTS;i++){
         joints_[i]->q[0]  = q[i];
         joints_[i]->qd[0]  = qd[i];
       }
       abrobot_->update_link_poses();
       abrobot_->update_link_velocities();
#endif
     // Deactivate all contacts
     NC = 0;
     for(int i=0;i<eefs_.size();i++)
       eefs_[i].active = false;

     qd_last = qd;
     std::cerr << " -- Quadruped::control(.) exited" << std::endl;

     return u;
}

#include <boost/assign/std/vector.hpp>
#include <boost/assign/list_of.hpp>

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
      eef_origins_["LF_FOOT"] = Ravelin::Vector3d( 0.12, 0.096278, -0.13);
      eef_origins_["RF_FOOT"] = Ravelin::Vector3d( 0.12,-0.096278, -0.13);
      eef_origins_["LH_FOOT"] = Ravelin::Vector3d(-0.07, 0.096278, -0.13);
      eef_origins_["RH_FOOT"] = Ravelin::Vector3d(-0.07,-0.096278, -0.13);
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

  OUT_LOG(logINFO)<< eef_names_.size() << " end effectors LISTED:" ;
  for(unsigned j=0;j<eef_names_.size();j++){
    for(unsigned i=0;i<links_.size();i++){
      if(eef_names_[j].compare(links_[i]->id) == 0){
        OUT_LOG(logINFO)<< eef_names_[j] << " FOUND!";
        eefs_.push_back(EndEffector(links_[i],eef_origins_[links_[i]->id],joint_names_));
        break;
      }
    }
  }

  NUM_EEFS = eefs_.size();
  OUT_LOG(logINFO)<< NUM_EEFS << " end effectors:" ;
  for(unsigned j=0;j<NUM_EEFS;j++){
    OUT_LOG(logINFO)<< eefs_[j].id ;
  }

  NK = 4;

  OUT_LOG(logINFO)<< "NUM_EEFS: " << NUM_EEFS ;
  OUT_LOG(logINFO)<< "N_FIXED_JOINTS: " << NUM_FIXED_JOINTS ;
  OUT_LOG(logINFO)<< "NUM_JOINTS: " << NUM_JOINTS ;
  OUT_LOG(logINFO)<< "NDOFS: " << NDOFS ;
  OUT_LOG(logINFO)<< "NSPATIAL: " << NSPATIAL ;
  OUT_LOG(logINFO)<< "NEULER: " << NEULER ;
  OUT_LOG(logINFO)<< "NK: " << NK ;

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
  std::map<std::string, double> torque_limits_;
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

  // push into robot
  torque_limits_l.resize(NUM_JOINTS);
  torque_limits_u.resize(NUM_JOINTS);
  for(int i=0;i<NUM_JOINTS;i++){
    torque_limits_l[i] = -torque_limits_[joints_[i]->id];
    torque_limits_u[i] = torque_limits_[joints_[i]->id];
  }

  // Set Initial State
  Ravelin::VectorNd q_start(NUM_JOINTS+NEULER),
                    qd_start(NUM_JOINTS+NSPATIAL);

  abrobot_->get_generalized_coordinates(Moby::DynamicBody::eEuler,q_start);
  qd_start.set_zero();
  qd_start.set_zero();
  OUTLOG(q_start,"q_start",logINFO);

  for(unsigned i=0;i< NUM_JOINTS;i++)
    q_start[i] = (joints_[i]->q[0]  = q0_[joints_[i]->id]);
  OUTLOG(q_start,"q_start",logINFO);
  abrobot_->update_link_poses();
  update();

  for(int i=0;i<NUM_EEFS;i++){
    RRMC(eefs_[i],Ravelin::VectorNd(q_start),eefs_[i].origin,q_start);
    for(int j=0;j<eefs_[i].chain.size();j++){
      (joints_[eefs_[i].chain[j]]->q[0] = q_start[eefs_[i].chain[j]]);
      qd_start[eefs_[i].chain[j]] = 0;
    }
  }
  abrobot_->update_link_poses();

  {
    std::vector<int> step;

    // Trotting gait 50/50 duty cycle
    step = boost::assign::list_of(-1)(1)(1)(-1);
    trot.push_back(step);
    step = boost::assign::list_of(1)(-1)(-1)(1);
    trot.push_back(step);

    // walk lf,rf,lh,rh
    step = boost::assign::list_of(-1)(-3)(1)(-2);
    walk.push_back(step);
    step = boost::assign::list_of(1)(-2)(-3)(-1);
    walk.push_back(step);
    step = boost::assign::list_of(-3)(-1)(-2)(1);
    walk.push_back(step);
    step = boost::assign::list_of(-2)(1)(-1)(-3);
    walk.push_back(step);

    // transverse gallop
    step = boost::assign::list_of(1)(2)(3)(4);
    gallop.push_back(step);

    // Rotary gallop

  }

  environment_frame = boost::shared_ptr<Ravelin::Pose3d>( new Ravelin::Pose3d(Moby::GLOBAL));
  environment_frame->x = Ravelin::Origin3d(0,0,0);
  environment_frame->q.set_identity();
}
  // Push initial state to robot


