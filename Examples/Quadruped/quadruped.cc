#include <quadruped.h>
#include <utilities.h>

#include <random>
std::default_random_engine generator;

using namespace Ravelin;
//using namespace Moby;


const bool WALK = false,
           TRUNK_STABILIZATION = false,
           CONTROL_IDYN = true,
           FRICTION_EST = false,
           PARALLEL_STIFFNESS = false;

extern Ravelin::VectorNd STAGE1, STAGE2;
extern int N_SYSTEMS;
std::vector<std::vector<int> > trot,trot2,walk, walk2;

extern Ravelin::VectorNd perturbation;

// TODO: This should be extern double to moby's (nominal) STEP_SIZE
double STEP_SIZE = 0.001;

extern bool new_sim_step;

Ravelin::VectorNd& Quadruped::control(double t,
                                      const Ravelin::VectorNd& q,
                                      const Ravelin::VectorNd& qd,
                                      Ravelin::VectorNd& q_des,
                                      Ravelin::VectorNd& qd_des,
                                      Ravelin::VectorNd& u){
  static Ravelin::VectorNd qd_last = qd;
  std::cerr << " -- Quadruped::control(.) entered" << std::endl;

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

  static std::normal_distribution<double> distribution_normal(0,0.1);
  static std::normal_distribution<double> distribution_point(0,0.01);

  for(int i=0;i<=0;i++){
    // PERTURB NORMAL
    eefs_[i].normal += Ravelin::Vector3d(distribution_normal(generator),distribution_normal(generator),0);
    eefs_[i].normal.normalize();
    // PERTURB POINT
    eefs_[i].point += Ravelin::Vector3d(distribution_point(generator),distribution_point(generator),distribution_point(generator));
    visualize_ray(eefs_[i].point+eefs_[i].normal,eefs_[i].point,Ravelin::Vector3d(1,1,0),sim);
  }

  update();

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
  if(WALK){
    std::vector<std::vector<int> > gait = trot;
    Ravelin::SVector6d go_to(0.1,0,0,0,0,0,base_horizontal_frame);
    double phase_time = 0.1, step_height = 0.01;
    walk_toward(go_to,gait,phase_time,step_height,t,q_des,qd_des,qdd_des);

    if(TRUNK_STABILIZATION){
      double alpha = 1.0;

      Ravelin::VectorNd id(NUM_JOINTS);
      id.set_zero();
      Ravelin::SVector6d vel_base = go_to,
                         pos_base(0,0,0.135,0,0,0);
      contact_jacobian_null_stabilizer(R,pos_base,vel_base,id);
      OUTLOG(id,"STABILIZATION_FORCES",logDEBUG);
      ufb += (id *= alpha);
    }
  } else {
    for(int i=0;i<NUM_EEFS;i++)
      RRMC(eefs_[i],q,eefs_[i].origin,q_des);
  }

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


  //  Determine FB forces
  if(PARALLEL_STIFFNESS){
    // Determine accuracy of IDYN
    // Apply eef Compliance
    eef_stiffness_fb(q_des, qd_des,q,qd,ufb);
  }
  else{
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
    Ravelin::VectorNd cf(NC*5);
    Ravelin::VectorNd id(NUM_JOINTS);
    id.set_zero();
    cf.set_zero();
    if(NC>0)
      inverse_dynamics(vel,qdd_des,M,N,D,fext,dt,MU,id,cf);
    else{
      // do simple inverse dynamics
      M.get_sub_mat(0,NUM_JOINTS,0,NUM_JOINTS,workM_).mult(qdd_des,id) -= fext.get_sub_vec(0,NUM_JOINTS,workv_);
    }

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

    for(int i=0,ii=0;ii<NUM_EEFS;ii++){
      Ravelin::Origin3d contact_impulse(0,0,0);
      if(eefs_[ii].active){
        Ravelin::Matrix3d R_foot(             eefs_[ii].normal[0],              eefs_[ii].normal[1],              eefs_[ii].normal[2],
                                 eefs_[ii].event->contact_tan1[0], eefs_[ii].event->contact_tan1[1], eefs_[ii].event->contact_tan1[2],
                                 eefs_[ii].event->contact_tan2[0], eefs_[ii].event->contact_tan2[1], eefs_[ii].event->contact_tan2[2]);
        contact_impulse = Ravelin::Origin3d(cf[i],(cf[i*NK+NC]-cf[i*NK+NC+NK/2]),(cf[i*NK+NC+1]-cf[i*NK+NC+NK/2+1]));
        OUTLOG(R_foot.transpose_mult(contact_impulse,workv3_),"cf",logINFO);
        i++;
      } else {
        OUTLOG(contact_impulse,"cf",logINFO);
      }
    }

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

  OUT_LOG(logINFO)<< "num_contacts = " << NC ;
  OUT_LOG(logINFO)<< "time = "<< t ;
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
      joints_[eefs_[i].chain[j]]->q[0] = q_start[eefs_[i].chain[j]];
      qd_start[eefs_[i].chain[j]] = 0;
    }
  }
  abrobot_->update_link_poses();

  {
    std::vector<int> step;
    step.push_back(-1);
    step.push_back(1);
    step.push_back(1);
    step.push_back(-1);
    trot.push_back(step);
    step.clear();
    step.push_back(1);
    step.push_back(-1);
    step.push_back(-1);
    step.push_back(1);
    trot.push_back(step);
  }

  {
    std::vector<int> step;
    step.push_back(-3);
    step.push_back(-1);
    step.push_back(-1);
    step.push_back(-3);
    trot2.push_back(step);
    step.clear();
    step.push_back(-2);
    step.push_back(1);
    step.push_back(1);
    step.push_back(-2);
    trot2.push_back(step);
    step.clear();
    step.push_back(-1);
    step.push_back(-3);
    step.push_back(-3);
    step.push_back(-1);
    trot2.push_back(step);
    step.clear();
    step.push_back(1);
    step.push_back(-2);
    step.push_back(-2);
    step.push_back(1);
    trot2.push_back(step);
  }

  {
    std::vector<int> step;
    step.push_back(1);
    step.push_back(-1);
    step.push_back(-2);
    step.push_back(-3);
    walk.push_back(step);
    step.clear();
    step.push_back(-3);
    step.push_back(1);
    step.push_back(-1);
    step.push_back(-2);
    walk.push_back(step);
    step.clear();
    step.push_back(-2);
    step.push_back(-3);
    step.push_back(1);
    step.push_back(-1);
    walk.push_back(step);
    step.clear();
    step.push_back(-1);
    step.push_back(-2);
    step.push_back(-3);
    step.push_back(1);
    walk.push_back(step);
  }

  {
    std::vector<int> step;
    step.push_back(1);
    step.push_back(-2);
    step.push_back(-3);
    step.push_back(-1);
    walk2.push_back(step);
    step.clear();
    step.push_back(-3);
    step.push_back(-1);
    step.push_back(-2);
    step.push_back(1);
    walk2.push_back(step);
    step.clear();
    step.push_back(-2);
    step.push_back(1);
    step.push_back(-1);
    step.push_back(-3);
    walk2.push_back(step);
    step.clear();
    step.push_back(-1);
    step.push_back(-3);
    step.push_back(1);
    step.push_back(-2);
    walk2.push_back(step);
  }
  for(int i=0;i<trot.size();i++){
    for(int j=0;j<trot[i].size();j++)
      OUT_LOG(logINFO)<< trot[i][j] << " ";
  OUT_LOG(logINFO);

  }
}
  // Push initial state to robot


