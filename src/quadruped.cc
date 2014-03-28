#include <quadruped.h>
#include <utilities.h>

using namespace Ravelin;
//using namespace Moby;

const bool FOOT_TRAJ = false,
           WALK = true,
           TRUNK_STABILIZATION = false,
           CONTROL_IDYN = true,
           FRICTION_EST = false,
           CONTROL_ZMP = false,
           PARALLEL_STIFFNESS = false;

extern Ravelin::VectorNd STAGE1, STAGE2;
extern int N_SYSTEMS;
std::vector<std::vector<int> > trot,trot2,walk, walk2;

// TODO: This should be extern double to moby's (nominal) STEP_SIZE
 double STEP_SIZE = 0.01;


Ravelin::VectorNd& Quadruped::control(double t,
                                      const Ravelin::VectorNd& q,
                                      const Ravelin::VectorNd& qd,
                                      Ravelin::VectorNd& q_des,
                                      Ravelin::VectorNd& qd_des,
                                      Ravelin::VectorNd& u){
  static Ravelin::VectorNd qd_last = qd;
  std::cerr << " -- Quadruped::control(.) entered" << std::endl;
  NC = 0;
  for (unsigned i=0; i< NUM_EEFS;i++)
    if(eefs_[i].active)
      NC++;

  ((qdd = qd)-=qd_last)/=STEP_SIZE;
#ifdef COLLECT_DATA   // record all input vars
  {
    // generate a unique filename
    std::ostringstream fname;
    fname << "moby_cf" << (N_SYSTEMS) << ".m";

    // open the file
    std::ofstream out(fname.str().c_str());

    for(unsigned i=0;i< eefs_.size();i++){
      if(!eefs_[i].active){
        out << "cfs_" << i << " = [0/0,0/0,0/0];" << std::endl;
        out << "pts_" << i << " = [0/0,0/0,0/0];" << std::endl;
        continue;
      }
      Ravelin::MatrixNd impulses(eefs_[i].contacts.size(),3),contacts(eefs_[i].contacts.size(),3);
      for(unsigned j=0;j< eefs_[i].contacts.size();j++){
        impulses.set_row(j,eefs_[i].contact_impulses[j]);
        contacts.set_row(j,Ravelin::Pose3d::transform_point(Moby::GLOBAL,eefs_[i].contacts[j]));
      }
      out << "cfs_" << i << " = [\n"<< impulses << "];" << std::endl;
      out << "pts_" << i << " = [\n"<< contacts << "];" << std::endl;
    }

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

  update();

  for(unsigned i=0;i< NUM_JOINTS;i++){
    joints_[i]->q[0]  = q[i];
    joints_[i]->qd[0]  = qd[i];
  }
  abrobot_->update_link_poses();
  abrobot_->update_link_velocities();

  draw_pose(*base_frame,sim);
  draw_pose(Moby::GLOBAL,sim);

  q_des = q;
  qd_des.set_zero();// = qd;

  for(int i=0;i<NUM_EEFS;i++)
    RRMC(eefs_[i],q,eefs_[i].origin,q_des);

  if(NC != 0) {
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
  }

  if(WALK){
    std::vector<std::vector<int> > gait = trot;
    Ravelin::SVector6d go_to(0.20,0,0,0,0,0,base_horizontal_frame);
    double phase_time = 0.25, step_height = 0.01;
    walk_toward(go_to,gait,phase_time,step_height,t,q_des,qd_des,qdd_des);
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
    OUTLOG(id,"STABILIZATION_FORCES",logDEBUG);
    uff += id;
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
  } else
    control_PID(q_des, qd_des,q,qd,joint_names_, gains_,ufb);

  check_finite(ufb);

  if(CONTROL_IDYN){
    double dt = STEP_SIZE;
    double alpha = 1.0;
    Ravelin::VectorNd cf;
    Ravelin::VectorNd id(NUM_JOINTS);
    id.set_zero();
    if(inverse_dynamics(vel,qdd_des,M,N,D,fext,dt,MU,id,cf)){
    // Parallel stiffness controller

      OUT_LOG(logINFO)  << "%foot\tcf = [cN cS cT]\t|| cf = [x y z],\t@\tpt = [x y z]";
      for(int i=0,ii=0;i<NC;i++,ii++){
        while(!eefs_[ii].active) ii++;
        Ravelin::Matrix3d R_foot(             eefs_[ii].normal[0],              eefs_[ii].normal[1],              eefs_[ii].normal[2],
                                 eefs_[ii].event->contact_tan1[0], eefs_[ii].event->contact_tan1[1], eefs_[ii].event->contact_tan1[2],
                                 eefs_[ii].event->contact_tan2[0], eefs_[ii].event->contact_tan2[1], eefs_[ii].event->contact_tan2[2]);
        Ravelin::Origin3d contact_impulse(cf[i]*STEP_SIZE/dt,(cf[i*NK+NC]-cf[i*NK+NC+NK/2])*STEP_SIZE/dt,(cf[i*NK+NC+1]-cf[i*NK+NC+NK/2+1])*STEP_SIZE/dt);
        OUT_LOG(logINFO) <<  std::setprecision(5) << eefs_[ii].id <<"\t" << contact_impulse << "\t|| " << R_foot.transpose_mult(contact_impulse,workv3_) << "\t@  "
                          << eefs_[ii].point;
      }

//      workv_.set_zero(fext.rows());
//      workv_.set_sub_vec(0,id);
//      (workv_ += fext)*=STEP_SIZE;
//      R.mult(cf,workv_,1,1);
//      LA_.solve_fast(M,workv_);
//      OUTLOG(workv_,"qdd == inv(M)(fext*h + R*z + [x; 0]*h)) IDYN");

      OUTLOG(STAGE1,"STAGE1",logDEBUG1);
      OUTLOG(STAGE2,"STAGE2",logDEBUG1);
#ifdef COLLECT_DATA   // record all input vars
    {
      // generate a unique filename
      std::ostringstream fname;
      fname << "idyn_soln" << (N_SYSTEMS) << ".m";

      // open the file
      std::ofstream out(fname.str().c_str());

      out  << "x = " << id << std::endl;
      out  << "x = x';" << std::endl;

      out  << "z = " << cf << std::endl;
      out  << "z = z';" << std::endl;

      for(unsigned i=0,ii=0;i< eefs_.size();i++){
        if(!eefs_[i].active){
          out << "cfs_" << i << " = [0/0,0/0,0/0];" << std::endl;
          out << "pts_" << i << " = [0/0,0/0,0/0];" << std::endl;
          continue;
        }
        Ravelin::Matrix3d R_foot(             eefs_[i].normal[0],              eefs_[i].normal[1],              eefs_[i].normal[2],
                                 eefs_[i].event->contact_tan1[0], eefs_[i].event->contact_tan1[1], eefs_[i].event->contact_tan1[2],
                                 eefs_[i].event->contact_tan2[0], eefs_[i].event->contact_tan2[1], eefs_[i].event->contact_tan2[2]);
        Ravelin::Origin3d contact_impulse(cf[ii]*STEP_SIZE/dt,(cf[ii*NK+NC]-cf[ii*NK+NC+NK/2])*STEP_SIZE/dt,(cf[ii*NK+NC+1]-cf[ii*NK+NC+NK/2+1])*STEP_SIZE/dt);
        out << "cfs_idyn_" << i << " = "<< R_foot.transpose_mult(contact_impulse,workv3_) << "; cfs_idyn_" << i <<"= cfs_idyn_" << i <<"'"<< std::endl;
        out << "pts_idyn_" << i << " = "<< eefs_[i].point << "; pts_idyn_" << i <<"= pts_idyn_" << i <<"'" << std::endl;
        ii++;
      }

      out.close();
    }
#endif
    }
    uff += (id*=alpha);
  }

  check_finite(uff);
  // combine ufb and uff
  u = ufb;
  u += uff;

  // Limit Torques
  for(unsigned i=0;i< NUM_JOINTS;i++){
    joints_[i]->q[0]  = q[i];
    joints_[i]->qd[0]  = qd[i];
  }
  abrobot_->update_link_poses();
  abrobot_->update_link_velocities();

  OUT_LOG(logINFO)<< "NC = " << NC << " @ time = "<< t << std::endl;
     ((workv_ = qd)-=qd_last)/=STEP_SIZE;
     std::cout<<"JOINT\t: U\t| Q\t: des\t: err\t|Qd\t: des\t: err\t|Qdd\t: des\t: err"<<std::endl;
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
     OUTLOG(center_of_mass,"CoM",logINFO);
     OUTLOG(q_des,"q_des",logDEBUG);
     OUTLOG(qd_des,"qd_des",logDEBUG);
     OUTLOG(qdd_des,"qdd_des",logDEBUG);
     OUTLOG(qdd,"qdd",logDEBUG);

     OUTLOG(uff,"uff",logDEBUG);
     OUTLOG(ufb,"ufb",logDEBUG);

#ifdef VISUALIZE_MOBY
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
//           OUTLOG(active_eefs[i].point,active_eefs[i].id);
//           OUTLOG(active_eefs[(i+1)%NC].point,active_eefs[(i+1)%NC].id);
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

       Ravelin::VectorNd g_qd_des(NDOFS),g_qdd_des(NDOFS),
                         xd_des(NUM_EEFS*3),xdd_des(NUM_EEFS*3);
       g_qd_des.set_zero();
       g_qd_des.set_sub_vec(0,qd_des);
       J.transpose_mult(g_qd_des,xd_des);
       g_qdd_des.set_zero();
       g_qdd_des.set_sub_vec(0,qdd);
       J.transpose_mult(g_qdd_des,xdd_des);
       for(int i=0;i<NUM_EEFS;i++){
         Ravelin::Vector3d p = Ravelin::Pose3d::transform_point(Moby::GLOBAL,eefs_[i].origin);
         Ravelin::Vector3d v = Ravelin::Vector3d(xd_des[i+NUM_EEFS],xd_des[i+NUM_EEFS*2],xd_des[i])*STEP_SIZE*100;
         Ravelin::Vector3d a = Ravelin::Vector3d(xdd_des[i+NUM_EEFS],xdd_des[i+NUM_EEFS*2],xdd_des[i])*STEP_SIZE;
         Ravelin::Vector3d pos = Ravelin::Pose3d::transform_point(Moby::GLOBAL,Ravelin::Vector3d(0,0,0,eefs_[i].link->get_pose()));
         visualize_ray( p, pos, Ravelin::Vector3d(0,0,0), sim);
         visualize_ray(v+p, p, Ravelin::Vector3d(1,0,0), sim);
         visualize_ray(a+v+p, v+p, Ravelin::Vector3d(1,0.5,0), sim);
       }
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
//      eef_origins_["LF_FOOT"] = Ravelin::Vector3d(0.12, 0.056278, -0.13);
//      eef_origins_["RF_FOOT"] = Ravelin::Vector3d(0.12, -0.056278, -0.13);
//      eef_origins_["LH_FOOT"] = Ravelin::Vector3d(-0.08, 0.0495, -0.13);
//      eef_origins_["RH_FOOT"] = Ravelin::Vector3d(-0.08, -0.0495, -0.13);
      eef_origins_["LF_FOOT"] = Ravelin::Vector3d( 0.11, 0.096278, -0.13);
      eef_origins_["RF_FOOT"] = Ravelin::Vector3d( 0.11,-0.096278, -0.13);
      eef_origins_["LH_FOOT"] = Ravelin::Vector3d(-0.08, 0.096278, -0.13);
      eef_origins_["RH_FOOT"] = Ravelin::Vector3d(-0.08,-0.096278, -0.13);
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
  OUT_LOG(logINFO)<< NUM_EEFS << " end effectors:" << std::endl;
  for(unsigned j=0;j<NUM_EEFS;j++){
    OUT_LOG(logINFO)<< eefs_[j].id << std::endl;
  }

  NK = 4;

  OUT_LOG(logINFO)<< "NUM_EEFS: " << NUM_EEFS << std::endl;
  OUT_LOG(logINFO)<< "N_FIXED_JOINTS: " << NUM_FIXED_JOINTS << std::endl;
  OUT_LOG(logINFO)<< "NUM_JOINTS: " << NUM_JOINTS << std::endl;
  OUT_LOG(logINFO)<< "NDOFS: " << NDOFS << std::endl;
  OUT_LOG(logINFO)<< "NSPATIAL: " << NSPATIAL << std::endl;
  OUT_LOG(logINFO)<< "NEULER: " << NEULER << std::endl;
  OUT_LOG(logINFO)<< "NK: " << NK << std::endl;

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
//    gains_[joints_[i]->id].kp = 2e0;
//    gains_[joints_[i]->id].kv = 2e-2;
//    gains_[joints_[i]->id].kp = 1e5;
//    gains_[joints_[i]->id].kv = 1e3;
    gains_[joints_[i]->id].ki = 0;
  }

  // Set Initial State
  Ravelin::VectorNd q_start(NUM_JOINTS+NEULER),
                    qd_start(NUM_JOINTS+NSPATIAL);

  abrobot_->get_generalized_coordinates(Moby::DynamicBody::eEuler,q_start);
  qd_start.set_zero();
  qd_start.set_zero();

  for(unsigned i=0;i< NUM_JOINTS;i++)
    q_start[i] = (joints_[i]->q[0]  = q0_[joints_[i]->id]);
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
  OUT_LOG(logINFO)<< std::endl;

  }
}
  // Push initial state to robot


