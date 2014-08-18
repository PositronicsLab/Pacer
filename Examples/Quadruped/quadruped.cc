#include <quadruped.h>
#include <utilities.h>
// -----------------------------------------------------------------------------
using namespace Ravelin;
//using namespace Moby;

extern Ravelin::VectorNd STAGE1, STAGE2;
extern int N_SYSTEMS;
double SIMULATION_TIME;
std::map<std::string , std::vector<double> > gait;

// TODO: This should be extern double to moby's (nominal) STEP_SIZE

extern bool new_sim_step;

// ============================================================================
// ======================= USER DEFINED PARAMETERS ============================

#ifdef VISUALIZE_MOBY
# include <OpenGL/gl.h>
# include <OpenGL/glu.h>
# include <GLUT/glut.h>
GLConsole theConsole;
#endif

 bool
        WALK                = false,   //  "Activate Walking?"
          TRACK_FOOTHOLDS     = false,//    EXPERIMENTAL -- "Locate and use footholds?"
        CONTROL_IDYN        = false,   //  "Activate IDYN?"
          WORKSPACE_IDYN      = false,//    EXPERIMENTAL -- "Activate WIDYN?"
          USE_LAST_CFS        = false,//    EXPERIMENTAL -- "Use last detected contact forces?"
        FRICTION_EST        = false,  //  EXPERIMENTAL
        TRUNK_STABILIZATION = false,   //  Balance Pitch (D) and Roll (PD) or robot base with compressive forces
        ERROR_FEEDBACK      = false,   //  "Use error-feedback control?"
          FEEDBACK_ACCEL      = false, //    "Apply error-feedback as accelerations?"
          JOINT_FEEDBACK      = false, //    "Apply error-feedback as forces?"
          WORKSPACE_FEEDBACK  = false;//    "Use error-feedback in workspace frame?"

// -- LOCOMOTION OPTIONS --
double
        gait_time   = 0.0,//,"Gait Duration over one cycle."),
        step_height = 0.0,//,""),
        goto_X      = 0.0,//,"command forward direction"),
        goto_Y      = 0.0,//,"command lateral direction"),
        goto_GAMMA  = 0.0,//,"command rotation");
        SIM_MU_COULOMB = 0,
        SIM_MU_VISCOSE = 0,
        SIM_PENALTY_KV = 0,
        SIM_PENALTY_KP = 0;

// Assign Gait to the locomotion controller
std::string
        gait_type   = "trot"; //,"Gait type [trot,walk,pace,bount,rgallop,tgallop]");

std::vector<double>
        duty_factor = std::vector<double>(),
        goto_command = std::vector<double>(),
        goto_point = std::vector<double>();

// -- IDYN OPTIONS --
double STEP_SIZE = 0.005;

// ============================================================================
// ============================================================================
#include <random>

Ravelin::VectorNd& Quadruped::control(double t,
                                      const Ravelin::VectorNd& q,
                                      const Ravelin::VectorNd& qd,
                                      Ravelin::VectorNd& q_des,
                                      Ravelin::VectorNd& qd_des,
                                      Ravelin::VectorNd& u){
  static Ravelin::VectorNd qd_last = qd;

  SIMULATION_TIME = t;

  OUT_LOG(logINFO)<< "time = "<< t ;

    if(SIMULATION_TIME == 0.0){
      theConsole.ScriptLoad("startup.script");
    }

  // ----------------------------------------------------------------
  Ravelin::Vector3d lead(known_leading_force[3],
                         known_leading_force[4],
                         known_leading_force[5],
                         Moby::GLOBAL);

  OUTLOG(lead,"LEAD_g",logDEBUG);

  ((qdd = qd)-=qd_last)/=0.001;

#  ifdef VISUALIZE_MOBY
  for(int i=0;i<NUM_EEFS;i++){
      if(!eefs_[i].active)
        continue;
      visualize_ray(eefs_[i].point,eefs_[i].point+eefs_[i].normal*0.05,Ravelin::Vector3d(1,1,0),sim);
  }
#  endif

  update_poses();

  Ravelin::Vector3d point_on_robot(known_leading_force[0],
                                   known_leading_force[1],
                                   known_leading_force[2],
                                   base_link_frame);
  boost::shared_ptr<Ravelin::Pose3d> lead_transform =
      boost::shared_ptr<Ravelin::Pose3d>(new Ravelin::Pose3d(
        Ravelin::Quatd::identity(),
        Ravelin::Origin3d(
          Ravelin::Pose3d::transform_point(Moby::GLOBAL,point_on_robot)
        )
      ));

  Ravelin::SForced lead_force = Ravelin::Pose3d::transform(
                                   Moby::GLOBAL,
                                   Ravelin::SForced(lead,Ravelin::Vector3d(0,0,0),lead_transform)
                                 );
  OUTLOG(lead,"LEAD_bt",logDEBUG);

#ifndef SET_KINEMATICS
  Ravelin::VectorNd perturbation(NUM_JOINTS+6);
  perturbation.set_zero();

  for(int i=0;i<6;i++){
    perturbation[NUM_JOINTS+i] += ( unknown_base_perturbation[i]
                                   + known_base_perturbation[i]
                                   + lead_force[i]);
    if(t > 0.5 && t < 0.6){
      Ravelin::SVector6d push(0,20.0,0,0,0,0,environment_frame);
//      perturbation[NUM_JOINTS+i] += push[i];
    }
  }
  abrobot_->add_generalized_force(perturbation);
  // ----------------------------------------------------------------

  update();
#endif

  // Subtract unknown perturbations from fext vector
  // ----------------------------------------------------------------
  for(int i=0;i<6;i++)
    fext[NUM_JOINTS+i] -= unknown_base_perturbation[i];
  // ----------------------------------------------------------------


  OUT_LOG(logINFO)<< "num_contacts = " << NC ;

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
  Ravelin::SVector6d go_to(0,0,0,0,0,0,base_frame);
  Ravelin::VectorNd vb_w(NUM_EEFS*3 + 6);
  std::vector<Ravelin::Vector3d> foot_vel(NUM_EEFS), foot_pos(NUM_EEFS), foot_acc(NUM_EEFS);

  for(unsigned i=0;i< NUM_EEFS;i++){
    foot_pos[i] = Ravelin::Pose3d::transform_point(base_frame,Ravelin::Vector3d(0,0,0,eefs_[i].link->get_pose()));
    foot_vel[i].set_zero();
    foot_acc[i].set_zero();
    foot_pos[i].pose = foot_vel[i].pose = foot_acc[i].pose = base_frame;
  }

  if(WALK){
    Ravelin::Vector3d lead_base = Ravelin::Pose3d::transform_vector(base_link_frame,lead);
    Ravelin::SForced lead_base_force = Ravelin::Pose3d::transform(base_link_frame,lead_force);
    OUTLOG(lead_base,"LEAD_base",logDEBUG);
    // FOOTHOLDS
    static std::vector<Ravelin::Vector3d> footholds;
    if(TRACK_FOOTHOLDS && (footholds.size() == 0) && NC > 0){
      for(double sag=-1;sag<1;sag+=0.03){
        for(double cor=-1;cor<1;cor+=0.03){
          footholds.push_back(Ravelin::Vector3d(sag,cor,Utility::get_z_plane(sag,cor,center_of_contact.normal,center_of_contact.point)));
        }
      }
    } else if (!TRACK_FOOTHOLDS) {
      footholds.clear();
    }

    // Foot Locations
    std::vector<Ravelin::Vector3d> foot_origin;
    for(unsigned i=0;i< NUM_EEFS;i++)
      foot_origin.push_back(eefs_[i].origin);

    for(int i=0;i<6;i++)
      go_to[i] = goto_command[i];

    if(goto_point.size() == 3){
      Ravelin::Vector3d goto_direction =
          Ravelin::Vector3d(goto_point[0],goto_point[1],0,environment_frame)
          - Ravelin::Vector3d(center_of_mass_x[0],center_of_mass_x[1],0,environment_frame);
      goto_direction = Ravelin::Pose3d::transform_vector(base_frame,goto_direction);
      goto_direction.normalize();

      go_to[5] = goto_direction[1]*0.5;
    }

    // Robot attempts to align base with force and then walk along force axis
    go_to[0] += lead_base_force[0];
    go_to[1] += lead_base_force[1];
    go_to[5] += lead_base_force[5]*100.0;

    OUTLOG(go_to,"go_to",logDEBUG);

#ifdef VISUALIZE_MOBY
    visualize_ray(  Ravelin::Vector3d(lead_transform->x.data()),
                    Ravelin::Vector3d(lead_transform->x.data())
                     + Ravelin::Vector3d(known_leading_force[3],
                                         known_leading_force[4],
                                         known_leading_force[5]),
                    Ravelin::Vector3d(1,0,1),
                    sim
                  );
#endif

    OUTLOG(gait[gait_type],gait_type,logINFO);
    OUTLOG(duty_factor,"duty_factor",logINFO);

    walk_toward(go_to,gait[gait_type],footholds,duty_factor,gait_time,step_height,foot_origin,t,q,qd,qdd,foot_pos,foot_vel, foot_acc);

    // Recalculate contact jacobians based on desired lift-off feet
//    if(!USE_LAST_CFS){
//      NC = 0;
//      for (unsigned i=0; i< NUM_EEFS;i++)
//        if(eefs_[i].active)
//          NC++;
//      calc_contact_jacobians(N,D,R);
//    }
  }
  else {
    for(int i=0;i<NUM_EEFS;i++){
      eefs_[i].origin.pose = base_frame;
      foot_pos[i] = eefs_[i].origin + Ravelin::Vector3d(0.03*sin(t),0,0,base_frame);
      foot_vel[i] = Ravelin::Vector3d(0.03*cos(t),0,0,base_frame);
      foot_acc[i] = Ravelin::Vector3d(0.03*-sin(t),0,0,base_frame);

      RRMC(eefs_[i],q,eefs_[i].origin,q_des);
      Ravelin::VectorNd q_diff;
      (q_diff= q_des) -= q;
      q_diff *= STEP_SIZE;
      (q_des = q) += q_diff;
    }
  }
  trajectory_ik(foot_pos,foot_vel, foot_acc,q_des,qd_des,qdd_des);

  static Ravelin::MatrixNd MU;
  MU.set_zero(NC,NK/2);

  // --------------------------- FRICTION ESTIMATION ---------------------------
  // EXPERIMENTAL
  if(FRICTION_EST){
    Ravelin::VectorNd cf;
    double err = friction_estimation(vel,fext,STEP_SIZE,N,D,M,MU,cf);
    OUT_LOG(logINFO)<< "err (friction estimation): " << err << std::endl;
    OUTLOG(MU,"MU",logDEBUG);
    OUTLOG(cf,"contact_forces",logDEBUG);
  } else {
    for(int i=0,ii=0;i<NUM_EEFS;i++){
      if(eefs_[i].active){
        for(int k=0;k<NK/2;k++)
          MU(ii,k) = (eefs_[i].event->compliance == Moby::UnilateralConstraint::eRigid)? eefs_[i].event->contact_mu_coulomb : 0.01;
        ii++;
      }
    }
  }

  // -------------Kinematic (Sticking) Stabilization----------------------------
  // EXPERIMENTAL
  if(TRUNK_STABILIZATION){
//    Ravelin::VectorNd id(NUM_JOINTS);
//    Ravelin::MatrixNd J;
//    calc_base_jacobian(J);
//    zmp_stabilizer(J,Ravelin::Vector2d(0,0),id);
    Ravelin::SVector6d vb_des(0,0,0,0,0,0),
                       pb_des(0,0,0,0,0,0);
    if(FEEDBACK_ACCEL){
      static Ravelin::VectorNd Kv(0),Kp(0);
      if(Kv.rows() == 0){
        // D gains
        Kv.set_zero(6);
        Kv[3] = 3e2;
        Kv[4] = 3e2;
        // P gains
        Kp.set_zero(6);
        Kp[3] = 1e5;
        Kp[4] = 1e3;
      }
      contact_jacobian_stabilizer(R,Kp,Kv,pb_des,vb_des,qdd_des);
    }else{
      static Ravelin::VectorNd Kv(0),Kp(0);
      if(Kv.rows() == 0){
        // D gains
        Kv.set_zero(6);
        Kv[3] = 3e0;
        Kv[4] = 3e0;
        // P gains
        Kp.set_zero(6);
        Kp[3] = 1e3;
        Kp[4] = 1e1;
      }
      contact_jacobian_stabilizer(R,Kp,Kv,pb_des,vb_des,ufb);
    }
  }

  // --------------------------- ERROR FEEDBACK --------------------------------

  if (ERROR_FEEDBACK){
    if(JOINT_FEEDBACK){
      static std::map<std::string, Gains>      gains;
      if(FEEDBACK_ACCEL){
        // GAINS FOR ACCELERATION ERROR FEEDABCK
        for(int i=0;i<NUM_JOINTS;i++){
//          gains[joints_[i]->id].perr_sum = 0;
          gains[joints_[i]->id].kp = 1e4;
          gains[joints_[i]->id].kv = 3e2;
          gains[joints_[i]->id].ki = 1e0;
        }
        PID::control(q_des, qd_des,q,qd,joint_names_, gains,qdd_des);
      } else {
        // GAINS FOR FORCE ERROR FEEDABCK
        for(int i=0;i<NUM_JOINTS;i++){
//          gains[joints_[i]->id].perr_sum = 0;
          gains[joints_[i]->id].kp = 1e1;
          gains[joints_[i]->id].kv = 1e-1;
          gains[joints_[i]->id].ki = 3e-3;
        }
        PID::control(q_des, qd_des,q,qd,joint_names_, gains,ufb);
      }
    }
    if(WORKSPACE_FEEDBACK){
      // CURRENTLY THIS IS ONLY FORCE
      // BUT IT CAN BE ACCELERATIONS TOO
      std::vector<Ravelin::Matrix3d> W(boost::assign::list_of(Ravelin::Matrix3d::identity())(Ravelin::Matrix3d::identity())(Ravelin::Matrix3d::identity())(Ravelin::Matrix3d::identity()).convert_to_container<std::vector<Ravelin::Matrix3d> >() );
      if(FEEDBACK_ACCEL){
        double  Kp = 1e6,
                Kv = 3e4,
                Ki = 1e1;
        eef_stiffness_fb(W,Kp,Kv,Ki,foot_pos,foot_vel,q,qd,qdd_des);
      } else {
        double  Kp = 1e3,
                Kv = 1e1,
                Ki = 1e-2;
        eef_stiffness_fb(W,Kp,Kv,Ki,foot_pos,foot_vel,q,qd,ufb);
      }
    }
  }

  // ------------------------ INVERSE DYNAMICS ---------------------------------

  if(CONTROL_IDYN){
    double dt = STEP_SIZE;
    double alpha = 1.0;
    Ravelin::VectorNd cf;
    Ravelin::VectorNd id = Ravelin::VectorNd::zero(NUM_JOINTS);

    if(USE_LAST_CFS){
      cf.set_zero(NC*5);
      for(unsigned i=0,ii=0;i< eefs_.size();i++){
        if(!eefs_[i].active) continue;
        Ravelin::Matrix3d R_foot(             eefs_[i].normal[0],              eefs_[i].normal[1],              eefs_[i].normal[2],
                               eefs_[i].event->contact_tan1[0], eefs_[i].event->contact_tan1[1], eefs_[i].event->contact_tan1[2],
                               eefs_[i].event->contact_tan2[0], eefs_[i].event->contact_tan2[1], eefs_[i].event->contact_tan2[2]);
        Ravelin::Origin3d contact_impulse = Ravelin::Origin3d(R_foot.mult(eefs_[i].contact_impulses[0],workv3_)*(STEP_SIZE/0.001));
        cf[ii] = contact_impulse[0];
        if(contact_impulse[1] >= 0)
          cf[NC+ii] = contact_impulse[1];
        else
          cf[NC+ii+NC*2] = -contact_impulse[1];
        if(contact_impulse[2] >= 0)
          cf[NC+ii+NC] = contact_impulse[2];
        else
          cf[NC+ii+NC*3] = -contact_impulse[2];
        ii++;
      }
      Utility::check_finite(cf);
      OUTLOG(cf,"cf z",logDEBUG);
    }

    if(WORKSPACE_IDYN){
      // EXPERIMENTAL
      vb_w.set_zero(Rw.rows());
      Ravelin::SVector6d go_to_global(go_to.get_upper(),go_to.get_lower());
      workspace_trajectory_goal(go_to_global,foot_pos,foot_vel,foot_acc,1e1,STEP_SIZE,vb_w);

      workspace_inverse_dynamics(vel,vb_w,M,fext,dt,MU,id,cf);
    } else {
//      clock_t start = clock(), diff;
      if(!inverse_dynamics(vel,qdd_des,M,N,D,fext,dt,MU,id,cf))
        cf.set_zero(NC*5);

//      diff = clock() - start;
//      double msec = (double) diff * 1000.0 / (double) CLOCKS_PER_SEC;
//        OUT_LOG(logERROR) << "IDYN_TIMING = " << msec;
        OUTLOG(cf,"cf",logDEBUG);

        std::cout << "cfs = [";
        for(int i=0, ii = 0;i<NUM_EEFS;i++){
          if(eefs_[i].active){
            std::cout << " " << cf[ii];
            ii++;
          } else {
            std::cout << " " << 0;
          }
        }
        std::cout << "]';" << std::endl;
    }
    uff += (id*=alpha);
  }

  // -----------------------------------------------------------------------------

  Utility::check_finite(ufb);
  Utility::check_finite(uff);
  // combine ufb and uff
  u = ufb;
  u += uff;

  // -----------------------------------------------------------------------------

  for(unsigned i=0;i< NUM_JOINTS;i++){
    if(u[i] > torque_limits_u[i])
      u[i] = torque_limits_u[i];
    if(u[i] < torque_limits_l[i])
      u[i] = torque_limits_l[i];

    joints_[i]->q[0]  = q[i];
    joints_[i]->qd[0]  = qd[i];
  }
  abrobot_->update_link_poses();
  abrobot_->update_link_velocities();

  if(LOG_TYPE.compare("DEBUG") == 0){
    Ravelin::MatrixNd Jf;
    for(int i=0;i<NUM_EEFS;i++){
      boost::shared_ptr<Ravelin::Pose3d> event_frame(new Ravelin::Pose3d(foot_pos[i].pose));
      EndEffector& foot = eefs_[i];

      // Positional Correction
      Ravelin::Vector3d foot_pos_now = Ravelin::Pose3d::transform_point(foot_pos[i].pose,Ravelin::Vector3d(0,0,0,eefs_[i].link->get_pose()));
      Ravelin::Vector3d x_err  = foot_pos[i] - foot_pos_now;
      OUTLOG( foot_pos_now,foot.id + "_x",logDEBUG);
      OUTLOG( foot_pos[i],foot.id + "_x_des",logDEBUG);
      OUTLOG( x_err,foot.id + "_x_err",logDEBUG);

      // Remove portion of foot velocity that can't be affected by corrective forces
      event_frame->x = Ravelin::Pose3d::transform_point(foot_pos[i].pose,Ravelin::Vector3d(0,0,0,eefs_[i].link->get_pose()));
      dbrobot_->calc_jacobian(event_frame,eefs_[i].link,Jf);
      Ravelin::SharedConstMatrixNd Jb = Jf.block(0,3,NUM_JOINTS,NDOFS);
      Ravelin::SharedConstVectorNd vb = vel.segment(NUM_JOINTS,NDOFS);
      Jb.mult(vb,workv3_);
      workv3_.pose = foot_pos[i].pose;

      // Velocity Correction
      Ravelin::Vector3d foot_vel_now = (Ravelin::Pose3d::transform_vector(foot_pos[i].pose,eefs_[i].link->get_velocity().get_linear()) - workv3_);
      Ravelin::Vector3d xd_err = foot_vel[i] - foot_vel_now;
      OUTLOG( foot_vel_now,foot.id + "_xd",logDEBUG);
      OUTLOG( foot_vel[i],foot.id + "_xd_des",logDEBUG);
      OUTLOG( xd_err,foot.id + "_xd_err",logDEBUG);
    }
  }

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
   OUTLOG(q,"q",logDEBUG);
   OUTLOG(qd,"qd",logDEBUG);
   OUTLOG(qdd,"qdd",logDEBUG);
   OUTLOG(q_des,"q_des",logDEBUG);
   OUTLOG(qd_des,"qd_des",logDEBUG);
   OUTLOG(qdd_des,"qdd_des",logDEBUG);
   OUTLOG(fext,"fext",logDEBUG);

   OUTLOG(uff,"uff",logINFO);
   OUTLOG(ufb,"ufb",logINFO);
   // -----------------------------------------------------------------------------

   // Deactivate all contacts
   NC = 0;
   for(int i=0;i<eefs_.size();i++)
     eefs_[i].active = false;

   qd_last = qd;

//   moving average u
   bool MOVING_AVERAGE= false;
   if(MOVING_AVERAGE){
       static Ravelin::VectorNd total_u(0);
       if(total_u.rows() == 0)
         total_u.set_zero(u.rows());

       total_u += u;

       static std::deque<Ravelin::VectorNd> u_list;

       u_list.push_back(u);
       if(u_list.size() > 2){
          total_u -= u_list.at(0);
          u_list.pop_front();
       }

       return (u = total_u) /= u_list.size();
    } else {
       return u;
    }
}


// ============================================================================
// ===========================  END CONTROLLER  ===============================
// ============================================================================

#ifdef VISUALIZE_MOBY
# include <thread>
  extern void init_glconsole();
  std::thread * tglc;
#endif


void Quadruped::init(){
  unknown_base_perturbation = boost::assign::list_of(0.0)(0.0)(0.0)(0.0)(0.0)(0.0).convert_to_container<std::vector<double> >();
  known_base_perturbation = boost::assign::list_of(0.0)(0.0)(0.0)(0.0)(0.0)(0.0).convert_to_container<std::vector<double> >();
  known_leading_force = boost::assign::list_of(0.13)(0.0)(0.0)(0.0)(0.0)(0.0).convert_to_container<std::vector<double> >();
//  goto_point = boost::assign::list_of(10)(0)(0).convert_to_container<std::vector<double> >();
  duty_factor = boost::assign::list_of(0.75)(0.75)(0.75)(0.75).convert_to_container<std::vector<double> >();
  goto_command = boost::assign::list_of(goto_X)(goto_Y)(0)(0)(0)(goto_GAMMA).convert_to_container<std::vector<double> >();
#ifdef VISUALIZE_MOBY
  CVarUtils::AttachCVar<std::vector<double> >( "qd.known_base_perturbation",&known_base_perturbation,"Apply a constant [3 linear,3 angular] force to robot base, the robot can sense the applied force");
  CVarUtils::AttachCVar<std::vector<double> >( "qd.unknown_base_perturbation",&unknown_base_perturbation,"Apply a constant [3 linear,3 angular] force to robot base, the robot can NOT sense the applied force");
  CVarUtils::AttachCVar<std::vector<double> >( "qd.known_leading_force",&known_leading_force,"Apply a constant [3 pt{base_frame}][3 linear] force to robot base, the robot can sense the applied force and will follow it");
  CVarUtils::AttachCVar<std::vector<double> >( "qd.locomotion.point",&goto_point,"Walk toward this point in environment [ (x,y,gamma) {environment_frame}]");

  CVarUtils::AttachCVar<bool>( "qd.locomotion.active",&WALK,"Activate Walking?");
  CVarUtils::AttachCVar<bool>( "qd.locomotion.track_footholds",&TRACK_FOOTHOLDS,"Locate and use footholds?");// EXPERIMENTAL
  CVarUtils::AttachCVar<bool>( "qd.idyn",&CONTROL_IDYN,"Activate IDYN?");
  CVarUtils::AttachCVar<bool>( "qd.widyn",&WORKSPACE_IDYN,"Activate WIDYN?");// EXPERIMENTAL
  CVarUtils::AttachCVar<bool>( "qd.use_cfs",&USE_LAST_CFS,"Use last detected contact forces?");// EXPERIMENTAL
  CVarUtils::AttachCVar<bool>( "qd.error-feedback.active",&ERROR_FEEDBACK,"Use error-feedback control?");
  CVarUtils::AttachCVar<bool>( "qd.error-feedback.joint",&JOINT_FEEDBACK,"Apply error-feedback to the joints?");
  CVarUtils::AttachCVar<bool>( "qd.error-feedback.accel",&FEEDBACK_ACCEL,"Apply error-feedback as accelerations?");
  CVarUtils::AttachCVar<bool>( "qd.error-feedback.workspace",&WORKSPACE_FEEDBACK,"Use error-feedback in workspace frame?");

  // -- LOCOMOTION OPTIONS --
  CVarUtils::AttachCVar<double>( "qd.locomotion.gait_time",&gait_time,"Gait Duration over one cycle.");
  CVarUtils::AttachCVar<double>( "qd.locomotion.step_height",&step_height,"Height of a step");
  CVarUtils::AttachCVar<std::vector<double> >( "qd.locomotion.command",&goto_command,"Base command differential");

  // Assign Gait to the locomotion controller
  CVarUtils::AttachCVar<std::string>( "qd.locomotion.gait_type",&gait_type,"Gait type [trot,walk,pace,bount,rgallop,tgallop]");
  CVarUtils::AttachCVar<std::vector<double> >( "qd.locomotion.duty_factor",&duty_factor,"duty_factor");
  // -- IDYN OPTIONS --
  CVarUtils::AttachCVar<double>( "qd.dt",&STEP_SIZE,"value for dt (also h) used in IDYN and other functions");

  CVarUtils::AttachCVar<double>( "sim.mu_coulomb",&SIM_MU_COULOMB,"Coulomb Friction for all contact");
  CVarUtils::AttachCVar<double>( "sim.mu_viscous",&SIM_MU_VISCOSE,"Viscous Friction for all contact");
  CVarUtils::AttachCVar<double>( "sim.penalty_kv",&SIM_PENALTY_KV,"Spring term for compliant contact");
  CVarUtils::AttachCVar<double>( "sim.penalty_kp",&SIM_PENALTY_KP,"Damper term for compliant contact");

   tglc = new std::thread(init_glconsole);
#endif
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

  int num_leg_stance = 2;
  switch(num_leg_stance){
    case 0:
      eef_origins_["LF_FOOT"] = Ravelin::Vector3d( 0.13, 0.096278, -0.16);
      eef_origins_["RF_FOOT"] = Ravelin::Vector3d( 0.13,-0.096278, -0.16);
      eef_origins_["LH_FOOT"] = Ravelin::Vector3d(-0.09, 0.096278, -0.16);
      eef_origins_["RH_FOOT"] = Ravelin::Vector3d(-0.09,-0.096278, -0.16);
    break;
    case 2:
      eef_origins_["LF_FOOT"] = Ravelin::Vector3d( 0.115, 0.096278, -0.135);
      eef_origins_["RF_FOOT"] = Ravelin::Vector3d( 0.115,-0.096278, -0.135);
      eef_origins_["LH_FOOT"] = Ravelin::Vector3d(-0.105, 0.096278, -0.16);
      eef_origins_["RH_FOOT"] = Ravelin::Vector3d(-0.105,-0.096278, -0.16);
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
  torque_limits_["BODY_JOINT"]=  6.00;
  torque_limits_["LF_HIP_AA"] =  6.00;
  torque_limits_["LF_HIP_FE"] =  6.00;
  torque_limits_["LF_LEG_FE"] =  6.00;

  torque_limits_["RF_HIP_AA"] =  6.00;
  torque_limits_["RF_HIP_FE"] =  6.00;
  torque_limits_["RF_LEG_FE"] =  6.00;

  torque_limits_["LH_HIP_AA"] =  6.00;
  torque_limits_["LH_HIP_FE"] =  6.00;
  torque_limits_["LH_LEG_FE"] =  6.00;

  torque_limits_["RH_HIP_AA"] =  6.00;
  torque_limits_["RH_HIP_FE"] =  6.00;
  torque_limits_["RH_LEG_FE"] =  6.00;

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
    // Trotting gait 50/50 duty cycle
    gait["trot"] = boost::assign::list_of(0.0)(0.5)(0.5)(0.0).convert_to_container<std::vector<double> >();

    // walk lf,rf,lh,rh
    gait["walk"] = boost::assign::list_of(0.25)(0.75)(0.0)(0.5).convert_to_container<std::vector<double> >();

    // pace
    gait["pace"] = boost::assign::list_of(0.0)(0.5)(0.0)(0.5).convert_to_container<std::vector<double> >();

    // bound
    gait["bound"] = boost::assign::list_of(0.5)(0.5)(0.0)(0.0).convert_to_container<std::vector<double> >();

    // transverse gallop
    gait["tgallop"] = boost::assign::list_of(0.8)(0.9)(0.3)(0.4).convert_to_container<std::vector<double> >();

    // Rotary gallop
    gait["rgallop"] = boost::assign::list_of(0.7)(0.6)(0.0)(0.1).convert_to_container<std::vector<double> >();
  }

  environment_frame = boost::shared_ptr<Ravelin::Pose3d>( new Ravelin::Pose3d(Moby::GLOBAL));
  environment_frame->x = Ravelin::Origin3d(0,0,0);
  environment_frame->q.set_identity();
}
