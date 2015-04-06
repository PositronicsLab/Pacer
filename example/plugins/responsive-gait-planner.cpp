/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/controller.h>
#include <Pacer/utilities.h>
using namespace Pacer;

static Ravelin::Vector3d workv3_;
static Ravelin::VectorNd q;

boost::shared_ptr<Pacer::Controller> ctrl_ptr;
  boost::shared_ptr<Ravelin::Pose3d> base_frame;
  std::string plugin_namespace;
  std::vector<std::string> foot_names;

void select_foothold(const std::vector<Ravelin::Vector3d>& footholds,const Ravelin::Origin3d &x, Ravelin::Origin3d& x_fh){
  double min_dist = INFINITY;
  int    min_vec = 0;
  for(int i=0;i<footholds.size();i++){
    double norm = (footholds[i]-x).norm();
    if(norm < min_dist){
      min_dist = norm;
      min_vec = i;
    }
  }
  x_fh = Ravelin::Origin3d(footholds[min_vec]);
}

bool gait_phase(double touchdown,double duty_factor,double gait_progress){
  double liftoff = touchdown + duty_factor,left_in_phase = 0;
  liftoff = liftoff - (double) ((int) liftoff);

  OUT_LOG(logDEBUG) << "gait_progress " << gait_progress;
  OUT_LOG(logDEBUG) << "touchdown " << touchdown;
  OUT_LOG(logDEBUG) << "duty_factor " << duty_factor;
  OUT_LOG(logDEBUG) << "liftoff " << liftoff;

  // ----- STANCE PHASE ------
  if(// during STANCE (no wrap around)
     ( gait_progress >= touchdown  &&  gait_progress  < liftoff)
     // during STANCE (with wrap around) -- same as !SWING
    || (!(gait_progress  < touchdown  &&  gait_progress >= liftoff) && liftoff < touchdown)
     ){
    return true;
  }
  // ----- SWING PHASE -----
  return false;

}

double gait_phase(double touchdown,double duty_factor,double gait_progress,double stance_phase){
  double liftoff = touchdown + duty_factor,left_in_phase = 0;
  liftoff = liftoff - (double) ((int) liftoff);

  OUT_LOG(logDEBUG) << "gait_progress " << gait_progress;
  OUT_LOG(logDEBUG) << "touchdown " << touchdown;
  OUT_LOG(logDEBUG) << "duty_factor " << duty_factor;
  OUT_LOG(logDEBUG) << "liftoff " << liftoff;

  // ----- STANCE PHASE ------
  if(stance_phase){
    // left in stance phase = negaive time left in phase
    left_in_phase = (liftoff - gait_progress);
    if(left_in_phase < 0)
      left_in_phase = (1.0 - gait_progress) + liftoff;
//    left_in_phase = -duty_factor;
  }
  // ----- SWING PHASE -----
  else {
    left_in_phase = (touchdown - gait_progress);
    if(left_in_phase < 0)
      left_in_phase = (1.0 - gait_progress) + touchdown;
//    left_in_phase = 1-duty_factor;
  }
  return left_in_phase;
}

/**
 * @brief walk_toward : OSRF Locomotion System Implementation
 * @param command : 6x1 vector of goal base velocity differential
 * @param touchdown : specify the moment in the gait where a foot touches down stance phase. touchdown_i \in [0..1)
 * @param footholds : vector of 3x1 points indicating locations for valid foot placement
 * @param duty_factor : portion of gait a foot is in stance phase after touchdown duty_factor_{i} \in [0..1)
 * @param gait_duration : duration of time in seconds of one gait cycle [0..inf)
 * @param step_height : maximum height of foot during a step
 * @param foot_origins : vector of NUM_EEFS 3x1 points indicating intermediate foot placement
 * @param t : current time
 * @param q : current generalized coordinates
 * @param qd : current generalized velocity
 * @param qdd : current generalized acceleration
 * @param foot_pos : NUM_FEET length vector of 3x1 cartesian coordinates for feet (populated with current values)
 * @param foot_vel : NUM_FEET length vector of 3x1 cartesian velocities for feet (populated with current values)
 * @param foot_acc : NUM_FEET length vector of 3x1 cartesian acceleration for feet (populated with current values)
 */
void walk_toward(
    // PARAMETERS
    const Ravelin::VectorNd& command,
    const std::vector<double>& touchdown,
    const std::vector<Ravelin::Vector3d>& footholds,
    const std::vector<double>& duty_factor,
    double gait_duration,
    double step_height,
    bool STANCE_ON_CONTACT,
    // MODEL
    std::vector<Ravelin::Vector3d>& origins,
    const Ravelin::Vector3d& center_of_mass_x,
    double t,
    // OUTPUT
    std::vector<Ravelin::Vector3d>& foot_pos,
    std::vector<Ravelin::Vector3d>& foot_vel,
    std::vector<Ravelin::Vector3d>& foot_acc)
{
  OUT_LOG(logDEBUG) << " -- walk_toward() entered";
  static bool inited = false;
  int spline_plan_length = 1,
      NUM_FEET = origins.size();
  static double last_time = t;
  static std::vector<bool> last_feet_active(NUM_FEET);

  if(t-last_time > gait_duration* (*std::min_element(duty_factor.begin(),duty_factor.end())) )
    inited = false;

  // persistent Vector storing spline coefs
  // [foot][dimension][interval]
  static std::vector< std::vector< std::vector<Ravelin::VectorNd> > > spline_coef(NUM_FEET);
  // persistent Vector storing time delimitations to each spline
  // [foot][interval]
  static std::vector< std::vector<Ravelin::VectorNd> > spline_t(NUM_FEET);
  static std::vector< bool > stance_phase(NUM_FEET);

  double gait_progress = t/gait_duration;
  gait_progress = gait_progress - (double) ((int) gait_progress);
  if(gait_progress >= 1) gait_progress = Moby::NEAR_ZERO;
  OUT_LOG(logDEBUG) << "\tprog : " << gait_progress;
  OUT_LOG(logDEBUG) << "\ttime : " << t;

  Ravelin::Vector3d turn_origin(0,command[0]/command[5],0,foot_pos[0].pose);

  // Ackerman steering, foot velocity
  std::vector<Ravelin::Vector3d> xd_stance(NUM_FEET);
  
  boost::shared_ptr< Ravelin::Pose3d> turning_frame = boost::shared_ptr< Ravelin::Pose3d>(new Ravelin::Pose3d(*base_frame.get()));

  turning_frame->x = Ravelin::Origin3d(center_of_mass_x) + Ravelin::Origin3d(Ravelin::Pose3d::transform_vector(Moby::GLOBAL,turn_origin));

  // (Re)Populate spline vectors
  if(!inited){
    for(int i=0;i<NUM_FEET;i++){
      last_feet_active[i] = false;
      stance_phase[i] = gait_phase(touchdown[i],duty_factor[i],gait_progress);
      spline_coef[i].resize(3);
      for(int d=0; d<3;d++){
        spline_coef[i][d].resize(spline_plan_length);
        for(int j = 0; j<  spline_plan_length;j++)
          spline_coef[i][d][j] = Ravelin::VectorNd();
      }

      spline_t[i].resize(spline_plan_length);
      for(int j = 0; j<  spline_plan_length;j++)
        spline_t[i][j] = Ravelin::VectorNd::zero(2);
    }
  }

  ctrl_ptr->set_model_state(q);
  ////////////////// PHASE PLANNING ///////////////////////
  for(int i=0;i<NUM_FEET;i++){

    OUT_LOG(logDEBUG) << "\t PHASE PROGRESS: { " << touchdown[i] << " .. " << gait_progress << " .. " << (touchdown[i] + duty_factor[i]) <<" }, stance: " << !stance_phase[i];
    Ravelin::Vector3d &x   = foot_pos[i],
                      &xd  = foot_vel[i],
                      &xdd = foot_acc[i];

    // Check if Spline must be reevaluated
    double left_in_phase;
    bool replan_path = false;
    if(inited){
      Ravelin::Vector3d x_   = foot_pos[i];
      // Set liftoff feet (opposite of stance_phase variable)
        for(int d=0; d<3;d++){
          replan_path = !Utility::eval_cubic_spline(spline_coef[i][d],spline_t[i],t,x[d],xd[d],xdd[d]);
          if(replan_path)
            break;
        }
        // Stance phase (opposite to signal switch on next re-program)
        if(!stance_phase[i]){
          Ravelin::VectorNd generalized_command, foot_vel;
          generalized_command.set_zero(q.rows() - Pacer::NEULER + Pacer::NSPATIAL);
          generalized_command.set_sub_vec(generalized_command.rows() - Pacer::NSPATIAL,command);
        
          Ravelin::MatrixNd J = ctrl_ptr->calc_link_jacobian(q,foot_names[i]);
        
          J.mult(generalized_command,foot_vel,-1.0,0);
          xd = Ravelin::Vector3d(foot_vel.segment(0,3).data(),base_frame);
          x_.pose = base_frame;
          x = x_+xd*0.001;
        }
    }

    // Plan a new spline for this foot
    if(replan_path || !inited ){
      x = Ravelin::Pose3d::transform_point(base_frame,Ravelin::Vector3d(0,0,0,ctrl_ptr->get_link(foot_names[i])->get_pose()));
      OUTLOG(origins[i],"foot_origin_" + std::to_string(i),logDEBUG);

      // What phase of the gait is the controller in?
      // NOTE: Don't ever use modf (this does modf(t/gait_duration,&intpart))
      // truncates t/gait_duration at decimal; gait_progress \in [0,1)

      // How much time is left in the current phase stance (-), swing (+)
      left_in_phase = gait_phase(touchdown[i],duty_factor[i],gait_progress,stance_phase[i]);

      OUT_LOG(logDEBUG) << "\tleft in phase (sec) : " << left_in_phase
                        << " (" << left_in_phase*gait_duration << ") ";

      OUT_LOG(logDEBUG) << "\tPlanning next Spline";
      // creat new spline at top of history

      // take time at end of spline_t history
      double t0 = 0;

      // Get starting point of spline

      if(!inited){ // first iteration
        xd.set_zero();
        xdd.set_zero();
      } else {
        if(STANCE_ON_CONTACT && t < *(spline_t[i].rbegin()->end()-1)){
          t0 = t;
        } else {
        // continue off of the end of the last spline
        t0 = *(spline_t[i].rbegin()->end()-1) - Moby::NEAR_ZERO;
        }
        Ravelin::Vector3d x_;
        for(int d=0; d<3;d++){
          bool pass = Utility::eval_cubic_spline(spline_coef[i][d],spline_t[i],t0,x_[d],xd[d],xdd[d]);
          assert(pass);
        }
      }

      // Calculate foot step info
      // Determine linear portion of step
      boost::shared_ptr< Ravelin::Pose3d> foot_frame = boost::shared_ptr< Ravelin::Pose3d>(new Ravelin::Pose3d(base_frame));
      foot_frame->x = Ravelin::Origin3d(origins[i].data());
      Ravelin::Origin3d x0 = origins[i].data();

      Ravelin::Vector3d foot_goal(command[0],command[1],0,foot_frame);
      
        
      if(fabs(command[5]) > Moby::NEAR_ZERO){
        // Determine shape of step needed to yaw robot
        Ravelin::Origin3d rotation_axis = Ravelin::Origin3d(0,0,1);

//        // Foot velocity to perform Ackerman steering
//        Ravelin::Origin3d body_origin = -Ravelin::Origin3d::cross(rotation_axis,Ravelin::Origin3d(foot_goal.data())) * command[5];
//        Ravelin::Origin3d foot_origin = turn_origin + Ravelin::Origin3d(origins[i]);
//          OUTLOG(body_origin,"body_origin",logDEBUG);
//          OUTLOG(foot_origin,"foot_origin",logDEBUG);

        Ravelin::AAngled aa_gamma(Ravelin::Vector3d(rotation_axis,turning_frame),command[5] *  left_in_phase*gait_duration);
        OUTLOG(aa_gamma,"aa_gamma",logDEBUG);
        Ravelin::Matrix3d R_gamma;
        R_gamma = aa_gamma;
        OUTLOG(R_gamma,"R_gamma",logDEBUG);
        // yaw_step = R(gamma)*r_com - r_com
        R_gamma.mult(Ravelin::Pose3d::transform_point(turning_frame, foot_goal), workv3_);
        workv3_.pose = turning_frame;
        foot_goal = Ravelin::Pose3d::transform_point(foot_frame, workv3_);
      }

      // scale step length by interval duration
      foot_goal *=  left_in_phase*gait_duration;

      // All steps wrt to x0
      foot_goal /= 2;

      OUT_LOG(logDEBUG) << "\tstep = " << foot_goal;

      // Set control point for either stance or swing phase
      std::vector<Ravelin::Origin3d> control_points;

      if(!stance_phase[i]){
        // SWING
        OUT_LOG(logDEBUG) << "\tPlanning Swing phase (phase > 0)...";

        foot_goal /= (1-duty_factor[i]);
        OUT_LOG(logDEBUG) << "\tstep = " << foot_goal;
        /*
        // velocity correction
        // FEEDBACK CAPTURE POINT
//        Ravelin::Vector3d hip_pos = Ravelin::Pose3d::transform_point(Moby::GLOBAL,Ravelin::Vector3d(0,0,0,links_[3+i]->get_pose()));
//        OUT_LOG(logDEBUG1) << "Getting " << links_[3+i]->id << " pose";
        double eta = 1.3,
               height = 0.30;
        Ravelin::Origin3d rfb = eta*(Ravelin::Origin3d(command.get_upper())
                                     -  Ravelin::Origin3d(
                                          Ravelin::Pose3d::transform_vector(foot_pos[i].pose, base_velocity)
                                        )
                                     ) * sqrt(height/grav);
//        rfb[2] = 0;
//        rfb[1] = 0;
//        foot_goal -= Ravelin::Origin3d(rfb);
         */ 
          control_points.push_back(Ravelin::Origin3d(x));
          control_points.push_back(Ravelin::Origin3d(x) + Ravelin::Origin3d(0,0,step_height));
        if(footholds.empty()){
          control_points.push_back(x0 + Ravelin::Origin3d(foot_goal) + Ravelin::Origin3d(0,0,step_height));
          control_points.push_back(x0 + Ravelin::Origin3d(foot_goal));
        } else {
          Ravelin::Origin3d x_fh;
          select_foothold(footholds,Ravelin::Pose3d::transform_point(Moby::GLOBAL, Ravelin::Vector3d((x0+Ravelin::Origin3d(foot_goal)).data(),base_frame)).data(),x_fh);
          // Reach new foothold, account for movement of robot during step
          control_points.push_back(Ravelin::Origin3d(Ravelin::Pose3d::transform_point(base_frame,Ravelin::Vector3d(x_fh.data(),Moby::GLOBAL)).data()) - Ravelin::Origin3d(foot_goal) + Ravelin::Origin3d(0,0,step_height));
          control_points.push_back(x_fh - Ravelin::Origin3d(foot_goal));
        }
      }
      else {
        // STANCE
        foot_goal /= -duty_factor[i];

        // project foot goal 'a' onto walking plane (defined by normal 'b')
        // x = v{||} - w{_|_}
        // w = b * (a.a / b.b)
        control_points.resize(3);
        control_points[0] = Ravelin::Origin3d(x);
        control_points[2] = Ravelin::Origin3d(x) + Ravelin::Origin3d(foot_goal);
        control_points[1] = (control_points[2] + control_points[0])/2.0;
        control_points[1].normalize();
        control_points[1] *= (control_points[2].norm() + control_points[0].norm())/2.0;
        if( (control_points[2] - control_points[1]).norm() < Moby::NEAR_ZERO)
          control_points.pop_back();
      }

      // create new spline!!
      // copy last spline to history erasing oldest spline

      for(int j=0;j<spline_plan_length-1;j++){
        for(int d=0; d<3;d++)
          spline_coef[i][d][j] = spline_coef[i][d][j+1];
        spline_t[i][j] = spline_t[i][j+1];
      }

      // create spline using set of control points, place at back of history
      int n = control_points.size();

      spline_t[i].rbegin()->set_zero(n);
      Ravelin::VectorNd           &T = *(spline_t[i].rbegin());
      for(int j=0;j<n;j++)
        T[j] = t0 + fabs( left_in_phase*gait_duration / (double)(n-1)) * (double)j ;

      for(int cp=0;cp<control_points.size();cp++){
        OUTLOG(control_points[cp],"control_point",logDEBUG1);
      }
      OUTLOG(xd,"control_point_V",logDEBUG1);
      for(int d=0;d<3;d++){
        Ravelin::VectorNd           X(n);
        Ravelin::VectorNd          &coefs = *(spline_coef[i][d].rbegin());

        for(int j=0;j<n;j++)
          X[j] = control_points[j][d];
        OUTLOG(T,"T",logDEBUG1);
        OUTLOG(X,"X",logDEBUG1);

        Utility::calc_cubic_spline_coefs(T,X,Ravelin::Vector2d(xd[d],xd[d]),coefs);

        // then re-evaluate spline
        // NOTE: this will only work if we replanned for a t_0  <  t  <  t_0 + t_I
        OUT_LOG(logDEBUG) << "Eval first step in spline";
        Utility::eval_cubic_spline(spline_coef[i][d],spline_t[i],t,x[d],xd[d],xdd[d]);
      }
      stance_phase[i] = !stance_phase[i];
    }
    ctrl_ptr->set_data<int>(foot_names[i]+".stance",(!stance_phase[i])? 1 : 0);
    //last_feet_active[i] = feet[i]->active;
    //feet[i]->stance = !stance_phase[i];
    x.pose = foot_pos[i].pose;
    xd.pose = foot_vel[i].pose;
    xdd.pose = foot_acc[i].pose;
  }

//#ifdef NDEBUG
///* VISUALIZE
  {

  for(int i=0;i<NUM_FEET;i++){
      {
        Ravelin::Vector3d p = Ravelin::Pose3d::transform_point(Moby::GLOBAL,foot_pos[i]);
        Ravelin::Vector3d v = Ravelin::Pose3d::transform_vector(Moby::GLOBAL,foot_vel[i])/10;
        Utility::visualize.push_back( Pacer::VisualizablePtr( new Pacer::Point( p,   Ravelin::Vector3d(0,0,1),0.1)));
        Utility::visualize.push_back( Pacer::VisualizablePtr( new Ray(  v+p,   p,   Ravelin::Vector3d(0,1,0),0.1)));
      }

      Ravelin::VectorNd &T1 = *(spline_t[i].begin()),
                      &T2 = *(spline_t[i].rbegin());


    for(double t=T1[0]+Moby::NEAR_ZERO ; t<=T2[T2.rows()-1] ; t += 0.01){
      Ravelin::Vector3d x,xd,xdd;
      for(int d=0;d<3;d++){
        Utility::eval_cubic_spline(spline_coef[i][d],spline_t[i],t,x[d],xd[d],xdd[d]);
      }
      x.pose = foot_pos[i].pose;
      xd.pose = foot_vel[i].pose;
      xdd.pose = foot_acc[i].pose;
      Ravelin::Vector3d p = Ravelin::Pose3d::transform_point(Moby::GLOBAL,x);
      Ravelin::Vector3d v = Ravelin::Pose3d::transform_vector(Moby::GLOBAL,xd)/10;
//      Ravelin::Vector3d a = Ravelin::Pose3d::transform_vector(Moby::GLOBAL,xdd)/100;
      Utility::visualize.push_back( Pacer::VisualizablePtr( new Pacer::Point( p,   Ravelin::Vector3d(0,1,0),0.01)));
      Utility::visualize.push_back( Pacer::VisualizablePtr( new Ray(  v+p,   p,   Ravelin::Vector3d(1,0,0),0.01)));
//     Utility::visualize.push_back( Pacer::VisualizablePtr( new Ray(a+v+p, v+p, Ravelin::Vector3d(1,0.5,0)));
    }
  }
  }
//  END VISUALIZE */
//#endif
  last_time = t;
  inited = true;
  OUT_LOG(logDEBUG) << " -- walk_toward() exited";
}

void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t){
  ctrl_ptr = ctrl;
  
  const int queue_size = 100;
  static std::deque<Ravelin::Origin3d> command_queue;
  static Ravelin::Origin3d sum_command = Ravelin::Origin3d(0,0,0);
  
  {
    Ravelin::Origin3d command = Ravelin::Origin3d(0,0,0);
    ctrl->get_data<Ravelin::Origin3d>("SE2_command",command);
    command_queue.push_front(command);
    sum_command += command;
  }

  if(command_queue.size() > queue_size){
    sum_command -= command_queue.back();
    command_queue.pop_back();
  }

  Ravelin::Origin3d command = sum_command / (double) command_queue.size();

  Ravelin::VectorNd go_to;
  go_to.set_zero(6);
  go_to[0] = command[0];
  go_to[1] = command[1];
  go_to[5] = command[2];

  static std::vector<double>
      &duty_factor = Utility::get_variable<std::vector<double> >(plugin_namespace+"duty-factor"),
      &this_gait = Utility::get_variable<std::vector<double> >(plugin_namespace+"gait");
  static double &gait_time = Utility::get_variable<double>(plugin_namespace+"gait-duration");
  static double &step_height = Utility::get_variable<double>(plugin_namespace+"step-height");
  static std::vector<Ravelin::Vector3d> footholds(0);
  foot_names = Utility::get_variable<std::vector<std::string> >(plugin_namespace+"feet");

  static double &width = Utility::get_variable<double>(plugin_namespace+"width");
  static double &length = Utility::get_variable<double>(plugin_namespace+"length");
  static double &height = Utility::get_variable<double>(plugin_namespace+"height");

  base_frame = boost::shared_ptr<Ravelin::Pose3d>( new Ravelin::Pose3d(
        ctrl->get_data<Ravelin::Pose3d>("base_link_frame")));
        //get_data<Ravelin::Pose3d>("base_horizontal_frame")));
  
  int NUM_FEET = foot_names.size();
  std::vector<Ravelin::Vector3d>
      foot_vel(NUM_FEET),
      foot_pos(NUM_FEET),
      foot_acc(NUM_FEET);
  q = ctrl->get_generalized_value(Pacer::Robot::position);
  
  for(int i=0;i<NUM_FEET;i++){
    foot_pos[i] = Ravelin::Vector3d(0,0,0,base_frame);
    ctrl->get_data<Ravelin::Vector3d>(foot_names[i]+".goal.x",foot_pos[i]);
    foot_vel[i] = Ravelin::Vector3d(0,0,0,base_frame);
    foot_acc[i] = Ravelin::Vector3d(0,0,0,base_frame);
    //double gait_progress = t/gait_time;
    //gait_progress = gait_progress - (double) ((int) gait_progress);
  }
  
  std::vector<Ravelin::Vector3d> origins;
  if(origins.empty()){
    origins.resize(NUM_FEET);
    for(int i=0;i<NUM_FEET;i++){
      origins[i] = Ravelin::Vector3d(
        Utility::sign<double>(foot_pos[i][0])*length/2,
        Utility::sign<double>(foot_pos[i][1])*width/2,
        -height,base_frame);
        //origins[i] = foot_pos[i];
    }
  }

  OUTLOG(this_gait,"this_gait",logINFO);
  OUTLOG(duty_factor,"duty_factor",logINFO);

  int STANCE_ON_CONTACT = Utility::get_variable<int>(plugin_namespace+"stance-on-contact");
  walk_toward(go_to,this_gait,footholds,duty_factor,gait_time,step_height,STANCE_ON_CONTACT,origins,ctrl->get_data<Ravelin::Vector3d>("center_of_mass.x"),t,foot_pos,foot_vel, foot_acc);
//  cpg_trot(go_to,this_gait,duty_factor,gait_time,step_height,foot_origin,t,foot_pos,foot_vel,foot_acc);
  for(int i=0;i<NUM_FEET;i++){
    ctrl->set_data<Ravelin::Vector3d>(foot_names[i]+".goal.x",foot_pos[i]);
    ctrl->set_data<Ravelin::Vector3d>(foot_names[i]+".goal.xd",foot_vel[i]);
    ctrl->set_data<Ravelin::Vector3d>(foot_names[i]+".goal.xdd",foot_acc[i]);
  }
}

/** This is a quick way to register your plugin function of the form:
  * void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t)
  */
#include "register-plugin"
