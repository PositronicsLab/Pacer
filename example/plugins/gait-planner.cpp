/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/controller.h>
#include <Pacer/utilities.h>
#include "plugin.h"

using namespace Pacer;
using namespace Ravelin;

static Vector3d workv3_;
static VectorNd q,qd_base;
static VectorNd workv_;

#define DISPLAY

enum FOOT_PHASE {
  STANCE,
  SWING,
  NONE
};

enum GAIT_PHASE {
  FULL_SUPPORT,
  PARTIAL_SUPPORT,
  FLIGHT
};

boost::shared_ptr<Pose3d> base_frame,base_horizontal_frame, gait_pose;
Ravelin::Origin3d roll_pitch_yaw;

std::vector<std::string> foot_names;

std::map<std::string,bool> active_feet;

Origin3d planar_robot(Origin3d x,Origin3d xd){
  double
  dX = xd[0]*cos(x[2])-xd[1]*sin(x[2]),
  dY = xd[0]*sin(x[2])+xd[1]*cos(x[2]);
  return Origin3d(dX,dY,xd[2]);
}

Pose3d end_state(Pose3d state,Origin3d planar_command, double t_max,double dt = 0.001){
  state.update_relative_pose(Pacer::GLOBAL);
  Ravelin::Origin3d roll_pitch_yaw;
  state.q.to_rpy(roll_pitch_yaw);
  
  Origin3d planar_state(state.x[0],state.x[1],roll_pitch_yaw[2]);
  
  for (double t=0; t<t_max; t+=dt) {
    planar_state += planar_robot(planar_state,dt*planar_command);
  }
  
  Ravelin::Pose3d state_t_max(
                              Ravelin::AAngled(0,0,1,planar_state[2]),
                              Origin3d(planar_state[0],planar_state[1],state.x[2]),GLOBAL);
  
  Origin3d dir = planar_robot(planar_state,Origin3d(0.1,0,0));
  
  Utility::visualize.push_back
  (Pacer::VisualizablePtr
   (new Pacer::Ray
    (Vector3d(planar_state[0],planar_state[1],0.01),
     Vector3d(planar_state[0]+dir[0],planar_state[1]+dir[1],0.01),
     Vector3d(0,0,0),0.05)
    )
   );
  
  return state_t_max;
}

Vector3d select_foothold(const std::vector<std::vector<double> >& footholds, Pose3d state, Origin3d x){
  // Find future robot position (command, time)
  // update: Planar process model
  boost::shared_ptr<Pose3d> state_ptr(new Pose3d(state));
  Vector3d x_global = Pose3d::transform_point(Pacer::GLOBAL,Vector3d(x.data(),state_ptr));
  
  // Find foothold closest to "x" at future base position (x,footholds)
  double min_dist = INFINITY;
  int    min_vec = 0;
  for(int i=0;i<footholds.size();i++){
    Vector3d foothold(footholds[i][0],footholds[i][1],footholds[i][2],GLOBAL);
    double norm = (foothold-x_global).norm();
    if(norm < min_dist){
      min_dist = norm;
      min_vec = i;
    }
  }
  
  const std::vector<double> &this_foothold = footholds[min_vec];
  
  Vector3d final_foothold(this_foothold[0],this_foothold[1],this_foothold[2], Pacer::GLOBAL);
  
  double max_dist = 0.05;
  // Interpolate closest point to desired foothold (featured are horizontal and planar circles)
  if(min_dist > this_foothold[3]){
    if (min_dist < max_dist) {
      Vector3d dir(x_global - final_foothold);
      dir.normalize();
      final_foothold = final_foothold + dir*this_foothold[3];
      Utility::visualize.push_back(Pacer::VisualizablePtr(new Pacer::Point(
                                                                           Vector3d(final_foothold[0],final_foothold[1],final_foothold[2]),
                                                                           Vector3d(1,1,1),0.05)));
      final_foothold[2] = this_foothold[2];
    } else {
      return Vector3d(x.data(),state_ptr);
      final_foothold = Vector3d(x_global[0],x_global[1],x_global[2], Pacer::GLOBAL);
    }
  }
  
  return Pose3d::transform_point(state_ptr,final_foothold);
}

bool in_interval(double t,double t0,double tF){
  // during STANCE (no wrap around)
  if (// during interval
      ( t >= t0  &&  t  < tF)
      // during interval (with wrap around)
      || (!(t  < t0  &&  t >= tF) && tF < t0)){
    return true;
  }
  return false;
}

FOOT_PHASE gait_phase(double touchdown,double duty_factor,double gait_progress){
  double liftoff = decimal_part(touchdown + duty_factor),
  left_in_phase = 0;
  
  OUT_LOG(logDEBUG) << "gait_progress " << gait_progress;
  OUT_LOG(logDEBUG) << "touchdown " << touchdown;
  OUT_LOG(logDEBUG) << "duty_factor " << duty_factor;
  OUT_LOG(logDEBUG) << "liftoff " << liftoff;
  
  // ----- STANCE PHASE ------
  if(in_interval(gait_progress,touchdown,liftoff)){
    return STANCE;
  }
  // ----- SWING PHASE -----
  return SWING;
  
}

double gait_phase(double touchdown,double duty_factor,double gait_progress,FOOT_PHASE this_phase){
  double liftoff = decimal_part(touchdown + duty_factor),left_in_phase = 0;
  
  OUT_LOG(logDEBUG) << "gait_progress " << gait_progress;
  OUT_LOG(logDEBUG) << "touchdown " << touchdown;
  OUT_LOG(logDEBUG) << "duty_factor " << duty_factor;
  OUT_LOG(logDEBUG) << "liftoff " << liftoff;
  
  // ----- STANCE PHASE ------
  if(this_phase == STANCE){
    // left in stance phase = negaive time left in phase
    left_in_phase = (liftoff - gait_progress);
    if(left_in_phase < 0)
      left_in_phase = (1.0 - gait_progress) + liftoff;
  }
  // ----- SWING PHASE -----
  else {
    left_in_phase = (touchdown - gait_progress);
    if(left_in_phase < 0)
      left_in_phase = (1.0 - gait_progress) + touchdown;
  }
  return left_in_phase;
}

bool next_flight_phase(const std::vector<double>& touchdown,const std::vector<double>& duty_factor, double initial_time,
                       double& time_until_takeoff,double& flight_phase_duration){
  double gait_progress = initial_time;
  
  double one_cycle = 1;
  
  double this_liftoff = 0;
  while(one_cycle>0){
    bool all_swing_phase = true;
    double next_touchdown = std::numeric_limits<double>::max();
    double next_liftoff = std::numeric_limits<double>::max();
    // Check the next phase change for each foot
    for (int i=0; i<foot_names.size(); i++) {
      FOOT_PHASE phase = gait_phase(touchdown[i],duty_factor[i],gait_progress);
      if(phase == SWING){
        double next_phase = gait_phase(touchdown[i],duty_factor[i],gait_progress,phase)+NEAR_ZERO;
        next_touchdown = std::min(next_touchdown,next_phase);
      }else{
        all_swing_phase = false;
        double next_phase = gait_phase(touchdown[i],duty_factor[i],gait_progress,phase)+NEAR_ZERO;
        next_liftoff = std::min(next_liftoff,next_phase);
      }
    }
    // if looking at a flight phase, then exit
    if(all_swing_phase == true){
      time_until_takeoff = this_liftoff;
      flight_phase_duration = next_touchdown;
      break;
    }
    // increment checking point
    this_liftoff += next_liftoff;
    gait_progress += next_liftoff+NEAR_ZERO;
    gait_progress = decimal_part(gait_progress);
    if(gait_progress >= 1.0) gait_progress = NEAR_ZERO;
    one_cycle -= next_liftoff;
    // and loop
  }
  
  if (one_cycle <= 0) {
    OUT_LOG(logDEBUG1) << "No flight phase in gait";
    
    time_until_takeoff = std::numeric_limits<double>::max();
    flight_phase_duration = 0;
    return false;
  }
  
  OUT_LOG(logDEBUG1) << "Next flight phase was found at: " << gait_progress << std::endl
  << "\tWith duration: " << flight_phase_duration << std::endl
  << "\toccuring in: " << time_until_takeoff <<" * (gait-duration) seconds";
  
  return true;
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
                 const VectorNd& command,
                 const std::vector<double>& touchdown,
                 const std::vector<std::vector<double> >& footholds,
                 const std::vector<double>& duty_factor,
                 double gait_duration,
                 double step_height,
                 bool STANCE_ON_CONTACT,
                 // MODEL
                 std::vector<Origin3d>& origins,
                 const Vector3d& center_of_mass_x,
                 double t,
                 std::vector<Vector3d>&
                 foot_pos,
                 std::vector<Vector3d>&
                 foot_vel,
                 std::vector<Vector3d>&
                 foot_acc)
{
  OUT_LOG(logDEBUG) << " -- walk_toward() entered";
  
  boost::shared_ptr<Pacer::Controller> ctrl = ctrl_weak_ptr.lock();
  
  const int NUM_FEET = origins.size(),
  NUM_JOINT_DOFS = q.size() - Pacer::NEULER;
  
  // Up is oriented wrt global up vector (+Z-axis for BASE_HORIZONTAL_FRAME)
  Vector3d up = Pose3d::transform_vector(base_frame,Vector3d(0,0,1,base_horizontal_frame));
  
  // Find time since last call
  static double last_time = -0.001;
  double dt = t - last_time;
  last_time = t;
  
  // persistent Vector storing spline coefs
  // indexing: [foot][dimension]
  static std::vector< std::vector<VectorNd> > spline_coef(NUM_FEET);
  
  // persistent Vector storing time delimitations to each spline
  // indexing: [foot]
  static std::vector<VectorNd> spline_t(NUM_FEET);
  static std::vector<FOOT_PHASE> last_phase;
  
  // Check if this method has been called recently,
  // reset if no call has been made for dt > 1 phase
  if(dt > gait_duration * (*std::min_element(duty_factor.begin(),duty_factor.end())) )
    last_phase.clear();
  
  if(last_phase.empty()){
    for(int i=0;i<NUM_FEET;i++){
      // -- Populate last phase vector --
      last_phase.push_back(NONE);
      // -- Populate spline vectors --
      spline_coef[i].resize(3);
      for(int d=0; d<3;d++)
        spline_coef[i][d] = VectorNd();
      spline_t[i] = VectorNd::zero(2);
    }
  }
  
#ifdef DO_NOT_DISPLAY
  //  Vector3d turn_origin(0,command[0]/command[5],0,base_horizontal_frame);
  //  turn_origin = Pose3d::transform_point(base_frame, turn_origin);
  Origin3d turn_origin(-command[1]/command[5],command[0]/command[5],0);
  
  if (std::fabs(command[5]) > Pacer::NEAR_ZERO) {
    boost::shared_ptr< Pose3d> turning_frame(new Pose3d(Ravelin::Quatd::identity(),turn_origin,base_horizontal_frame));
    
    turning_frame->update_relative_pose(Pacer::GLOBAL);
    Utility::visualize.push_back( Pacer::VisualizablePtr( new Ray(  Vector3d(turning_frame->x.data())-Vector3d(0,0,1), Vector3d(turning_frame->x.data())+Vector3d(0,0,1),   Vector3d(0,1,0),0.05)));
    Utility::visualize.push_back( Pacer::VisualizablePtr( new Ray(  Vector3d(turning_frame->x.data()), Vector3d(base_horizontal_frame->x.data()),   Vector3d(0,1,0),0.05)));
    
    turning_frame->update_relative_pose(base_horizontal_frame);
  }
#endif
  
  ////////////////// FOOT PHASE ASSIGNMENT ///////////////////////
  double gait_progress = decimal_part(t/gait_duration);
  // Get the decimal part of gait_progress
  if(gait_progress >= 1) gait_progress = Pacer::NEAR_ZERO;
  
  // Check for flight phase
  double flight_phase_duration;
  double until_takeoff;
  bool has_flight_phase = next_flight_phase(touchdown,duty_factor,gait_progress,
                                            until_takeoff,flight_phase_duration);
  
  // Figure out the phase of each foot
  std::vector<FOOT_PHASE> phase_vector(NUM_FEET);
  std::vector<double> left_in_phase_v(NUM_FEET);
  std::vector<bool> early_stance(NUM_FEET);
  std::vector<double> liftoff(NUM_FEET);
  std::vector<Pose3d> end_of_step_state(NUM_FEET);
  std::vector<Pose3d> mid_of_next_step_state(NUM_FEET);
  static std::vector<Vector3d> start_of_step_foothold(NUM_FEET);
  
  for(int i=0;i<NUM_FEET;i++){
    // Assign liftoff times
    liftoff[i] = decimal_part(touchdown[i] + duty_factor[i]);
    if(liftoff[i] >= 1.0) liftoff[i] = 0;
    
    FOOT_PHASE this_phase = gait_phase(touchdown[i],duty_factor[i],gait_progress);
    phase_vector[i] = this_phase;
    
    bool active_foot = active_feet[foot_names[i]];
    // Make sure Early Contact is valid
    bool early_contact = STANCE_ON_CONTACT && ((this_phase == SWING) && active_foot);
    if(early_contact){
      if (// If in latter part of swing phase
          in_interval(gait_progress,liftoff[i],decimal_part(liftoff[i]+(1.0-duty_factor[i])/2.0))
          // AND robot is not tilted toward new contact
          // avoid Positive pitch (forward) and X positive contact (forward)
          && (sgn(roll_pitch_yaw[1]) != sgn(origins[i][0]) || sgn(roll_pitch_yaw[1]) == 0)
          // avoid negative roll (left) and Y positive contact (left)
          && (sgn(roll_pitch_yaw[0]) == sgn(origins[i][1]) || sgn(roll_pitch_yaw[0]) == 0)
          )
      {
        // Switch foot to stance phase early
        this_phase = STANCE;
        early_stance[i] = true;
      }
    }
    
    if(this_phase == STANCE)
      early_stance[i] = false;
    
    double left_in_phase = gait_phase(touchdown[i],duty_factor[i],gait_progress,this_phase);
    
    if(early_stance[i]){
      this_phase = STANCE;
      left_in_phase = left_in_phase + duty_factor[i];
    }

    left_in_phase_v[i] = left_in_phase;
    OUT_LOG(logDEBUG) << "\t Stance Phase? : " << ((this_phase == STANCE)? "STANCE" : "SWING");
    OUT_LOG(logDEBUG) << "\t left in phase (%) : " << left_in_phase * 100.0;
    OUT_LOG(logDEBUG) << "\t left in phase (sec) : " << left_in_phase * gait_duration;
    ctrl->set_data<bool>(foot_names[i]+".stance",((this_phase == STANCE)? true : false));
    
    OUT_LOG(logDEBUG) << "\t PHASE PROGRESS: { " << touchdown[i] << " .. " << gait_progress << " .. " << (touchdown[i] + duty_factor[i]) <<" }, " << ((this_phase == STANCE)? "STANCE" : "SWING");
    
    // At the end of the current swing phase:
    end_of_step_state[i]
    = end_state(*(base_horizontal_frame.get()),
                Origin3d(command[0],command[1],command[5]),
                fabs(left_in_phase*gait_duration));
    
    end_of_step_state[i].update_relative_pose(Pacer::GLOBAL);
    
    OUTLOG(end_of_step_state[i].x,"end_of_step_state["+boost::icl::to_string<double>::apply(i)+"]",logERROR);
    // At the middle of the next stance phase:
    mid_of_next_step_state[i]
    = end_state(end_of_step_state[i],
                Origin3d(command[0],command[1],command[5]),
                fabs( (((this_phase == STANCE)? 1.0-duty_factor[i] : duty_factor[i])/2.0) *gait_duration));
    
    mid_of_next_step_state[i].update_relative_pose(Pacer::GLOBAL);
    
    OUTLOG(mid_of_next_step_state[i].x,"mid_of_next_step_state["+boost::icl::to_string<double>::apply(i)+"]",logERROR);
    
#ifdef DISPLAY
    Vector3d color;
    switch(i){
      case 0: color = Vector3d(1,0,0);
        break;
      case 1: color = Vector3d(0,1,0);
        break;
      case 2: color = Vector3d(0,0,1);
        break;
      case 3: color = Vector3d(1,0,1);
        break;
      default:
        color = Vector3d(0,0,0);
    }
    
    Utility::visualize.push_back
    (Pacer::VisualizablePtr
     (new Pacer::Point
      (Vector3d(end_of_step_state[i].x[0],
                end_of_step_state[i].x[1],
                end_of_step_state[i].x[2]-0.05),
       color,0.1)
      )
     );
#endif
    
  }
  
  
  ///////////////////////////////////////////////////////////////////////
  //////////////////////// FOOT PHASE PLANNING //////////////////////////
  ///////////////////////////////////////////////////////////////////////
  
  //////////////////////// PLANNING STANCE PHASE /////////////////////////
  for(int i=0;i<NUM_FEET;i++){
    FOOT_PHASE &this_phase = phase_vector[i];
    double  &left_in_phase = left_in_phase_v[i];
    
    Vector3d
    &x   = foot_pos[i],
    &xd  = foot_vel[i],
    &xdd = foot_acc[i];
    
    Origin3d xt(x);
    
    // If stance foot OR foot is contacting ground
    if(this_phase != STANCE){
      continue;
    }

    VectorNd zero_base_generalized_q;
    zero_base_generalized_q.set_zero(NUM_JOINT_DOFS+Pacer::NEULER);
    zero_base_generalized_q.set_sub_vec(0,q.segment(0,NUM_JOINT_DOFS));
    /// STANCE FEET ARE IN GAIT POSE
    // Put goal in gait pose
    VectorNd gait_pose_vec = Utility::pose_to_vec(gait_pose);
    OUTLOG(gait_pose,"gait_pose",logDEBUG);
    zero_base_generalized_q.set_sub_vec(NUM_JOINT_DOFS,gait_pose_vec);
    OUTLOG(gait_pose_vec,"gait_pose_vec",logDEBUG);
    OUTLOG(zero_base_generalized_q,"zero_base_generalized_q",logDEBUG);
    
    Ravelin::MatrixNd J = ctrl->calc_link_jacobian(zero_base_generalized_q,foot_names[i]);
    
    Ravelin::VectorNd base_command = command;
    
    // add forces to propel robot over upcoming flight phase
    if(has_flight_phase){
      // Duration of flight
      double flight_phase_seconds = flight_phase_duration * gait_duration;
      // upward velocity we must reach to return to ground after flight_phase_seconds
      double gravity = 9.8;// m / s*s
      double upward_velocity = gravity * flight_phase_seconds * 0.5;
      
      static double last_until_takeoff = -1;
      if (until_takeoff > last_until_takeoff)
        last_until_takeoff = until_takeoff;
      
      double duration_until_flight = 1.0 - until_takeoff/last_until_takeoff;
      OUTLOG(duration_until_flight,"duration_until_flight",logDEBUG);

      double z_axis_command = sigmoid_interp(0.0,upward_velocity,duration_until_flight);
      OUTLOG(upward_velocity,"upward_velocity for flight",logDEBUG);
      OUTLOG(z_axis_command,"z_axis_command for flight",logDEBUG);
      base_command[2] += z_axis_command;
    }
    
    OUTLOG(base_command,"base_command(with flight factored in)",logDEBUG);
    
    
    J.block(0,3,NUM_JOINT_DOFS,NUM_JOINT_DOFS+6).mult(base_command,workv3_,-1.0,0);
    
    workv3_.pose = base_frame;
    xd.pose = base_frame;
    
    if (dt < Pacer::NEAR_ZERO)
      xdd.set_zero();
    else
      xdd = (workv3_ - xd)/dt;
    
    xd = workv3_;
    xdd.pose = base_frame;
    xd.pose = base_frame;
    x.pose = base_frame;
    x = x + xd * dt;
    
#ifdef DISPLAY
    Vector3d p = Pose3d::transform_point(Pacer::GLOBAL,x);
    Vector3d v = Pose3d::transform_vector(Pacer::GLOBAL,xd) * (left_in_phase*gait_duration);
    Utility::visualize.push_back( Pacer::VisualizablePtr( new Ray(  v+p,   p,   Vector3d(0,1,0),0.05)));
#endif
    
    OUT_LOG(logDEBUG) << "x " << x;
    OUT_LOG(logDEBUG) << "xd " << xd;
    OUT_LOG(logDEBUG) << "xdd " << xdd;
  }
  
  ///////////////////////////////////////////////////////////////////////
  //////////////////////// PLANNING SWING PHASE /////////////////////////
  ///////////////////////////////////////////////////////////////////////
  for(int i=0;i<NUM_FEET;i++){
    FOOT_PHASE &this_phase = phase_vector[i];
    
    // Skip plan if in stance phase for this foot
    if (this_phase == STANCE)
      continue;
    
    Vector3d
    &x   = foot_pos[i],
    &xd  = foot_vel[i],
    &xdd = foot_acc[i];
    xdd.pose = base_frame;
    xd.pose = base_frame;
    x.pose = base_frame;
    
    // Calc where the robot will be if the current command remains the same
    double  &left_in_phase = left_in_phase_v[i];
    double swing_phase_duration = (1.0-duty_factor[i]);
    
    // Check if Spline must be re-evaluated
    bool replan_path = (last_phase[i] != this_phase);
    
    double redirect_path_interval = 0;
    ctrl->get_data<double>(plugin_namespace+".redirect-swing-interval",redirect_path_interval);
    
    bool redirect_path = false;
    // TODO: Fix Redirect path
    // NOTE: 'redirect_path_interval' is the progress through the step at which the step is finalized.
    // (the last 1/4 of the swing phase is not replanned based on new commands)
    
    if(// if a new swing pahse is not being planned
       (!replan_path)
       // AND if we are in the early half of the swing phase
       ){
      double finalize_path_time
      = decimal_part(liftoff[i] + swing_phase_duration*redirect_path_interval);
      if(in_interval(gait_progress,liftoff[i],finalize_path_time))
        redirect_path = true;
    }
    
    // Stance phase (opposite to signal switch on next re-program)
    // Plan a new spline for this foot
    if( !(replan_path || redirect_path) ){
      OUT_LOG(logDEBUG1) << "Using old spline: " << i;
      // Try to use current (existing spline)
      for(int d=0; d<3;d++){
        OUT_LOG(logDEBUG) << "\t Spline coefs: " << spline_coef[i][d];
        if(!Utility::eval_cubic_spline(spline_coef[i][d],spline_t[i],t,x[d],xd[d],xdd[d])){
          OUT_LOG(logDEBUG) << "\t Time: " << t;
          OUT_LOG(logDEBUG) << "\t Time coefs: " << spline_t[i];
          throw std::runtime_error("Error: swing foot path spline ended early!");
        }
      }
    } else {
      OUT_LOG(logDEBUG1) << "Creating new spline: " << i;
      
      Vector3d x_start = x,xd_start(0,0,0),xdd_;
      
      if(replan_path){
        OUT_LOG(logDEBUG1) << "NEW SPLINE";
        start_of_step_foothold[i] = Pose3d::transform_point(base_horizontal_frame,x_start);
      }
      if(redirect_path)
        OUT_LOG(logDEBUG1) << "REDIRECT SPLINE Mid-Step";
      
      // take time at end of spline_t history
      double t0 = t - ( swing_phase_duration-left_in_phase ) * gait_duration;
      
      /// SWING FEET ARE IN HORIZONTAL POSE
      // set down foot where the origin will be in the middle of the next stance phase
      // (with respect to the start of the next stance phase)
      boost::shared_ptr<Pose3d> mid_of_next_step_state_ptr(new Pose3d(mid_of_next_step_state[i]));
      boost::shared_ptr<Pose3d> end_of_step_state_ptr(new Pose3d(end_of_step_state[i]));
      Vector3d foot_goal(origins[i],mid_of_next_step_state_ptr);
      foot_goal = Pose3d::transform_point(end_of_step_state_ptr,foot_goal);
      
#ifdef DISPLAY
      Vector3d p1 = Pose3d::transform_point(Pacer::GLOBAL,start_of_step_foothold[i]);
      p1[2] = 0;
      Utility::visualize.push_back( Pacer::VisualizablePtr( new Point( p1,   Vector3d(1,0,0),0.075)));
      Vector3d p2 = Pose3d::transform_point(Pacer::GLOBAL,foot_goal);
      p2[2] = 0;
      Utility::visualize.push_back( Pacer::VisualizablePtr( new Point( p2,   Vector3d(0,1,0),0.075)));
#endif
      
      // Generate control points for swing foot spline
      std::vector<Origin3d> control_points;
      
      Origin3d up_step(up.data());
      up_step*=step_height;
      
      if(redirect_path){
        // transform location of step start to current pose
        x_start = start_of_step_foothold[i];
        x_start.pose = base_horizontal_frame;
        OUT_LOG(logDEBUG1) << "x_start local: " << x_start;
      }
      control_points.push_back(Origin3d(x_start));
      
      OUT_LOG(logDEBUG) << "Calculating step [" << i << "] : " << control_points[0] << " --> " << foot_goal;
      
      Origin3d foot_destination = Origin3d(foot_goal);
      Origin3d foot_direction = foot_destination - Origin3d(x_start);
      
      // Perturb spline control point values to make them solvable
      Origin3d spline_robustness = origins[i];
      spline_robustness[2] = 0;
      spline_robustness[1] = -spline_robustness[1] * sgn(foot_destination[1]);
      spline_robustness[0] = fabs(spline_robustness[0]) * sgn(foot_destination[0]);
      spline_robustness.normalize();
      spline_robustness = spline_robustness * 1e-3;
      spline_robustness[2] = 1e-3;
      control_points.push_back(control_points[0] + up_step - spline_robustness);
      
      if(footholds.empty()){
        control_points.push_back(foot_destination + up_step + 0.1*foot_direction + spline_robustness);
        control_points.push_back(foot_destination);
      } else {
        Vector3d x_fh = select_foothold(footholds, end_of_step_state[i], foot_destination);
        
        foot_destination = Origin3d(x_fh);
        
        // Reach new foothold, account for movement of robot during step
        control_points.push_back(foot_destination + up_step);
        control_points.push_back(foot_destination);
      }
      
      // create spline using set of control points, place at back of history
      int n = control_points.size();
      std::vector<Origin3d> new_control_points;
      std::vector<double>           T;
      
      // Add current position via point at current time in spline
      new_control_points.push_back(control_points[0]);
      T.push_back(t0);
      
      
      //if(redirect_path && false){
      //  // NOTE: This method results in bad trajectories, might work with higher order splines
      //  bool is_set = false;
      //  for(int j=1,jj=1;jj<n;j++){
      //    double this_time =
      //    t0 + ( swing_phase_duration*gait_duration / (double)(n-1)) * (double)jj ;
      //
      //    double last_time =
      //    t0 + (swing_phase_duration*gait_duration / (double)(n-1)) * (double)(jj-1) ;
      //    OUTLOG(this_time,"redirect",logERROR);
      //
      //    if ( (this_time-Pacer::NEAR_ZERO > t) && (!is_set) && (t > last_time+Pacer::NEAR_ZERO) ){
      //      is_set = true;
      //      T.push_back(t);
      //      new_control_points.push_back(Origin3d(x));
      //      OUTLOG(t,"insert_at",logERROR);
      //    } else {
      //      T.push_back(this_time);
      //      new_control_points.push_back(control_points[jj]);
      //      jj++;
      //    }
      //  }
      //  control_points = new_control_points;
      //  n = control_points.size();
      //} else {
      for(int j=1;j<n;j++){
        double this_time = t0 + ( swing_phase_duration*gait_duration / (double)(n-1)) * (double)j ;
        T.push_back(this_time);
      }
      //}
      
      spline_t[i] = VectorNd(n,&T[0]);
      
      //      Origin3d xd0, xdF;
      // Get touchdown and takeoff vel;
      {
        VectorNd zero_base_generalized_q;
        zero_base_generalized_q.set_zero(NUM_JOINT_DOFS+Pacer::NEULER);
        zero_base_generalized_q.set_sub_vec(0,q.segment(0,NUM_JOINT_DOFS));
        /// STANCE FEET ARE IN GAIT POSE
        // Put goal in gait pose
        VectorNd gait_pose_vec = Utility::pose_to_vec(gait_pose);
        zero_base_generalized_q.set_sub_vec(NUM_JOINT_DOFS,gait_pose_vec);
        Ravelin::MatrixNd J = ctrl->calc_link_jacobian(zero_base_generalized_q,foot_names[i]);
        
        J.block(0,3,NUM_JOINT_DOFS,NUM_JOINT_DOFS+Pacer::NSPATIAL).mult(command,workv3_,-1.0,0);
        //        OUTLOG(J,"J",logDEBUG);
        xd = workv3_;
        xd.pose = base_frame;
        if (replan_path) {
          xd_start = xd;
        }
      }
      
      OUTLOG(xd_start,"xd1",logDEBUG);
      OUTLOG(xd,"xd2",logDEBUG);

      n = control_points.size();
      
      OUT_LOG(logDEBUG1) << "Calc Spline AND Eval first step: " << i << " @ time: " << t;
      OUTLOG(spline_t[i],"T",logDEBUG);
      for(int d=0;d<3;d++){
        VectorNd          &T = spline_t[i];
        VectorNd           X(n);
        VectorNd          &coefs = spline_coef[i][d];
        
        for(int j=0;j<n;j++)
          X[j] = control_points[j][d];
        
        OUT_LOG(logDEBUG1) << "dimension: " << d;
        OUTLOG(X,"X_control-points",logDEBUG);
        
        Utility::calc_cubic_spline_coefs(T,X,Ravelin::Vector2d(xd_start[d],xd[d]),coefs);
        
        // then re-evaluate spline
        // NOTE: this will only work if we replanned for a t_0  <  t  <  t_0 + t_I
        Utility::eval_cubic_spline(spline_coef[i][d],spline_t[i],t+NEAR_ZERO,x[d],xd[d],xdd[d]);
      }
    }
    
    x.pose = base_frame;
    xd.pose = base_frame;
    xdd.pose = base_frame;
    
    OUT_LOG(logDEBUG) << "x " << x;
    OUT_LOG(logDEBUG) << "xd " << xd;
    OUT_LOG(logDEBUG) << "xdd " << xdd;
  }
  
  // Reassign last
  for(int i=0;i<NUM_FEET;i++){
    last_phase[i] = phase_vector[i];
  }
  
#ifdef DISPLAY
  for(int i=0;i<footholds.size();i++){
    Utility::visualize.push_back
    (Pacer::VisualizablePtr
     (new Pacer::Point
      (Vector3d(footholds[i][0],footholds[i][1],footholds[i][2])
       ,Vector3d(1,0,0)
       ,10.0*footholds[i][3])
      )
     );
  }
  
  for(int i=0;i<NUM_FEET;i++){
    if(phase_vector[i] == STANCE)
      continue;
    VectorNd &T = spline_t[i];
    double max_t = T[T.rows()-1];
    double min_t = T[0]+Pacer::NEAR_ZERO;
    for(double t=min_t ; t<=max_t ; t += (max_t-min_t)/20){
      Vector3d x,xd,xdd;
      for(int d=0;d<3;d++){
        Utility::eval_cubic_spline(spline_coef[i][d],spline_t[i],t,x[d],xd[d],xdd[d]);
      }
      x.pose = base_frame;
      xd.pose = base_frame;
      xdd.pose = base_frame;
      Vector3d p = Pose3d::transform_point(Pacer::GLOBAL,x);
      Vector3d v = Pose3d::transform_vector(Pacer::GLOBAL,xd)/10;
      //      Vector3d a = Pose3d::transform_vector(Pacer::GLOBAL,xdd)/100;
      Utility::visualize.push_back( Pacer::VisualizablePtr( new Pacer::Point( p,   Vector3d(0,1,0),0.025)));
      Utility::visualize.push_back( Pacer::VisualizablePtr( new Ray(  v+p,   p,   Vector3d(1,0,0),0.005)));
      //     Utility::visualize.push_back( Pacer::VisualizablePtr( new Ray(a+v+p, v+p, Vector3d(1,0.5,0)));
    }
  }
#endif
  OUT_LOG(logDEBUG) << " -- walk_toward() exited";
}

void loop(){
boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  // Find time since last call
  static double last_time = t;
  double dt = t - last_time;
  last_time = t;
  
  /// Moving average for command
  const int queue_size = 1;
  static std::deque<Origin3d> command_queue;
  static Origin3d sum_command = Origin3d(0,0,0);
  
  {
    Origin3d command = Origin3d(0,0,0);
    ctrl->get_data<Origin3d>("SE2_command",command);
    command_queue.push_front(command);
    sum_command += command;
  }
  
  if(command_queue.size() > queue_size){
    sum_command -= command_queue.back();
    command_queue.pop_back();
  }
  
  Origin3d command = sum_command / (double) command_queue.size();
  
  OUTLOG(command,"SE2_command",logERROR);
  
  /// Command velocity differential
  //  static Origin3d command = Origin3d(0,0,0);
  //
  //  Origin3d dcommand = Origin3d(0,0,0);
  //  ctrl->get_data<Origin3d>("SE2_command",dcommand);
  //  command *= 0.999;
  //  command += dcommand*dt;
  
  VectorNd go_to;
  go_to.set_zero(6);
  go_to[0] = command[0];
  go_to[1] = command[1];
  go_to[5] = command[2];
  
  std::vector<double>
  duty_factor = ctrl->get_data<std::vector<double> >(plugin_namespace+".duty-factor"),
  this_gait = ctrl->get_data<std::vector<double> >(plugin_namespace+".gait");
  double gait_time = ctrl->get_data<double>(plugin_namespace+".gait-duration");
  double step_height = ctrl->get_data<double>(plugin_namespace+".step-height");
  
  double vec_width = 5;
  std::vector<std::vector<double> > footholds;
  std::vector<double> footholds_vec;
  ctrl->get_data<std::vector<double> >(plugin_namespace+".footholds",footholds_vec);
  if(footholds.empty() || footholds_vec.size()/vec_width != footholds.size()){
    footholds.clear();
    for(int i=0, ind=0;i<footholds_vec.size()/vec_width;i++){
      footholds.push_back(std::vector<double>() );
      for(int j=0;j<vec_width;j++,ind++){
        footholds[i].push_back(footholds_vec[ind]);
      }
    }
  }
  
  foot_names = ctrl->get_data<std::vector<std::string> >(plugin_namespace+".feet");
  
  double width = -1;
  ctrl->get_data<double>(plugin_namespace+".width",width);
  double length = -1;
  ctrl->get_data<double>(plugin_namespace+".length",length);
  double height = -1;
  ctrl->get_data<double>(plugin_namespace+".height",height);
  
  base_frame = boost::shared_ptr<Pose3d>( new Pose3d(
                                                     ctrl->get_data<Pose3d>("base_link_frame")));
  base_horizontal_frame = boost::shared_ptr<Pose3d>( new Pose3d(
                                                                ctrl->get_data<Pose3d>("base_horizontal_frame")));
  
  base_horizontal_frame->update_relative_pose(Pacer::GLOBAL);
  
  // Set up output vectors for gait planner
  int NUM_FEET = foot_names.size();
  std::vector<Vector3d>
  foot_vel(NUM_FEET),
  foot_pos(NUM_FEET),
  foot_acc(NUM_FEET),
  foot_init(NUM_FEET);
  q = ctrl->get_generalized_value(Pacer::Robot::position);
  qd_base = ctrl->get_base_value(Pacer::Robot::velocity);
  
  //  ctrl->set_model_state(q);
  double min_reach = INFINITY;
  for(int i=0;i<NUM_FEET;i++){
//    min_reach = std::min(min_reach, ctrl->get_data<double>(foot_names[i]+".reach"));
    foot_init[i] = Vector3d(ctrl->get_data<Origin3d>(foot_names[i]+".init.x"),base_frame);
    foot_pos[i] = foot_init[i];
    Origin3d workv;
    if(ctrl->get_data<Origin3d>(foot_names[i]+".goal.x",workv))
      foot_pos[i] = Vector3d(workv,base_frame);
    foot_vel[i] = Vector3d(0,0,0,base_frame);
    if(ctrl->get_data<Origin3d>(foot_names[i]+".goal.xd",workv))
      foot_vel[i] = Vector3d(workv,base_frame);
    foot_acc[i] = Vector3d(0,0,0,base_frame);
  }
  
  
  gait_pose = boost::shared_ptr<Pose3d>( new Pose3d() );
  
  if( !ctrl->get_data<Pose3d>("base_stability_frame",*(gait_pose.get())))
    *(gait_pose.get()) = *(base_frame.get());
  gait_pose->update_relative_pose(base_frame);
  gait_pose->q.to_rpy(roll_pitch_yaw);
  
  // Assign foot origins (ideal foot placemenet at rest)
  std::vector<Origin3d> origins;
  if(origins.empty()){
    origins.resize(NUM_FEET);
    for(int i=0;i<NUM_FEET;i++){
      Vector3d origin = foot_init[i];
//      if(length >= 0)
        origin[0] = Utility::sign<double>(foot_init[i][0])*length/2;
//      if(width >= 0)
        origin[1] = Utility::sign<double>(foot_init[i][1])*width/2;
//      if(height >= 0)
        origin[2] = -std::min(height,min_reach);
      origin.pose = base_frame;
      OUT_LOG(logDEBUG1) << "Length is " << origin[0];
      OUT_LOG(logDEBUG1) << "Width is " << origin[1];
      OUT_LOG(logDEBUG1) << "Height is " << origin[2];

      origins[i] = Pose3d::transform_point(gait_pose,origin);
      
    }
  }
  
  OUTLOG(this_gait,"this_gait",logINFO);
  OUTLOG(duty_factor,"duty_factor",logINFO);
  
  bool STANCE_ON_CONTACT = ctrl->get_data<bool>(plugin_namespace+".stance-on-contact");
  
  for(int i=0;i<foot_names.size();i++){
    std::vector< boost::shared_ptr< const Pacer::Robot::contact_t> > c;
    ctrl->get_link_contacts(foot_names[i],c);
    if(!c.empty())
      active_feet[foot_names[i]] = true;
    else
      active_feet[foot_names[i]] = false;
  }
  
  walk_toward(go_to,this_gait,footholds,duty_factor,gait_time,step_height,STANCE_ON_CONTACT,origins,ctrl->get_data<Vector3d>("center_of_mass.x"),t,foot_pos,foot_vel, foot_acc);
  
  for(int i=0;i<NUM_FEET;i++){
    ctrl->set_data<Origin3d>(foot_names[i]+".goal.x",Origin3d(foot_pos[i]));
    ctrl->set_data<Origin3d>(foot_names[i]+".goal.xd",Origin3d(foot_vel[i]));
    ctrl->set_data<Origin3d>(foot_names[i]+".goal.xdd",Origin3d(foot_acc[i]));

    ctrl->set_foot_value(foot_names[i],Pacer::Controller::position_goal,Origin3d(foot_pos[i]));
    ctrl->set_foot_value(foot_names[i],Pacer::Controller::velocity_goal,Origin3d(foot_vel[i]));
    ctrl->set_foot_value(foot_names[i],Pacer::Controller::acceleration_goal,Origin3d(foot_acc[i]));
  }
}

void setup(){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);

  const  std::vector<std::string>
  eef_names_ = ctrl->get_data<std::vector<std::string> >("init.end-effector.id");
  for(unsigned i=0;i<eef_names_.size();i++){
    variable_names.push_back(eef_names_[i]+".stance");
  }
}