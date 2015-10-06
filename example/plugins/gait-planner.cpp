/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/controller.h>
#include <Pacer/utilities.h>
using namespace Pacer;
using namespace Ravelin;

static Vector3d workv3_;
static VectorNd q,qd_base;
static VectorNd workv_;

boost::shared_ptr<Pacer::Controller> ctrl_ptr;
  boost::shared_ptr<Pose3d> base_frame,base_horizontal_frame, gait_pose;
  std::string plugin_namespace;
  std::vector<std::string> foot_names;
std::map<std::string,bool> active_feet;

Origin3d planar_robot(Origin3d x,Origin3d xd){
  double dX =xd[0]*cos(x[2])+xd[1]*sin(x[2]),
  dY =xd[0]*sin(x[2])+xd[1]*cos(x[2]);
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
  
  Utility::visualize.push_back(
    Pacer::VisualizablePtr(
      new Pacer::Ray(
        Vector3d(planar_state[0],planar_state[1],state.x[2]-0.1),
        Vector3d(planar_state[0]+dir[0],planar_state[1]+dir[1],state.x[2]-0.1),
        Vector3d(0,0,0),0.05)));

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

double gait_phase(double touchdown,double duty_factor,double gait_progress,bool this_phase){
  double liftoff = touchdown + duty_factor,left_in_phase = 0;
  liftoff = liftoff - (double) ((int) liftoff);

  OUT_LOG(logDEBUG) << "gait_progress " << gait_progress;
  OUT_LOG(logDEBUG) << "touchdown " << touchdown;
  OUT_LOG(logDEBUG) << "duty_factor " << duty_factor;
  OUT_LOG(logDEBUG) << "liftoff " << liftoff;

  // ----- STANCE PHASE ------
  if(this_phase){
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
  
  static bool inited = false;
  
  const int NUM_FEET = origins.size(),
  NUM_JOINT_DOFS = q.size() - Pacer::NEULER;
  
  Vector3d up = Pose3d::transform_vector(base_frame,Vector3d(0,0,1,base_horizontal_frame));
  
  // Find time since last call
  static double last_time = -0.001;
  double dt = t - last_time;
  last_time = t;
  
  // Check if this method has been called recently,
  // reset if no call has been made for dt > 1 phase
  if(dt > gait_duration * (*std::min_element(duty_factor.begin(),duty_factor.end())) )
    inited = false;

  // persistent Vector storing spline coefs
  // [foot][dimension]
  static std::vector< std::vector<VectorNd> > spline_coef(NUM_FEET);
  
  // persistent Vector storing time delimitations to each spline
  // [foot]
  static std::vector<VectorNd> spline_t(NUM_FEET);
  
  double gait_progress = t/gait_duration;
  
  // Get the decimal part of gait_progress
  gait_progress = gait_progress - (double) ((int) gait_progress);
  if(gait_progress >= 1) gait_progress = Pacer::NEAR_ZERO;

//  Vector3d turn_origin(0,command[0]/command[5],0,base_horizontal_frame);
//  turn_origin = Pose3d::transform_point(base_frame, turn_origin);
  Origin3d turn_origin(0,command[0]/command[5],0);
  
  if (command[5] > Pacer::NEAR_ZERO) {
    boost::shared_ptr< Pose3d> turning_frame(new Pose3d(Ravelin::Quatd::identity(),turn_origin,base_horizontal_frame));

    turning_frame->update_relative_pose(Pacer::GLOBAL);
    Utility::visualize.push_back( Pacer::VisualizablePtr( new Ray(  Vector3d(turning_frame->x.data())-Vector3d(0,0,1), Vector3d(turning_frame->x.data())+Vector3d(0,0,1),   Vector3d(0,1,0),0.05)));
    Utility::visualize.push_back( Pacer::VisualizablePtr( new Ray(  Vector3d(turning_frame->x.data()), Vector3d(base_horizontal_frame->x.data()),   Vector3d(0,1,0),0.05)));
    
    turning_frame->update_relative_pose(base_horizontal_frame);
  }

  
  // (Re)Populate spline vectors
  if(!inited){
    for(int i=0;i<NUM_FEET;i++){
      spline_coef[i].resize(3);
      for(int d=0; d<3;d++){
        spline_coef[i][d] = VectorNd();
      }

      spline_t[i] = VectorNd::zero(2);
    }
  }
  
  ////////////////// PHASE PLANNING ///////////////////////
  static std::vector<bool> last_phase = std::vector<bool>(NUM_FEET);
  for(int i=0;i<NUM_FEET;i++){
    bool this_phase = gait_phase(touchdown[i],duty_factor[i],gait_progress);
    double left_in_phase = gait_phase(touchdown[i],duty_factor[i],gait_progress,this_phase);
    OUT_LOG(logDEBUG) << "\t Stance Phase? : " << this_phase;
    OUT_LOG(logDEBUG) << "\t left in phase (%) : " << left_in_phase * 100.0;
    OUT_LOG(logDEBUG) << "\t left in phase (sec) : " << left_in_phase * gait_duration;
    ctrl_ptr->set_data<bool>(foot_names[i]+".stance",this_phase);
    
    OUT_LOG(logDEBUG) << "\t PHASE PROGRESS: { " << touchdown[i] << " .. " << gait_progress << " .. " << (touchdown[i] + duty_factor[i]) <<" }, stance: " << this_phase;
    
    Pose3d end_of_step_state
      = end_state(*(base_horizontal_frame.get()),
                  Origin3d(command[0],command[1],command[5]),
                  fabs(left_in_phase*gait_duration));
    
    end_of_step_state.update_relative_pose(Pacer::GLOBAL);

    OUTLOG(end_of_step_state.x,"end_of_step_state["+boost::icl::to_string<double>::apply(i)+"]",logERROR);
    
    Pose3d mid_of_next_step_state
    = end_state(end_of_step_state,
                Origin3d(command[0],command[1],command[5]),
                fabs( (((this_phase)? duty_factor[i] : 1.0-duty_factor[i] )/2.0) *gait_duration));
    
    end_of_step_state.update_relative_pose(Pacer::GLOBAL);
    
    OUTLOG(end_of_step_state.x,"end_of_step_state["+boost::icl::to_string<double>::apply(i)+"]",logERROR);
    
    
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
    
    Utility::visualize.push_back(
      Pacer::VisualizablePtr(
        new Pacer::Point(
          Vector3d(end_of_step_state.x[0],end_of_step_state.x[1],end_of_step_state.x[2]-0.05),
                         color,0.1)));
    
    Vector3d
            &x   = foot_pos[i],
            &xd  = foot_vel[i],
            &xdd = foot_acc[i];
            
    Origin3d xt(x);
    
    // Check if Spline must be re-evaluated
    bool replan_path = (!inited || last_phase[i] != this_phase);
    bool active_foot = active_feet[foot_names[i]];
    if(this_phase || (STANCE_ON_CONTACT && active_foot)){
      if(STANCE_ON_CONTACT && active_foot)
        if (gait_progress > (touchdown[i] + duty_factor[i]/2))
          this_phase = true;
      replan_path = false;
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

      Ravelin::MatrixNd J = ctrl_ptr->calc_link_jacobian(zero_base_generalized_q,foot_names[i]);
      
      J.block(0,3,NUM_JOINT_DOFS,NUM_JOINT_DOFS+6).mult(command,workv3_,-1.0,0);

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
    
      { // Visualize this
        Vector3d p = Pose3d::transform_point(Pacer::GLOBAL,x);
        Vector3d v = Pose3d::transform_vector(Pacer::GLOBAL,xd) * (left_in_phase*gait_duration);
        Utility::visualize.push_back( Pacer::VisualizablePtr( new Ray(  v+p,   p,   Vector3d(0,1,0),0.05)));
      }
      
      OUT_LOG(logDEBUG) << "x " << x;
      OUT_LOG(logDEBUG) << "xd " << xd;
      OUT_LOG(logDEBUG) << "xdd " << xdd;
      OUT_LOG(logDEBUG) << "U " << workv3_;
      
      last_phase[i] = this_phase;
      continue;
    } else {
      OUT_LOG(logDEBUG) << "\t Time: " << t;
      OUT_LOG(logDEBUG) << "\t Time coefs: " << spline_t[i];
      // Set liftoff feet (opposite of this_phase variable)
      for(int d=0; d<3;d++){
        OUT_LOG(logDEBUG) << "\t Spline coefs: " << spline_coef[i][d];
        replan_path = !Utility::eval_cubic_spline(spline_coef[i][d],spline_t[i],t,x[d],xd[d],xdd[d]);
        if(replan_path){
          OUT_LOG(logDEBUG) << "\t Must replan path!";
          break;
        }
      }
      OUT_LOG(logDEBUG) << "x " << x;
      OUT_LOG(logDEBUG) << "xd " << xd;
      OUT_LOG(logDEBUG) << "xdd " << xdd;
    }

    bool redirect_path = false;
    if(!replan_path){
      redirect_path = true;
    }
    // Stance phase (opposite to signal switch on next re-program)
    // Plan a new spline for this foot
    if(replan_path || redirect_path){
      // take time at end of spline_t history
      double t0 = t;
      
      if(redirect_path)
        t0 = *(spline_t[i].begin());
      
      xd.set_zero();
      xdd.set_zero();
      
      double gait_progress = t0/gait_duration;
      
      // Get the decimal part of gait_progress
      gait_progress = gait_progress - (double) ((int) gait_progress);
      if(gait_progress >= 1) gait_progress = Pacer::NEAR_ZERO;
      
      bool this_phase = gait_phase(touchdown[i],duty_factor[i],gait_progress);
      double left_in_phase = gait_phase(touchdown[i],duty_factor[i],gait_progress,this_phase);
      
    
      ///////////////////////////////////////////////////////////////////////
      
      /// SWING FEET ARE IN HORIZONTAL POSE
      boost::shared_ptr<Pose3d> state_ptr(new Pose3d(mid_of_next_step_state));
      Vector3d foot_goal(origins[i].data(),state_ptr);
      foot_goal = Pose3d::transform_vector(base_horizontal_frame,foot_goal);
      
      // Generate control points for swing foot spline
      std::vector<Origin3d> control_points;
      {
        
        Origin3d up_step(up.data());
        up_step*=step_height;
        
        if(redirect_path){
          Vector3d x_,xd_,xdd_;
          for(int d=0; d<3;d++)
            Utility::eval_cubic_spline(spline_coef[i][d],spline_t[i],t0,x_[d],xd_[d],xdd_[d]);
          control_points.push_back(Origin3d(x_));
        } else {
          control_points.push_back(xt);
        }
        
        OUT_LOG(logDEBUG) << "\tstep: " << control_points[0] << " --> " << foot_goal;

        control_points.push_back(control_points[0] + up_step);

        Origin3d foot_destination = Origin3d(foot_goal);
        Origin3d foot_direction = foot_destination - control_points[0];
        if(footholds.empty()){
          control_points.push_back(foot_destination + up_step + 0.1*foot_direction);
          control_points.push_back(foot_destination);
        } else {
          Vector3d x_fh = select_foothold(footholds, end_of_step_state, foot_destination);
            
          foot_destination = Origin3d(x_fh);
          
          // Reach new foothold, account for movement of robot during step
          control_points.push_back(foot_destination + up_step);
          control_points.push_back(foot_destination);
        }
      }

      // create spline using set of control points, place at back of history
      int n = control_points.size();
      std::vector<Origin3d> new_control_points;
      std::vector<double>           T;
    
      // Add current position via point at current time in spline
      new_control_points.push_back(control_points[0]);
      T.push_back(t0);

//      bool is_set = false;
      for(int j=1,jj=1;jj<n;j++){
        double this_time = t0 + fabs( left_in_phase*gait_duration / (double)(n-1)) * (double)jj ;
//        if(redirect_path){
//          OUTLOG(this_time,"redirect",logERROR);
//
//          if (this_time>t && !is_set && (t > t0+Pacer::NEAR_ZERO) ){
//            is_set = true;
//            T.push_back(t);
//            new_control_points.push_back(xt);
//            OUTLOG(t,"insert_at",logERROR);
//          } else {
//            if(this_time > T[T.size()-1]+Pacer::NEAR_ZERO){
//              T.push_back(this_time);
//              new_control_points.push_back(control_points[jj]);
//            }
//            jj++;
//          }
//          OUTLOG(new_control_points[j],"control_point",logERROR);
//        } else {
          T.push_back(this_time);
          new_control_points.push_back(control_points[jj]);
//          OUTLOG(new_control_points[j],"control_point",logERROR);
          jj++;
//        }
//        OUTLOG(T[j],"t",logERROR);
      }
      
      if(redirect_path)
        control_points = new_control_points;
      n = control_points.size();
      spline_t[i] = VectorNd(n,&T[0]);
      
      OUTLOG(spline_t[i],"T",logDEBUG);

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
        Ravelin::MatrixNd J = ctrl_ptr->calc_link_jacobian(zero_base_generalized_q,foot_names[i]);
        
        J.block(0,3,NUM_JOINT_DOFS,NUM_JOINT_DOFS+Pacer::NSPATIAL).mult(command,workv3_,-1.0,0);
        OUTLOG(J,"J",logDEBUG);
        xd = workv3_;
        xd.pose = base_frame;
      }
      
      n = control_points.size();
      for(int d=0;d<3;d++){
        VectorNd          &T = spline_t[i];
        VectorNd           X(n);
        VectorNd          &coefs = spline_coef[i][d];

        for(int j=0;j<n;j++)
          X[j] = control_points[j][d];
        OUTLOG(T,"T",logDEBUG);
        OUTLOG(X,"X",logDEBUG);

        Utility::calc_cubic_spline_coefs(T,X,Ravelin::Vector2d(xd[d],xd[d]),coefs);

        // then re-evaluate spline
        // NOTE: this will only work if we replanned for a t_0  <  t  <  t_0 + t_I
        OUT_LOG(logDEBUG) << "Eval first step in spline";
        Utility::eval_cubic_spline(spline_coef[i][d],spline_t[i],t,x[d],xd[d],xdd[d]);
      }
    }
    //last_feet_active[i] = feet[i]->active;
    //feet[i]->stance = !this_phase;
    x.pose = base_frame;
    xd.pose = base_frame;
    xdd.pose = base_frame;
    
    last_phase[i] = this_phase;
  }

  
  for(int i=0;i<footholds.size();i++){
    Utility::visualize.push_back(
      Pacer::VisualizablePtr(
        new Pacer::Point(
                         Vector3d(footholds[i][0],footholds[i][1],footholds[i][2]),
                         Vector3d(1,0,0),10.0*footholds[i][3])));
  }

  for(int i=0;i<NUM_FEET;i++){

///* VISUALIZE
//#ifdef NDEBUG
    if(last_phase[i])
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
//      Utility::visualize.push_back( Pacer::VisualizablePtr( new Ray(  v+p,   p,   Vector3d(1,0,0),0.005)));
//     Utility::visualize.push_back( Pacer::VisualizablePtr( new Ray(a+v+p, v+p, Vector3d(1,0.5,0)));
    }
//#endif
    //  END VISUALIZE */
  }
  inited = true;
  OUT_LOG(logDEBUG) << " -- walk_toward() exited";
}

void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t){
  ctrl_ptr = ctrl;
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
      duty_factor = ctrl_ptr->get_data<std::vector<double> >(plugin_namespace+".duty-factor"),
    this_gait = ctrl_ptr->get_data<std::vector<double> >(plugin_namespace+".gait"),
    input_gait_pose = ctrl_ptr->get_data<std::vector<double> >(plugin_namespace+".pose");
  double gait_time = ctrl_ptr->get_data<double>(plugin_namespace+".gait-duration");
  double step_height = ctrl_ptr->get_data<double>(plugin_namespace+".step-height");
  
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
  
  foot_names = ctrl_ptr->get_data<std::vector<std::string> >(plugin_namespace+".feet");

  double width = ctrl_ptr->get_data<double>(plugin_namespace+".width");
  double length = ctrl_ptr->get_data<double>(plugin_namespace+".length");
  double height = ctrl_ptr->get_data<double>(plugin_namespace+".height");

  base_frame = boost::shared_ptr<Pose3d>( new Pose3d(
        ctrl->get_data<Pose3d>("base_link_frame")));
  base_horizontal_frame = boost::shared_ptr<Pose3d>( new Pose3d(
      ctrl->get_data<Pose3d>("base_horizontal_frame")));
  
  { /// Adjust Gait
    
    
  }
  
//  { /// Display command
//    Origin3d dir(Pose3d::transform_vector(Pacer::GLOBAL,Vector3d(command,base_horizontal_frame)).data());
//    dir *= 2.0;
//    dir[2] = command[2]/10;
//    Utility::visualize.push_back( Pacer::VisualizablePtr( new Ray(  (dir+base_horizontal_frame->x).data(),   (base_horizontal_frame->x).data(),   Vector3d(0,1,0),0.5)));
//  }
  
  // Set up output vectors for gait planner
  int NUM_FEET = foot_names.size();
  std::vector<Vector3d>
      foot_vel(NUM_FEET),
      foot_pos(NUM_FEET),
      foot_acc(NUM_FEET),
      foot_init(NUM_FEET);
  q = ctrl->get_generalized_value(Pacer::Robot::position);
  qd_base = ctrl->get_base_value(Pacer::Robot::velocity);
  
//  ctrl_ptr->set_model_state(q);
  for(int i=0;i<NUM_FEET;i++){
    foot_init[i] = ctrl_ptr->get_data<Vector3d>(foot_names[i]+".init.x");
    foot_pos[i] = foot_init[i];
    ctrl_ptr->get_data<Vector3d>(foot_names[i]+".goal.x",foot_pos[i]);
    foot_vel[i] = Vector3d(0,0,0,base_frame);
    ctrl_ptr->get_data<Vector3d>(foot_names[i]+".goal.xd",foot_vel[i]);
    foot_acc[i] = Vector3d(0,0,0,base_frame);
  }

  
  gait_pose = boost::shared_ptr<Pose3d>(
      new Pose3d(
        Ravelin::Quatd::rpy(
          input_gait_pose[3],
          input_gait_pose[4],
          input_gait_pose[5]
        ),
        Origin3d(
          input_gait_pose[0] /*+ command[0]/(4.0*gait_time)*/,
          input_gait_pose[1] /*+ command[1]/(4.0*gait_time)*/,
          input_gait_pose[2]
        ),
        base_frame
      )
    );
  
  // Assign foot origins (ideal foot placemenet at rest)
  std::vector<Origin3d> origins;
  if(origins.empty()){
    origins.resize(NUM_FEET);
    for(int i=0;i<NUM_FEET;i++){
      Vector3d origin(
                   Utility::sign<double>(foot_init[i][0])*length/2,
                   Utility::sign<double>(foot_init[i][1])*width/2,
                   -height,base_frame);
      origins[i] = Pose3d::transform_point(gait_pose,origin);
    }
  }
  
  OUTLOG(this_gait,"this_gait",logINFO);
  OUTLOG(duty_factor,"duty_factor",logINFO);

  int STANCE_ON_CONTACT = ctrl_ptr->get_data<int>(plugin_namespace+".stance-on-contact");
  
  for(int i=0;i<foot_names.size();i++){
    std::vector< boost::shared_ptr< const Pacer::Robot::contact_t> > c;
    ctrl->get_link_contacts(foot_names[i],c);
    if(!c.empty())
      active_feet[foot_names[i]] = true;
    else
      active_feet[foot_names[i]] = false;
  }
  
  walk_toward(go_to,this_gait,footholds,duty_factor,gait_time,step_height,STANCE_ON_CONTACT,origins,ctrl->get_data<Vector3d>("center_of_mass.x"),t,foot_pos,foot_vel, foot_acc);
  
  gait_pose->update_relative_pose(Pacer::GLOBAL);
  ctrl->set_data<Pose3d>("base_stability_frame",*(gait_pose.get()));
  
  for(int i=0;i<NUM_FEET;i++){
    ctrl->set_data<Vector3d>(foot_names[i]+".goal.x",foot_pos[i]);
    ctrl->set_data<Vector3d>(foot_names[i]+".goal.xd",foot_vel[i]);
    ctrl->set_data<Vector3d>(foot_names[i]+".goal.xdd",foot_acc[i]);
  }
}

/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
/** This is a quick way to register your plugin function of the form:
 * void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t)
 * void Deconstruct(const boost::shared_ptr<Pacer::Controller>& ctrl)
 */

void update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t){
  static int ITER = 0;
  int RTF = (int) ctrl->get_data<double>(plugin_namespace+".real-time-factor");
  if(ITER%RTF == 0)
    Update(ctrl,t);
  ITER+=1;
}

void deconstruct(const boost::shared_ptr<Pacer::Controller>& ctrl, double t){
  ctrl->remove_data("base_stability_frame");

  std::vector<std::string> foot_names
  = ctrl_ptr->get_data<std::vector<std::string> >(plugin_namespace+".feet");

  int NUM_FEET = foot_names.size();

  for(int i=0;i<NUM_FEET;i++){
    ctrl->remove_data(foot_names[i]+".goal.x");
    ctrl->remove_data(foot_names[i]+".goal.xd");
    ctrl->remove_data(foot_names[i]+".goal.xdd");
    ctrl->remove_data(foot_names[i]+".stance");
  }
}

extern "C" {
  void init(const boost::shared_ptr<Pacer::Controller> ctrl, const char* name){
    plugin_namespace = std::string(std::string(name));
    
    int priority = ctrl->get_data<double>(plugin_namespace+".priority");
    
    ctrl->add_plugin_update(priority,name,&update);
    ctrl->add_plugin_deconstructor(name,&deconstruct);
  }
}

