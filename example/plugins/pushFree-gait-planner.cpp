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
static bool push_stable = true;
static int stable_clock = 0;
static int stable_idx = 0;
static int phase_nb[] = {0, 0, 0, 0};

boost::shared_ptr<Pacer::Controller> ctrl_ptr;
  boost::shared_ptr<Pose3d> base_frame,base_horizontal_frame, gait_pose;
  std::string plugin_namespace;
  std::vector<std::string> foot_names;
std::map<std::string,bool> active_feet;

void select_foothold(const std::vector<Vector3d>& footholds,const Origin3d &x, Origin3d& x_fh){
  double min_dist = INFINITY;
  int    min_vec = 0;
  for(int i=0;i<footholds.size();i++){
    double norm = (footholds[i]-x).norm();
    if(norm < min_dist){
      min_dist = norm;
      min_vec = i;
    }
  }
  x_fh = Origin3d(footholds[min_vec]);
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
    const std::vector<Vector3d>& footholds,
    const std::vector<double>& duty_factor,
    double gait_duration,
    double step_height,
    bool STANCE_ON_CONTACT,
    // MODEL
    std::vector<Origin3d>& origins,
    const Vector3d& center_of_mass_x,
    double t,
    std::vector<Vector3d>& foot_pos,
    std::vector<Vector3d>& foot_vel,
    std::vector<Vector3d>& foot_acc,
    double kdx,
    double step_factor)
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
  if(gait_progress >= 1) gait_progress = Moby::NEAR_ZERO;

//  Vector3d turn_origin(0,command[0]/command[5],0,base_horizontal_frame);
//  turn_origin = Pose3d::transform_point(base_frame, turn_origin);
  Origin3d turn_origin(0,command[0]/command[5],0);
  
  boost::shared_ptr< Pose3d> turning_frame(new Pose3d(Ravelin::Quatd::identity(),turn_origin,base_horizontal_frame));

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

//    VectorNd ackermann_command(6);
//    Origin3d foot_radius = origins[i] - turn_origin;
//    Origin3d body_radius = -turn_origin;
//    
//    // Forward speed at center of body = command[0]
//    // Forward speed at feet must induce a turn about turn_origin in base_frame
//    xd_stance =
//    
    
    OUT_LOG(logDEBUG) << "\t PHASE PROGRESS: { " << touchdown[i] << " .. " << gait_progress << " .. " << (touchdown[i] + duty_factor[i]) <<" }, stance: " << this_phase;
    
    Vector3d &x   = foot_pos[i],
                      &xd  = foot_vel[i],
                      &xdd = foot_acc[i];

    // Count phases
    if (last_phase[i] != this_phase)
        phase_nb[i]++;
    
    // Check if Spline must be re-evaluated
    bool replan_path = (!inited || last_phase[i] != this_phase);
    bool active_foot = active_feet[foot_names[i]];
    if(this_phase || (STANCE_ON_CONTACT && active_foot)){
      if(STANCE_ON_CONTACT && active_foot)
        this_phase = true;
      replan_path = false;
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
      workv3_.pose = base_frame;
      xd.pose = base_frame;

      xdd = (dt < Moby::NEAR_ZERO)? 0 : (workv3_ - xd)/dt;
      
      xd = workv3_;
      xdd.pose = base_frame;
      xd.pose = base_frame;
      x.pose = base_frame;
      x = x + xd * dt;
      
//      if(active_foot){
//        workv3_ *= 100;
//        J.block(0,3,0,NUM_JOINT_DOFS).transpose_mult(workv3_,workv_);
//        ctrl_ptr->set_joint_generalized_value(Pacer::Robot::load_goal,workv_);
//      }
    
      Vector3d p = Pose3d::transform_point(Moby::GLOBAL,x);
      Vector3d v = Pose3d::transform_vector(Moby::GLOBAL,xd);// * (left_in_phase*gait_duration);
      Utility::visualize.push_back( Pacer::VisualizablePtr( new Ray(  v+p,   p,   Vector3d(1,0,1),0.01)));
      
      OUT_LOG(logDEBUG) << "x " << x;
      OUT_LOG(logDEBUG) << "xd " << xd;
      OUT_LOG(logDEBUG) << "xdd " << xdd;
      OUT_LOG(logDEBUG) << "U " << workv3_;
      
      last_phase[i] = this_phase;
      //std::cout << "continue" << std::endl;
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
      if(gait_progress >= 1) gait_progress = Moby::NEAR_ZERO;
      
      bool this_phase = gait_phase(touchdown[i],duty_factor[i],gait_progress);
      double left_in_phase = gait_phase(touchdown[i],duty_factor[i],gait_progress,this_phase);


      // Calculate foot step info
      // Determine linear portion of step
      boost::shared_ptr< Pose3d> foot_frame(new Pose3d(Ravelin::Quatd::identity(),origins[i],base_frame));

      Origin3d x0 = origins[i];

      /// SWING FEET ARE IN HORIZONTAL POSE
      Vector3d foot_goal(command[0],command[1],0,base_horizontal_frame);
      foot_goal = Pose3d::transform_vector(foot_frame,foot_goal);

      
      if(fabs(command[5]) > Moby::NEAR_ZERO){
        // Determine shape of step needed to yaw robot
        Origin3d rotation_axis = Origin3d(up.data());

        Ravelin::AAngled aa_gamma(Vector3d(rotation_axis,turning_frame),command[5] *  left_in_phase*gait_duration);
        OUTLOG(aa_gamma,"aa_gamma",logDEBUG);
        Ravelin::Matrix3d R_gamma;
        R_gamma = aa_gamma;
        OUTLOG(R_gamma,"R_gamma",logDEBUG);

        R_gamma.mult(Pose3d::transform_vector(turning_frame, foot_goal), workv3_);
        workv3_.pose = turning_frame;
        foot_goal = Pose3d::transform_vector(foot_frame, workv3_);
      }

      // scale step length by interval duration
      foot_goal *=  left_in_phase*gait_duration;

      // All steps wrt to x0
      foot_goal /= 2;

      OUT_LOG(logDEBUG) << "\tstep = " << foot_goal;

      // Generate control points for swing foot spline
      std::vector<Origin3d> control_points;
      {
        // SWING
        OUT_LOG(logDEBUG) << "\tPlanning Swing phase (phase > 0)...";

        foot_goal /= (1-duty_factor[i]);
        OUT_LOG(logDEBUG) << "\tstep = " << foot_goal;
        Origin3d up_step(up.data());

        /// Check if body is stable again
        int nextStableClock = (int)((stable_clock + 1) / 2) * 2;
        if ( (qd_base[1] < 0.01) && (qd_base[1] > -0.01) &&         // small side velocity
             (stable_idx == i) && (phase_nb[i] > nextStableClock) ) // wait for a complete gait cycle after push
        {
                push_stable = true;
        }

        /// Check if body was pushed [steering speed smaller than 0.2]
        if ((qd_base[1] > 0.2 || qd_base[1] < -0.2))
            //(qd_base[0] > 0.2 || qd_base[0] < -0.2) )
        {
            stable_clock = phase_nb[i]; // phase number when body was pushed
            stable_idx = i;             // active leg when body was pushed
            push_stable = false;        // body not stable
        }

        /// Make the step bigger if body was pushed
        if (!push_stable)
            up_step*=step_height * step_factor; // good factor values in range [1.5, 2]
        else
            up_step*=step_height;
        
        if(redirect_path){
          Vector3d x_,xd_,xdd_;
          for(int d=0; d<3;d++)
            Utility::eval_cubic_spline(spline_coef[i][d],spline_t[i],t0,x_[d],xd_[d],xdd_[d]);
          
          control_points.push_back(Origin3d(x_));
          control_points.push_back(Origin3d(x_) + up_step);
        } else {
          control_points.push_back(Origin3d(x));
          control_points.push_back(Origin3d(x) + up_step);
        }
        
        if(footholds.empty()){
          //std::cout << "Just plan spline for foot " << i << "Info "<< current_vel[i] << "Base " << qd_base << std::endl;
          //std::cout << "T=" << t << " Just plan spline for foot " << i << "Info "<< current_vel[i] << std::endl;

          //std::cout << qd_base << qd_base[1] << std::endl;
          //double Ts = gait_duration * duty_factor;
          //double xf0 = Ts * qd_base[0] / 2;
         // double yf0 = Ts * qd_base[1] / 2;
          //if (qd_base[1] > 0.1 || qd_base[1] < -0.1)

          /// Step based on the stage: normal step, guard step
          if (!push_stable)
          { // guard step
            //double kdx = 0.7; is read now from file
            double xdf = kdx * (qd_base[0] - 0);
            double ydf = kdx * (qd_base[1] - 0);

            // Clamp side step to the half of body
            double side_step_max_factor = 0.5;
            if (xdf > 0 && xdf > (side_step_max_factor * 0.14) )
                xdf = side_step_max_factor * 0.14;
            if (xdf < 0 && xdf < (-side_step_max_factor * 0.14) )
                xdf = -side_step_max_factor * 0.14;

            if (ydf > 0 && ydf > (side_step_max_factor * 0.1) )
                ydf = side_step_max_factor * 0.1;
            if (ydf < 0 && ydf < (-side_step_max_factor * 0.1) )
                ydf = -side_step_max_factor * 0.1;

            //std::cout << t << "  " << i << " My stepping " << std::endl;
            //std::cout << t << "  " << i << "  " << xdf << "   " << ydf  << "  " << qd_base << std::endl;
            //std::cout << t << "  " << i << "  " << xdf << "   " << ydf  << "  " << foot_goal << std::endl;

            Origin3d dx = Origin3d(xdf, ydf, 0.0);
            control_points.push_back(x0 + dx + up_step);
            control_points.push_back(x0 + dx );
          }
          else
          { // normal step towards goal
              //std::cout << t << "  " << i << " Gait planner " << std::endl;
              //std::cout << t << "  " << i << "  " << 0 << "   " << 0  << "  " << qd_base << std::endl;
              //std::cout << t << "  " << i << "  " << 0 << "   " << 0  << "  " << foot_goal << std::endl;

              control_points.push_back(x0 + Origin3d(foot_goal) + up_step);
              control_points.push_back(x0 + Origin3d(foot_goal) );
          }

          // Hardcoded guard step
          /*if (qd_base[1] > 0.1)
            dx = Origin3d(0.0, 0.05, 0.0);
          else if (qd_base[1] < -0.1)
            dx = Origin3d(0.0, -0.05, 0.0);
          else
            dx = Origin3d(0.0, 0.0, 0.0);*/
        } else {
          //std::cout << "Why this?" << std::endl;
          Origin3d x_fh;
          select_foothold(footholds,Pose3d::transform_point(Moby::GLOBAL, Vector3d((x0+Origin3d(foot_goal)).data(),base_frame)).data(),x_fh);
          // Reach new foothold, account for movement of robot during step
          control_points.push_back(Origin3d(Pose3d::transform_point(base_frame,Vector3d(x_fh.data(),Moby::GLOBAL)).data()) - Origin3d(foot_goal) + Origin3d(0,0,step_height));
          control_points.push_back(x_fh - Origin3d(foot_goal));
        }

        //std::cout << control_points[0] << control_points[1] << control_points[2] << control_points[3] << std::endl;
      }

      // create spline using set of control points, place at back of history
      int n = control_points.size();
//      std::vector<Origin3d> new_control_points;
//      if(redirect_path)
//        spline_t[i].set_zero(n+1);
//      else
        spline_t[i].set_zero(n);
      VectorNd           &T = spline_t[i];
      bool unset = true;
      for(int j=0,jj=0;j<n;j++){
//        if(redirect_path){
//          T[j] = t0 + fabs( left_in_phase*gait_duration / (double)(n-2)) * (double)jj ;
//          if (T[j]>t && unset){
//            unset = false;
//            T[j] = t;
//            new_control_points.back() = Origin3d(x.data());
//            new_control_points.push_back(control_points[jj-1]);
//          } else {
//            jj++;
//          }
//          new_control_points.push_back(control_points[jj]);
//        } else {
          T[j] = t0 + fabs( left_in_phase*gait_duration / (double)(n-1)) * (double)j ;
//        }
      }
//      if(redirect_path)
//        control_points = new_control_points;
      OUTLOG(T,"T",logDEBUG1);
      for(int cp=0;cp<control_points.size();cp++){
        OUTLOG(control_points[cp],"control_point",logDEBUG1);
      }
      OUTLOG(xd,"control_point_V",logDEBUG1);
      for(int d=0;d<3;d++){
        VectorNd           X(n);
        VectorNd          &coefs = spline_coef[i][d];

        for(int j=0;j<n;j++)
          X[j] = control_points[j][d];
        OUTLOG(X,"X",logDEBUG1);

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

  //std::cout << phase_nb[0] << " " << phase_nb[1] << " " << phase_nb[2] << " " << phase_nb[3] << std::endl;

  for(int i=0;i<NUM_FEET;i++){

///* VISUALIZE
//#ifdef NDEBUG
    {
      Vector3d p = Pose3d::transform_point(Moby::GLOBAL,foot_pos[i]);
      Vector3d v = Pose3d::transform_vector(Moby::GLOBAL,foot_vel[i])/10;
      Utility::visualize.push_back( Pacer::VisualizablePtr( new Pacer::Point( p,   Vector3d(0,0,1),0.1)));
      Utility::visualize.push_back( Pacer::VisualizablePtr( new Ray(  v+p,   p,   Vector3d(0,1,0),0.1)));
    }
    if(last_phase[i])
      continue;

    VectorNd &T = spline_t[i];

    for(double t=T[0]+Moby::NEAR_ZERO ; t<=T[T.rows()-1] ; t += 0.01){
      Vector3d x,xd,xdd;
      for(int d=0;d<3;d++){
        Utility::eval_cubic_spline(spline_coef[i][d],spline_t[i],t,x[d],xd[d],xdd[d]);
      }
      x.pose = base_frame;
      xd.pose = base_frame;
      xdd.pose = base_frame;
      Vector3d p = Pose3d::transform_point(Moby::GLOBAL,x);
      Vector3d v = Pose3d::transform_vector(Moby::GLOBAL,xd)/10;
//      Vector3d a = Pose3d::transform_vector(Moby::GLOBAL,xdd)/100;
      Utility::visualize.push_back( Pacer::VisualizablePtr( new Pacer::Point( p,   Vector3d(0,1,0),0.01)));
      Utility::visualize.push_back( Pacer::VisualizablePtr( new Ray(  v+p,   p,   Vector3d(1,0,0),0.01)));
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
  const int queue_size = 10;
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

  static std::vector<double>
      duty_factor = ctrl_ptr->get_data<std::vector<double> >(plugin_namespace+"duty-factor"),
    this_gait = ctrl_ptr->get_data<std::vector<double> >(plugin_namespace+"gait"),
    input_gait_pose = ctrl_ptr->get_data<std::vector<double> >(plugin_namespace+"pose");
  static double gait_time = ctrl_ptr->get_data<double>(plugin_namespace+"gait-duration");
  static double step_height = ctrl_ptr->get_data<double>(plugin_namespace+"step-height");
  static double kdx = ctrl_ptr->get_data<double>(plugin_namespace+"push-feedback-gain");
  static double step_factor = ctrl_ptr->get_data<double>(plugin_namespace+"push-step-factor");
  static std::vector<Vector3d> footholds(0);
  foot_names = ctrl_ptr->get_data<std::vector<std::string> >(plugin_namespace+"feet");

  static double width = ctrl_ptr->get_data<double>(plugin_namespace+"width");
  static double length = ctrl_ptr->get_data<double>(plugin_namespace+"length");
  static double height = ctrl_ptr->get_data<double>(plugin_namespace+"height");

  base_frame = boost::shared_ptr<Pose3d>( new Pose3d(
        ctrl->get_data<Pose3d>("base_link_frame")));
  base_horizontal_frame = boost::shared_ptr<Pose3d>( new Pose3d(
      ctrl->get_data<Pose3d>("base_horizontal_frame")));
  
  { /// Adjust Gait
    
    
  }
  
  { /// Display command
    Origin3d dir(Pose3d::transform_vector(Moby::GLOBAL,Vector3d(command,base_horizontal_frame)).data());
    dir *= 2.0;
    dir[2] = command[2]/10;
    Utility::visualize.push_back( Pacer::VisualizablePtr( new Ray(  (dir+base_horizontal_frame->x).data(),   (base_horizontal_frame->x).data(),   Vector3d(0,1,0),0.5)));
  }
  
  // Set up output vectors for gait planner
  int NUM_FEET = foot_names.size();
  std::vector<Vector3d>
      foot_vel(NUM_FEET),
      foot_pos(NUM_FEET),
      foot_acc(NUM_FEET),
      current_vel(NUM_FEET),
      current_pos(NUM_FEET),
      current_acc(NUM_FEET),
      foot_init(NUM_FEET);

  // get pos and velocity for the base
  q = ctrl->get_generalized_value(Pacer::Robot::position);
  //std::cout << q << std::endl;
  qd_base = ctrl->get_base_value(Pacer::Robot::velocity);
  
  // get current foot velocities
  for(int i=0;i<NUM_FEET;i++){
    ctrl->get_data<Vector3d>(foot_names[i]+".state.x",current_pos[i]);
    ctrl->get_data<Vector3d>(foot_names[i]+".state.xd",current_vel[i]);
    ctrl->get_data<Vector3d>(foot_names[i]+".state.xdd",current_acc[i]);
  }

  // ctrl_ptr->set_model_state(q);

  // get goal foot velocity
  for(int i=0;i<NUM_FEET;i++){
    foot_init[i] = ctrl_ptr->get_data<Vector3d>(foot_names[i]+".init.x");
    foot_pos[i] = foot_init[i];
    //ctrl_ptr->get_data<Vector3d>(foot_names[i]+".goal.x",foot_pos[i]);
    ctrl->get_data<Vector3d>(foot_names[i]+".goal.x",foot_pos[i]);
    foot_vel[i] = Vector3d(0,0,0,base_frame); 
    ctrl_ptr->get_data<Vector3d>(foot_names[i]+".goal.xd",foot_vel[i]);
    foot_acc[i] = Vector3d(0,0,0,base_frame);
    //double gait_progress = t/gait_time;
    //gait_progress = gait_progress - (double) ((int) gait_progress);
  }

  
  gait_pose = boost::shared_ptr<Pose3d>(
      new Pose3d(
        Ravelin::Quatd::rpy(
          input_gait_pose[3],
          input_gait_pose[4],
          input_gait_pose[5]
        ),
        Origin3d(
          input_gait_pose[0],
          input_gait_pose[1],
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
  
  ctrl->set_data<Pose3d>("base_stability_frame",*(gait_pose.get()));

  OUTLOG(this_gait,"this_gait",logINFO);
  OUTLOG(duty_factor,"duty_factor",logINFO);

  int STANCE_ON_CONTACT = ctrl_ptr->get_data<int>(plugin_namespace+"stance-on-contact");
  
  for(int i=0;i<foot_names.size();i++){
    std::vector< boost::shared_ptr< const Pacer::Robot::contact_t> > c;
    ctrl->get_link_contacts(foot_names[i],c);
    if(!c.empty())
      active_feet[foot_names[i]] = true;
    else
      active_feet[foot_names[i]] = false;
  }
  
  walk_toward(go_to,this_gait,footholds,duty_factor,gait_time,step_height,STANCE_ON_CONTACT,origins,ctrl->get_data<Vector3d>("center_of_mass.x"),t,foot_pos,foot_vel, foot_acc, kdx, step_factor);
  
  for(int i=0;i<NUM_FEET;i++){
    ctrl->set_data<Vector3d>(foot_names[i]+".goal.x",foot_pos[i]);
    ctrl->set_data<Vector3d>(foot_names[i]+".goal.xd",foot_vel[i]);
    ctrl->set_data<Vector3d>(foot_names[i]+".goal.xdd",foot_acc[i]);
  }
}

/** This is a quick way to register your plugin function of the form:
  * void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t)
  */
#include "register-plugin"
