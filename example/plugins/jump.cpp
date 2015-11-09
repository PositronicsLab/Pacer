#include <Pacer/controller.h>
#include <Pacer/utilities.h>

std::string plugin_namespace;
using namespace Ravelin;

const double grav     = 9.81; // m / s*s

void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t){
  static double start_jump_time = t;
  
  static double last_time = -0.001;
  double dt = t - last_time;
  last_time = t;
  
  static Vector3d com_x = ctrl->get_data<Vector3d>("center_of_mass.x");
  
  const  std::vector<std::string>
  eef_names_ = ctrl->get_data<std::vector<std::string> >("init.end-effector.id");
  
  static std::vector<Vector3d> x_foot_goal(eef_names_.size());
  
  
  // Set takeoff angle
  double angle_of_elevation = ctrl->get_data<double>(plugin_namespace + ".angle-of-elevation");
  double heading = ctrl->get_data<double>(plugin_namespace + ".heading");
  //distance to goal
  double range = ctrl->get_data<double>(plugin_namespace + ".range");
  double duration = ctrl->get_data<double>(plugin_namespace + ".duration");
  
  double jump_progress = (t - start_jump_time)/duration;

  // 3d velocity
  double v_spatial = sqrt((grav * range) / sin(2.0 * angle_of_elevation));
  OUTLOG(v_spatial, "v_spatial", logDEBUG);
  
  // 2d (planar) velocity
  double v_horizontal = v_spatial * std::cos(angle_of_elevation);
  OUTLOG(v_horizontal, "v_horizontal", logDEBUG);
  
  // directional velocities
  // X
  double v_forward = v_horizontal * std::cos(heading);
  OUTLOG(v_forward, "v_forward", logDEBUG);
  // Y
  double  v_sideways = v_horizontal * std::sin(heading);
  OUTLOG(v_sideways, "v_sideways", logDEBUG);
  // Z
  double v_vertical = v_spatial * std::sin(angle_of_elevation);
  OUTLOG(v_vertical, "v_vertical", logDEBUG);
  
  Vector3d v_liftoff(v_forward,v_sideways,v_vertical);
  OUTLOG(v_liftoff.norm(), "||v_liftoff||", logERROR);
  OUTLOG(fabs(v_spatial), "|v_spatial|", logERROR);
  
  // Check that we did this right
  assert(fabs( v_liftoff.norm() - fabs(v_spatial) ) < Pacer::NEAR_ZERO);
  
  if(t < start_jump_time)
    return;
  
  if(jump_progress > 1.0){
    ctrl->open_plugin("reset-trajectory");
    ctrl->close_plugin("eef-PID-controller");
    ctrl->close_plugin(plugin_namespace);
    return;
  }
  
#ifndef USE_OSG_DISPLAY
  com_x.pose = Pacer::GLOBAL;
  Utility::visualize.push_back( Pacer::VisualizablePtr( new Pacer::Point(  Vector3d(cos(heading)*range,sin(heading)*range,0) + com_x,   Vector3d(1,0,1),0.1)));
#endif
  
  OUT_LOG(logDEBUG) << "Jumping";
  
  boost::shared_ptr<Ravelin::Pose3d>
    base_frame( new Ravelin::Pose3d(ctrl->get_data<Ravelin::Pose3d>("base_link_frame")));

  // Get base state then set to global origin
  Ravelin::VectorNd local_q = ctrl->get_generalized_value(Pacer::Controller::position);
  int NUM_JOINT_DOFS = local_q.rows()-7;
  local_q.set_sub_vec(NUM_JOINT_DOFS,Utility::pose_to_vec(Ravelin::Pose3d()));
  
  static Ravelin::VectorNd start_vel = ctrl->get_base_value(Pacer::Controller::velocity);

  Ravelin::VectorNd goal_vel = VectorNd::zero(6);
  goal_vel.set_sub_vec(0,v_liftoff);
  
  VectorNd acc,vel;
  
  // Velocity
  // v = vf * (t*t)
  (vel = goal_vel) -= start_vel;
  vel *= (jump_progress*jump_progress);
  
  // Acceleration
  // v' = 2 vf * (t)
  (acc = goal_vel) -= start_vel;
  acc *= 2*(jump_progress);
  // error correction term
  Ravelin::VectorNd base_vel = ctrl->get_base_value(Pacer::Controller::velocity);
  double Ka = 10;
  acc +=  ( (base_vel -= vel) *= (Ka*dt) );
  
#ifndef USE_OSG_DISPLAY
  Vector3d base_x(base_frame->x.data());
  
  Utility::visualize.push_back
  ( Pacer::VisualizablePtr
   ( new Pacer::Ray
    (base_x,base_x+Vector3d(vel.segment(0,3).data())*dt,Vector3d(1,0,0),0.02)));
  Utility::visualize.push_back
  ( Pacer::VisualizablePtr
   ( new Pacer::Ray
    (base_x+Vector3d(vel.segment(0,3).data())*dt,base_x+Vector3d(vel.segment(0,3).data())*dt+Vector3d(vel.segment(0,3).data())*dt*dt,Vector3d(1,0,0),0.02)));
#endif
  
  for(unsigned i=0;i<eef_names_.size();i++){
    ctrl->set_model_state(local_q);
    Ravelin::Vector3d xd_foot,xdd_foot;
    
    // Calc jacobian for AB at this EEF
    Ravelin::MatrixNd J = ctrl->calc_link_jacobian(local_q,eef_names_[i]);
    
    // Now that model state is set ffrom jacobian calculation
    if(start_jump_time == t){
      const boost::shared_ptr<Ravelin::RigidBodyd> link = ctrl->get_link(eef_names_[i]);
      x_foot_goal[i] = Ravelin::Pose3d::transform_point(Pacer::GLOBAL,Ravelin::Vector3d(0,0,0,link->get_pose()));
    }
    
    J.block(0,3,NUM_JOINT_DOFS,NUM_JOINT_DOFS+6).mult(vel,xd_foot,-1,0);
    J.block(0,3,NUM_JOINT_DOFS,NUM_JOINT_DOFS+6).mult(acc,xdd_foot,-1,0);
    
    x_foot_goal[i].pose = base_frame;
    xd_foot.pose = base_frame;
    xdd_foot.pose = base_frame;
    x_foot_goal[i] += xd_foot*dt;
    ctrl->set_data<Ravelin::Vector3d>(eef_names_[i]+".goal.x",x_foot_goal[i]);
    ctrl->set_data<Ravelin::Vector3d>(eef_names_[i]+".goal.xd",xd_foot);
    ctrl->set_data<Ravelin::Vector3d>(eef_names_[i]+".goal.xdd",xdd_foot);
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
  std::vector<std::string>
  foot_names = ctrl->get_data<std::vector<std::string> >("init.end-effector.id");
  
  int NUM_FEET = foot_names.size();
  
  for(int i=0;i<NUM_FEET;i++){
    ctrl->remove_data(foot_names[i]+".goal.x");
    ctrl->remove_data(foot_names[i]+".goal.xd");
    ctrl->remove_data(foot_names[i]+".goal.xdd");
  }
}

extern "C" {
  void init(const boost::shared_ptr<Pacer::Controller> ctrl, const char* name){
    plugin_namespace = std::string(name);
    
    int priority = ctrl->get_data<double>(plugin_namespace+".priority");
    
    ctrl->add_plugin_update(priority,name,&update);
    ctrl->add_plugin_deconstructor(name,&deconstruct);
  }
}
