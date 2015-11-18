#include <Pacer/controller.h>
#include <Pacer/utilities.h>

std::string plugin_namespace;
using namespace Ravelin;

const double grav     = 9.81; // m / s*s

template <typename T> T sqr(T val) {
  return val*val;
}

double sigmoid(double x){
  return (1.0 / (1.0 + exp(-x)));
}

double dsigmoid(double x){
  return (exp(x) / sqr(1.0 + exp(x)));
}

double sigmoid_interp(double v0, double vF, double alpha){
  // sigmoid curve interpolates wrt to alpha \in {0..1}
  double diff = vF - v0;
  return v0 + diff*sigmoid(alpha*10.0 - 5.0);
}

double sigmoid_interp2(double v0, double vF, double alpha, double& deriv){
  // sigmoid curve interpolates wrt to alpha \in {0..1}
  double diff = vF - v0;
  deriv = dsigmoid(alpha*10.0 - 5.0);
  return v0 + diff*sigmoid(alpha*10.0 - 5.0);
}

void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t){
  static double start_jump_time = t;
  
  static double last_time = -0.001;
  double dt = t - last_time;
  last_time = t;
  
  const  std::vector<std::string>
  eef_names_ = ctrl->get_data<std::vector<std::string> >("init.end-effector.id");
  
  static std::vector<Origin3d> x_foot_goal(eef_names_.size());
  
  // check the timing of the jump
  double duration = ctrl->get_data<double>(plugin_namespace + ".duration");
  // alpha = 0 .. 1
  double alpha = sigmoid_interp(0,1,(t - start_jump_time)/duration);

  // check reach (find smallest distance to reach maximum)
  // min reach proportion
  // beta = 0 .. 1
  double beta = 0;
  if(start_jump_time != t){
    for (int i=0; i<eef_names_.size(); i++) {
      double ratio = 0;
      
      Origin3d pos_des = x_foot_goal[i];
      Ravelin::Origin3d base_joint = ctrl->get_data<Ravelin::Origin3d>(eef_names_[i]+".base");
      double max_reach = ctrl->get_data<double>(eef_names_[i]+".reach");
      
      Ravelin::Origin3d goal_from_base_joint = pos_des - base_joint;
      double goal_reach = goal_from_base_joint.norm();
      OUT_LOG(logDEBUG1) << eef_names_[i] << " - goal_reach < max_reach : " << goal_reach<<  " < "  << max_reach ;
      ratio = goal_reach/max_reach;
      OUT_LOG(logDEBUG1) << eef_names_[i] << " - ratio : " << ratio ;
      
      beta = std::max(beta,ratio);
    }
    OUT_LOG(logDEBUG1) << " beta : " << beta ;
    OUT_LOG(logDEBUG1) << " alpha : " << alpha ;
  }
  
  // jump_progress = 0 .. 1
  double jump_progress = std::max(alpha,beta);
  OUT_LOG(logDEBUG1) << " jump_progress : " << jump_progress ;

  // if finished jumping reset to stand with joint PID
  // turn off eef controllers and self
  if(jump_progress > 1.0){
    ctrl->open_plugin("reset-trajectory");
    ctrl->open_plugin("joint-PID-controller");

    ctrl->close_plugin("eef-PID-controller");
    ctrl->close_plugin(plugin_namespace);
    return;
  }
  
  static Vector3d com_x = ctrl->get_data<Vector3d>("center_of_mass.x");
  // Set takeoff angle
  static double angle_of_elevation = ctrl->get_data<double>(plugin_namespace + ".angle-of-elevation");
  static double heading = ctrl->get_data<double>(plugin_namespace + ".heading");
  //distance to goal
  static double range = ctrl->get_data<double>(plugin_namespace + ".range");

#ifndef USE_OSG_DISPLAY
  com_x.pose = Pacer::GLOBAL;
  Utility::visualize.push_back( Pacer::VisualizablePtr( new Pacer::Point(  Vector3d(cos(heading)*range,sin(heading)*range,0) + com_x,   Vector3d(1,0,1),0.1)));
#endif
  
  static Vector3d v_liftoff;
  if(start_jump_time == t){
    
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
    
    v_liftoff = Origin3d(v_forward,v_sideways,v_vertical);
    OUTLOG(v_liftoff.norm(), "||v_liftoff||", logERROR);
    OUTLOG(fabs(v_spatial), "|v_spatial|", logERROR);
    
    // Check that we did this right
    assert(fabs( v_liftoff.norm() - fabs(v_spatial) ) < Pacer::NEAR_ZERO);
  }
  
  OUT_LOG(logDEBUG) << "Jumping";
  
  boost::shared_ptr<Ravelin::Pose3d>
    base_frame( new Ravelin::Pose3d(ctrl->get_data<Ravelin::Pose3d>("base_link_frame")));

  // Get base state then set to global origin
  Ravelin::VectorNd local_q = ctrl->get_generalized_value(Pacer::Controller::position);
  int NUM_JOINT_DOFS = local_q.rows()-7;
  local_q.set_sub_vec(NUM_JOINT_DOFS,Utility::pose_to_vec(Ravelin::Pose3d()));
  
  static Ravelin::VectorNd start_vel = ctrl->get_base_value(Pacer::Controller::velocity);

  static Ravelin::VectorNd goal_vel = VectorNd::zero(6);
  goal_vel.set_sub_vec(0,v_liftoff);
  
  VectorNd acc,vel;
  
  // Velocity
  // v = vf * (t*t)
  (vel = goal_vel) -= start_vel;
  (acc = vel) *= dsigmoid(jump_progress*10.0 - 5.0);
  vel *= sigmoid(jump_progress*10.0 - 5.0);
  vel += start_vel;
  
  // Acceleration
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
    Ravelin::Origin3d xd_foot,xdd_foot;
    
    // Calc jacobian for AB at this EEF
    Ravelin::MatrixNd J = ctrl->calc_link_jacobian(local_q,eef_names_[i]);
    
    // Now that model state is set ffrom jacobian calculation
    if(start_jump_time == t){
      x_foot_goal[i] = ctrl->get_data<Ravelin::Origin3d>(eef_names_[i]+".goal.x");
    }
    
    J.block(0,3,NUM_JOINT_DOFS,NUM_JOINT_DOFS+6).mult(vel,xd_foot,-1,0);
    J.block(0,3,NUM_JOINT_DOFS,NUM_JOINT_DOFS+6).mult(acc,xdd_foot,-1,0);
    
    x_foot_goal[i] += xd_foot*dt;
    ctrl->set_data<bool>(eef_names_[i]+".stance",true);
    ctrl->set_data<Ravelin::Origin3d>(eef_names_[i]+".goal.x",x_foot_goal[i]);
    ctrl->set_data<Ravelin::Origin3d>(eef_names_[i]+".goal.xd",xd_foot);
    ctrl->set_data<Ravelin::Origin3d>(eef_names_[i]+".goal.xdd",xdd_foot);
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
    ctrl->remove_data(foot_names[i]+".stance");
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
