#include <Pacer/controller.h>
#include <Pacer/utilities.h>

std::string plugin_namespace;
using namespace Ravelin;

const double grav     = 9.81; // m / s*s

boost::shared_ptr<Pacer::Controller> ctrl;

struct Trajectory {
  VectorNd coefs;
  VectorNd T;
  VectorNd X;
  Vector2d Xd;
};

std::vector<Trajectory> init_Nd_cubic_spline(const std::vector<VectorNd>& X,const std::vector<VectorNd>& Xd,const std::vector<double>& T){
  int knots = X.size();
  int dims = X[0].rows();
  std::vector<Trajectory> trajs(dims);
  for(int d=0;d<dims;d++){
    Trajectory& traj = trajs[d];
    traj.T = VectorNd(knots);
    traj.X = VectorNd(knots);
    traj.Xd[0] = Xd[0][d];
    traj.Xd[1] = Xd[1][d];
    for(int i=0;i<knots;i++){
      traj.X[i] = X[i][d];
      traj.T[i] = T[i];
    }
    OUTLOG(traj.T,"T"+boost::icl::to_string<double>::apply(d),logDEBUG);
    OUTLOG(traj.X,"X"+boost::icl::to_string<double>::apply(d),logDEBUG);
  }
  return trajs;
}

bool calc_Nd_cubic_spline(std::vector<Trajectory>& trajs){
  bool RETURN_FLAG = true;
  for(int d=0;d<trajs.size();d++){
    Trajectory& traj = trajs[d];
    Utility::calc_cubic_spline_coefs(traj.T,traj.X,traj.Xd,traj.coefs);
    //      RETURN_FLAG = false;
  }
  return RETURN_FLAG;
}

bool eval_Nd_cubic_spline(std::vector<Trajectory>& trajs, double t, VectorNd& x, VectorNd& xd, VectorNd& xdd){
  bool RETURN_FLAG = true;
  for(int d=0;d<trajs.size();d++){
    Trajectory& traj = trajs[d];
    if(!Utility::eval_cubic_spline(traj.coefs,traj.T,t,x[d],xd[d],xdd[d]))
      RETURN_FLAG = false;
  }
  return RETURN_FLAG;
}

std::vector<Trajectory> calc_jump(){
  // Set takeoff angle
  double angle_of_elevation = ctrl->get_data<double>(plugin_namespace + ".angle-of-elevation");
  double heading_angle = ctrl->get_data<double>(plugin_namespace + ".heading");
  //distance to goal
  double x = ctrl->get_data<double>(plugin_namespace + ".range");
  double duration = ctrl->get_data<double>(plugin_namespace + ".duration");
  
  // 3d velocity
  double v_spatial = sqrt((grav * x) / sin(2.0 * angle_of_elevation));
  OUTLOG(v_spatial, "v_spatial", logDEBUG);
  
  // 2d (planar) velocity
  double v_horizontal = v_spatial * std::cos(angle_of_elevation);
  OUTLOG(v_horizontal, "v_horizontal", logDEBUG);
  
  // directional velocities
  // X
  double v_forward = v_horizontal * std::cos(heading_angle);
  OUTLOG(v_forward, "v_forward", logDEBUG);
  // Y
  double  v_sideways = v_horizontal * std::sin(heading_angle);
  OUTLOG(v_sideways, "v_sideways", logDEBUG);
  // Z
  double v_vertical = v_spatial * std::sin(angle_of_elevation);
  OUTLOG(v_vertical, "v_vertical", logDEBUG);
  
  Vector3d v_liftoff(v_forward,v_sideways,v_vertical);
  OUTLOG(v_liftoff.norm(), "||v_liftoff||", logERROR);
  OUTLOG(fabs(v_spatial), "|v_spatial|", logERROR);
  
  // Check that we did this right
  assert(fabs( v_liftoff.norm() - fabs(v_spatial) ) < Pacer::NEAR_ZERO);
  
  std::vector<VectorNd> X, Xd;
  //Xd0
  Xd.push_back(VectorNd::zero(3));
  //Xdf
  Xd.push_back(VectorNd(3,v_liftoff.data()));
  
  // X0
  X.push_back(VectorNd::zero(3));
  // Xf
  X.push_back(VectorNd::zero(3));
  X[1][2] += 0.05;
  std::vector<double> T;
  T.push_back(0);
  T.push_back(duration);
  
  std::vector<Trajectory> leap_trajectory = init_Nd_cubic_spline(X,Xd,T);
  if(!calc_Nd_cubic_spline(leap_trajectory)){
    throw std::runtime_error("Jump calculation failed");
  }
  
  return leap_trajectory;
}


void Update(const boost::shared_ptr<Pacer::Controller>& ctrl_ptr, double t){
  ctrl = ctrl_ptr;
  static double start_jump_time = t;
  double duration = ctrl->get_data<double>(plugin_namespace + ".duration");

  static bool first_step = true;
  
  const  std::vector<std::string>
  eef_names_ = ctrl->get_data<std::vector<std::string> >("init.end-effector.id");
  
  static std::vector<Vector3d> x_foot_goal(eef_names_.size());
  
  VectorNd x(3), xd(3), xdd(3);
  
  std::cout << start_jump_time << " : " << t << std::endl;
  
  
  { // Do spline calculations
    static std::vector<Trajectory> leap_spline;
    
    if(t >= start_jump_time && t < start_jump_time+duration){
      
      if (leap_spline.empty()) {
        std::cout << "Calculating jump" << std::endl;
        leap_spline = calc_jump();
      }
      
      // Make sure we're not out of time bounds
      //      if(leap_spline[0].T[leap_spline[0].T.rows()-1] > t-start_time)
      //        throw std::runtime_error("End of jump!");
      
      // Disable joint Feedback
      VectorNd q_current, qd_current;
      ctrl->get_joint_generalized_value(Pacer::Controller::position,q_current);
      ctrl->set_joint_generalized_value(Pacer::Controller::position_goal,q_current);
      
      ctrl->get_joint_generalized_value(Pacer::Controller::velocity,qd_current);
      ctrl->set_joint_generalized_value(Pacer::Controller::velocity_goal,qd_current);
      
      // Find base trajectory
      if(!eval_Nd_cubic_spline(leap_spline,t-start_jump_time,x,xd,xdd)){
        for(unsigned i=0;i<eef_names_.size();i++){
          ctrl->set_data<Ravelin::Vector3d>(eef_names_[i]+".goal.x",x_foot_goal[i]);
          ctrl->set_data<Ravelin::Vector3d>(eef_names_[i]+".goal.xd",Vector3d(0,0,0));
          ctrl->set_data<Ravelin::Vector3d>(eef_names_[i]+".goal.xdd",Vector3d(0,0,0));
        }
        return;
      }
    } else {
      return;
    }
  }
  
  std::cout << "Jumping" << std::endl;
  
  static double last_time = -0.001;
  double dt = t - last_time;
  last_time = t;
  
  boost::shared_ptr<Ravelin::Pose3d> base_frame( new Ravelin::Pose3d(ctrl->get_data<Ravelin::Pose3d>("base_link_frame")));
  
  Ravelin::VectorNd qd  = ctrl->get_joint_generalized_value(Pacer::Controller::velocity);
  Ravelin::VectorNd qdd = ctrl->get_joint_generalized_value(Pacer::Controller::acceleration);
  // Initialize end effectors
  Ravelin::VectorNd local_q = ctrl->get_generalized_value(Pacer::Controller::position);
  int NUM_JOINT_DOFS = qd.rows();
  local_q.set_sub_vec(NUM_JOINT_DOFS,Utility::pose_to_vec(Ravelin::Pose3d()));
  
  for(unsigned i=0;i<eef_names_.size();i++){
    ctrl->set_model_state(local_q);
    Ravelin::Vector3d xd_foot,xdd_foot;
    
    // Calc jacobian for AB at this EEF
    Ravelin::MatrixNd J = ctrl->calc_link_jacobian(local_q,eef_names_[i]);
    
    // Now that model state is set ffrom jacobian calculation
    if(first_step){
      const boost::shared_ptr<Ravelin::RigidBodyd>  link = ctrl->get_link(eef_names_[i]);
      x_foot_goal[i] = Ravelin::Pose3d::transform_point(Pacer::GLOBAL,Ravelin::Vector3d(0,0,0,link->get_pose()));
    }
    
    J.block(0,3,NUM_JOINT_DOFS,NUM_JOINT_DOFS+3).mult(xd,xd_foot,-1,0);
    J.block(0,3,NUM_JOINT_DOFS,NUM_JOINT_DOFS+3).mult(xdd,xdd_foot,-1,0);
    
    x_foot_goal[i].pose = base_frame;
    xd_foot.pose = base_frame;
    xdd_foot.pose = base_frame;
    x_foot_goal[i] += xd_foot*dt;
    ctrl->set_data<Ravelin::Vector3d>(eef_names_[i]+".goal.x",x_foot_goal[i]);
    ctrl->set_data<Ravelin::Vector3d>(eef_names_[i]+".goal.xd",xd_foot);
    ctrl->set_data<Ravelin::Vector3d>(eef_names_[i]+".goal.xdd",xdd_foot);
  }
  first_step = false;
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
    plugin_namespace = std::string(std::string(name));
    
    int priority = ctrl->get_data<double>(plugin_namespace+".priority");
    
    ctrl->add_plugin_update(priority,name,&update);
    ctrl->add_plugin_deconstructor(name,&deconstruct);
  }
}
