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

std::vector<Trajectory> calc_crouch(){
  double duration = ctrl->get_data<double>(plugin_namespace + ".duration");
  std::vector<double> position = ctrl->get_data<std::vector<double> >(plugin_namespace + ".position");
  
  std::vector<VectorNd> X, Xd;
  std::vector<double> T;
  // Only 2 vel values (start and end)
  //Xd0
  Xd.push_back(VectorNd::zero(3));
  //Xdf
  Xd.push_back(VectorNd::zero(3));
  
  // n pos values (start, via, and end)
  // X0
  X.push_back(VectorNd::zero(3));
  T.push_back(0);
  
  // Xf
  X.push_back(VectorNd(3,Vector3d(position[0],position[1],position[2]).data()));
  T.push_back(T.back()+duration);
  
  std::vector<Trajectory> movement_trajectory = init_Nd_cubic_spline(X,Xd,T);
  if(!calc_Nd_cubic_spline(movement_trajectory)){
    throw std::runtime_error("Movement calculation failed");
  }
  
  return movement_trajectory;
}


void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t){
  ::ctrl = ctrl;
  static double start_jump_time = t;
  double duration = ctrl->get_data<double>(plugin_namespace + ".duration");
  static Vector3d com_x = ctrl->get_data<Vector3d>("center_of_mass.x");
  
  static bool first_step = true;
  
  const  std::vector<std::string>
  eef_names_ = ctrl->get_data<std::vector<std::string> >("init.end-effector.id");
  
  static std::vector<Vector3d> x_foot_goal(eef_names_.size());
  
  VectorNd x(3), xd(3), xdd(3);
  
  std::cout << start_jump_time << " : " << t << std::endl;
  
  
  { // Do spline calculations
    static std::vector<Trajectory> crouch_spline;
    
    if(t >= start_jump_time){
      
      if (crouch_spline.empty()) {
        OUT_LOG(logDEBUG) << "Calculating Crouch";
        crouch_spline = calc_crouch();
      }
      
      // Make sure we're not out of time bounds
      //      if(crouch_spline[0].T[crouch_spline[0].T.rows()-1] > t-start_time)
      //        throw std::runtime_error("End of jump!");
      
      // Disable joint Feedback
      VectorNd q_current, qd_current;
      ctrl->get_joint_generalized_value(Pacer::Controller::position,q_current);
      ctrl->set_joint_generalized_value(Pacer::Controller::position_goal,q_current);
      
      ctrl->get_joint_generalized_value(Pacer::Controller::velocity,qd_current);
      ctrl->set_joint_generalized_value(Pacer::Controller::velocity_goal,qd_current);
      
      // Find base trajectory
      if(!eval_Nd_cubic_spline(crouch_spline,t-start_jump_time,x,xd,xdd)){
        return;
      }
    } else {
      return;
    }
  }
  
  OUT_LOG(logDEBUG) << "Crouching";
  
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
    plugin_namespace = std::string(name);
    
    int priority = ctrl->get_data<double>(plugin_namespace+".priority");
    
    ctrl->add_plugin_update(priority,name,&update);
    ctrl->add_plugin_deconstructor(name,&deconstruct);
  }
}
