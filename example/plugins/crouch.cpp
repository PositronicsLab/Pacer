#include <Pacer/controller.h>
#include <Pacer/utilities.h>

#include "plugin.h"

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


void loop(){
boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  ::ctrl = ctrl;
  static double start_jump_time = t;
  double duration = ctrl->get_data<double>(plugin_namespace + ".duration");
  static Vector3d com_x = ctrl->get_data<Vector3d>("center_of_mass.x");
  
  const  std::vector<std::string>
  eef_names_ = ctrl->get_data<std::vector<std::string> >("init.end-effector.id");
  
  static std::vector<Origin3d> x_foot_goal(eef_names_.size());
  
  VectorNd x(3), xd(3), xdd(3);
  
  { // Do spline calculations
    static std::vector<Trajectory> crouch_spline;
    
    if(t >= start_jump_time){
      
      if (crouch_spline.empty()) {
        OUT_LOG(logDEBUG) << "Calculating Crouch";
        crouch_spline = calc_crouch();
      }
    
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
    Ravelin::Origin3d xd_foot,xdd_foot;
    
    // Calc jacobian for AB at this EEF
    Ravelin::MatrixNd J = ctrl->calc_link_jacobian(local_q,eef_names_[i]);
    
    // Now that model state is set ffrom jacobian calculation
    if(start_jump_time == t){
      x_foot_goal[i] = ctrl->get_data<Ravelin::Origin3d>(eef_names_[i]+".goal.x");
    }
    
    J.block(0,3,NUM_JOINT_DOFS,NUM_JOINT_DOFS+3).mult(xd,xd_foot,-1,0);
    J.block(0,3,NUM_JOINT_DOFS,NUM_JOINT_DOFS+3).mult(xdd,xdd_foot,-1,0);
    
    x_foot_goal[i] += xd_foot*dt;
    ctrl->set_data<bool>(eef_names_[i]+".stance",true);
    ctrl->set_data<Ravelin::Origin3d>(eef_names_[i]+".goal.x",x_foot_goal[i]);
    ctrl->set_data<Ravelin::Origin3d>(eef_names_[i]+".goal.xd",xd_foot);
    ctrl->set_data<Ravelin::Origin3d>(eef_names_[i]+".goal.xdd",xdd_foot);
  }
}
void setup(){
}