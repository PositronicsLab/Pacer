#include <Pacer/controller.h>
#include <Pacer/utilities.h>

#include "plugin.h"
using namespace Ravelin;

const double grav     = 9.81; // m / s*s

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
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);

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
  std::vector<double> T;
  // Only 2 vel values (start and end)
  //Xd0
  Xd.push_back(VectorNd::zero(3));
  //Xdf
  Xd.push_back(VectorNd(3,v_liftoff.data()));
  
  // n pos values (start, via, and end)
  // X0
  X.push_back(VectorNd::zero(3));
  T.push_back(0);
  
  // Xf
  X.push_back(VectorNd::zero(3));
  X.back()[0] += 0.05;
  X.back()[2] += 0.08;
  T.push_back(T.back()+duration);
  
  std::vector<Trajectory> leap_trajectory = init_Nd_cubic_spline(X,Xd,T);
  if(!calc_Nd_cubic_spline(leap_trajectory)){
    throw std::runtime_error("Jump calculation failed");
  }
  
  return leap_trajectory;
}


void loop(){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  static double start_jump_time = t;
  double duration = ctrl->get_data<double>(plugin_namespace + ".duration");
  static Vector3d com_x = ctrl->get_data<Vector3d>("center_of_mass.x");
  
  static bool first_step = true;
  
  const  std::vector<std::string>
  eef_names_ = ctrl->get_data<std::vector<std::string> >("init.end-effector.id");
  
  static std::vector<Vector3d> x_end_effector_goal(eef_names_.size());
  
  VectorNd x(3), xd(3), xdd(3);
  
  std::cout << start_jump_time << " : " << t << std::endl;
  
  
  { // Do spline calculations
    static std::vector<Trajectory> leap_spline;
    
    if(t >= start_jump_time){
      
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
        ctrl->open_plugin("reset-trajectory");
        ctrl->close_plugin("eef-PID-controller");
        ctrl->close_plugin(plugin_namespace);
        return;
      }
      
#ifndef USE_OSG_DISPLAY
      com_x.pose = Pacer::GLOBAL;
      double heading = ctrl->get_data<double>(plugin_namespace + ".heading");
      double range = ctrl->get_data<double>(plugin_namespace + ".range");
      
      VISUALIZE(POINT(  Vector3d(cos(heading)*range,sin(heading)*range,0) + com_x,   Vector3d(1,0,1),0.1));
      
      for(double i=start_jump_time;i<start_jump_time+duration;i+=0.01){
        VectorNd x(3), xd(3), xdd(3);
        if(eval_Nd_cubic_spline(leap_spline,i,x,xd,xdd)){
          VISUALIZE(RAY(  Vector3d(x[0],x[1],x[2]) + com_x, Vector3d(x[0],x[1],x[2]) + com_x + Vector3d(xd[0],xd[1],xd[2])*0.01,   Vector3d(1,0,0),0.02));
          //      OUTLOG(x,"x",logERROR);
        }
      }
#endif
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
      x_end_effector_goal[i] = Ravelin::Pose3d::transform_point(Pacer::GLOBAL,Ravelin::Vector3d(0,0,0,link->get_pose()));
    }
    
    J.block(0,3,NUM_JOINT_DOFS,NUM_JOINT_DOFS+3).mult(xd,xd_foot,-1,0);
    J.block(0,3,NUM_JOINT_DOFS,NUM_JOINT_DOFS+3).mult(xdd,xdd_foot,-1,0);
    
    x_end_effector_goal[i].pose = base_frame;
    xd_foot.pose = base_frame;
    xdd_foot.pose = base_frame;
    x_end_effector_goal[i] += xd_foot*dt;
    ctrl->set_data<Ravelin::Vector3d>(eef_names_[i]+".goal.x",x_end_effector_goal[i]);
    ctrl->set_data<Ravelin::Vector3d>(eef_names_[i]+".goal.xd",xd_foot);
    ctrl->set_data<Ravelin::Vector3d>(eef_names_[i]+".goal.xdd",xdd_foot);
  }
  first_step = false;
}

void setup(){
}