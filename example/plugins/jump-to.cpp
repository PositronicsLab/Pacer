#include <Pacer/controller.h>
#include <Pacer/utilities.h>

std::string plugin_namespace;
using namespace Ravelin;

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
    OUTLOG(traj.T,"T"+std::to_string(d),logDEBUG);
    OUTLOG(traj.X,"X"+std::to_string(d),logDEBUG);
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
  double angle_of_elevation = ctrl->get_data<double>(plugin_namespace + "angle-of-elevation");
  double heading_angle = ctrl->get_data<double>(plugin_namespace + "heading");
  //distance to goal
  double x = ctrl->get_data<double>(plugin_namespace + "range");
  double duration = ctrl->get_data<double>(plugin_namespace + "duration");
  
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
  assert(fabs( v_liftoff.norm() - fabs(v_spatial) ) < Moby::NEAR_ZERO);
  
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
  double start_jump_time = ctrl->get_data<double>(plugin_namespace + "start-jump-time");
  double duration = ctrl->get_data<double>(plugin_namespace + "duration");

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
      
      
      // Find base trajectory
      if(!eval_Nd_cubic_spline(leap_spline,t-start_jump_time,x,xd,xdd)){
        for(unsigned i=0;i<eef_names_.size();i++){
          ctrl->set_data<Ravelin::Vector3d>(eef_names_[i]+".goal.x",x_foot_goal[i]);
          ctrl->set_data<Ravelin::Vector3d>(eef_names_[i]+".goal.xd",Vector3d(0,0,0));
          ctrl->set_data<Ravelin::Vector3d>(eef_names_[i]+".goal.xdd",Vector3d(0,0,0));
        }
        return;
//        throw std::runtime_error("Spline evaluation failed!");
      }
      
      VectorNd q_current, qd_current;
      ctrl->get_joint_generalized_value(Pacer::Controller::position,q_current);
      ctrl->set_joint_generalized_value(Pacer::Controller::position_goal,q_current);

      ctrl->get_joint_generalized_value(Pacer::Controller::velocity,qd_current);
      ctrl->set_joint_generalized_value(Pacer::Controller::velocity_goal,qd_current);

    } else {
      std::cout << "Resetting jump" << std::endl;
      static VectorNd q_start  = ctrl->get_data<Ravelin::VectorNd>("init.q");

      if (!leap_spline.empty()) {
        first_step = true;
        leap_spline.clear();
        ctrl->get_joint_generalized_value(Pacer::Controller::position,q_start);
      }
      
      // Disable eef controller gains
      for(unsigned i=0;i<eef_names_.size();i++){
        Ravelin::Vector3d x_foot, xd_foot,xdd_foot;

        ctrl->get_data<Ravelin::Vector3d>(eef_names_[i]+".state.x",x_foot);
        ctrl->get_data<Ravelin::Vector3d>(eef_names_[i]+".state.xd",xd_foot);

        ctrl->set_data<Ravelin::Vector3d>(eef_names_[i]+".goal.x",x_foot);
        ctrl->set_data<Ravelin::Vector3d>(eef_names_[i]+".goal.xd",xd_foot);
        ctrl->set_data<Ravelin::Vector3d>(eef_names_[i]+".goal.xdd",Vector3d(0,0,0));
      }

      VectorNd
        q_goal = ctrl->get_data<Ravelin::VectorNd>("init.q");
      VectorNd
        qd_goal = Ravelin::VectorNd::zero(q_goal.rows()),
        qdd_goal = Ravelin::VectorNd::zero(q_goal.rows());
      
      VectorNd q_target = q_start;
      double alpha = (t-(start_jump_time+duration))/0.25;
      std::cout << "alpha " << alpha << std::endl;
      OUTLOG(q_start, "q_start", logDEBUG);
      OUTLOG(q_goal, "q_goal", logDEBUG);

      if(alpha <= 1){
        q_target *= 1-alpha;
        q_goal *= alpha;
        q_target += q_goal;
      } else {
        q_target = q_goal;
      }
      OUTLOG(q_target, "q_target", logDEBUG);
      ctrl->set_joint_generalized_value(Pacer::Controller::position_goal,q_target);
      ctrl->set_joint_generalized_value(Pacer::Controller::velocity_goal,qd_goal);
      ctrl->set_joint_generalized_value(Pacer::Controller::acceleration_goal,qdd_goal);
      
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
      const Moby::RigidBodyPtr link = ctrl->get_link(eef_names_[i]);
      x_foot_goal[i] = Ravelin::Pose3d::transform_point(Moby::GLOBAL,Ravelin::Vector3d(0,0,0,link->get_pose()));
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


/** This is a quick way to register your plugin function of the form:
 * void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t)
 */
#include "register-plugin"
