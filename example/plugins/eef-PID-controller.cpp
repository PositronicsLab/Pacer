#include <Pacer/utilities.h>
#include <Pacer/controller.h>

#include "plugin.h"

boost::shared_ptr<Pacer::Controller> ctrl_ptr;
using namespace Pacer;
using namespace Ravelin;

boost::shared_ptr<Pose3d> base_link_frame;


class eefPID {
public:
  eefPID() { init(); }

    Ravelin::VectorNd u;

  struct Gains
  {
    double perr_sum;
    double kp;
    double kv;
    double ki;
  };
    
  int num_eefs;
  std::vector<std::vector<Gains> > gains;
  std::vector<Ravelin::Vector3d>   eef_u;
  std::vector<std::string>         eef_names;
  
  Ravelin::VectorNd q,
                    qd;

  void init(){

    eef_names = ctrl_ptr->get_data<std::vector<std::string> >(plugin_namespace+".id");
    num_eefs = eef_names.size();
    eef_u.resize(num_eefs);
    
    std::vector<double>
        Kp = ctrl_ptr->get_data<std::vector<double> >(plugin_namespace+".gains.kp"),
        Kv = ctrl_ptr->get_data<std::vector<double> >(plugin_namespace+".gains.kv"),
        Ki = ctrl_ptr->get_data<std::vector<double> >(plugin_namespace+".gains.ki");
 
    assert(num_eefs*3 == Kp.size());
    assert(num_eefs*3 == Kv.size());
    assert(num_eefs*3 == Ki.size());
    
    for(int i=0,ii=0;i<eef_names.size();i++){
      gains.push_back(std::vector<Gains>(3));
      for(int j=0;j<3;j++,ii++){
        gains[i][j].kp = Kp[ii];
        gains[i][j].kv = Kv[ii];
        gains[i][j].ki = Ki[ii];
        gains[i][j].perr_sum = 0;
      }
    }

    OUT_LOG(logERROR) << "Controller: " << plugin_namespace << " inited!";
  }

  void update(){
    
    base_link_frame = boost::shared_ptr<Pose3d>( new Ravelin::Pose3d(ctrl_ptr->get_data<Ravelin::Pose3d>("base_link_frame")));

    Ravelin::VectorNd q;
    ctrl_ptr->get_generalized_value(Pacer::Controller::position,q);
    u.set_zero(ctrl_ptr->joint_dofs());
    q.set_sub_vec(ctrl_ptr->joint_dofs(),Utility::pose_to_vec(Ravelin::Pose3d()));
    OUTLOG(q,"control_pose",logERROR);
    
    for (int i=0; i< num_eefs; i++)
    {
      Ravelin::Origin3d
      pos =  ctrl_ptr->get_foot_value(eef_names[i],Pacer::Robot::position),
      vel =  ctrl_ptr->get_foot_value(eef_names[i],Pacer::Robot::velocity);
      
      Ravelin::Origin3d
      pos_des =  ctrl_ptr->get_foot_value(eef_names[i],Pacer::Robot::position_goal),
      vel_des =  ctrl_ptr->get_foot_value(eef_names[i],Pacer::Robot::velocity_goal);
      
      Ravelin::Origin3d base_joint = ctrl_ptr->get_data<Ravelin::Origin3d>(eef_names[i]+".base");
      double max_reach = ctrl_ptr->get_data<double>(eef_names[i]+".reach");
      
      Ravelin::Origin3d goal_from_base_joint = pos_des - base_joint;
      double goal_reach = goal_from_base_joint.norm();
      OUT_LOG(logDEBUG1) << " goal_reach < max_reach : " << goal_reach<<  " < "  << max_reach ;
      
      if(goal_reach > max_reach){
        pos_des = base_joint + goal_from_base_joint * (max_reach/goal_reach);
        vel_des.set_zero();
      }
      
#ifdef USE_OSG_DISPLAY
      {
        Vector3d p1 = Pose3d::transform_point(Pacer::GLOBAL,Ravelin::Vector3d(pos_des,base_link_frame));
        Vector3d p2 = Pose3d::transform_point(Pacer::GLOBAL,Ravelin::Vector3d(pos,base_link_frame));
        //      Vector3d a = Pose3d::transform_vector(Pacer::GLOBAL,xdd)/100;
        Utility::visualize.push_back( Pacer::VisualizablePtr( new Pacer::Point( p1,   Vector3d(0,1,0),0.01)));
        Utility::visualize.push_back( Pacer::VisualizablePtr( new Ray(  p2,   p1,   Vector3d(1,0,0),0.01)));
        Vector3d v = Pose3d::transform_vector(Pacer::GLOBAL,vel_des)/10;
        Utility::visualize.push_back( Pacer::VisualizablePtr( new Ray(  v+p1,   p1,   Vector3d(0,1,0),0.01)));
      }
#endif
      
      Ravelin::Origin3d
        perr = pos_des-pos,
        derr = vel_des-vel;
      for(int j=0;j<3;j++){
        const double KP = gains[i][j].kp;
        const double KV = gains[i][j].kv;
        const double KI = gains[i][j].ki;

        gains[i][j].perr_sum += perr[j];
        double ierr = gains[i][j].perr_sum;
        
        eef_u[i][j] = (perr[j]*KP + derr[j]*KV + ierr*KI);
      }
      
      OUTLOG(perr,eef_names[i]+"_perr",logDEBUG1);
      OUTLOG(derr,eef_names[i]+"_derr",logDEBUG1);

      OUTLOG(eef_u[i],eef_names[i]+"_U",logDEBUG1);
      
      Ravelin::MatrixNd J = ctrl_ptr->calc_link_jacobian(q,eef_names[i]);
      OUTLOG(J,eef_names[i]+"_J",logDEBUG1);


      Ravelin::VectorNd workv_;
      J.block(0,3,0,J.columns()).transpose_mult(eef_u[i],workv_);
      
      OUTLOG(workv_,eef_names[i]+"_joint_u",logDEBUG1);
      u += workv_.segment(0,u.rows());
    }
  }
};

void loop(){
boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);  
  ctrl_ptr = ctrl;
  static eefPID pid;
      
  pid.update();

  bool acceleration_ff = false;
  ctrl->get_data<bool>(plugin_namespace+".acceleration",acceleration_ff);
  
  if(acceleration_ff){
    Ravelin::VectorNd u = ctrl->get_joint_generalized_value(Pacer::Controller::acceleration_goal);
    OUTLOG(pid.u,"eef_pid_U",logDEBUG);
    u += pid.u;
    ctrl->set_joint_generalized_value(Pacer::Controller::acceleration_goal,u);
  } else {
    Ravelin::VectorNd u = ctrl->get_joint_generalized_value(Pacer::Controller::load_goal);
    OUTLOG(pid.u,"eef_pid_U",logDEBUG);
    u += pid.u;
    ctrl->set_joint_generalized_value(Pacer::Controller::load_goal,u);
  }
}


void setup(){
}