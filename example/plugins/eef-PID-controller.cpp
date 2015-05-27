#include <Pacer/utilities.h>
#include <Pacer/controller.h>

std::string plugin_namespace;

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

    eef_names = ctrl_ptr->get_data<std::vector<std::string> >(plugin_namespace+"id");
    num_eefs = eef_names.size();
    eef_u.resize(num_eefs);
    
    std::vector<double>
        Kp = ctrl_ptr->get_data<std::vector<double> >(plugin_namespace+"gains.kp"),
        Kv = ctrl_ptr->get_data<std::vector<double> >(plugin_namespace+"gains.kv"),
        Ki = ctrl_ptr->get_data<std::vector<double> >(plugin_namespace+"gains.ki");
 
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
      Ravelin::Vector3d
      pos = ctrl_ptr->get_data<Ravelin::Vector3d>(eef_names[i]+".state.x"),
      vel = ctrl_ptr->get_data<Ravelin::Vector3d>(eef_names[i]+".state.xd");
      pos.pose = base_link_frame;
      vel.pose = base_link_frame;
      
      Ravelin::Vector3d
      pos_des = ctrl_ptr->get_data<Ravelin::Vector3d>(eef_names[i]+".goal.x"),
      vel_des = ctrl_ptr->get_data<Ravelin::Vector3d>(eef_names[i]+".goal.xd");
      pos_des.pose = base_link_frame;
      vel_des.pose = base_link_frame;
      
      {
        Vector3d p1 = Pose3d::transform_point(Moby::GLOBAL,pos_des);
        Vector3d p2 = Pose3d::transform_point(Moby::GLOBAL,pos);
        //      Vector3d a = Pose3d::transform_vector(Moby::GLOBAL,xdd)/100;
        Utility::visualize.push_back( Pacer::VisualizablePtr( new Pacer::Point( p1,   Vector3d(0,1,0),0.01)));
        Utility::visualize.push_back( Pacer::VisualizablePtr( new Ray(  p2,   p1,   Vector3d(1,0,0),0.01)));
      }
      
      Ravelin::Vector3d 
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
      eef_u[i].pose = base_link_frame;
      
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

void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t){
  
  ctrl_ptr = ctrl;
  static eefPID pid;
      
  pid.update();

  bool acceleration_ff = false;
  ctrl->get_data<bool>(plugin_namespace+"acceleration",acceleration_ff);
  
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

/** This is a quick way to register your plugin function of the form:
  * void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t)
  */
#include "register-plugin"
