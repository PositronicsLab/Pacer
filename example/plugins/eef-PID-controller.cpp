#include <Pacer/utilities.h>
#include <Pacer/controller.h>

std::string plugin_namespace;

boost::shared_ptr<Pacer::Controller> ctrl_ptr;

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
    Ravelin::VectorNd q;
    ctrl_ptr->get_generalized_value(Pacer::Controller::position,q);
    u.set_zero(q.rows()-Pacer::NEULER);
      
    for (int i=0; i< num_eefs; i++)
    {
      Ravelin::Vector3d
        pos = ctrl_ptr->get_data<Ravelin::Vector3d>(eef_names[i]+".state.x"),
        vel = ctrl_ptr->get_data<Ravelin::Vector3d>(eef_names[i]+".state.xd");      
      
      Ravelin::Vector3d
        pos_des = ctrl_ptr->get_data<Ravelin::Vector3d>(eef_names[i]+".goal.x"),
        vel_des = ctrl_ptr->get_data<Ravelin::Vector3d>(eef_names[i]+".goal.xd");    
      
      pos.pose = Moby::GLOBAL;
      pos_des.pose = Moby::GLOBAL;
      vel.pose = Moby::GLOBAL;
      vel_des.pose = Moby::GLOBAL;
      
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
      
      OUTLOG(perr,eef_names[i]+"_perr",logDEBUG1);
      OUTLOG(derr,eef_names[i]+"_derr",logDEBUG1);

      OUTLOG(eef_u[i],eef_names[i]+"_U",logDEBUG1);
      
      Ravelin::MatrixNd J;
      
      eef_u[i].pose = Moby::GLOBAL;
      
      //if(!ctrl_ptr->get_data<Ravelin::MatrixNd>(eef_names[i]+".J",J))
        J = ctrl_ptr->calc_link_jacobian(q,eef_names[i]);
            
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

  Ravelin::VectorNd u = ctrl->get_joint_generalized_value(Pacer::Controller::load_goal);
  OUTLOG(pid.u,"eef_pid_U",logDEBUG);
  u += pid.u.segment(0,u.rows());
  ctrl->set_joint_generalized_value(Pacer::Controller::load_goal,u);
      
}

/** This is a quick way to register your plugin function of the form:
  * void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t)
  */
#include "register-plugin"
