#include <Pacer/controller.h>
std::string plugin_namespace;

void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t){
  
  // init a key-value mapping to each joint dof of the robot
  std::map<std::string,std::vector<std::string> >
    joint_names_map = ctrl->make_id_value_map<std::string>();
  
  // Populate a map with joint names and degrees of freedom to find the joint dof mapping to generalized values
  for (std::map<std::string,std::vector<std::string> >::iterator it = joint_names_map.begin();
       it != joint_names_map.end(); it++) {
    for (int j = 0; j<(*it).second.size(); j++) {
      (*it).second[j] = std::string((*it).first + ":" + std::to_string(j));
    }
  }
  
  // Convert the map to a generalized vector
  std::vector<std::string> generalized_names;
  ctrl->convert_to_generalized(joint_names_map,generalized_names);

  // Get joint values
  Ravelin::VectorNd q,qd,qdd,fext,q_des,qd_des,qdd_des,u_des;
  
  ctrl->get_joint_generalized_value(Pacer::Robot::position,q);
  ctrl->get_joint_generalized_value(Pacer::Robot::velocity,qd);
  ctrl->get_joint_generalized_value(Pacer::Robot::acceleration,qdd);
  ctrl->get_joint_generalized_value(Pacer::Robot::load,fext);

  ctrl->get_joint_generalized_value(Pacer::Robot::position_goal,q_des);
  ctrl->get_joint_generalized_value(Pacer::Robot::velocity_goal,qd_des);
  ctrl->get_joint_generalized_value(Pacer::Robot::acceleration_goal,qdd_des);
  ctrl->get_joint_generalized_value(Pacer::Robot::load_goal,u_des);
  
  Ravelin::VectorNd base_x,base_xd,base_xdd,base_fext;
  
  ctrl->get_base_value(Pacer::Robot::position,base_x);
  ctrl->get_base_value(Pacer::Robot::velocity,base_xd);
  ctrl->get_base_value(Pacer::Robot::acceleration,base_xdd);
  ctrl->get_base_value(Pacer::Robot::load,base_fext);
  
  
  
  // Arbitrary data
  std::vector<std::string> to_print;
  // Frames
  {
    to_print.clear();
    ctrl->get_data<std::vector<std::string> >(plugin_namespace+"print-data.pose3",to_print);

    Ravelin::Pose3d data;
    for (std::vector<std::string>::iterator it = to_print.begin();
         it != to_print.end(); it++) {
      if(ctrl->get_data<Ravelin::Pose3d>(*it,data))
        std::cout << (*it) << " " << data << std::endl;
    }
  }
  
  // Vector3
  {
    to_print.clear();
    ctrl->get_data<std::vector<std::string> >(plugin_namespace+"print-data.vec3",to_print);
    
    Ravelin::Origin3d data;
    for (std::vector<std::string>::iterator it = to_print.begin();
         it != to_print.end(); it++) {
      if(ctrl->get_data<Ravelin::Origin3d>(*it,data))
        std::cout << (*it) << " " << data << std::endl;
    }
  }
  
  // Bool
  {
    to_print.clear();
    ctrl->get_data<std::vector<std::string> >(plugin_namespace+"print-data.bool",to_print);
    
    bool data;
    for (std::vector<std::string>::iterator it = to_print.begin();
         it != to_print.end(); it++) {
      if(ctrl->get_data<bool>(*it,data))
        std::cout << (*it) << " " << data << std::endl;
   }
  }
  
  // double
  {
    to_print.clear();
    ctrl->get_data<std::vector<std::string> >(plugin_namespace+"print-data.real",to_print);
    
    double data;
    for (std::vector<std::string>::iterator it = to_print.begin();
         it != to_print.end(); it++) {
      if(ctrl->get_data<double>(*it,data))
         std::cout << (*it) << " " << data << std::endl;
    }
  }
  
  int ndofs = q.rows();
  
//  std::cout << t << " JOINT:gc\t: U\t| Q\t: des\t: err\t|Qd\t: des\t: err\t|Qdd\t: des\t: err" << std::endl;
  printf("JOINT:gc\t|    U   ||    Q   :  des   :  err   ||   Qd   :  des   :   err  ||  Qdd   :  des   :  err   \n");
  for(unsigned i=0;i< ndofs;i++){
    printf("%s:%d\t", generalized_names[i].c_str(),i);
    printf("|%8.3f|", u_des[i]);
    printf("|%8.3f:%8.3f:%8.3f|", q[i],q_des[i],(q[i] - q_des[i]));
    printf("|%8.3f:%8.3f:%8.3f|", qd[i],qd_des[i],(qd[i] - qd_des[i]));
    printf("|%8.3f:%8.3f:%8.3f|", qdd[i],qdd_des[i],(qdd[i] - qdd_des[i]));
    printf("\n");
//      std::cout << generalized_names[i] << ":"<< i
//      << "\t "  <<  std::setprecision(3) << u_des[i]
//      << "\t| " << q[i]
//      << "\t "  << q_des[i]
//      << "\t "  << (q[i] - q_des[i])
//      << "\t| " << qd[i]
//      << "\t "  << qd_des[i]
//      << "\t "  << (qd[i] - qd_des[i])
//      << "\t| " << qdd[i]
//      << "\t "  << qdd_des[i]
//      << "\t "  << (qdd[i] - qdd_des[i]) << std::endl;
  }
  /*
  OUTLOG(data->roll_pitch_yaw,"roll_pitch_yaw",logINFO);
  OUTLOG(data->zero_moment_point,"ZmP",logINFO);
  OUTLOG(data->center_of_mass_x,"CoM_x",logINFO);
  OUTLOG(data->center_of_mass_xd,"CoM_xd",logINFO);
  OUTLOG(data->center_of_mass_xdd,"CoM_xdd",logINFO);
  OUTLOG(data->q,"q",logINFO);
  OUTLOG(data->qd,"qd",logINFO);
  OUTLOG(last_qd_des,"qd_des{t-1}",logINFO);
  OUTLOG(data->qdd,"qdd",logINFO);
  OUTLOG(last_qdd_des,"qdd_des{t-1}",logINFO);
  OUTLOG(q_des,"q_des",logINFO);
  OUTLOG(qd_des,"qd_des",logINFO);
  OUTLOG(qdd_des,"qdd_des",logINFO);
  OUTLOG(data->generalized_fext,"fext",logINFO);
  OUTLOG(data->generalized_q,"generalized_q",logDEBUG);
  OUTLOG(data->generalized_qd,"generalized_qd",logDEBUG);
  
  last_qdd_des = qdd_des;
  last_qd_des = qd_des;
  OUTLOG(uff,"uff",logINFO);
  OUTLOG(ufb,"ufb",logINFO);
  OUTLOG(u,"u",logINFO);
  
  Ravelin::MatrixNd Jf;
  for(int i=0;i<NUM_EEFS;i++){
    boost::shared_ptr<Ravelin::Pose3d> event_frame(new Ravelin::Pose3d(x_des[i].pose));
    EndEffector& foot = eefs_[i];
    
    // Positional Correction
    Ravelin::Vector3d x_now_g = Ravelin::Pose3d::transform_point(Moby::GLOBAL,Ravelin::Vector3d(0,0,0,eefs_[i].link->get_pose()));
    Ravelin::Vector3d x_des_g = Ravelin::Pose3d::transform_point(Moby::GLOBAL,x_des[i]);
    Ravelin::Vector3d x_err  = x_des_g - x_now_g;
    OUTLOG( x_now_g,foot.id + "_x",logINFO);
    OUTLOG( x_des_g,foot.id + "_x_des",logINFO);
    OUTLOG( x_err,foot.id + "_x_err",logINFO);
    
    
    // Remove portion of foot velocity that can't be affected by corrective forces
    event_frame->x = Ravelin::Pose3d::transform_point(x_des[i].pose,Ravelin::Vector3d(0,0,0,eefs_[i].link->get_pose()));
    dbrobot_->calc_jacobian(event_frame,eefs_[i].link,Jf);
    Ravelin::SharedConstMatrixNd Jb = Jf.block(0,3,NUM_JOINTS,NDOFS);
    Ravelin::SharedConstVectorNd vb = data->generalized_qd.segment(NUM_JOINTS,NDOFS);
    Jb.mult(vb,workv3_);
    workv3_.pose = x_des[i].pose;
    
    // Velocity Correction
    // Need to account for base being a moving frame (but only converting as a static frame)
    Ravelin::Vector3d xd_now = (Ravelin::Pose3d::transform_vector(x_des[i].pose,eefs_[i].link->get_velocity().get_linear()) - workv3_);
    Ravelin::Vector3d xd_err = xd_des[i] - xd_now;
    OUTLOG( xd_now,foot.id + "_xd",logINFO);
    OUTLOG( xd_des[i],foot.id + "_xd_des",logINFO);
    OUTLOG( xd_err,foot.id + "_xd_err",logINFO);
  }
  
  int ii = 0;
  for(int i=0;i<NUM_EEFS;i++){
    OUT_LOG(logINFO) << eefs_[i].id << " contacts: " << eefs_[i].point.size();
    for(int j=0;j<eefs_[i].point.size();j++,ii++){
      OUTLOG(eefs_[i].point[j],eefs_[i].id + "_point[" + std::to_string(i) + "]",logINFO);
      OUTLOG(eefs_[i].normal[j],eefs_[i].id + "_normal[" + std::to_string(i) + "]",logINFO);
      OUTLOG(eefs_[i].impulse[j],eefs_[i].id + "_impulse[" + std::to_string(i) + "]",logINFO);
      OUTLOG(eefs_[i].mu_coulomb[j],eefs_[i].id + "_mu[" + std::to_string(i) + "]",logINFO);
    }
  }
  OUT_LOG(logINFO) << "num_contacts = " << ii;
  OUT_LOG(logINFO) << "==============================================" << std::endl;
  */
  
}

/** This is a quick way to register your plugin function of the form:
 * void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t)
 */
#include "register-plugin"
