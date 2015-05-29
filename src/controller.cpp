/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/controller.h>
#include <Pacer/utilities.h>
#include <sys/time.h>
#include <dlfcn.h>
#include <errno.h>
#include <boost/foreach.hpp>
#include <stdlib.h>     /* getenv */

#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <vector>

using namespace Pacer;
  
Controller::Controller(): Robot(){

}

Controller::~Controller(){
  close_plugins();
}


std::vector<void*> handles;
bool Controller::close_plugins(){
  // close the loaded plugin libraries
  for(size_t i = 0; i < handles.size(); ++i){
    dlclose(handles[i]);
  }
  handles.clear();
  _update_priority_map.clear();
  return true;
}

typedef void (*init_t)(const boost::shared_ptr<Controller>, const char*);

bool Controller::init_plugins(){
  OUT_LOG(logDEBUG) << ">> Controller::init_plugins()";

  bool RETURN_FLAG = true;
  close_plugins();
  std::vector<init_t> INIT;

  // call the initializers, if any
  std::vector<std::string> plugin_names = get_data<std::vector<std::string> >("plugin.id");
  std::vector<std::string> plugin_files = get_data<std::vector<std::string> >("plugin.file");
  std::vector<int> plugin_active = get_data<std::vector<int> >("plugin.active");

  // Load all the plugins
  for(unsigned i=0;i<plugin_names.size();i++){
    if(plugin_active[i] == 0) continue;
    std::string filename = plugin_files[i];

  if (!getenv("PACER_PLUGIN_PATH"))
    throw std::runtime_error("Environment variable PACER_PLUGIN_PATH not defined");

    std::string pPath(getenv("PACER_PLUGIN_PATH"));
    std::string lib_path = pPath+"/"+filename;
    OUT_LOG(logINFO) << "Loading Plugin: " << plugin_names[i];
    OUT_LOG(logINFO) << "\tLIB: " << filename.c_str();
    OUT_LOG(logINFO) << "\tPATH: " << pPath.c_str();
    // attempt to read the file
    void* HANDLE = dlopen(lib_path.c_str(), RTLD_LAZY);
    if (!HANDLE)
    {
      std::cerr << "driver: failed to read plugin from " << filename << std::endl;
      std::cerr << "  " << dlerror() << std::endl;
      RETURN_FLAG = false;
    }
 
    handles.push_back(HANDLE);
 
    // attempt to load the initializer
    dlerror();
    INIT.push_back((init_t) dlsym(HANDLE, "init"));
    const char* dlsym_error = dlerror();
    if (dlsym_error)
    {
      std::cerr << "driver warning: cannot load symbol 'init' from " << filename << std::endl;
      std::cerr << "        error follows: " << std::endl << dlsym_error << std::endl;
      INIT.pop_back();
      RETURN_FLAG = false;
    } else {
      // Init the plugin
      (*INIT.back())(this->ptr(),plugin_names[i].c_str());
    }
  }
    
  OUT_LOG(logDEBUG) << "<< Controller::init_plugins()";
  return RETURN_FLAG;
}

// ============================================================================
// =========================== Begin Robot Controller =========================
// ============================================================================

void Controller::control(double t){
    OUT_LOG(logDEBUG) << ">> Controller::control(.)";
  // Import Robot Data
  static double last_time = -0.001;
  const double dt = t - last_time;
  OUTLOG(t,"virtual_time",logERROR);
  OUTLOG(dt,"virtual_time_step",logERROR);
  update();
  lock_state();
#ifdef USE_PLUGINS
  update_plugins(t);
#endif
  unlock_state();
  /*
  OUT_LOG(logINFO)<< "time = "<< t ;

  set_model_state(data->q,data->qd);

   OUT_LOG(logINFO) <<"JOINT:A\t: U\t| Q\t: des\t: err\t|Qd\t: des\t: err\t|Qdd\t: des\t: err";
   for(unsigned i=0,ii=0;i< NUM_JOINTS;i++){
     if(joints_[i])
     for(int j=0;j<joints_[i]->num_dof();j++,ii++){
     OUT_LOG(logINFO)<< std::to_string(j)+joints_[i]->id << ":"<< active_joints_[std::to_string(j)+joints_[i]->id]
               << "\t " <<  std::setprecision(3) << u[ii]
               << "\t| " << joints_[i]->q[j]
               << "\t " << q_des[ii]
               << "\t " << data->q[ii] - q_des[ii]
               << "\t| " << joints_[i]->qd[j]
               << "\t " << qd_des[ii]
               << "\t " <<  data->qd[ii] - qd_des[ii]
               << "\t| " << data->qdd[ii]
               << "\t " << qdd_des[ii]
               << "\t " <<  (data->qdd[ii] - qdd_des[ii]);
     }
   }
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
   // -----------------------------------------------------------------------------

   assert(data->generalized_fext.norm() < 1e+6);
*/
  reset_contact();
  last_time = t;
  OUT_LOG(logDEBUG) << "<< Controller::control(.)";
}
// ===========================  END CONTROLLER  ===============================
// ============================================================================
