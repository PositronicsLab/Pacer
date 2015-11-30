/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/controller.h>
#include <Pacer/utilities.h>
#include "plugin.h"

using namespace Pacer;
using namespace Ravelin;

static Vector3d workv3_;
static VectorNd q,qd_base;
static VectorNd workv_;

boost::shared_ptr<Pose3d> base_frame,base_horizontal_frame, gait_pose;
std::vector<std::string> foot_names;
std::map<std::string,bool> active_feet;

std::vector<Ravelin::Vector3d>& foot_oscilator(
    const std::vector<Ravelin::Origin3d>& x0,
    const std::vector<Ravelin::Vector3d>& x,
    const Ravelin::MatrixNd& C,
    double Ls,
    const Ravelin::VectorNd& Hs,
    const std::vector<double>& Df,
    double Vf,
    double bp,
    std::vector<Ravelin::Vector3d>& xd){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);

  int NUM_EEFS = xd.size();

  /* Tunable parameters
   * Ls    : length of step
   * Hs    : height of step
   * Df    : step duty factor
   * Vf    : forward velocity
   * bp    : the transition rate between phases
   * bf    : the transition rate between phases
   */
  // a/b/c : affect the convergence rate of the limit cycle
  double a = ctrl->get_data<double>(plugin_namespace+".convergence.a");
  double b = ctrl->get_data<double>(plugin_namespace+".convergence.b");
  double c = ctrl->get_data<double>(plugin_namespace+".convergence.c");
  
//  // B is negative
//  b*=-1.0;

  // Stepping Filter params
  // depth of step phase where touchdown occurs (fraction of Hs)
  double ztd = ctrl->get_data<double>(plugin_namespace+".Ztd"); //0
  // speed at which behavior changes (arbitrary scale)
  double bf = ctrl->get_data<double>(plugin_namespace+".Bf"); //1000

  std::vector<Ravelin::Vector3d> xb(NUM_EEFS);
  for(int i=0;i<NUM_EEFS;i++)
    xb[i] = x[i] - x0[i];

  for(int i=0;i<NUM_EEFS;i++){
    double Cp = 0;
    // Eqn: 3
    for(int j=0;j<NUM_EEFS;j++)
      Cp += C(i,j)*Hs[i]*xb[j][2]/Hs[j];

    Ravelin::Vector3d& xbar = xb[i];

    // Eqn 4, 5, 6
    double Sp1 = 1.0/(exp(-bp*xbar[2]) + 1.0);
    double Sp2 = 1.0/(exp( bp*xbar[2]) + 1.0);
    double ws  = M_PI * (Vf/Ls) * ((Df[i]*Sp1)/(1.0-Df[i]) + Sp2);

    // realtively thin gait [shoulder width]
    double dyc = ctrl->get_data<double>(plugin_namespace+".Dyc"); //0;

    // \dot{x}
    // Eqn: 1, 2, 3
    double oscil = 1.0 - (4*xbar[0]*xbar[0])/(Ls*Ls) - (xbar[2]*xbar[2])/(Hs[i]*Hs[i]) ;
    xd[i][0] = a*oscil*xbar[0] + (ws*Ls*xbar[2])/(2*Hs[i]);
    xd[i][1] = b*(xbar[1] + dyc);
    xd[i][2] = c*oscil*xbar[2] - (ws*2*Hs[i]*xbar[0])/Ls + Cp;

    // Stepping Terrain Filter
    // Eqns: 7, 8, 9
//    if(center_of_contact.active){
//      OUTLOG(center_of_contact.normal[0],"normal",logDEBUG1);
//      OUTLOG(center_of_contact.point[0],"point",logDEBUG1);
//      OUTLOG(Ravelin::Pose3d::transform_point(environment_frame,x[i]),"foot_pos",logDEBUG1);
//      double dist_plane = Utility::distance_from_plane(center_of_contact.normal[0],
//                                              center_of_contact.point[0],Ravelin::Pose3d::transform_point(environment_frame,x[i]));
//      std::cout << eefs_[i].id << " " << dist_plane << std::endl;
//      double Sf1 = 1.0/(exp(-bf*(dist_plane)) + 1.0);
//      double Sf2 = 1.0/(exp( bf*(dist_plane)) + 1.0);
//      double Sf1 = 1.0/(exp(-bf*(xbar[2] - ground_plane(x[i]))) + 1.0);
//      double Sf2 = 1.0/(exp( bf*(xbar[2] - ground_plane(x[i]))) + 1.0);
    double Sf1 = 1.0/(exp(-bf*(xbar[2] - ztd)) + 1.0);
    double Sf2 = 1.0/(exp( bf*(xbar[2] - ztd)) + 1.0);
    
      ctrl->set_data<bool>(foot_names[i]+".stance",(xbar[2]<=ztd));

//      xd[i] = (xd[i])*Sf1 - Ravelin::Vector3d(Vf,0,0,base_frame)*Sf2;
//    }
  }

  return xd;
}

void cpg_trot(
    const Ravelin::VectorNd& command,
    const std::vector<double>& touchdown,
    const std::vector<double>& duty_factor,
    double gait_duration,
    double step_height,
    // MODEL
    const std::vector<Ravelin::Origin3d>& foot_origin,
    double t,
    // OUT
    std::vector<Ravelin::Vector3d>& foot_pos,
    std::vector<Ravelin::Vector3d>& foot_vel,
    std::vector<Ravelin::Vector3d>& foot_acc){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);

  static double last_t = 0;
  int NUM_EEFS = foot_origin.size();
  static std::vector<Ravelin::Vector3d> last_foot_vel(NUM_EEFS);
  Ravelin::VectorNd Hs(NUM_EEFS);
  Ravelin::MatrixNd C(NUM_EEFS,NUM_EEFS);

  // SRZ: this is the coupling matrix C
  // see: Pattern generators with sensory feedback for the control of quadruped locomotion
  C.set_zero();
  unsigned gait_pattern = 0;
  switch(gait_pattern){
    case 0: // Trotting gait
                   C(0,1) = -1; C(0,2) = -1; C(0,3) =  1;
      C(1,0) = -1;              C(1,2) =  1; C(1,3) = -1;
      C(2,0) = -1; C(2,1) =  1;              C(2,3) = -1;
      C(3,0) =  1; C(3,1) = -1; C(3,2) = -1;
      break;
    case 1: // Pacing gait
                   C(0,1) = -1; C(0,2) =  1; C(0,3) = -1;
      C(1,0) = -1;              C(1,2) = -1; C(1,3) =  1;
      C(2,0) =  1; C(2,1) = -1;              C(2,3) = -1;
      C(3,0) = -1; C(3,1) =  1; C(3,2) = -1;
    break;
    case 2: // Bounding gait
                   C(0,1) =  1; C(0,2) = -1; C(0,3) = -1;
      C(1,0) =  1;              C(1,2) = -1; C(1,3) = -1;
      C(2,0) = -1; C(2,1) = -1;              C(2,3) =  1;
      C(3,0) = -1; C(3,1) = -1; C(3,2) =  1;
    break;
    case 3: // Walking gait
                   C(0,1) = -1; C(0,2) =  1; C(0,3) = -1;
      C(1,0) = -1;              C(1,2) = -1; C(1,3) =  1;
      C(2,0) = -1; C(2,1) =  1;              C(2,3) = -1;
      C(3,0) =  1; C(3,1) = -1; C(3,2) = -1;
    break;
    case 4: // squatting
                   C(0,1) =  1; C(0,2) = -1; C(0,3) =  1;
      C(1,0) =  1;              C(1,2) =  1; C(1,3) = -1;
      C(2,0) = -1; C(2,1) =  1;              C(2,3) =  1;
      C(3,0) =  1; C(3,1) = -1; C(3,2) =  1;
    break;
  }

  // Additional parameters for CPG
  double bp = ctrl->get_data<double>(plugin_namespace+".Bp"); //1000


  for(int i=0;i<NUM_EEFS;i++){
    // set height of gait (each foot)
    Hs[i] = step_height;

    // SET EEF ORIGINS TO the ground below that EEF SHOULDER
    last_foot_vel[i].pose = foot_vel[i].pose = base_frame;
  }

  // retrieve oscilator value
  foot_oscilator(foot_origin,foot_pos,C,command[0]/gait_duration,Hs,duty_factor,command[0],bp,foot_vel);

  double dt = t - last_t;

  for(int i=0;i<NUM_EEFS;i++){
    foot_pos[i].pose = foot_vel[i].pose;
    foot_pos[i] = foot_pos[i] + foot_vel[i]*dt;
    foot_acc[i] = (foot_vel[i] - last_foot_vel[i])/dt;
    if(foot_acc[i][2] > 0.0)
      ctrl->set_data<bool>(foot_names[i]+".stance",false);
  }


  last_foot_vel = foot_vel;
  last_t = t;
}

void loop(){
boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  // Find time since last call
  static double last_time = t;
  double dt = t - last_time;
  last_time = t;
  
  /// Moving average for command
  const int queue_size = 10;
  static std::deque<Origin3d> command_queue;
  static Origin3d sum_command = Origin3d(0,0,0);
  
  {
    Origin3d command = Origin3d(0,0,0);
    ctrl->get_data<Origin3d>("SE2_command",command);
    command_queue.push_front(command);
    sum_command += command;
  }
  
  if(command_queue.size() > queue_size){
    sum_command -= command_queue.back();
    command_queue.pop_back();
  }
  
  Origin3d command = sum_command / (double) command_queue.size();
  
  /// Command velocity differential
  //  static Origin3d command = Origin3d(0,0,0);
  //
  //  Origin3d dcommand = Origin3d(0,0,0);
  //  ctrl->get_data<Origin3d>("SE2_command",dcommand);
  //  command *= 0.999;
  //  command += dcommand*dt;
  
  VectorNd go_to;
  go_to.set_zero(6);
  go_to[0] = command[0];
  go_to[1] = command[1];
  go_to[5] = command[2];
  
  static std::vector<double>
  duty_factor = ctrl->get_data<std::vector<double> >(plugin_namespace+".duty-factor"),
  this_gait = ctrl->get_data<std::vector<double> >(plugin_namespace+".gait"),
  input_gait_pose = ctrl->get_data<std::vector<double> >(plugin_namespace+".pose");
  static double gait_time = ctrl->get_data<double>(plugin_namespace+".gait-duration");
  static double step_height = ctrl->get_data<double>(plugin_namespace+".step-height");
  static std::vector<Vector3d> footholds(0);
  foot_names = ctrl->get_data<std::vector<std::string> >(plugin_namespace+".feet");
  
  static double width = ctrl->get_data<double>(plugin_namespace+".width");
  static double length = ctrl->get_data<double>(plugin_namespace+".length");
  static double height = ctrl->get_data<double>(plugin_namespace+".height");
  
  base_frame = boost::shared_ptr<Pose3d>( new Pose3d(
                                                     ctrl->get_data<Pose3d>("base_link_frame")));
  base_horizontal_frame = boost::shared_ptr<Pose3d>( new Pose3d(
                                                                ctrl->get_data<Pose3d>("base_horizontal_frame")));
  
  { /// Adjust Gait
    
    
  }
  
  { /// Display command
    Origin3d dir(Pose3d::transform_vector(Pacer::GLOBAL,Vector3d(command,base_horizontal_frame)).data());
    dir *= 2.0;
    dir[2] = command[2]/10;
    Utility::visualize.push_back( Pacer::VisualizablePtr( new Ray(  (dir+base_horizontal_frame->x).data(),   (base_horizontal_frame->x).data(),   Vector3d(0,1,0),0.5)));
  }
  
  // Set up output vectors for gait planner
  int NUM_FEET = foot_names.size();
  std::vector<Vector3d>
  foot_vel(NUM_FEET),
  foot_pos(NUM_FEET),
  foot_acc(NUM_FEET),
  foot_init(NUM_FEET);
  q = ctrl->get_generalized_value(Pacer::Robot::position);
  qd_base = ctrl->get_base_value(Pacer::Robot::velocity);
  
  //  ctrl->set_model_state(q);
  for(int i=0;i<NUM_FEET;i++){
    foot_init[i] = ctrl->get_data<Vector3d>(foot_names[i]+".init.x");
    foot_pos[i] = foot_init[i];
    ctrl->get_data<Vector3d>(foot_names[i]+".goal.x",foot_pos[i]);
    foot_vel[i] = Vector3d(0,0,0,base_frame);
    ctrl->get_data<Vector3d>(foot_names[i]+".goal.xd",foot_vel[i]);
    foot_acc[i] = Vector3d(0,0,0,base_frame);
  }
  
  
  gait_pose = boost::shared_ptr<Pose3d>(
                new Pose3d(
                      Ravelin::Quatd::rpy(
                        input_gait_pose[3],
                        input_gait_pose[4],
                        input_gait_pose[5]
                      ),
                      Origin3d(
                        input_gait_pose[0] /*+ command[0]/(4.0*gait_time)*/,
                        input_gait_pose[1] /*+ command[1]/(4.0*gait_time)*/,
                        input_gait_pose[2]
                      ),
                      base_frame
                      )
                    );
  
  // Assign foot origins (ideal foot placemenet at rest)
  std::vector<Origin3d> origins;
  if(origins.empty()){
    origins.resize(NUM_FEET);
    for(int i=0;i<NUM_FEET;i++){
      Vector3d origin(
                      Utility::sign<double>(foot_init[i][0])*length/2,
                      Utility::sign<double>(foot_init[i][1])*width/2,
                      -height,base_frame);
      origins[i] = Pose3d::transform_point(gait_pose,origin);
    }
  }
  
  ctrl->set_data<Pose3d>("base_stability_frame",*(gait_pose.get()));
  
  OUTLOG(this_gait,"this_gait",logINFO);
  OUTLOG(duty_factor,"duty_factor",logINFO);
    
  for(int i=0;i<foot_names.size();i++){
    std::vector< boost::shared_ptr< const Pacer::Robot::contact_t> > c;
    ctrl->get_link_contacts(foot_names[i],c);
    if(!c.empty())
      active_feet[foot_names[i]] = true;
    else
      active_feet[foot_names[i]] = false;
  }
  
  cpg_trot(go_to,this_gait,duty_factor,gait_time,step_height,origins,t,foot_pos,foot_vel,foot_acc);
  
  for(int i=0;i<NUM_FEET;i++){
    foot_acc[i].set_zero();
    ctrl->set_data<Vector3d>(foot_names[i]+".goal.x",foot_pos[i]);
    ctrl->set_data<Vector3d>(foot_names[i]+".goal.xd",foot_vel[i]);
    Utility::visualize.push_back( Pacer::VisualizablePtr( new Ray(  Ravelin::Pose3d::transform_point(Pacer::GLOBAL,foot_pos[i]).data(),   (Ravelin::Pose3d::transform_point(Pacer::GLOBAL,foot_pos[i])+Ravelin::Pose3d::transform_vector(Pacer::GLOBAL,foot_vel[i])).data(),   Vector3d(0,1,0),0.5)));

    ctrl->set_data<Vector3d>(foot_names[i]+".goal.xdd",foot_acc[i]);
  }
}
void setup(){
}