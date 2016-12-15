/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/controller.h>
#include <Pacer/utilities.h>
#include "plugin.h"
using namespace Ravelin;

std::map<std::string,Origin3d>
x_goal, xd_goal, xdd_goal;

Origin3d x;

void loop(){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  
  static double last_time = t;
  double dt = t - last_time;
  last_time = t;
  
  boost::shared_ptr<Ravelin::Pose3d> base_frame;
  base_frame = boost::shared_ptr<Ravelin::Pose3d>( new Ravelin::Pose3d(
                                                                       ctrl->get_data<Ravelin::Pose3d>("base_link_frame")));
  
  const Origin3d X_STEP(dt,0,0),Y_STEP(0,dt,0),Z_STEP(0,0,dt);
  std::vector<Origin3d> STEPS;
  STEPS.push_back(X_STEP);
  STEPS.push_back(Y_STEP);
  STEPS.push_back(Z_STEP);
  
  double speed = ctrl->get_data< double >(plugin_namespace+".speed");

  double TOL = speed * dt * 10;
  
  std::vector<double> goal_vec = ctrl->get_data< std::vector<double> >(plugin_namespace+".goal");
  const Origin3d GOAL(&goal_vec[0]);

  static std::string plan_type;
  if (plan_type.size() == 0) ctrl->get_data<std::string>(plugin_namespace+".plan",plan_type);
  
  Origin3d displacement = x - GOAL;
  std::cout << "displacement: " << displacement << std::endl;

  static int STAGE = -1;

  if (plan_type.compare("direct") == 0) {
    Origin3d dist = displacement;
    if (dist.norm() < 0.01) {
      std::cout << "Done moving, start grasping" << std::endl;
      plan_type = "indirect";
      STAGE = 3;
    } else {
      dist.normalize();
      dist *= dt*speed;
      x = x - dist;
    }
  }
  
  if (plan_type.compare("indirect") == 0){

    switch (STAGE) {
      case -1: 
        x =  x - X_STEP * speed;
        if( fabs(displacement[0]) > 0.15 ) STAGE++;
        break;
      case 0: // move to X
      case 1: // move to Y
      case 2: // move to Z
        x =  x - Utility::sign(displacement[2-STAGE]) * STEPS[2-STAGE] * speed;
        if( fabs(displacement[2-STAGE]) < TOL ) STAGE++;
        break;
      case 3: // grab
      {
        std::cout << "Grabbing" << std::endl;

        static double start_time = t;
        ctrl->set_data<std::string>("pick.command","grab");
        if(t-start_time > 0.5) STAGE++;
      }
        break;
      case 4: // lift
      {
        std::cout << "Lifting" << std::endl;

        static double start_time = t;
        x =  x + Z_STEP * speed;
        if(displacement[2] > 0.1) STAGE++;
      }
        break;
      default:
        
        std::cout << "Done moving" << std::endl;
        break;
    }
  }
  
  {
    Vector3d x_global = Pose3d::transform_point(Pacer::GLOBAL,Vector3d(x.data(),base_frame));
    VISUALIZE(POINT(x_global,Vector3d(0,1,0),0.1));
    Vector3d goal_global = Pose3d::transform_point(Pacer::GLOBAL,Vector3d(GOAL.data(),base_frame));
    VISUALIZE(POINT(goal_global,Vector3d(0,1,1),0.1));

    std::cout << "hand_global: " << x_global << std::endl;
  }
  
  std::cout << "hand_Arm: " << x << std::endl;

  // End Effectors
  x_goal["HAND"] = x;
//  add grip-center offset
  boost::shared_ptr<Ravelin::Pose3d> hand_frame;
  hand_frame = boost::shared_ptr<Ravelin::Pose3d>( new Ravelin::Pose3d(
                                                                       ctrl->get_data<Ravelin::Pose3d>("HAND.frame")));
  std::vector<double> grasp_point = ctrl->get_data< std::vector<double> >(plugin_namespace+".grasp-point");
  x_goal["HAND"] += Origin3d(Pose3d::transform_vector(base_frame,-Vector3d(grasp_point[0],grasp_point[1],grasp_point[2],hand_frame)).data());
  
  ctrl->set_end_effector_value(Pacer::Controller::position_goal,x_goal);
  std::cout << "true: " << x_goal["HAND"] << std::endl;
}

void setup(){
  boost::shared_ptr<Pacer::Controller> ctrl(ctrl_weak_ptr);
  
  //  add grip-center offset
  boost::shared_ptr<Ravelin::Pose3d> base_frame;
  base_frame = boost::shared_ptr<Ravelin::Pose3d>( new Ravelin::Pose3d(
                                                                       ctrl->get_data<Ravelin::Pose3d>("base_link_frame")));
  
  boost::shared_ptr<Ravelin::Pose3d> hand_frame;
  hand_frame = boost::shared_ptr<Ravelin::Pose3d>( new Ravelin::Pose3d(
                                                                       ctrl->get_data<Ravelin::Pose3d>("HAND.frame")));
  std::vector<double> grasp_point = ctrl->get_data< std::vector<double> >(plugin_namespace+".grasp-point");
  x = Origin3d(Pose3d::transform_point(base_frame,Vector3d(grasp_point[0],grasp_point[1],grasp_point[2],hand_frame)).data());
  
  x_goal = ctrl->get_end_effector_value(Pacer::Controller::position);
  for (std::map<std::string,Origin3d>::iterator it = x_goal.begin();  it != x_goal.end(); it++) {
    xd_goal[it->first] = Origin3d(0,0,0);
    xdd_goal[it->first] = Origin3d(0,0,0);
  }
  
  ctrl->set_data<std::string>("pick.command","release");
}
