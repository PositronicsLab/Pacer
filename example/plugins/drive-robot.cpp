/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 *
 * This code shows how to control Pacer to get a robot to walk in a Figure-8
 * pattern.
 ****************************************************************************/
#include <Pacer/controller.h>
#include <Pacer/utilities.h>

std::string plugin_namespace;

typedef std::pair<double,double> Point;

void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double time){
  
  const unsigned X=0,Y=1,Z=2,THETA=5;
    // World frame
    boost::shared_ptr<Ravelin::Pose3d>
        environment_frame(new Ravelin::Pose3d());
    Utility::visualize.push_back(Pacer::VisualizablePtr( new Pacer::Pose(*environment_frame.get())));

    Ravelin::Vector3d com = ctrl->get_data<Ravelin::Vector3d>("center_of_mass.x");

      boost::shared_ptr<Ravelin::Pose3d>
        base_horizontal_frame(new Ravelin::Pose3d(ctrl->get_data<Ravelin::Pose3d>("base_horizontal_frame")));

    OUT_LOG(logDEBUG1) << "x = " << com ;

    /////////////////////
    /// SET VARIABLES ///

    // Command robot to walk in a direction
    Ravelin::VectorNd command;
    command.set_zero(6);

    double max_turn_speed = 1.0; // rad/sec
    double max_forward_speed = 0.05;
    double max_strafe_speed = 0.025;

    // if FALSE, drive like a car (x and theta)
    // Q: if TRUE, turn toward waypoint while stepping in direction of waypoint
    bool HOLONOMIC = false;

    /////////////////////////////////////
    /// Assign WAYPOINTS in the plane ///
    static std::vector<Point> waypoints;

    static std::vector<double>
      &waypoints_vec = Utility::get_variable<std::vector<double> >(plugin_namespace+"waypoints");

    if(waypoints.empty())
      for(int i=0;i<waypoints_vec.size();i+=2)
        waypoints.push_back(Point(waypoints_vec[i],waypoints_vec[i+1]));
        
    /////////////////////////////
    /// CHOOSE NEXT WAYPOINT  ///
    Ravelin::Vector3d goto_point;

    int num_waypoints = waypoints.size();
    if(num_waypoints > 1){
      static int waypoint_index = 0;
      static Ravelin::Vector3d
      next_waypoint(waypoints[waypoint_index].first,waypoints[waypoint_index].second,0,environment_frame);

      double distance_to_wp = (Ravelin::Origin3d(next_waypoint.data()) - Ravelin::Origin3d(com[X],com[Y],0)).norm();

      if( distance_to_wp < 0.025){
      OUT_LOG(logDEBUG1) << "waypoint reached, incrementing waypoint.";
      OUTLOG(next_waypoint,"this_wp",logDEBUG1);
      OUT_LOG(logDEBUG1) << "this waypoint: " << next_waypoint;
      OUT_LOG(logDEBUG1) << "robot position: " << com;

      waypoint_index = (waypoint_index+1)% num_waypoints;

      next_waypoint = Ravelin::Vector3d(waypoints[waypoint_index].first,waypoints[waypoint_index].second,0,environment_frame);
      }

      OUT_LOG(logDEBUG1) << "num_wps = " << num_waypoints;
      OUT_LOG(logDEBUG1) << "distance_to_wp = " << distance_to_wp;
      OUT_LOG(logDEBUG1) << "waypoint_index = " << waypoint_index;
      Utility::visualize.push_back(Pacer::VisualizablePtr(new Pacer::Ray(next_waypoint,com,Ravelin::Vector3d(1,0.5,0))));
      OUT_LOG(logDEBUG1) << "next_wp" << next_waypoint;

      for(int i=0;i<num_waypoints;i++){
          Ravelin::Vector3d wp(waypoints[i].first,waypoints[i].second,0,environment_frame);
          OUT_LOG(logDEBUG1) << "\twp" << wp;
      Utility::visualize.push_back(Pacer::VisualizablePtr( new Pacer::Point(wp,Ravelin::Vector3d(1,0.5,0),1.0)));
      }

      goto_point = next_waypoint;
    } else {
      goto_point = Ravelin::Vector3d(waypoints[0].first,waypoints[0].second,0,environment_frame);
    }


    ///////////////////////
    /// GO TO WAYPOINT  ///

    // Find direction to waypoint from robot position
    Ravelin::Vector3d goto_direction =
      Ravelin::Vector3d(goto_point[X],goto_point[Y],0,environment_frame)
      - Ravelin::Vector3d(com[X],com[Y],0,environment_frame);
    goto_direction = Ravelin::Pose3d::transform_vector(base_horizontal_frame,goto_direction);
    goto_direction.normalize();

    double angle_to_goal = atan2(goto_direction[Y],goto_direction[X]);
    OUT_LOG(logDEBUG1) << "angle_to_goal = " << angle_to_goal;

    // If robot is facing toward goal already, walk in that direction
    if(fabs(angle_to_goal) < M_PI_8){
      if(HOLONOMIC){
        command[Y] = goto_direction[Y]*max_strafe_speed;
      }
      command[X] = goto_direction[X]*max_forward_speed;
      command[THETA] = angle_to_goal;
    } else {
      command[THETA] = Utility::sign(angle_to_goal)*max_turn_speed;
      if(HOLONOMIC){
        command[X] = goto_direction[X]*max_forward_speed;
        command[Y] = goto_direction[Y]*max_strafe_speed;
      } else {
        command[X] = 0;
        command[Y] = 0;
      }
    }

    Ravelin::Origin3d command_SE2(command[X],command[Y],command[THETA]);
    ctrl->set_data<Ravelin::Origin3d>("SE2_command",command_SE2);
}

/** This is a quick way to register your plugin function of the form:
  * void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t)
  */
#include "register-plugin"