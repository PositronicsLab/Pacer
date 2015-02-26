/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 *
 * This code shows how to control Pacer to get a robot to walk in a Figure-8
 * pattern.
 ****************************************************************************/
#include <Pacer/utilities.h>
#include <Pacer/Log.h>
#include <Pacer/Visualizable.h>

extern std::vector<boost::shared_ptr<Pacer::Visualizable> > visualize;

typedef std::pair<double,double> Point;
void controller(double time, const Ravelin::VectorNd& q, const Ravelin::VectorNd& qd,
                Ravelin::VectorNd& command,
                boost::shared_ptr<Ravelin::Pose3d>& gait_pose,
                Ravelin::VectorNd& params)
{

  gait_pose = boost::shared_ptr<Ravelin::Pose3d>(new Ravelin::Pose3d(Ravelin::AAngled(1,1,0,M_PI_8/2.0),Ravelin::Origin3d(0,0,0.01),Moby::GLOBAL));

  const unsigned X=0,Y=1,Z=2,THETA=5;

    // n is the number of independent coordinates in the robot
    int n = qd.rows();

    // nq is the number of joints in the robot
    int nq = n - 6;

    // World frame
    boost::shared_ptr<Ravelin::Pose3d>
        environment_frame(new Ravelin::Pose3d());
    visualize.push_back(Pacer::VisualizablePtr( new Pacer::Pose(*environment_frame.get())));

    // com is the current position of the center-of-mass of the robot's base
    Ravelin::Vector3d com(q[nq+X],q[nq+Y],q[nq+Z],environment_frame);

    // quat is the current orientation of the robot's base
    Ravelin::Quatd quat(q[nq+3],q[nq+4],q[nq+5],q[nq+6]);

    // make a rigid body pose out of com and quat
    Ravelin::Pose3d pose(quat,com.data(),environment_frame);
    visualize.push_back(Pacer::VisualizablePtr( new Pacer::Pose(pose)));

    // get the base orientation as roll-pitch-yaw
    Ravelin::Vector3d rpy;
    quat.to_rpy(rpy[0],rpy[1],rpy[2]);

    // Collocate frame with robot, preserving only the yaw of robot.
    boost::shared_ptr<Ravelin::Pose3d>
        base_horizontal_frame(new Ravelin::Pose3d(Ravelin::AAngled(0,0,1,rpy[2]),com.data(),environment_frame));

    OUT_LOG(logDEBUG1) << "x = " << com ;
    OUT_LOG(logDEBUG1) << "quat = " << quat ;
    OUT_LOG(logDEBUG1) << "rpy = " << rpy ;
    OUT_LOG(logDEBUG1) << "pose = " << pose ;


    /////////////////////
    /// SET VARIABLES ///

    // Command robot to walk in a direction
    command.set_zero(6);

    double max_turn_speed = 1.0; // rad/sec
    double max_forward_speed = 0.05;
    double max_strafe_speed = 0.025;

    // if FALSE, drive like a car (x and theta)
    // Q: if TRUE, ???
    bool HOLONOMIC = false;

    /////////////////////////////////////
    /// Assign WAYPOINTS in the plane ///
    std::vector<Point> waypoints;

    // Q: Sam, is it possible to assign the waypoints only once? (the first
    // time that the controller is called?)

    // points on a figure eight
    for (double t=0.01; t<1.0; t+=0.05) {
        waypoints.push_back(Point(sin(M_PI*2.0 * t),-sin(M_PI*2.0 * t * 2.0)*0.25));
    }

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
      OUT_LOG(logDEBUG1) << "this waypoint: " << next_waypoint ;
      OUT_LOG(logDEBUG1) << "robot position: " << com ;

      waypoint_index = (waypoint_index+1)% num_waypoints;

      next_waypoint = Ravelin::Vector3d(waypoints[waypoint_index].first,waypoints[waypoint_index].second,0,environment_frame);
      }

      OUT_LOG(logDEBUG1) << "num_wps = " << num_waypoints;
      OUT_LOG(logDEBUG1) << "distance_to_wp = " << distance_to_wp;
      OUT_LOG(logDEBUG1) << "waypoint_index = " << waypoint_index;
      visualize.push_back(Pacer::VisualizablePtr(new Pacer::Ray(next_waypoint,com,Ravelin::Vector3d(1,0.5,0))));
      OUT_LOG(logDEBUG1) << "next_wp" << next_waypoint ;

      for(int i=0;i<num_waypoints;i++){
          Ravelin::Vector3d wp(waypoints[i].first,waypoints[i].second,0,environment_frame);
          OUT_LOG(logDEBUG1) << "\twp" << wp ;
      visualize.push_back(Pacer::VisualizablePtr( new Pacer::Point(wp,Ravelin::Vector3d(1,0.5,0),1.0)));
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

    // Find difference
    // Q: difference betwen what?
    double angle_to_goal = atan2(goto_direction[Y],goto_direction[X]);

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
}

