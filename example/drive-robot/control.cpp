/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Pacer/utilities.h>
#include <Pacer/Log.h>
#include <Pacer/Log.h>

void controller(double time, const Ravelin::VectorNd& q, const Ravelin::VectorNd& qd, Ravelin::VectorNd& command)
{
    OUT_LOG(logDEBUG1) << ">> ENTERED USER CONTROLLER" ;

    int n = q.rows();
    int nq = n - 7;

    // World frame
    boost::shared_ptr<Ravelin::Pose3d>
        environment_frame(new Ravelin::Pose3d());

    Ravelin::Vector3d x(q[nq],q[nq+1],q[nq+2],environment_frame);
    Ravelin::Quatd quat(q[nq+3],q[nq+4],q[nq+5],q[nq+6]);
    Ravelin::Vector3d rpy;
    quat.to_rpy(rpy[0],rpy[1],rpy[2]);

    // Frame colocated with robot, preserves only yaw of robot.
    boost::shared_ptr<Ravelin::Pose3d>
        base_horizontal_frame(new Ravelin::Pose3d(Ravelin::AAngled(0,0,1,rpy[2]),x.data(),environment_frame));

    OUT_LOG(logDEBUG1) << "x = " << x ;
    OUT_LOG(logDEBUG1) << "quat = " << quat ;
    OUT_LOG(logDEBUG1) << "rpy = " << rpy ;



    // Command robot to walk in a direction
    command.set_zero(6);
    command[0] = 0.05;
    // ================= EXAMPLES ================

    double max_turn_speed = 1.0; // rad/sec
    double max_forward_speed = 0.05;
    double max_strafe_speed = 0.025;
//#define WALK_ON_LINE
//#define WAYPOINTS
//#define WALK_TO_POINT

#ifdef WALK_ON_LINE
    //for now we'll walk at a constant 5 cm/s speed
    command[0] = 0.05;

    // We want to get the robot walking WEST (+y), at x=0.5
    double d_theta = 0;

    double position_err = x[1] - 0.5;
    double heading_err = Utilities::WrapPosNegPI(M_PI_2 - rpy[2]);

    // if NORTH of line and facing WEST
    if( (position_err > 0) && (fabs(heading_err) < M_PI ) ){

    }
    // if NORTH of line facing EAST
    else if( (position_err > 0) && (fabs(heading_err) > M_PI ) ){

    }
    // if SOUTH of line facing WEST
    else if( (position_err < 0) && (fabs(heading_err) < M_PI ) ){

    }
    // if SOUTH of line facing EAST
    else if( (position_err < 0) && (fabs(heading_err) > M_PI ) ){

    }



    // Limit turning rate to 1 rad/sec
    if(fabs(d_theta) > max_turn_speed)
    d_theta = Utility::sign(d_theta)*max_turn_speed

    command[5] = d_theta;
#endif

    std::vector<double> goto_point;
    std::vector<double> patrol_points;

#if defined(WAYPOINTS) || defined(WALK_TO_POINT)

#ifdef WAYPOINTS
    double N_WAYPOINTS = 4;

    // we'll just make it walk to N_WAYPOINTS points on a unit circle
    //for (double i=0; i<N_WAYPOINTS; i+=1.0) {
    //    // X coord of ith waypoint
    //    patrol_points.push_back(cos(2*M_PI * i / N_WAYPOINTS));  // NOTE: SET THIS
    //    // Y coord of ith waypoint
    //    patrol_points.push_back(sin(2*M_PI * i / N_WAYPOINTS));  // NOTE: SET THIS
    //}

    // Or walk in a bowtie pattern
    patrol_points.push_back(0);
    patrol_points.push_back(1);

    patrol_points.push_back(1);
    patrol_points.push_back(0);

    patrol_points.push_back(-1);
    patrol_points.push_back(0);

    patrol_points.push_back(0);
    patrol_points.push_back(-1);
    /// HANDLE WAYPOINTS
    assert(patrol_points.size() >= 4);
    int num_waypoints = patrol_points.size()/2;
    static int patrol_index = 0;
    static Ravelin::Vector3d
    next_waypoint(patrol_points[patrol_index*2],patrol_points[patrol_index*2+1],0,environment_frame);

    double distance_to_wp = (Ravelin::Origin3d(next_waypoint.data()) - Ravelin::Origin3d(x.data())).norm();

    if( distance_to_wp < 0.025){
    OUT_LOG(logDEBUG1) << "waypoint reached, incrementing waypoint.";
    OUTLOG(next_waypoint,"this_wp",logDEBUG1);
    OUTLOG(x,"robot_pos",logDEBUG1);

    patrol_index = (patrol_index+1) % num_waypoints;

    next_waypoint = Ravelin::Vector3d(patrol_points[patrol_index*2],patrol_points[patrol_index*2+1],x[2],environment_frame);
    }

    OUT_LOG(logDEBUG1) << "num_wps = " << num_waypoints;
    OUT_LOG(logDEBUG1) << "distance_to_wp = " << distance_to_wp;
    OUT_LOG(logDEBUG1) << "patrol_index = " << patrol_index;
    Robot::visualize.push_back(Ray(next_waypoint,data->center_of_mass_x,Ravelin::Vector3d(1,0.5,0)));
    OUTLOG(next_waypoint,"next_wp",logDEBUG1);

    for(int i=0;i<num_waypoints;i++){
        Ravelin::Vector3d wp(patrol_points[i*2],patrol_points[i*2+1],x[2],environment_frame);
        OUTLOG(wp,"wp",logDEBUG1);
        Robot::visualize.push_back(Point(wp,Ravelin::Vector3d(1,0.5,0),1.0);
    }
    goto_point.resize(2);
    goto_point[0] = next_waypoint[0];
    goto_point[1] = next_waypoint[1];
#endif

    // Use dx and dtheta to reach goal (no strafing)
    bool HOLONOMIC = false;                         // NOTE: SET THIS
#   ifdef WALK_TO_POINT
        // go to slow moving point on a circle

        // point performs a revolution every 10 seconds
        goto_point.resize(2);
        goto_point[0] = cos(2.0*M_PI*time* 0.1) * 0.25;
        goto_point[1] = sin(2.0*M_PI*time* 0.1) * 0.25;
        OUTLOG(goto_point,"goto_point",logDEBUG1);
#   endif
    assert(goto_point.size() == 2);

    Ravelin::Vector3d goto_direction =
    Ravelin::Vector3d(goto_point[0],goto_point[1],0,environment_frame)
    - Ravelin::Vector3d(x[0],x[1],0,environment_frame);
    goto_direction = Ravelin::Pose3d::transform_vector(base_horizontal_frame,goto_direction);
    goto_direction.normalize();

    double angle_to_goal = atan2(goto_direction[1],goto_direction[0]);
    if(fabs(angle_to_goal) < M_PI_8){
    if(HOLONOMIC){
        command[1] = goto_direction[1]*max_forward_speed;
        // goal-centric coords
        command[0] =-goto_direction[1]*max_strafe_speed;
        command[2] = goto_direction[0]*max_strafe_speed;
    }
    command[0] = goto_direction[0]*max_forward_speed;
    command[5] = angle_to_goal;
    } else {
    command[5] = Utility::sign(angle_to_goal)*1.5;
    if(!HOLONOMIC){
        command[0] = 0;
        command[1] = 0;
    } else {
        command[0] = goto_direction[0]*max_forward_speed;
        command[1] = goto_direction[1]*max_forward_speed;
        // goal-centric coords
        command[0] =-goto_direction[1]*max_strafe_speed;
        command[2] = goto_direction[0]*max_strafe_speed;
    }
    }
#endif

    OUT_LOG(logDEBUG1) << "command = " << command ;

    OUT_LOG(logDEBUG1) << "<< EXIT USER CONTROLLER" ;
}
