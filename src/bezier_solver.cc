#include <project_common.h>

// Calculate the next bezier point.
Ravelin::Vector3d& drawBezier(const Ravelin::Vector3d& A, const Ravelin::Vector3d& B, const Ravelin::Vector3d& C, const Ravelin::Vector3d& D, double t,Ravelin::Vector3d& P) {

    P[0] = pow((1 - t), 3) * A[0] + 3 * t * pow((1 -t), 2) * B[0] + 3 * (1-t) * pow(t, 2)* C[0] + pow (t, 3)* D[0];
    P[1] = pow((1 - t), 3) * A[1] + 3 * t * pow((1 -t), 2) * B[1] + 3 * (1-t) * pow(t, 2)* C[1] + pow (t, 3)* D[1];
    P[2] = pow((1 - t), 3) * A[2] + 3 * t * pow((1 -t), 2) * B[2] + 3 * (1-t) * pow(t, 2)* C[2] + pow (t, 3)* D[2];

    return P;
}

std::vector<Ravelin::Vector3d>& stepTrajectory(const std::vector<Ravelin::Vector3d>& control_points, int num_segments, std::vector<Ravelin::Vector3d>& trajectory){
//    int num_segments = trajectory.size();
    for(int t = 0;t < num_segments; t++) {
      drawBezier(control_points[0], control_points[1], control_points[2], control_points[3], (double)t/(double)num_segments ,trajectory[t]);
    }
    return trajectory;
}
