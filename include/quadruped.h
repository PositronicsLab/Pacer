#ifndef CONTROL_H
#define CONTROL_H

#include <robot.h>

class Quadruped : public Robot{
  public:
    Quadruped();
    Quadruped(Moby::RCArticulatedBodyPtr abrobot){
      abrobot_ = abrobot;
      init();
    }
    Ravelin::VectorNd& control(double dt,
                               const Ravelin::VectorNd& q,
                               const Ravelin::VectorNd& qd,
                               Ravelin::VectorNd& q_des,
                               Ravelin::VectorNd& qd_des,
                               Ravelin::VectorNd& u);


    double friction_estimation(const Ravelin::VectorNd& v, const Ravelin::VectorNd& fext,
                               double dt, const Ravelin::MatrixNd& N,
                               const Ravelin::MatrixNd& ST, const Ravelin::MatrixNd& M,
                               Ravelin::MatrixNd& MU, Ravelin::VectorNd& cf);


    void inverse_dynamics(const Ravelin::VectorNd& v, const Ravelin::VectorNd& qdd, const Ravelin::MatrixNd& M,
                          const  Ravelin::MatrixNd& N, const Ravelin::MatrixNd& ST, const Ravelin::VectorNd& fext,
                          double h, const Ravelin::MatrixNd& MU, Ravelin::VectorNd& uff);

    void sinusoidal_trot(Ravelin::VectorNd& q_des,Ravelin::VectorNd& qd_des,Ravelin::VectorNd& qdd,double dt);
    void cpg_trot(Ravelin::VectorNd& q_des,Ravelin::VectorNd& qd_des,Ravelin::VectorNd& qdd,double dt);

    void contact_jacobian_null_stabilizer(const Ravelin::MatrixNd& R, Ravelin::VectorNd& uff);

    void calc_contact_jacobians(Ravelin::MatrixNd& N,Ravelin::MatrixNd& ST,Ravelin::MatrixNd& D,Ravelin::MatrixNd& R);

    std::vector<Ravelin::Vector3d>& foot_oscilator(
      const std::vector<Ravelin::Vector3d>& x0,  const std::vector<Ravelin::Vector3d>& x, const Ravelin::MatrixNd& C,
      double Ls,const Ravelin::VectorNd& Hs,double Df,double Vf,double bp,std::vector<Ravelin::Vector3d>& xd);

    // 4 Control Point Bezier curve EEF calculation
    std::vector<Ravelin::Vector3d>& stepTrajectory(
        const std::vector<Ravelin::Vector3d>& control_points,
        int num_segments, std::vector<Ravelin::Vector3d>& trajectory);

    /// 4 foot (body-fixed) state-space traj to joint-space traj
    std::vector<std::vector<Ravelin::Vector3d> > &trajectoryIK(
            const std::vector<std::vector<Ravelin::Vector3d> >& feet_positions,
            std::vector<std::vector<Ravelin::Vector3d> >& joint_positions);

    std::vector<Ravelin::Vector3d>& feetIK(
            const std::vector<Ravelin::Vector3d>& feet_positions,
            std::vector<Ravelin::Vector3d>& joint_positions);
    void init();

    std::map<std::string, Ravelin::Vector3d> eef_origins_;

  private:
    // Useful Stored Data
    boost::shared_ptr<Ravelin::Pose3d>   base_horizonal_frame;
    boost::shared_ptr<Ravelin::Pose3d>   base_frame;
    unsigned          NC;
    Ravelin::VectorNd uff, ufb;
    Ravelin::VectorNd qdd;
    Ravelin::MatrixNd N,D,M,ST,R;
    Ravelin::VectorNd fext;
    Ravelin::VectorNd vel, gc, acc;
};

#endif // CONTROL_H
