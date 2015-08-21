//double Robot::calc_energy(const Ravelin::VectorNd& v, const Ravelin::MatrixNd& M) const {
//    // Potential Energy: this calculation assumes that ground is always at zero
//    Ravelin::VectorNd workv_;
//    double PE = 0;
//    std::map<std::string, boost::shared_ptr<Ravelin::RigidBodyd> >::const_iterator it;
//    for(it=_id_link_map.begin();it!=_id_link_map.end();it++){
//        const boost::shared_ptr<Ravelin::RigidBodyd>  link = (*it).second;
//        double m = link->get_mass();
//        Ravelin::Pose3d
//        link_com(link->get_inertial_pose());
//        link_com.update_relative_pose(Pacer::GLOBAL);
//        PE += link_com.x[2] * m * grav;
//    }
//    M.mult(v, workv_);
//    double KE = workv_.dot(v)*0.5;
//    //#ifndef NDEBUG
//    OUT_LOG(logDEBUG) << "KE = " << KE << ", PE = " << PE;
//    OUT_LOG(logDEBUG) << "Total Energy = " << (KE + PE);
//    //#endif
//    return KE;
//    // Kinetic Energy
//}


#include <Pacer/controller.h>
std::string plugin_namespace;

void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t){
  Ravelin::Vector3d center_of_mass_x;
  center_of_mass_x.pose = Pacer::GLOBAL;
  double total_mass=0;
  
  const std::map<std::string, boost::shared_ptr<Ravelin::RigidBodyd> >& _id_link_map = ctrl->get_links();
  std::map<std::string, boost::shared_ptr<Ravelin::RigidBodyd> >::const_iterator it;
  for(it=_id_link_map.begin();it!=_id_link_map.end();it++){
    double m = (*it).second->get_mass();
    total_mass += m;
    center_of_mass_x += (Ravelin::Pose3d::transform_point(Pacer::GLOBAL,Ravelin::Vector3d(0,0,0,(*it).second->get_inertial_pose())) *= m);
  }
  ctrl->set_data<double>("mass",total_mass);
  center_of_mass_x /= total_mass;
  
  boost::shared_ptr<Ravelin::RigidBodyd>  _root_link = ctrl->get_root_link();
  boost::shared_ptr<Ravelin::Pose3d> base_com_w(new Ravelin::Pose3d(Pacer::GLOBAL));
  base_com_w->x = Ravelin::Origin3d(center_of_mass_x);
  boost::shared_ptr<const Ravelin::Pose3d> base_com_w_const = boost::const_pointer_cast<const Ravelin::Pose3d>(base_com_w);
//  Ravelin::SVector6d com_vel = Ravelin::Pose3d::transform(base_com_w_const, _root_link->get_velocity());
  
//  Ravelin::Vector3d center_of_mass_xd = com_vel.get_upper();
  
//  Ravelin::SAcceld com_acc = Ravelin::Pose3d::transform(base_com_w_const, _root_link->get_accel(), _root_link->get_velocity());
//  Ravelin::Vector3d center_of_mass_xdd = com_acc.get_linear();
  
//  center_of_mass_xd.pose = center_of_mass_xdd.pose = Pacer::GLOBAL;
  
  ctrl->set_data<Ravelin::Vector3d>("center_of_mass.x",center_of_mass_x);
  //  ctrl->set_data<Ravelin::Vector3d>("center_of_mass.xd",center_of_mass_xd);
  //  ctrl->set_data<Ravelin::Vector3d>("center_of_mass.xdd",center_of_mass_xdd);
  
  //  center_of_mass_wd = com_acc.get_angular();
  //  center_of_mass_w = com_vel.get_angular();
  
  // ZMP
  // x(k+1) = A x(k) + B u(k)
  // p(k)   = C x(k)
  
  // From Kajita et al. 2003
  // x(k) = [x(kT),xd(kT),xdd(kT)]'
  // u(k) = u_x(kT)
  // p(k) = p_x(kT)
  // A = [1  T  (T^2)/2;
  //      0  1  T      ;
  //      0  0  1      ]
  // B = [(T^3)/2 ;
  //      (T^2)/2 ;
  //          T   ]
  // C = [1 0 -z/g]
  
  // e = p - p_ref
  //
//  Ravelin::Vector3d C(1,0,-center_of_mass_x[2]/grav,GLOBAL);
//  Ravelin::Vector3d zero_moment_point =
//  Ravelin::Vector3d(C.dot(Ravelin::Vector3d(center_of_mass_x[0],center_of_mass_xd[0],center_of_mass_xdd[0],GLOBAL)),
//                    C.dot(Ravelin::Vector3d(center_of_mass_x[1],center_of_mass_xd[1],center_of_mass_xdd[1],GLOBAL)),
//                    0,GLOBAL);
//  ctrl->set_data<Ravelin::Vector3d>("zero_moment_point",zero_moment_point);
//  
//  // ZMP and COM
//  Ravelin::Vector3d CoM_2D(center_of_mass_x[0],center_of_mass_x[1],center_of_mass_x[2]-0.10,GLOBAL);
//  Utility::visualize.push_back( Pacer::VisualizablePtr( new Pacer::Ray(CoM_2D,center_of_mass_x,Ravelin::Vector3d(0,0,1))));
//  //  Utility::visualize.push_back( Pacer::VisualizablePtr( new Pacer::Ray(CoM_2D + center_of_mass_xd*0.1,CoM_2D,Ravelin::Vector3d(0.5,0,1)));
//  //  Utility::visualize.push_back( Pacer::VisualizablePtr( new Pacer::Ray(CoM_2D + center_of_mass_xd*0.1 + center_of_mass_xdd*0.01,CoM_2D + center_of_mass_xd*0.1,Ravelin::Vector3d(1,0,0)));
//  Utility::visualize.push_back( Pacer::VisualizablePtr( new Pacer::Ray(CoM_2D+Ravelin::Vector3d(zero_moment_point[0],zero_moment_point[1],0,GLOBAL)*0.1,CoM_2D,Ravelin::Vector3d(0,1,0))));
}


/** This is a quick way to register your plugin function of the form:
 * void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t)
 */
#include "register-plugin"
