void apply_transform(Ravelin::Transform3d& T,const std::set<shared_ptr<Jointd> >& outer_joints){
  BOOST_FOREACH(shared_ptr<Jointd> jp, outer_joints){
    shared_ptr<RigidBodyd> rb = jp->get_outboard_link();
    rb->set_pose(T.transform(*(rb->get_pose().get())));
    //    jp->set_pose(T.transform(*(jp->get_pose().get())));
    //    jp->set_location(
    //                     Ravelin::Vector3d(0,0,0,jp->get_pose()),
    //                     jp->get_inboard_link(),
    //                     jp->get_outboard_link());
    apply_transform(T,rb->get_outer_joints());
  }
}

#include <unistd.h>
#include <boost/program_options.hpp>
namespace po = boost::program_options;

void apply_state_uncertainty(int argc,char* argv[],shared_ptr<RCArticulatedBodyd>& robot){
  logging1 << ">> apply_state_uncertainty" << std::endl;
  
  po::options_description desc("Monte Carlo Method state (applied at simulator start) uncertainty options");
  desc.add_options()
  ("help", "produce help message")
  ("stand", "put robot on ground")
  ("float", "move robot away from ground")
  ("BODY0.x"    ,   po::value<std::vector<double> >()->multitoken(),  "Absolute Position [m OR rad] of Robot base")
  ("BODY0.xd"    ,   po::value<std::vector<double> >()->multitoken(),  "Absolute Velocity [m/s OR rad/s] of Robot base");
  
  //  BOOST_FOREACH(shared_ptr<Jointd> jp, robot->get_joints()){
  //    std::string name,help;
  //    name = std::string(jp->joint_id+".x");
  //    help = std::string("Absolute Position [m OR rad] of Joint: "+jp->joint_id);
  //
  //    desc.add_options()
  //    (name.c_str(), po::value<std::vector<double> >()->multitoken() ,help.c_str());
  //
  //    name = std::string(jp->joint_id+".xd");
  //    help = std::string("Absolute Velocity [m/s OR rad/s] of Joint: "+jp->joint_id);
  //
  //    desc.add_options()
  //    (name.c_str(), po::value<std::vector<double> >()->multitoken() ,help.c_str());
  //  }
  
  po::variables_map vm;
  
  po::parsed_options parsed
  = po::command_line_parser(argc, argv).options(desc).style(po::command_line_style::unix_style ^ po::command_line_style::allow_short).allow_unregistered().run();
  po::store(parsed, vm);
  
  if ( vm.count("help")  )
  {
    logging1 << "Available options: " << std::endl
    << desc << std::endl;
  }
  
  /*
   *  Parameter Application
   */
  if(USE_UNCER)
    if(vm.count("BODY0.x")){
      std::vector<double> position = vm["BODY0.x"].as<std::vector<double> >();
      logging1 << "applying state uncertainty to base (position): "<< position << std::endl;
      
      // Get robot positon
      Ravelin::VectorNd q;
      robot->get_generalized_coordinates_euler(q);
      int N_JOINT_DOFS = q.rows()-7;
      // update base position
      for (int i=0;i<3; i++) {
        q[N_JOINT_DOFS+i] = position[i];
      }
      Ravelin::Quatd quat=Ravelin::Quatd::rpy(position[3],position[4],position[5]);
      for (int i=0;i<4; i++) {
        q[N_JOINT_DOFS+3+i] = quat[i];
      }
      
      // apply changes
      robot->set_generalized_coordinates_euler(q);
      logging1 << "Set coords to : " << q << std::endl;
      
    }
  
  ////   Joints
  //  if(USE_UNCER)
  //  BOOST_FOREACH(shared_ptr<Jointd> jp, robot->get_joints()){
  //    if(vm.count(jp->joint_id+".x")){
  //      std::vector<double> position = vm[jp->joint_id+".x"].as<std::vector<double> >();
  //      for (int i=0; i<jp->num_dof(); i++) {
  //        jp->q[i] = position[i];
  //      }
  //    }
  //  }
  
  // Update robot state
  robot->update_link_poses();
  
  double lowest_point_z = 0;
  
  ///////// Project robot up to standing on plane //////////////
  if(vm.count("stand")){
    double lowest_point_z = 0;
    
    BOOST_FOREACH(shared_ptr<Ravelin::RigidBodyd> rbd, robot->get_links()){
      shared_ptr<Moby::RigidBody> rb = boost::dynamic_pointer_cast<Moby::RigidBody>(rbd);
      if(rb->is_end_effector()){ // radius of spherical link
        boost::shared_ptr<Ravelin::Pose3d> foot_pose(new Ravelin::Pose3d(*(rb->get_pose().get())));
        foot_pose->update_relative_pose(robot->get_base_link()->get_pose());
        
        // get foot geometry
        Moby::CollisionGeometryPtr foot_geometry;
        shared_ptr<Moby::SpherePrimitive> foot_primitive;
        
        BOOST_FOREACH(Moby::CollisionGeometryPtr cg, rb->geometries){
          Moby::PrimitivePtr primitive = cg->get_geometry();
          foot_primitive = boost::dynamic_pointer_cast<Moby::SpherePrimitive>(primitive);
          if(foot_primitive){
            foot_geometry = cg;
            break;
          }
        }
        
        Vector3d foot_radius(0,0,-foot_primitive->get_radius(),Moby::GLOBAL);
        Vector3d lowest_point = Ravelin::Pose3d::transform_vector(robot->get_base_link()->get_pose(),foot_radius);
        lowest_point += Vector3d(foot_pose->x,robot->get_base_link()->get_pose());
        lowest_point_z = std::min(lowest_point_z,lowest_point[2]);
      }
    }
    
    {
      logging1 << "moving robot to just above the ground: "<< lowest_point_z << std::endl;
      
      // Get robot velocity
      Ravelin::VectorNd q;
      robot->get_generalized_coordinates_euler(q);
      int N_JOINT_DOFS = q.rows()-7;
      // update base position
      q[N_JOINT_DOFS+2] = -lowest_point_z;
      
      // apply changes
      robot->set_generalized_coordinates_euler(q);
      logging1 << "Set coords to : " << q << std::endl;
    }
  }
  
  if(vm.count("float")){
    std::cerr << "moving robot to not touch the ground" << std::endl;
    
    // Get robot velocity
    Ravelin::VectorNd q;
    robot->get_generalized_coordinates_euler(q);
    int N_JOINT_DOFS = q.rows()-7;
    // update base position
    q[N_JOINT_DOFS+2] = 1;
    
    // apply changes
    robot->set_generalized_coordinates_euler(q);
    std::cerr << "Set coords to : " << q << std::endl;
  }
  
  // Update robot state
  robot->update_link_poses();
  
  if(USE_UNCER)
    if(vm.count("BODY0.xd")){
      logging1 << "applying state uncertainty to base (velocity)" << std::endl;
      
      std::vector<double> velocity = vm["BODY0.xd"].as<std::vector<double> >();
      
      // Get robot velocity
      Ravelin::VectorNd qd;
      robot->get_generalized_velocity(DynamicBodyd::eSpatial,qd);
      int N_JOINT_DOFS = qd.rows()-6;
      // update base velocity
      for (int i=0;i<6; i++) {
        qd[N_JOINT_DOFS+i] = velocity[i];
      }
      // apply changes
      robot->set_generalized_velocity(DynamicBodyd::eSpatial,qd);
    }
  
  //  // Joints
  //  if(USE_UNCER)
  //  BOOST_FOREACH(shared_ptr<Jointd> jp, robot->get_joints()){
  //    if(vm.count(jp->joint_id+".xd")){
  //      std::vector<double> velocity = vm[jp->joint_id+".xd"].as<std::vector<double> >();
  //      for (int i=0; i<jp->num_dof(); i++) {
  //        jp->qd[i] = velocity[i];
  //      }
  //    }
  //  }
  
  // Update robot state
  robot->update_link_velocities();
  
}

void apply_manufacturing_uncertainty(int argc,char* argv[],shared_ptr<RCArticulatedBodyd>& robot){
  // Add permissible robot parameters
  po::options_description desc("Monte Carlo Method manufacturing (applied at simulator start) uncertainty options");
  desc.add_options()
  ("help", "produce help message");
  
  logging1 << "Adding Options" << std::endl;
  
  // Joints
  BOOST_FOREACH(shared_ptr<Jointd> jp, robot->get_joints()){
    std::string name,help;
    
    if (jp->num_dof() == 1) {
      name = std::string(jp->joint_id+".axis");
      help = std::string("Axis vector in model (base link) frame of REVOLUTE Joint: "+jp->joint_id);

      desc.add_options()
      (name.c_str(), po::value<std::vector<double> >()->multitoken() ,help.c_str());

      name = std::string(jp->joint_id+".tare");
      help = std::string("Absolute position [m OR rad] offset of Joint: "+jp->joint_id);

      desc.add_options()
      (name.c_str(), po::value<std::vector<double> >()->multitoken() ,help.c_str());

    }
  }
  
  
  BOOST_FOREACH(shared_ptr<RigidBodyd> rb, robot->get_links()){
    std::string name,help;
    { // Density
      name = std::string(rb->body_id+".density");
      help = std::string("Density [kg/m^3] applied of link (default is solid polycarbonate): "+rb->body_id);
      
      desc.add_options()
      (name.c_str(), po::value<double>(),help.c_str());
      
      name = std::string(rb->body_id+".mass");
      help = std::string("Mass [kg] applied to link: "+rb->body_id);
      
      desc.add_options()
      (name.c_str(), po::value<double>(),help.c_str());
      
    }
    
    if(rb->is_base()){
      name = std::string(rb->body_id+".size");
      help = std::string("Body box geometry dimensions (length, width, height) [m] of base link: " +rb->body_id);
      
      desc.add_options()
      (name.c_str(), po::value<std::vector<double> >()->multitoken() ,help.c_str());
    }
    
    {  // Length & radius of cylindrical link
      name = std::string(rb->body_id+".length");
      help = std::string("LENGTH [m] of link LENGTH dimension (along limb axis): "+rb->body_id);
      desc.add_options()
      (name.c_str(), po::value<double>(),help.c_str());
      
      name = std::string(rb->body_id+".radius");
      help = std::string("Radius [m] of link RADIUS dimension: "+rb->body_id);
      
      desc.add_options()
      (name.c_str(), po::value<double>(),help.c_str());
    }
    
    if(rb->is_end_effector()){ // radius of spherical link
      name = std::string(rb->body_id+".foot.radius");
      help = std::string("Radius [m] of foot RADIUS dimension: "+rb->body_id);
      
      desc.add_options()
      (name.c_str(), po::value<double>(),help.c_str());
      
      name = std::string(rb->body_id+".foot.density");
      help = std::string("Density [kg/m^3] applied to link's spherical end effector (default is solid polycarbonate): "+rb->body_id);
      
      desc.add_options()
      (name.c_str(), po::value<double>(),help.c_str());
      
      name = std::string(rb->body_id+".foot.mass");
      help = std::string("Mass [kg] applied to link's spherical end effector: "+rb->body_id);
      
      desc.add_options()
      (name.c_str(), po::value<double>(),help.c_str());
    }
  }
  
  logging1 << "Parsing Options" << std::endl;
  
  po::variables_map vm;
  
  po::parsed_options parsed
  = po::command_line_parser(argc, argv).options(desc).style(po::command_line_style::unix_style ^ po::command_line_style::allow_short).allow_unregistered().run();
  
  po::store(parsed, vm);
  
  if ( vm.count("help")  )
  {
    logging1 << "Available options: " << std::endl
    << desc << std::endl;
  }
  
  /*
   *  Parameter Application
   */
  
  logging1 << "Joints" << std::endl;
  
  
  //  shared_ptr<RigidBodyd> base_link_ptr = robot->get_base_link();
  //  shared_ptr<const Pose3d> model_pose = base_link_ptr->get_pose();
  BOOST_FOREACH(shared_ptr<Jointd> jp, robot->get_joints()){
    shared_ptr<RevoluteJointd> rjp = boost::dynamic_pointer_cast<RevoluteJointd>(jp);
    logging1 << "Joint: " << jp->joint_id << std::endl;
    
    if(vm.count(jp->joint_id+".axis")){
      std::vector<double> axis = vm[jp->joint_id+".axis"].as<std::vector<double> >();
      
      const Vector3d joint_axis = rjp->get_axis();
      logging1 << "    joint_axis: " << joint_axis << std::endl;
      Vector3d new_joint_axis(&axis[0],joint_axis.pose);
      new_joint_axis.normalize();
      logging1 << "    new_joint_axis" << new_joint_axis << std::endl;
      //      Vector3d new_joint_axis_inner_fame = Pose3d::transform_vector(joint_axis.pose,new_joint_axis);
      //      logging1 << "new_joint_axis_inner_fame" << new_joint_axis_inner_fame << std::endl;
      rjp->set_axis(new_joint_axis);
    }
    
    if(vm.count(jp->joint_id+".tare")){
      std::vector<double> tare = vm[jp->joint_id+".tare"].as<std::vector<double> >();
      std::cout << "    joint_tare (set): " << tare << std::endl;
      Ravelin::VectorNd tare_v = rjp->get_q_tare();
      std::cout << "    joint_tare (moby): " << tare_v << std::endl;
      
      assert(tare_v.rows() == tare.size());
      for (int i=0; i<tare.size(); i++) {
        tare_v[i] += tare[i];
      }
      rjp->set_q_tare(tare_v);
    }
    rjp->update_spatial_axes();
    robot->update_link_poses();
  }
  
  
  logging1 << "Links" << std::endl;
  
  // Perturb mass of robot link by scaling by parameter
  BOOST_FOREACH(shared_ptr<Ravelin::RigidBodyd> rbd, robot->get_links()){
    shared_ptr<Moby::RigidBody> rb = boost::dynamic_pointer_cast<Moby::RigidBody>(rbd);
    logging1 << "Link: " << rb->body_id << std::endl;
    
    // Link Length Adjustment
    //#ifdef USE_OSG_DISPLAY
    //    osg::Group* no_viz_data = new osg::Group();
    //    rb->set_visualization_data(no_viz_data);
    //#endif
    
    bool LINK_IS_END_EFFECTOR = rb->is_end_effector();
    bool LINK_IS_BASE = rb->is_base();
    bool LINK_IS_LIMB = !LINK_IS_BASE;
    //////////////// LINK DATA /////////////////
    double mass     = -1;
    if (vm.count(rb->body_id+".mass")) mass = vm[rb->body_id+".mass"].as<double>();
    
    double density  = -1;
    if(   vm.count(rb->body_id+".density")) density = vm[rb->body_id+".density"].as<double>();
    double length   = -1;
    if(   vm.count(rb->body_id+".length")) length = (LINK_IS_LIMB)? vm[rb->body_id+".length"].as<double>() : -1;
    double radius   = -1;
    if(   vm.count(rb->body_id+".radius")) radius = (LINK_IS_LIMB)? vm[rb->body_id+".radius"].as<double>() : -1;
    
    //////////////// FOOT DATA /////////////////
    double foot_mass    = -1;
    if (vm.count(rb->body_id+".foot.mass")) foot_mass = vm[rb->body_id+".foot.mass"].as<double>();
    
    double foot_density = -1;
    if(   vm.count(rb->body_id+".foot.density")) foot_density = (LINK_IS_END_EFFECTOR)? vm[rb->body_id+".foot.density"].as<double>() : 0;
    double foot_radius  = -1;
    if(   vm.count(rb->body_id+".foot.radius")) foot_radius = (LINK_IS_END_EFFECTOR)? vm[rb->body_id+".foot.radius"].as<double>() : 0;
    Moby::CollisionGeometryPtr foot_geometry;
    shared_ptr<Moby::SpherePrimitive> foot_primitive;
    
    //    Moby::CollisionGeometryPtr limb_geometry;
    //    shared_ptr<Moby::CylinderPrimitive> limb_primitive;
    
    if(density == -1 && mass == -1 && length == -1 && radius == -1)
      continue;
    
    Moby::CollisionGeometryPtr base_geometry;
    shared_ptr<Moby::BoxPrimitive> base_primitive;
    
    if (LINK_IS_END_EFFECTOR) {
      BOOST_FOREACH(Moby::CollisionGeometryPtr cg, rb->geometries){
        Moby::PrimitivePtr primitive = cg->get_geometry();
        foot_primitive = boost::dynamic_pointer_cast<Moby::SpherePrimitive>(primitive);
        if(foot_primitive){
          foot_geometry = cg;
          break;
        }
      }
    }
    
    //    if(LINK_IS_LIMB){
    //      BOOST_FOREACH(Moby::CollisionGeometryPtr cg, rb->geometries){
    //        Moby::PrimitivePtr primitive = cg->get_geometry();
    //        limb_primitive = boost::dynamic_pointer_cast<Moby::CylinderPrimitive>(primitive);
    //        if(foot_primitive){
    //          limb_geometry = cg;
    //          break;
    //        }
    //      }
    //    }
    
    if (LINK_IS_BASE) {
      BOOST_FOREACH(Moby::CollisionGeometryPtr cg, rb->geometries){
        Moby::PrimitivePtr primitive = cg->get_geometry();
        base_primitive = boost::dynamic_pointer_cast<Moby::BoxPrimitive>(primitive);
        if(base_primitive){
          base_geometry = cg;
          break;
        }
      }
    }
    
    
    //////////////////////////  APPLY LINK LENGTH  /////////////////////////////
    boost::shared_ptr<const Ravelin::Pose3d> outer_pose;
    if (LINK_IS_LIMB){
      boost::shared_ptr<const Ravelin::Pose3d> inner_joint_pose = rb->get_inner_joint_explicit()->get_pose();
      if (LINK_IS_END_EFFECTOR) {
#ifndef NDEBUG
        logging1 << "End Effector link: " << rb->body_id << std::endl;
#endif
        // assumes no outer joint
        // pose of the outer joint
        outer_pose = foot_geometry->get_pose();
      } else {  // stretch the distance between 2 joints
#ifndef NDEBUG
        logging1 << "Interior link: " << rb->body_id << std::endl;
#endif
        
        // assumes one outer joint
        // pose of the outer joint
        const std::set<shared_ptr<Jointd> >& outers = rb->get_outer_joints();
        //sets inner and outer joints for the current link
        
        outer_pose = (*(outers.begin()))->get_pose();
      }
      
      boost::shared_ptr<Ravelin::Pose3d> outer_joint_wrt_inner(new Ravelin::Pose3d(outer_pose->q,outer_pose->x,outer_pose->rpose));
      
    
      // pose of the outer joint defined wrt inner joint
//#ifndef NDEBUG
      std::cerr << "Sample " << SAMPLE_NUMBER << " : Link "<< rb->body_id.c_str() <<" length = " << length << std::endl;
//#endif
      
      outer_joint_wrt_inner->update_relative_pose(inner_joint_pose);
      std::cerr << "Before: " << *outer_joint_wrt_inner << std::endl;
      boost::shared_ptr<Ravelin::Pose3d> outer_joint_wrt_inner_before(new Ravelin::Pose3d(outer_joint_wrt_inner->q,outer_joint_wrt_inner->x,outer_joint_wrt_inner->rpose));

      // Update transform in link forward direction
      
      outer_joint_wrt_inner->x.normalize();
      outer_joint_wrt_inner->x *= length;
      std::cerr << "After: " <<  *outer_joint_wrt_inner << std::endl;
      length = outer_joint_wrt_inner->x.norm();
      
      if (LINK_IS_END_EFFECTOR) {
//        outer_joint_wrt_inner->update_relative_pose(outer_pose->rpose);
//        foot_geometry->set_relative_pose(*outer_joint_wrt_inner);
        Ravelin::Transform3d T_21 = Ravelin::Pose3d::calc_relative_pose(outer_joint_wrt_inner_before,outer_joint_wrt_inner);
        std::cerr << "Transform: \n" << Ravelin::Matrix3d(T_21.q) << " , " << T_21.x << std::endl;
        boost::shared_ptr<const Ravelin::Pose3d> p1_const = rb->get_pose();
        Ravelin::Pose3d p1(p1_const->q,p1_const->x,p1_const->rpose);
        p1.update_relative_pose(outer_joint_wrt_inner_before->rpose);
        std::cerr << "Pose1: " << p1 << std::endl;

        // TODO: This leads to orientation change in pose, but rotation is Identity, bug in Transform3d
//        Ravelin::Pose3d p2 = T_21.transform(p1);
        Ravelin::Pose3d p2 = p1;
        p2.x += T_21.x;
        std::cerr << "Pose2: " << p2 << std::endl;
        p2.update_relative_pose(p1_const->rpose);
        rb->set_pose(p2);
        robot->update_link_poses();
      } else {
        const std::set<shared_ptr<Jointd> >& outers = rb->get_outer_joints();
        
        // Update transforms
        try {
          (*(outers.begin()))->Ravelin::Jointd::set_location(Ravelin::Vector3d(0,0,0,outer_joint_wrt_inner),(*(outers.begin()))->get_inboard_link(),(*(outers.begin()))->get_outboard_link());
        } catch (...) {}
        // Apply new transform to all children (recursive)
        // NOTE: Doesnt seem to be necessary
        //          apply_transform(T,rb->get_outer_joints());
        robot->update_link_poses();
      }
    }
    
    
    
    // Link Mass Adjustment
    {
      Ravelin::SpatialRBInertiad J_link(rb->get_inertia());
      if (LINK_IS_LIMB && (mass > 0 || (density > 0 && radius > 0) ) ){
        // Cylinder limb
        // cylinder V = pi*h*r^2
        double volume = M_PI*length*sqr(radius);
        // m = d*V
        if (mass <= 0) {
          mass = density * volume;
        }
        // cylinder J =
        // (m*r^2)/2          0                   0
        //    0      (m/12)(3r^2 + h^2)           0
        //    0               0          (m/12)(3r^2 + h^2)
        double J[3] = {
          (mass*sqr(radius)*0.5),
          (mass/12.0)*(3.0*sqr(radius) + sqr(length)),
          (mass/12.0)*(3.0*sqr(radius) + sqr(length))};
        
        J_link.m = mass;
        J_link.J = Ravelin::Matrix3d(J[0],0,0,
                                     0,J[1],0,
                                     0,0,J[2]);
        
        if(LINK_IS_END_EFFECTOR){ // sphere foot
          foot_primitive->set_radius(foot_radius);
          
          // sphere V = 4/3 pi*r^3
          double volume = (4.0/3.0)*M_PI*(foot_radius*foot_radius*foot_radius);
          
          double mass = foot_mass;
          // m = d*V
          if (mass <= 0) {
            mass = density * volume;
          }
          
          // sphere J =
          // (2*r^2)/5     0         0
          //    0      (2*r^2)/5     0
          //    0          0     (2*r^2)/5
          double inertia = 2.0*mass*sqr(foot_radius) / 5.0;
          double J[3] = {inertia,inertia,inertia};
          
          logging1 << "Sample " << SAMPLE_NUMBER << " : Link (foot) "<< rb->body_id.c_str() << " mass = "<< mass ;
          
          Ravelin::SpatialRBInertiad J_foot;
          J_foot.m = mass;
          J_foot.J = Ravelin::Matrix3d(J[0],0,0,
                                       0,J[1],0,
                                       0,0,J[2]);
          
          //-- transform foot inertia to link inertial frame --//
          // foot pose
          shared_ptr<const Ravelin::Pose3d> foot_pose = foot_geometry->get_pose();
          Ravelin::Pose3d foot_relative_to_link(foot_pose->q,foot_pose->x,foot_pose->rpose);
          // foot pose relative to link
          foot_relative_to_link.update_relative_pose(J_link.pose);
          // transform foot to link
          Ravelin::Transform3d foot_link_transform(foot_relative_to_link.q,foot_relative_to_link.x);
          // transform inertia and add to link inertia
          Ravelin::SpatialRBInertiad J_foot_new = foot_link_transform.transform(J_foot);
          J_foot_new.pose = J_link.pose;
          J_link += J_foot_new;
        }
        
      } else if (LINK_IS_BASE) {
        if (vm.count(rb->body_id+".size")) {
          std::vector<double> size = vm[rb->body_id+".size"].as<std::vector<double> >();
          double x = size[0],
          y = size[1],
          z = size[2];
          
          // Adjust collision and weight affecting values
          base_primitive->set_size(x,y,z);
        }
        
        
        // Box V = x * y * z
        double x = base_primitive->get_x_len(),
        y = base_primitive->get_y_len(),
        z = base_primitive->get_z_len();
        
        const std::set<shared_ptr<Jointd> >& outers = rb->get_outer_joints();
        // Adjust kinematics
        BOOST_FOREACH( shared_ptr<Jointd> outer_joint, outers){
          boost::shared_ptr<const Ravelin::Pose3d> inner_joint_pose = rb->get_pose();
#ifndef NDEBUG
          logging1 << "Interior link: " << rb->body_id << std::endl;
#endif
          
          // assumes one outer joint
          // pose of the outer joint
          outer_pose = outer_joint->get_pose();
          
          boost::shared_ptr<Ravelin::Pose3d> outer_joint_wrt_inner(new Ravelin::Pose3d(outer_pose->q,outer_pose->x,outer_pose->rpose));
          
          // pose of the outer joint defined wrt inner joint
#ifndef NDEBUG
          logging1 << "Sample " << SAMPLE_NUMBER << " : Link "<< rb->body_id.c_str() <<" length = " << length << std::endl;
#endif
          
          outer_joint_wrt_inner->update_relative_pose(inner_joint_pose);
          logging1 << "Before: " << *outer_joint_wrt_inner << std::endl;
          // Update transform in link's quadrant
          Origin3d dir = outer_joint_wrt_inner->x;
          dir.normalize();
          outer_joint_wrt_inner->x = Origin3d(Utility::sign(dir[0])*x*0.5,Utility::sign(dir[1])*y*0.5,-z*0.5);
          logging1 << "After: " <<  *outer_joint_wrt_inner << std::endl;
          
          // Update transforms
          try{
            outer_joint->Ravelin::Jointd::set_location(Ravelin::Vector3d(0,0,0,outer_joint_wrt_inner),outer_joint->get_inboard_link(),outer_joint->get_outboard_link());
          } catch (...) {}
          // Apply new transform to all children (recursive)
          // NOTE: Doesnt seem to be necessary
          //          apply_transform(T,rb->get_outer_joints());
          robot->update_link_poses();
        }
        
        double volume = x*y*z;
        // m = d*V
        if (mass <= 0) {
          mass = density * volume;
        }
        // cylinder J =
        // (m*r^2)/2          0                   0
        //    0      (m/12)(3r^2 + h^2)           0
        //    0               0          (m/12)(3r^2 + h^2)
        double J[3] = {
          (mass/12.0)*(      y*y + z*z),
          (mass/12.0)*(x*x       + z*z),
          (mass/12.0)*(x*x + y*y      )};
        
        J_link.m = mass;
        J_link.J = Ravelin::Matrix3d(J[0],0,0,
                                     0,J[1],0,
                                     0,0,J[2]);
      }
      logging1 << "Sample " << SAMPLE_NUMBER << " : Link "<< rb->body_id.c_str() <<" mass = " << J_link.m << std::endl
      <<" inertia = " << J_link.J << std::endl;
      rb->set_inertia(J_link);
    }
  }
}

void parse_sample_options(int argc, char* argv[]){
  // Declare the supported options.
  po::options_description desc("Moby initialization options");
  desc.add_options()
  ("help", "produce help message")
  ("xml,y","Output XML model file for robot")
  ("sample",po::value<int>(),"sample number for this process")
  ("duration", po::value<std::string>()->default_value("0"),"Duration of simulation.");
  
  logging1 << "Parsing Variable Map from command line" << std::endl;
  
  po::variables_map vm;
  //  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::parsed_options parsed
  = po::command_line_parser(argc, argv).style(po::command_line_style::unix_style ^ po::command_line_style::allow_short).options(desc).allow_unregistered().run();
  po::store(parsed, vm);
  //  po::notify(vm);
  
  logging1 << "Parsed Variable Map from command line" << std::endl;
  
  if(vm.count("sample")){
    std::string
    duration = vm["duration"].as<std::string>();
    DURATION = atof(duration.c_str());
  }
  
  if(vm.count("sample")){
    SAMPLE_NUMBER = vm["sample"].as<int>();
  } else {
    SAMPLE_NUMBER = 0;
  }
  
  logging1 << "Sample with PID: "<< pid << " has sample number "<< SAMPLE_NUMBER << std::endl;
  
  if (vm.count("xml")) {
    EXPORT_XML = true;
  }
  
  if ( vm.count("help")  )
  {
    logging1 << "Available options: " << std::endl
    << desc << std::endl;
  }
  
  return;
}

void preload_simulation(int argc, char* argv[], shared_ptr<Simulator>& sim){
  // Declare the supported options.
  po::options_description desc("Moby initialization options (preload)");
  desc.add_options()
  ("visual",po::value<int>(),"-1: only last frame; 0 no visual; >0: every n frames & last frame")
  ("moby",  po::value<std::vector<std::string> >()->multitoken(), "moby options?");
  
  //  ("sample", po::value<unsigned>(),"Sample Number");
  logging1 << "Parsing Variable Map from command line" << std::endl;
  
  po::variables_map vm;
  //  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::parsed_options parsed
  = po::command_line_parser(argc, argv).style(po::command_line_style::unix_style ^ po::command_line_style::allow_short).options(desc).allow_unregistered().run();
  po::store(parsed, vm);
  //  po::notify(vm);
  
  
  logging1 << "Parsed Variable Map from command line" << std::endl;
  
  if (vm.count("visual")) {
    VISUAL_MOD = vm["visual"].as< int >();
  }
  
  if(!PACER_ONLY){
    /*
     *  Moby Initialization
     */
    logging1 << " -- Populating command line options -- " << std::endl;
    
    // run sample
    std::vector<std::string> argvs;
    argvs.push_back("montecarlo-moby-sample");
    // Max time is 0.3 seconds
    if (!vm["moby"].empty()) {
      std::vector<std::string> opts = vm["moby"].as<std::vector<std::string> >();
      logging1 << " -- setting moby options: " << opts << std::endl;
      for (int i = 0; i<opts.size()-1; i++) {
        argvs.push_back("-" + opts[i]);
      }
      argvs.push_back(opts[opts.size()-1]);
    }
    logging1 << "Converting command line options: " << argvs << std::endl;
    
    char** moby_argv = param_array_noconst(argvs);
    
    logging1 << "Moby Starting: " << std::endl;
    for ( size_t i = 0 ; i < argvs.size() ; i++ ){
      logging1 << argvs[i] << " ";
    }
    logging1 << std::endl;
    
    // Ask Moby to init the simulator with options
    Moby::init(argvs.size(), moby_argv,sim);
    
    logging1 << "Moby Started: " << std::endl;
    
    // clean up argv
    //  for ( size_t i = 0 ; i < argvs.size() ; i++ ){
    //    delete[] moby_argv[i];
    //  }
    //  delete[] moby_argv;
  } else {
    logging1 << "Moby not Started, using only pacer: " << std::endl;
  }
  return;
}


void parse_command_line_options(int argc, char* argv[]){
  // Declare the supported options.
  po::options_description desc("Moby initialization options");
  desc.add_options()
  ("help", "produce help message")
  ("no-pipe","expect piped parameters?")
  ("same-model","do not update model?")
  ("kinematic", "load pacer alone (no moby physics)");
  
  logging1 << "Parsing Variable Map from command line" << std::endl;
  
  po::variables_map vm;
  //  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::parsed_options parsed
  = po::command_line_parser(argc, argv).style(po::command_line_style::unix_style ^ po::command_line_style::allow_short).options(desc).allow_unregistered().run();
  po::store(parsed, vm);
  //  po::notify(vm);
  
  logging1 << "Parsed Variable Map from command line" << std::endl;
  
  if(vm.count("same-model"))
    USE_UNCER = false;
  
  if (vm.count("kinematic")) {
    PACER_ONLY = true;
    USE_UNCER = false;
  }
  
  if (vm.count("no-pipe")) {
    USE_PIPES = false;
  } else {
    USE_PIPES = true;
  }
  
  
}
