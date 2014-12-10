/*****************************************************************************
 * Controller for LINKS robot
 ****************************************************************************/

#include <controller.h>
#include <boost/bind.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>

#include <stdio.h>

 boost::shared_ptr<Controller> robot_ptr;

 void init_plugin()
 {
   /// Set up quadruped robot, linking data from moby's articulated body
   /// to the quadruped model used by Control-Moby
   robot_ptr = boost::shared_ptr<Controller>(new Controller());
 }

 namespace gazebo
 {
   class ControllerPlugin : public ModelPlugin
   {
     private: physics::WorldPtr world;
     private: std::vector<sensors::ContactSensorPtr> contacts;
     private: std::vector<std::string> foot_names;

     // Pointer to the model
     private: physics::ModelPtr model;
     // Pointer to the update event connection
     private: std::vector<event::ConnectionPtr> connections;
     private: std::map<int,int> joint_map;

     gazebo::transport::NodePtr receiver;
     std::vector<gazebo::transport::SubscriberPtr> subs;

     public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
     {
       OUT_LOG(logERROR) << ">> start Plugin: Load(.)";
       // Store the pointer to the model
       this->model = _parent;

       // store the pointer to the world
       world = model->GetWorld();

       init_plugin();

       Utility::get_variable("init.end-effector.id",foot_names);

       // get the sensor
       for(int i=0;i<foot_names.size();i++){
         std::string sensor_name(foot_names[i]+"_contact");
         OUT_LOG(logERROR) << "initing sensor : " << sensor_name;
         contacts.push_back(
               boost::dynamic_pointer_cast<sensors::ContactSensor>(
                 sensors::SensorManager::Instance()->GetSensor(sensor_name)
               )
             );
//         sensors::ContactSensorPtr->SetActive(true);
         if(contacts[i])
           contacts[i]->SetActive(true);
       }

       this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
           boost::bind(&ControllerPlugin::UpdateController, this)));

       // SET UP JOINT MAPPING AND INIT POS

       std::vector<Moby::JointPtr> joints_quad = robot_ptr->get_joints();
       physics::Joint_V joints = model->GetJoints();

       for(int i=0;i<joints.size();i++){
         for(int j=0;j<joints_quad.size();j++){
           if(!joints_quad[j]) continue;
           if(
              joints_quad[j]->id.compare(
                joints[i]->GetName().substr(
                  joints[i]->GetName().length()-joints_quad[j]->id.length(),
                  joints_quad[j]->id.length())
                ) == 0){
//               for(int d = 0;d<joints_quad[j]->num_dof();d++)
               joint_map[i] = joints_quad[j]->get_coord_index();//+d;
           }
         }
       }

       OUT_LOG(logERROR) << "Gazebo -> Moby Mapping";
       for(int i=0;i<joints.size();i++){
         OUT_LOG(logERROR) << " Gazebo " <<  i << " : " << joints[i]->GetName();
         OUT_LOG(logERROR) << " = Moby " << joint_map[i] << " : " << joints_quad[joint_map[i]]->id;
       }

       // ================= INIT ROBOT STATE ==========================

       std::vector<double>
           workvd,
           base_start = Utility::get_variable("init.base.x",workvd);
//       physics::LinkPtr base_ptr = model->GetLink("ROBOT::"+robot_ptr->get_links()[0]->id);
       math::Pose p(base_start[0],base_start[1],base_start[2],base_start[3],base_start[4],base_start[5]);
       model->SetWorldPose(p);
       model->SetLinkWorldPose(p,"ROBOT::"+robot_ptr->get_links()[0]->id);

       std::map<std::string,double> q0 = robot_ptr->get_q0();

       for(int i=0,ii=0;i<joints.size();i++){
         for(int j=0;j<joints[i]->GetAngleCount();j++){
           math::Angle a(q0[std::to_string(j)+joints_quad[joint_map[i]]->id]);
           joints[i]->SetAngle(j,a);
         }
       }

       BOOST_FOREACH(physics::JointPtr j_ptr,joints)
       {
         OUT_LOG(logERROR) << j_ptr->GetId() << " " << j_ptr->GetName() << " " << j_ptr->GetAngle(0);
       }

       OUT_LOG(logERROR) << "<< end Plugin: Load(.)";
       sleep(5);
     }

   public: void UpdateController()
   {
       // Control Robot
       OUT_LOG(logERROR) << ">> start Plugin: Update(.)";
       static double last_time = -0.001;
       double t = world->GetSimTime().Double();
       double dt = t - last_time;
       last_time = t;

       OUT_LOG(logERROR) << "**Starting Sensor**";

       for(int f=0;f<contacts.size();f++){
         EndEffector& eef = robot_ptr->get_end_effectors()[f];
         OUT_LOG(logERROR) << "Contact Sensor # (foot): " << f << " : " << eef.id;

         OUT_LOG(logERROR) << "Contact Sensor ID: " << contacts[f]->GetId();
         OUT_LOG(logERROR) << "Contact Sensor Collision Name: " << contacts[f]->GetCollisionName(0);
         OUT_LOG(logERROR) << "Contact Sensor Topic: " << contacts[f]->GetTopic();

         const msgs::Contacts& c = contacts[f]->GetContacts();

         if(c.contact_size() > 0){
           eef.active = true;
           for (unsigned i = 0; i < c.contact_size(); i++)
           {
             std::string c1 = c.contact(i).collision1();
             std::string c2 = c.contact(i).collision2();

             OUT_LOG(logERROR) << "Collision between[" << c1
                       << "] and [" << c2 << "]\n";

             for (unsigned j = 0; j < c.contact(i).position_size(); ++j)
             {
               OUT_LOG(logERROR) << j << "   Position: ["
                         << c.contact(i).position(j).x() << " "
                         << c.contact(i).position(j).y() << " "
                         << c.contact(i).position(j).z() << "]\n";
               OUT_LOG(logERROR) << "   Normal: ["
                         << c.contact(i).normal(j).x() << " "
                         << c.contact(i).normal(j).y() << " "
                         << c.contact(i).normal(j).z() << "]\n";
               OUT_LOG(logERROR) << "   Depth:" << c.contact(i).depth(j) << "\n";

                // Add contact point to this end effector
                eef.point.push_back(Ravelin::Vector3d(c.contact(i).position(j).x(),c.contact(i).position(j).y(),c.contact(i).position(j).z()));

                // Add contact point to this end effector
                eef.normal.push_back(Ravelin::Vector3d(c.contact(i).normal(j).x(),c.contact(i).normal(j).y(),c.contact(i).normal(j).z()));

                Ravelin::Vector3d tan1, tan2;
                Ravelin::Vector3d::determine_orthonormal_basis(eef.normal.back(),tan1,tan2);
                eef.tan1.push_back(tan1);
                eef.tan2.push_back(tan2);

                // Add contact point to this end effector
                eef.impulse.push_back(Ravelin::Vector3d(c.contact(i).wrench(j).body_1_wrench().force().x(),c.contact(i).wrench(j).body_1_wrench().force().y(),c.contact(i).wrench(j).body_1_wrench().force().z()));
                eef.impulse.back() *= dt;
                eef.mu_coulomb.push_back(0.1);

                OUT_LOG(logERROR) <<  "-- 1 :\n\tp=("
                                  << eef.point.front() << ")\n\tn=("
                                  << eef.normal.front() << ")\n\ti=("
                                  << eef.impulse.front() << ")";
              }
           }
         }
       }

       physics::Joint_V joints = model->GetJoints();

       OUT_LOG(logERROR) << "**Starting controller**";

       std::vector<Moby::JointPtr> joints_quad = robot_ptr->get_joints();

       int num_joints = joints_quad.size();
       int num_joints_dofs = 0;
       for(int i=0;i<joints.size();i++)
         for(int j=0;j<joints[i]->GetAngleCount();j++,num_joints_dofs++){}

       Ravelin::VectorNd generalized_q(num_joints_dofs+7),generalized_qd(num_joints_dofs+6),generalized_qdd(num_joints_dofs+6), generalized_fext(num_joints_dofs+6);

       // Re-map state simulation->robot joints
       {
         physics::LinkPtr base_ptr = model->GetLink("ROBOT::"+robot_ptr->get_links()[0]->id);

         math::Pose base_pose = base_ptr->GetWorldInertialPose();
         math::Vector3    pos = base_pose.pos;
         math::Quaternion rot = base_pose.rot;
         generalized_q.set_sub_vec(num_joints_dofs  ,Ravelin::Vector3d(pos.x,pos.y,pos.z));
         generalized_q[num_joints_dofs+3+0] = rot.w;
         generalized_q[num_joints_dofs+3+1] = rot.x;
         generalized_q[num_joints_dofs+3+2] = rot.y;
         generalized_q[num_joints_dofs+3+3] = rot.z;

         math::Vector3    vel = base_ptr->GetWorldLinearVel();
         math::Vector3   avel = base_ptr->GetWorldAngularVel();

         generalized_qd.set_sub_vec(num_joints_dofs  ,Ravelin::Vector3d(vel.x,vel.y,vel.z) );
         generalized_qd.set_sub_vec(num_joints_dofs+3,Ravelin::Vector3d(avel.x,avel.y,avel.z) );


         math::Vector3  force = base_ptr->GetWorldForce();
         math::Vector3 torque = base_ptr->GetWorldTorque();

         generalized_fext.set_sub_vec(num_joints_dofs  , Ravelin::Vector3d(force.x,force.y,force.z) );
         generalized_fext.set_sub_vec(num_joints_dofs+3,Ravelin::Vector3d(torque.x,torque.y,torque.z));

         // Set joint params in Moby joint coordinates
         for(int i=0;i<joints.size();i++){
           physics::JointPtr joint = joints[i];
           for(int j=0;j<joint->GetAngleCount();j++){
             generalized_q[joint_map[i]+j] = joint->GetAngle(j).Radian();
             generalized_qd[joint_map[i]+j] = joint->GetVelocity(j);
             generalized_fext[joint_map[i]+j] = joint->GetForce(j);
           }
         }

         // Calculate joint accelerations
         static Ravelin::VectorNd  generalized_qd_last = generalized_qd;
         ((generalized_qdd = generalized_qd) -= generalized_qd_last) /= dt;
         generalized_qd_last = generalized_qd;

         math::Vector3  accel = base_ptr->GetWorldLinearAccel();
         math::Vector3 aaccel = base_ptr->GetWorldAngularAccel();

         // Get base accelerations
         generalized_qdd.set_sub_vec(num_joints_dofs,Ravelin::Vector3d(accel.x,accel.y,accel.z));
         generalized_qdd.set_sub_vec(num_joints_dofs+3,Ravelin::Vector3d(aaccel.x,aaccel.y,aaccel.z));
       }

       OUTLOG(generalized_fext,"generalized_fext",logERROR);
       OUTLOG(generalized_q,"generalized_q",logERROR);
       OUTLOG(generalized_qd,"generalized_qd",logERROR);
       OUTLOG(generalized_qdd,"generalized_qdd",logERROR);

       Ravelin::VectorNd q_des(num_joints),
                         qd_des(num_joints),
                         qdd_des(num_joints);
       Ravelin::VectorNd u(num_joints);

       robot_ptr->control(t,generalized_q,generalized_qd,generalized_qdd,generalized_fext,q_des,qd_des,qdd_des,u);
       OUTLOG(u,"u-gazebo",logERROR);

     //  apply_sim_perturbations();
       for(int i=0;i<joints.size();i++){
         physics::JointPtr joint = joints[i];
         for(int j=0;j<joint->GetAngleCount();j++){
           joint->SetForce(j,u[joint_map[i]+j]);
         }
       }

       // Apply a small linear velocity to the model.
       OUT_LOG(logERROR) << "<< end Plugin: Update(.)";
     }
     };

   // Register this plugin with the simulator
   GZ_REGISTER_MODEL_PLUGIN(ControllerPlugin)
 }
