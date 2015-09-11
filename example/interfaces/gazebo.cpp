/*****************************************************************************
 * Controller for LINKS robot
 ****************************************************************************/

#include <Pacer/controller.h>
#include <boost/bind.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>

#include <stdio.h>

using namespace Pacer;

boost::shared_ptr<Controller> robot_ptr;

void init_plugin()
{
  /// Set up quadruped robot, linking data from moby's articulated body
  /// to the quadruped model used by Control-Moby
  robot_ptr = boost::shared_ptr<Controller>(new Controller());
  robot_ptr->init();
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
      
      foot_names = robot_ptr->get_data<std::vector<std::string> >("init.end-effector.id");
      
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
      
      std::vector<double>
      base_start = robot_ptr->get_data<std::vector<double> >("init.base.x");
      
      math::Pose p(base_start[0],base_start[1],base_start[2],base_start[3],base_start[4],base_start[5]);
      model->SetWorldPose(p);
      OUT_LOG(logERROR) << "Gazebo model name = " << model->GetName();
      model->SetLinkWorldPose(p,model->GetName()+"::"+robot_ptr->get_root_link()->body_id);
      
      // Re-map state simulation->robot joints
      std::map<std::string, Ravelin::VectorNd> q_start;
      robot_ptr->get_joint_value(Pacer::Robot::position,q_start);
      
      physics::Joint_V joints = model->GetJoints();
      for(int i=0,ii=0;i<joints.size();i++){
        //OUT_LOG(logERROR) << joints[i]->GetName() << ", dofs: " << joints[i]->GetAngleCount();
        int JDOFS = robot_ptr->get_joint_dofs(joints[i]->GetName());
        OUT_LOG(logERROR) << joints[i]->GetName() << ", dofs: " << JDOFS;
        for(int j=0;j<JDOFS;j++){
          math::Angle a(q_start[joints[i]->GetName()][j]);
          joints[i]->SetAngle(j,a);
        }
      }
      
      OUT_LOG(logERROR) << "<< end Plugin: Load(.)";
    }
    
  public: void UpdateController()
    {
      // Control Robot
      OUT_LOG(logERROR) << ">> start Plugin: Update(.)";
      static double last_time = -0.001;
      double t = world->GetSimTime().Double();
      double dt = t - last_time;
      OUTLOG(last_time,"last_time",logERROR);
      OUTLOG(t,"t",logERROR);
      OUTLOG(dt,"dt",logERROR);
      last_time = t;
      
      physics::Joint_V joints = model->GetJoints();

      if(dt > 1e-4){
        robot_ptr->reset_state();

        OUT_LOG(logERROR) << "**Starting Sensor**";
        
        for(int f=0;f<contacts.size();f++){
          OUT_LOG(logERROR) << "Contact Sensor ID: " << contacts[f]->GetId();
          OUT_LOG(logERROR) << "Contact Sensor Collision Name: " << contacts[f]->GetCollisionName(0);
          OUT_LOG(logERROR) << "Contact Sensor Topic: " << contacts[f]->GetTopic();
          
          const msgs::Contacts& c = contacts[f]->GetContacts();
          
          if(c.contact_size() > 0){
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
                Ravelin::Vector3d point(c.contact(i).position(j).x(),c.contact(i).position(j).y(),c.contact(i).position(j).z());
                
                // Add contact point to this end effector
                Ravelin::Vector3d normal(c.contact(i).normal(j).x(),c.contact(i).normal(j).y(),c.contact(i).normal(j).z());
                Ravelin::Vector3d tangent(c.contact(i).normal(j).y(),-c.contact(i).normal(j).x(),c.contact(i).normal(j).z());
                
                // Add contact point to this end effector
                Ravelin::Vector3d impulse(c.contact(i).wrench(j).body_1_wrench().force().x(),c.contact(i).wrench(j).body_1_wrench().force().y(),c.contact(i).wrench(j).body_1_wrench().force().z());
                impulse *= dt;
                double mu_coulomb = 0.1;
                robot_ptr->add_contact(foot_names[f],point,normal,tangent,impulse,mu_coulomb);
              }
            }
          }
        }
        
        
        //robot_ptr->add_contact(foot_names[f],point,normal,impulse,mu_coulomb);
        
        OUT_LOG(logERROR) << "**Starting controller**";
        
        Ravelin::VectorNd
        base_q(Pacer::NEULER),
        base_qd(Pacer::NSPATIAL),
        base_qdd(Pacer::NSPATIAL),
        base_fext(Pacer::NSPATIAL);
        
        // Re-map state simulation->robot joints
        {
          physics::LinkPtr base_ptr = model->GetLink(model->GetName()+"::"+robot_ptr->get_root_link()->body_id);
          // Get Gazebo Pose
          math::Pose base_pose = base_ptr->GetWorldInertialPose();
          math::Vector3    pos = base_pose.pos;
          math::Quaternion rot = base_pose.rot;
          
          // Set Ravelin Pose
          Ravelin::Pose3d robot_pose(Ravelin::Quatd(rot.x,rot.y,rot.z,rot.w),Ravelin::Origin3d(pos.x,pos.y,pos.z));
          base_q = Utility::pose_to_vec(robot_pose);
          
          
          math::Vector3    vel = base_ptr->GetWorldLinearVel();
          math::Vector3   avel = base_ptr->GetWorldAngularVel();
          
          base_qd.set_sub_vec(0  ,Ravelin::Vector3d(vel.x,vel.y,vel.z) );
          base_qd.set_sub_vec(3,Ravelin::Vector3d(avel.x,avel.y,avel.z) );
          
          math::Vector3  force = base_ptr->GetWorldForce();
          math::Vector3 torque = base_ptr->GetWorldTorque();
          
          base_fext.set_sub_vec(0  , Ravelin::Vector3d(force.x,force.y,force.z) );
          base_fext.set_sub_vec(3,Ravelin::Vector3d(torque.x,torque.y,torque.z));
          
          math::Vector3  accel = base_ptr->GetWorldLinearAccel();
          math::Vector3 aaccel = base_ptr->GetWorldAngularAccel();
          
          // Get base accelerations
          base_qdd.set_sub_vec(0,Ravelin::Vector3d(accel.x,accel.y,accel.z));
          base_qdd.set_sub_vec(3,Ravelin::Vector3d(aaccel.x,aaccel.y,aaccel.z));
          
          robot_ptr->set_base_value(Pacer::Robot::position,base_q);
          robot_ptr->set_base_value(Pacer::Robot::velocity,base_qd);
          robot_ptr->set_base_value(Pacer::Robot::acceleration,base_qdd);
          //robot_ptr->set_base_value(Pacer::Robot::load,base_fext);
          
          // Set joint params in Moby joint coordinates
          std::map<std::string,Ravelin::VectorNd>
          joint_q,joint_qd,joint_qdd,joint_fext;
          
          robot_ptr->get_joint_value(Pacer::Robot::position,joint_q);
          robot_ptr->get_joint_value(Pacer::Robot::velocity,joint_qd);
          robot_ptr->get_joint_value(Pacer::Robot::acceleration,joint_qdd);
          robot_ptr->get_joint_value(Pacer::Robot::load,joint_fext);
          
          static std::map<std::string,Ravelin::VectorNd>
          last_joint_qd = joint_qd;
          
          for(int i=0;i<joints.size();i++){
            physics::JointPtr joint = joints[i];
            int JDOFS = robot_ptr->get_joint_dofs(joints[i]->GetName());
            for(int j=0;j<JDOFS;j++){
              joint_q[joint->GetName()][j]    = joint->GetAngle(j).Radian();
              joint_qd[joint->GetName()][j]   = joint->GetVelocity(j);
              joint_qdd[joint->GetName()][j]
              = (joint_qd[joint->GetName()][j] - last_joint_qd[joint->GetName()][j])/dt;
              //joint_fext[joint->GetName()][j] = joint->GetForce(j);
            }
          }
          OUTLOG(last_joint_qd,"last_joint_qd",logERROR);
          OUTLOG(joint_qd,"joint_qd",logERROR);
          OUTLOG(joint_qdd,"joint_qdd",logERROR);
          OUTLOG(joint_q,"joint_q",logERROR);
          
          last_joint_qd = joint_qd;
          
          robot_ptr->set_joint_value(Pacer::Robot::position,joint_q);
          robot_ptr->set_joint_value(Pacer::Robot::velocity,joint_qd);
          robot_ptr->set_joint_value(Pacer::Robot::acceleration,joint_qdd);
          //robot_ptr->set_joint_value(Pacer::Robot::load,joint_fext);
        }
        
        robot_ptr->control(t);
      }
      
      {
        std::map<std::string, Ravelin::VectorNd > q, qd, u;
//        robot_ptr->get_joint_value(Pacer::Robot::position_goal, q);
//        robot_ptr->get_joint_value(Pacer::Robot::velocity_goal, qd);
        robot_ptr->get_joint_value(Pacer::Robot::load_goal, u);
        
        int is_kinematic = robot_ptr->get_data<int>("init.kinematic");
        
        //  apply_sim_perturbations();
        for(int i=0;i<joints.size();i++){
          physics::JointPtr joint = joints[i];
          int JDOFS = robot_ptr->get_joint_dofs(joints[i]->GetName());
          for(int j=0;j<JDOFS;j++){
            //if(!is_kinematic){
            joint->SetForce(j,u[joint->GetName()][j]);
            //} else {
            //  math::Angle a(q[joints[i]->GetName()][j]);
            //  joint->SetAngle(j,a);
            //  joint->SetVelocity(j,qd[joint->GetName()][j]);
            //}
          }
        }
      }
      OUT_LOG(logERROR) << "<< end Plugin: Update(.)";
    }
  };
  
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ControllerPlugin)
}
