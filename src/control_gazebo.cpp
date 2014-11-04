/*****************************************************************************
 * Controller for LINKS robot
 ****************************************************************************/

#include <controller.h>

 boost::shared_ptr<Controller> robot_ptr;

 void init_plugin()
 {
   /// Set up quadruped robot, linking data from moby's articulated body
   /// to the quadruped model used by Control-Moby
   robot_ptr = boost::shared_ptr<Controller>(new Controller(std::string("Links")));

   // ================= INIT ROBOT STATE ==========================
   std::vector<Moby::JointPtr> joints_quad = robot_ptr->get_joints();
//   std::vector<Moby::JointPtr> joints = abrobot->get_joints();

   std::map<std::string,double> q0 = robot_ptr->get_q0();

   std::vector<double>
       workvd,
       base_start = Utility::get_variable("quadruped.init.base.x",workvd);

//   Ravelin::VectorNd q_start;
//   abrobot->get_generalized_coordinates(Moby::DynamicBody::eSpatial,q_start);
//   unsigned NDOFS = abrobot->num_generalized_coordinates(Moby::DynamicBody::eSpatial) - 6;

//   for(unsigned i=NDOFS;i<q_start.rows();i++)
//     q_start[i] = base_start[i-NDOFS];

//   abrobot->set_generalized_coordinates(Moby::DynamicBody::eSpatial,q_start);
//   abrobot->update_link_poses();
//   for(int i=0,ii=0;i<joints.size();i++){
//     for(int j=0;j<joints[i]->num_dof();j++,ii++){
//       joints[i]->q[j] = q0[std::to_string(j)+joints[i]->id];
//     }
//   }
//   abrobot->update_link_poses();
 }

 /*
  * Copyright (C) 2012-2014 Open Source Robotics Foundation
  *
  * Licensed under the Apache License, Version 2.0 (the "License");
  * you may not use this file except in compliance with the License.
  * You may obtain a copy of the License at
  *
  *     http://www.apache.org/licenses/LICENSE-2.0
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
 */

 #include <boost/bind.hpp>
 #include <gazebo/gazebo.hh>
 #include <gazebo/physics/physics.hh>
 #include <gazebo/common/common.hh>
 #include <stdio.h>

 namespace gazebo
 {
   class ControllerPlugin : public ModelPlugin
   {
     public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
     {
       // Store the pointer to the model
       this->model = _parent;
       this->updateConnection = event::Events::ConnectWorldUpdateBegin(
           boost::bind(&ControllerPlugin::OnUpdate, this, _1));

       physics::Joint_V joints = model->GetJoints();
       BOOST_FOREACH(physics::JointPtr j,joints)
       {
         OUT_LOG(logERROR) << j->GetId();
       }

       init_plugin();
     }

     // Called by the world update start event
     public: void OnUpdate(const common::UpdateInfo & /*_info*/)
     {
       physics::Joint_V joints = model->GetJoints();
       BOOST_FOREACH(physics::JointPtr j,joints)
       {
         OUT_LOG(logERROR) << j->GetId();
       }
       // Apply a small linear velocity to the model.
     }

     // Pointer to the model
     private: physics::ModelPtr model;

     // Pointer to the update event connection
     private: event::ConnectionPtr updateConnection;
     };

   // Register this plugin with the simulator
   GZ_REGISTER_MODEL_PLUGIN(ControllerPlugin);
 }
