/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#include <Moby/ArticulatedBody.h>
#include <Moby/ConstraintSimulator.h>

// pointer to the simulator
 boost::shared_ptr<Moby::Simulator> sim;
// pointer to the articulated body in Moby
 boost::shared_ptr<Moby::ArticulatedBody> abrobot;

// implements a controller callback for Moby
void controller_callback(boost::shared_ptr<Moby::ControlledBody> dbp, double t, void*)
{

}

// ============================================================================
// ================================ CALLBACKS =================================
// ============================================================================

// examines contact events (after they have been handled in Moby)
void post_event_callback_fn(const std::vector<Moby::UnilateralConstraint>& e,
                            boost::shared_ptr<void> empty)
{
  Ravelin::VectorNd generalized_q,generalized_qd,generalized_qdd, generalized_fext;
  
  {
    abrobot->get_generalized_coordinates(Ravelin::DynamicBodyd::eEuler,generalized_q);
    abrobot->get_generalized_velocity(Ravelin::DynamicBodyd::eSpatial,generalized_qd);
    abrobot->get_generalized_forces(generalized_fext);
  }
  
  std::cout << "generalized_q (post-contact) " << generalized_q << std::endl;
  std::cout << "generalized_qd (post-contact) " << generalized_qd << std::endl;

}

boost::shared_ptr<Moby::ContactParameters> get_contact_parameters(Moby::CollisionGeometryPtr geom1, Moby::CollisionGeometryPtr geom2){

  boost::shared_ptr<Moby::ContactParameters> e = boost::shared_ptr<Moby::ContactParameters>(new Moby::ContactParameters());
  return e;
}

// hooks into Moby's post integration step callback function
void post_step_callback_fn(Moby::Simulator* s){}

/// Event callback function for setting friction vars pre-event
void pre_event_callback_fn(std::vector<Moby::UnilateralConstraint>& e, boost::shared_ptr<void> empty){
}

// ============================================================================
// ================================ INIT ======================================
// ============================================================================

//boost::shared_ptr<Moby::ContactParameters> get_contact_parameters(Moby::CollisionGeometryPtr geom1, Moby::CollisionGeometryPtr geom2);
//void post_event_callback_fn(const std::vector<Moby::UnilateralConstraint>& e, boost::shared_ptr<void> empty);
//void pre_event_callback_fn(std::vector<Moby::UnilateralConstraint>& e, boost::shared_ptr<void> empty);

/// plugin must be "extern C"

// this is called by Moby for a plugin
void init_cpp(const std::map<std::string, Moby::BasePtr>& read_map, double time){
  std::cout << "STARTING MOBY PLUGIN" << std::endl;
  
  // If use robot is active also init dynamixel controllers
  // get a reference to the Simulator instance
  for (std::map<std::string, Moby::BasePtr>::const_iterator i = read_map.begin();
       i !=read_map.end(); i++)
  {
    // Find the simulator reference
    
    if (!sim)
      sim = boost::dynamic_pointer_cast<Moby::Simulator>(i->second);

    // find the robot reference
    if (!abrobot)
    {
      abrobot = boost::dynamic_pointer_cast<Moby::ArticulatedBody>(i->second);
    }
  }
  
  if(!abrobot)
    throw std::runtime_error("Could not find robot in simulator!");
  
  boost::shared_ptr<Moby::ConstraintSimulator> csim;

  csim = boost::dynamic_pointer_cast<Moby::ConstraintSimulator>(sim);

  std::cout << "STARTING ROBOT" << std::endl;

  //robot_ptr = boost::shared_ptr<Controller>(new Controller());
  //robot_ptr->init();
  
  std::cout << "ROBOT INITED" << std::endl;

  // CONTACT PARAMETER CALLBACK (MUST BE SET)
 //sim->get_contact_parameters_callback_fn = &get_contact_parameters;
  // CONTACT CALLBACK
  if (csim){
    //  sim->constraint_callback_fn             = &pre_event_callback_fn;
    csim->constraint_post_callback_fn        = &post_event_callback_fn;
  }
  // CONTROLLER CALLBACK
  abrobot->controller                     = &controller_callback;

}

// plugins must be declared 'extern "C"'
extern "C" {

void init(void* separator, const std::map<std::string, Moby::BasePtr>& read_map, double time)
{
  init_cpp(read_map,time);
}
} // end extern C
