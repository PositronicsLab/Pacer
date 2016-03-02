#include <Pacer/robot.h>

bool Pacer::Robot::is_end_effector(const std::string& id){
  std::map<std::string,boost::shared_ptr<Pacer::Robot::end_effector_t> >::iterator
  it = _id_end_effector_map.find(id);
  if(it != _id_end_effector_map.end())
    return true;
  return false;
}

std::vector<std::string> Pacer::Robot::get_end_effector_names(){
  std::vector<std::string> eefs;
  std::map<std::string,boost::shared_ptr<Pacer::Robot::end_effector_t> >::iterator
  it = _id_end_effector_map.begin();
  for(;it != _id_end_effector_map.end();it++)
    eefs.push_back((*it).first);
  return eefs;
}

void Pacer::Robot::add_contact(
                 std::string& id,
                 Ravelin::Vector3d point,
                 Ravelin::Vector3d normal,
                 Ravelin::Vector3d tangent,
                 Ravelin::Vector3d impulse ,
                 double mu_coulomb ,double mu_viscous ,double restitution , bool compliant )
{
  check_phase_internal(misc_sensor);
  _id_contacts_map[id].push_back(create_contact(id,point,normal, tangent, impulse,mu_coulomb,mu_viscous,restitution,compliant));
}

boost::shared_ptr<Pacer::Robot::contact_t> Pacer::Robot::create_contact(
                                            std::string& id,
                                            Ravelin::Vector3d point,
                                            Ravelin::Vector3d normal,
                                            Ravelin::Vector3d tangent,
                                            Ravelin::Vector3d impulse,
                                            double mu_coulomb ,double mu_viscous ,double restitution , bool compliant )
{
  boost::shared_ptr<Pacer::Robot::contact_t> c(new Pacer::Robot::contact_t);
  c->id         = id;
  c->point      = point;
  c->normal     = normal;
  c->tangent    = tangent;
  c->impulse    = impulse;
  c->mu_coulomb = mu_coulomb;
  c->mu_viscous = mu_viscous;
  c->restitution= restitution;
  c->compliant  = compliant;
  return c;
}

void Pacer::Robot::add_contact(boost::shared_ptr<Pacer::Robot::contact_t>& c)
{
  check_phase_internal(misc_sensor);
  _id_contacts_map[c->id].push_back(c);
}

void Pacer::Robot::get_contacts(std::map<std::string,std::vector< boost::shared_ptr<const Pacer::Robot::contact_t> > >& id_contacts_map){
  id_contacts_map.insert(_id_contacts_map.begin(), _id_contacts_map.end());
}

int Pacer::Robot::get_all_contacts(std::vector< boost::shared_ptr<const Pacer::Robot::contact_t> >& contacts){
  for(int i=0;i<_link_ids.size();i++){
    std::vector< boost::shared_ptr<const Pacer::Robot::contact_t> >& c = _id_contacts_map[_link_ids[i]];
    contacts.insert(contacts.end(), c.begin(), c.end());
  }
  return contacts.size();
}

int Pacer::Robot::get_link_contacts(const std::string& link_id, std::vector< boost::shared_ptr<const Pacer::Robot::contact_t> >& contacts){
  contacts = _id_contacts_map[link_id];
  return contacts.size();
}

void Pacer::Robot::get_link_contacts(const std::vector<std::string> link_id,std::vector< boost::shared_ptr<const Pacer::Robot::contact_t> >& contacts){
  for(int i=0;i<link_id.size();i++){
    std::vector< boost::shared_ptr<const Pacer::Robot::contact_t> >& c = _id_contacts_map[link_id[i]];
    contacts.insert(contacts.end(), c.begin(), c.end());
  }
}

void Pacer::Robot::reset_contact(){
  check_phase_internal(clean_up);
  _id_contacts_map.clear();
}