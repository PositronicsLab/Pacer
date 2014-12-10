#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>

namespace gazebo
{
  class ContactPlugin : public SensorPlugin
  {

    /////////////////////////////////////////////////
    public: ContactPlugin() : SensorPlugin()
    {
    }

    /////////////////////////////////////////////////
    public: ~ContactPlugin()
    {
    }

    /////////////////////////////////////////////////
    public: void Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
    {
      // Get the parent sensor.
      this->parentSensor =
        boost::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

      // Make sure the parent sensor is valid.
      if (!this->parentSensor)
      {
        gzerr << "ContactPlugin requires a ContactSensor.\n";
        return;
      }

      // Connect to the sensor update event.
      this->updateConnection = this->parentSensor->ConnectUpdated(
          boost::bind(&ContactPlugin::OnUpdate, this));

      // Make sure the parent sensor is active.
      this->parentSensor->SetActive(true);
    }

    /////////////////////////////////////////////////
    public: void OnUpdate()
    {
      // Get all the contacts.
      msgs::Contacts contacts;
      contacts = this->parentSensor->GetContacts();
      for (unsigned int i = 0; i < contacts.contact_size(); ++i)
      {
        std::cout << "Collision between[" << contacts.contact(i).collision1()
                  << "] and [" << contacts.contact(i).collision2() << "]\n";

        for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j)
        {
          std::cout << j << "  Position:"
                    << contacts.contact(i).position(j).x() << " "
                    << contacts.contact(i).position(j).y() << " "
                    << contacts.contact(i).position(j).z() << "\n";
          std::cout << "   Normal:"
                    << contacts.contact(i).normal(j).x() << " "
                    << contacts.contact(i).normal(j).y() << " "
                    << contacts.contact(i).normal(j).z() << "\n";
          std::cout << "   Depth:" << contacts.contact(i).depth(j) << "\n";
        }
      }
    }


    /// \brief Pointer to the contact sensor
    private: sensors::ContactSensorPtr parentSensor;

    /// \brief Connection that maintains a link between the contact sensor's
    /// updated signal and the OnUpdate callback.
    private: event::ConnectionPtr updateConnection;
};
  GZ_REGISTER_SENSOR_PLUGIN(ContactPlugin)
}
