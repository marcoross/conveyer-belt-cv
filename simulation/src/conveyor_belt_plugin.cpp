#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <thread>
#include <ros/callback_queue.h>

namespace gazebo
{
  class ConveyorBeltPlugin : public ModelPlugin
  {
  public:
    ConveyorBeltPlugin() : ModelPlugin() {}

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      this->model = _model;

      if (_sdf->HasElement("belt_speed"))
        this->beltSpeed = _sdf->Get<double>("belt_speed");

      // Initialize ROS node
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = nullptr;
        ros::init(argc, argv, "conveyor_belt_plugin", ros::init_options::NoSigintHandler);
      }

      this->rosNode.reset(new ros::NodeHandle("conveyor_belt_plugin"));

      // Subscribe to the /conveyor_belt/cmd_vel topic
      ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Twist>(
          "/conveyor_belt/cmd_vel",
          1,
          boost::bind(&ConveyorBeltPlugin::OnRosMsg, this, _1),
          ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);

      // Spin up the queue helper thread
      this->rosQueueThread = std::thread(std::bind(&ConveyorBeltPlugin::QueueThread, this));

      // Connect to the contact sensor
      this->contactSensor = std::dynamic_pointer_cast<sensors::ContactSensor>(
          sensors::SensorManager::Instance()->GetSensor("contact_sensor"));
      if (this->contactSensor)
      {
        this->contactSensor->SetActive(true);
        this->updateConnection = this->contactSensor->ConnectUpdated(
            std::bind(&ConveyorBeltPlugin::OnContact, this));
      } else {
        ROS_ERROR_STREAM("Contact sensor not found");
        /*
        auto sensorManager = sensors::SensorManager::Instance();
        auto sensors = sensorManager->GetSensors();
        ROS_INFO_STREAM("Listing available sensors:");
        for (auto sensor : sensors)
        {
          ROS_INFO_STREAM("Sensor name: " << sensor->Name() << ", Type: " << sensor->Type());
        }
        */
      }
    }

    

    void OnContact()
    {
      // ROS_INFO_STREAM("Contact detected!");
      // Get the contacts
      auto contacts = this->contactSensor->Contacts();
      for (unsigned int i = 0; i < contacts.contact_size(); ++i)
      {
        for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j)
        {
          // Log the collision names
          // ROS_INFO_STREAM("Collision1: " << contacts.contact(i).collision1());
          // ROS_INFO_STREAM("Collision2: " << contacts.contact(i).collision2());

          // Determine which collision is the conveyor belt and which is the object
          std::string collision1 = contacts.contact(i).collision1();
          std::string collision2 = contacts.contact(i).collision2();
          std::string objectCollision = (collision1.find("conveyor_belt") != std::string::npos) ? collision2 : collision1;

          // Apply force to the parent link of the collision object
          auto collisionEntity = this->model->GetWorld()->EntityByName(objectCollision);
          if (collisionEntity)
          {
            auto collision = boost::dynamic_pointer_cast<physics::Collision>(collisionEntity);
            if (collision)
            {
              auto parentLink = collision->GetLink();
              if (parentLink)
              {
                // ROS_INFO_STREAM("Applying force to link: " << parentLink->GetName());
                auto objectVelocity = parentLink->WorldLinearVel();
                auto relativeSpeed = this->beltSpeed - objectVelocity.X();
                auto frictionCoefficient = 0.5; // Example friction coefficient
                auto force = relativeSpeed * frictionCoefficient;
                //ROS_INFO_STREAM("Applying force: " << force << " to link: " << parentLink->GetName());
                parentLink->AddForce(ignition::math::Vector3d(force, 0, 0));
              }
              else
              {
                // ROS_ERROR_STREAM("Parent link not found for collision: " << objectCollision);
              }
            }
            else
            {
              // ROS_ERROR_STREAM("Collision entity is not a physics::Collision");
            }
          }
          else
          {
            // ROS_ERROR_STREAM("Collision entity not found for collision: " << objectCollision);
          }
        }
      }
    }

    void OnRosMsg(const geometry_msgs::Twist::ConstPtr &msg)
    {
      this->beltSpeed = msg->linear.x;
      // ROS_INFO_STREAM("Received belt speed: " << this->beltSpeed);
    }

    void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

  private:
    physics::ModelPtr model;
    sensors::ContactSensorPtr contactSensor;
    event::ConnectionPtr updateConnection;
    double beltSpeed = 0.05;  // 5 centimeters per secondq

    std::unique_ptr<ros::NodeHandle> rosNode;
    ros::Subscriber rosSub;
    ros::CallbackQueue rosQueue;
    std::thread rosQueueThread;
  };

  GZ_REGISTER_MODEL_PLUGIN(ConveyorBeltPlugin)
}