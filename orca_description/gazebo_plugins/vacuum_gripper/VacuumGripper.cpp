//
// Created by ocadovski on 2/10/24.
//
#include <gz/common/Console.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/transport/Node.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Pose.hh>

// #include <gz/common/Profiler.hh>

#include "VacuumGripper.hpp"

// Register plugin
GZ_ADD_PLUGIN(gz::sim::systems::VacuumGripper, gz::sim::System, gz::sim::systems::VacuumGripper::ISystemConfigure,
              gz::sim::systems::VacuumGripper::ISystemUpdate)

// Add plugin alias so that we can refer to the plugin without the version
// namespace
GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::VacuumGripper, "VacuumGripperPlugin")

class gz::sim::systems::VacuumGripperPrivate
{
  /// \brief The model
public:
  msgs::Boolean updateGripperForce(EntityComponentManager& _ecm);

  /// \brief Callback for enable/disable subscription
  /// \param[in] _msg Boolean message
  void onEnable(const msgs::Boolean& _msg);

  gz::sim::Model model{ gz::sim::kNullEntity };

  /// \brief The model name;
  std::string modelName;

  /// \brief The world
  gz::sim::World world{ gz::sim::kNullEntity };

  /// \brief The world name;
  std::string worldName;

  /// \brief for setting ROS name space
  std::string robot_namespace;

  /// \brief The entity representing link this plugin is attached to
  gz::sim::Entity link{ gz::sim::kNullEntity };

  /// \brief The name of Link this plugin is attached to, and will exert forces on.
  std::string linkName;

  /// \brief ROS Wrench topic name inputs
  std::string topicName;

  /// \brief Gazebo communication node.
  transport::Node node;

  /// \brief Enable/disable state of the controller.
  bool enabled{ false };

  /// \brief Mutex for protecting access to enabled state.
  std::mutex mutex;

  /// \brief Grasping message publisher
  transport::Node::Publisher graspingPub;
};

gz::sim::systems::VacuumGripper::VacuumGripper()
{
  dataPtr = std::make_unique<VacuumGripperPrivate>();
}

void gz::sim::systems::VacuumGripper::Configure(const gz::sim::Entity& _entity,
                                                const std::shared_ptr<const sdf::Element>& _sdf,
                                                gz::sim::EntityComponentManager& _ecm, gz::sim::EventManager&)
{
  gzlog << "Inside VacuumGripper::Configure" << "\n";

  this->dataPtr->model = gz::sim::Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "VacuumGripperPlugin should be attached to a model "
          << "entity. Failed to initialize."
          << "\n";
    return;
  }

  this->dataPtr->modelName = this->dataPtr->model.Name(_ecm);

  this->dataPtr->world = gz::sim::World(_ecm.EntityByComponents(components::World()));
  if (!this->dataPtr->world.Valid(_ecm))
  {
    gzerr << "World entity not found" << std::endl;
    return;
  }

  if (this->dataPtr->world.Name(_ecm).has_value())
  {
    this->dataPtr->worldName = this->dataPtr->world.Name(_ecm).value();
  }

  this->dataPtr->robot_namespace = "";
  if (_sdf->HasElement("robotNamespace"))
  {
    this->dataPtr->robot_namespace = _sdf->Get<std::string>("robotNamespace");
  }

  if (!_sdf->HasElement("bodyName"))
  {
    gzerr << "vacuum_gripper plugin missing <bodyName>, cannot proceed" << std::endl;
    return;
  }
  else
  {
    this->dataPtr->linkName = _sdf->Get<std::string>("bodyName");
  }

  this->dataPtr->link = this->dataPtr->model.LinkByName(_ecm, this->dataPtr->linkName);
  if (this->dataPtr->link == gz::sim::kNullEntity)
  {
    std::string found;
    auto links = this->dataPtr->model.Links(_ecm);
    for (const auto& link : links)
    {
      auto nameComp = _ecm.Component<gz::sim::components::Name>(link);
      if (nameComp)
      {
        found += " " + nameComp->Data();
      }
    }

    gzerr << "gazebo_ros_vacuum_gripper plugin error: link named: " << this->dataPtr->linkName << " does not exist\n";
    gzerr << "gazebo_ros_vacuum_gripper plugin error: You should check it exists and is not connected with fixed "
             "joint\n";
    gzerr << "gazebo_ros_vacuum_gripper plugin error: Found links are: " << found << "\n";
  }

  if (!_sdf->HasElement("topicName"))
  {
    gzerr << "Vacuum gripper plugin missing <topicName>, cannot proceed." << std::endl;
    return;
  }
  else
  {
    this->dataPtr->topicName = _sdf->Get<std::string>("topicName");
  }

  // Subscribe to enable/disable
  std::vector<std::string> enableTopics;
  enableTopics.push_back("/model/" + this->dataPtr->model.Name(_ecm) + "/enable");

  if (auto enableTopic = validTopic(enableTopics); !enableTopic.empty())
  {
    this->dataPtr->node.Subscribe(enableTopic, &VacuumGripperPrivate::onEnable, this->dataPtr.get());
  }
  this->dataPtr->enabled = true;

  this->dataPtr->graspingPub = this->dataPtr->node.Advertise<msgs::Boolean>(this->dataPtr->topicName);
}

void gz::sim::systems::VacuumGripper::Update(const gz::sim::UpdateInfo&, gz::sim::EntityComponentManager& _ecm)
{
  msgs::Boolean graspingMsg;
  graspingMsg.set_data(false);
  if (this->dataPtr->enabled)
  {
    graspingMsg = this->dataPtr->updateGripperForce(_ecm);
  }

  this->dataPtr->graspingPub.Publish(graspingMsg);
}

gz::msgs::Boolean gz::sim::systems::VacuumGripperPrivate::updateGripperForce(EntityComponentManager& _ecm)
{
  msgs::Boolean graspingMsg;
  graspingMsg.set_data(false);

  [[maybe_unused]] std::scoped_lock lock(mutex);

  gz::math::Pose3d parent_pose = worldPose(link, _ecm);

  auto models = world.Models(_ecm);

  double max_force = 20;  // Maximum force limit

  for (const auto& modelIt : models)
  {
    auto modelInstance = gz::sim::Model(modelIt);
    if (auto linkModel = gz::sim::Link(link);
        modelInstance.Name(_ecm) == linkModel.Name(_ecm) || modelInstance.Name(_ecm) == model.Name(_ecm))
    {
      continue;
    }

    auto links = modelInstance.Links(_ecm);
    for (const auto& linkIt : links)
    {
      auto linkInstance = gz::sim::Link(linkIt);

      gz::math::Pose3d link_pose = linkInstance.WorldPose(_ecm).value();
      gz::math::Pose3d diff = link_pose.Inverse() * parent_pose;  // Instead of writing 'parent_pose - link_pose'
                                                                  // because minus operator is deprecated
      double distance = diff.Pos().Length();
      if (distance < 0.05)
      {
        linkInstance.SetLinearVelocity(_ecm, linkInstance.WorldLinearVelocity(_ecm).value());
        linkInstance.SetAngularVelocity(_ecm, linkInstance.WorldAngularVelocity(_ecm).value());
        double force_strength = 1 / distance;
        if (distance < 0.01)
        {
          // Apply friction-like force
          // TODO: Apply friction
          link_pose.Set(parent_pose.Pos(), link_pose.Rot());

          _ecm.Component<components::Pose>(linkIt)->Data() =
              link_pose;  // TODO: Check if this is equal to SetWorldPose from Gazebo
                          // Classic - which was done like this: linkInstance.SetWorldPose(link_pose, _ecm);
                          //          modelInstance.SetWorldPoseCmd(_ecm, link_pose);
        }
        if (force_strength > max_force)
        {
          force_strength = max_force;
        }
        gz::math::Vector3d force = force_strength * diff.Pos().Normalize();
        linkInstance.AddWorldForce(_ecm, force);

        graspingMsg.set_data(true);
      }
    }
  }

  return graspingMsg;
}

void gz::sim::systems::VacuumGripperPrivate::onEnable(const msgs::Boolean& _msg)
{
  bool oldState;

  auto changeState = [this](const msgs::Boolean& msg) {
    [[maybe_unused]] std::scoped_lock lock(this->mutex);
    bool old = this->enabled;
    this->enabled = msg.data();
    return old;
  };

  oldState = changeState(_msg);

  // Check if the state is changing
  if (oldState != _msg.data())
  {
    // Log the transition
    gzmsg << "VacuumGripper: Transitioning from " << (oldState ? "Enabled" : "Disabled") << " to "
          << (_msg.data() ? "Enabled" : "Disabled") << std::endl;
  }
}
