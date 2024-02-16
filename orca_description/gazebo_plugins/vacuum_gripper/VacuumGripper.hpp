//
// Created by ocadovski on 2/10/24.
//

#pragma once

#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>

namespace gz::sim::systems
{

class VacuumGripperPrivate;

class VacuumGripper final : public System, public ISystemConfigure, public ISystemUpdate
{
public:
  /// \brief Constructor.
public:
  VacuumGripper();

  /// \brief Destructor.
public:
  ~VacuumGripper() final = default;

  void Configure(const Entity& _entity, const std::shared_ptr<const sdf::Element>& _sdf, EntityComponentManager& _ecm,
                 EventManager& /*_eventMgr*/) override;

  void Update(const UpdateInfo& /*_info*/, EntityComponentManager& _ecm) override;

private:
  /// Recommended PIMPL pattern. This variable should hold all private
  /// data members.
  std::unique_ptr<VacuumGripperPrivate> dataPtr;
};

}  // namespace gz::sim::systems
