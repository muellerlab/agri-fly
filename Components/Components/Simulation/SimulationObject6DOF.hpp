#pragma once
#include <fstream>

#include "Common/DataTypes/RadioTypes.hpp"
#include "Components/Simulation/SimulationObject.hpp"
#include "Components/Simulation/UWBRadio.hpp"
#include "Common/DataTypes/TelemetryPacket.hpp"

namespace Simulation {

//A class that all simulated objects *must* inherit, to make the simulator's life easier.
class SimulationObject6DOF : public SimulationObject {
 public:
  SimulationObject6DOF(BaseTimer* const timer)
      : SimulationObject(timer),
        _pos(0, 0, 0),
        _vel(0, 0, 0),
        _att(Rotationd::Identity()),
        _angVel(0, 0, 0) {
  }
  virtual ~SimulationObject6DOF() {
  }

  virtual void Run() = 0;

  Vec3d GetPosition() const {
    return _pos;
  }

  Vec3d GetVelocity() const {
    return _vel;
  }

  Rotationd GetAttitude() const {
    return _att;
  }

  Vec3d GetAngularVelocity() const {
    return _angVel;
  }

  void SetPosition(Vec3d in) {
    _pos = in;
  }

  void SetVelocity(Vec3d in) {
    _vel = in;
  }

  void SetAttitude(Rotationd in) {
    _att = in;
  }

  void SetAngularVelocity(Vec3d in) {
    _angVel = in;
  }

  virtual std::shared_ptr<UWBRadio> GetRadio() {
    return _radio;
  }

  virtual void AddUWBRadioTarget(uint8_t id, Vec3f pos) = 0;

  virtual void SetCommandRadioMsg(
      RadioTypes::RadioMessageDecoded::RawMessage const raw) = 0;

  virtual void GetTelemetryDataPackets(
      TelemetryPacket::data_packet_t &dataPacket1,
      TelemetryPacket::data_packet_t &dataPacket2) = 0;

 protected:

  //vehicle state:
  Vec3d _pos;
  Vec3d _vel;
  Rotationd _att;  //<vector in world frame> = _att*<vector in body frame>
  Vec3d _angVel;

  std::shared_ptr<UWBRadio> _radio;

}
;
}
