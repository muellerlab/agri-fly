#pragma once

#include <vector>

#include "Common/Math/Vec3.hpp"
#include "Components/Simulation/SimulationObject.hpp"

namespace Simulation {

/* Simulate a single UWB radio. Not very sophisticated at the moment, but
 * should do a reasonable job of emulating what happens on the hardware.
 *
 * TODO: would be nice to fully implement the communication protocall,
 * where a ranging consist of multiple messages being passed back and
 * forth.
 */
class UWBRadio : public SimulationObject {
 public:
  struct RangingMeasurement {
    bool haveNew;
    float range;
    uint8_t responderId;
    bool failure;
  };

  UWBRadio(BaseTimer* const timer, uint8_t myId)
      : SimulationObject(timer),
        _myUWBId(myId),
        _nextUWBRangingTargetId(0),
        _uwbTruePosition() {

    _meas.haveNew = false;
  }

  virtual ~UWBRadio() {
    //nothing here
  }

  virtual void Run() {
    //nothing here
  }

  void SetPosition(Vec3d const in) {
    _uwbTruePosition = in;
  }

//  void SetPositionEstimate(Vec3f const pos, float const posCov[6]) {
//    _uwbEstPosition = pos;
//    for (int i = 0; i < 6; i++) {
//      _uwbEstPositionCov[i] = posCov[i];
//    }
//  }

  void SetNextRangingTarget(uint8_t id) {
    _nextUWBRangingTargetId = id;
  }

  Vec3d GetPosition() const {
    return _uwbTruePosition;
  }

  void SetMeasurement(RangingMeasurement meas) {
    _meas = meas;
    _meas.haveNew = true;
  }

  bool GetHaveNewMeasurement(void) {
    return _meas.haveNew;
  }

  RangingMeasurement GetMeasurement(void) {
    RangingMeasurement outMeas = _meas;
    _meas.haveNew = false;
    return outMeas;
  }

  uint8_t GetId() const {
    return _myUWBId;
  }

  uint8_t GetNextRangingTargetId() const {
    return _nextUWBRangingTargetId;
  }

 private:
  uint8_t _myUWBId;
  uint8_t _nextUWBRangingTargetId;
  Vec3d _uwbTruePosition;  //where the radio really is
  //TODO: add this
//  Vec3f _uwbEstPosition;  //where the radio thinks it is
//  float _uwbEstPositionCov[6];  //what the radio thinks its error is

  RangingMeasurement _meas;
};

}  //namespace Simulation
