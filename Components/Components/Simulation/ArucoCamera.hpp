#pragma once

#include "Common/Math/Vec3.hpp"
#include "Common/Math/Rotation.hpp"
#include "Components/Simulation/SimulationObject.hpp"

namespace Simulation {

class ArucoCamera : public SimulationObject {
 public:

  ArucoCamera(BaseTimer* const masterTimer, double fakeRunTime);
  // Fake run time is to simulate how fast or slow the camera generates new images.
  virtual ~ArucoCamera() {
  }

  void SetPosition(Vec3d in) {
    _position = in;
  }
  void SetAttitude(Rotationd in) {
    _attitude = in;
  }

  Vec3d GetPositionMeasurement() const {
    return _arucoPosMeas;
  }
  Rotationd GetAttitudeMeasurement() const {
    return _arucoAttMeas;
  }
  bool IsNewMeasurement() const {
    return _isNewMeas;
  }

  virtual void Run();

 private:
  Vec3d _position;  // (s_CE_E) position of camera (in earth frame)
  Rotationd _attitude;  // (R_EC) vector in world frame = _attitude * vector in camera frame
  Vec3d _arucoPosMeas;  // Relative position measurement from the camera
  Rotationd _arucoAttMeas;  // Relative attitude measurement from the camera
  bool _isNewMeas;  // Is a new measurement generated?
  double _fakeRunTime;  // Time it takes to generate a single image (or data from a single image)
};

}  //namespace Simulation
