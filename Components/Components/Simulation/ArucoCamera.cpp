#include "ArucoCamera.hpp"

using namespace Simulation;

ArucoCamera::ArucoCamera(BaseTimer* const masterTimer, double fakeRunTime)
    : SimulationObject(masterTimer),
      _position(Vec3d(0, 0, 0)),
      _attitude(Rotationd::Identity()),
      _arucoPosMeas(Vec3d(0, 0, 0)),
      _arucoAttMeas(Rotationd::Identity()),
      _isNewMeas(false),
      _fakeRunTime(fakeRunTime) {
}

void ArucoCamera::Run() {
  const double dt = _integrationTimer.GetSeconds<double>();
  _isNewMeas = false;
//  printf("Aruco cam time: %.3f s\n", dt);

  // Set a new measurement only after a certain amount of time
  if (dt >= _fakeRunTime) {
    _integrationTimer.Reset();
    _isNewMeas = true;
    // No coordinate transforms (this is just for a simple test)
    _arucoPosMeas = _position;
    _arucoAttMeas = _attitude;
  }

  return;
}
