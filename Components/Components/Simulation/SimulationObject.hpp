#pragma once

#include <Common/Math/Vec3.hpp>
#include <Common/Math/Rotation.hpp>
#include <Common/Time/Timer.hpp>

namespace Simulation {

//A class that all simulated objects *must* inherit, to make the simulator's life easier.
class SimulationObject {
 public:
  SimulationObject(BaseTimer* const timer)
      : _integrationTimer(timer) {
  }
  virtual ~SimulationObject() {
  }

  virtual void Run() = 0;

 protected:
  Timer _integrationTimer;  //Use this to keep track of time since last integration
};
}
