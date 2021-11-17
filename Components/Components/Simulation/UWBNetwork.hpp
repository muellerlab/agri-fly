#pragma once
#include <memory>
#include <vector>

#include "Common/Time/Timer.hpp"
#include "UWBRadio.hpp"
#include "Components/Simulation/SimulationObject.hpp"

namespace Simulation {

/* Simulates a network of UWB radios.
 *
 * Not super sophisticated, but allows to measure distances between
 * different radios.
 */
class UWBNetwork : public SimulationObject {
 public:

  UWBNetwork(BaseTimer* const masterTimer, double communicationPeriod);

  virtual ~UWBNetwork() {
  }

  void AddRadio(std::shared_ptr<UWBRadio> r) {
    _radios.push_back(r);
  }

  void SetNoiseProperties(double noiseStdDev, double outlierProbability,
                          double outlierStdDev) {
    _addNoiseStdDev = noiseStdDev;
    _outlierProbability = outlierProbability;
    _outlierStdDev = outlierStdDev;
  }

  virtual void Run();

 private:
  std::vector<std::shared_ptr<UWBRadio> > _radios;
  double _commPeriod;
  Timer _timeSinceLastRange;

  uint8_t _currentRangingRequester;
  uint8_t _currentRangingResponder;

  double _addNoiseStdDev;  //[m]
  double _outlierProbability;  //[1]
  double _outlierStdDev;  //[m]
};

}  //namespace Simulation
