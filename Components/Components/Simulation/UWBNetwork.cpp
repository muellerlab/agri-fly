#include "UWBNetwork.hpp"
#include <random>

std::mt19937 rng;
std::uniform_real_distribution<double_t> distUniform(0, 1);
std::normal_distribution<double> distNormal(0, 1);

using namespace Simulation;

UWBNetwork::UWBNetwork(BaseTimer* const masterTimer, double communicationPeriod)
    : SimulationObject(masterTimer),
      _commPeriod(communicationPeriod),
      _timeSinceLastRange(masterTimer),
      _currentRangingRequester(0),
      _currentRangingResponder(0),
      _addNoiseStdDev(0),
      _outlierProbability(0),
      _outlierStdDev(0) {
  rng.seed(0);  //to be repeatible.
}

void UWBNetwork::Run() {
  //Run individual radios:
  for (auto radio = _radios.begin(); radio != _radios.end(); radio++) {
    (*radio)->Run();
  }

  if (_timeSinceLastRange.GetSeconds<double>() < _commPeriod) {
    return;
  }

  if (!_currentRangingRequester || !_currentRangingResponder) {
    //don't have two active parties, see if anyone wants to initiate range:
    for (auto radio = _radios.begin(); radio != _radios.end(); radio++) {
      if ((*radio)->GetNextRangingTargetId()) {
        //start a new ranging:
        _currentRangingRequester = (*radio)->GetId();
        _currentRangingResponder = (*radio)->GetNextRangingTargetId();
        break;
      }
    }
    //Generate a new measurement
    _timeSinceLastRange.Reset();
    return;
  }

  //We have a ranging transaction that should be completed:

  //ugly way of finding the two involved radios...
  Vec3d reqTruePos, resTruePos;  //true position  of the two radios
  bool haveRequester = false;
  bool haveResponder = false;
  UWBRadio::RangingMeasurement meas;
  for (auto radio = _radios.begin(); radio != _radios.end(); radio++) {
    if ((*radio)->GetId() == _currentRangingRequester) {
      reqTruePos = (*radio)->GetPosition();
      haveRequester = true;
    }
    if ((*radio)->GetId() == _currentRangingResponder) {
      resTruePos = (*radio)->GetPosition();
      haveResponder = true;
    }
  }

  if (haveRequester && haveResponder) {

    //Is this an outlier?
    if (distUniform(rng) < _outlierProbability) {
      meas.range = distNormal(rng) * _outlierStdDev;
    } else {
      double measNoise = distNormal(rng) * _addNoiseStdDev;
      meas.range = (reqTruePos - resTruePos).GetNorm2() + measNoise;
    }

    meas.haveNew = true;
    meas.responderId = _currentRangingResponder;
    meas.failure = false;//todo: we want to fail with the same probability as in real life.

    for (auto radio = _radios.begin(); radio != _radios.end(); radio++) {
      //everyone "hears" the measurement
      (*radio)->SetMeasurement(meas);
    }
  } else {
    assert(0);
  }

  _currentRangingRequester = 0;
  _currentRangingResponder = 0;
}
