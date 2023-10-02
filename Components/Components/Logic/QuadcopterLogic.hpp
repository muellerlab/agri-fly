#pragma once

#include "Common/Time/Timer.hpp"
#include "Common/DataTypes/TelemetryPacket.hpp"
#include "Common/DataTypes/RadioTypes.hpp"
#include "Common/Math/LowPassFilterFirstOrder.hpp"
#include "Common/Math/LowPassFilterSecondOrder.hpp"

#include "Components/Logic/QuadcopterPositionController.hpp"
#include "Components/Logic/QuadcopterAttitudeController.hpp"
#include "Components/Logic/QuadcopterAngularVelocityController.hpp"
#include "Components/Logic/QuadcopterMixer.hpp"
#include "Components/Logic/KalmanFilter6DOF.hpp"

#include "Components/Logic/QuadcopterConstants.hpp"
#include "Components/Logic/PanicReason.hpp"

namespace Onboard {

class QuadcopterLogic {
 public:
  QuadcopterLogic(BaseTimer* const timer, float onboardLogicPeriod);
  virtual ~QuadcopterLogic() {

  }

  void ResetCounters(void);
  void Initialise(QuadcopterConstants::QuadcopterType type, uint8_t vehId);

  void Run();

  void SetBatteryMeasurement(float voltage, float current) {
    _battMeas.isNew = true;
    _battMeas.count++;
    _battMeas.voltageRaw = voltage;
    _battMeas.currentRaw = current;
    _battMeas.voltageFiltered = _battVoltageLowPass.Apply(voltage);
  }

  void SetIMUMeasurementRateGyro(float x, float y, float z) {
    _imuRateGyro.isNew = true;
    _imuRateGyro.count++;
    _imuRateGyro.rawMeas = _R * Vec3f(x, y, z);
    _imuRateGyro.lowPass.Apply(_imuRateGyro.rawMeas - _gyroCalibrationBias);
  }

  void SetIMUMeasurementAccelerometer(float x, float y, float z) {
    _imuAccelerometer.isNew = true;
    _imuAccelerometer.count++;
    _imuAccelerometer.rawMeas = _R * Vec3f(x, y, z);
    _imuAccelerometer.lowPass.Apply(_imuAccelerometer.rawMeas);
  }

  void SetIMUMeasurementTemperature(float temperature) {
    _imuTemperature.isNew = true;
    _imuTemperature.count++;
    _imuTemperature.rawMeas = temperature;
    _imuTemperature.lowPass.Apply(_imuTemperature.rawMeas);
  }

  void SetUWBMeasurement(float const range, uint8_t const responderId,
                         bool const failure) {
    //TODO, nicer logic here
    _timeSinceLastUwb.Reset();
    _uwbRangeMeas.isNew = true;
    _uwbRangeMeas.targetId = responderId;
    _uwbRangeMeas.range = range;
    _uwbRangeMeas.failure = failure;
  }

  inline Vec3f GetAccelerometer(void) const {
    return _imuAccelerometer.lowPass.GetValue();
  }

  inline Vec3f GetRateGyro(void) const {
    return _imuRateGyro.lowPass.GetValue();
  }

  inline float GetIMUTemperature() const {
    return _imuTemperature.rawMeas;
  }

  inline float GetBatteryVoltage() const {
    return _battMeas.voltageRaw;

  }

  inline float GetBatteryCurrent() const {
    return _battMeas.currentRaw;

  }

  int GetCycleCounter() const {
    return _cycleCounter;
  }

  int GetUWBMeasurementCounter() const {
    return _uwbRangeMeas.count;
  }

  bool GetAreMotorsRunning() const {
    for (int i = 0; i < 4; i++) {
      if (_desMotorSpeeds[i] > 0) {
        return true;
      }
    }
    return false;
  }

  void SetRadioMessage(RadioTypes::RadioMessageDecoded const msg) {
    _radioMessage.isNew = true;
    _radioMessage.count++;
    _radioMessage.msg = msg;
    _timeSinceLastRadioMessage.Reset();
    _monitorCmdRate.Update();
  }

  void SetGyroCalibration(bool flag) {
    if (_gyroCalibrationEnabled && !flag) {
      //end of calibration: we disable the calibration
      if (_gyroCalibrationNumSamples) {
        _gyroCalibrationBias = _gyroCalibrationAccumulated
            / float(_gyroCalibrationNumSamples);
      } else {
        _gyroCalibrationBias = Vec3f(0, 0, 0);
      }
    }
    _gyroCalibrationEnabled = flag;
  }

  bool GetGyroCalibration() const {
    return _gyroCalibrationEnabled;
  }

  void SetGoAutonomous() {
    if (_state == FS_IDLE) {
      _state = FS_FULLY_AUTONOMOUS;
    }
  }

  void ResetGyroCalibration() {
    _gyroCalibrationEnabled = false;
    _gyroCalibrationNumSamples = 0;
    _gyroCalibrationAccumulated = Vec3f(0, 0, 0);
    _gyroCalibrationBias = Vec3f(0, 0, 0);
  }

  enum FlightState {
    //TODO: add an initial state to enable...
    FS_UNINITIALIZED,
    FS_IDLE,
    FS_FULLY_AUTONOMOUS,
    FS_PANIC,  //sink, motors always remain off
    FS_KILLED,  //sink, motors always remain off
    FS_EXTERNAL_ACCELERATION_CONTROL,
    FS_EXTERNAL_RATES_CONTROL,
  };

  FlightState GetFlightState(void) const {
    return _state;
  }

  void GetEstimate(Vec3f &pos, Vec3f &vel, Rotationf &att,
                   Vec3f &angVel) const {
    pos = _kf.GetPosition();
    vel = _kf.GetVelocity();
    att = _kf.GetAttitude();
    angVel = _kf.GetAngularVelocity();
  }

  float GetMotorSpeedCmd(unsigned i) const {
    assert(i < 4);
    return _desMotorSpeeds[i];
  }

  bool ShouldStartNewUWBConversation(void) const {
    return _shouldStartNewUWBConversation;
  }

  uint8_t GetNextUWBRangingTarget(void) const {
    if (!_numRangingTargets) {
      return 0;
    }
    return _rangingTargets[_nextRangingTargetIdx].id;
  }

  uint8_t GetCurrentUWBRangingTarget(void) const {
    return _uwbRangeMeas.targetId;
  }

  void GetTelemetryDataPackets(TelemetryPacket::data_packet_t &dataPacket1,
                               TelemetryPacket::data_packet_t &dataPacket2);

  bool GetAndResetShouldWriteParameters(void) {
    bool tmp = _shouldWriteParameters;
    _shouldWriteParameters = false;
    return tmp;
  }

  float GetPropellerCorrectionFactor(unsigned i) const {
    return _propellerCalibration.activeFactors[i];
  }

  void SetPropellerCorrectionFactor(float f0, float f1, float f2, float f3) {
    _propellerCalibration.activeFactors[0] = f0;
    _propellerCalibration.activeFactors[1] = f1;
    _propellerCalibration.activeFactors[2] = f2;
    _propellerCalibration.activeFactors[3] = f3;
    _mixer.SetPropellerCorrectionFactors(_propellerCalibration.activeFactors);
  }

  float GetDebugFloats(unsigned i) const {
    return _debug[i];
  }

  void UpdateRangingDiagnostics(unsigned id, float range, bool failure);

  void PrintStatus() const;

  void ClearRangingTargets() {
    _numRangingTargets = 0;
  }

  int AddRangingTargetId(uint8_t targetId, Vec3f targetPosition) {
    if (_numRangingTargets >= MAX_NUM_RANGING_TARGETS) {
      return -1;
    }
    _rangingTargets[_numRangingTargets].id = targetId;
    _rangingTargets[_numRangingTargets].position = targetPosition;
    _rangingTargets[_numRangingTargets].lastMeas = -1.0f;
    _rangingTargets[_numRangingTargets].numBadMeas = 0;
    _rangingTargets[_numRangingTargets].numGoodMeas = 0;
    _numRangingTargets++;
    return 0;
  }

  void TestMotors(bool in, float thrustFrac) {
    _testMotorsOn = in;
    _testMotorsThrustFrac = thrustFrac;
  }

  int GetLogicRunNumber() const {
    return _cycleCounter;
  }

  int GetFirstPanicReason() const {
    return _firstPanicReason;
  }

 protected:

  virtual void UpdateEstimator();
  virtual void ParseIncomingCommunications();
  virtual void UpdateWarnings();
  virtual void CheckPanicReasons();

  // Override these functions in the child logic class to implement different controllers
  virtual void RunControllerExternalAcceleration();
  virtual void RunControllerExternalRatesControl();
  virtual void RunControllerFullyAutonomous();

 private:

  int GetRangingTargetPosition(const uint8_t id, Vec3f &posOut);

  bool AreInSafetyCriticalFlightState() {  //Use this for safety checks, for example
    switch (_state) {
      case FS_UNINITIALIZED:
      case FS_IDLE:
      case FS_PANIC:
      case FS_KILLED:
        return false;
      default:
        return true;
    }
  }

  FlightState _state;

  Timer _timer;

  unsigned _cycleCounter;  //how many times has Run() been called?
  const float _onboardPeriod;  //expected time between calls to Run() [s]
  const float _radioCmdPeriod;  //expected time between radio commands [s]

  QuadcopterConstants::QuadcopterType _quadType;
  //controllers
  QuadcopterPositionController _posCtrl;
  QuadcopterAttitudeController _attCtr;
  QuadcopterAngularVelocityController _angVelCtrl;
  QuadcopterMixer _mixer;

  //state estimator
  KalmanFilter6DOF _kf;

  //desired position:
  Vec3f _desPos;

  float _desMotorSpeeds[4];  //[rad/s]
  float _desMotorForcesForTelemetry[4];  //[N]
  float _mass;  //[kg]

  float _IMU_yaw;  //[rad]
  float _IMU_pitch;
  float _IMU_roll;
  Matrix<float, 3, 3> _R;  // Rotation matrix corresponding to IMU's rotation

  struct {
    bool isNew;
    unsigned count;
    float voltageRaw;
    float currentRaw;

    float voltageFiltered;
  } _battMeas;

  template<typename T>
  struct RawMeasContainer {
    bool isNew;
    unsigned count;
    T rawMeas;
    LowPassFilterSecondOrder<float, T> lowPass;
  };

  RawMeasContainer<Vec3f> _imuRateGyro, _imuAccelerometer;  //imu measurements
  RawMeasContainer<float> _imuTemperature;

  struct {
    bool isNew;
    unsigned count;
    uint8_t targetId;
    float range;
    bool failure;
  } _uwbRangeMeas;
  Timer _timeSinceLastUwb;
  bool _shouldStartNewUWBConversation;

  struct {
    bool isNew;
    unsigned count;
    RadioTypes::RadioMessageDecoded msg;
  } _radioMessage;
  Timer _timeSinceLastRadioMessage;

  enum {
    MAX_NUM_RANGING_TARGETS = 32,
  };

  struct UWBTarget {
    uint8_t id;
    Vec3f position;
    unsigned numGoodMeas;
    unsigned numBadMeas;
    float lastMeas;
  };

  UWBTarget _rangingTargets[MAX_NUM_RANGING_TARGETS];

  unsigned _numRangingTargets;
  uint8_t _nextRangingTargetIdx;

  bool _gyroCalibrationEnabled;
  unsigned _gyroCalibrationNumSamples;
  Vec3f _gyroCalibrationAccumulated;
  Vec3f _gyroCalibrationBias;
  bool _testMotorsOn;
  float _testMotorsThrustFrac;

  enum {
    NUM_DEBUG_VARS = 6,
  };
  float _debug[NUM_DEBUG_VARS];  //first three are sent over telemetry

  int _firstPanicReason;

  struct {
    bool currentlyRunning;
    float activeFactors[4];  //should all be near one, used in flight, overwritten by _accumulators
    float accumulators[4];  //only used during the calibration
    unsigned accumulatorCount;
    float calibrationFactorMin, calibrationFactorMax;  //limits on the factors
    unsigned minAccumulatorCount;  //need at least this many data points
  } _propellerCalibration;

  uint8_t _telWarnings;

  struct MonitorSimplePeriod {
    MonitorSimplePeriod(BaseTimer* const timeIn, float samplePeriod,
                        float cuttoffFrq, float expectedDtIn)
        : timeSinceLast(timeIn) {
      filter.Initialise(samplePeriod, cuttoffFrq, expectedDtIn);
      lpDt = expectedDtIn;
    }
    void Update() {
      float dt = timeSinceLast.GetSeconds_f();
      lpDt = filter.Apply(dt);
      timeSinceLast.AdjustTimeBySeconds(-dt);
    }
    volatile float lpDt;
   private:
    LowPassFilterFirstOrder<float, float> filter;
    Timer timeSinceLast;
  };
  MonitorSimplePeriod _monitorCmdRate;
  MonitorSimplePeriod _monitorMainLoopPeriod;

  struct MonitorUWBEstimatorResets {
    MonitorUWBEstimatorResets(BaseTimer* const timeIn)
        : timeSinceLast(timeIn) {
      //nothing.
    }
    Timer timeSinceLast;  //keep track of when we're not running at the right rate
  };

  MonitorUWBEstimatorResets _monitorUWBEstimatorResets;

  bool _shouldWriteParameters;  //sets to true if we have parameters to store (e.g. calibration of props)

  uint8_t _myId;  //each vehicle should have a unique identifier.

  uint32_t _telPacketCounter;

  //Battery monitor stuff
  float _battThresholdCriticalVoltage;  // the critical value below which the battery voltage shouldn't drop
  float _battThresholdWarningVoltage;  //at which point we raise the telemetry flag.
  LowPassFilterSecondOrder<float, float> _battVoltageLowPass;
};

}  // namespace Onboard
