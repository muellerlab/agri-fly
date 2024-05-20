#include "ExampleVehicleStateMachine.hpp"

using namespace std;
using namespace Offboard;

ExampleVehicleStateMachine::ExampleVehicleStateMachine() {
  _name = "INVALID";
  _id = 0;

  _flightStage = StageWaitForStart;
  _lastFlightStage = StageComplete;
  _systemLatencyTime = 0;

  _vehicleIsReadyForProgramToExit = false;

  _initPosition = Vec3d(0, 0, 0);
  _desiredPosition = Vec3d(0, 0, 0);
  _desiredYawAngle = 0;
  _cmdYawAngle = 0;
  _lastTelWarnings = 0;

  _lastPos = Vec3d(0, 0, 0);
  _lastVel = Vec3d(0, 0, 0);
  _lastAcc = Vec3d(0, 0, 0);
  _lastYaw = 0;
}

void ExampleVehicleStateMachine::CallbackGPS(
    const hiperlab_rostools::gps_output& msg) {
  if (_est->GetID() == msg.vehicleID) {
    _est->UpdateWithMeasurement(Vec3d(msg.posx, msg.posy, msg.posz));
  }
}

void ExampleVehicleStateMachine::CallbackIMU(
    const hiperlab_rostools::imu_output& msg) {
  if (_est->GetID() == msg.vehicleID) {
    Vec3d accMeas(msg.accmeasx, msg.accmeasy, msg.accmeasz);
    Vec3d gyroMeas(msg.gyromeasx, msg.gyromeasy, msg.gyromeasz);
    _est->Predict(accMeas, gyroMeas);
  }
}

void ExampleVehicleStateMachine::CallbackTelemetry(
    const hiperlab_rostools::telemetry& msg) {
  _lastTelWarnings = msg.warnings;
}

void ExampleVehicleStateMachine::Initialize(int id, std::string name,
                                            ros::NodeHandle &n,
                                            BaseTimer* timer,
                                            double systemLatencyTime) {
  _id = id;
  stringstream ss;
  ss << "[" << name << " (" << _id << ")]: ";
  _name = ss.str();

  //set up ROS subscribers and publishers:
  _subGPS.reset(
      new ros::Subscriber(
          n.subscribe("gps_output" + std::to_string(_id), 1,
                      &ExampleVehicleStateMachine::CallbackGPS, this)));

  _subIMU.reset(
      new ros::Subscriber(
          n.subscribe("imu_output" + std::to_string(_id), 1,
                      &ExampleVehicleStateMachine::CallbackIMU, this)));

  _subTelemetry.reset(
      new ros::Subscriber(
          n.subscribe("telemetry" + std::to_string(_id), 1,
                      &ExampleVehicleStateMachine::CallbackTelemetry, this)));
  _pubEstimate.reset(
      new ros::Publisher(
          n.advertise<hiperlab_rostools::estimator_output>(
              "estimator" + std::to_string(_id), 1)));
  _pubCmd.reset(
      new ros::Publisher(
          n.advertise<hiperlab_rostools::radio_command>(
              "radio_command" + std::to_string(_id), 1)));

  //set up components:

  _est.reset(new GPSIMUStateEstimator(timer, _id));
  _systemLatencyTime = systemLatencyTime;
  _ctrl.reset(new QuadcopterController());
  _safetyNet.reset(new SafetyNet());
  _safetyNet->SetSafeCorners(Vec3d(-100,-100,-2.0), Vec3d(100,100,20), 0.0);
  _stageTimer.reset(new Timer(timer));

  _flightStage = StageWaitForStart;
  _lastFlightStage = StageComplete;

  // Initialize control parameters
  Onboard::QuadcopterConstants::QuadcopterType quadcopterType =
      Onboard::QuadcopterConstants::GetVehicleTypeFromID(_id);
  Onboard::QuadcopterConstants vehConsts(quadcopterType);
  _ctrl->SetParameters(vehConsts.posControl_natFreq,
                       vehConsts.posControl_damping,
                       vehConsts.attControl_timeConst_xy,
                       vehConsts.attControl_timeConst_z);

  cout << _name << "Created.\n";
}

void ExampleVehicleStateMachine::Run(bool shouldStart, bool shouldStop) {

  // Check for flight stage change
  bool stageChange = _flightStage != _lastFlightStage;
  _lastFlightStage = _flightStage;
  if (stageChange) {
    _stageTimer->Reset();
  }

  // Get the current state estimate and publish to ROS
  EstimatedState estState = _est->GetCurrentEstimate();

  _safetyNet->UpdateWithEstimator(estState,
                                  _est->GetTimeSinceLastGoodMeasurement());
  PublishEstimate(estState);

  // Create radio message depending on the current flight stage
  hiperlab_rostools::radio_command cmdMsg;
  uint8_t rawMsg[RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE];
  switch (_flightStage) {
    case StageWaitForStart:
      if (stageChange) {
        cout << _name << "Waiting for start signal.\n";
      }
      if (shouldStart) {
        _flightStage = StageSpoolUp;
      }
      break;

    case StageSpoolUp:
      if (stageChange) {
        cout << _name << "Spooling up motors.\n";
      }

      if (!_safetyNet->GetIsSafe()) {
        _flightStage = StageEmergency;
      }

      {
        double const motorSpoolUpTime = 0.5;  //[s]
        double const spoolUpThrustByWeight = 0.25;  //[]

        double cmdThrust = 9.81 * spoolUpThrustByWeight;
        Vec3d cmdAngVel(0, 0, 0);
        RadioTypes::RadioMessageDecoded::CreateRatesCommand(0, float(cmdThrust),
                                                            Vec3f(cmdAngVel),
                                                            rawMsg);
        cmdMsg.debugtype = RadioTypes::externalRatesCmd;
        cmdMsg.debugvals[0] = float(cmdThrust);
        cmdMsg.debugvals[1] = float(cmdAngVel.x);
        cmdMsg.debugvals[2] = float(cmdAngVel.y);
        cmdMsg.debugvals[3] = float(cmdAngVel.z);
        for (int i = 0; i < RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE;
            i++) {
          cmdMsg.raw[i] = rawMsg[i];
        }

        if (_stageTimer->GetSeconds<double>() > motorSpoolUpTime) {
          _flightStage = StageTakeoff;
        }
      }

      if (HaveLowBattery()) {
        printf("LOW BATTERY!\n");
        _flightStage = StageLanding;
      }
      break;

    case StageTakeoff:
      if (stageChange) {
        cout << _name << "Taking off.\n";
        _initPosition = estState.pos;
      }

      if (!_safetyNet->GetIsSafe()) {
        _flightStage = StageEmergency;
      }

      {
        double const takeOffTime = 2.0;  //[s]
        double frac = _stageTimer->GetSeconds<double>() / takeOffTime;
        if (frac >= 1.0) {
          _flightStage = StageFlight;
          frac = 1.0;
        }
        Vec3d cmdPos = (1 - frac) * _initPosition + frac * _desiredPosition;
        cmdMsg = RunControllerAndUpdateEstimator(estState, cmdPos,
                                                 Vec3d(0, 0, 0),
                                                 Vec3d(0, 0, 0));
      }

      if (HaveLowBattery()) {
        printf("LOW BATTERY!\n");
        _flightStage = StageLanding;
      }
      break;

    case StageFlight:
      if (stageChange) {
        cout << _name << "Entering flight stage.\n";
      }

      if (!_safetyNet->GetIsSafe()) {
        _flightStage = StageEmergency;
      }

      if (HaveLowBattery()) {
        printf("LOW BATTERY!\n");
        _flightStage = StageLanding;
      }

      {
        // Command a specific trajectory
        int trajID = 3;  // 0 - fixed pt, 1 - circle, 2 - SHM
        Vec3d cmdPos(0, 0, 0), cmdVel(0, 0, 0), cmdAcc(0, 0, 0);
        double t = _stageTimer->GetSeconds<double>();  // I know 't' is a really bad name, sorry
        double const getIntoActionTime = 2.0;  //[s]
        double frac = min(t / getIntoActionTime, 1.0);
        switch (trajID) {
          case 0:  // Fixed set point
          {
            cmdPos = _desiredPosition;
            cmdVel = Vec3d(0, 0, 0);
            cmdAcc = Vec3d(0, 0, 0);
            _cmdYawAngle = 0;
          }
            break;

          case 1:  // Circular trajectory
          {
            Vec3d circleCenter(0.0, -2.0, _desiredPosition.z);
            double radius = 1.0;  // [m]
            double angSpeed = 0.5;  // [rad/s]
            cmdPos = circleCenter
                + radius * Vec3d(cos(angSpeed * t), sin(angSpeed * t), 0);
            cmdVel = radius * angSpeed
                * Vec3d(-sin(angSpeed * t), cos(angSpeed * t), 0);
            cmdAcc = radius * pow(angSpeed, 2)
                * Vec3d(-cos(angSpeed * t), -sin(angSpeed * t), 0);
            _cmdYawAngle = _desiredYawAngle + angSpeed * t;
          }
            break;

          case 2:  // SHM trajectory
          {
            double amplitude = 1.0;  // [m]
            double angFreq = 2.0;  // [rad/s]
            cmdPos = _desiredPosition
                + amplitude * Vec3d(0, sin(angFreq * t), 0);
            cmdVel = amplitude * angFreq * Vec3d(0, cos(angFreq * t), 0);
            cmdAcc = amplitude * pow(angFreq, 2)
                * Vec3d(0, -sin(angFreq * t), 0);
            _cmdYawAngle = _desiredYawAngle;
          }
            break;

          case 3:  // Circle at Fixed Height Line Test
          {
            Vec3d circleCenter(0.0, 0.0, _desiredPosition.z);
            double radius = 0.5;  // [m]
            double angSpeed = 1;  // [rad/s]
            cmdPos = circleCenter
                + radius * Vec3d(cos(angSpeed * t), sin(angSpeed * t), 0);
            cmdVel = radius * angSpeed
                * Vec3d(-sin(angSpeed * t), cos(angSpeed * t), 0);
            cmdAcc = radius * pow(angSpeed, 2)
                * Vec3d(-cos(angSpeed * t), -sin(angSpeed * t), 0);
            _cmdYawAngle = 0;
          }
            break;

          case 4:  // Circle at Sinusoidal Height and Yaw
          {
            Vec3d circleCenter(0.0, 0.0, _desiredPosition.z);
            double radius = 0.5;  // [m]
            double angSpeed = 0.5;  // [rad/s]
            cmdPos = circleCenter
                + radius * Vec3d(cos(angSpeed * t), sin(angSpeed * t), cos(angSpeed * t * 4));
            cmdVel = radius * angSpeed
                * Vec3d(-sin(angSpeed * t), cos(angSpeed * t), -sin(angSpeed * t * 4));
            cmdAcc = radius * pow(angSpeed, 2)
                * Vec3d(-cos(angSpeed * t), -sin(angSpeed * t), -cos(angSpeed * t * 4));
            _cmdYawAngle = angSpeed*t;
          }
            break;

          case 5:  // Rotate Yaw at fixed position
          {
            cmdPos = _desiredPosition;
            _cmdYawAngle = 0.2*t;
          }
            break;
        }

        _lastPos = (1 - frac) * _desiredPosition + frac * cmdPos;
        _lastVel = frac * cmdVel;
        _lastAcc = frac * cmdAcc;

        cmdMsg = RunControllerAndUpdateEstimator(estState, _lastPos, _lastVel,
                                                 _lastAcc);
      }
      if (shouldStop) {
        _flightStage = StageLanding;
      }
      break;

    case StageLanding:
      if (stageChange) {
        cout << _name << "Starting landing.\n";
      }

      if (!_safetyNet->GetIsSafe()) {
        _flightStage = StageEmergency;
      }

      {
        double const LANDING_SPEED = 0.5;  //m/s
        double const getIntoActionTime = 2.0;  //[s]
        double frac = min(_stageTimer->GetSeconds<double>() / getIntoActionTime,
                          1.0);
        Vec3d cmdPos = _lastPos
            + _stageTimer->GetSeconds<double>() * Vec3d(0, 0, -LANDING_SPEED);
        if (cmdPos.z < 0) {
          _flightStage = StageComplete;
        }
        cmdMsg = RunControllerAndUpdateEstimator(
            estState, (1 - frac) * _lastPos + frac * cmdPos,
            (1 - frac) * _lastVel + frac * Vec3d(0, 0, -LANDING_SPEED),
            (1 - frac) * _lastAcc + frac * Vec3d(0, 0, 0));
      }
      break;

    case StageComplete:
      if (stageChange) {
        cout << _name << "Landing complete. Idling.\n";
      }

      {
        //Publish the commands:
        RadioTypes::RadioMessageDecoded::CreateIdleCommand(0, rawMsg);
        for (int i = 0; i < RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE;
            i++) {
          cmdMsg.raw[i] = rawMsg[i];
        }
        cmdMsg.debugtype = RadioTypes::idleCommand;
      }

      if (_stageTimer->GetSeconds<double>() > 1.0) {
        cout << _name << "Exiting.\n";
        _vehicleIsReadyForProgramToExit = true;
      }
      break;

    default:
    case StageEmergency:
      if (stageChange) {
        cout << _name << "Emergency stage! Safety net = <"
             << _safetyNet->GetStatusString() << ">.\n";
      }

      RadioTypes::RadioMessageDecoded::CreateKillCommand(0, rawMsg);
      for (int i = 0; i < RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE;
          i++) {
        cmdMsg.raw[i] = rawMsg[i];
      }
      cmdMsg.debugtype = RadioTypes::emergencyKill;
      break;
  }

  cmdMsg.header.stamp = ros::Time::now();

  _pubCmd->publish(cmdMsg);

}

void ExampleVehicleStateMachine::PublishEstimate(
    EstimatedState estState) {
  // Publish the current state estimate

  hiperlab_rostools::estimator_output estOutMsg;
  estOutMsg.header.stamp = ros::Time::now();
  estOutMsg.vehicleID = _est->GetID();

  estOutMsg.posx = estState.pos.x;
  estOutMsg.posy = estState.pos.y;
  estOutMsg.posz = estState.pos.z;

  estOutMsg.velx = estState.vel.x;
  estOutMsg.vely = estState.vel.y;
  estOutMsg.velz = estState.vel.z;

  estOutMsg.attq0 = estState.att[0];
  estOutMsg.attq1 = estState.att[1];
  estOutMsg.attq2 = estState.att[2];
  estOutMsg.attq3 = estState.att[3];

  estOutMsg.attyaw = estState.att.ToEulerYPR().x;
  estOutMsg.attpitch = estState.att.ToEulerYPR().y;
  estOutMsg.attroll = estState.att.ToEulerYPR().z;

  estOutMsg.angvelx = estState.angVel.x;
  estOutMsg.angvely = estState.angVel.y;
  estOutMsg.angvelz = estState.angVel.z;

  _pubEstimate->publish(estOutMsg);
}

hiperlab_rostools::radio_command ExampleVehicleStateMachine::RunControllerAndUpdateEstimator(
    EstimatedState estState, Vec3d desPos,
    Vec3d desVel, Vec3d desAcc) {
  // Run the rates controller
  Vec3d cmdAngVel;
  double cmdThrust;
  _ctrl->Run(estState.pos, estState.vel, estState.att, desPos, desVel, desAcc,
             _cmdYawAngle, cmdAngVel, cmdThrust);

  //Create and return the radio command
  uint8_t rawMsg[RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE];
  RadioTypes::RadioMessageDecoded::CreateRatesCommand(0, float(cmdThrust),
                                                      Vec3f(cmdAngVel), rawMsg);
  hiperlab_rostools::radio_command cmdMsg;
  for (int i = 0; i < RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE; i++) {
    cmdMsg.raw[i] = rawMsg[i];
  }
  cmdMsg.debugvals[0] = float(cmdThrust);
  cmdMsg.debugvals[1] = float(cmdAngVel.x);
  cmdMsg.debugvals[2] = float(cmdAngVel.y);
  cmdMsg.debugvals[3] = float(cmdAngVel.z);
  cmdMsg.debugtype = RadioTypes::externalRatesCmd;

  return cmdMsg;
}
