#include "ExampleVehicleStateMachine.hpp"

using namespace std;
using namespace Offboard;
using namespace RectangularPyramidPlanner;
ExampleVehicleStateMachine::ExampleVehicleStateMachine() {
  _name = "INVALID";
  _id = 0;

  _flightStage = StageWaitForStart;
  _estType = GPSEstimator; // Default use odometry estimator
  _lastFlightStage = StageComplete;
  _systemLatencyTime = 0;

  _vehicleIsReadyForProgramToExit = false;

  _initPosition = Vec3d(0, 0, 0);
  _desiredPosition = Vec3d(0, 0, 2.0);
  _desiredYawAngle = 0.0;
  _cmdYawAngle = 0.0;
  _lastTelWarnings = 0;

  _lastPos = Vec3d(0, 0, 0);
  _lastVel = Vec3d(0, 0, 0);
  _lastAcc = Vec3d(0, 0, 0);
  _lastYaw = 0;

  _depthScale = 10.0 / (256.0);
  _depthCamAtt = Rotationd::FromEulerYPR(-90.0 * M_PI / 180.0, 0 * M_PI / 180.0,
                                         -90 * M_PI / 180.0);

  _depthImageHeight = 0;
  _depthImageWidth = 0;
  _focalLength = 0;
  _plannedTrajCount = 0;

  _trajAtt = Rotationd::Identity();
  _trajOffset = Vec3d(0, 0, 0);
  _imageReady = false;
  _startPlan = false;
  _previousThrust = 9.81;
  _physicalVehicleRadius = 0.2;
  _vehicleRadiusPlanning = 0.2;
  _minCollisionDist = 0.5;
  _plannedTrajDuration = 0;
  _lookAheadTime = 0.02;
  _goalWorld = Vec3d(20.0, 0.0, 2.5);
  _lastGoal = _goalWorld;
  _depthImageCount = 0;
  _rgbImageCount = 0;
  _firstTrajReady = false;
}

void ExampleVehicleStateMachine::CallbackTelemetry(
    const hiperlab_rostools::telemetry &msg) {
  _lastTelWarnings = msg.warnings;
}

void ExampleVehicleStateMachine::CallbackMocapEstimator(
    const hiperlab_rostools::mocap_output &msg) {
  if (_mocapEst->GetID() == msg.vehicleID) {
    _mocapEst->UpdateWithMeasurement(
        Vec3d(msg.posx, msg.posy, msg.posz),
        Rotationd(msg.attq0, msg.attq1, msg.attq2, msg.attq3));
  }
}

void ExampleVehicleStateMachine::CallbackIMU(
    const hiperlab_rostools::imu_output& msg) {
  if (_estType == GPSEstimator) {
    if (_gpsEst->GetID() == msg.vehicleID) {
      Vec3d accMeas(msg.accmeasx, msg.accmeasy, msg.accmeasz);
      Vec3d gyroMeas(msg.gyromeasx, msg.gyromeasy, msg.gyromeasz);
      _gpsEst->Predict(accMeas, gyroMeas);
    }
  }
}

void ExampleVehicleStateMachine::CallbackGPSEstimator(
    const hiperlab_rostools::gps_output &msg) {
  if (_gpsEst->GetID() == msg.vehicleID) {
    _gpsEst->UpdateWithMeasurement(
        Vec3d(msg.posx, msg.posy, msg.posz));
  }
}

void ExampleVehicleStateMachine::CallbackOdometry(
    const nav_msgs::Odometry &msg) {

  // Frame definitions according to the tracking camera ros node:
  // Tracking camera world frame: x-axis in the direction of the camera FOV, z-axis upwards (opposite gravity)
  // Tracking camera fixed frame: x-axis in the direction of the camera FOV, y-axis towards left camera (opposite direction of the realsense logo)
  // When the tracking camera is first turned on, it defines the tracking camera world frame to have z-axis upwards and x-axis pointing in the same
  // direction as the tracking camera FOV. The tracking camera orientation is then defined as the rotation from the tracking camera world frame
  // to the tracking camera fixed frame (not necessarily an identity rotation, e.g. if the camera is mounted upside down).

  Rotationd worldFrameToOdometryWorldFrame = Rotationd::Identity();  //Update this when tracking camera uses a different frame

  // !!! Warning: check the frame here before modification of the code!
  Vec3d posOdometry = worldFrameToOdometryWorldFrame
      * Vec3d(msg.pose.pose.position.x, msg.pose.pose.position.y,
              msg.pose.pose.position.z);

  Rotationd attOdometry = worldFrameToOdometryWorldFrame
      * Rotationd(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
                  msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);  //Attitude estimation given by T265

  _odometryEst->UpdateWithMeasurement(posOdometry, attOdometry);

  if (_estType == OdometryEstimator){
    //! linear velocity and angular velocity are expressed in Body frame
    Vec3d _estVel_Body = Vec3d(msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                              msg.twist.twist.linear.z);
    //! rewritten in Inertia frame
    _estVel = worldFrameToOdometryWorldFrame * _estVel_Body;
  }
}

/// @brief 
/// @param msg 
void ExampleVehicleStateMachine::CallbackDepthImages(
    const sensor_msgs::ImageConstPtr &msg) {

  std_msgs::Header h = msg->header;
  ros::Time publishTime = h.stamp;
  ros::Time receiveImageTime = ros::Time::now();
  ros::Duration transMissionTime = receiveImageTime - publishTime;
  cv::Mat depthImage;
  cv::Mat depthImage_uint8 = cv_bridge::toCvShare(
      msg, sensor_msgs::image_encodings::RGB8)->image;

  depthImage_uint8.convertTo(depthImage, CV_16U);
  /*char buffer[256];
   sprintf(buffer, "%04d", _depthImageCount);
   std::string str(buffer);
   std::string file_path = "/home/jbirtman/Documents/AirSim/img" + str + ".bmp";
   cv::imwrite(file_path, depthImage); */

//  {
//    // This block prints the value of the largest and smallest depth in the image, and save the depth image to local. Uncomment for debug purpose
//    uint16_t *depthData;
//    depthData = reinterpret_cast<uint16_t*>(depthImage.data);
//    unsigned smallestDepth(100000);
//    unsigned largestDepth(0);
//    for (unsigned i = 0; i < _depthImageWidth; i++) {
//      for (unsigned j = 0; j < _depthImageHeight; j++)
//        if ((depthData[i * _depthImageWidth + j] < smallestDepth)
//            && (depthData[i * _depthImageWidth + j] >= 0)) {
//          smallestDepth = depthData[i * _depthImageWidth + j];
//        }
//    }
//
//    for (unsigned i = 0; i < _depthImageWidth; i++) {
//      for (unsigned j = 0; j < _depthImageHeight; j++)
//        if (depthData[i * _depthImageWidth + j] > largestDepth) {
//          largestDepth = depthData[i * _depthImageWidth + j];
//        }
//    }
//
//    printf("largest: %d\n", largestDepth);
//    printf("smallest: %d\n", smallestDepth);
//
  //  char buffer[256];
  //  sprintf(buffer, "%04d", _depthImageCount);
  //  std::string str(buffer);
  //  std::string file_path = "/home/teaya/Documents/AirSim/img" + str + ".bmp";
  //  cv::imwrite(file_path, depthImage);
//  }
  _depthImageWidth = depthImage.cols;
  _depthImageHeight = depthImage.rows;
  _focalLength = _depthImageWidth / 2.0;

  ros::Time finishDecoding = ros::Time::now();
  DepthImagePlanner planner(depthImage, _depthScale, _focalLength,
                            _depthImageWidth / 2.0, _depthImageHeight / 2.0,
                            _physicalVehicleRadius, _vehicleRadiusPlanning,
                            _minCollisionDist);

  planner.SetRandomSeed(_depthImageCount);  // Trajectory ID = Depth image ID (=> Random seed)

  ros::Duration decodingTime = finishDecoding - receiveImageTime;
  double offsetTime = (finishDecoding - publishTime).toSec();
  double compTime = 0.015;
  EstimatedState estState = EstGetState(compTime);

  _estAtt = estState.att;
  _estPos = estState.pos;

  Vec3d vel = _depthCamAtt.Inverse() * _estAtt.Inverse() * estState.vel;
  Vec3d acc = _depthCamAtt.Inverse() * _estAtt.Inverse()
      * (Vec3d(0, 0, 1) * _previousThrust - Vec3d(0, 0, 9.81));
//  Vec3d g = _depthCamAtt.Inverse() * Vec3d(0, 0, -9.81);
  Vec3d g = _depthCamAtt.Inverse() * _estAtt.Inverse() * Vec3d(0, 0, -9.81);

  RapidQuadrocopterTrajectoryGenerator::RapidTrajectoryGenerator candidateTraj(
      Vec3d(0, 0, 0), vel, acc, g);

  std::vector<TrajectoryTest> trajectories;
  if (_startPlan) {
    ros::Time planningStartTime = ros::Time::now();

//    {
//      // Alternative single direction exploration planner
//      Vec3d exploration_direction(0, 0, 1.0);
//      bool planResult = planner.FindFastestTrajRandomCandidates(
//          candidateTraj, trajectories, compTime, exploration_direction);
//    }

    bool planResult = planner.FindLowestCostTrajectoryRandomCandidates(
        candidateTraj, trajectories, compTime, (void*) this,
        &ExampleVehicleStateMachine::GetTrajCostWrapper);

    if (planResult) {
      _plannedTrajCount++;
      _trajAtt = _estAtt * _depthCamAtt;
      _trajOffset = _estPos;
      _plannedTraj = std::make_shared<
          RapidQuadrocopterTrajectoryGenerator::RapidTrajectoryGenerator>(
          candidateTraj);
      _plannedTrajDuration = _plannedTraj->GetFinalTime();
      _trajResetTime = ros::Time::now();
      _timeOnPlannedTraj->Reset();

      {
        double traj_y, traj_p, traj_r;
        _trajAtt.ToEulerYPR(traj_y, traj_p, traj_r);
        CommonMath::Trajectory trajToRecord = candidateTraj.GetTrajectory();
        std::vector<Vec3d> trajCoeffs = trajToRecord.GetCoeffs();

        // Record the trajectory
        _plannedTrajLog << _plannedTrajCount << ",";
        _plannedTrajLog << trajCoeffs[0].x << "," << trajCoeffs[0].y << ","
            << trajCoeffs[0].z << ",";
        _plannedTrajLog << trajCoeffs[1].x << "," << trajCoeffs[1].y << ","
            << trajCoeffs[1].z << ",";
        _plannedTrajLog << trajCoeffs[2].x << "," << trajCoeffs[2].y << ","
            << trajCoeffs[2].z << ",";
        _plannedTrajLog << trajCoeffs[3].x << "," << trajCoeffs[3].y << ","
            << trajCoeffs[3].z << ",";
        _plannedTrajLog << trajCoeffs[4].x << "," << trajCoeffs[4].y << ","
            << trajCoeffs[4].z << ",";
        _plannedTrajLog << trajCoeffs[5].x << "," << trajCoeffs[5].y << ","
            << trajCoeffs[5].z << ",";
        _plannedTrajLog << traj_y << "," << traj_p << "," << traj_r << ",";
        _plannedTrajLog << _trajOffset.x << "," << _trajOffset.y << ","
            << _trajOffset.z << ",";
        _plannedTrajLog << trajToRecord.GetStartTime() << ","
            << trajToRecord.GetEndTime() << "\n";
      }

      if (not _firstTrajReady) {
        _firstTrajReady = true;
      }
    }

    // planner internal logging
    // should not use _traj* variables, which is only for found collision-free trajectories
    {
      hiperlab_rostools::planner_diagnostics planner_msg = { };
      planner_msg.header.stamp = ros::Time::now();
      // input
      planner_msg.input.random_seed = planner.GetRandomSeed();
      planner_msg.input.velocity_D = Vec3dToVector3(vel);
      planner_msg.input.acceleration_D = Vec3dToVector3(acc);
      planner_msg.input.gravity_D = Vec3dToVector3(g);
      planner_msg.input.goal_W = Vec3dToVector3(_goalWorld);

      // output
      CommonMath::Trajectory trajectory_parameters =
          candidateTraj.GetTrajectory();
      planner_msg.output.trajectory_id = msg->header.seq;

      planner_msg.output.planner_statistics.trajectory_found = planResult;
      planner_msg.output.planner_statistics.NumCollisionFree = planner
          .GetNumCollisionFree();
      planner_msg.output.planner_statistics.NumPyramids =
          planner.GetNumPyramids();
      planner_msg.output.planner_statistics.NumVelocityChecks = planner
          .GetNumVelocityChecks();
      planner_msg.output.planner_statistics.NumCollisionChecks = planner
          .GetNumCollisionChecks();
      planner_msg.output.planner_statistics.NumCostChecks = planner
          .GetNumCostChecks();
      planner_msg.output.planner_statistics.NumTrajectoriesGenerated = planner
          .GetNumTrajectoriesGenerated();

      planner_msg.output.trajectory_parameters_D.coeff0 = Vec3dToVector3(
          trajectory_parameters[0]);
      planner_msg.output.trajectory_parameters_D.coeff1 = Vec3dToVector3(
          trajectory_parameters[1]);
      planner_msg.output.trajectory_parameters_D.coeff2 = Vec3dToVector3(
          trajectory_parameters[2]);
      planner_msg.output.trajectory_parameters_D.coeff3 = Vec3dToVector3(
          trajectory_parameters[3]);
      planner_msg.output.trajectory_parameters_D.coeff4 = Vec3dToVector3(
          trajectory_parameters[4]);
      planner_msg.output.trajectory_parameters_D.coeff5 = Vec3dToVector3(
          trajectory_parameters[5]);
      planner_msg.output.trajectory_parameters_D.duration = ros::Time(
          trajectory_parameters.GetEndTime());

      planner_msg.output.trajectory_reset_time = planningStartTime;
      planner_msg.output.trajectory_transform.translation = Vec3dToVector3(
          _trajOffset);
      planner_msg.output.trajectory_transform.rotation = RotationdToQuaternion(
          _trajAtt);
      _pubPlannerDiagnotics->publish(planner_msg);
    }

  }

  if (_depthImageCount < 1000) {
    _imageReceiveLog << _depthImageCount << ",";
    _imageReceiveLog << transMissionTime.toSec() << ",";
    _imageReceiveLog << decodingTime.toSec() << ",";
    _imageReceiveLog << "\n";
  } else if (_depthImageCount == 1000) {
    _imageReceiveLog.close();
  }

  if (_plannedTrajCount > 1000) {
    _plannedTrajLog.close();
  }

  _depthImageCount++;
}

void ExampleVehicleStateMachine::CallbackRGBImages(
    const sensor_msgs::ImageConstPtr &msg) {

  std_msgs::Header h = msg->header;
  //ros::Time publishTime = h.stamp;
  //ros::Time receiveImageTime = ros::Time::now();
  //ros::Duration transMissionTime = receiveImageTime - publishTime;
  cv::Mat otherImage;
  cv::Mat otherImage_uint8 = cv_bridge::toCvShare(
      msg, sensor_msgs::image_encodings::RGB8)->image;

  otherImage_uint8.convertTo(otherImage, CV_16U);

  /*char buffer[256];
   sprintf(buffer, "%04d", _rgbImageCount);
   std::string str(buffer);
   std::string file_path = "/home/{path_to_debug_folder}" + str
   + ".bmp";
   cv::imwrite(file_path, otherImage); */
  _rgbImageCount++;
}

void ExampleVehicleStateMachine::Initialize(int id, std::string name,
                                            ros::NodeHandle &n,
                                            BaseTimer *timer,
                                            double systemLatencyTime) {
  _id = id;
  stringstream ss;
  ss << "[" << name << " (" << _id << ")]: ";
  _name = ss.str();

//Set the trajectory file path:
  n.getParam("traj_file", trajectory_file);

//set up networking stuff:
  _subMocap.reset(
      new ros::Subscriber(
          n.subscribe("mocap_output" + std::to_string(_id), 1,
                      &ExampleVehicleStateMachine::CallbackMocapEstimator, this)));

  _subGPS.reset(
      new ros::Subscriber(
          n.subscribe("gps_output" + std::to_string(_id), 1,
                      &ExampleVehicleStateMachine::CallbackGPSEstimator, this)));

  _subIMU.reset(
      new ros::Subscriber(
          n.subscribe("imu_output" + std::to_string(_id), 1,
                      &ExampleVehicleStateMachine::CallbackIMU, this)));

  _subTelemetry.reset(
      new ros::Subscriber(
          n.subscribe("telemetry" + std::to_string(_id), 1,
                      &ExampleVehicleStateMachine::CallbackTelemetry, this)));

  _subOdometry.reset(
      new ros::Subscriber(
          n.subscribe("/camera/t265/odom/sample", 1,
                      &ExampleVehicleStateMachine::CallbackOdometry, this,
                      ros::TransportHints().tcpNoDelay())));

  image_transport::ImageTransport _it(n); // Depth image transporter
  image_transport::ImageTransport _rgbIt(n); // Rgp image transporter

  _subDepthImages = _it.subscribe(
      "depthImage", 1, &ExampleVehicleStateMachine::CallbackDepthImages, this);  // Follow Nathan's example in generating the image subscriber

  _subRGBImages = _rgbIt.subscribe(
      "rgbImage", 1, &ExampleVehicleStateMachine::CallbackRGBImages, this);

  _pubEstimate.reset(
      new ros::Publisher(
          n.advertise < hiperlab_rostools::estimator_output
              > ("estimator" + std::to_string(_id), 1)));

  _pubCmd.reset(
      new ros::Publisher(
          n.advertise < hiperlab_rostools::radio_command
              > ("radio_command" + std::to_string(_id), 1)));

  _pubPlannerDiagnotics.reset(
      new ros::Publisher(
          n.advertise < hiperlab_rostools::planner_diagnostics
              > ("planner_diagnostics", 10)));

  _pubControllerDiagnotics.reset(
      new ros::Publisher(
          n.advertise < hiperlab_rostools::controller_diagnostics
              > ("controller_diagnostics", 10)));

//set up components:
  _gpsEst.reset(new GPSIMUStateEstimator(timer, _id));
  _mocapEst.reset(new MocapStateEstimator(timer, _id, systemLatencyTime));
  _odometryEst.reset(new MocapStateEstimator(timer, _id, systemLatencyTime)); //Abuse of name here. We reuse MocapStateEstimator for odometry class as their update methods are same. 

  _systemLatencyTime = systemLatencyTime;
  _ctrl.reset(new QuadcopterController());
  _safetyNet.reset(new SafetyNet());
  _safetyNet->SetSafeCorners(Vec3d(-100,-100,-2.0), Vec3d(100,100,20), 0.0);

  _flightStage = StageWaitForStart;
  _lastFlightStage = StageComplete;
  _stageTimer.reset(new Timer(timer));
  _timeOnPlannedTraj.reset(new Timer(timer));

// Initialize control parameters
  Onboard::QuadcopterConstants::QuadcopterType quadcopterType =
      Onboard::QuadcopterConstants::GetVehicleTypeFromID(_id);
  Onboard::QuadcopterConstants vehConsts(quadcopterType);
  _ctrl->SetParameters(vehConsts.posControl_natFreq,
                       vehConsts.posControl_damping,
                       vehConsts.attControl_timeConst_xy,
                       vehConsts.attControl_timeConst_z);
  double armLength = vehConsts.armLength;
  _physicalVehicleRadius = armLength * 2.0;
  _vehicleRadiusPlanning = armLength * 2.0 * 1.5;
  _depthCamAtt = Rotationd::FromEulerYPR(-90.0 * M_PI / 180.0, 0 * M_PI / 180.0,
                                         -90 * M_PI / 180.0);

  //  _imageReceiveLog.open("depthImageReceiver.csv");
  //  _plannedTrajLog.open("plannedTraj.csv");

  //Reading first goal
  //TODO This is not general!!!
  _trajFile.open(trajectory_file.c_str(), ios::in);
  string _trajLine;
  if(getline(_trajFile,_trajLine)){
    // first line exists
    stringstream _trajLineSS(_trajLine);
    string linePiece;
    getline(_trajLineSS,linePiece,',');
    _goalWorld.x = stof(linePiece);
    getline(_trajLineSS,linePiece,',');
    _goalWorld.y = stof(linePiece);
    getline(_trajLineSS,linePiece,',');
    _goalWorld.z = stof(linePiece);
    cout << _name << "Initial waypoint = <" << _goalWorld.x << "," << _goalWorld.y << "," << _goalWorld.z << ">\n";
  }
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
  EstimatedState estState = EstGetState(_systemLatencyTime);
  _safetyNet->UpdateWithEstimator(estState,
                                  EstGetTimeSinceLastGoodMeasurement());
  PublishEstimate(estState);

// Create radio message depending on the current flight stage
  hiperlab_rostools::radio_command cmdMsg;
  uint8_t rawMsg[RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE];
  switch (_flightStage) {

    case StageWaitForStart: {
      if (stageChange) {
        cout << _name << "Waiting for start signal.\n";
      }
      if (shouldStart) {
        _flightStage = StageSpoolUp;
      }
      break;
    }
    case StageSpoolUp: {
      if (stageChange) {
        cout << _name << "Spooling up motors.\n";
      }
      if (!_safetyNet->GetIsSafe()) {
        _flightStage = StageEmergency;
      }
      {
        double const motorSpoolUpTime = 0.5;  //[s]
        double const spoolUpThrustByWeight = 0.25;  //[]

        EstSetPredictedValues(Vec3d(0, 0, 0), Vec3d(0, 0, 0));
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
    }

    case StageTakeoff: {
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
          _flightStage = StageHover;
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
    }

    case StageHover: {
      if (stageChange) {
        cout << _name << "Hover stage.\n";
      }

      if (!_safetyNet->GetIsSafe()) {
        _flightStage = StageEmergency;
      }

      {
        Vec3d cmdPos = _desiredPosition;
        cmdMsg = RunControllerAndUpdateEstimator(estState, cmdPos,
                                                 Vec3d(0, 0, 0),
                                                 Vec3d(0, 0, 0));
      }

      if (_stageTimer->GetSeconds<double>() > 3.0) {
        _flightStage = StageFlight;
        _startPlan = true;
        cout << _name << "Start Rappids\n";
      }

      if (HaveLowBattery()) {
        printf("LOW BATTERY!\n");
        _flightStage = StageLanding;
      }
      break;
    }

    case StageFlight: {
      if (stageChange) {
        cout << _name << "Entering flight stage.\n";
      }

      if (HaveLowBattery()) {
        printf("LOW BATTERY!\n");
        _flightStage = StageLanding;
      }

      if (not _firstTrajReady) {
        cmdMsg = RunControllerAndUpdateEstimator(estState, _desiredPosition,
                                                 Vec3d(0, 0, 0),
                                                 Vec3d(0, 0, 0));
      } else {
        static Vec3d refPos;
        Vec3d refVel;
        Vec3d refAcc;
        double refThrust;
        Vec3d refAngVel;

        Rotationf cmdAttOutput;
        Vec3d cmdAngVelOutput;
        double cmdThrustOutput;

        double t;

        t = (ros::Time::now() - _trajResetTime).toSec();
        static Vec3d hold_pos = _estPos;  // current est pose when it's enabled
        if (t > _plannedTraj->GetFinalTime()) {
          //Mode 1: When the vehicle reached its final goal point (Desired action: Hovering)
          // POSITION CONTROL HOVERING
          refPos = hold_pos;
          refVel = Vec3d(0, 0, 0);
          refAcc = Vec3d(0, 0, 0);
          refAngVel = Vec3d(0, 0, 0);
          refThrust = 9.80665;
        } else {
          //Mode 2: Vehicle tracking collision-avoidance trajectory
          hold_pos = _estPos;
          // RATE CONTROL RAPPIDS
          // in Depth Camera Frame
          Vec3d trajPos = _plannedTraj->GetPosition(t);
          Vec3d trajVel = _plannedTraj->GetVelocity(t);
          Vec3d trajAcc = _plannedTraj->GetAcceleration(t);

          // in World Frame
          refPos = _trajAtt * trajPos + _trajOffset;  // R * x + p
          refVel = _trajAtt * trajVel;
          refAcc = _trajAtt * trajAcc;

          const double loopRate = 100;
          // TODO Depth to Body (_est_att.Inverse() * _trajAtt) can be hard-coded
          refAngVel = _estAtt.Inverse() * _trajAtt
              * _plannedTraj->GetOmega(t, 1.0 / loopRate);  // in Body Frame
          refThrust = _plannedTraj->GetThrust(t);
        
        // This block changes the desired yaw to towards the goal point
         Vec3d desired_direction_w = _goalWorld - _estPos;    // in World Frame
         _desiredYawAngle = atan2(desired_direction_w.y,
                                  desired_direction_w.x);  // atan2 to return -pi to pi
        }

        cmdMsg = RunTrackingControllerAndUpdateEstimator(estState, refPos,
                                                         refVel, refAcc,
                                                         refThrust, refAngVel,
                                                         cmdThrustOutput,
                                                         cmdAttOutput,
                                                         cmdAngVelOutput);

        // controller internal logging
        {
          // input
          hiperlab_rostools::controller_diagnostics ctl_diag_msg = { };
          ctl_diag_msg.header.stamp = ros::Time::now();

          ctl_diag_msg.input.desired_yaw = _desiredYawAngle;
          ctl_diag_msg.input.position_estimate_W = Vec3dToVector3(estState.pos);
          ctl_diag_msg.input.velocity_estimate_W = Vec3dToVector3(estState.vel);
          ctl_diag_msg.input.attitude_estimate_W = RotationdToQuaternion(
              estState.att);

          ctl_diag_msg.input.trajectory_id = _depthImageCount;  // Trajectory ID = Depth image ID (=> Random seed)
          ctl_diag_msg.input.trajectory_time = ros::Time(t);  // From sec

          ctl_diag_msg.input.position_reference_W = Vec3dToVector3(refPos);
          ctl_diag_msg.input.velocity_reference_W = Vec3dToVector3(refVel);
          ctl_diag_msg.input.acceleration_reference_W = Vec3dToVector3(refAcc);
          ctl_diag_msg.input.thrust_reference_B = refThrust;
          ctl_diag_msg.input.angular_velocity_reference_B = Vec3dToVector3(
              refAngVel);

          ctl_diag_msg.input.current_battery = 12.4;  //Made up number. We don't care that much for the simulator.
          // output
          ctl_diag_msg.output.attitude_command_W = RotationfToQuaternion(
              cmdAttOutput);
          ctl_diag_msg.output.angular_velocity_command_B = Vec3dToVector3(
              cmdAngVelOutput);
          ctl_diag_msg.output.thrust_command_B = cmdThrustOutput;
          ctl_diag_msg.output.thrust_adapt_coefficient = float(1.0f);  //Synthesized value. We don't have this correction yet
          _pubControllerDiagnotics->publish(ctl_diag_msg);
        }
      }
      if (shouldStop) {
        _flightStage = StageLanding;
      }

      // Changing waypoint
      Vec3d desired_direction_w = _goalWorld - _estPos;
      float dist_to_goal = desired_direction_w.GetNorm2();

      // What to do if waypoint is reached
      if (dist_to_goal < 1.0) {
        string _trajLine;
        if (getline(_trajFile,_trajLine)) {
          // If intermediate waypoint is reached
          cout << _name << "Waypoint reached.\n";

          _lastGoal = _goalWorld;// store previous goal point
          // Read waypint and set new goal
          stringstream _trajLineSS(_trajLine);
          string linePiece;
          getline(_trajLineSS,linePiece,',');
          _goalWorld.x = stof(linePiece);
          getline(_trajLineSS,linePiece,',');
          _goalWorld.y = stof(linePiece);
          getline(_trajLineSS,linePiece,',');
          _goalWorld.z = stof(linePiece);
          cout << _name << "Waypoint updated to <" << _goalWorld.x << "," << _goalWorld.y << "," << _goalWorld.z << ">\n";

        } else {
          // If final goal is reached
          cout << _name << "Final goal reached.\n";
          _flightStage = StageLanding;
        }
      }
      break;
    }

    case StageLanding: {
      if (stageChange) {
        cout << _name << "Starting landing.\n";
        _lastPos = _estPos;
        _lastVel = _estVel;
        // cout << "lastPos = <" << _lastPos.x << "," << _lastPos.y << "," << _lastPos.z << ">\n";
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
    }
    
    case StageComplete: {
      if (stageChange) {
        cout << _name << "Landing complete. Idling.\n";
      }

      {
        EstSetPredictedValues(Vec3d(0, 0, 0), Vec3d(0, 0, 0));
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
    }

    default:
    case StageEmergency: {
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
  }
  cmdMsg.header.stamp = ros::Time::now();

  _pubCmd->publish(cmdMsg);

}

EstimatedState ExampleVehicleStateMachine::EstGetState(double const dt){
  switch (_estType) {
    case MocapEstimator:
        return _mocapEst->GetPrediction(dt);

    case GPSEstimator:
        return _gpsEst->GetCurrentEstimate();

    case OdometryEstimator:
        return _odometryEst->GetPrediction(dt);

    default:
        // You should decide on a default return value or perhaps throw an exception
        throw std::runtime_error("Unknown estimator type!");
  }
}

void ExampleVehicleStateMachine::EstSetPredictedValues(Vec3d angVel, Vec3d acceleration){
  switch (_estType) {
    case MocapEstimator:
        return _mocapEst->SetPredictedValues(angVel, acceleration);

    case GPSEstimator:
        // Do nothing for the GPS estimator. It does not depend on commands.
        return;

    case OdometryEstimator:
        return _odometryEst->SetPredictedValues(angVel, acceleration);
    default:
        // You should decide on a default return value or perhaps throw an exception
        throw std::runtime_error("Unknown estimator type!");
  }
}

unsigned ExampleVehicleStateMachine::EstGetID() const {
  switch (_estType) {
    case MocapEstimator:
        return _mocapEst->GetID();

    case GPSEstimator:
        return _gpsEst->GetID();

    case OdometryEstimator:
        return _odometryEst->GetID();
    default:
        // You should decide on a default return value or perhaps throw an exception
        throw std::runtime_error("Unknown estimator type!");
  }
}

double ExampleVehicleStateMachine::EstGetTimeSinceLastGoodMeasurement() const {
  switch (_estType) {
    case MocapEstimator:
        return _mocapEst->GetTimeSinceLastGoodMeasurement();

    case GPSEstimator:
        return _gpsEst->GetTimeSinceLastGoodMeasurement();

    case OdometryEstimator:
        return _odometryEst->GetTimeSinceLastGoodMeasurement();
    default:
        // You should decide on a default return value or perhaps throw an exception
        throw std::runtime_error("Unknown estimator type!");
  }
}

void ExampleVehicleStateMachine::PublishEstimate(EstimatedState estState) {
// Publish the current state estimate

  hiperlab_rostools::estimator_output estOutMsg;
  estOutMsg.header.stamp = ros::Time::now();
  estOutMsg.vehicleID = EstGetID();

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

  //Tell the estimator:
  EstSetPredictedValues(
      cmdAngVel,
      (estState.att * Vec3d(0, 0, 1) * cmdThrust - Vec3d(0, 0, 9.81)));

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

hiperlab_rostools::radio_command ExampleVehicleStateMachine::RunTrackingControllerAndUpdateEstimator(
    EstimatedState estState, Vec3d refPos,
    Vec3d refVel, Vec3d refAcc, double refThrust, Vec3d refAngVel,
    double &cmdThrustOutput, Rotationf &cmdAttOutput, Vec3d &cmdAngVelOutput) {

// Run the rates controller
  Vec3d cmdAngVel;
  double cmdThrust;
  Rotationf cmdAtt;

  _ctrl->RunTracking(estState.pos, estState.vel, estState.att, refPos, refVel,
                     refAcc, _desiredYawAngle, refThrust, refAngVel, cmdAngVel,
                     cmdThrust, cmdAtt);

  cmdThrustOutput = cmdThrust;
  cmdAttOutput = cmdAtt;
  cmdAngVelOutput = cmdAngVel;
  //Tell the estimator:
  EstSetPredictedValues(
      cmdAngVel,
      (estState.att * Vec3d(0, 0, 1) * cmdThrust - Vec3d(0, 0, 9.81)));

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
  if (!std::isnan(cmdThrust)) {
    // do not update when NaN
    _previousThrust = cmdThrust;  //used for the accelermation term of init state for generating new traj
  }

}

