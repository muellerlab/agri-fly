/* A simple simulator for a single quadcopter.
 *
 */

#include <memory>
#include <Eigen/Dense>
#include <stdio.h>
#include <unistd.h>
#include <boost/program_options.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <ctime>
#include <opencv2/opencv.hpp>
#include <random>
#include <string>

#include "Common/Math/Vec3.hpp"
#include "Common/Math/Trajectory.hpp"
#include "Components/TrajectoryGenerator/RapidTrajectoryGenerator.hpp"

#include "Common/Time/ManualTimer.hpp"
#include "Common/Time/HardwareTimer.hpp"
#include "Components/Simulation/Quadcopter_T.hpp"
#include "Components/Simulation/UWBNetwork.hpp"
#include "Components/Simulation/CommunicationsDelay.hpp"
#include "Components/Simulation/ArucoCamera.hpp"

#include "Components/Offboard/MocapStateEstimator.hpp"
#include "Components/Offboard/QuadcopterController.hpp"
#include "Components/Offboard/SafetyNet.hpp"

#include "Components/Logic/QuadcopterLogic.hpp"

#include <fstream>

// AirSim includes
//#include "api/RpcLibClientBase.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"
#include <iostream> // for image saving

#include "Components/DepthImagePlanner/DepthImagePlanner.hpp"

using namespace std;
using namespace Offboard;
using namespace CommonMath;
using namespace RapidQuadrocopterTrajectoryGenerator;
using namespace RectangularPyramidPlanner;

enum {
  CTRL_ONBOARD_UWB,
  CTRL_OFFBOARD_RATES,
} controllerType;

template<typename Real>
string toCSV(const Vec3<Real> v) {
  stringstream ss;
  ss << v.x << "," << v.y << "," << v.z << ",";
  return ss.str();
}
string toCSV(const Rotationf r) {
  stringstream ss;
  ss << r[0] << "," << r[1] << "," << r[2] << "," << r[3] << ",";
  return ss.str();
}
string toCSV(const std::vector<float> v) {
  stringstream ss;
  for (int i = 0; i < v.size(); i++) {
    ss << v[i] << ",";
  }
  return ss.str();
}

class ExplorationCost {
 public:
  ExplorationCost(Vec3d estPos, Rotationd estAtt, Vec3d goalWorld,
                  Rotationd const depthCamAtt) {
    _estPos = estPos;
    _estAtt = estAtt;
    _goalWorld = goalWorld;
    _depthCamAtt = depthCamAtt;
  }
  void updateWorld(Vec3d estPos, Rotationd estAtt, Vec3d goalWorld) {
    _estPos = estPos;
    _estAtt = estAtt;
    _goalWorld = goalWorld;
  }
  static double GetTrajCostWrapper(
      void *ptr2obj,
      RapidQuadrocopterTrajectoryGenerator::RapidTrajectoryGenerator &traj) {
    ExplorationCost *explorationCost = (ExplorationCost*) ptr2obj;
    return explorationCost->GetTrajCost(traj);
  }

  double GetTrajCost(
      RapidQuadrocopterTrajectoryGenerator::RapidTrajectoryGenerator &traj) {
    double duration = traj.GetFinalTime();
    Vec3d Pi_C = traj.GetPosition(duration);  // position of camera at end of trajectory relative to start written in camera frame

    Vec3d G_W = _goalWorld;                               // goal in world frame
    Vec3d G_C = _depthCamAtt.Inverse() * _estAtt.Inverse()
        * (_goalWorld - _estPos);  // goal in camera frame
    Vec3d S_C = Vec3d(0, 0, 0);                // starting point in camera frame

    double SG = (G_C - S_C).GetNorm2();
    double PiG = (G_C - Pi_C).GetNorm2();
    return -(SG - PiG) / duration;
  }
 private:
  Vec3d _goalWorld;
  Rotationd _depthCamAtt;
  Vec3d _estPos;
  Rotationd _estAtt;
};

int main(void) {
////////////////////////////////////////////

//depthImageSetting
  // Far and near clipping plane distance in Unity
  double far = 10.0;
  double depthScale = far / (256.0);  //output depth image in uint_8
  Rotationd const depthCamAtt = Rotationd::FromEulerYPR(-90.0 * M_PI / 180.0,
                                                        0 * M_PI / 180.0,
                                                        -90 * M_PI / 180.0);

  cv::Mat depthImage;
  cv::Mat depthImage8;
  double focalLength;
  unsigned depthImageHeight;
  unsigned depthImageWidth;
  Rotationd trajAtt;
  Vec3d trajOffset;
  bool imageReady;
  unsigned plannedTrajCount = 0;

  std::shared_ptr<RapidQuadrocopterTrajectoryGenerator::RapidTrajectoryGenerator> _traj;

//Basic timing:
  const double dt = 1.0 / 500.0;  //run the simulation at this rate
  const double startFlightTime = 5.0f;  // time for rappids to begin
  const double endTime = 8.0f;  //[s]  total simulation time
  ManualTimer simTimer;  // Tracks time in the simulation
  HardwareTimer globalTimer;  // Tracks the actual wall clock time (in the real world)

//Create the quadcopter:
  uint8_t vehicleId = 1;  //For UWB network, commands, etc.
  Onboard::QuadcopterConstants::QuadcopterType quadcopterType =
      Onboard::QuadcopterConstants::GetVehicleTypeFromID(vehicleId);
  Onboard::QuadcopterConstants vehConsts(quadcopterType);
//Create the quadcopter:
  double const mass = vehConsts.mass;  //[kg]
  double const inertia_xx = vehConsts.inertia_xx;  //[kg.m**2]
  double const inertia_yy = inertia_xx;  //[kg.m**2]
  double const inertia_zz = vehConsts.inertia_zz;  //[kg.m**2]
  double armLength = vehConsts.armLength;  //[m]
  double propThrustFromSpeedSqr = vehConsts.propellerThrustFromSpeedSqr;  //[N/(rad/s)**2]
  double propTorqueFromSpeedSqr = vehConsts.propellerTorqueFromThrust
      * vehConsts.propellerThrustFromSpeedSqr;

  double motorTimeConst = vehConsts.motorTimeConst;  // [s]
  double motorInertia = vehConsts.motorInertia;  // [kg.m**2]
  double motorMinSpeed = vehConsts.motorMinSpeed;  // [rad/s]
  double motorMaxSpeed = vehConsts.motorMaxSpeed;  // [rad/s]

//Estimate radius for planning
  double const physicalVehicleRadius = armLength * 2;
  double const vehicleRadiusPlanning = armLength * 2 * 1.5;
  double const minCollisionDist = 0.5;  //[m]

  Vec3d centreOfMassError = Vec3d(0, 0, 0);

// Mocap system:
  double const periodMocapSystem = 1.0 / 200.0;  //[s] time between mocap measurments
  double const periodOffboardMainLoop = 1.0 / 100.0;  //[s] time between offboard main loop runs
  double const periodTelemetryLoop = 1.0 / 100.0;  //[s] time between telemetry update
  double const periodOnboardLogic = 1.0 / 500.0;  //[s] time between onboard logic runs
  double const timeDelayOffboardControlLoopTrue = 0.03;  //[s]
  double const timeDelayOffboardControlLoopEstimate = 0.03;  //[s]

// AirSim client setup
  using namespace msr::airlib;
  MultirotorRpcLibClient client("", 41451, 1);
  client.confirmConnection();  // Need to have env open and running (press play)

// AirSim image capture setup
  typedef ImageCaptureBase::ImageRequest ImageRequest;
  typedef ImageCaptureBase::ImageResponse ImageResponse;
  typedef ImageCaptureBase::ImageType ImageType;
  typedef common_utils::FileSystem FileSystem;
  bool recordImage = false;  // Toggle this to record the images generated by Unity.

  /*
   * Essentially, once the simulation timer reaches a time beyond the periodBetweenImages variable
   * it will wait for Unity to generate an image. It is Unity's job
   * to let this simulator know that it has generated an image, so that the simulator may proceed.
   */

  bool requestNewImage = true;
  double const desiredFrameRate = 30.0;
  double const periodBetweenImages = 1 / desiredFrameRate;

  Eigen::Matrix<double, 3, 3> inertiaMatrix;
  inertiaMatrix << inertia_xx, 0, 0, 0, inertia_yy, 0, 0, 0, inertia_zz;

//drag coefficients
  Vec3d linDragCoeffB = Vec3d(vehConsts.linDragCoeffBx,
                              vehConsts.linDragCoeffBy,
                              vehConsts.linDragCoeffBz);

  std::shared_ptr<Simulation::Quadcopter> quad;
  quad.reset(
      new Simulation::Quadcopter(&simTimer, mass, inertiaMatrix, armLength,
                                 centreOfMassError, motorMinSpeed,
                                 motorMaxSpeed, propThrustFromSpeedSqr,
                                 propTorqueFromSpeedSqr, motorTimeConst,
                                 motorInertia, linDragCoeffB, vehicleId,
                                 quadcopterType, periodOnboardLogic));

//Offboard estimation / control code:
  std::shared_ptr<MocapStateEstimator> est;
  est.reset(
      new MocapStateEstimator(&simTimer, vehicleId,
                              timeDelayOffboardControlLoopEstimate));
  QuadcopterController ctrl;
  SafetyNet safetyNet;
  ctrl.SetParameters(vehConsts.posControl_natFreq, vehConsts.posControl_damping,
                     vehConsts.attControl_timeConst_xy,
                     vehConsts.attControl_timeConst_z);

  controllerType = CTRL_OFFBOARD_RATES;

//create an initial error:
  Vec3d const initErrPos = Vec3d(0, 0, 0);
  Rotationd const initErrAtt = Rotationd::FromEulerYPR(0 * M_PI / 180.0,
                                                       0 * M_PI / 180.0, 0);

//where we want the quadcopter to fly to:
  Vec3d desiredPosition(0, 0, 3.5);
  Vec3d desiredVelocity(0, 0, 0);
  Vec3d desiredAcc(0, 0, 0);
  Vec3d desiredAngVel(0, 0, 0);
  double desiredThrust;
  double desYawAngleDeg = 0;  //[deg]
  Vec3d const goalWorld(120.0, 0.0, 3.5);

//keep track of last set of inputs
  double previousThrust;
  Vec3d previousAngVel;

//file for logging:
  ofstream logfile;
  ofstream logfile2;
  ofstream trajLog;
  ofstream airsimSetPoseTimeLog;
  ofstream airsimGetPoseTimeLog;

  logfile.open("Logs/rappids_simulator/simulation.csv");
  logfile2.open("Logs/rappids_simulator/PictureFrequencyLog.csv");
  trajLog.open("Logs/rappids_simulator/PlannedTrajectory.csv");

  airsimSetPoseTimeLog.open("Logs/rappids_simulator/airsimSetPoseTimeLog.csv");
  airsimGetPoseTimeLog.open("Logs/rappids_simulator/airsimGetPoseTimeLog.csv");

// create header, a lot of columns
  logfile
      << "t,posx,posy,posz,velx,vely,velz,attY,attP,attR,angvelx,angvely,angvelz,m1,m2,m3,m4,"
//          "tx,ty,tz,vx,vy,vz,qx,qy,qz,qw,Pr11,Pr12,Pr13,Pr22,Pr23,Pr33,Pt11,Pt12,Pt13,Pt22,Pt23,Pt33,"
          "estposx,estposy,estposz,estvelx,estvely,estvelz,esty,estp,estr,estangx,estangy,estangz,"
          "desposx,desposy,desposz,desvelx,desvely,desvelz,panic,r1,r2,r3,r4\n";
  logfile2 << "realTime,SimulationTime,picTime,picNum\n";
  printf("Starting simulation\n");

  Timer t(&simTimer);
  Timer tReal(&globalTimer);
  Timer tPic(&globalTimer);  //Time to track the seconds between pics.
  Timer tAirSimLoop(&globalTimer);  //Measures the time between set airsim pose to pose being updated;
  Timer tGetPose(&globalTimer);  //Measures the time between two instincts between getting airsim getpose;
  quad->SetPosition(initErrPos);
  quad->SetAttitude(initErrAtt);

  Simulation::CommunicationsDelay<RadioTypes::RadioMessageDecoded::RawMessage> cmdRadioChannel(
      &simTimer, timeDelayOffboardControlLoopTrue);

  Timer timerPrint(&simTimer);
  Timer timerMocap(&simTimer);
  Timer timerOffboardMainLoop(&simTimer);
  Timer timerRequestNewImage(&simTimer);
  Timer timerTelemetryLoop(&simTimer);

  Timer trackTrajTime(&simTimer);
  bool trajPlanned = false;
  double trajEndTime;

  float lastRadioCommand[4];
  for (int i = 0; i < 4; i++) {
    lastRadioCommand[i] = 0;
  }
// Hack for now: Set the start x-y position of the airsim Position as 0,0

  auto initPosition = client.getMultirotorState().getPosition();
  double const initX = initPosition.x();
  double const initY = initPosition.y();

  printf("initx=%.3fs\n", initX);
  printf("inity=%.3fs\n", initY);

  unsigned imageCount = 0;
  tPic.Reset();
// aligns airsim cameras using openVINS json camera extrinsics
// set TCtoI in airsim
  msr::airlib::Pose airsim_pose;
  airsim_pose.position[0] = initX;
  airsim_pose.position[1] = initY;
  airsim_pose.position[2] = 0.0;
  airsim_pose.orientation.w() = 1.0;
  airsim_pose.orientation.x() = 0.0;
  airsimGetPoseTimeLog << tGetPose.GetSeconds<double>() << "\n";
  airsim_pose.orientation.y() = 0.0;
  airsim_pose.orientation.z() = 0.0;
  client.simSetVehiclePose(airsim_pose, true);

  bool isStereo = false;
  bool measCSVNew = false;
  string measCSV;

  ExplorationCost expCost(Vec3d(0.0, 0.0, 0.0), Rotationd::Identity(),
                          Vec3d(0.0, 0.0, 0.0), depthCamAtt);

  while (t.GetSeconds<double>() < endTime) {
    if (requestNewImage) {
      msr::airlib::vector<ImageRequest> request = { ImageRequest(
          "0", ImageType::DepthVis, false, true) };
      const msr::airlib::vector<ImageResponse> &response = client.simGetImages(
          request);

      // We do not store the first three images, which often has erroneous setup due to the latency of initialization of Unity simulator.
      if (imageCount > 3) {

        for (const ImageResponse &image_info : response) {
          std::string path;
          path = "/home/clark/Documents/AirSim";
          char buffer[256];
          sprintf(buffer, "%04d", imageCount - 3);
          std::string str(buffer);
          std::string file_path = FileSystem::combine(path, "img" + str);
          std::ofstream file(file_path + ".png", std::ios::binary);
          file.write(
              reinterpret_cast<const char*>(image_info.image_data_uint8.data()),
              image_info.image_data_uint8.size());
          file.close();
          depthImage8 = cv::imdecode(response.at(0).image_data_uint8,
                                     cv::IMREAD_UNCHANGED);
          depthImage8.convertTo(depthImage, CV_16U);

        }

        depthImageHeight = response.at(0).height;
        depthImageWidth = response.at(0).width;
        focalLength = depthImageWidth / 2.0;

        // Plot out the smallest depth in picture
        uint16_t *depthData;
        depthData = reinterpret_cast<uint16_t*>(depthImage.data);

        unsigned smallestDepth(1000);

        for (unsigned i = 0; i < depthImageWidth; i++) {
          for (unsigned j = 0; j < depthImageHeight; j++)
            if ((depthData[i * depthImageWidth + j] < smallestDepth)
                && (depthData[i * depthImageWidth + j] > 0)) {
              smallestDepth = depthData[i * depthImageWidth + j];
            }
        }

        double closestObstacle = smallestDepth * depthScale;
        printf("closest=%.8fs\n", closestObstacle);

        requestNewImage = false;  // Continue running the simulation loop.
        imageReady = true;  // We have an image for planning for this round.
        logfile2 << tReal.GetSeconds<double>() << ",";
        logfile2 << t.GetSeconds<double>() << ",";
        logfile2 << tPic.GetSeconds<double>() << ",";
        tPic.Reset();
        logfile2 << imageCount << ",";
        logfile2 << "\n";
      }
      imageCount++;
    }

    quad->Run();
    simTimer.AdvanceMicroSeconds(uint64_t(dt * 1e6));

    Vec3d simTruthPos(quad->GetPosition());
    Rotationd simTruthAtt(quad->GetAttitude());

    msr::airlib::Pose airsim_pose;

    airsim_pose.position[0] = simTruthPos.x + initX;
    airsim_pose.position[1] = -simTruthPos.y + initY;
    airsim_pose.position[2] = -simTruthPos.z;

    airsim_pose.orientation.w() = simTruthAtt[0];
    airsim_pose.orientation.x() = simTruthAtt[1];
    airsim_pose.orientation.y() = -simTruthAtt[2];
    airsim_pose.orientation.z() = -simTruthAtt[3];

    client.simSetVehiclePose(airsim_pose, true);
    tAirSimLoop.Reset();  //Measures the time between set airsim pose to pose being updated;

    if (t.GetSeconds<double>() > startFlightTime) {
      bool wait_flag = true;
      msr::airlib::Pose receivedAirSimPose;
      Vec3d receivedPos;
      double PosDiffThreshold = 0.001;
      while (wait_flag) {
        // wait for the AirSim communication, until unity reach same state as simulation
        tGetPose.Reset();
        receivedAirSimPose = client.simGetVehiclePose();
        airsimGetPoseTimeLog << t.GetSeconds<double>() << ",";
        airsimGetPoseTimeLog << tGetPose.GetSeconds<double>() << ",";

        receivedPos[0] = receivedAirSimPose.position[0] - initX;
        receivedPos[1] = -(receivedAirSimPose.position[1] - initY);
        receivedPos[2] = -receivedAirSimPose.position[2];
        Vec3d PosDiff = receivedPos - simTruthPos;
        double PosDiff_d = PosDiff.GetNorm2();

        airsimGetPoseTimeLog << receivedPos[0] << ",";
        airsimGetPoseTimeLog << receivedPos[1] << ",";
        airsimGetPoseTimeLog << receivedPos[2] << ",";
        airsimGetPoseTimeLog << "\n";
        if (PosDiff_d < PosDiffThreshold) {
          wait_flag = false;
        }
      }
      airsimSetPoseTimeLog << tAirSimLoop.GetSeconds<double>() << "\n";
    }

    if (timerRequestNewImage.GetSeconds<double>() > periodBetweenImages) {
      requestNewImage = true;  // We wait until we get an image.
      // If we don't this part of loop won't run anymore.
      timerRequestNewImage.AdjustTimeBySeconds(-periodBetweenImages);
    }

    if (timerPrint.GetSeconds<double>() >= 1) {
      timerPrint.AdjustTimeBySeconds(-1);
      printf("Current sim time = %.1fs\n", t.GetSeconds<double>());
    }

    if (timerMocap.GetSeconds<double>() > periodMocapSystem) {
      //simulate mocap packets over the network
      timerMocap.AdjustTimeBySeconds(-periodMocapSystem);
      Vec3d measPos(quad->GetPosition());
      Rotationd measAtt(quad->GetAttitude());
      est->UpdateWithMeasurement(measPos, measAtt);
    }

    if (timerTelemetryLoop.GetSeconds<double>() > periodTelemetryLoop) {
      timerTelemetryLoop.AdjustTimeBySeconds(-periodTelemetryLoop);
      TelemetryPacket::data_packet_t dataPacketRaw1, dataPacketRaw2;
      quad->GetTelemetryDataPackets(dataPacketRaw1, dataPacketRaw2);
      TelemetryPacket::TelemetryPacket dataPacket;
      TelemetryPacket::DecodeTelemetryPacket(dataPacketRaw1, dataPacket);
      TelemetryPacket::DecodeTelemetryPacket(dataPacketRaw2, dataPacket);
    }

    MocapStateEstimator::MocapEstimatedState estState;
    estState = est->GetPrediction(timeDelayOffboardControlLoopEstimate);

    if (timerOffboardMainLoop.GetSeconds<double>() > periodOffboardMainLoop) {

      expCost.updateWorld(estState.pos, estState.att, goalWorld);

      //Simulate the offboard main loop
      timerOffboardMainLoop.AdjustTimeBySeconds(-periodOffboardMainLoop);

      if (t.GetSeconds<double>() > startFlightTime) {
        if (imageReady) {
//          cv::imwrite(
//              "/home/clark/Documents/Repos/LabCode/GeneralCode/Logs/rappids_simulator/depthImage.bmp",
//              depthImage);

          DepthImagePlanner planner(depthImage, depthScale, focalLength,
                                    depthImageWidth / 2.0,
                                    depthImageHeight / 2.0,
                                    physicalVehicleRadius,
                                    vehicleRadiusPlanning, minCollisionDist);

          Vec3d vel = depthCamAtt.Inverse() * estState.att.Inverse()
              * estState.vel;
          Vec3d acc = depthCamAtt.Inverse() * estState.att.Inverse()
              * (Vec3d(0, 0, 1) * previousThrust - Vec3d(0, 0, 9.81));
          Vec3d g = depthCamAtt.Inverse() * estState.att.Inverse()
              * Vec3d(0, 0, -9.81);
          RapidQuadrocopterTrajectoryGenerator::RapidTrajectoryGenerator candidateTraj(
              Vec3d(0, 0, 0), vel, acc, g);
          double compTime = 0.05;

          std::vector<TrajectoryTest> trajectories;
          bool planResult = planner.FindLowestCostTrajectoryRandomCandidates(
              candidateTraj, trajectories, compTime, (void*) &expCost,
              &ExplorationCost::GetTrajCostWrapper);

//          bool planResult = planner.FindLowestCostTrajectoryRandomCandidates(
//              candidateTraj, compTime, (void*) &expCost,
//              &ExplorationCost::GetTrajCostWrapper);

          Vec3d exploration_direction(0, 0, 1);

//          double compTime = 0.02;
//          bool planResult = planner.FindFastestTrajRandomCandidates(
//              candidateTraj, compTime, exploration_direction);

          // printf("Plan result = %d", planResult);
          // Record the rotation between the trajectory and the world frame

          if (planResult) {
//            printf("traj found \n");
            trajAtt = estState.att * depthCamAtt;
            double traj_y, traj_p, traj_r;
            trajAtt.ToEulerYPR(traj_y, traj_p, traj_r);
            trajOffset = estState.pos;
            trackTrajTime.Reset();
            trajPlanned = true;
            _traj = std::make_shared
                < RapidQuadrocopterTrajectoryGenerator::RapidTrajectoryGenerator
                > (candidateTraj);

            Trajectory trajToRecord = candidateTraj.GetTrajectory();
            std::vector<Vec3d> trajCoeffs = trajToRecord.GetCoeffs();
            plannedTrajCount++;

            // Record the trajectory
            trajLog << plannedTrajCount << ",";
            trajLog << trajCoeffs[0].x << "," << trajCoeffs[0].y << ","
                << trajCoeffs[0].z << ",";
            trajLog << trajCoeffs[1].x << "," << trajCoeffs[1].y << ","
                << trajCoeffs[1].z << ",";
            trajLog << trajCoeffs[2].x << "," << trajCoeffs[2].y << ","
                << trajCoeffs[2].z << ",";
            trajLog << trajCoeffs[3].x << "," << trajCoeffs[3].y << ","
                << trajCoeffs[3].z << ",";
            trajLog << trajCoeffs[4].x << "," << trajCoeffs[4].y << ","
                << trajCoeffs[4].z << ",";
            trajLog << trajCoeffs[5].x << "," << trajCoeffs[5].y << ","
                << trajCoeffs[5].z << ",";
            trajLog << traj_y << "," << traj_p << "," << traj_r << ",";
            trajLog << trajOffset.x << "," << trajOffset.y << ","
                << trajOffset.z << ",";
            trajEndTime = trajToRecord.GetEndTime();
            trajLog << trajToRecord.GetStartTime() << ","
                << trajToRecord.GetEndTime() << "\n";
            imageReady = false;
          }
        }

        double traj_t = trackTrajTime.GetSeconds<double>();
        Vec3d trajPos, trajVel, trajAcc;

        if (not trajPlanned) {
          desiredPosition = Vec3d(0, 0, 2);
          desiredVelocity = Vec3d(0, 0, 0);
          desiredAcc = Vec3d(0, 0, 0);
        } else {
          if (traj_t < trajEndTime) {
            traj_t += 0.04;
            trajPos = _traj->GetPosition(traj_t);
            trajVel = _traj->GetVelocity(traj_t);
            trajAcc = _traj->GetAcceleration(traj_t);
          } else {
            trajPos = _traj->GetPosition(trajEndTime);
            trajVel = Vec3d(0, 0, 0);
            trajAcc = Vec3d(0, 0, 0);
          }

          // Disallow the vehicle from going backwards
          // This can occur if the velocity estimate is noisy, which will result in trajectories
          // that move backwards briefly before moving into the camera FOV
          // However, applying these trajectories in a receding horizon fashion will result
          // in only the portion of the trajectory that moves backwards being executed, resulting
          // in the vehicle slowly drifting backwards
          if (trajPos.z < 0) {
            trajPos.z = 0;
            // Only saturate velocity and acceleration if the trajectory is actually behind the camera
            // (We don't want to prevent the vehicle from slowing down at the end of a trajectory)
            if (trajVel.z < 0) {
              trajVel.z = 0;
            }
            if (trajAcc.z < 0) {
              trajAcc.z = 0;
            }
          }

          Vec3d refPos = trajAtt * trajPos + trajOffset;
          Vec3d refVel = trajAtt * trajVel;
          Vec3d refAcc = trajAtt * trajAcc;
          double refThrust = _traj->GetThrust(traj_t);
          // Rotate angular velocity into body frame
          Vec3d refAngVel = estState.att.Inverse() * trajAtt
              * _traj->GetOmega(traj_t, 0.02);

          desiredPosition = refPos;
          desiredVelocity = refVel;
          desiredAcc = refAcc;
          desiredAngVel = refAngVel;
          desiredThrust = refThrust;
        }
      }

      //Publish the commands:
      RadioTypes::RadioMessageDecoded::RawMessage rawMsg;

      if (controllerType == CTRL_OFFBOARD_RATES) {
        safetyNet.UpdateWithEstimator(estState,
                                      est->GetTimeSinceLastGoodMeasurement());

        Vec3d cmdAngVel;
        double cmdThrust;
        Rotationf cmdAtt;
        uint8_t flags = 0;

        if (t.GetSeconds<double>() < startFlightTime) {

          ctrl.Run(estState.pos, estState.vel, estState.att, desiredPosition,
                   Vec3d(0, 0, 0), Vec3d(0, 0, 0),
                   desYawAngleDeg * M_PI / 180.0, cmdAngVel, cmdThrust);
        } else {

          ctrl.RunTracking(estState.pos, estState.vel, estState.att,
                           desiredPosition, desiredVelocity, desiredAcc,
                           desYawAngleDeg * M_PI / 180.0, desiredThrust,
                           desiredAngVel, cmdAngVel, cmdThrust, cmdAtt);
        }
        RadioTypes::RadioMessageDecoded::CreateRatesCommand(flags,
                                                            float(cmdThrust),
                                                            Vec3f(cmdAngVel),
                                                            rawMsg.raw);

        previousThrust = cmdThrust;
        previousAngVel = cmdAngVel;

        lastRadioCommand[0] = cmdThrust;
        lastRadioCommand[1] = cmdAngVel.x;
        lastRadioCommand[2] = cmdAngVel.y;
        lastRadioCommand[3] = cmdAngVel.z;
        est->SetPredictedValues(
            cmdAngVel,
            (estState.att * Vec3d(0, 0, 1) * cmdThrust - Vec3d(0, 0, 9.81)));

        //We disable this for now, since it triggers on the first run.
        //        if (!safetyNet.GetIsSafe()) {
        //          RadioTypes::RadioMessageDecoded::CreateKillOneCommand(vehicleId, 0,
        //                                                                rawMsg.raw);
        //        }
      } else if (controllerType == CTRL_ONBOARD_UWB) {
        uint8_t flags = 0;
        RadioTypes::RadioMessageDecoded::CreatePositionCommand(flags,
                                                               desiredPosition,
                                                               desiredVelocity,
                                                               Vec3d(0, 0, 0),  //zero acceleration
                                                               rawMsg.raw);
      }

      TelemetryPacket::data_packet_t dataPacketRaw1, dataPacketRaw2;
      quad->GetTelemetryDataPackets(dataPacketRaw1, dataPacketRaw2);

      TelemetryPacket::TelemetryPacket dataPacket;
      TelemetryPacket::DecodeTelemetryPacket(dataPacketRaw1, dataPacket);
      TelemetryPacket::DecodeTelemetryPacket(dataPacketRaw2, dataPacket);

      //add message to command radio queue
      cmdRadioChannel.AddMessage(rawMsg);

      //Write the simulation state to a file:
      logfile << t.GetSeconds<double>() << ",";
      logfile << toCSV(quad->GetPosition());
      logfile << toCSV(quad->GetVelocity());
      logfile << toCSV(quad->GetAttitude().ToEulerYPR());
//    logfile << toCSV(quad->GetAttitude());
      logfile << toCSV(quad->GetAngularVelocity());
      logfile << dataPacket.motorForces[0] << ",";
      logfile << dataPacket.motorForces[1] << ",";
      logfile << dataPacket.motorForces[2] << ",";
      logfile << dataPacket.motorForces[3] << ",";

      // get openVINS state
//        ov_est_state = ov_est->get_state(t.GetSeconds<float>());
//        logfile << toCSV(ov_est_state.pos);
//        logfile << toCSV(ov_est_state.vel);
//        logfile << toCSV(ov_est_state.q);
//        logfile << toCSV(ov_est_state.cov_upper);

      //estimator state:
      Vec3f pos;
      Vec3f vel;
      Rotationf att;
      Vec3f angVel;
      if (controllerType != CTRL_ONBOARD_UWB) {
        MocapStateEstimator::MocapEstimatedState estState = est->GetPrediction(
            0);
        pos = estState.pos;
        vel = estState.vel;
        att = estState.att;
        angVel = estState.angVel;
      } else {
        pos = Vec3d(dataPacket.position);
        vel = Vec3d(dataPacket.velocity);
        att = Rotationd::FromVectorPartOfQuaternion(Vec3d(dataPacket.attitude));
        angVel = Vec3d(dataPacket.gyro);
      }

      logfile << toCSV(pos);
      logfile << toCSV(vel);
      logfile << toCSV(att.ToEulerYPR());
      logfile << toCSV(angVel);

      //desired position:
      logfile << toCSV(desiredPosition);
      logfile << toCSV(desiredVelocity);

      logfile << int(dataPacket.panicReason) << ",";

      for (int i = 0; i < 4; i++) {
        logfile << double(lastRadioCommand[i]) << ",";
      }

      // Adding measured values to the logfile
      if (measCSVNew) {
        logfile << measCSV;
        measCSVNew = false;
      }
      logfile << "\n";
    }  //main loop

    //Check if we have a new message to transmit:
    if (cmdRadioChannel.HaveNewMessage()) {
      quad->SetCommandRadioMsg(cmdRadioChannel.GetMessage());
    }

  }  //sim loop
  printf("Done.\n");
//  ov_est->end_sim();
  logfile.close();
  logfile2.close();
  trajLog.close();
  airsimGetPoseTimeLog.close();
  airsimSetPoseTimeLog.close();
}

