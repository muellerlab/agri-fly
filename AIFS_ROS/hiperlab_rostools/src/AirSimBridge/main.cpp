/* A simple simulator.
 *
 */

#include <memory>
#include <mutex>
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

#include <ros/ros.h>

#include "Common/Time/ManualTimer.hpp"
#include "Common/Time/HardwareTimer.hpp"
#include "Components/Simulation/Quadcopter_T.hpp"
#include "Components/Simulation/UWBNetwork.hpp"
#include "Components/Simulation/CommunicationsDelay.hpp"

#include <fstream>

#include "hiperlab_rostools/mocap_output.h"
#include "hiperlab_rostools/simulator_truth.h"
#include "hiperlab_rostools/radio_command.h"
#include "hiperlab_rostools/telemetry.h"

// AirSim includes
#include "api/RpcLibClientBase.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"
#include <iostream> // for image saving

// OpenCV includes
#include <opencv/cv.hpp>

// Code for image publisher
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/Bool.h"
using namespace std;

class ImagePollAgent {
 public:
  std::shared_ptr<ros::Subscriber> depthPollTrigger;
  unsigned imageCount = 0;
  std::shared_ptr<Simulation::SimulationObject6DOF> vehicle;
  std::shared_ptr<image_transport::Publisher> pubDepthImage;
  std::shared_ptr<ros::Publisher> pubImageReceivedFlag;  //Publish a flag indicate that the image has been received. This unlocks the

  std::shared_ptr<MultirotorRpcLibClient> client;
  ofstream imageFetchlog;
  void initiateAirSimClient() {
    using namespace msr::airlib;
    client.reset(new MultirotorRpcLibClient("", 41451, 1));
    client->confirmConnection();  // Need to have env open and running (press play)
    imageFetchlog.open("depthImageFetcher.csv");
  }

  void callBackImagePoll(const std_msgs::Header &msg) {
    ros::Time pollTime = msg.stamp;
    ros::Time timeStartFetch = ros::Time::now();
    msr::airlib::vector<ImageCaptureBase::ImageRequest> request = {
        ImageCaptureBase::ImageRequest("0",
                                       ImageCaptureBase::ImageType::DepthVis,
                                       false, true) };
    const msr::airlib::vector<ImageCaptureBase::ImageResponse> &response =
        client->simGetImages(request);
    imageCount++;
    // We do not use the first two images, which often has erroneous setup due to the latency of initialization of renderer.
    for (const ImageCaptureBase::ImageResponse &image_info : response) {
      std::string path;
      cv::Mat depthImage;
      cv::Mat depthImage_uint8;

//        {
////            This part of the code saves image to local. Uncomment for debug purpose.
//                      path = "/home/clark/Documents/AirSim";
//                      char buffer[256];
//                      sprintf(buffer, "%04d", imageCount - 3);
//                      std::string str(buffer);
//                      std::string file_path = FileSystem::combine(path, "img" + str);
//                      std::ofstream file(file_path + ".png", std::ios::binary);
//                      file.write(
//                          reinterpret_cast<const char*>(image_info.image_data_uint8.data()),
//                          image_info.image_data_uint8.size());
//                      file.close();
//        }

      depthImage_uint8 = cv::imdecode(response.at(0).image_data_uint8,
                                      cv::IMREAD_UNCHANGED);

      ros::Time timeStartEncode = ros::Time::now();
      ros::Duration fetchImageTime = timeStartEncode - timeStartFetch;
      std_msgs::Header header;  // empty header
      header.seq = imageCount - 2;  // user defined counter
      header.stamp = ros::Time::now();  // time
      cv_bridge::CvImage img_bridge;
      sensor_msgs::Image img_msg;  // >> message to be sent

      img_bridge = cv_bridge::CvImage(header,
                                      sensor_msgs::image_encodings::RGB8,
                                      depthImage_uint8);

      img_bridge.toImageMsg(img_msg);  // from cv_bridge to sensor_msgs::Image
      pubDepthImage->publish(img_msg);  // ros::Publisher pub_img = node.advertise<sensor_msgs::Image>("topic", queuesize);
      ros::Time timePublish = ros::Time::now();

      std_msgs::Header imageReceived;  // empty header
      imageReceived.seq = imageCount;  // user defined counter
      imageReceived.stamp = ros::Time::now();  // time
      pubImageReceivedFlag->publish(imageReceived);
      ros::Duration convertImageTime = timePublish - timeStartEncode;
      if (imageCount < 1000) {
        imageFetchlog << imageCount - 2 << ",";
        imageFetchlog << fetchImageTime.toSec() << ",";
        imageFetchlog << convertImageTime.toSec() << ",";
        imageFetchlog << "\n";
      } else if (imageCount == 1000) {
        imageFetchlog.close();
      }
    }
    return;
  }
};

int main(int argc, char **argv) {
  ////////////////////////////////////////////////////////////////
  //ROS setup
  ////////////////////////////////////////////////////////////////
  ros::init(argc, argv, "AirSimBridge");
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  cout << "ros setup.\n";
  std::shared_ptr<ImagePollAgent> depthImageAgent;
  depthImageAgent.reset(new ImagePollAgent());
  depthImageAgent->initiateAirSimClient();
  depthImageAgent->depthPollTrigger.reset(
      new ros::Subscriber(
          n.subscribe("imagePoll", 1, &ImagePollAgent::callBackImagePoll,
                      depthImageAgent.get())));
  depthImageAgent->pubDepthImage.reset(
      new image_transport::Publisher(it.advertise("depthImage", 1)));

  depthImageAgent->pubImageReceivedFlag.reset(
      new ros::Publisher(
          n.advertise<std_msgs::Header>("imageReceivedFlag", 1)));

  cout << "Publisher setup.\n";
////////////////////////////////////////////////////////////////
//Simulator setup
////////////////////////////////////////////////////////////////
//Basic timing:
//  const double frequencySimulation = 500;
//  ros::Rate loop_rate(frequencySimulation);
  while (ros::ok()) {
    ros::spinOnce();
  }
//  loop_rate.sleep();
}
