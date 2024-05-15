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

// Code for image publisher
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/Bool.h"
using namespace std;

class ImagePollAgent {
 public:
  ImagePollAgent(int requestedImage1, int requestedImage2) {
    imageType = requestedImage1;
    imageType2 = requestedImage2;
  }
  ImagePollAgent(int requestedImage1 = 3) {
    imageType = requestedImage1;
  }

  std::shared_ptr<ros::Subscriber> depthPollTrigger;
  unsigned imageCount = 0;
  std::shared_ptr<Simulation::SimulationObject6DOF> vehicle;
  std::shared_ptr<image_transport::Publisher> pubDepthImage;
  std::shared_ptr<image_transport::Publisher> pubRGBImage;
  std::shared_ptr<ros::Publisher> pubImageReceivedFlag;  //Publish a flag indicate that the image has been received. This unlocks the
  int imageType = 3, imageType2 = -1;
  std::shared_ptr<MultirotorRpcLibClient> client;
  ofstream imageFetchlog;
  void initiateAirSimClient() {
    using namespace msr::airlib;
    client.reset(new MultirotorRpcLibClient("", 41451, 100));
    client->confirmConnection();  // Need to have env open and running (press play)
    imageFetchlog.open("depthImageFetcher.csv");
  }

  void callBackImagePoll(const std_msgs::Header &msg) {
    ros::Time pollTime = msg.stamp;
    ros::Time timeStartFetch = ros::Time::now();

    ImageCaptureBase::ImageType camera1 =
        static_cast<ImageCaptureBase::ImageType>(imageType);
    ImageCaptureBase::ImageType camera2;
    msr::airlib::vector<ImageCaptureBase::ImageRequest> request;

    if (imageType2 == -1) {
      request = { ImageCaptureBase::ImageRequest("0", camera1, false, true) };
    } else {
      camera2 = static_cast<ImageCaptureBase::ImageType>(imageType2);
      //request = { ImageCaptureBase::ImageRequest("0", camera1, false, true),
      //    ImageCaptureBase::ImageRequest("0", camera2, false, true) };
      request = { ImageCaptureBase::ImageRequest("0", camera2, false, true),
          ImageCaptureBase::ImageRequest("0", camera1, false, true) };
    }

    const msr::airlib::vector<ImageCaptureBase::ImageResponse> &response =
        client->simGetImages(request);
    imageCount++;
// We do not use the first two images, which often has erroneous setup due to the latency of initialization of renderer.
    for (const ImageCaptureBase::ImageResponse &image_info : response) {
      std::string path;
      cv::Mat depthImage;
      cv::Mat depthImage_uint8;
      cv::Mat rgbImage_uint8;
      /*{
       //            This part of the code saves image to local. Uncomment for debug purpose.
       char *homdir = getenv("HOME");
       string homdirstr = homdir;
       path = homdirstr + "/Documents/AirSim";
       char buffer[256];
       sprintf(buffer, "%04d", imageCount - 3);
       std::string str(buffer);
       std::string file_path = common_utils::FileSystem::combine(path,
       "img" + str);
       std::ofstream file(file_path + ".png", std::ios::binary);
       file.write(
       reinterpret_cast<const char*>(response.at(0).image_data_uint8.data()),  //image_info.image_data_uint8.data()),
       image_info.image_data_uint8.size());
       file.close();
       } */

      depthImage_uint8 = cv::imdecode(response.at(0).image_data_uint8,
                                      cv::IMREAD_UNCHANGED);
      //decode RGB image if requesting 2 images:
      if (imageType2 != -1) {
        rgbImage_uint8 = cv::imdecode(response.at(1).image_data_uint8,
                                      cv::IMREAD_UNCHANGED);
      }

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
      //ros::Time timePublish = ros::Time::now();

      //Publish RGB image:
      if (imageType2 != -1) {
        std_msgs::Header header2;  // empty header
        header2.seq = imageCount - 2;  // user defined counter
        header2.stamp = ros::Time::now();  // time
        cv_bridge::CvImage img_bridge2;
        sensor_msgs::Image img_msg2;  // >> message to be sent

        img_bridge2 = cv_bridge::CvImage(header2,
                                         sensor_msgs::image_encodings::RGB8,
                                         rgbImage_uint8);

        img_bridge2.toImageMsg(img_msg2);  // from cv_bridge to sensor_msgs::Image
        pubRGBImage->publish(img_msg2);  // ros::Publisher pub_img = node.advertise<sensor_msgs::Image>("topic", queuesize);
      }

      ros::Time timePublish = ros::Time::now();

      std_msgs::Header imageReceived;  // empty header
      imageReceived.seq = imageCount;  // user defined counter
      imageReceived.stamp = ros::Time::now();  // time
      pubImageReceivedFlag->publish(imageReceived);
      ros::Duration convertImageTime = timePublish - timeStartEncode;
      if (imageCount < 10000) {
        imageFetchlog << imageCount - 2 << ",";
        imageFetchlog << fetchImageTime.toSec() << ",";
        imageFetchlog << convertImageTime.toSec() << ",";
        imageFetchlog << "\n";
      } else if (imageCount == 10000) {
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
  image_transport::ImageTransport RGBit(n);
  cout << "ros setup.\n";
  std::shared_ptr<ImagePollAgent> ImageAgent;
  int camera1Type, camera2Type;
  if (argc == 3) {
    camera1Type = atol(argv[1]), camera2Type = atol(argv[2]);
    cout << "camera1Type: " << camera1Type << ", camera2Type: " << camera2Type
        << "\n";
    ImageAgent.reset(new ImagePollAgent(camera1Type, camera2Type));
    ImageAgent->pubRGBImage.reset(
        new image_transport::Publisher(RGBit.advertise("rgbImage", 1)));
  } else if (argc == 2) {
    camera1Type = atol(argv[1]);
    cout << "CameraType: " << camera1Type << "\n";
    ImageAgent.reset(new ImagePollAgent(camera1Type));
  } else {
    ImageAgent.reset(new ImagePollAgent());
  }

  ImageAgent->initiateAirSimClient();
  ImageAgent->depthPollTrigger.reset(
      new ros::Subscriber(
          n.subscribe("imagePoll", 1, &ImagePollAgent::callBackImagePoll,
                      ImageAgent.get())));

  ImageAgent->pubDepthImage.reset(
      new image_transport::Publisher(it.advertise("depthImage", 1)));

  ImageAgent->pubImageReceivedFlag.reset(
      new ros::Publisher(
          n.advertise < std_msgs::Header > ("imageReceivedFlag", 1)));

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
