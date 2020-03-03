#pragma once

#include <cmath>
#include <ctime>
#include <real_time_tools/threadsafe/threadsafe_timeseries.hpp>
// #include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

#include<robot_interfaces/camera_data.hpp>
#include<robot_interfaces/camera_driver.hpp>
#include<robot_interfaces/camera_backend.hpp>
#include<robot_interfaces/camera_frontend.hpp>

namespace robot_interfaces
{

struct CameraDataTypes
{

  typedef cv::Mat Image;
  // typedef Eigen::Matrix<uint8_t, N, M> Image;
  // typedef Pylon::CGrabResultData Image; //in private scope, cannot do this meh
  // typedef Pylon::CGrabResultPtr *Image;
  typedef time_t TimeStamp;

  struct CameraObservation
  {
    Image image;
    TimeStamp time_stamp;
  };

  typedef CameraData<CameraObservation> CData;
  typedef std::shared_ptr<CData> CDataPtr;
  typedef CameraDriver<CameraObservation> CDriver;
  typedef std::shared_ptr<CDriver> CDriverPtr;
  typedef CameraFrontend<CameraObservation> CFrontend;
  typedef CameraBackend<CameraObservation> CBackend;

};

} //namespace robot_interfaces
