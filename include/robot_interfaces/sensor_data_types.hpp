#pragma once

#include <cmath>
#include <ctime>
#include <real_time_tools/threadsafe/threadsafe_timeseries.hpp>
#include <time_series/time_series.hpp>

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

#include<robot_interfaces/sensor_data.hpp>
#include<robot_interfaces/opencv_driver.hpp>
#include<robot_interfaces/sensor_backend.hpp>
#include<robot_interfaces/sensor_frontend.hpp>

namespace robot_interfaces
{

struct SensorDataTypes
{

  // typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Image;
  // typedef Eigen::Matrix<uint8_t, N, M> Image;
  // typedef Pylon::CGrabResultData Image; //in private scope, cannot do this meh
  // typedef Pylon::CGrabResultPtr *Image;
  typedef cv::Mat Image;
  typedef time_t TimeStamp;

  struct CameraObservation
  {
    Image image;
    TimeStamp time_stamp;
  };

  typedef SensorData<CameraObservation> Data;
  typedef std::shared_ptr<Data> DataPtr;
  typedef OpenCVDriver<CameraObservation> CVDriver;
  typedef std::shared_ptr<CVDriver> CVDriverPtr;
  typedef SensorFrontend<CameraObservation> Frontend;
  typedef SensorBackend<CameraObservation> Backend;

};

} //namespace robot_interfaces
