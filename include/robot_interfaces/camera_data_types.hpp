#pragma once

#include <cmath>
#include <ctime>
#include <real_time_tools/threadsafe/threadsafe_timeseries.hpp>
#include <Eigen/Eigen>

#include<robot_interfaces/camera_data.hpp>
#include<robot_interfaces/camera_driver.hpp>
#include<robot_interfaces/camera_backend.hpp>
#include<robot_interfaces/camera_frontend.hpp>

namespace robot_interfaces
{

struct CameraDataTypes
{

  typedef Eigen::Matrix<uint8_t, 720, 540> Image;
  typedef time_t TimeStamp;

  struct CameraObservation
  {
    Image image;
    TimeStamp time_stamp;
  };

  typedef CameraData<CameraObservation> CData;
  typedef std::shared_ptr<CData> CDataPtr;
  typedef CameraDriver<CameraObservation> CDriver;
  typedef CameraFrontend<CameraObservation> CFrontend;
  typedef CameraBackend<CameraObservation> CBackend;

};

} //namespace robot_interfaces
