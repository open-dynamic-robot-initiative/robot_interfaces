#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <real_time_tools/process_manager.hpp>
#include <real_time_tools/thread.hpp>
#include <real_time_tools/timer.hpp>

#include <robot_interfaces/sensor_data.hpp>
#include <robot_interfaces/opencv_driver.hpp>


namespace robot_interfaces
{
template <typename CameraObservation>
class SensorBackend
{
public:

  SensorBackend(
    std::shared_ptr<OpenCVDriver<CameraObservation>> opencv_driver,
    std::shared_ptr<SensorData<CameraObservation>> sensor_data)
    : opencv_driver_(opencv_driver),
      sensor_data_(sensor_data),
      destructor_was_called_(false)
  {
      thread_ = std::make_shared<real_time_tools::RealTimeThread>();
      thread_->create_realtime_thread(&SensorBackend::loop, this);
  }

  virtual ~SensorBackend()
  {
      destructor_was_called_ = true;
      thread_->join();
  }

private:

  std::shared_ptr<OpenCVDriver<CameraObservation>> opencv_driver_;
  std::shared_ptr<SensorData<CameraObservation>> sensor_data_;

  bool destructor_was_called_;

  std::vector<real_time_tools::Timer> timers_;

  std::shared_ptr<real_time_tools::RealTimeThread> thread_;

  static void *loop(void *instance_pointer)
  {
      ((SensorBackend *)(instance_pointer))->loop();
      return nullptr;
  }

  void loop()
  {

    while (!destructor_was_called_)
    {
    }

    for (long int t = 0; !destructor_was_called_; t++)
    {
      CameraObservation camera_observation;
      // opencv_driver_->start_grabbing();

      int flag = opencv_driver_->is_grabbing_successful();
      if (flag == 1)
      {
        camera_observation = (opencv_driver_->grab_frame());
        sensor_data_->observation->append(camera_observation);
      }
      else
      {
        std::cout << "Cannot store images when cannot access camera." << std::endl;
      }
    }
  }
};

} //namespace robot_interfaces
