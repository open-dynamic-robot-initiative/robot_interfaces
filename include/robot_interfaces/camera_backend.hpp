#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <real_time_tools/process_manager.hpp>
#include <real_time_tools/thread.hpp>
#include <real_time_tools/threadsafe/threadsafe_timeseries.hpp>
#include <real_time_tools/timer.hpp>

#include <robot_interfaces/camera_data.hpp>
#include <robot_interfaces/camera_driver.hpp>

namespace robot_interfaces
{
template <typename CameraObservation>
class CameraBackend
{
public:

  CameraBackend(std::shared_ptr<CameraDriver<CameraObservation>> camera_driver, std::shared_ptr<CameraData<CameraObservation>> camera_data):camera_driver_(camera_driver), camera_data_(camera_data), destructor_was_called_(false)
  {
      thread_ = std::make_shared<real_time_tools::RealTimeThread>();
      thread_->create_realtime_thread(&CameraBackend::loop, this);
  }

  virtual ~CameraBackend()
  {
      destructor_was_called_ = true;
      thread_->join();
  }

private:

  std::shared_ptr<CameraDriver<CameraObservation>> camera_driver_;
  std::shared_ptr<CameraData<CameraObservation>> camera_data_;

  bool destructor_was_called_;

  std::vector<real_time_tools::Timer> timers_;

  std::shared_ptr<real_time_tools::RealTimeThread> thread_;

  static void *loop(void *instance_pointer)
  {
      ((CameraBackend *)(instance_pointer))->loop();
      return nullptr;
  }

  void loop()
  {

    while (!destructor_was_called_)
    {
    }

    for (long int t = 0; !destructor_was_called_; t++)
    {
      CameraObservation camera_observation_ptr;
      camera_observation_ptr = camera_driver_->grab_frame();

      camera_data_->observation->append(camera_observation_ptr);

    }

  }
};

} //namespace robot_interfaces
