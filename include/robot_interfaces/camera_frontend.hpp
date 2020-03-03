#pragma once

#include <algorithm>
#include <cmath>

#include <real_time_tools/threadsafe/threadsafe_timeseries.hpp>

#include <robot_interfaces/camera_data.hpp>

namespace robot_interfaces
{

  template <typename Type>
  using Timeseries = real_time_tools::ThreadsafeTimeseries<Type>;

  typedef Timeseries<int>::Index TimeIndex;

  template <typename CameraObservation>
  class CameraFrontend
  {

  public:

      typedef Timeseries<int>::Timestamp TimeStamp;

      CameraFrontend(
          std::shared_ptr<CameraData<CameraObservation>> camera_data)
          : camera_data_(camera_data)
      {
      }

      CameraObservation get_observation()
      {
          return (*camera_data_->observation)[get_current_timeindex()];
      }

      TimeStamp get_time_stamp_ms(const TimeIndex &t)
      {
          return camera_data_->observation->timestamp_ms(t);
      }
      TimeIndex get_current_timeindex()
      {
          return camera_data_->observation->newest_timeindex();
      }

  private:
      std::shared_ptr<CameraData<CameraObservation>> camera_data_;

  };

} //namespace robot_interfaces
