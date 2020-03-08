#pragma once

#include <algorithm>
#include <cmath>

#include <time_series/time_series.hpp>

#include <robot_interfaces/sensor_data.hpp>

namespace robot_interfaces
{

  template <typename Type>
  using Timeseries = time_series::TimeSeries<Type>;

  typedef time_series::Index TimeIndex;

  template <typename CameraObservation>
  class SensorFrontend
  {

  public:

      typedef time_series::Timestamp TimeStamp;

      SensorFrontend(
          std::shared_ptr<SensorData<CameraObservation>> sensor_data)
          : sensor_data_(sensor_data)
      {
      }

      CameraObservation get_observation(const TimeIndex &t)
      {
          return (*sensor_data_->observation)[t];
      }

      CameraObservation get_latest_observation()
      {
          return (*sensor_data_->observation)[get_current_timeindex()];
      }

      TimeStamp get_timestamp_ms(const TimeIndex &t)
      {
          return sensor_data_->observation->timestamp_ms(t);
      }
      TimeIndex get_current_timeindex()
      {
          return sensor_data_->observation->newest_timeindex();
      }

  private:
      std::shared_ptr<SensorData<CameraObservation>> sensor_data_;

  };

} //namespace robot_interfaces
