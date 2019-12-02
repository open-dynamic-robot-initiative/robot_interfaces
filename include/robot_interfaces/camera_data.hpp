#pragma once

#include <iostream>
#include <memory>
#include <string>

#include <real_time_tools/threadsafe/threadsafe_timeseries.hpp>

namespace robot_interfaces
{
  template <typename Type>
  using Timeseries = real_time_tools::ThreadsafeTimeseries<Type>;

template <typename CameraObservation>
class CameraData
{
public:
  template <typename Type>
  using Ptr = std::shared_ptr<Type>;

  CameraData(size_t history_length = 15000,
            bool use_shared_memory = false,
            // suppress unused warning (will be used in the future)
            __attribute__((unused)) std::string shared_memory_address = "")
  {
      if (use_shared_memory)
      {
          std::cout << "shared memory camera data is not implemented yet"
                    << std::endl;
          exit(-1);

          // TODO: here we should check if the shared memory at that
          // address already exists, otherwise we create it.
          // we will also have to update timeseries such as to handle
          // serialization internally (it will simply assume that the
          // templated class has a method called serialize() and
          // from_serialized())
      }
      else
      {
          observation = std::make_shared<Timeseries<CameraObservation>>(history_length);
      }
  }
public:
    Ptr<Timeseries<CameraObservation>> observation;

};

} //namespace robot_interfaces
