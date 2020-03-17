/**
 * @file
 * @brief Driver to interface with the camera using Pylon abd OpenCV.
 * @copyright 2020, New York University, Max Planck Gesellschaft. All rights
 *            reserved.
 * @license BSD 3-clause
 */

#pragma once

#include <ctime>
#include <iostream>

#include <real_time_tools/timer.hpp>

#include <robot_interfaces/sensors/camera_observation.hpp>
#include <robot_interfaces/sensors/sensor_driver.hpp>

#include <pylon/PylonIncludes.h>
#include <opencv2/opencv.hpp>

namespace robot_interfaces
{
/**
 * @brief Driver for interacting with a camera via Pylon and store
 * images using OpenCV.
 */
class PylonDriver : public SensorDriver<CameraObservation>
{
public:
    Pylon::PylonAutoInitTerm auto_init_term_;
    Pylon::CInstantCamera camera_;
    Pylon::CTlFactory& tl_factory_;
    Pylon::DeviceInfoList_t device_list_;
    Pylon::DeviceInfoList_t::const_iterator device_iterator_;
    Pylon::CImageFormatConverter format_converter_;
    Pylon::CPylonImage pylon_image_;
    int num_devices_;

    PylonDriver()
        : tl_factory_(Pylon::CTlFactory::GetInstance())
    {
        device_iterator_ = device_list_.begin();
        camera_.Attach(tl_factory_.CreateDevice(*device_iterator_));
        camera_.Open();
        camera_.MaxNumBuffer = 5;
        format_converter_.OutputPixelFormat = Pylon::PixelType_BGR8packed;

        camera_.StartGrabbing();
    }

    ~PylonDriver()
    {
        camera_.StopGrabbing();
    }

    /**
     * @brief Dummy method (Pylon raises exceptions elsewhere in case access
     * is not successful).
     * @return true
     */
    bool is_access_successful()
    {
        return true;
    }

    /**
     * @brief Get the latest observation (image frame + timestamp of when the
     * frame's grabbed).
     * @return CameraObservation
     */
    CameraObservation get_observation()
    {
        CameraObservation image_frame;
        Pylon::CGrabResultPtr ptr_grab_result;

        try
        { 
            camera_.RetrieveResult(
                5000, ptr_grab_result, Pylon::TimeoutHandling_ThrowException);
            image_frame.time_stamp =
                real_time_tools::Timer::get_current_time_sec();
            if (ptr_grab_result->GrabSucceeded())
            {
                format_converter_.Convert(pylon_image_, ptr_grab_result);
                image_frame.image = cv::Mat(ptr_grab_result->GetHeight(),
                                            ptr_grab_result->GetWidth(),
                                            CV_8UC3,
                                            (uint8_t*)pylon_image_.GetBuffer());
            }
            
        }
        catch (const std::exception& e)
        {
            std::cerr << "Failed to access images from the camera."
                      << std::endl;
        }
        return image_frame;
    }
};

}  // namespace robot_interfaces