/**
 * @file
 * @brief Driver to interface with the camera using Pylon and OpenCV.
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
    Pylon::CImageFormatConverter format_converter_;
    Pylon::CPylonImage pylon_image_;
    const std::string& device_user_id_to_open_; 

    PylonDriver(const std::string& device_user_id) : 
        device_user_id_to_open_(device_user_id),
        tl_factory_(Pylon::CTlFactory::GetInstance())
    {
        Pylon::PylonInitialize();
        Pylon::DeviceInfoList_t device_list;
        std::cout << tl_factory_.EnumerateDevices(device_list) << std::endl;

        if (tl_factory_.EnumerateDevices(device_list) == 0)
        {
            throw std::runtime_error("No devices present, please connect one.");
            Pylon::PylonTerminate();
        }
        
        else
        {
            Pylon::DeviceInfoList_t::const_iterator device_iterator;
            if (device_user_id_to_open_.empty())
            {
                device_iterator = device_list.begin();
                camera_.Attach(tl_factory_.CreateDevice(*device_iterator));
                std::cout << "Desired device not found. Creating a camera object with the first device id in the device list." << std::endl;                
            }
            else
            {
                bool found_desired_device = false;

                for(device_iterator = device_list.begin(); device_iterator != device_list.end(); ++device_iterator)
                {
                    std::string device_user_id_found(device_iterator->GetUserDefinedName());
                    if (device_user_id_to_open_ == device_user_id_found)
                    {
                        found_desired_device = true;
                        break;
                    } 
                }

                if (found_desired_device)
                {
                    camera_.Attach(tl_factory_.CreateDevice(*device_iterator));
                }
                else
                {
                    throw std::runtime_error("Device id specified doesn't correspond to any connected devices, please retry with a valid id.");
                    Pylon::PylonTerminate();                
                }
                
                camera_.Open();
                camera_.MaxNumBuffer = 5;
                format_converter_.OutputPixelFormat = Pylon::PixelType_BGR8packed;

                camera_.StartGrabbing();
            }
        }              
        
    }

    ~PylonDriver()
    {
        camera_.StopGrabbing();
        Pylon::PylonTerminate();
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
            
            camera_.RetrieveResult(5000, ptr_grab_result, Pylon::TimeoutHandling_ThrowException);
            image_frame.time_stamp = real_time_tools::Timer::get_current_time_sec();
            format_converter_.Convert(pylon_image_, ptr_grab_result);
            image_frame.image = cv::Mat(ptr_grab_result->GetHeight(),
                                            ptr_grab_result->GetWidth(),
                                            CV_8UC3,
                                            (uint8_t*)pylon_image_.GetBuffer());  
            std::cout << image_frame.image << std::endl;                                   
            if (ptr_grab_result->GrabSucceeded())
            {
                format_converter_.Convert(pylon_image_, ptr_grab_result);
                image_frame.image = cv::Mat(ptr_grab_result->GetHeight(),
                                            ptr_grab_result->GetWidth(),
                                            CV_8UC3,
                                            (uint8_t*)pylon_image_.GetBuffer());
            }
            else
            {
                throw std::runtime_error("Failed to access images from the camera.");
            }
            
        }
        catch (std::runtime_error &e)
        {
            std::cerr << e.what()
                      << std::endl;
        }
        return image_frame;
    }
};

}  // namespace robot_interfaces