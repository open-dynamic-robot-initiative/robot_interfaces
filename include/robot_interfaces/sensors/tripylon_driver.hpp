/**
 * @file
 * @brief Driver to interface with three cameras using Pylon and OpenCV.
 * @copyright 2020, New York University, Max Planck Gesellschaft. All rights
 *            reserved.
 * @license BSD 3-clause
 * References-
 * https://www.baslerweb.com/en/sales-support/downloads/document-downloads/pylon-sdk-samples-manual/
 * https://github.com/basler/pylon-ros-camera/blob/9f3832127fc39a2c181cbeb5257054352e2ef7fe/pylon_camera/src/pylon_camera/pylon_camera.cpp#L132
 *
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
 * @brief Driver for interacting with a camera via Pylon and storing
 * images using OpenCV.
 */
class TripylonDriver : public SensorDriver<CameraObservation>
{
public:
    Pylon::PylonAutoInitTerm auto_init_term_;
    Pylon::CInstantCameraArray cameras_;
    Pylon::CImageFormatConverter format_converter_;
    Pylon::CPylonImage pylon_image_;

    PylonDriver(const std::vector<std::string&> device_user_ids_to_open)
    {
        Pylon::CTlFactory& tl_factory = Pylon::CTlFactory::GetInstance();
        Pylon::PylonInitialize();
        Pylon::DeviceInfoList_t device_list;

        if (tl_factory.EnumerateDevices(device_list) != 3)
        {
            Pylon::PylonTerminate();
            std::cout << "Found number of camera devices connected presently: " << tl_factory.EnumerateDevices(device_list) << std::endl;
            throw std::runtime_error("Tripylon driver is designed to be used with three pylon compatible cameras. Please connect three such cameras, and retry.");
        }
        else
        {
            if (device_user_ids_to_open.size() != 3)
            {
                Pylon::PylonTerminate();
                throw std::runtime_error("Tripylon driver is designed to be used with three pylon compatible cameras. Please pass three device user ids to be opened, and retry.");
            }

            else
            {
                cameras_(3);
                //TODO: check this initialization, and add a check here- (min(len(device_list)), 3), but then we don't want to creat an array with no. of cameras other than 3

                int found_desired_device = 0;
                Pylon::DeviceInfoList_t::const_iterator device_iterator;
                const std::string& device_user_id_to_open;

                for (index = 0; index < device_user_ids_to_open.size(); ++index)
                {
                    for (device_iterator = device_list.begin();
                     device_iterator != device_list.end();
                     ++device_iterator)
                    {
                        std::string device_user_id_found(
                        device_iterator->GetUserDefinedName());
                        if(device_user_id_found == device_user_ids_to_open[index])
                        {
                            found_desired_device +=1;
                            break;
                        }
                    }
                }
                if (found_desired_device == 3)
                {
                    index = 0;
                    for (device_iterator = device_list.begin();
                     device_iterator != device_list.end();
                     ++device_iterator)
                    {
                        cameras_[index].Attach(tl_factory.CreateDevice(*device_iterator));
                        index +=1;
                    }
                }
                else
                {
                    Pylon::PylonTerminate();
                    throw std::runtime_error("Device ids specified do not correspond to the ids given to the connected devices. Please specify a valid vector of ids, and retry.")
                }
                cameras_.Open();
                cameras_.MaxNumBuffer = 5;
                format_converter_.OutputPixelFormat =
                    Pylon::PixelType_BGR8packed;

                cameras_.StartGrabbing(Pylon::GrabStrategy_OneByOne, Pylon::GrabLoop_ProvidedByInstantCamera);
            }
        }
    }

    ~TripylonDriver()
    {
        cameras_.StopGrabbing();
        Pylon::PylonTerminate();
    }

    std::vector<CameraObservation> get_observation()
    {
        std::vector<CameraObservation> image_frames;
        Pylon::CGrabResultPtr ptr_grab_result;

        cameras_.RetrieveResult(
            5000, ptr_grab_result, Pylon::TimeoutHandling_ThrowException);
        time_stamp = real_time_tools::Timer::get_current_time_sec();

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
            throw std::runtime_error(
                "Failed to access images from the cameras.");
        }
        return image_frames;
    }

    


};

}    