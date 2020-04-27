/**
 * @file
 * @brief Driver to interface with the camera using Pylon and OpenCV.
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
class PylonDriver : public SensorDriver<CameraObservation>
{
// TODO implementations should be moved to cpp file
public:
    Pylon::PylonAutoInitTerm auto_init_term_;
    Pylon::CInstantCamera camera_;
    Pylon::CImageFormatConverter format_converter_;
    Pylon::CPylonImage pylon_image_;

    /**
     * @param device_user_id_to_open The id of the camera device to open and
     * grab images from
     */
    PylonDriver(const std::string& device_user_id_to_open)
    {
        Pylon::CTlFactory& tl_factory = Pylon::CTlFactory::GetInstance();
        Pylon::PylonInitialize();
        Pylon::DeviceInfoList_t device_list;

        if (tl_factory.EnumerateDevices(device_list) == 0)
        {
            Pylon::PylonTerminate();
            throw std::runtime_error("No devices present, please connect one.");
        }

        else
        {
            Pylon::DeviceInfoList_t::const_iterator device_iterator;
            if (device_user_id_to_open.empty())
            {
                device_iterator = device_list.begin();
                camera_.Attach(tl_factory.CreateDevice(*device_iterator));
                std::cout
                    << "Desired device not found. Creating a camera object "
                       "with the first device id in the device list."
                    << std::endl;
            }
            else
            {
                bool found_desired_device = false;

                for (device_iterator = device_list.begin();
                     device_iterator != device_list.end();
                     ++device_iterator)
                {
                    std::string device_user_id_found(
                        device_iterator->GetUserDefinedName());
                    if (device_user_id_to_open == device_user_id_found)
                    {
                        found_desired_device = true;
                        break;
                    }
                }

                if (found_desired_device)
                {
                    camera_.Attach(tl_factory.CreateDevice(*device_iterator));
                }
                else
                {
                    Pylon::PylonTerminate();
                    throw std::runtime_error(
                        "Device id specified doesn't correspond to any "
                        "connected devices, please retry with a valid id.");
                }

                camera_.Open();
                camera_.MaxNumBuffer = 5;
                format_converter_.OutputPixelFormat =
                    Pylon::PixelType_BGR8packed;

                set_camera_configuration(camera_.GetNodeMap());

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

        camera_.RetrieveResult(
            5000, ptr_grab_result, Pylon::TimeoutHandling_ThrowException);
        image_frame.time_stamp = real_time_tools::Timer::get_current_time_sec();

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
                "Failed to access images from the camera.");
        }
        return image_frame;
    }

private:
    void set_camera_configuration(GenApi::INodeMap &nodemap)
    {
        Pylon::CFloatParameter exposure_time(nodemap, "ExposureTime");
        exposure_time.SetValue(1500);

        //Pylon::CBooleanParameter enable_frame_rate(nodemap, "EnableAcquisitionFrameRate");
        //enable_frame_rate.SetValue(true);

        Pylon::CFloatParameter frame_rate(nodemap, "AcquisitionFrameRate");
        frame_rate.SetValue(100);

        Pylon::CEnumParameter(nodemap, "BalanceWhiteAuto").SetValue("Once");


        //Pylon::CFeaturePersistence::Save("/tmp/camera_settings.xml", &camera_.GetNodeMap() );
        //Pylon::CFeaturePersistence::Load( Filename, &camera.GetNodeMap(), true );
    }
};

}  // namespace robot_interfaces
