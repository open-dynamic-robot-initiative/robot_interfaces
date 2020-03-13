#pragma once

#include <iostream>
#include <ctime>

#include <real_time_tools/process_manager.hpp>
#include <real_time_tools/thread.hpp>
#include <real_time_tools/threadsafe/threadsafe_timeseries.hpp>
#include <real_time_tools/timer.hpp>

#include <robot_interfaces/sensors/camera_observation.hpp>
#include <robot_interfaces/sensors/sensor_driver.hpp>

#include <opencv2/opencv.hpp>
#include <pylon/PylonIncludes.h>

namespace robot_interfaces
{

class PylonDriver : public SensorDriver<CameraObservation>
{
public:

    Pylon::PylonAutoInitTerm auto_init_term;
    GenApi::CIntegerPtr width, height;
    
    Pylon::CImageFormatConverter format_converter;
    Pylon::CPylonImage pylon_image;
    Pylon::CGrabResultPtr ptr_grab_result;
    
    Pylon::CInstantCamera camera;

    cv::VideoCapture video_capture;
    real_time_tools::Timer timer;
    static const uint32_t count_of_images_to_grab = 1;

    PylonDriver() : camera(Pylon::CTlFactory::GetInstance().CreateFirstDevice())
    { 
      GenApi::INodeMap& nodemap = camera.GetNodeMap();
      camera.Open();
      width = nodemap.GetNode("Width");
      height = nodemap.GetNode("Height");
      
      camera.MaxNumBuffer = 5;

      format_converter.OutputPixelFormat = Pylon::PixelType_BGR8packed;  
    }

    bool is_access_successful()
    {
      return true;
      // try
      // {
      //   pylon_init();
      // }
      // catch(const std::exception& e)
      // {
      //   std::cerr << e.what() << '\n';
      // }
      
    }

    CameraObservation get_observation()
    {          
        CameraObservation image_frame;
        cv::Mat frame;
        double current_time = timer.get_current_time_sec();
        camera.StartGrabbing();

        while(camera.IsGrabbing())
        {
          camera.RetrieveResult(15000, ptr_grab_result, Pylon::TimeoutHandling_ThrowException);
          image_frame.time_stamp = timer.get_current_time_sec();
          if(ptr_grab_result->GrabSucceeded())
          {
            format_converter.Convert(pylon_image, ptr_grab_result);
            image_frame.image = cv::Mat(ptr_grab_result->GetHeight(), ptr_grab_result->GetWidth(),
                                        CV_8UC3, (uint8_t*)pylon_image.GetBuffer());
          }
        }
    }

};

} //namespace robot_interfaces