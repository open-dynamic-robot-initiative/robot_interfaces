#pragma once

#include <iostream>
#include <ctime>

#include <real_time_tools/process_manager.hpp>
#include <real_time_tools/thread.hpp>
#include <real_time_tools/threadsafe/threadsafe_timeseries.hpp>
#include <real_time_tools/timer.hpp>

#include <pylon/PylonIncludes.h>

// using namespace Pylon;
namespace robot_interfaces

{

template <typename CameraObservation>
class CameraDriver

{

public:

  // CameraDriver()
  // {
  // }
  //
  // ~CameraDriver()
  // {
  // }

  int exitCode = 0;
  // typedef Eigen::Matrix<uint8_t, N, M> Image;
  void PylonInitialize(void);

  CameraObservation* grab_frame()
  {
    CameraObservation *CameraObservationPtr;
    try
    {
      Pylon::CInstantCamera camera(Pylon::CTlFactory::GetInstance().CreateFirstDevice());
      std::cout << "Using device " << camera.GetDeviceInfo().GetModelName() << std::endl;
      camera.MaxNumBuffer = 5;

      camera.StartGrabbing(1);

      Pylon::CGrabResultPtr ptrGrabResult;
      Pylon::CImageFormatConverter formatConverter;
      formatConverter.OutputPixelFormat = Pylon::PixelType_BayerBG8;
      Pylon::CPylonImage pylonImage;

      // Image save_image;

      while (camera.IsGrabbing())
      {
        camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
        if (ptrGrabResult->GrabSucceeded())
        {
          const uint8_t *pImageBuffer = (uint8_t *) ptrGrabResult->GetBuffer();
          formatConverter.Convert(pylonImage, ptrGrabResult);

          // save_image = (uint8_t *)pylonImage.GetBuffer();
          time_t current_time = time(NULL);

          // CameraObservationPtr->image = save_image;
          CameraObservationPtr->image = (uint8_t *)pylonImage.GetBuffer();
          CameraObservationPtr->time_stamp = current_time;
        }
        else
        {
          std::cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << std::endl;
        }
      }
    }
    catch (const Pylon::GenericException &e)
    {
      std::cerr << "An exception occurred." << std::endl
      << e.GetDescription() << std::endl;
      exitCode = 1;
    }

    return CameraObservationPtr;
  }
};
}//namespace robot_interfaces













  // // Pylon::CInstantCamera() camera_object;
  // Pylon::CTlFactory create_camera;
  // // camera = (create_camera.GetInstance()).CreateFirstDevice();
  //
  // camera = Pylon::CInstantCamera(create_camera.GetInstance()).CreateFirstDevice())
  //
  // CameraDriver()
  // {
  //   camera.Open();
  // }
  //
  // ~CameraDriver()
  // {
  //   camera.Close();
  // }
  //
  // // typedef Eigen::Matrix<uint8_t, 720, 540> grabResult;
  // Pylon::CGrabResult grabResult;
  //
  // //TODO: check return type, should be SensorObservation
  // //TODO: This derives directly from grab.py? NO. This should not open and close
  // //the camera for every frame since the time required to capture th first frame
  // //is invariably higher.
  // //TODO: This method is to grab frames from multiple cameras
  // //Source- grab.py in the pypylon samples
  //
  // CameraObservation* grab_frame()
  // {
  //   CameraObservation *CameraObservationPtr;
  //
  //       std::string camera_model = (camera.GetDeviceInfo()).GetModelName();
  //       std::cout << "Camera being used is: " << camera_model;
  //
  //       camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
  //
  //       // if (camera.WaitForFrameTriggerReady(1000))
  //       // {
  //       //   camera.ExecuteSoftwareTrigger();
  //       // }
  //       grabResult = camera.RetrieveResult(1000, Pylon::TimeoutHandling_ThrowException);
  //       time_t current_time = time(NULL);
  //
  //       if (grabResult.GrabSucceeded())
  //       {
  //         CameraObservationPtr->image = grabResult.Array;
  //         CameraObservationPtr->time_stamp = current_time;
  //       }
  //
  //       grabResult.Release();
  //       return CameraObservationPtr;
  //
  //
  //
  //
  //
  // }

  //NOTE: This was being written with capture acc to a number of frames specified.
  // SensorObservation* grab_frame(num_imgs_, num_cameras_)
  // {
  //   if (num_cameras == 1)
  //   {
  //     SensorObservation *SensorObservationPtr;
  //
  //     std::string camera_model = camera.GetDeviceInfo().GetModelName();
  //     std::cout << "Camera being used is: " << camera_model;
  //
  //     camera.StartGrabbingMax(num_imgs_, Pylon::GrabStrategy_LatestImageOnly);
  //
  //     while camera.IsGrabbing()
  //     {
  //       //TODO: select the strategy for RetrieveResult
  //       grabResult = camera.RetrieveResult(100, pylon.TimeoutHandling_ThrowException);
  //       time_t current_time = time(NULL);
  //
  //       if (grabResult.GrabSucceeded())
  //       {
  //         SensorObservationPtr->sensor_observation = grabResult.Array;
  //         SensorObservationPtr->time_stamp = current_time;
  //       }
  //       return SensorObservationPtr;
  //       grabResult.Release();
  //     }
  //   }
  //
  //   //TODO: Will work on this when can test this.
  //   // else if (num_cams > 1)
  //   // {
  //   //   maxCamerasToUse = num_cameras;
  //   //
  //   //   tlFactory = pylon.TlFactory.GetInstance();
  //   //   devices = tlFactory.EnumerateDevices();
  //   //   if (len(devices) == 0)
  //   //   {
  //   //     raise pylon.RUNTIME_EXCEPTION("No camera present.");
  //   //   }
  //   //   cameras = pylon.InstantCameraArray(min(len(devices), maxCamerasToUse));
  //   //   l = cameras.GetSize();
  //   //
  //   //
  //   //
  //   // }
  //
  //   else if (num_cams < 1)
  //   {
  //     std::cout << "Specify a valid number of cameras to be used." <<std::endl;
//   //   }
//   //
//   // }
//
// };
//
// }
