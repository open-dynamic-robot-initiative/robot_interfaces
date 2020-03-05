#pragma once

#include <iostream>
#include <ctime>

#include <real_time_tools/process_manager.hpp>
#include <real_time_tools/thread.hpp>
#include <real_time_tools/threadsafe/threadsafe_timeseries.hpp>
#include <real_time_tools/timer.hpp>

#include "opencv2/opencv.hpp"

// #include <pylon/PylonIncludes.h>

namespace robot_interfaces
{

  template <typename CameraObservation>
  class OpenCVDriver
  {

  public:

      cv::VideoCapture video_capture;

      OpenCVDriver()
      {

        cv::VideoCapture cap(0);
        video_capture = cap;
        
      }

      // void start_grabbing()
      // {
      //     cv::VideoCapture cap(0);
      //     video_capture = cap;
      // }
      
      int is_grabbing_successful()
      {
          if (!video_capture.isOpened())  // check if we succeeded
          {
            std::cout << "Could not access camera stream :(" <<std::endl;
            return -1;
          }
          else
          {
            std::cout << "Succeeded in accessing camera stream!" <<std::endl;
            return 1;
          }
      }

      CameraObservation grab_frame()
      {   
          CameraObservation image_frame;
          cv::Mat frame;
          time_t current_time = time(NULL);
          video_capture >> frame;
          image_frame.image = frame;
          image_frame.time_stamp = current_time;
          return image_frame;
      }

  };

} //namespace robot_interfaces

/*
FIXME: latest non-working 
namespace robot_interfaces

{

template <typename CameraObservation>
class CameraDriver

{

public:
  int exitCode = 0;
  void PylonInitialize(void);

  CameraObservation grab_frame()
  {
    CameraObservation image_frame;
    //FIXME: Can;t append a pointer to a threadsafe_timeseries as defined in real_time_tools
    try
    {
      Pylon::CInstantCamera camera(Pylon::CTlFactory::GetInstance().CreateFirstDevice());
      std::cout << "Using device " << camera.GetDeviceInfo().GetModelName() << std::endl;
      // get this error when trying to use this: ImportError: /home/sjoshi/blmc_ei/workspace/devel/lib/python3/dist-packages/robot_interfaces/py_camera_types.cpython-35m-x86_64-linux-gnu.so: undefined symbol: _ZTVN5Pylon11CDeviceInfoE
      camera.MaxNumBuffer = 5;

      camera.StartGrabbing(1);

      Pylon::CGrabResultPtr ptrGrabResult; //*imgGrabResult;
      // Pylon::CGrabResultData imgGrabResult;
      // Pylon::CImageFormatConverter formatConverter;
      // formatConverter.OutputPixelFormat = Pylon::PixelType_BayerBG8;
      // Pylon::CPylonImage pylonImage;

      while (camera.IsGrabbing())
      {
        camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
        if (ptrGrabResult->GrabSucceeded())
        {
          // formatConverter.Convert(pylonImage, ptrGrabResult);
          uint8_t *pImageBuffer = (uint8_t *) ptrGrabResult->GetBuffer();

          time_t current_time = time(NULL);
          // imgGrabResult = (uint8_t *)ptrGrabResult->GetBuffer();
          // imgGrabResult = *(ptrGrabResult->CArray());

          // image_frame.image = (uint8_t *)pylonImage.GetBuffer();
          // image_frame.image = imgGrabResult;
          //TODO: conversion to an Eigen matrix is not supported here
          image_frame.time_stamp = current_time;

          Pylon::CImagePersistence::Save(Pylon::ImageFileFormat_Raw, "test.rw2", ptrGrabResult);
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

    return image_frame;
  }
};
}//namespace robot_interfaces

*/

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
