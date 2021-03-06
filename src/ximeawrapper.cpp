/*
This file is part of the Xamla ROS node for Ximea cameras
Copyright (C) 2018 Xamla and/or its affiliates

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#include <stdio.h>
#include <iostream>

#include <xiApi.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


#define HandleResult(res,place) if (res != XI_OK) { printf("CameraXIMEA: Error at %s (%d)\n",place,res); fflush(stdout); }


struct StereoHandle {
  HANDLE cam1;
  HANDLE cam2;
};


extern "C" {
#include <TH/TH.h>

bool closeStereoCameras(StereoHandle *handle);
}


extern "C" bool getSerialNumber(int camNum, char* serial) {
  XI_RETURN stat = xiGetDeviceInfoString(camNum, XI_PRM_DEVICE_SN, serial, 20);
  if (stat != XI_OK) {
    return false;
  }
  return true;
}


void printCamList() {
  XI_RETURN stat = XI_OK;
  DWORD numCams;
  stat = xiGetNumberDevices(&numCams);

  for (unsigned int i=0; i<numCams; i++) {
    char name[20];
    xiGetDeviceInfoString(i, XI_PRM_DEVICE_NAME, name, 20);
    printf("%s\n", name);
  }
}


void initCam(unsigned int camNum, HANDLE &camera, XI_IMG_FORMAT color_fmt, bool startAcquisition) {

  if (! (color_fmt == XI_MONO8 || color_fmt == XI_RGB24)) {
    std::cout << "Unsupported color format" << std::endl;
    return;
  }

  int stat = 0;

  printCamList();

  xiSetParamInt(0, XI_PRM_DEBUG_LEVEL, XI_DL_FATAL);

  // Disable auto bandwidth determination (takes some seconds in initialization)
  xiSetParamInt(0, XI_PRM_AUTO_BANDWIDTH_CALCULATION, XI_OFF);

  // Retrieve a handle to the camera device
  stat = xiOpenDevice(camNum, &camera);
  HandleResult(stat, "xiOpenDevice");

  // Configure unsafe buffers (prevents old buffers, memory leak)
  xiSetParamInt(camera, XI_PRM_BUFFER_POLICY, XI_BP_UNSAFE);

  stat = xiSetParamInt(camera, XI_PRM_BUFFERS_QUEUE_SIZE, 10);
  HandleResult(stat, "xiSetParam (XI_PRM_BUFFERS_QUEUE_SIZE)");

  // Configure queue mode (0 = next frame in queue, 1 = most recent frame)
  stat = xiSetParamInt(camera, XI_PRM_RECENT_FRAME, 1);
  HandleResult(stat, "xiSetParam (XI_PRM_RECENT_FRAME)");

  // Configure image type
  stat = xiSetParamInt(camera, XI_PRM_IMAGE_DATA_FORMAT, color_fmt);
  HandleResult(stat, "xiSetParam (XI_PRM_IMAGE_DATA_FORMAT)");

  xiSetParamFloat(camera, XI_PRM_GAMMAY, 1.0);
  xiSetParamInt(camera, XI_PRM_EXPOSURE, 32666);  // us
  xiSetParamFloat(camera, XI_PRM_GAIN, 0);
  xiSetParamInt(camera, XI_PRM_COLUMN_FPN_CORRECTION, 1);

  std::cout << "Exposure set" << std::endl;

  stat = xiSetParamInt(camera, XI_PRM_TRG_SOURCE, XI_TRG_SOFTWARE);
  HandleResult(stat, "xiSetParam (XI_PRM_TRG_SOURCE)");

  if (startAcquisition) {
    stat = xiStartAcquisition(camera);
    HandleResult(stat, "xiStartAcquisition");
  }
}


extern "C" void closeDevice(HANDLE camera) {
  int stat = xiStopAcquisition(camera);
  HandleResult(stat, "xiStopAcquisition");

  stat = xiCloseDevice(camera);
  HandleResult(stat, "xiCloseDevice");
}


extern "C" void getNumberConnectedDevices(int *n) {
  XI_RETURN stat = XI_OK;
  DWORD numCams;
  stat = xiGetNumberDevices(&numCams);
  *n = numCams;
}


extern "C" bool getSerialsStereo(StereoHandle *handle, char *serial_cam1, char *serial_cam2) {
  if (!handle)
    return false;
  memset(serial_cam1, 0, sizeof(char[21]));
  memset(serial_cam2, 0, sizeof(char[21]));
  xiGetParamString(handle->cam1, XI_PRM_DEVICE_SN, serial_cam1, 20);
  xiGetParamString(handle->cam2, XI_PRM_DEVICE_SN, serial_cam2, 20);
  return true;
}


extern "C" bool getSingleImage(HANDLE camera, XI_IMG_FORMAT img_format, THByteTensor *image_out, bool triggered, int timeout = 1000) {
  XI_IMG image;
  image.size = SIZE_XI_IMG_V2; // must be initialized
  image.bp = NULL;
  image.bp_size = 0;

  int stat = 0;
  if (!triggered) {
    stat = xiSetParamInt(camera, XI_PRM_TRG_SOFTWARE, 1);
    HandleResult(stat, "xiSetParam (XI_PRM_TRG_SOFTWARE)");
  }

  // retrieve image from camera
  stat = xiGetImage(camera, timeout, &image);
  HandleResult(stat, "xiGetImage");

  if (stat == XI_OK) {
    if (img_format == XI_MONO8 || img_format == XI_RAW8 ) {
      THByteTensor_resize2d(image_out, image.height, image.width);
      memcpy(image_out->storage->data, image.bp, image.height * image.width * sizeof(uint8_t));
    } else if (img_format == XI_MONO16 || img_format == XI_RAW16) {
      THByteTensor_resize3d(image_out, image.height, image.width, 2);
      memcpy(image_out->storage->data, image.bp, image.height * image.width * sizeof(uint8_t) * 2);
    } else if (img_format == XI_RGB24) {
      THByteTensor_resize3d(image_out, image.height, image.width, 3);
      memcpy(image_out->storage->data, image.bp, image.height * image.width * sizeof(uint8_t) * 3);
    } else if (img_format == XI_RGB32) {
      THByteTensor_resize3d(image_out, image.height, image.width, 4);
      memcpy(image_out->storage->data, image.bp, image.height * image.width * sizeof(uint8_t) * 4);
    }
    else {
      return false;     // unknown image format specified
    }
    return true;
  }

  return false;
}


extern "C" HANDLE openCamera(unsigned int camera_index, XI_IMG_FORMAT color_mode, bool start_acquisition) {
  HANDLE handle = NULL;
  bool init_successful = false;
  int retry_counter = 0;
  int max_retries = 3;
  XI_IMG image;
  image.size = SIZE_XI_IMG_V2; // must be initialized
  image.bp = NULL;
  image.bp_size = 0;
  int timeout = 250;

  while ((retry_counter < max_retries) && !init_successful) {
    printf("Try to initialize camera %d\n", camera_index);
    initCam(camera_index, handle, color_mode, true);
    xiSetParamInt(handle, XI_PRM_TRG_SOFTWARE, 1);
    int stat = xiGetImage(handle, timeout, &image);
    if (stat == XI_OK) {
      printf("Successfully initialized camera %d\n", camera_index);
      init_successful = true;
    } else {
      printf("Initialization of camera %d was not successful (error: %d), closing camera and trying again.\n", camera_index, stat);
      closeDevice(handle);
    }
    retry_counter += 1;
  }

  if (!init_successful) {
    printf("FATAL: Could not open camera %d after %d retries. Exiting.\n", camera_index, max_retries);
    exit(EXIT_FAILURE);
  } else {
    if (!start_acquisition) {
      int stat = xiStopAcquisition(handle);
      HandleResult(stat, "xiStopAcquisition");
    }
  }

  return handle;
}


// This opens two cameras with the given serial numbe. The first provided serial number
// is connected to the first camera and the second serial number accordingly. So if you
// call function getStereoImage the first image returned is the image of the camera
// with the first provided serial number!
extern "C" StereoHandle *openStereoCamerasBySerial(
  const char *serial_cam1,
  const char *serial_cam2,
  XI_IMG_FORMAT color_mode
) {

  int nCams = 0;
  getNumberConnectedDevices(&nCams);

  StereoHandle *handle = new StereoHandle();
  handle->cam1 = NULL;
  handle->cam2 = NULL;

  for (size_t i = 0; i < nCams; i++) {
    char serial[20] = { 0 };
    int stat = xiGetDeviceInfoString(i, XI_PRM_DEVICE_SN, serial, 20);
    std::cout << "Camera "<< i <<" has serial number " << serial << std::endl;
    if (std::string(serial_cam1) == std::string(serial)) {
      handle->cam1 = openCamera(i, color_mode, true);
      std::cout << "Camera 1 found and opened" << std::endl;
    }
    if (std::string(serial_cam2) == std::string(serial)) {
      handle->cam2 = openCamera(i, color_mode, true);
      std::cout << "Camera 2 found and opened" << std::endl;
    }
  }

  if (handle->cam1 == NULL || handle->cam2 == NULL) {
    closeStereoCameras(handle);   // close device connection if any
    std::cout << "camera 1 or camera 2 not found! -> Cannot open Stero camera!" << std::endl;
    return NULL;
  }

  return handle;
}


extern "C" StereoHandle *openStereoCamera(XI_IMG_FORMAT color_mode) {
  int nCams = 0;
  getNumberConnectedDevices(&nCams);
  if (nCams < 2) {
    return NULL;
  }

  StereoHandle *handle = new StereoHandle();
  handle->cam1 = openCamera(0, color_mode, true);
  handle->cam2 = openCamera(1, color_mode, true);
  return handle;
}


extern "C" bool getStereoImage(
  StereoHandle *handle,
  XI_IMG_FORMAT color_mode,
  THByteTensor *image1,
  THByteTensor *image2
) {
  if (handle == NULL) {
    return false;
  }

  bool cam1_ok = getSingleImage(handle->cam1, color_mode, image1, false);
  bool cam2_ok = getSingleImage(handle->cam2, color_mode, image2, false);

  return cam1_ok && cam2_ok;
}


extern "C" bool closeStereoCameras(StereoHandle *handle) {
  if (handle == NULL)
    return false;

  if (handle->cam1) {
    closeDevice(handle->cam1);
    handle->cam1 = NULL;
  }
  if (handle->cam2) {
    closeDevice(handle->cam2);
    handle->cam2 = NULL;
  }

  delete handle;
  return true;
}


extern "C" bool setExposure(HANDLE handle, int micro_sec)
{
  if (handle == NULL) {
    return false;
  }

  int stat = xiSetParamInt(handle, XI_PRM_EXPOSURE, micro_sec);   // us
  if (stat != XI_OK) {
    return false;
  }

 return true;
}


extern "C" bool setExposureCam1(StereoHandle *handle, int micro_sec) {
  return setExposure(handle->cam1, micro_sec);
}


extern "C" bool setExposureCam2(StereoHandle *handle, int micro_sec) {
  return setExposure(handle->cam2, micro_sec);
}


extern "C" bool setExposureStereo(StereoHandle *handle, int micro_sec) {
  bool cam1_ok = setExposure(handle->cam1, micro_sec);
  bool cam2_ok = setExposure(handle->cam2, micro_sec);
  return cam1_ok && cam2_ok;
}
