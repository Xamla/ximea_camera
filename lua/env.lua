local ffi = require 'ffi'

local ximea = {}
local ximea_cdef = [[
  typedef struct HANDLE {} HANDLE;
  typedef struct StereoHandle {} StereoHandle;
  typedef enum 
  {
	XI_MONO8                     =0, // 8 bits per pixel
	XI_MONO16                    =1, // 16 bits per pixel
	XI_RGB24                     =2, // RGB data format
	XI_RGB32                     =3, // RGBA data format
	XI_RGB_PLANAR                =4, // RGB planar data format
	XI_RAW8                      =5, // 8 bits per pixel raw data from sensor
	XI_RAW16                     =6, // 16 bits per pixel raw data from sensor
	XI_FRM_TRANSPORT_DATA        =7, // Data from transport layer (e.g. packed). Format see XI_PRM_TRANSPORT_PIXEL_FORMAT
} XI_IMG_FORMAT;


  bool setExposureCam1(StereoHandle* handle, int micro_sec);
  bool setExposureCam2(StereoHandle* handle, int micro_sec); 


  bool getSerialNumber(int camNum, char* serial);
  bool getSerialsStereo(StereoHandle* handle, char* serial_cam1, char* serial_cam2);
  StereoHandle* openStereoCamerasBySerial(const char* serial_cam1, const char* serial_cam2, XI_IMG_FORMAT color_mode);
  StereoHandle* openStereoCamera(XI_IMG_FORMAT color);
  bool getStereoImage(StereoHandle* handle, XI_IMG_FORMAT color_mode, THByteTensor* image1, THByteTensor* image2);
  bool closeStereoCameras(StereoHandle* handle);
  bool setExposureStereo(StereoHandle* handle, int micro_sec);
  
  HANDLE* openCamera(unsigned int camNum, XI_IMG_FORMAT color_mode);

  bool getSingleImage(HANDLE* handle, XI_IMG_FORMAT format,  THByteTensor* image_out);
  bool setExposure(HANDLE* handle, int micro_sec);
  void getNumberConnectedDevices(int* n);
  void closeDevice(HANDLE* handle);
]]

ffi.cdef(ximea_cdef)
ximea.lib = ffi.load(package.searchpath('libXimea', package.cpath))

return ximea
