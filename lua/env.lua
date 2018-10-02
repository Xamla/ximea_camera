--[[
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
]]

local ffi = require 'ffi'


local ximea = {}


local ximea_cdef = [[
typedef struct xiHANDLE_ {} *XI_HANDLE;
typedef struct StereoHandle {} StereoHandle;
typedef enum
{
  XI_MONO8                     = 0, // 8 bits per pixel
  XI_MONO16                    = 1, // 16 bits per pixel
  XI_RGB24                     = 2, // RGB data format
  XI_RGB32                     = 3, // RGBA data format
  XI_RGB_PLANAR                = 4, // RGB planar data format
  XI_RAW8                      = 5, // 8 bits per pixel raw data from sensor
  XI_RAW16                     = 6, // 16 bits per pixel raw data from sensor
  XI_FRM_TRANSPORT_DATA        = 7, // Data from transport layer (e.g. packed). Format see XI_PRM_TRANSPORT_PIXEL_FORMAT
} XI_IMG_FORMAT;

bool setExposureCam1(StereoHandle *handle, int micro_sec);
bool setExposureCam2(StereoHandle *handle, int micro_sec);

bool getSerialNumber(int camNum, char *serial);
bool getSerialsStereo(StereoHandle *handle, char *serial_cam1, char *serial_cam2);
StereoHandle *openStereoCamerasBySerial(const char *serial_cam1, const char *serial_cam2, XI_IMG_FORMAT color_mode);
StereoHandle *openStereoCamera(XI_IMG_FORMAT color);
bool getStereoImage(StereoHandle *handle, XI_IMG_FORMAT color_mode, THByteTensor *image1, THByteTensor *image2);
bool closeStereoCameras(StereoHandle *handle);
bool setExposureStereo(StereoHandle *handle, int micro_sec);

XI_HANDLE openCamera(unsigned int camera_index, XI_IMG_FORMAT color_mode, bool start_acquisition);
bool getSingleImage(XI_HANDLE handle, XI_IMG_FORMAT format, THByteTensor *image_out, bool triggered, int timeout);
bool setExposure(XI_HANDLE handle, int micro_sec);
void getNumberConnectedDevices(int *n);
void closeDevice(XI_HANDLE handle);
]]


-- direct imports from xiAPI
local m3api_cdef = [[
typedef uint32_t DWORD;
typedef int XI_RETURN;

// Error codes xiApi
typedef enum
{
	XI_OK                             = 0, // Function call succeeded
	XI_INVALID_HANDLE                 = 1, // Invalid handle
	XI_READREG                        = 2, // Register read error
	XI_WRITEREG                       = 3, // Register write error
	XI_FREE_RESOURCES                 = 4, // Freeing resources error
	XI_FREE_CHANNEL                   = 5, // Freeing channel error
	XI_FREE_BANDWIDTH                 = 6, // Freeing bandwith error
	XI_READBLK                        = 7, // Read block error
	XI_WRITEBLK                       = 8, // Write block error
	XI_NO_IMAGE                       = 9, // No image
	XI_TIMEOUT                        =10, // Timeout
	XI_INVALID_ARG                    =11, // Invalid arguments supplied
	XI_NOT_SUPPORTED                  =12, // Not supported
	XI_ISOCH_ATTACH_BUFFERS           =13, // Attach buffers error
	XI_GET_OVERLAPPED_RESULT          =14, // Overlapped result
	XI_MEMORY_ALLOCATION              =15, // Memory allocation error
	XI_DLLCONTEXTISNULL               =16, // DLL context is NULL
	XI_DLLCONTEXTISNONZERO            =17, // DLL context is non zero
	XI_DLLCONTEXTEXIST                =18, // DLL context exists
	XI_TOOMANYDEVICES                 =19, // Too many devices connected
	XI_ERRORCAMCONTEXT                =20, // Camera context error
	XI_UNKNOWN_HARDWARE               =21, // Unknown hardware
	XI_INVALID_TM_FILE                =22, // Invalid TM file
	XI_INVALID_TM_TAG                 =23, // Invalid TM tag
	XI_INCOMPLETE_TM                  =24, // Incomplete TM
	XI_BUS_RESET_FAILED               =25, // Bus reset error
	XI_NOT_IMPLEMENTED                =26, // Not implemented
	XI_SHADING_TOOBRIGHT              =27, // Shading is too bright
	XI_SHADING_TOODARK                =28, // Shading is too dark
	XI_TOO_LOW_GAIN                   =29, // Gain is too low
	XI_INVALID_BPL                    =30, // Invalid bad pixel list
	XI_BPL_REALLOC                    =31, // Bad pixel list realloc error
	XI_INVALID_PIXEL_LIST             =32, // Invalid pixel list
	XI_INVALID_FFS                    =33, // Invalid Flash File System
	XI_INVALID_PROFILE                =34, // Invalid profile
	XI_INVALID_CALIBRATION            =35, // Invalid calibration
	XI_INVALID_BUFFER                 =36, // Invalid buffer
	XI_INVALID_DATA                   =38, // Invalid data
	XI_TGBUSY                         =39, // Timing generator is busy
	XI_IO_WRONG                       =40, // Wrong operation open/write/read/close
	XI_ACQUISITION_ALREADY_UP         =41, // Acquisition already started
	XI_OLD_DRIVER_VERSION             =42, // Old version of device driver installed to the system.
	XI_GET_LAST_ERROR                 =43, // To get error code please call GetLastError function.
	XI_CANT_PROCESS                   =44, // Data cannot be processed
	XI_ACQUISITION_STOPED             =45, // Acquisition is stopped. It needs to be started to perform operation.
	XI_ACQUISITION_STOPED_WERR        =46, // Acquisition has been stopped with an error.
	XI_INVALID_INPUT_ICC_PROFILE      =47, // Input ICC profile missing or corrupted
	XI_INVALID_OUTPUT_ICC_PROFILE     =48, // Output ICC profile missing or corrupted
	XI_DEVICE_NOT_READY               =49, // Device not ready to operate
	XI_SHADING_TOOCONTRAST            =50, // Shading is too contrast
	XI_ALREADY_INITIALIZED            =51, // Module already initialized
	XI_NOT_ENOUGH_PRIVILEGES          =52, // Application does not have enough privileges (one or more app)
	XI_NOT_COMPATIBLE_DRIVER          =53, // Installed driver is not compatible with current software
	XI_TM_INVALID_RESOURCE            =54, // TM file was not loaded successfully from resources
	XI_DEVICE_HAS_BEEN_RESETED        =55, // Device has been reset, abnormal initial state
	XI_NO_DEVICES_FOUND               =56, // No Devices Found
	XI_RESOURCE_OR_FUNCTION_LOCKED    =57, // Resource (device) or function locked by mutex
	XI_BUFFER_SIZE_TOO_SMALL          =58, // Buffer provided by user is too small
	XI_COULDNT_INIT_PROCESSOR         =59, // Couldnt initialize processor.
	XI_NOT_INITIALIZED                =60, // The object/module/procedure/process being referred to has not been started.
	XI_RESOURCE_NOT_FOUND             =61, // Resource not found(could be processor, file, item...).
	XI_UNKNOWN_PARAM                  =100, // Unknown parameter
	XI_WRONG_PARAM_VALUE              =101, // Wrong parameter value
	XI_WRONG_PARAM_TYPE               =103, // Wrong parameter type
	XI_WRONG_PARAM_SIZE               =104, // Wrong parameter size
	XI_BUFFER_TOO_SMALL               =105, // Input buffer is too small
	XI_NOT_SUPPORTED_PARAM            =106, // Parameter is not supported
	XI_NOT_SUPPORTED_PARAM_INFO       =107, // Parameter info not supported
	XI_NOT_SUPPORTED_DATA_FORMAT      =108, // Data format is not supported
	XI_READ_ONLY_PARAM                =109, // Read only parameter
	XI_BANDWIDTH_NOT_SUPPORTED        =111, // This camera does not support currently available bandwidth
	XI_INVALID_FFS_FILE_NAME          =112, // FFS file selector is invalid or NULL
	XI_FFS_FILE_NOT_FOUND             =113, // FFS file not found
	XI_PARAM_NOT_SETTABLE             =114, // Parameter value cannot be set (might be out of range or invalid).
	XI_SAFE_POLICY_NOT_SUPPORTED      =115, // Safe buffer policy is not supported. E.g. when transport target is set to GPU (GPUDirect).
	XI_GPUDIRECT_NOT_AVAILABLE        =116, // GPUDirect is not available. E.g. platform isn't supported or CUDA toolkit isn't installed.
	XI_PROC_OTHER_ERROR               =201, // Processing error - other
	XI_PROC_PROCESSING_ERROR          =202, // Error while image processing.
	XI_PROC_INPUT_FORMAT_UNSUPPORTED  =203, // Input format is not supported for processing.
	XI_PROC_OUTPUT_FORMAT_UNSUPPORTED =204, // Output format is not supported for processing.
} XI_RET;

typedef enum
{
	XI_TRG_OFF                   =0, // Camera works in free run mode.
	XI_TRG_EDGE_RISING           =1, // External trigger (rising edge).
	XI_TRG_EDGE_FALLING          =2, // External trigger (falling edge).
	XI_TRG_SOFTWARE              =3, // Software(manual) trigger.
} XI_TRG_SOURCE;

XI_RETURN xiSetParamInt(XI_HANDLE hDevice, const char* prm, const int val);
XI_RETURN xiSetParamFloat(XI_HANDLE hDevice, const char* prm, const float val);
XI_RETURN xiSetParamString(XI_HANDLE hDevice, const char* prm, void* val, DWORD size);
XI_RETURN xiGetParamInt(XI_HANDLE hDevice, const char* prm, int* val);
XI_RETURN xiGetParamFloat(XI_HANDLE hDevice, const char* prm, float* val);
XI_RETURN xiGetParamString(XI_HANDLE hDevice, const char* prm, void* val, DWORD size);
XI_RETURN xiGetDeviceInfoInt(DWORD DevId, const char* prm, int* value);
XI_RETURN xiGetDeviceInfoString(DWORD DevId, const char* prm, char* value, DWORD value_size);

XI_RETURN xiStartAcquisition(XI_HANDLE hDevice);
XI_RETURN xiStopAcquisition(XI_HANDLE hDevice);
XI_RETURN xiGetNumberDevices(DWORD *pNumberDevices);
]]


ffi.cdef(ximea_cdef)
ximea.lib = ffi.load(package.searchpath('libXimea', package.cpath))

ffi.cdef(m3api_cdef)
ximea.m3api = ffi.load('m3api')



local XI_IMG_FORMAT = {
  MONO8               = ffi.C.XI_MONO8,                 -- 8 bits per pixel
  MONO16              = ffi.C.XI_MONO16,                -- 16 bits per pixel
  RGB24               = ffi.C.XI_RGB24,                 -- RGB data format
  RGB32               = ffi.C.XI_RGB32,                 -- RGBA data format
  RGB_PLANAR          = ffi.C.XI_RGB_PLANAR,            -- RGB planar data format
  RAW8                = ffi.C.XI_RAW8,                  -- 8 bits per pixel raw data from sensor
  RAW16               = ffi.C.XI_RAW16,                 -- 16 bits per pixel raw data from sensor
  FRM_TRANSPORT_DATA  = ffi.C.XI_FRM_TRANSPORT_DATA     -- Data from transport layer (e.g. packed). Format see XI_PRM_TRANSPORT_PIXEL_FORMAT
}
ximea.XI_IMG_FORMAT = XI_IMG_FORMAT


local XI_RET = {
  XI_OK                             = 0, -- Function call succeeded
  XI_INVALID_HANDLE                 = 1, -- Invalid handle
  XI_READREG                        = 2, -- Register read error
  XI_WRITEREG                       = 3, -- Register write error
  XI_FREE_RESOURCES                 = 4, -- Freeing resources error
  XI_FREE_CHANNEL                   = 5, -- Freeing channel error
  XI_FREE_BANDWIDTH                 = 6, -- Freeing bandwith error
  XI_READBLK                        = 7, -- Read block error
  XI_WRITEBLK                       = 8, -- Write block error
  XI_NO_IMAGE                       = 9, -- No image
  XI_TIMEOUT                        =10, -- Timeout
  XI_INVALID_ARG                    =11, -- Invalid arguments supplied
  XI_NOT_SUPPORTED                  =12, -- Not supported
  XI_ISOCH_ATTACH_BUFFERS           =13, -- Attach buffers error
  XI_GET_OVERLAPPED_RESULT          =14, -- Overlapped result
  XI_MEMORY_ALLOCATION              =15, -- Memory allocation error
  XI_DLLCONTEXTISNULL               =16, -- DLL context is NULL
  XI_DLLCONTEXTISNONZERO            =17, -- DLL context is non zero
  XI_DLLCONTEXTEXIST                =18, -- DLL context exists
  XI_TOOMANYDEVICES                 =19, -- Too many devices connected
  XI_ERRORCAMCONTEXT                =20, -- Camera context error
  XI_UNKNOWN_HARDWARE               =21, -- Unknown hardware
  XI_INVALID_TM_FILE                =22, -- Invalid TM file
  XI_INVALID_TM_TAG                 =23, -- Invalid TM tag
  XI_INCOMPLETE_TM                  =24, -- Incomplete TM
  XI_BUS_RESET_FAILED               =25, -- Bus reset error
  XI_NOT_IMPLEMENTED                =26, -- Not implemented
  XI_SHADING_TOOBRIGHT              =27, -- Shading is too bright
  XI_SHADING_TOODARK                =28, -- Shading is too dark
  XI_TOO_LOW_GAIN                   =29, -- Gain is too low
  XI_INVALID_BPL                    =30, -- Invalid bad pixel list
  XI_BPL_REALLOC                    =31, -- Bad pixel list realloc error
  XI_INVALID_PIXEL_LIST             =32, -- Invalid pixel list
  XI_INVALID_FFS                    =33, -- Invalid Flash File System
  XI_INVALID_PROFILE                =34, -- Invalid profile
  XI_INVALID_CALIBRATION            =35, -- Invalid calibration
  XI_INVALID_BUFFER                 =36, -- Invalid buffer
  XI_INVALID_DATA                   =38, -- Invalid data
  XI_TGBUSY                         =39, -- Timing generator is busy
  XI_IO_WRONG                       =40, -- Wrong operation open/write/read/close
  XI_ACQUISITION_ALREADY_UP         =41, -- Acquisition already started
  XI_OLD_DRIVER_VERSION             =42, -- Old version of device driver installed to the system.
  XI_GET_LAST_ERROR                 =43, -- To get error code please call GetLastError function.
  XI_CANT_PROCESS                   =44, -- Data cannot be processed
  XI_ACQUISITION_STOPED             =45, -- Acquisition is stopped. It needs to be started to perform operation.
  XI_ACQUISITION_STOPED_WERR        =46, -- Acquisition has been stopped with an error.
  XI_INVALID_INPUT_ICC_PROFILE      =47, -- Input ICC profile missing or corrupted
  XI_INVALID_OUTPUT_ICC_PROFILE     =48, -- Output ICC profile missing or corrupted
  XI_DEVICE_NOT_READY               =49, -- Device not ready to operate
  XI_SHADING_TOOCONTRAST            =50, -- Shading is too contrast
  XI_ALREADY_INITIALIZED            =51, -- Module already initialized
  XI_NOT_ENOUGH_PRIVILEGES          =52, -- Application does not have enough privileges (one or more app)
  XI_NOT_COMPATIBLE_DRIVER          =53, -- Installed driver is not compatible with current software
  XI_TM_INVALID_RESOURCE            =54, -- TM file was not loaded successfully from resources
  XI_DEVICE_HAS_BEEN_RESETED        =55, -- Device has been reset, abnormal initial state
  XI_NO_DEVICES_FOUND               =56, -- No Devices Found
  XI_RESOURCE_OR_FUNCTION_LOCKED    =57, -- Resource (device) or function locked by mutex
  XI_BUFFER_SIZE_TOO_SMALL          =58, -- Buffer provided by user is too small
  XI_COULDNT_INIT_PROCESSOR         =59, -- Couldnt initialize processor.
  XI_NOT_INITIALIZED                =60, -- The object/module/procedure/process being referred to has not been started.
  XI_RESOURCE_NOT_FOUND             =61, -- Resource not found(could be processor, file, item...).
  XI_UNKNOWN_PARAM                  =100, -- Unknown parameter
  XI_WRONG_PARAM_VALUE              =101, -- Wrong parameter value
  XI_WRONG_PARAM_TYPE               =103, -- Wrong parameter type
  XI_WRONG_PARAM_SIZE               =104, -- Wrong parameter size
  XI_BUFFER_TOO_SMALL               =105, -- Input buffer is too small
  XI_NOT_SUPPORTED_PARAM            =106, -- Parameter is not supported
  XI_NOT_SUPPORTED_PARAM_INFO       =107, -- Parameter info not supported
  XI_NOT_SUPPORTED_DATA_FORMAT      =108, -- Data format is not supported
  XI_READ_ONLY_PARAM                =109, -- Read only parameter
  XI_BANDWIDTH_NOT_SUPPORTED        =111, -- This camera does not support currently available bandwidth
  XI_INVALID_FFS_FILE_NAME          =112, -- FFS file selector is invalid or NULL
  XI_FFS_FILE_NOT_FOUND             =113, -- FFS file not found
  XI_PARAM_NOT_SETTABLE             =114, -- Parameter value cannot be set (might be out of range or invalid).
  XI_SAFE_POLICY_NOT_SUPPORTED      =115, -- Safe buffer policy is not supported. E.g. when transport target is set to GPU (GPUDirect).
  XI_GPUDIRECT_NOT_AVAILABLE        =116, -- GPUDirect is not available. E.g. platform isn't supported or CUDA toolkit isn't installed.
  XI_PROC_OTHER_ERROR               =201, -- Processing error - other
  XI_PROC_PROCESSING_ERROR          =202, -- Error while image processing.
  XI_PROC_INPUT_FORMAT_UNSUPPORTED  =203, -- Input format is not supported for processing.
  XI_PROC_OUTPUT_FORMAT_UNSUPPORTED =204, -- Output format is not supported for processing.
}
ximea.XI_RET = XI_RET


local XI_RET_TEXT = {
  [XI_RET.XI_OK]                             = "Function call succeeded",
  [XI_RET.XI_INVALID_HANDLE]                 = "Invalid handle",
  [XI_RET.XI_READREG]                        = "Register read error",
  [XI_RET.XI_WRITEREG]                       = "Register write error",
  [XI_RET.XI_FREE_RESOURCES]                 = "Freeing resources error",
  [XI_RET.XI_FREE_CHANNEL]                   = "Freeing channel error",
  [XI_RET.XI_FREE_BANDWIDTH]                 = "Freeing bandwith error",
  [XI_RET.XI_READBLK]                        = "Read block error",
  [XI_RET.XI_WRITEBLK]                       = "Write block error",
  [XI_RET.XI_NO_IMAGE]                       = "No image",
  [XI_RET.XI_TIMEOUT]                        = "Timeout",
  [XI_RET.XI_INVALID_ARG]                    = "Invalid arguments supplied",
  [XI_RET.XI_NOT_SUPPORTED]                  = "Not supported",
  [XI_RET.XI_ISOCH_ATTACH_BUFFERS]           = "Attach buffers error",
  [XI_RET.XI_GET_OVERLAPPED_RESULT]          = "Overlapped result",
  [XI_RET.XI_MEMORY_ALLOCATION]              = "Memory allocation error",
  [XI_RET.XI_DLLCONTEXTISNULL]               = "DLL context is NULL",
  [XI_RET.XI_DLLCONTEXTISNONZERO]            = "DLL context is non zero",
  [XI_RET.XI_DLLCONTEXTEXIST]                = "DLL context exists",
  [XI_RET.XI_TOOMANYDEVICES]                 = "Too many devices connected",
  [XI_RET.XI_ERRORCAMCONTEXT]                = "Camera context error",
  [XI_RET.XI_UNKNOWN_HARDWARE]               = "Unknown hardware",
  [XI_RET.XI_INVALID_TM_FILE]                = "Invalid TM file",
  [XI_RET.XI_INVALID_TM_TAG]                 = "Invalid TM tag",
  [XI_RET.XI_INCOMPLETE_TM]                  = "Incomplete TM",
  [XI_RET.XI_BUS_RESET_FAILED]               = "Bus reset error",
  [XI_RET.XI_NOT_IMPLEMENTED]                = "Not implemented",
  [XI_RET.XI_SHADING_TOOBRIGHT]              = "Shading is too bright",
  [XI_RET.XI_SHADING_TOODARK]                = "Shading is too dark",
  [XI_RET.XI_TOO_LOW_GAIN]                   = "Gain is too low",
  [XI_RET.XI_INVALID_BPL]                    = "Invalid bad pixel list",
  [XI_RET.XI_BPL_REALLOC]                    = "Bad pixel list realloc error",
  [XI_RET.XI_INVALID_PIXEL_LIST]             = "Invalid pixel list",
  [XI_RET.XI_INVALID_FFS]                    = "Invalid Flash File System",
  [XI_RET.XI_INVALID_PROFILE]                = "Invalid profile",
  [XI_RET.XI_INVALID_CALIBRATION]            = "Invalid calibration",
  [XI_RET.XI_INVALID_BUFFER]                 = "Invalid buffer",
  [XI_RET.XI_INVALID_DATA]                   = "Invalid data",
  [XI_RET.XI_TGBUSY]                         = "Timing generator is busy",
  [XI_RET.XI_IO_WRONG]                       = "Wrong operation open/write/read/close",
  [XI_RET.XI_ACQUISITION_ALREADY_UP]         = "Acquisition already started",
  [XI_RET.XI_OLD_DRIVER_VERSION]             = "Old version of device driver installed to the system.",
  [XI_RET.XI_GET_LAST_ERROR]                 = "To get error code please call GetLastError function.",
  [XI_RET.XI_CANT_PROCESS]                   = "Data cannot be processed",
  [XI_RET.XI_ACQUISITION_STOPED]             = "Acquisition is stopped. It needs to be started to perform operation.",
  [XI_RET.XI_ACQUISITION_STOPED_WERR]        = "Acquisition has been stopped with an error.",
  [XI_RET.XI_INVALID_INPUT_ICC_PROFILE]      = "Input ICC profile missing or corrupted",
  [XI_RET.XI_INVALID_OUTPUT_ICC_PROFILE]     = "Output ICC profile missing or corrupted",
  [XI_RET.XI_DEVICE_NOT_READY]               = "Device not ready to operate",
  [XI_RET.XI_SHADING_TOOCONTRAST]            = "Shading is too contrast",
  [XI_RET.XI_ALREADY_INITIALIZED]            = "Module already initialized",
  [XI_RET.XI_NOT_ENOUGH_PRIVILEGES]          = "Application does not have enough privileges (one or more app)",
  [XI_RET.XI_NOT_COMPATIBLE_DRIVER]          = "Installed driver is not compatible with current software",
  [XI_RET.XI_TM_INVALID_RESOURCE]            = "TM file was not loaded successfully from resources",
  [XI_RET.XI_DEVICE_HAS_BEEN_RESETED]        = "Device has been reset, abnormal initial state",
  [XI_RET.XI_NO_DEVICES_FOUND]               = "No Devices Found",
  [XI_RET.XI_RESOURCE_OR_FUNCTION_LOCKED]    = "Resource (device) or function locked by mutex",
  [XI_RET.XI_BUFFER_SIZE_TOO_SMALL]          = "Buffer provided by user is too small",
  [XI_RET.XI_COULDNT_INIT_PROCESSOR]         = "Couldnt initialize processor.",
  [XI_RET.XI_NOT_INITIALIZED]                = "The object/module/procedure/process being referred to has not been started.",
  [XI_RET.XI_RESOURCE_NOT_FOUND]             = "Resource not found(could be processor, file, item...).",
  [XI_RET.XI_UNKNOWN_PARAM]                  = "Unknown parameter",
  [XI_RET.XI_WRONG_PARAM_VALUE]              = "Wrong parameter value",
  [XI_RET.XI_WRONG_PARAM_TYPE]               = "Wrong parameter type",
  [XI_RET.XI_WRONG_PARAM_SIZE]               = "Wrong parameter size",
  [XI_RET.XI_BUFFER_TOO_SMALL]               = "Input buffer is too small",
  [XI_RET.XI_NOT_SUPPORTED_PARAM]            = "Parameter is not supported",
  [XI_RET.XI_NOT_SUPPORTED_PARAM_INFO]       = "Parameter info not supported",
  [XI_RET.XI_NOT_SUPPORTED_DATA_FORMAT]      = "Data format is not supported",
  [XI_RET.XI_READ_ONLY_PARAM]                = "Read only parameter",
  [XI_RET.XI_BANDWIDTH_NOT_SUPPORTED]        = "This camera does not support currently available bandwidth",
  [XI_RET.XI_INVALID_FFS_FILE_NAME]          = "FFS file selector is invalid or NULL",
  [XI_RET.XI_FFS_FILE_NOT_FOUND]             = "FFS file not found",
  [XI_RET.XI_PARAM_NOT_SETTABLE]             = "Parameter value cannot be set (might be out of range or invalid).",
  [XI_RET.XI_SAFE_POLICY_NOT_SUPPORTED]      = "Safe buffer policy is not supported. E.g. when transport target is set to GPU (GPUDirect).",
  [XI_RET.XI_GPUDIRECT_NOT_AVAILABLE]        = "GPUDirect is not available. E.g. platform isn't supported or CUDA toolkit isn't installed.",
  [XI_RET.XI_PROC_OTHER_ERROR]               = "Processing error - other",
  [XI_RET.XI_PROC_PROCESSING_ERROR]          = "Error while image processing.",
  [XI_RET.XI_PROC_INPUT_FORMAT_UNSUPPORTED]  = "Input format is not supported for processing.",
  [XI_RET.XI_PROC_OUTPUT_FORMAT_UNSUPPORTED] = "Output format is not supported for processing.",
}
ximea.XI_RET_TEXT = XI_RET_TEXT


local PARAM = {
  XI_PRM_EXPOSURE                       = "exposure",               -- Exposure time in microseconds
  XI_PRM_EXPOSURE_BURST_COUNT           = "exposure_burst_count",   -- Sets the number of times of exposure in one frame.
  XI_PRM_GAIN_SELECTOR                  = "gain_selector",          -- Gain selector for parameter Gain allows to select different type of gains. XI_GAIN_SELECTOR_TYPE
  XI_PRM_GAIN                           = "gain",                   -- Gain in dB
  XI_PRM_DOWNSAMPLING                   = "downsampling",           -- Change image resolution by binning or skipping. XI_DOWNSAMPLING_VALUE
  XI_PRM_DOWNSAMPLING_TYPE              = "downsampling_type",      -- Change image downsampling type. XI_DOWNSAMPLING_TYPE
  XI_PRM_BINNING_SELECTOR               = "binning_selector",       -- Binning engine selector. XI_BIN_SELECTOR
  XI_PRM_BINNING_VERTICAL               = "binning_vertical",       -- Vertical Binning - number of vertical photo-sensitive cells to combine together.
  XI_PRM_BINNING_HORIZONTAL             = "binning_horizontal",     -- Horizontal Binning - number of horizontal photo-sensitive cells to combine together.
  XI_PRM_BINNING_PATTERN                = "binning_pattern",        -- Binning pattern type. XI_BIN_PATTERN
  XI_PRM_DECIMATION_SELECTOR            = "decimation_selector",    -- Decimation engine selector. XI_DEC_SELECTOR
  XI_PRM_DECIMATION_VERTICAL            = "decimation_vertical",    -- Vertical Decimation - vertical sub-sampling of the image - reduces the vertical resolution of the image by the specified vertical decimation factor.
  XI_PRM_DECIMATION_HORIZONTAL          = "decimation_horizontal",  -- Horizontal Decimation - horizontal sub-sampling of the image - reduces the horizontal resolution of the image by the specified vertical decimation factor.
  XI_PRM_DECIMATION_PATTERN             = "decimation_pattern",     -- Decimation pattern type. XI_DEC_PATTERN
  XI_PRM_TEST_PATTERN_GENERATOR_SELECTOR= "test_pattern_generator_selector", -- Selects which test pattern generator is controlled by the TestPattern feature. XI_TEST_PATTERN_GENERATOR
  XI_PRM_TEST_PATTERN                   = "test_pattern",           -- Selects which test pattern type is generated by the selected generator. XI_TEST_PATTERN
  XI_PRM_IMAGE_DATA_FORMAT              = "imgdataformat",          -- Output data format. XI_IMG_FORMAT
  XI_PRM_SHUTTER_TYPE                   = "shutter_type",           -- Change sensor shutter type(CMOS sensor). XI_SHUTTER_TYPE
  XI_PRM_SENSOR_TAPS                    = "sensor_taps",            -- Number of taps XI_SENSOR_TAP_CNT
  XI_PRM_AEAG                           = "aeag",                   -- Automatic exposure/gain
  XI_PRM_AEAG_ROI_OFFSET_X              = "aeag_roi_offset_x",      -- Automatic exposure/gain ROI offset X
  XI_PRM_AEAG_ROI_OFFSET_Y              = "aeag_roi_offset_y",      -- Automatic exposure/gain ROI offset Y
  XI_PRM_AEAG_ROI_WIDTH                 = "aeag_roi_width",         -- Automatic exposure/gain ROI Width
  XI_PRM_AEAG_ROI_HEIGHT                = "aeag_roi_height",        -- Automatic exposure/gain ROI Height
  XI_PRM_BPC                            = "bpc",                    -- Correction of bad pixels
  XI_PRM_AUTO_WB                        = "auto_wb",                -- Automatic white balance
  XI_PRM_MANUAL_WB                      = "manual_wb",              -- Calculates White Balance(xiGetImage function must be called)
  XI_PRM_WB_KR                          = "wb_kr",                  -- White balance red coefficient
  XI_PRM_WB_KG                          = "wb_kg",                  -- White balance green coefficient
  XI_PRM_WB_KB                          = "wb_kb",                  -- White balance blue coefficient
  XI_PRM_WIDTH                          = "width",                  -- Width of the Image provided by the device (in pixels).
  XI_PRM_HEIGHT                         = "height",                 -- Height of the Image provided by the device (in pixels).
  XI_PRM_OFFSET_X                       = "offsetX",                -- Horizontal offset from the origin to the area of interest (in pixels).
  XI_PRM_OFFSET_Y                       = "offsetY",                -- Vertical offset from the origin to the area of interest (in pixels).
  XI_PRM_REGION_SELECTOR                = "region_selector",        -- Selects Region in Multiple ROI which parameters are set by width, height, ... ,region mode
  XI_PRM_REGION_MODE                    = "region_mode",            -- Activates/deactivates Region selected by Region Selector
  XI_PRM_HORIZONTAL_FLIP                = "horizontal_flip",        -- Horizontal flip enable
  XI_PRM_VERTICAL_FLIP                  = "vertical_flip",          -- Vertical flip enable
  XI_PRM_FFC                            = "ffc",                    -- Image flat field correction
  XI_PRM_FFC_FLAT_FIELD_FILE_NAME       = "ffc_flat_field_file_name", -- Set name of file to be applied for FFC processor.
  XI_PRM_FFC_DARK_FIELD_FILE_NAME       = "ffc_dark_field_file_name", -- Set name of file to be applied for FFC processor.
-- AE Setup
  XI_PRM_EXP_PRIORITY                   = "exp_priority",           -- Exposure priority (0.8 - exposure 80%, gain 20%).
  XI_PRM_AG_MAX_LIMIT                   = "ag_max_limit",           -- Maximum limit of gain in AEAG procedure
  XI_PRM_AE_MAX_LIMIT                   = "ae_max_limit",           -- Maximum time (us) used for exposure in AEAG procedure
  XI_PRM_AEAG_LEVEL                     = "aeag_level",             -- Average intensity of output signal AEAG should achieve(in %)
-- Performance
  XI_PRM_LIMIT_BANDWIDTH                = "limit_bandwidth",        -- Set/get bandwidth(datarate)(in Megabits)
  XI_PRM_LIMIT_BANDWIDTH_MODE           = "limit_bandwidth_mode",   -- Bandwidth limit enabled XI_SWITCH
  XI_PRM_SENSOR_DATA_BIT_DEPTH          = "sensor_bit_depth",       -- Sensor output data bit depth. XI_BIT_DEPTH
  XI_PRM_OUTPUT_DATA_BIT_DEPTH          = "output_bit_depth",       -- Device output data bit depth. XI_BIT_DEPTH
  XI_PRM_IMAGE_DATA_BIT_DEPTH           = "image_data_bit_depth",   -- bitdepth of data returned by function xiGetImage XI_BIT_DEPTH
  XI_PRM_OUTPUT_DATA_PACKING            = "output_bit_packing",     -- Device output data packing (or grouping) enabled. Packing could be enabled if output_data_bit_depth > 8 and packing capability is available.
  XI_PRM_OUTPUT_DATA_PACKING_TYPE       = "output_bit_packing_type",-- Data packing type. Some cameras supports only specific packing type. XI_OUTPUT_DATA_PACKING_TYPE
-- Temperature
  XI_PRM_IS_COOLED                      = "iscooled",               -- Returns 1 for cameras that support cooling.
  XI_PRM_COOLING                        = "cooling",                -- Start camera cooling.
  XI_PRM_TARGET_TEMP                    = "target_temp",            -- Set sensor target temperature for cooling.
  XI_PRM_TEMP_SELECTOR                  = "temp_selector",          -- Selector of mechanical point where thermometer is located. XI_TEMP_SELECTOR
  XI_PRM_TEMP                           = "temp",                   -- Camera temperature (selected by XI_PRM_TEMP_SELECTOR)
  XI_PRM_CHIP_TEMP                      = "chip_temp",              -- Camera sensor temperature
  XI_PRM_HOUS_TEMP                      = "hous_temp",              -- Camera housing tepmerature
  XI_PRM_HOUS_BACK_SIDE_TEMP            = "hous_back_side_temp",    -- Camera housing back side tepmerature
  XI_PRM_SENSOR_BOARD_TEMP              = "sensor_board_temp",      -- Camera sensor board temperature
-- Color Correction
  XI_PRM_CMS                            = "cms",                    -- Mode of color management system. XI_CMS_MODE
  XI_PRM_APPLY_CMS                      = "apply_cms",              -- Enable applying of CMS profiles to xiGetImage (see XI_PRM_INPUT_CMS_PROFILE, XI_PRM_OUTPUT_CMS_PROFILE).
  XI_PRM_INPUT_CMS_PROFILE              = "input_cms_profile",      -- Filename for input cms profile (e.g. input.icc)
  XI_PRM_OUTPUT_CMS_PROFILE             = "output_cms_profile",     -- Filename for output cms profile (e.g. input.icc)
  XI_PRM_IMAGE_IS_COLOR                 = "iscolor",                -- Returns 1 for color cameras.
  XI_PRM_COLOR_FILTER_ARRAY             = "cfa",                    -- Returns color filter array type of RAW data. XI_COLOR_FILTER_ARRAY
  XI_PRM_GAMMAY                         = "gammaY",                 -- Luminosity gamma
  XI_PRM_GAMMAC                         = "gammaC",                 -- Chromaticity gamma
  XI_PRM_SHARPNESS                      = "sharpness",              -- Sharpness Strenght
  XI_PRM_CC_MATRIX_00                   = "ccMTX00",                -- Color Correction Matrix element [0][0]
  XI_PRM_CC_MATRIX_01                   = "ccMTX01",                -- Color Correction Matrix element [0][1]
  XI_PRM_CC_MATRIX_02                   = "ccMTX02",                -- Color Correction Matrix element [0][2]
  XI_PRM_CC_MATRIX_03                   = "ccMTX03",                -- Color Correction Matrix element [0][3]
  XI_PRM_CC_MATRIX_10                   = "ccMTX10",                -- Color Correction Matrix element [1][0]
  XI_PRM_CC_MATRIX_11                   = "ccMTX11",                -- Color Correction Matrix element [1][1]
  XI_PRM_CC_MATRIX_12                   = "ccMTX12",                -- Color Correction Matrix element [1][2]
  XI_PRM_CC_MATRIX_13                   = "ccMTX13",                -- Color Correction Matrix element [1][3]
  XI_PRM_CC_MATRIX_20                   = "ccMTX20",                -- Color Correction Matrix element [2][0]
  XI_PRM_CC_MATRIX_21                   = "ccMTX21",                -- Color Correction Matrix element [2][1]
  XI_PRM_CC_MATRIX_22                   = "ccMTX22",                -- Color Correction Matrix element [2][2]
  XI_PRM_CC_MATRIX_23                   = "ccMTX23",                -- Color Correction Matrix element [2][3]
  XI_PRM_CC_MATRIX_30                   = "ccMTX30",                -- Color Correction Matrix element [3][0]
  XI_PRM_CC_MATRIX_31                   = "ccMTX31",                -- Color Correction Matrix element [3][1]
  XI_PRM_CC_MATRIX_32                   = "ccMTX32",                -- Color Correction Matrix element [3][2]
  XI_PRM_CC_MATRIX_33                   = "ccMTX33",                -- Color Correction Matrix element [3][3]
  XI_PRM_DEFAULT_CC_MATRIX              = "defccMTX",               -- Set default Color Correction Matrix
-- Device IO
  XI_PRM_TRG_SOURCE                     = "trigger_source",         -- Defines source of trigger. XI_TRG_SOURCE
  XI_PRM_TRG_SOFTWARE                   = "trigger_software",       -- Generates an internal trigger. XI_PRM_TRG_SOURCE must be set to TRG_SOFTWARE.
  XI_PRM_TRG_SELECTOR                   = "trigger_selector",       -- Selects the type of trigger. XI_TRG_SELECTOR
  XI_PRM_ACQ_FRAME_BURST_COUNT          = "acq_frame_burst_count",  -- Sets number of frames acquired by burst. This burst is used only if trigger is set to FrameBurstStart
-- GPIO Setup
  XI_PRM_GPI_SELECTOR                   = "gpi_selector",           -- Selects GPI XI_GPI_SELECTOR
  XI_PRM_GPI_MODE                       = "gpi_mode",               -- Defines GPI functionality XI_GPI_MODE
  XI_PRM_GPI_LEVEL                      = "gpi_level",              -- GPI level
  XI_PRM_GPO_SELECTOR                   = "gpo_selector",           -- Selects GPO XI_GPO_SELECTOR
  XI_PRM_GPO_MODE                       = "gpo_mode",               -- Defines GPO functionality XI_GPO_MODE
  XI_PRM_LED_SELECTOR                   = "led_selector",           -- Selects LED XI_LED_SELECTOR
  XI_PRM_LED_MODE                       = "led_mode",               -- Defines LED functionality XI_LED_MODE
  XI_PRM_DEBOUNCE_EN                    = "dbnc_en",                -- Enable/Disable debounce to selected GPI
-- Debounce Setup
  XI_PRM_DEBOUNCE_T0                    = "dbnc_t0",                -- Debounce time (x * 10us)
  XI_PRM_DEBOUNCE_T1                    = "dbnc_t1",                -- Debounce time (x * 10us)
  XI_PRM_DEBOUNCE_POL                   = "dbnc_pol",               -- Debounce polarity (pol = 1 t0 - falling edge, t1 - rising edge)
-- Lens Control
  XI_PRM_LENS_MODE                      = "lens_mode",              -- Status of lens control interface. This shall be set to XI_ON before any Lens operations.
  XI_PRM_LENS_APERTURE_VALUE            = "lens_aperture_value",    -- Current lens aperture value in stops. Examples: 2.8, 4, 5.6, 8, 11
  XI_PRM_LENS_FOCUS_MOVEMENT_VALUE      = "lens_focus_movement_value", -- Lens current focus movement value to be used by XI_PRM_LENS_FOCUS_MOVE in motor steps.
  XI_PRM_LENS_FOCUS_MOVE                = "lens_focus_move",        -- Moves lens focus motor by steps set in XI_PRM_LENS_FOCUS_MOVEMENT_VALUE.
  XI_PRM_LENS_FOCUS_DISTANCE            = "lens_focus_distance",    -- Lens focus distance in cm.
  XI_PRM_LENS_FOCAL_LENGTH              = "lens_focal_length",      -- Lens focal distance in mm.
  XI_PRM_LENS_FEATURE_SELECTOR          = "lens_feature_selector",  -- Selects the current feature which is accessible by XI_PRM_LENS_FEATURE. XI_LENS_FEATURE
  XI_PRM_LENS_FEATURE                   = "lens_feature",           -- Allows access to lens feature value currently selected by XI_PRM_LENS_FEATURE_SELECTOR.
-- Device info parameters
  XI_PRM_DEVICE_NAME                    = "device_name",            -- Return device name
  XI_PRM_DEVICE_TYPE                    = "device_type",            -- Return device type
  XI_PRM_DEVICE_MODEL_ID                = "device_model_id",        -- Return device model id
  XI_PRM_DEVICE_SN                      = "device_sn",              -- Return device serial number
  XI_PRM_DEVICE_SNE                     = "device_sne",             -- Return device enhanced serial number (for cameras having both serials legacy and enhanced)
  XI_PRM_DEVICE_SENS_SN                 = "device_sens_sn",         -- Return sensor serial number
  XI_PRM_DEVICE_INSTANCE_PATH           = "device_inst_path",       -- Return device system instance path.
  XI_PRM_DEVICE_LOCATION_PATH           = "device_loc_path",        -- Represents the location of the device in the device tree.
  XI_PRM_DEVICE_USER_ID                 = "device_user_id",         -- Return custom ID of camera.
  XI_PRM_DEVICE_MANIFEST                = "device_manifest",        -- Return device capability description XML.
  XI_PRM_IMAGE_USER_DATA                = "image_user_data",        -- User image data at image header to track parameters synchronization.
-- Device acquisition settings
  XI_PRM_IMAGE_DATA_FORMAT_RGB32_ALPHA  = "imgdataformatrgb32alpha",-- The alpha channel of RGB32 output image format.
  XI_PRM_IMAGE_PAYLOAD_SIZE             = "imgpayloadsize",         -- Buffer size in bytes sufficient for output image returned by xiGetImage
  XI_PRM_TRANSPORT_PIXEL_FORMAT         = "transport_pixel_format", -- Current format of pixels on transport layer. XI_GenTL_Image_Format_e
  XI_PRM_TRANSPORT_DATA_TARGET          = "transport_data_target",  -- Target selector for data - CPU RAM or GPU RAM XI_TRANSPORT_DATA_TARGET_MODE
  XI_PRM_SENSOR_CLOCK_FREQ_HZ           = "sensor_clock_freq_hz",   -- Sensor clock frequency in Hz.
  XI_PRM_SENSOR_CLOCK_FREQ_INDEX        = "sensor_clock_freq_index",-- Sensor clock frequency index. Sensor with selected frequencies have possibility to set the frequency only by this index.
  XI_PRM_SENSOR_OUTPUT_CHANNEL_COUNT    = "sensor_output_channel_count", -- Number of output channels from sensor used for data transfer. XI_SENSOR_OUTPUT_CHANNEL_COUNT
  XI_PRM_FRAMERATE                      = "framerate",              -- Define framerate in Hz
  XI_PRM_COUNTER_SELECTOR               = "counter_selector",       -- Select counter XI_COUNTER_SELECTOR
  XI_PRM_COUNTER_VALUE                  = "counter_value",          -- Counter status
  XI_PRM_ACQ_TIMING_MODE                = "acq_timing_mode",        -- Type of sensor frames timing. XI_ACQ_TIMING_MODE
  XI_PRM_AVAILABLE_BANDWIDTH            = "available_bandwidth",    -- Measure and return available interface bandwidth(int Megabits)
  XI_PRM_BUFFER_POLICY                  = "buffer_policy",          -- Data move policy XI_BP
  XI_PRM_LUT_EN                         = "LUTEnable",              -- Activates LUT.
  XI_PRM_LUT_INDEX                      = "LUTIndex",               -- Control the index (offset) of the coefficient to access in the LUT.
  XI_PRM_LUT_VALUE                      = "LUTValue",               -- Value at entry LUTIndex of the LUT
  XI_PRM_TRG_DELAY                      = "trigger_delay",          -- Specifies the delay in microseconds (us) to apply after the trigger reception before activating it. XI_GPI_SELECTOR
  XI_PRM_TS_RST_MODE                    = "ts_rst_mode",            -- Defines how time stamp reset engine will be armed XI_TS_RST_MODE
  XI_PRM_TS_RST_SOURCE                  = "ts_rst_source",          -- Defines which source will be used for timestamp reset. Writing this parameter will trigger settings of engine (arming) XI_TS_RST_SOURCE
-- Extended Device parameters
  XI_PRM_IS_DEVICE_EXIST                = "isexist",                -- Returns 1 if camera connected and works properly.
  XI_PRM_ACQ_BUFFER_SIZE                = "acq_buffer_size",        -- Acquisition buffer size in buffer_size_unit. Default bytes.
  XI_PRM_ACQ_BUFFER_SIZE_UNIT           = "acq_buffer_size_unit",   -- Acquisition buffer size unit in bytes. Default 1. E.g. Value 1024 means that buffer_size is in KiBytes
  XI_PRM_ACQ_TRANSPORT_BUFFER_SIZE      = "acq_transport_buffer_size", -- Acquisition transport buffer size in bytes
  XI_PRM_ACQ_TRANSPORT_PACKET_SIZE      = "acq_transport_packet_size", -- Acquisition transport packet size in bytes
  XI_PRM_BUFFERS_QUEUE_SIZE             = "buffers_queue_size",     -- Queue of field/frame buffers
  XI_PRM_ACQ_TRANSPORT_BUFFER_COMMIT    = "acq_transport_buffer_commit", -- Number of buffers to commit to low level
  XI_PRM_RECENT_FRAME                   = "recent_frame",           -- GetImage returns most recent frame
  XI_PRM_DEVICE_RESET                   = "device_reset",           -- Resets the camera to default state.
-- Sensor Defects Correction
  XI_PRM_COLUMN_FPN_CORRECTION          = "column_fpn_correction",  -- Correction of column FPN XI_SWITCH
  XI_PRM_ROW_FPN_CORRECTION             = "row_fpn_correction",     -- Correction of row FPN XI_SWITCH
-- Sensor features
  XI_PRM_SENSOR_MODE                    = "sensor_mode",            -- Current sensor mode. Allows to select sensor mode by one integer. Setting of this parameter affects: image dimensions and downsampling. XI_SENSOR_MODE
  XI_PRM_HDR                            = "hdr",                    -- Enable High Dynamic Range feature.
  XI_PRM_HDR_KNEEPOINT_COUNT            = "hdr_kneepoint_count",    -- The number of kneepoints in the PWLR.
  XI_PRM_HDR_T1                         = "hdr_t1",                 -- position of first kneepoint(in % of XI_PRM_EXPOSURE)
  XI_PRM_HDR_T2                         = "hdr_t2",                 -- position of second kneepoint (in % of XI_PRM_EXPOSURE)
  XI_PRM_KNEEPOINT1                     = "hdr_kneepoint1",         -- value of first kneepoint (% of sensor saturation)
  XI_PRM_KNEEPOINT2                     = "hdr_kneepoint2",         -- value of second kneepoint (% of sensor saturation)
  XI_PRM_IMAGE_BLACK_LEVEL              = "image_black_level",      -- Last image black level counts. Can be used for Offline processing to recall it.
-- Version info
  XI_PRM_API_VERSION                    = "api_version",            -- Returns version of API.
  XI_PRM_DRV_VERSION                    = "drv_version",            -- Returns version of current device driver.
  XI_PRM_MCU1_VERSION                   = "version_mcu1",           -- Returns version of MCU1 firmware.
  XI_PRM_MCU2_VERSION                   = "version_mcu2",           -- Returns version of MCU2 firmware.
  XI_PRM_FPGA1_VERSION                  = "version_fpga1",          -- Returns version of FPGA1 firmware.
  XI_PRM_HW_REVISION                    = "hw_revision",            -- Returns hardware revision number.
-- API features
  XI_PRM_DEBUG_LEVEL                    = "debug_level",            -- Set debug level XI_DEBUG_LEVEL
  XI_PRM_AUTO_BANDWIDTH_CALCULATION     = "auto_bandwidth_calculation", -- Automatic bandwidth calculation,
  XI_PRM_NEW_PROCESS_CHAIN_ENABLE       = "new_process_chain_enable", -- Enables (2015/FAPI) processing chain for MQ MU cameras
-- Camera FFS
  XI_PRM_READ_FILE_FFS                  = "read_file_ffs",          -- Read file from camera flash filesystem.
  XI_PRM_WRITE_FILE_FFS                 = "write_file_ffs",         -- Write file to camera flash filesystem.
  XI_PRM_FFS_FILE_NAME                  = "ffs_file_name",          -- Set name of file to be written/read from camera FFS.
  XI_PRM_FFS_FILE_ID                    = "ffs_file_id",            -- File number.
  XI_PRM_FFS_FILE_SIZE                  = "ffs_file_size",          -- Size of file.
  XI_PRM_FREE_FFS_SIZE                  = "free_ffs_size",          -- Size of free camera FFS.
  XI_PRM_USED_FFS_SIZE                  = "used_ffs_size",          -- Size of used camera FFS.
  XI_PRM_FFS_ACCESS_KEY                 = "ffs_access_key",         -- Setting of key enables file operations on some cameras.
-- APIContextControl
  XI_PRM_API_CONTEXT_LIST               = "xiapi_context_list",     -- List of current parameters settings context - parameters with values. Used for offline processing.
-- Sensor Control
  XI_PRM_SENSOR_FEATURE_SELECTOR        = "sensor_feature_selector",-- Selects the current feature which is accessible by XI_PRM_SENSOR_FEATURE_VALUE. XI_SENSOR_FEATURE_SELECTOR
  XI_PRM_SENSOR_FEATURE_VALUE           = "sensor_feature_value",   -- Allows access to sensor feature value currently selected by XI_PRM_SENSOR_FEATURE_SELECTOR.
}
ximea.PARAM = PARAM


return ximea
