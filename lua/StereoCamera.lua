local ffi = require 'ffi'
local ximea = require 'ximea.env'

local StereoCam = torch.class('ximea.StereoCam', ximea)

function StereoCam:getImage()
  local img_cam1
  local img_cam2

  if self.mode == ffi.C.XI_MONO8 then
    img_cam1 = torch.ByteTensor(0,0)
    img_cam2 = torch.ByteTensor(0,0)
  else 
    img_cam1 = torch.ByteTensor(0,0,0)
    img_cam2 = torch.ByteTensor(0,0,0)
  end
  if ximea.lib.getStereoImage(self.o, self.mode, img_cam1:cdata(), img_cam2:cdata()) then 
    return img_cam1, img_cam2, true
  else
    return nil, nil, false
  end
end

function StereoCam:setExposure(exposure_micro_sec)
  ximea.lib.setExposureStereo(self.o, exposure_micro_sec)
end

function StereoCam:getNConnectedDevices()
  local intPtr = ffi.typeof"int[1]"
  local n = intPtr()
  ximea.lib.getNumberConnectedDevices(n)
  return n[0]
end

function StereoCam:close() 
  ximea.lib.closeStereoCameras(self.o)
  self.serial_cam1 = nil
  self.serial_cam2 = nil
end

function StereoCam:setExposureWithSerial(serial, micro_sec)
  if serial == self.serial_cam1 then
    ximea.lib.setExposureCam1(self.o, micro_sec)
  elseif serial == self.serial_cam2 then
    ximea.lib.setExposureCam2(self.o, micro_sec)
  else
    print("Could not set exposure time because no cam with this serial has been initialized")
    return false
  end

  return true
end

function StereoCam:openCameraWithSerial(serial_cam1, serial_cam2, mode) 
  self.mode = ffi.C.XI_MONO8
  if mode ~= nil and  mode == "RGB24" then
    self.mode = ffi.C.XI_RGB24
  end
  local ptr = ximea.lib.openStereoCamerasBySerial(serial_cam1, serial_cam2, self.mode)
  if ptr ~= nil then
    self.o = ptr
    self.serial_cam1 = serial_cam1
    self.serial_cam2 = serial_cam2
  else
    return false
  end  
end

function StereoCam:openCamera(mode)
  self.mode = ffi.C.XI_MONO8
  if mode ~= nil and  mode == "RGB24" then
    self.mode = ffi.C.XI_RGB24
  end

  print("Open mode: ")
  print(self.mode)

  self.o = ximea.lib.openStereoCamera(self.mode)
  
  local serial_cam1 = ffi.new("char[32]")
  local serial_cam2 = ffi.new("char[32]")
  ximea.lib.getSerialsStereo(self.o, serial_cam1, serial_cam2);

  self.serial_cam1 = ffi.string(serial_cam1)
  self.serial_cam2 = ffi.string(serial_cam2)

  print(self.serial_cam1)
  print(self.serial_cam2)
end

function StereoCam:__init() 
  self.serial_cam1 = nil
  self.serial_cam2 = nil    
end
