local ffi = require 'ffi'
local ximea = require 'ximea.env'

local StereoCam = torch.class('ximea.StereoCam', ximea)

function StereoCam:__init()
end

function StereoCam:openCamera(color_mode)
  color_mode = ximea.getXiModeByName(color_mode or 'MONO8')

  -- handle calls to open if camera has been opened before
  if self.o ~= nil then
    if self.color_mode == color_mode then
      return
    else
      self:close()
    end
  end

  self.color_mode = color_mode
  print(string.format("Opening with XI_IMG_FORMAT: %d", self.color_mode))

  self.o = ximea.lib.openStereoCamera(self.color_mode)
  if self.o == nil then
    error('Opening stereo cameras failed!')
  end

  ffi.gc(self.o, ximea.lib.closeStereoCameras)        -- register finalizer to free handle

  local serial_cam1 = ffi.new("char[32]")
  local serial_cam2 = ffi.new("char[32]")
  ximea.lib.getSerialsStereo(self.o, serial_cam1, serial_cam2);

  self.serial_cam1 = ffi.string(serial_cam1)
  self.serial_cam2 = ffi.string(serial_cam2)

  print(self.serial_cam1)
  print(self.serial_cam2)
end

function StereoCam:openCameraWithSerial(serial_cam1, serial_cam2, color_mode)
  color_mode = ximea.getXiModeByName(color_mode or 'MONO8')

  -- handle calls to open if camera has been opened before
  if self.o ~= nil then
    if self.serial_cam1 == serial_cam1 and self.serial_cam2 == serial_cam2 and self.color_mode == color_mode then
      return
    else
      self:close()
    end
  end

  local ptr = ximea.lib.openStereoCamerasBySerial(serial_cam1, serial_cam2, color_mode)
  if ptr ~= nil then
    self.o = ptr
    self.color_mode = color_mode
    ffi.gc(self.o, ximea.lib.closeStereoCameras)        -- register finalizer to free handle
    self.serial_cam1 = serial_cam1
    self.serial_cam2 = serial_cam2
  end
  return ptr ~= nil
end

function StereoCam:close()
  if self.o ~= nil then
    ximea.lib.closeStereoCameras(ffi.gc(self.o, nil))   -- unregister finalizer to free handle
    self.o = nil
  end
  self.serial_cam1 = nil
  self.serial_cam2 = nil
end

function StereoCam:isOpen()
  return self.o ~= nil
end

function StereoCam:getImage()
  if self:isOpen() then
    local img_cam1 = torch.ByteTensor()
    local img_cam2 = torch.ByteTensor()
    if ximea.lib.getStereoImage(self.o, self.color_mode, img_cam1:cdata(), img_cam2:cdata()) then
      return img_cam1, img_cam2, true
    end
  end

  return nil, nil, false    -- return values for failure case
end

function StereoCam:getSerials()
  return { self.serial_cam1, self.serial_cam2 }
end

function StereoCam:getColorMode()
  return self.color_mode
end

function StereoCam:setExposure(exposure_micro_sec)
  return ximea.lib.setExposureStereo(self.o, exposure_micro_sec)
end

function StereoCam:getNConnectedDevices()
  local intPtr = ffi.typeof("int[1]")
  local n = intPtr()
  ximea.lib.getNumberConnectedDevices(n)
  return n[0]
end

function StereoCam:setExposureWithSerial(serial, micro_sec)
  if serial == self.serial_cam1 then
    ximea.lib.setExposureCam1(self.o, micro_sec)
  elseif serial == self.serial_cam2 then
    ximea.lib.setExposureCam2(self.o, micro_sec)
  else
    error("Could not set exposure time because no cam with this serial has been initialized.")
  end
end
