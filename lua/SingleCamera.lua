local ffi = require 'ffi'
local ximea = require 'ximea.env'

local SingleCam = torch.class('ximea.SingleCam', ximea)

local DEFAULT_MODE = 'MONO8'

function SingleCam:__init()
end

function SingleCam:getImage()
  local img = torch.ByteTensor()
  if not ximea.lib.getSingleImage(self.o, self.color_mode, img:cdata()) then
    return nil, false
  else
    return img, true
  end
end

function SingleCam:setExposure(exposure_micro_sec)
  ximea.lib.setExposure(self.o, exposure_micro_sec)
end

function SingleCam:getNConnectedDevices()
  local intPtr = ffi.typeof('int[1]')
  local n = intPtr()
  ximea.lib.getNumberConnectedDevices(n)
  return n[0]
end

function SingleCam:openCameraWithSerial(serial, mode)
  local serials = ximea.getSerialNumbers()
  for i=1,#serials do
    if serials[i] == serial then
      self:openCamera(i-1, mode)
      return
    end
  end
end

function SingleCam:openCamera(device_index, mode)
  self.color_mode = ximea.getXiModeByName(mode or DEFAULT_MODE)
  self.device_index = device_index or 0

  print('Open mode: ')
  print(self.color_mode)

  self.o = ximea.lib.openCamera(self.device_index, self.color_mode)

  local c_str = ffi.new("char[32]")
  ximea.lib.getSerialNumber(self.device_index, c_str)
  self.serial = ffi.string(c_str)
end

function SingleCam:getColorMode()
  return self.color_mode
end

function SingleCam:getSerial()
  return self.serial
end

function SingleCam:close()
  if self.o ~= nil then
    ximea.lib.closeDevice(self.o)
    self.o = nil
  end
  self.serial= nil
end

function SingleCam:isOpen()
  return self.o ~= nil
end
