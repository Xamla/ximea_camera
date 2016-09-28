local ffi = require 'ffi'
local ximea = require 'ximea.env'

local SingleCam = torch.class('ximea.SingleCam', ximea)

local DEFAULT_MODE = 'MONO8'

function SingleCam:__init()
end

function SingleCam:getImage()
  local img = torch.ByteTensor()
  if not ximea.lib.getSingleImage(self.o, self.mode, img:cdata()) then
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

function SingleCam:openCamera(number_of_device, mode)
  self.mode = ximea.getXiModeByName(mode or DEFAULT_MODE)

  print('Open mode: ')
  print(self.mode)

  self.o = ximea.lib.openCamera(number_of_device, self.mode)
end

function SingleCam:close()
  ximea.lib.closeDevice(self.o)
end
