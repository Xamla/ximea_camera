local ffi = require 'ffi'

local ximea = require 'ximea.env'


local SingleCam = torch.class('ximea.SingleCam', ximea)

function SingleCam:getImage()
  local img 
  if self.mode == ffi.C.XI_MONO8 then
    img = torch.ByteTensor(0,0)
  else 
    img = torch.ByteTensor(0,0,0)
  end
  if ximea.lib.getSingleImage(self.o, self.mode, img:cdata()) then 
    return img, true
  else
    return nil, false
  end
end

function SingleCam:setExposure(exposure_micro_sec)
  ximea.lib.setExposure(self.o, exposure_micro_sec)
end

function SingleCam:getNConnectedDevices()
  local intPtr = ffi.typeof"int[1]"
  local n = intPtr()
  ximea.lib.getNumberConnectedDevices(n)
  return n[0]
end

function SingleCam:close() 
  ximea.lib.closeDevice(self.o)
end


function SingleCam:openCameraWithSerial(serial, mode) 
  self.mode = ffi.C.XI_MONO8
  if mode ~= nil and  mode == "RGB24" then
    self.mode = ffi.C.XI_RGB24
  end
 
  local serials = ximea.getSerialNumbers()
  for i = 1,#serials do 
    if serials[i] == serial then
      self:openCamera(i-1, mode)
      return
    end
  end
 
end

function SingleCam:openCamera(number_of_device, mode)
  print("Open mode: ")

  self.mode = ffi.C.XI_MONO8
  if mode ~= nil and  mode == "RGB24" then
    self.mode = ffi.C.XI_RGB24
  end
  print(self.mode)
  self.o = ximea.lib.openCamera(number_of_device, self.mode)
end

function SingleCam:__init() 
    
end
