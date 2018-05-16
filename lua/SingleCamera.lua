local ffi = require 'ffi'
local ximea = require 'ximea.env'


local XI_RET,XI_RET_TEXT = ximea.XI_RET, ximea.XI_RET_TEXT


local SingleCam = torch.class('ximea.SingleCam', ximea)


local DEFAULT_MODE = 'MONO8'


function SingleCam:__init()
end

function SingleCam:getImage(hardwareTriggered, timeout)
  timeout = timeout or 1000
  local img = torch.ByteTensor()
  hardwareTriggered = hardwareTriggered or false
  if not ximea.lib.getSingleImage(self.o, self.color_mode, img:cdata(), hardwareTriggered, timeout) then
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


function SingleCam:openCameraWithSerial(serial, mode, start_acquisition, rate_limit)
  local serials = ximea.getSerialNumbers()
  for i=1,#serials do
    if serials[i] == serial then
      self:openCamera(i-1, mode, start_acquisition, rate_limit)
      return
    end
  end
end


function SingleCam:openCamera(device_index, mode, start_acquisition, rate_limit)
  self.color_mode = ximea.getXiModeByName(mode or DEFAULT_MODE)
  self.device_index = device_index or 0

  if start_acquisition == nil then
    start_acquisition = true
  end

  print('Open mode: ')
  print(self.color_mode)

  self.o = ximea.lib.openCamera(self.device_index, self.color_mode, start_acquisition)

  local c_str = ffi.new("char[32]")
  ximea.lib.getSerialNumber(self.device_index, c_str)
  self.serial = ffi.string(c_str)

  if rate_limit ~= nil then
    print('[SingleCam] Set rate limit to ', rate_limit, self.serial, ximea.PARAM.XI_PRM_LIMIT_BANDWIDTH, rate_limit)
    local status, status_text = self:setParamInt(ximea.PARAM.XI_PRM_LIMIT_BANDWIDTH, rate_limit)
    print('[SingleCam] Result of rate limit xiApi call: ', status, status_text)
  end
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


function SingleCam:startAcquisition()
  local status = ximea.m3api.xiStartAcquisition(self.o)
  return status, XI_RET_TEXT[status]
end


function SingleCam:stopAcquisition()
  local status = ximea.m3api.xiStopAcquisition(self.o)
  return status, XI_RET_TEXT[status]
end


function SingleCam:setParamInt(paramName, value)
  local status = ximea.m3api.xiSetParamInt(self.o, paramName, value)
  return status, XI_RET_TEXT[status]
end


function SingleCam:setParamFloat(paramName, value)
  local status = ximea.m3api.xiSetParamFloat(self.o, paramName, value)
  return status, XI_RET_TEXT[status]
end


function SingleCam:setParamString(paramName, value)
  assert(type(value) == 'string', 'Argument "value": String expected')
  local status = ximea.m3api.xiSetParamString(self.o, paramName, value, #value)
  return status, XI_RET_TEXT[status]
end


function SingleCam:getParamInt(paramName)
  local buffer = ffi.new('int[1]')
  local status = ximea.m3api.xiGetParamInt(self.o, paramName, buffer)
  local v
  if status == XI_RET.XI_OK then
    v = buffer[0]
  end
  return v, status, XI_RET_TEXT[status]
end


function SingleCam:getParamFloat(paramName)
  local buffer = ffi.new('float[1]')
  local status = ximea.m3api.xiSetParamFloat(self.o, paramName, buffer)
  local v
  if status == XI_RET.XI_OK then
    v = buffer[0]
  end
  return v, status, XI_RET_TEXT[status]
end


function SingleCam:getParamString(paramName, length)
  length = length or 256
  local buffer = ffi.new(string.format('char[%d]', length))
  local status = ximea.m3api.xiGetParamString(self.o, paramName, buffer)
  local v
  if status == XI_RET.XI_OK then
    v = ffi.string(buffer)
  end
  return v, status, XI_RET_TEXT[status]
end
