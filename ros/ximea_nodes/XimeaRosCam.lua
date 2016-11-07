local ros = require 'ros'
local ximea = require 'ximea'
local ximea_ros = require 'ximea_ros'


local XimeaRosCam = torch.class('ximea_ros.XimeaRosCam', ximea_ros)


function XimeaRosCam:__init(nh, ns, serial, mode)
  self.nh = nh
  self.camera = ximea.SingleCam()
  if type(serial) == 'number' then
    self.camera:open(serial, mode)    -- interpret serial arg as device_index
  elseif type(serial) == 'string' then
    self.camera:openCameraWithSerial(serial, mode)
  else
    error('serial argument must be of type sring or number.')
  end

  self.serial = self.camera:getSerial()
  self.camera_topic = nh:advertise(ns .. '/' .. self.camera:getSerial(), ximea_ros.image_spec, 1, false)
end


function XimeaRosCam:hasSubscribers()
  return self.camera_topic ~= nil and self.camera_topic:getNumSubscribers() > 0
end


function XimeaRosCam:capture()
  local img = self.camera:getImage()
  if img == nil then
    ros.WARN('Capturing image faild (cam serial: %s)', self.serial)
    return nil
  end

  return ximea_ros.createImageMessage(img, self.serial, self.camera:getColorMode())
end


function XimeaRosCam:publishFrame()
  if not self:hasSubscribers() then
    return
  end

  local msg = self:capture()
  if msg ~= nil then
    self.camera_topic:publish(msg)
  end
end


function XimeaRosCam:setExposure(value)
  self.camera:setExposure(value)
end


function XimeaRosCam:close()
  if self.camera_topic ~= nil then
    self.camera_topic:shutdown()
    self.camera_topic = nil
  end

  if self.camera ~= nil then
    self.camera:close()
    self.camera = nil
  end
end
