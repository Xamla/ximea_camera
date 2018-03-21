local torch = require 'torch'
local ros = require 'ros'
local cv = require 'cv'
require 'cv.imgproc'

require "ros.actionlib.SimpleActionClient"
local actionlib = ros.actionlib

local XimeaClient = torch.class('XimeaClient')
XimeaClient.ERROR_TYPE = {
  ACTION_CLIENT_TIMEOUT = 3,
  CAMERA_NOT_READY = 0,
  WAIT_FOR_FRAMES_BEING_TRIGGERED = 1,
  CAMERA_IN_ERROR_STATE = -1
}


local function initializeActionClient(self, ximea_action_name, nodeHandle)
  local action_spec = 'ximea_msgs/Trigger'
  self.ximea_action_client = actionlib.SimpleActionClient(action_spec, ximea_action_name, nodeHandle)
end


function XimeaClient:__init(nodeHandle, mode, permute_channels, rgb_conversion, persistent, ximea_action_name)

  nodeHandle = nodeHandle or ros.NodeHandle()

  self.mode = mode or "ximea_stereo"
  local captureServiceName = string.format('%s/capture', self.mode)
  local sendCommandServiceName = string.format('%s/send_command', self.mode)

  local ok, err = pcall(function() initializeActionClient(self, ximea_action_name, nodeHandle) end)
  if not ok then
    ros.ERROR('Initialization of ximea action client has failed: ' .. err)
  end

  self.captureClient = nodeHandle:serviceClient(captureServiceName, ros.SrvSpec('ximea_msgs/Capture'), persistent)
  self.sendCommandClient = nodeHandle:serviceClient(sendCommandServiceName, ros.SrvSpec('ximea_msgs/SendCommand'), persistent)
  self.permute_channels = permute_channels or false
  self.rgb_conversion = rgb_conversion or true

  local timeout = ros.Duration(5)
  local ok = self.captureClient:waitForExistence(timeout) and self.sendCommandClient:waitForExistence(timeout)
  if not ok then
    error('ximea_stereo ROS node not running.')
  end
  -- check if services are valid (e.g. persistent services might require reconnect when service initially was not available)
  if not self.captureClient:isValid() then
    self.captureClient:shutdown()
    self.captureClient = nodeHandle:serviceClient(captureServiceName, ros.SrvSpec('ximea_msgs/Capture'), persistent)
  end
  if not self.sendCommandClient:isValid() then
    self.sendCommandClient:shutdown()
    self.sendCommandClient = nodeHandle:serviceClient(sendCommandClient, ros.SrvSpec('ximea_msgs/SendCommand'), persistent)
  end

  assert(self.captureClient:isValid() and self.sendCommandClient:isValid())
end


function XimeaClient:shutdown()
  self.captureClient:shutdown()
  self.sendCommandClient:shutdown()
end


local function msg2image(self, m)
  if m == nil then
     return nil
  end
  local img
  if m.encoding == "rgb8"  then
    img = torch.ByteTensor(m.width * m.height * 3)
    img:copy(m.data)
    img = img:reshape(m.height, m.width, 3)
    if self.permute_channels then
      img = img:permute(3,1,2)
    end
  elseif m.encoding == "bgr8" then
    local imgbgr = torch.ByteTensor(m.width * m.height * 3)
    imgbgr:copy(m.data)
    imgbgr = imgbgr:reshape(m.height, m.width, 3)
    if self.rgb_conversion then
      img = cv.cvtColor{imgbgr, nil, cv.COLOR_BGR2RGB}
    else
      img = imgbgr
    end
    if self.permute_channels then
      img = img:permute(3,1,2)
    end
  elseif m.encoding == "mono8" then
    img = m.data:view(m.height, m.width)  -- directly return image without copying for max performance
  elseif m.encoding == "mono16" then
    img = torch.ByteTensor(m.width * m.height, 2)
    img:copy(m.data)
    img = img:reshape(m.height, m.width, 2)
    if self.permute_channels then
      img = img:transpose(1,3)
    end
  end
  return img
end


local function sendCommand(self, command_name, value, serials)
  if type(serials) == 'string' then
    serials = { serials }
  end
  local req = self.sendCommandClient:createRequest()
  req.command_name = command_name
  req.serials = serials or {}
  req.value = value or 0
  self.sendCommandClient:call(req)
end


function XimeaClient:setExposure(exposure_micro_sec, serials)
  sendCommand(self, "setExposure", exposure_micro_sec, serials)
end


function XimeaClient:getImage(index)
  local response = self.capture:call()
  return msg2image(self, response.images[index]), response.serials[index]
end


function XimeaClient:capture(serials)
  local req = self.captureClient:createRequest()
  if type(serials) == 'string' then
    serials = { serials }
  end
  req.serials = serials or {}
  return self.captureClient:call(req)
end


function XimeaClient:trigger(serial, numberOfFrames, exposureTimeInMicroSeconds, timeout)
  local images = {}
  if self.ximea_action_client:waitForServer(ros.Duration(5.0)) then
    local goal = self.ximea_action_client:createGoal()
    goal.serial = serial or ""
    goal.frame_count = numberOfFrames or 0
    goal.exposure_time_in_microseconds = exposureTimeInMicroSeconds or 20000
    goal.timeout_in_ms = timeout or 5000
    self.ximea_action_client:sendGoal(goal)
    local result = self.ximea_action_client:getResult()
    local state = self.ximea_action_client:getState()
    if state.status == 2 then
      ros.INFO("Captured all frames")
      for i = 1, result.total_frame_count do	
        table.insert(images, msg2image(self, result.images[i]))	
      end
    elseif state.status == 0 then
      error({code=XimeaClient.ERROR_TYPE.CAMERA_NOT_READY, message='Camera not ready'})
    elseif state.status == 1 then
      local status = {code=1, message='Camera ready, wait for frames being triggered'}
      return status
    elseif state.status == -1 then
      error({code=XimeaClient.ERROR_TYPE.CAMERA_IN_ERROR_STATE, message=string.format('Error state with error message: %s', status.error_message)})
    end
  else
    error({code=XimeaClient.ERROR_TYPE.ACTION_CLIENT_TIMEOUT, message='Could not contact ximea action server'})
  end
  return images
end


function XimeaClient:getImages(serials)
  local response = nil
  while not response do
    response = self:capture(serials)
  end
  return msg2image(self, response.images[1]), msg2image(self, response.images[2]), msg2image(self, response.images[3]), response.serials
end
