local torch = require 'torch'
local ros = require 'ros'
local cv = require 'cv'
require 'cv.imgproc'


local XimeaClient = torch.class('XimeaClient')


function XimeaClient:__init(nodeHandle, mode, permute_channels, rgb_conversion)

  nodeHandle = nodeHandle or ros.NodeHandle()

  self.mode = mode or "ximea_stereo"
  self.captureClient = nodeHandle:serviceClient(string.format('%s/capture',self.mode), ros.SrvSpec('ximea_msgs/Capture'))
  self.sendCommandClient = nodeHandle:serviceClient(string.format('%s/send_command',self.mode), ros.SrvSpec('ximea_msgs/SendCommand'))
  self.permute_channels = permute_channels or false
  self.rgb_conversion = rgb_conversion or true
  local timeout = ros.Duration(5)
  local ok = self.captureClient:waitForExistence(timeout) and self.sendCommandClient:waitForExistence(timeout)
  if not ok then
    error('ximea_stereo ROS node not running.')
  end
end


function XimeaClient:shutdown()
  self.captureClient:shutdown()
  self.sendCommandClient:shutdown()
end


local function msg2image(self, m, serial)
  if m == nil then
     return nil
  end
  local img
  if m.encoding == "rgb8"  then
    img = torch.ByteTensor(m.width * m.height * 3)
    img:copy(m.data)
    img = img:reshape(m.height, m.width, 3)--actual matrix data, size is (step * rows)
    if self.permute_channels then
      img = img:permute(3,1,2)
    end
  elseif m.encoding == "bgr8" then
    local imgbgr = torch.ByteTensor(m.width * m.height * 3)
    imgbgr:copy(m.data)
    imgbgr = imgbgr:reshape(m.height, m.width, 3) --actual matrix data, size is (step * rows)
    if self.rgb_conversion then
      img = cv.cvtColor{imgbgr, nil, cv.COLOR_RGB2BGR}
    else
      img = imgbgr
    end
    if self.permute_channels then
      img = img:permute(3,1,2)
    end
  elseif m.encoding == "mono8" then
    img = torch.ByteTensor(m.width * m.height * 2)
    img:copy(m.data)
    img = img:reshape(m.height, m.width, 2)--actual matrix data, size is (step * rows)
    if self.permute_channels then
      img = img:transpose(1,3)
    end
  end
  return img
end


local function sendCommand(self, command_name, value)
  local req = self.sendCommandClient:createRequest()
  req.command_name = command_name
  req.value = value or 0
  self.sendCommandClient:call(req)
end


function XimeaClient:setExposure(exposure_micro_sec)
  sendCommand(self, "setExposure", exposure_micro_sec)
end


function XimeaClient:open()
  sendCommand(self, "open")
end


function XimeaClient:close()
  sendCommand(self, "close")
end


function XimeaClient:getImage(index)
  local response = self.capture:call()
  return msg2image(self, response.images[index]), response.serials[index]
end


function XimeaClient:capture()
  return self.captureClient:call()
end


function XimeaClient:getImages()
  local response = nil
  while not response do
    response = self:capture()
  end
  return msg2image(self, response.images[1]), msg2image(self, response.images[2]), response.serials
end
