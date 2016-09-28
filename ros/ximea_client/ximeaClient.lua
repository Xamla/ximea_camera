local torch = require 'torch'
local ros = require 'ros'
local  XC = torch.class('ximeaClient')

function XC:__init(nodeHandle,mode)
  self.mode = mode or "ximea_stereo"
  self.capture = nodeHandle:serviceClient(string.format('%s/capture',self.mode), ros.SrvSpec('ximea_msgs/Capture'))
  self.sendCommand = nodeHandle:serviceClient(string.format('%s/send_command',self.mode), ros.SrvSpec('ximea_msgs/SendCommand'))
  local timeout = ros.Duration(5)
  local ok = self.capture:waitForExistence(timeout)
  if not ok then
    error('ximea_stereo ROS node not running.')
  end

end

local function msg2image(m, serial)
  local img
  if m.encoding == "rgb8"  then
    img = torch.ByteTensor(3* m.width *m.height)
    img:copy(m.data)
    img = img:reshape(m.height,m.width,3)--actual matrix data, size is (step * rows)
    img = img:transpose(1,3)
    img = img:transpose(2,3)
  elseif m.encoding == "bgr8" then
    local imgbgr = torch.ByteTensor(3* m.width *m.height)
    imgbgr:copy(m.data)
    imgbgr = imgbgr:reshape(m.height,m.width,3)--actual matrix data, size is (step * rows)
    img = torch.ByteTensor(m.height ,m.width, 3)
    img[{{},{},{1}}]:copy(imgbgr[{{},{},{3}}])
    img[{{},{},{2}}]:copy(imgbgr[{{},{},{2}}])
    img[{{},{},{3}}]:copy(imgbgr[{{},{},{1}}])

    img = img:transpose(1,3)
    img = img:transpose(2,3)
  elseif m.encoding == "mono8" then
    img = torch.ByteTensor(m.width *m.height*2)
    img:copy(m.data)
    img = img:reshape(m.height, m.width,2)--actual matrix data, size is (step * rows)
    img = img:transpose(1,3)

  end

  return img
end


local function sendCommand(command_name, value)
  local req = sendCommandClient:createRequest()
  req.command_name = command_name
  req.value = value or 0
  sendCommandClient:call(req)
end


function XC:setExposure(exposure_micro_sec)
  sendCommand("setExposure", exposure_micro_sec)
end


function XC:open()
  sendCommand("open")
end


function XC:close()
  sendCommand("close")
end


function XC:getImage(index)
  local response = self.capture:call({})
  return msg2image(response.images[index])
end


function XC:getImages()
  return self:getImage(1),self:getImage(2)
end
