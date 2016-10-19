local ros = require 'ros'

ros.init('ximea_set_exposure')

local nh = ros.NodeHandle()

local sendCommandClient = nh:serviceClient('ximea_stereo/send_command', ros.SrvSpec('ximea_msgs/SendCommand'))

local function sendCommand(command_name, value)
  local req = sendCommandClient:createRequest()
  req.command_name = command_name
  req.value = value or 0
  sendCommandClient:call(req)
end

local function setExposure(exposure_micro_sec)
  sendCommand("setExposure", exposure_micro_sec)
end

local exposure = 16666
for i=1,10 do
  print('Capture frame ' .. i)
  setExposure(exposure * i)
--  exposure = exposure + 1000
end

ros.shutdown()
