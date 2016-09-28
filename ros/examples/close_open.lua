local ros = require 'ros'

ros.init('ximea_close_open_demo')

local nh = ros.NodeHandle()

local sendCommandClient = nh:serviceClient('ximea_stereo/send_command', ros.SrvSpec('ximea_msgs/SendCommand'))

local function sendCommand(command_name, value)
  local req = sendCommandClient:createRequest()
  req.command_name = command_name
  req.value = value or 0
  sendCommandClient:call(req)
end

local function closeCamera()
  sendCommand("close")
end

local function openCamera()
  sendCommand("open")
end

for i=1,10 do
  print('press return to close')
  io.read()
  closeCamera()
  print('press return to open')
  io.read()
  openCamera()
end

ros.shutdown()
