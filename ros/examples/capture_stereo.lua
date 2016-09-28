local ros = require 'ros'
local image = require 'image'

ros.init('ximea_capture_demo')

local nh = ros.NodeHandle()
local capture = nh:serviceClient('ximea_stereo/capture', ros.SrvSpec('ximea_msgs/Capture'))
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

setExposure(366666)

for i=1,10 do
  print('Capture frame ' .. i)
  local response = capture:call()
  local img = response.images[1].data
  if img then
    print(img:size())
  end
end

ros.shutdown()

