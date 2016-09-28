local ros = require 'ros'

ros.init('ximea_set_exposure')

local nh = ros.NodeHandle()

local sendCommand = nh:serviceClient('ximea_stereo/send_command', ros.SrvSpec('ximea_msgs/SendCommand'))

local function setExposure(exposure_micro_sec)
  local req = sendCommand:createRequest()
  req.command_name = "setExposure";
  req.value = exposure_micro_sec
  sendCommand:call(req)
end

local exposure = 16666
for i=1,10 do
  print('Capture frame ' .. i)
  setExposure(exposure + 80000)
  exposure = exposure + 1000

end

ros.shutdown()
