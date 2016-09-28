local ros = require 'ros'

ros.init('ximea_capture_demo')


local nh = ros.NodeHandle()

local captureClient = nh:serviceClient('ximea_stereo/capture', ros.SrvSpec('ximea_msgs/Capture'))

local timeout = ros.Duration(5)
local ok = captureClient:waitForExistence(timeout)
if ok then
  local result = captureClient:call({})
else
  error('ximea_stereo ROS node not running.')
end

for i=1,10 do
  --local capture:call()

end

ros.shutdown()
