local ros = require 'ros'

ros.init('ximea_capture_demo')


local nh = ros.NodeHandle()

local capture = nh:serviceClient('ximea_stereo/capture', ros.SrvSpec('ximea_msgs/Capture'))

local timeout = ros.Duration(5)
local ok = capture:waitForExistence(timeout)
if not ok then
  error('ximea_stereo ROS node not running.')
end

for i=1,10 do
  print('Capture frame ' .. i)
  local response = capture:call({})
  print(response)
end

ros.shutdown()
