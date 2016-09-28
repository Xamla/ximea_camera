local ros = require 'ros'

ros.init('ximea_get_connected_devices_demo')

local nh = ros.NodeHandle()

local function getConnectedDevices()
  local client = nh:serviceClient('ximea_stereo/get_connected_devices', ros.SrvSpec('ximea_msgs/GetConnectedDevices'))
  if not client:exists() then
    error('ximea_stereo ROS node not running.')
  end
  local response = client:call()
  client:shutdown()
  return response.serials
end

local serials = getConnectedDevices()
print('Serials of connected Ximea cameras:')
print(serials)

ros.shutdown()
