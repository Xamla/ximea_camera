local ros = require('ros')
local ximea = require 'ximea'

local NODE_NAME = 'ximea_stereo'
local SLEEP_INTERVAL = 0.05


local cv = require 'cv'
require 'cv.highgui'

local nh, spinner
local image_spec = ros.MsgSpec('sensor_msgs/Image')
local cameraInfo_spec = ros.MsgSpec('sensor_msgs/CameraInfo')
local srvGetConnectedDevices, srvSendCommand, srvCapture


local function initializeCameras()
  print('initializeCameras')
end

local function shudtownCameras()
  print('shutdownCameras')
end


local function handleCapture(request, response, header)

  return true
end


local function handleGetConnectedDevices(request, response, header)
  print('request:')
  print(request)
  print('header:')
  print(header)

  print('response:')
  print(response)

  return true
end


local function handleSendCommand(request, response, header)
  print('Received command:' .. request.command_name)
  return true
end


local function startServices()
  srvGetConnectedDevices = nh:advertiseService(NODE_NAME .. '/get_connected_devices', ros.SrvSpec('ximea_msgs/GetConnectedDevices'), handleGetConnectedDevices)
  srvSendCommand = nh:advertiseService(NODE_NAME .. '/send_command', ros.SrvSpec('ximea_msgs/SendCommand'), handleSendCommand)
  srvCapture = nh:advertiseService(NODE_NAME .. '/capture', ros.SrvSpec('ximea_msgs/Capture'), handleCapture)
end


local function shutdownServices()
  srvGetConnectedDevices:shutdown()
  srvSendCommand:shutdown()
  srvCapture:shutdown()
end


local function main()
  ros.init(NODE_NAME)

  nh = ros.NodeHandle()

  spinner = ros.AsyncSpinner()
  spinner:start()

  startServices()

  while ros.ok() do
    ros.spinOnce()
    sys.sleep(SLEEP_INTERVAL)
  end

  print('shutting down...')
  shutdownServices()
  spinner:stop()
  ros.shutdown()
  print('bye.')
end

main()
