local ffi = require 'ffi'
local ros = require 'ros'
local ximea = require 'ximea'

local NODE_NAME = 'ximea_mono'
local MAX_FPS = 80


local nh, spinner
local image_spec = ros.MsgSpec('sensor_msgs/Image')
local cameraInfo_spec = ros.MsgSpec('sensor_msgs/CameraInfo')
local srvSendCommand, srvCapture
local camera = ximea.SingleCam()
local camera_topic

local opt   -- command line options


local function parseCmdLine()
  cmd = torch.CmdLine()
  cmd:addTime()

  cmd:text()
  cmd:text('ROS node for a single Ximea camera')
  cmd:text()

  cmd:option('-mode', 'RGB24', 'The default camera mode.')
  cmd:option('-serial', '', 'Camera serial number to use.')

  opt = cmd:parse(arg or {})
  print('Effective options:')
  print(opt)
end


local function createImageMessage(img, serial)
  local msg = ros.Message(image_spec)
  msg.header.stamp = ros.Time.now()
  msg.header.frame = serial

  msg.height = img:size(1)    -- image height
  msg.width = img:size(2)     -- image width

  local color_mode = camera:getColorMode()
  if color_mode == ximea.XI_IMG_FORMAT.RGB24 then
    msg.encoding = "bgr8"
  elseif color_mode == ximea.XI_IMG_FORMAT.MONO8 or color_mode == ximea.XI_IMG_FORMAT.RAW8 then
    msg.encoding = "mono8"
  elseif color_mode == ximea.XI_IMG_FORMAT.MONO16 or color_mode == ximea.XI_IMG_FORMAT.RAW16 then
    msg.encoding = "mono16"
  else
    error('Unsupported color format.')
  end
  msg.step = img:stride(1)
  msg.data = img:reshape(img:size(1) * img:size(2) * img:size(3))   -- actual matrix data, size is (step * rows)
  msg.is_bigendian = ffi.abi('be')
  return msg
end


local function shutdownPublisher()
  if camera_topic ~= nil then
    camera_topic:shutdown()
    camera_topic = nil
  end
end


local function createPublisher()
  shutdownPublisher()
  camera_topic = nh:advertise(NODE_NAME .. '/image', image_spec, 1, false)
end


local function openCamera()
  if opt.serial ~= nil and #opt.serial > 0 then
    camera:openCameraWithSerial(opt.serial, opt.mode)
  else
    camera:openCamera(0, opt.mode)
  end
  createPublisher()
end


local function closeCamera()
  shutdownPublisher()
  camera:close()
end


local COMMAND_HANDLER_TABLE = {
  setExposure = function(args, value) camera:setExposure(value) end,
  close = function() closeCamera() end,
  open = function() openCamera() end
}


local function handleGetConnectedDevices(request, response, header)
  response.serials = ximea.getSerialNumbers()
  return true
end


local function handleSendCommand(request, response, header)
  local args = string.split(request.command_name, ',')
  local cmd = table.remove(args, 1)

  local handler = COMMAND_HANDLER_TABLE[cmd]
  if handler ~= nil then
    handler(args, request.value)
    return true
  end

  return false
end


local function handleCapture(request, response, header)
  if camera:isOpen() then
    local img = camera:getImage()
    if img then
      response.serials = { camera:getSerial() }
      response.images[1] = createImageMessage(img, response.serials[1])
      return true
    else
      print("Capturing image failed.")
    end
  else
    print("Camera is closed.")
  end

  return false
end


local function hasSubscribers()
  return camera_topic ~= nil and camera_topic:getNumSubscribers() > 0
end


local function publishFrame()
  if not hasSubscribers() then
    return
  end

  local img = camera:getImage()
  if img == nil then
    print('WARNING: Capturing imagesfailed.')
    return
  end

  local serial = camera:getSerial()
  local msg = createImageMessage(img, serial)
  camera_topic:publish(msg)
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
  parseCmdLine()

  ros.init(NODE_NAME)

  nh = ros.NodeHandle()

  openCamera()

  spinner = ros.AsyncSpinner()
  spinner:start()

  startServices()

  local last_publish_time = ros.Time(0)
  local min_wait = 1/MAX_FPS
  local wait = ros.Duration()
  while ros.ok() do
    ros.spinOnce()

    local delta = ros.Time.now() - last_publish_time
    if delta:toSec() < min_wait then
      wait:set(min_wait - delta:toSec())
      wait:sleep()
    end

    last_publish_time = ros.Time.now()
    publishFrame()
    collectgarbage()
  end

  print('shutting down...')
  shutdownServices()
  closeCamera()
  spinner:stop()
  ros.shutdown()

  print('bye.')
end

main()
