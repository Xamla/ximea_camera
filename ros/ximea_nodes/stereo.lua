local ros = require('ros')
local ximea = require 'ximea'

local NODE_NAME = 'ximea_stereo'
local SLEEP_INTERVAL = 0.05

local cv = require 'cv'
require 'cv.highgui'

local nh, spinner
local image_spec = ros.MsgSpec('sensor_msgs/Image')
local cameraInfo_spec = ros.MsgSpec('sensor_msgs/CameraInfo')
local stereoPair_spec = ros.MsgSpec('ximea_msgs/StereoPair')
local srvGetConnectedDevices, srvSendCommand, srvCapture
local stereoCam = ximea.StereoCam()
local cameraOpen = false
local stereo_topic
local camera_topics = {}

local opt   -- command line options


local function parseCmdLine()
  cmd = torch.CmdLine()
  cmd:addTime()

  cmd:text()
  cmd:text('ROS node for a stereo rig with Ximea cameras')
  cmd:text()

  cmd:option('-mode', 'RGB24', 'default camera mode')
  cmd:option('-serials', '', 'two camera serials separated by comma')

  opt = cmd:parse({})
  print('Effective options:')
  print(opt)
end


local function shutdownPublishers()
  if stereo_topic ~= nil then
    stereo_topic:shutdown()
    stereo_topic = nil
  end
  for k,v in pairs(camera_topics) do
    v:shutdown()
  end
  camera_topics = {}
end


local function createPublishers()
  shutdownPublishers()
  stereo_topic = nh:advertise(NODE_NAME .. '/stereo_pair', stereoPair_spec, 1, false)
  for i,serial in ipairs(stereoCam:getSerials()) do
    camera_topics[serial] = nh:advertise(NODE_NAME .. '/' .. serial, image_spec, 1, false)
  end
end


local function openCamera()
  stereoCam:openCamera(opt.mode)
  createPublishers()
end


local function closeCamera()
  stereoCam:close()
  shutdownPublishers()
end


local function createImageMessage(img, serial)
  local msg = ros.Message(image_spec)
  msg.header.stamp = ros.Time.now()
  msg.header.frame = serial

  msg.height = img:size(1)    -- image height
  msg.width = img:size(2)     -- image width
  msg.encoding = "rgb8"       -- encoding of pixels -- channel meaning, ordering, size
  msg.is_bigendian = false
  msg.step = img:stride(1)
  msg.data = img:reshape(img:size(1) * img:size(2) * img:size(3))   -- actual matrix data, size is (step * rows)
  return msg
end


local function createStereoPairMessage(img1, img2, serials, output)
  output = output or ros.Message(stereoPair_spec)
  output.serials = serials
  output.images[1] = createImageMessage(img1, serials[1])
  output.images[2] = createImageMessage(img2, serials[2])
  return output
end


local function handleCapture(request, response, header)
  if stereoCam:isOpen() then
    local img1, img2 = stereoCam:getImage()
    response.serials = stereoCam:getSerials()
    response.images[1] = createImageMessage(img1, response.serials[1])
    response.images[2] = createImageMessage(img2, response.serials[2])
    return true
  end

  return false
end


local function handleGetConnectedDevices(request, response, header)
  response.serials = ximea.getSerialNumbers()
  return true
end


local COMMAND_HANDLER_TABLE = {
  setExposure = function(args, value) stereoCam:setExposure(value) end,
  close = function() stereoCam:close() end,
  open = function() stereoCam:open() end
}


local function handleSendCommand(request, response, header)
  local args = string.split(request.command_name)
  local cmd = table.remove(args, 1)

  local handler = COMMAND_HANDLER_TABLE[cmd]
  if handler ~= nil then
    handler(args, request.value)
    return true
  end

  return false
end


local function hasSubscribers()
  if stereo_topic ~= nil and stereo_topic:getNumSubscribers() > 0 then
    return true
  end

  for k,v in pairs(camera_topics) do
    if v:getNumSubscribers() > 0 then
      return true
    end
  end

  return false
end


local function publishFrames()
  if not stereoCam:isOpen() or not hasSubscribers() then
    return
  end

  local img1, img2 = stereoCam:getImage()
  local serials = stereoCam:getSerials()
  local stereo_msg = createStereoPairMessage(img1, img2, serials)

  if stereo_topic ~= nil and stereo_topic:getNumSubscribers() > 0 then
    stereo_topic:publish(stereo_msg)
  end
  for k,v in pairs(camera_topics) do
    if v:getNumSubscribers() > 0 then
      for i=1,#serials do
        if serials[i] == k then
          v:publish(stereo_msg.images[i])
        end
      end
    end
  end
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

  while ros.ok() do
    ros.spinOnce()

    publishFrames()
    collectgarbage()

    sys.sleep(SLEEP_INTERVAL)
  end

  print('shutting down...')
  shutdownServices()
  closeCamera()
  spinner:stop()
  ros.shutdown()

  print('bye.')
end

main()
