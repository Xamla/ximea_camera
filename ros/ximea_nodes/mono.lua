#!/usr/bin/env th
local ros = require 'ros'
local ximea = require 'ximea'
local xr = require 'ximea_ros'
require 'XimeaRosCam'


local NODE_NAME = 'ximea_mono'
local MAX_FPS = 250

local nh, spinner
local configuredSerialNumbers
local configuredModes = {}
local srvSendCommand, srvCapture, srvTrigger
local cameras = {}
local opt   -- command line options


local function keys(t)
  local l = {}
  for k,_ in pairs(t) do
    l[#l+1] = k
  end
  return l
end


local function parseCmdLine()
  cmd = torch.CmdLine()
  cmd:addTime()

  cmd:text()
  cmd:text('ROS node for a Ximea cameras (mono mode)')
  cmd:text()

  cmd:option('-mode', 'RGB24', 'The default camera mode (MONO8, MONO16, RGB24, RGB32, RAW8, RAW16).')
  cmd:option('-serials', '', 'Camera serial numbers (separated by comma).')
  cmd:option('-modes', '', 'Camera modes corresponding to serials (separated by comma)')

  opt = cmd:parse(arg or {})
  print('Effective options:')
  print(opt)

  if opt.serials ~= nil and #opt.serials > 1 then
    local serials = string.split(opt.serials, ',')
    configuredSerialNumbers = serials
  end
  if opt.modes ~= nil and #opt.modes > 1 then
    local modes = string.split(opt.modes, ',')
    configuredModes = modes
  end
end


local function openCameras()
  local serials = configuredSerialNumbers
  if serial == nil then
    serials = ximea.getSerialNumbers()     -- all cameras
  end

  for i,serial in ipairs(serials) do
    local mode = configuredModes[i] or opt.mode
    print('hallo')
    print(serial)
    print(mode)


    local cam = xr.XimeaRosCam(nh, NODE_NAME, serial, mode)
    cameras[serial] = cam
  end
end


local function closeCameras()
  for serial,cam in pairs(cameras) do
    cam:close()
    cameras[serial] = nil
  end
end


function selectCameras(serials, nonMeansAll)
  if serials == nil or (#serials == 0 and nonMeansAll) then
    serials = keys(cameras)
  end
  local l = {}  -- resulting list of cameras
  for i,serial in ipairs(serials) do
    local cam = cameras[serial]
    if cam ~= nil then
      l[#l+1] = cam
    else
      ros.WARN("Camera with serial '%s' not found.", serial)
    end
  end
  return l
end


local COMMAND_HANDLER_TABLE = {
  startAcquisition = function(args, value, serials)
    for i,cam in ipairs(selectCameras(serials, true)) do
      cam.camera:startAcquisition()
    end
  end,
  stopAcquisition = function(args, value, serials)
    for i,cam in ipairs(selectCameras(serials, true)) do
      cam.camera:stopAcquisition()
    end
  end,
  setParamInt = function(args, value, serials)
    for i,cam in ipairs(selectCameras(serials, true)) do
      cam.camera:setParamInt(args[1], value)
    end
  end,
  setParamFloat = function(args, value, serials)
    for i,cam in ipairs(selectCameras(serials, true)) do
      cam.camera:setParamFloat(args[1], value)
    end
  end,
  setParamString = function(args, value, serials)
    for i,cam in ipairs(selectCameras(serials, true)) do
      cam.camera:setParamString(args[1], args[2])
    end
  end,
  setExposure = function(args, value, serials)
    for i,cam in ipairs(selectCameras(serials, true)) do
      cam:setExposure(value)
    end
  end,
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
    local ok, err = pcall(function() handler(args, request.value, request.serials) end)
    if ok then
      response.response = 'ok'
      return true
    else
      ros.WARN(err)
      response.response = err
      return false
    end
  end

  response.response = 'unknown command'
  return false
end


local function handleCapture(request, response, header)
  local serials = request.serials
  if serials == nil or #serials == 0 then
    serials = keys(cameras)
  end

  response.serials = {}
  for i,serial in ipairs(serials) do
    local cam = cameras[serial]
    if cam ~= nil then
      local img_msg = cam:capture()
      if img_msg then
        table.insert(response.serials, serial)
        table.insert(response.images, img_msg)
      else
        ros.ERROR("Capturing image from camera with serial '%s' failed.", serial)
      end
    else
      ros.WARN("Camera with serial '%s' not found.", serial)
    end
  end

  return true
end


local function handleTrigger(request, response, header)
  local camera = cameras[request.serial]
  local frames

  response.serial = request.serial
  response.totalFrameCount = 0

  if camera then
    frames = camera:hardwareTriggeredCapture(request.frameCount, request.exposureTimeInMicroSeconds)
  else
    ros.WARN("Camera with serial '%s' not found.", request.serial)
  end

  if frames then
    response.images = frames
    response.totalFrameCount = #frames
  else
    ros.ERROR("Triggered capturing from camera with serial '%s' failed.", request.serial)
  end

  return true
end


local function startServices()
  srvGetConnectedDevices = nh:advertiseService(NODE_NAME .. '/get_connected_devices', ros.SrvSpec('ximea_msgs/GetConnectedDevices'), handleGetConnectedDevices)
  srvSendCommand = nh:advertiseService(NODE_NAME .. '/send_command', ros.SrvSpec('ximea_msgs/SendCommand'), handleSendCommand)
  srvCapture = nh:advertiseService(NODE_NAME .. '/capture', ros.SrvSpec('ximea_msgs/Capture'), handleCapture)
  srvCapture = nh:advertiseService(NODE_NAME .. '/trigger', ros.SrvSpec('ximea_msgs/Trigger'), handleTrigger)
end


local function shutdownServices()
  srvGetConnectedDevices:shutdown()
  srvSendCommand:shutdown()
  srvCapture:shutdown()
end


local function publishFrames()
  for serial,cam in pairs(cameras) do
    cam:publishFrame()
  end
end


local function main()
  parseCmdLine()

  ros.init(NODE_NAME)
  nh = ros.NodeHandle()

  openCameras()

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
    publishFrames()
    collectgarbage()
  end

  print('shutting down...')
  shutdownServices()
  closeCameras()
  spinner:stop()
  ros.shutdown()

  print('bye.')
end

main()
