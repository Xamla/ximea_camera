#!/usr/bin/env th
local ros = require 'ros'
local xamla_sysmon = require 'xamla_sysmon'
local ximea = require 'ximea'
local xr = require 'ximea_ros'
require 'XimeaRosCam'

require 'ros.actionlib.SimpleActionServer'
local actionlib = ros.actionlib

local NODE_NAME = 'ximea_mono'
local MAX_FPS = 250

local nh, spinner, heartbeat
local configuredSerialNumbers
local configuredModes = {}
local srvSendCommand, srvCapture, srvTrigger
local action_server
local cameras = {}
local opt   -- command line options
local goal_state


local function keys(t)
  local l = {}
  for k,_ in pairs(t) do
    l[#l+1] = k
  end
  return l
end


local function seperateRosParams(args)
    local rospattern = "^__"
    local result = {}
    local rosparam = {}
    table.insert(rosparam, args[0])
    for i,v in pairs(args) do
        if not string.match(v, rospattern) then
            result[i] = v
        else
            table.insert(rosparam, v)
        end
    end
    return result,rosparam
end


local function parseCmdLine(args)
  cmd = torch.CmdLine()
  cmd:addTime()

  cmd:text()
  cmd:text('ROS node for a Ximea cameras (mono mode)')
  cmd:text()

  cmd:option('-mode', 'RGB24', 'The default camera mode (MONO8, MONO16, RGB24, RGB32, RAW8, RAW16).')
  cmd:option('-serials', '', 'Camera serial numbers (separated by comma).')
  cmd:option('-modes', '', 'Camera modes corresponding to serials (separated by comma)')

  opt = cmd:parse(args or {})

  local overrideInputArguments = function (key, value, ok)
    if ok == true  then
      opt[key] = value
    end
  end

  overrideInputArguments('mode', nh:getParamString('mode'))
  overrideInputArguments('serials', nh:getParamString('serials'))
  overrideInputArguments('modes', nh:getParamString('modes'))

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

local function hasValue (tab, val)
  if (tab == nil) then
    return false
  end
  for index, value in ipairs(tab) do
      if value == val then
          return true
      end
  end

  return false
end

local function openCameras()
  local serials = configuredSerialNumbers
  if serials == nil or #serials == 0 then
    serials = ximea.getSerialNumbers()     -- all cameras
  end

  for i,serial in ipairs(serials) do
    local mode = configuredModes[i] or opt.mode
    local cam = xr.XimeaRosCam(nh, serial, mode, {})
    cameras[serial] = cam
  end
end


local function closeCameras()
  for serial,cam in pairs(cameras) do
    cam:close()
    cameras[serial] = nil
  end
end


local function selectCameras(serials, nonMeansAll)
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
    frames = camera:hardwareTriggeredCaptureFullAuto(request.frameCount, request.exposureTimeInMicroSeconds)
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

local function sendFeedback(targetFrameCount, currentFrameCount, errorMessage)
  local fb = goal_state.handle:createFeeback()
  fb.status = goal_state.status
  fb.serials = goal_state.camera_serials
  fb.total_frame_count = targetFrameCount or 0
  fb.number_of_taken_frames = currentFrameCount or 0
  fb.error_message = errorMessage or ""
  goal_state.handle:publishFeedback(fb)
end

local function handleNewTriggerGoal(as)
  ros.INFO("Received new goal")

  local goal = as:acceptNewGoal()
  print(goal)

  if (goal == nil) then
    ros.ERROR('Received goal is nil')
    return
  end

  for key, serial in pairs(goal.goal.serials) do
    if not hasValue(configuredSerialNumbers, serial) then
      ros.WARN("Received goal for unkown serial number %s. Aborting.", goal.goal.serial)
      local r = as:createResult()
      as:setAborted(r, 'Serial number ' .. goal.goal.serial .. ' not found')
      return
    end
  end

  ros.INFO('Switch trigger modes of cameras to hardware')
  goal_state = {}
  goal_state.camera_serials = goal.goal.serials
  goal_state.start_time = ros.Time.now()
  goal_state.time_out_in_ms = goal.goal.timeout_in_ms
  goal_state.status = 0
  goal_state.handle = action_server:getCurrentGoalHandle()
  sendFeedback()
  for key, serial in pairs(goal.goal.serials) do
    local camera = cameras[serial]
    if camera ~= nil then
      camera:startTrigger(goal.goal.frame_count, goal.goal.exposure_time_in_microseconds)
    else
      ros.WARN('Camera %s not found', goal.goal.serial)
    end
  end
  goal_state.status = 1
end


local function triggerWorker()
  if goal_state ~= nil then
    local time = ros.Time.now() - goal_state.start_time
    if goal_state.timeOutInMs > 0 and time:toSec() * 1000 > goal_state.time_out_in_ms then
      handleTriggerTimeout()
    else
      checkAllCamerasInGoalForNewFrames()
      if isTriggerCompleted() then
        handleTriggerCompleted()
      end
    end
  end
end


local function handleTriggerTimeout()
  local camera_serials = "";
  for key, serial in pairs(goal_state.camera_serials) do
    local camera = cameras[serial]
    camera_serials = camera_serials + serial + ", "
    if camera ~= nil then
      camera:stopTrigger()
    end
  end

  ros.WARN('Could not capture enough frames with cameras %s in the given time. Aborting', camera_serials)
  local r = action_server:createResult()
  r.serials = goal_state.camera_serials
  r.total_frame_count = 0
  r.images = {}
  action_server:setAborted(r, 'Capturing timed out.')
  goal_state = nil
end


local function handleTriggerCompleted()
  ros.INFO('Completed trigger goal')
  goal_state.status = 2
  sendFeedback(state.target_frame_count, #state.frames)

  local r = action_server:createResult()
  r.serials = goal_state.camera_serials
  r.total_frame_count = #state.frames
  r.images = state.frames
  action_server:setSucceeded(r, 'Captured ' .. #state.frames .. 'frames')

  goal_state = nil
  camera:stopTrigger()
end


local function checkAllCamerasInGoalForNewFrames()
  local result_state = {}
  for key, serial in pairs(goal_state.camera_serials) do
    local camera = cameras[serial]
    if camera ~= nil then
      local state = camera:getState()
      if (state.mode == 1) then -- trigger mode
        camera:checkForNewFrame()
      else
        ros.ERROR('Camera %s in wrong mode', serial)
      end
    end
  end
end


local function isTriggerCompleted()
  local all_triggered_cameras_finished = true;
  for key, serial in pairs(goal_state.camera_serials) do
    if (not #state.frames > 0 or not #state.frames == state.target_frame_count) then
      all_triggered_cameras_finished = false
    end
  end

  return all_triggered_cameras_finished
end

local function startServices()
  srvGetConnectedDevices = nh:advertiseService('get_connected_devices', ros.SrvSpec('ximea_msgs/GetConnectedDevices'), handleGetConnectedDevices)
  srvSendCommand = nh:advertiseService('send_command', ros.SrvSpec('ximea_msgs/SendCommand'), handleSendCommand)
  srvCapture = nh:advertiseService('capture', ros.SrvSpec('ximea_msgs/Capture'), handleCapture)
  action_server = actionlib.Simpleaction_server(nh, 'trigger', 'ximea_msgs/Trigger')
  action_server:registerGoalCallback(handleNewTriggerGoal)
  action_server:start()
end


local function shutdownServices()
  srvGetConnectedDevices:shutdown()
  srvSendCommand:shutdown()
  srvCapture:shutdown()
  action_server:shutdown()
end


local function publishFrames()
  for serial,cam in pairs(cameras) do
    cam:publishFrame()
  end
end


local function main()
  local args,rosparam = seperateRosParams(arg)

  ros.init(NODE_NAME, 0, rosparam)
  nh = ros.NodeHandle('~')

  heartbeat = xamla_sysmon.Heartbeat()
  heartbeat:start(nh, 5)

  parseCmdLine(args)

  openCameras()

  spinner = ros.AsyncSpinner()
  spinner:start()

  startServices()

  local last_publish_time = ros.Time(0)
  local min_wait = 1/MAX_FPS
  local wait = ros.Duration()
  ros.INFO('Set status to ' .. heartbeat.STARTING)
  heartbeat:updateStatus(heartbeat.STARTING, "")
  while ros.ok() do
    heartbeat:publish()
    ros.spinOnce()

    local delta = ros.Time.now() - last_publish_time
    if delta:toSec() < min_wait then
      wait:set(min_wait - delta:toSec())
      wait:sleep()
    end

    last_publish_time = ros.Time.now()
    publishFrames()
    triggerWorker()
    heartbeat:updateStatus(heartbeat.GO, "")
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
