#!/usr/bin/env th

--[[
This file is part of the Xamla ROS node for Ximea cameras
Copyright (C) 2018 Xamla and/or its affiliates

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
]]

local ros = require 'ros'
local xamla_sysmon = require 'xamla_sysmon'
local ximea = require 'ximea'
local xr = require 'ximea_ros'
require 'XimeaRosCam'

require 'ros.actionlib.SimpleActionServer'
local actionlib = ros.actionlib

local XI_RET,XI_RET_TEXT = ximea.XI_RET, ximea.XI_RET_TEXT

local NODE_NAME = 'ximea_mono'
local MAX_FPS = 250

local nh, spinner, heartbeat
local configuredSerialNumbers
local configuredModes = {}
local srvSendCommand, srvCapture, srvSoftwareTrigger
local action_server_trigger
local action_server_trigger2
local cameras = {}
local opt   -- command line options
local goal_state
local image_array_spec = ros.MsgSpec('ximea_msgs/ImageArray')


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
    configuredSerialNumbers = serials
  end

  local rate_limit
  if (#serials > 1) then
    --ros.INFO('[mono] Detected several cameras -> Disabling auto bandwidth calculation')
    local XI_OFF = 0
    --local status = ximea.m3api.xiSetParamInt(0, ximea.XI_PRM_AUTO_BANDWIDTH_CALCULATION, XI_OFF)
    --ros.INFO('[mono] set auto band width: %d (%s)', status, ximea.XI_RET_TEXT[status])
    rate_limit = (2400 / #serials) * 0.9
  end

  for i,serial in ipairs(serials) do
    local mode = configuredModes[i] or opt.mode
    local cam = xr.XimeaRosCam(nh, serial, mode, {rateLimit = rate_limit})
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


local function handleSoftwareTrigger(request, response, header)
  local serials = request.serials

  local ok = true
  response.message = ''

  if serials == nil or #serials == 0 then
    ok = false
    response.message = 'No camera serial number provided.'
  end

  for i,serial in ipairs(serials) do
    local cam = cameras[serial]
    if cam ~= nil then
      local res, msg = cam:softwareTrigger()
      ros.INFO("Software triggering cam '%s': %s (%d)", serial, msg, res)
      ok = ok and res == XI_RET.XI_OK
    else
      ok = false
      local error_msg = string.format("Camera with serial '%s' not found.", serial)
      ros.WARN(error_msg)
      response.message = response.message .. error_msg
    end
  end

  response.success = ok
  return true
end


local function sendFeedback(currentFrameCount, errorMessage)
  local fb = goal_state.handle:createFeeback()
  local status = {}
  fb.serials = {}
  for key, serial in ipairs(goal_state.camera_serials) do
    local camera = cameras[serial]
    if camera ~= nil then
      table.insert(status, camera:getState().mode)
    end
    table.insert(fb.serials, serial)
  end

  fb.status = torch.IntTensor(status)
  fb.serials = goal_state.camera_serials
  fb.total_frame_count = goal_state.target_frame_count or 0
  fb.total_frame_count = fb.total_frame_count * #fb.serials
  fb.number_of_taken_frames = currentFrameCount or 0
  fb.error_message = errorMessage or ""
  goal_state.handle:publishFeedback(fb)
end


local function handleNewTriggerGoal(action_server)
  ros.INFO("Received new goal")

  local goal = action_server:acceptNewGoal()
  if goal == nil then
    ros.ERROR('Received goal is nil')
    return
  end

  print(goal)
  print('Goal serials: ', goal.goal.serials)

  -- handle goal parameters only provided by Trigger2 action
  local XI_TRG_EDGE_RISING = 1
  local trigger_source = goal.goal.trigger_source or XI_TRG_EDGE_RISING
  local gpi_index = goal.goal.gpi_index or 1

  if (#goal.goal.serials == 0 or goal.goal.exposure_times_in_microseconds:size(1) == 0) then
    ros.WARN("Received goal without serials or exposure times. Aborting.", goal.goal.serial)
    local r = action_server:createResult()
    action_server:setAborted(r, 'Received goal without serials or exposire times.')
    return
  end

  if (#goal.goal.serials ~= goal.goal.exposure_times_in_microseconds:size(1)) then
    ros.WARN("Received goal with mismatching length of serials and exposure times. Aborting.", goal.goal.serial)
    local r = action_server:createResult()
    action_server:setAborted(r, 'Missmatch of array length of serials and exposure_times_in_microseconds')
    return
  end

  for key, serial in ipairs(goal.goal.serials) do
    if not hasValue(configuredSerialNumbers, serial) then
      ros.WARN("Received goal for unkown serial number %s. Aborting.", goal.goal.serial)
      local r = action_server:createResult()
      action_server:setAborted(r, 'Serial number ' .. serial .. ' not found')
      return
    end
  end

  goal_state = {}
  goal_state.camera_serials = goal.goal.serials
  goal_state.start_time = ros.Time.now()
  goal_state.time_out_in_ms = goal.goal.timeout_in_ms
  goal_state.status = 0
  goal_state.handle = action_server:getCurrentGoalHandle()
  goal_state.last_frame_check = ros.Time.now()
  goal_state.last_feedback_publish = ros.Time.now()
  goal_state.frame_check_interval = goal.goal.exposure_times_in_microseconds:min()
  goal_state.target_frame_count = goal.goal.frame_count
  goal_state.action_server = action_server

  for key, serial in ipairs(goal.goal.serials) do
    local camera = cameras[serial]
    if camera ~= nil then
      camera:startTrigger(goal.goal.frame_count, goal.goal.exposure_times_in_microseconds[key], trigger_source, gpi_index)
      sendFeedback()
    else
      ros.WARN('Camera %s not found', serial)
    end
  end

  goal_state.status = 1
  sendFeedback()
end


local function checkAllCamerasInGoalForNewFrames()
  local frame_count = 0
  local time_since_last_check = ros.Time.now() - goal_state.last_frame_check
  for key, serial in ipairs(goal_state.camera_serials) do
    local camera = cameras[serial]
    local state = camera:getState()
    if camera ~= nil then
      if (state.mode == 1) then -- trigger mode
        if (#state.frames < goal_state.target_frame_count) then
          if (time_since_last_check:toSec() * 1000000 > goal_state.frame_check_interval) then
            goal_state.last_frame_check = ros.Time.now()
            camera:checkForNewFrame()
            state = camera:getState()
          end
        end
      else
        ros.ERROR('Camera %s in wrong mode', serial)
      end

      frame_count = frame_count + #state.frames
    end
  end

  return frame_count
end


local function isTriggerCompleted()
  local all_triggered_cameras_finished = true;
  for key, serial in ipairs(goal_state.camera_serials) do
    local camera = cameras[serial]
    if camera ~= nil then
      local state = camera:getState()
      if (#state.frames <= 0 or #state.frames ~= state.target_frame_count) then
        all_triggered_cameras_finished = false
      end
    end

  end

  return all_triggered_cameras_finished
end


local function stopTriggerOfAllCameras()
  if goal_state == nil then
    return
  end

  for key, serial in ipairs(goal_state.camera_serials) do
    local camera = cameras[serial]
    if camera ~= nil then
      camera:stopTrigger()
    end
  end
end


local function handleTriggerTimeout()
  stopTriggerOfAllCameras()
  ros.WARN('Could not capture enough frames with cameras %s in the given time. Aborting', table.concat(goal_state.camera_serials, ""))
  local r = goal_state.action_server:createResult()
  r.serials = goal_state.camera_serials
  r.total_frame_count = 0
  goal_state.action_server:setAborted(r, 'Capturing timed out.')
  goal_state = nil
end


local function handleTriggerCompleted()
  ros.INFO('Completed trigger goal')
  goal_state.status = 2

  local r = goal_state.action_server:createResult()
  r.serials = goal_state.camera_serials
  local total_frames = 0
  for key, serial in ipairs(goal_state.camera_serials) do
    local camera = cameras[serial]
    if camera ~= nil then
      local state = camera:getState()
      local msg = ros.Message(image_array_spec)
      msg.header.stamp = ros.Time.now()
      msg.header.frame = serial
      msg.serial = serial
      msg.frame_count = #state.frames
      msg.images = state.frames
      total_frames = total_frames + #state.frames
      table.insert(r.images_all_cameras, msg);
    end
  end

  sendFeedback(total_frames)
  r.total_frame_count = total_frames
  goal_state.action_server:setSucceeded(r, 'Captured ' .. total_frames .. 'frames')
  ros.INFO('Goal succeeded')

  for key, serial in ipairs(goal_state.camera_serials) do
    local camera = cameras[serial]
    if camera ~= nil then
      camera:stopTrigger()
    end
  end

  goal_state = nil
end


local function triggerWorker()
  if goal_state ~= nil then
    local time = ros.Time.now() - goal_state.start_time
    if goal_state.time_out_in_ms > 0 and time:toSec() * 1000 > goal_state.time_out_in_ms then
      handleTriggerTimeout()
    else
      local frame_count = checkAllCamerasInGoalForNewFrames()
      local time_since_last_feedback = ros.Time.now() - goal_state.last_feedback_publish
      if (time_since_last_feedback:toSec() > 0.2) then
        sendFeedback(frame_count)
        goal_state.last_feedback_publish = ros.Time.now()
      end
      if isTriggerCompleted() then
        handleTriggerCompleted()
      end
    end
  end
end


local function handleTriggerGoalPreempted(action_server)
  stopTriggerOfAllCameras()
  local r = action_server:createResult()
  r.serials = goal_state.camera_serials
  r.total_frame_count = 0
  action_server:setPreempted(r, 'Goal preempted')
  goal_state = nil
end


local function startServices()
  srvGetConnectedDevices = nh:advertiseService('get_connected_devices', ros.SrvSpec('ximea_msgs/GetConnectedDevices'), handleGetConnectedDevices)
  srvSendCommand = nh:advertiseService('send_command', ros.SrvSpec('ximea_msgs/SendCommand'), handleSendCommand)
  srvCapture = nh:advertiseService('capture', ros.SrvSpec('ximea_msgs/Capture'), handleCapture)
  srvSoftwareTrigger = nh:advertiseService('software_trigger', ros.SrvSpec('ximea_msgs/Trigger'), handleSoftwareTrigger)

  action_server_trigger = actionlib.SimpleActionServer(nh, 'trigger', 'ximea_msgs/Trigger')
  action_server_trigger:registerGoalCallback(handleNewTriggerGoal)
  action_server_trigger:registerPreemptCallback(handleTriggerGoalPreempted)
  action_server_trigger:start()

  action_server_trigger2 = actionlib.SimpleActionServer(nh, 'trigger2', 'ximea_msgs/Trigger2')
  action_server_trigger2:registerGoalCallback(handleNewTriggerGoal)
  action_server_trigger2:registerPreemptCallback(handleTriggerGoalPreempted)
  action_server_trigger2:start()
end


local function shutdownServices()
  srvGetConnectedDevices:shutdown()
  srvSendCommand:shutdown()
  srvCapture:shutdown()
  srvSoftwareTrigger:shutdown()
  action_server_trigger:shutdown()
  action_server_trigger2:shutdown()
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

    if goal_state == nil then   -- do not publish frames while trigger action is active
      last_publish_time = ros.Time.now()
      publishFrames()
    end

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
