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

local torch = require 'torch'
local ros = require 'ros'
require 'ros.actionlib.SimpleActionClient'
local cv = require 'cv'
require 'cv.imgproc'

local Task = ros.std.Task
local TaskState = ros.std.TaskState
local actionlib = ros.actionlib
local SimpleClientGoalState = actionlib.SimpleClientGoalState


local XimeaClient = torch.class('XimeaClient')


XimeaClient.ERROR_TYPE = {
  ACTION_CLIENT_TIMEOUT = "ACTION_CLIENT_TIMEOUT",
  ACTION_NOT_DONE = "ACTION_NOT_DONE",
  NO_RESULT_RECEIVED = "NO_RESULT_RECEIVED",
  UNKNOWN_ERROR = "UNKOWN_ERROR"
}


local function initializeActionClient(self, ximea_action_name, nodeHandle)
  local action_spec = 'ximea_msgs/Trigger2'
  self.ximea_action_client = actionlib.SimpleActionClient(action_spec, ximea_action_name, nodeHandle)
end


function XimeaClient:__init(nodeHandle, mode, permute_channels, rgb_conversion, persistent, ximea_action_name)

  nodeHandle = nodeHandle or ros.NodeHandle()

  self.mode = mode or "ximea_stereo"
  local captureServiceName = string.format('%s/capture', self.mode)
  local sendCommandServiceName = string.format('%s/send_command', self.mode)
  local softwareTriggerServiceName = string.format('%s/software_trigger', self.mode)

  self.ximea_action_name = ximea_action_name
  local ok, err = pcall(function() initializeActionClient(self, ximea_action_name, nodeHandle) end)
  if not ok then
    ros.ERROR('Initialization of ximea action client has failed: ' .. err)
  end

  self.captureClient = nodeHandle:serviceClient(captureServiceName, ros.SrvSpec('ximea_msgs/Capture'), persistent)
  self.sendCommandClient = nodeHandle:serviceClient(sendCommandServiceName, ros.SrvSpec('ximea_msgs/SendCommand'), persistent)
  self.softwareTriggerClient = nodeHandle:serviceClient(softwareTriggerServiceName, ros.SrvSpec('ximea_msgs/Trigger'), persistent)
  self.permute_channels = permute_channels or false
  self.rgb_conversion = rgb_conversion or true

  local timeout = ros.Duration(5)
  local ok = self.captureClient:waitForExistence(timeout) and
    self.sendCommandClient:waitForExistence(timeout) and
    self.softwareTriggerClient:waitForExistence(timeout)

  if not ok then
    error('ximea_stereo ROS node not running.')
  end

  -- check if services are valid (e.g. persistent services might require reconnect when service initially was not available)
  if not self.captureClient:isValid() then
    self.captureClient:shutdown()
    self.captureClient = nodeHandle:serviceClient(captureServiceName, ros.SrvSpec('ximea_msgs/Capture'), persistent)
  end
  if not self.sendCommandClient:isValid() then
    self.sendCommandClient:shutdown()
    self.sendCommandClient = nodeHandle:serviceClient(sendCommandClient, ros.SrvSpec('ximea_msgs/SendCommand'), persistent)
  end

  assert(self.captureClient:isValid() and self.sendCommandClient:isValid() and self.softwareTriggerClient:isValid())
end


function XimeaClient:shutdown()
  self.captureClient:shutdown()
  self.sendCommandClient:shutdown()
  self.softwareTriggerClient:shutdown()
end


local function msg2image(self, m)
  if m == nil then
     return nil
  end
  local img
  if m.encoding == "rgb8"  then
    img = torch.ByteTensor(m.width * m.height * 3)
    img:copy(m.data)
    img = img:reshape(m.height, m.width, 3)
    if self.permute_channels then
      img = img:permute(3,1,2)
    end
  elseif m.encoding == "bgr8" then
    local imgbgr = torch.ByteTensor(m.width * m.height * 3)
    imgbgr:copy(m.data)
    imgbgr = imgbgr:reshape(m.height, m.width, 3)
    if self.rgb_conversion then
      img = cv.cvtColor{imgbgr, nil, cv.COLOR_BGR2RGB}
    else
      img = imgbgr
    end
    if self.permute_channels then
      img = img:permute(3,1,2)
    end
  elseif m.encoding == "mono8" then
    img = m.data:view(m.height, m.width)  -- directly return image without copying for max performance
  elseif m.encoding == "mono16" then
    img = torch.ByteTensor(m.width * m.height, 2)
    img:copy(m.data)
    img = img:reshape(m.height, m.width, 2)
    if self.permute_channels then
      img = img:transpose(1,3)
    end
  end
  return img
end


local function sendCommand(self, command_name, value, serials)
  if type(serials) == 'string' then
    serials = { serials }
  end
  local req = self.sendCommandClient:createRequest()
  req.command_name = command_name
  req.serials = serials or {}
  req.value = value or 0
  self.sendCommandClient:call(req)
end


function XimeaClient:setExposure(exposure_micro_sec, serials)
  sendCommand(self, "setExposure", exposure_micro_sec, serials)
end


function XimeaClient:getImage(index)
  local response = self.capture:call()
  return msg2image(self, response.images[index]), response.serials[index]
end


function XimeaClient:capture(serials)
  local req = self.captureClient:createRequest()
  if type(serials) == 'string' then
    serials = { serials }
  end
  req.serials = serials or {}
  return self.captureClient:call(req)
end


function XimeaClient:captureTriggeredAsync(serials, numberOfFrames, exposureTimesInMicroSeconds, timeout, trigger_source, gpi_index)

  local start_handler = function(task)

    local done_callback = function(goal_state, goal_result)
      ros.INFO('Trigger action completed with state: %d (%s)', goal_state, SimpleClientGoalState[goal_state])
      local status
      if goal_result and goal_result.status then
        status = goal_result.status
      end

      if goal_state == SimpleClientGoalState.SUCCEEDED and goal_result ~= nil then
        local images = {}

        ros.INFO("Captured all %d frames.", goal_result.total_frame_count)
        for key, _ in ipairs(serials) do
          local images_per_camera = {}
          for i = 1, goal_result.images_all_cameras[key].frame_count do
            table.insert(images_per_camera, msg2image(self, goal_result.images_all_cameras[key].images[i]))
          end
          table.insert(images, images_per_camera)
        end

        task:setResult(TaskState.Succeeded, images)
      else
        ros.ERROR("Could not capture all frames.")
        if goal_result == nil then
          task:setResult(TaskState.Failed, {code=XimeaClient.ERROR_TYPE.NO_RESULT_RECEIVED, message='Result message from action server was empty.'})
        elseif state ~= SimpleClientGoalState.SUCCEEDED then
          task:setResult(TaskState.Failed, {code=XimeaClient.ERROR_TYPE.ACTION_NOT_DONE, message=string.format('Action has not been succeeded and is in state: %s', SimpleClientGoalState[goal_state])})
        else
          task:setResult(TaskState.Failed, {code=XimeaClient.ERROR_TYPE.UNKNOWN_ERROR, message=string.format('An unkown error has occured. Action client reported: \'%s\'', SimpleClientGoalState[goal_state]) })
        end
      end

    end

    if self.ximea_action_client:waitForServer(ros.Duration(5.0)) then
      local goal = self.ximea_action_client:createGoal()
      goal.serials = serials or {}
      local XI_TRG_EDGE_RISING = 1
      goal.trigger_source = trigger_source or XI_TRG_EDGE_RISING
      goal.gpi_index = gpi_index or 1
      goal.frame_count = numberOfFrames or 0
      goal.exposure_times_in_microseconds = torch.IntTensor(exposureTimesInMicroSeconds) or {}
      goal.timeout_in_ms = timeout or 5000
      local last_feedback
      --print('sending goal', goal)

      local feedback_callback = function (feedback)
        last_feedback = feedback
      end

      self.ximea_action_client:sendGoal(goal, done_callback, nil, feedback_callback)

      -- wait for first feedback message which indicates that camera is waiting for trigger
      local feedback_timeout = ros.Duration(2.0)
      local timeout_time = ros.Time.now() + feedback_timeout

      local wait_timeout = ros.Duration(0.01)
      while last_feedback == nil and ros.ok() and self.ximea_action_client.nh:ok() do
        -- truncate wait-time if necessary
        local time_left = timeout_time - ros.Time.now()

        if time_left < ZERO_DURATION then
          -- timeout
          self.ximea_action_client:cancelGoal()
          task:setResult(TaskState.Failed, {code=XimeaClient.ERROR_TYPE.ACTION_CLIENT_TIMEOUT, message = 'Waiting for initial feedback from capture action server timed out.'})
          break
        end

        if not self.ximea_action_client:isServerConnected() then
          -- server connection lost
          ros.WARN("Lost connection to capture action server.")
          self.ximea_action_client:cancelGoal()
          task:setResult(TaskState.Failed, {code=XimeaClient.ERROR_TYPE.UNKNOWN_ERROR, message = 'Lost contact to ximea action server.'})
          break
        end

        if time_left < wait_timeout then
          wait_timeout = time_left
        end

        self.ximea_action_client.callback_queue:callAvailable(wait_timeout)
      end

      if last_feedback ~= nil then
        ros.INFO('Camera ready.')
      end

    else
      ros.ERROR("Could not contact ximea action server.")
      task:setResult(TaskState.Failed, {message = string.format('Could not contact ximea action server: \'%s\'', self.ximea_action_name)})
    end
  end

  local cancel_handler = function(task)
    self.ximea_action_client:cancelGoal()
  end

  return Task.create(start_handler, cancel_handler, nil, true)
end


--- Manually trigger the camera.
-- This method can be used to perform software triggering manually while an captureTriggeredAsync call is in progress.
-- The method will immediately return without waiting the exposure time or the image result.
function XimeaClient:softwareTrigger(serials)
  local req = self.softwareTriggerClient:createRequest()
  if type(serials) == 'string' then
    serials = { serials }
  end
  req.serials = serials or {}
  assert(#req.serials > 0, 'No camera serial number provided.')
  return self.softwareTriggerClient:call(req)
end


function XimeaClient:trigger(serials, numberOfFrames, exposureTimesInMicroSeconds, timeout)
  local images = {}
  if self.ximea_action_client:waitForServer(ros.Duration(5.0)) then
    local goal = self.ximea_action_client:createGoal()
    goal.serials = serials or {}
    goal.gpi_index = 1
    local XI_TRG_EDGE_RISING = 1
    goal.trigger_source = XI_TRG_EDGE_RISING
    goal.frame_count = numberOfFrames or 1
    goal.exposure_times_in_microseconds = torch.IntTensor(exposureTimesInMicroSeconds) or {}
    goal.timeout_in_ms = timeout or 5000
    --print('goal', goal)
    --print('before wait')
    local state = self.ximea_action_client:sendGoalAndWait(goal, ros.Duration(5.0))
    local result = self.ximea_action_client:getResult()
    --print('after wait')
    if state == 7 and result ~= nil then
      ros.INFO("Captured all %d frames.", result.total_frame_count)
      for key, _ in ipairs(serials) do
        local images_per_camera = {}
        for i = 1, result.images_all_cameras[key].frame_count do
            table.insert(images_per_camera, msg2image(self, result.images_all_cameras[key].images[i]))
        end
        table.insert(images, images_per_camera)
      end
    else
      ros.ERROR("Could not capture all frames.")
      if result == nil then
        error({code=XimeaClient.ERROR_TYPE.NO_RESULT_RECEIVED, message='Result message from action server was empty.'})
      elseif state ~= 7 then
        error({code=XimeaClient.ERROR_TYPE.ACTION_NOT_DONE, message=string.format('Action has not been succeeded and is in state: %s', SimpleClientGoalState[state])})
      else
        error({code=XimeaClient.ERROR_TYPE.UNKNOWN_ERROR, message='An unkown error has occured.'})
      end
    end
  else
    error({code=XimeaClient.ERROR_TYPE.ACTION_CLIENT_TIMEOUT, message='Could not contact ximea action server.'})
  end
  return images
end


function XimeaClient:getImages(serials)
  local response = nil
  while not response do
    response = self:capture(serials)
  end
  return msg2image(self, response.images[1]), msg2image(self, response.images[2]), msg2image(self, response.images[3]), response.serials
end
