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
local ximea = require 'ximea'
local ximea_ros = require 'ximea_ros'

local cv = require 'cv'
require 'cv.imgcodecs'
require 'cv.imgproc'
require 'cv.calib3d'

local XI_RET,XI_RET_TEXT = ximea.XI_RET, ximea.XI_RET_TEXT

local XimeaRosCam = torch.class('ximea_ros.XimeaRosCam', ximea_ros)

local CaptureMode = {
  continuous = 0,
  triggered = 1
}

local DEFAULT_FLAGS = {
  enableFPNCorrection = true,
  autoWhiteBalance = true
}


local function XI_CHECK(status, msg)
  if status ~= XI_RET.XI_OK then
    ros.ERROR(msg)
    error(msg)
  end
end


local function escapeRosName(s)
  -- add undersore if s starts with number
  if s:match('^[0-9]') then
    s = '_' .. s
  end
  -- replace all non alpha-numeric chars with safe char
  return string.gsub(s, "([^A-Za-z0-9_])", '_')
end


function XimeaRosCam:__init(nh, serial, mode, flags)
  self.nh = nh
  self.camera = ximea.SingleCam()

  local rate_limit = flags.rateLimit

  if type(serial) == 'number' then
    self.camera:open(serial, mode, false, rate_limit)    -- interpret serial arg as device_index
  elseif type(serial) == 'string' then
    self.camera:openCameraWithSerial(serial, mode, false, rate_limit)
  else
    error('serial argument must be of type sring or number.')
  end

  self.serial = self.camera:getSerial()
  self.camera_topic = nh:advertise('SN_' .. escapeRosName(self.serial), ximea_ros.image_spec, 1, false)
  self.flags = flags or DEFAULT_FLAGS

  if self.flags.enableFPNCorrection then
    print('Enabling PFN correction')
    XI_CHECK(self.camera:setParamInt(ximea.PARAM.XI_PRM_COLUMN_FPN_CORRECTION, 1))
  end

  if self.flags.autoWhiteBalance and (mode == 'RGB24' or mode == 'RGB32') then
    print('Enabling auto white-balance')
    XI_CHECK(self.camera:setParamInt(ximea.PARAM.XI_PRM_AUTO_WB, 1))
  end

  print('Start camera acquisition')
  XI_CHECK(self.camera:startAcquisition())

  self.state = {
    mode = CaptureMode.continuous,
    target_frame_count = 0,
    frames = {}
  }
end


function XimeaRosCam:hasSubscribers()
  return self.camera_topic ~= nil and self.camera_topic:getNumSubscribers() > 0
end


function XimeaRosCam:capture(hardwareTriggered, timeout)
  hardwareTriggered = hardwareTriggered or false
  timeout = timeout or 1000

  -- AKo: retry was added, because in long-running tests failures had been observed (not retrying when hw-triggered)
  for i=1,5 do
    local img = self.camera:getImage(hardwareTriggered, timeout)
    if img ~= nil then
      local rosMessage = ximea_ros.createImageMessage(img, self.serial, self.camera:getColorMode())
      return rosMessage
    end

    local b = -1
    if hardwareTriggered == true then
      b = 1
    elseif hardwareTriggered == false then
      b = 0
    end
    ros.WARN('Capturing image failed (cam serial: %s, hw trigger %d, timeout %d, try: %d)', self.serial, b, timeout or -1, i)

    -- do not retry if hardware triggering is used
    if hardwareTriggered then break end
  end

  return nil
end


function XimeaRosCam:publishFrame()
  if not self:hasSubscribers() then
    return
  end

  if self.state.mode == CaptureMode.triggered then
    return
  end

  local msg = self:capture()
  if msg ~= nil then
    self.camera_topic:publish(msg)
  end
end


function XimeaRosCam:setExposure(value)
  self.camera:setExposure(value)
end


function XimeaRosCam:getState()
  return self.state
end

function XimeaRosCam:close()
  if self.camera_topic ~= nil then
    self.camera_topic:shutdown()
    self.camera_topic = nil
  end

  if self.camera ~= nil then
    self.camera:close()
    self.camera = nil
  end
end


function XimeaRosCam:startTrigger(numberOfFrames, exposureTimeInMicroSeconds)
  -- Set camera into hardware triggered mode
  print(string.format("[XimeaRosCam:hwTrigger] start hw triggering for camera %s", self.serial))
  print("[XimeaRosCam:hwTrigger] re-configure camera")
  local XI_TRG_EDGE_RISING = 1
  local XI_GPI_TRIGGER = 1
  local camera = self.camera
  camera:setParamInt("gpi_selector", 1)
  camera:setParamInt("gpi_mode", XI_GPI_TRIGGER)
  camera:setParamInt("trigger_source", XI_TRG_EDGE_RISING)
  camera:setParamInt("buffers_queue_size", numberOfFrames + 1)
  camera:setParamInt("recent_frame", 0)
  print("[XimeaRosCam:hwTrigger] set exposure to " .. exposureTimeInMicroSeconds)
  self:setExposure(exposureTimeInMicroSeconds)
  self.state = {
    mode = CaptureMode.triggered,
    target_frame_count = numberOfFrames,
    frames = {}
  }
end


function XimeaRosCam:stopTrigger()
  -- Stop triggering and restore buffer settings
  local XI_TRG_SOFTWARE = 3
  print("[XimeaRosCam:hwTrigger] hold triggering")
  local camera = self.camera
  camera:setParamInt("trigger_source", XI_TRG_SOFTWARE)
  camera:setParamInt("buffers_queue_size", 4)
  camera:setParamInt("recent_frame", 1)
  self.state = {
    mode = CaptureMode.continuous,
    target_frame_count = 0,
    frames = {}
  }
end


function XimeaRosCam:hardwareTriggeredCapture(numberOfFrames, exposureTimeInMicroSeconds)
  local t = torch.Timer()

  local XI_TRG_EDGE_RISING = 1
  local XI_TRG_SOFTWARE = 3
  local camera = self.camera
  --camera:stopAcquisition()
  --camera:setParamInt("trigger_source", XI_TRG_EDGE_RISING)
  --camera:startAcquisition()

  local frames = {}
  local processingDelayInMicroSeconds = 1000
  local waitForSeconds = (133333 - exposureTimeInMicroSeconds + processingDelayInMicroSeconds) / 1000000

  for i = 1, numberOfFrames do
    local last = t:time().real

    --print("[XimeaRosCam:hwTrigger] wait for s:", waitForSeconds)
    --sys.sleep(waitForSeconds)
    local imageMessage = self:capture(false)
    if i > 20 then -- fringe patterns are shown 133ms, because it takes the projector 400ms to load the next image
      sys.sleep(waitForSeconds)
    end

    if imageMessage then
      table.insert(frames, imageMessage)
    else
      print("[XimeaRosCam:hwTrigger] ERR: missed frame!")
    end

    print ("Retrieved " .. i .. " in " .. t:time().real - last)
    last = t:time().real
  end

  --camera:stopAcquisition()
  --camera:setParamInt("trigger_source", XI_TRG_SOFTWARE)
  --camera:startAcquisition()

  print("[XimeaRosCam:hwTrigger] done in " .. t:time().real)
  return frames
end


function XimeaRosCam:hardwareTriggeredCaptureFullAuto(numberOfFrames, exposureTimeInMicroSeconds)
  local t = torch.Timer()
  self:startTrigger(numberOfFrames, exposureTimeInMicroSeconds)

  -- Retrieve frames from camera buffer
  local processingDelayInMicroSeconds = 1000
  local waitForSeconds = (exposureTimeInMicroSeconds + processingDelayInMicroSeconds) / 1000000
  print("[XimeaRosCam:hwTrigger] retrieving frames")
  local frames = {}
  for i = 1, numberOfFrames do
    sys.sleep(waitForSeconds);
    local last = t:time().real
    local imageMessage = self:capture(true)
    if imageMessage then
      table.insert(frames, imageMessage)
    end
    print ("Retrieved " .. i .. " in " .. t:time().real - last)
    last = t:time().real
  end

  self:stopTrigger()
  print("[XimeaRosCam:hwTrigger] done in " .. t:time().real)
  return frames
end


function XimeaRosCam:checkForNewFrame()
  --ros.INFO('[XimeaRosCam] Check for frame of camera: %s', self.serial)
  local imageMessage = self:capture(true, 100)
  if imageMessage and self.state.frames ~= nil then
    table.insert(self.state.frames, imageMessage)
    --ros.INFO('[XimeaRosCam] Received frame #%d from %s', #self.state.frames, self.serial)
  end
end