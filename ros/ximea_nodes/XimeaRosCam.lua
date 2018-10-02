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

local XI_TRG_EDGE_RISING = 1
local XI_TRG_EDGE_FALLING = 2
local XI_TRG_SOFTWARE = 3
local XI_GPI_TRIGGER = 1


local XimeaRosCam = torch.class('ximea_ros.XimeaRosCam', ximea_ros)


local CaptureMode = {
  continuous = 0,
  triggered = 1
}

local DEFAULT_FLAGS = {
  enableFPNCorrection = true,
  autoWhiteBalance = true
}


local function log(...)
  local msg = string.format(...)
  ros.INFO(msg)
end


local function fail(...)
  local msg = string.format(...)
  ros.ERROR(msg)
  error(msg)
end


local function XI_CHECK(status, msg)
  if status ~= XI_RET.XI_OK then
    fail(msg)
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
    fail('Serial argument must be of type sring or number.')
  end

  if not self.camera:isOpen() then
    fail("Failed to open camera with serial '%s'.", serial)
  end

  self.serial = self.camera:getSerial()
  if self.serial == nil or #self.serial == 0 then
    fail('Unexpected: Internal camera serial number is nil or empty.')
  end

  self.camera_topic = nh:advertise('SN_' .. escapeRosName(self.serial), ximea_ros.image_spec, 1, false)
  self.flags = flags or DEFAULT_FLAGS

  if self.flags.enableFPNCorrection then
    log('Enabling PFN correction')
    XI_CHECK(self.camera:setParamInt(ximea.PARAM.XI_PRM_COLUMN_FPN_CORRECTION, 1))
  end

  if self.flags.autoWhiteBalance and (mode == 'RGB24' or mode == 'RGB32') then
    log('Enabling auto white-balance')
    XI_CHECK(self.camera:setParamInt(ximea.PARAM.XI_PRM_AUTO_WB, 1))
  end

  self.camera:setParamInt(ximea.PARAM.XI_PRM_TRG_SOURCE, XI_TRG_SOFTWARE)
  self.camera:setParamInt(ximea.PARAM.XI_PRM_BUFFERS_QUEUE_SIZE, 4)
  self.camera:setParamInt(ximea.PARAM.XI_PRM_RECENT_FRAME, 1)

  log('Start camera acquisition')
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


function XimeaRosCam:capture(triggered, timeout)
  triggered = triggered or false
  timeout = timeout or 1000

  -- AKo: retry was added, because in long-running tests failures had been observed (not retrying when hw-triggered)
  for i=1,5 do
    local img = self.camera:getImage(triggered, timeout)
    if img ~= nil then
      local rosMessage = ximea_ros.createImageMessage(img, self.serial, self.camera:getColorMode())
      return rosMessage
    end

    local b = -1
    if triggered == true then
      b = 1
    elseif triggered == false then
      b = 0
    end
    ros.WARN('Capturing image failed (cam serial: %s, hw trigger %d, timeout %d, try: %d)', self.serial, b, timeout or -1, i)

    -- do not retry if hardware triggering is used
    if triggered then break end
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


function XimeaRosCam:softwareTrigger()
  return self.camera:softwareTrigger()
end


function XimeaRosCam:startTrigger(numberOfFrames, exposureTimeInMicroSeconds, trigger_source, gpi_index)
  trigger_source = trigger_source or XI_TRG_EDGE_RISING
  gpi_index = gpi_index or 1

  -- Set camera into hardware triggered mode
  log("[XimeaRosCam:hwTrigger] start triggering for camera %s", self.serial)
  log("[XimeaRosCam:hwTrigger] re-configure camera")

  local camera = self.camera

  --camera:stopAcquisition()    -- AKo: Commented out since it is working without it and starting/stopping acquisition is slow
  if trigger_source == XI_TRG_EDGE_RISING or trigger_source == XI_TRG_EDGE_FALLING then
    camera:setParamInt(ximea.PARAM.XI_PRM_GPI_SELECTOR, gpi_index)
    camera:setParamInt(ximea.PARAM.XI_PRM_GPI_MODE, XI_GPI_TRIGGER)
    camera:setParamInt(ximea.PARAM.XI_PRM_TRG_SOURCE, trigger_source)
  elseif trigger_source == XI_TRG_SOFTWARE then
    camera:setParamInt(ximea.PARAM.XI_PRM_TRG_SOURCE, XI_TRG_SOFTWARE)
  else
    fail("Invalid trigger source '%d' specified", trigger_source)
  end
  camera:setParamInt(ximea.PARAM.XI_PRM_BUFFERS_QUEUE_SIZE, numberOfFrames + 1)
  camera:setParamInt(ximea.PARAM.XI_PRM_RECENT_FRAME, 0)
  log("[XimeaRosCam:hwTrigger] set exposure to " .. exposureTimeInMicroSeconds)
  self:setExposure(exposureTimeInMicroSeconds)
  --camera:startAcquisition()
  self.state = {
    mode = CaptureMode.triggered,
    target_frame_count = numberOfFrames,
    frames = {}
  }
end


function XimeaRosCam:stopTrigger()
  -- Stop triggering and restore buffer settings
  log("[XimeaRosCam:hwTrigger] stop triggering")
  local camera = self.camera
  --camera:stopAcquisition()
  camera:setParamInt(ximea.PARAM.XI_PRM_TRG_SOURCE, XI_TRG_SOFTWARE)
  camera:setParamInt(ximea.PARAM.XI_PRM_BUFFERS_QUEUE_SIZE, 4)
  camera:setParamInt(ximea.PARAM.XI_PRM_RECENT_FRAME, 1)
  --camera:startAcquisition()
  self.state = {
    mode = CaptureMode.continuous,
    target_frame_count = 0,
    frames = {}
  }
end


function XimeaRosCam:checkForNewFrame()
  --ros.INFO('[XimeaRosCam] Check for frame of camera: %s', self.serial)
  local imageMessage = self:capture(true, 100)
  if imageMessage and self.state.frames ~= nil then
    table.insert(self.state.frames, imageMessage)
    --ros.INFO('[XimeaRosCam] Received frame #%d from %s', #self.state.frames, self.serial)
  end
end
