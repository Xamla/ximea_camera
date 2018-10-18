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
local slstudio_service = require 'slstudio_service'
require 'ros.actionlib.SimpleActionServer'
local actionlib = ros.actionlib
local tf = ros.tf
require 'ros.PointCloud2SerializationHandler'

local SlstudioService = torch.class('slstudio_service.SlstudioService', slstudio_service)
local slstudio -- is required during constructor, because it might not be installed on all systems and is enabled via a flag
local setupService
local cloudPublisher
local scanActionServer
local slstudioInstance = {
  initialized = false,
  shutterSpeed = nil,
  scanning = false,
}


local function handleSlstudioSetup(request, response, header)
  local calibrationFile = request.stereo_configuration_file_path
  local leftSerial = request.serials[1]
  local rightSerial = request.serials[2]
  local minDist = 300
  local maxDist = 2500
  local shutterSpeed = request.shutter_speed_in_ms
  if (#request.crop_parameter == 2) then
    minDist = request.crop_parameter[1]
    maxDist = request.crop_parameter[2]
  end

  if calibrationFile == nil or paths.filep(calibrationFile) == false then
    ros.WARN('Received setup service call with a calibration file that could not be found.')
    response.success = false
    return false
  end

  if leftSerial == nil or rightSerial == nil then
    ros.WARN('Received setup service call where at least one serial number is missing.')
    response.success = false
    return false
  end

  if slstudioInstance.initialized == true then
    ros.INFO('slstudio has been already been initialized. Shutting down current instance, before starting new one.')
    slstudioInstance.initialized = false
    slstudio:quitScanStereo()
  end

  local code = slstudio:initScanStereoAsync(calibrationFile, slstudio.CAMERA_ROS, leftSerial, slstudio.CAMERA_ROS, rightSerial, minDist, maxDist, true, shutterSpeed);

  if code == 0 then
    slstudioInstance.initialized = true
    slstudioInstance.shutterSpeed = shutterSpeed
    ros.INFO('slstudio has been initialized with a shutter speed of %f', shutterSpeed)
    response.success = true
    return true
  else
    response.success = false
    response.error_message = 'Could not initialize slstudio.'
    ros.ERROR('Could not initialize slstudio.')
    return false
  end
end


function handleNewScanGoal(self)
  ros.INFO("Received new scan goal")

  if slstudioInstance.scanning == true then
    ros.WARN('A scan was triggered while another one was ongoing. Aborting both.')
    local r = scanActionServer:createResult()
    r.success = false
    scanActionServer:setAborted(r, 'A scan was triggered while another scan was already ongoing.')

    local goal = scanActionServer:acceptNewGoal()
    scanActionServer:setAborted(r, 'The scan was triggered while another scan was already ongoing.')

    return
  end

  local goal = scanActionServer:acceptNewGoal()

  if slstudioInstance.initialized == false then
    local r = scanActionServer:createResult()
    r.success = false
    scanActionServer:setAborted(r, 'You need to call the slstudio setup service before.')
    ros.WARN('A scan was triggered before the setup service has been executed. Aborting.')
    return
  end

  if goal.goal.shutter_speed_in_ms > slstudioInstance.shutterSpeed then
    local r = scanActionServer:createResult()
    r.success = false
    scanActionServer:setAborted(r, 'Shutter speed must not be larger as the shutter speed set during setup.')
    ros.WARN('Received a goal with a shutter speed larger than the shutter speed set during setup. Aborting.')
    return
  end

  slstudioInstance.scanning = true
  local code, cloud, imageOn, imageOff, imageOnboard, imageShading = slstudio:scanStereoAsync(goal.goal.shutter_speed_in_ms, self.spinCallback)
  slstudioInstance.scanning = false

  local r = scanActionServer:createResult()
  if code == 0 then
    r.success = true
    r.cloud = cloud
    scanActionServer:setSucceeded(r, '')
    ros.INFO('Scan succeeded.')
    cloud:setHeaderFrameId('world')
    local d = torch.tic()
    cloudPublisher:publish(cloud)
    print(torch.toc(d))
  else
    r.success = false
    ros.WARN('Scan failed with code %d.', code)
    scanActionServer:setAborted(r, 'Scan failed.')
  end
end


function SlstudioService:__init(spinCallback)
  slstudio = require 'slstudio'
  self.spinCallback = spinCallback
  print('Slstudio initialized')
end


function SlstudioService:startServices(nh)
  self:shutdown()

  local handler = ros.PointCloud2SerializationHandler()
  nh:addSerializationHandler(handler)

  setupService = nh:advertiseService('slstudio/setup', ros.SrvSpec('ximea_msgs/SlstudioSetup'), handleSlstudioSetup)
  cloudPublisher = nh:advertise('slstudio/cloud', 'sensor_msgs/PointCloud2', 1)

  scanActionServer = actionlib.SimpleActionServer(nh, 'slstudio/scan', 'ximea_msgs/SlstudioScan')
  scanActionServer:registerGoalCallback(function() handleNewScanGoal(self) end)
  scanActionServer:start()
end


function SlstudioService:shutdown()
  if setupService ~= nil then
    setupService:shutdown()
    setupService = nil
  end

  if cloudPublisher ~= nil then
    cloudPublisher:shutdown()
    cloudPublisher = nil
  end

  if scanActionServer ~= nil then
    scanActionServer:shutdown()
    scanActionServer = nil
  end
end