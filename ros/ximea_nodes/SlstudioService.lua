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
local slstudio_service = require 'slstudio_service'
require 'ros.actionlib.SimpleActionServer'
local actionlib = ros.actionlib
local tf = ros.tf
require 'ros.PointCloud2SerializationHandler'

local SlstudioService = torch.class('slstudio_service.SlstudioService', slstudio_service)
local slstudio -- is required during constructor, because it might not be installed on all systems and is enabled via a flag
local setupService
local heightAnalysisSetupService
local cloudPublisher
local scanActionServer
local heightAnalysisActionServer
local slstudioInstance = {
  initialized = false,
  shutterSpeed = nil,
  scanning = false,
}
local leftCameraSerial = ''
local MONO_COLOR_MODE = ximea.XI_IMG_FORMAT.MONO8
local RGB_COLOR_MODE = ximea.XI_IMG_FORMAT.RGB24
local heightAnalysis = slstudio_service.HeightAnalysis()
local heightAnalysisConfig = {
  maskMin = -0.0025,
  maskMax = 0.0025
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

  leftCameraSerial = leftSerial
  local code = slstudio:initScanStereoThreaded(calibrationFile, slstudio.CAMERA_ROS, leftSerial, slstudio.CAMERA_ROS, rightSerial, minDist, maxDist, true, shutterSpeed);

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

local function handleHeightAnalysisSetup(request, response, header)
  heightAnalysisConfig.maskMin = request.mask_min
  heightAnalysisConfig.maskMax = request.mask_max
  ros.INFO('Height Analysis has been configured to mask_min: %f, mask_max: %f', heightAnalysisConfig.maskMin, heightAnalysisConfig.maskMax)

  response.success = true
  return true
end

local function isScanOnGoing(actionServer)
  if slstudioInstance.scanning == true then
    ros.WARN('A scan was triggered while another one was ongoing. Aborting both.')
    local r = actionServer:createResult()
    r.success = false
    actionServer:setAborted(r, 'A scan was triggered while another scan was already ongoing.')

    local goal = actionServer:acceptNewGoal()
    actionServer:setAborted(r, 'The scan was triggered while another scan was already ongoing.')

    return true
  end
  return false
end


local function slstudioSetupCorrectly(actionServer, goal)
  if slstudioInstance.initialized == false then
    local r = actionServer:createResult()
    r.success = false
    actionServer:setAborted(r, 'You need to call the slstudio setup service before.')
    ros.WARN('A scan was triggered before the slstudio setup service has been executed. Aborting.')
    return false
  end

  if goal.goal.shutter_speed_in_ms > slstudioInstance.shutterSpeed then
    local r = actionServer:createResult()
    r.success = false
    actionServer:setAborted(r, 'Shutter speed must not be larger as the shutter speed set during setup.')
    ros.WARN('Received a goal with a shutter speed larger than the shutter speed set during slstudio setup. Aborting.')
    return false
  end

  return true
end


function handleNewScanGoal(self)
  ros.INFO("Received new scan goal")

  if (isScanOnGoing(scanActionServer) == true) then
    return
  end

  local goal = scanActionServer:acceptNewGoal()

  if (slstudioSetupCorrectly(scanActionServer, goal) == false) then
    return;
  end

  slstudioInstance.scanning = true
  local code, cloud, imageOn, imageOff, imageOnboard, imageShading = slstudio:scanStereoSpinning(goal.goal.shutter_speed_in_ms, self.spinCallback)
  slstudioInstance.scanning = false

  local r = scanActionServer:createResult()
  if code == 0 and cloud ~= nil then
    r.success = true
    r.cloud = cloud
    r.image_on = ximea_ros.createImageMessage(imageOn, leftCameraSerial, MONO_COLOR_MODE)
    r.image_off = ximea_ros.createImageMessage(imageOff, leftCameraSerial, MONO_COLOR_MODE)
    scanActionServer:setSucceeded(r, 'Scan succeeded.')
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


local function doHeightAnalysis(cloud, image_on)
  local h = heightAnalysis

  local height_map = h:generateHeightMap(cloud)
  local color_map = h:generateColorMap(height_map)
  local mask = h:generateMask(height_map, mask_min, mask_max, 1)
  return height_map, color_map, mask
end


function handleNewHeightAnalysisGoal(self)
  ros.INFO("Received new height analysis goal")

  if (isScanOnGoing(heightAnalysisActionServer) == true) then
    return
  end

  local goal = heightAnalysisActionServer:acceptNewGoal()

  if (slstudioSetupCorrectly(heightAnalysisActionServer, goal) == false) then
    return;
  end

  slstudioInstance.scanning = true
  local code, cloud, imageOn, imageOff, imageOnboard, imageShading = slstudio:scanStereoSpinning(goal.goal.shutter_speed_in_ms, self.spinCallback)
  slstudioInstance.scanning = false

  local height_map, color_map, mask
  if code == 0 and cloud ~= nil and cloud:points():dim() > 0 then
    height_map, color_map, mask = doHeightAnalysis(cloud, imageOn)
  end

  local r = heightAnalysisActionServer:createResult()
  if code == 0 and cloud ~= nil and color_map ~= nil then
    r.success = true
    r.image_on = ximea_ros.createImageMessage(imageOn, leftCameraSerial, MONO_COLOR_MODE)
    r.height_map = ximea_ros.createImageMessage(height_map, leftCameraSerial, ximea.OPENCV_IMG_FORMAT.CV_32FC1)
    r.mask = ximea_ros.createImageMessage(mask, leftCameraSerial, MONO_COLOR_MODE)
    r.color_map = ximea_ros.createImageMessage(color_map, leftCameraSerial, RGB_COLOR_MODE)
    heightAnalysisActionServer:setSucceeded(r, 'Height analysis was successfully.')
    ros.INFO('Scan succeeded.')
    cloud:setHeaderFrameId('world')
    local d = torch.tic()
    cloudPublisher:publish(cloud)
    print(torch.toc(d))
  else
    r.success = false
    ros.WARN('Scan failed with code %d.', code)
    heightAnalysisActionServer:setAborted(r, 'Scan failed.')
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

  setupService = nh:advertiseService('slstudio/setupSlstudio', ros.SrvSpec('ximea_msgs/SlstudioSetup'), handleSlstudioSetup)
  heightAnalysisSetupService = nh:advertiseService('slstudio/setupHeightAnalysis', ros.SrvSpec('ximea_msgs/HeightAnalysisSetup'), handleHeightAnalysisSetup)
  cloudPublisher = nh:advertise('slstudio/cloud', 'sensor_msgs/PointCloud2', 1)

  scanActionServer = actionlib.SimpleActionServer(nh, 'slstudio/scan', 'ximea_msgs/SlstudioScan')
  scanActionServer:registerGoalCallback(function() handleNewScanGoal(self) end)
  scanActionServer:start()

  heightAnalysisActionServer = actionlib.SimpleActionServer(nh, 'slstudio/heightAnalysis', 'ximea_msgs/HeightAnalysis')
  heightAnalysisActionServer:registerGoalCallback(function() handleNewHeightAnalysisGoal(self) end)
  heightAnalysisActionServer:start()
end


function SlstudioService:shutdown()
  if setupService ~= nil then
    setupService:shutdown()
    setupService = nil
  end

  if heightAnalysisSetupService ~= nil then
    heightAnalysisSetupService:shutdown();
    heightAnalysisSetupService = nil
  end

  if cloudPublisher ~= nil then
    cloudPublisher:shutdown()
    cloudPublisher = nil
  end

  if scanActionServer ~= nil then
    scanActionServer:shutdown()
    scanActionServer = nil
  end

  if heightAnalysisActionServer ~= nil then
    heightAnalysisActionServer:shutdown()
    heightAnalysisActionServer = nil
  end
end