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

local ximea = require 'ximea'
local cv = require 'cv'
require 'cv.highgui'
local ros = require('ros')
ros.init('Ximea')

local cim = require 'CameraInfoManager'

local imagePublisher
local infoPublisher
local camera

--[[ command line arguments ]]--
cmd = torch.CmdLine()
cmd:text()
cmd:text('Expose Ximea camera to ros environment.')
cmd:text('')
cmd:text('Example:')
cmd:text('$> th ros_node_image_publisher.lua ')
cmd:option('--exposure', 16666, 'exposure time')
cmd:option('--cameraName', "ximea", 'exposure time')
cmd:text()
opt = cmd:parse(arg or {})

print(opt)
local exposure = opt.exposure
local is_reconnected = false -- Is used in init() function and in run()

local nh = ros.NodeHandle()

-- JointPosition --
local image_spec = ros.MsgSpec('sensor_msgs/Image')
imagePublisher = nh:advertise(string.format("%s/image_raw",opt.cameraName), image_spec)

local sensor_spec = ros.MsgSpec('sensor_msgs/CameraInfo')
infoPublisher = nh:advertise(string.format("%s/camera_info",opt.cameraName), sensor_spec)


local spinner
spinner = ros.AsyncSpinner()
spinner:start()


local function generateHeader(header)
  header.stamp = ros.Time.now()
  header.frame_id = opt.cameraName
  return header
end

local function sendImage(img)
  local m = ros.Message(image_spec)
  m.header = generateHeader(m.header)
  m.height = img:size(1)--uint32 image height, that is, number of rows
  m.width = img:size(2) --uint32 image width, that is, number of columns
  m.encoding = "rgb8"--string Encoding of pixels -- channel meaning, ordering, size
             --taken from the list of strings in include/sensor_msgs/image_encodings.h
  m.is_bigendian = 0--uint8 is this data bigendian?
  m.step = img:stride(1)--img:size(2) * img:size(3)--uint32 Full row length in bytes
  m.data = img:reshape(img:size(1)*img:size(2) * img:size(3))--actual matrix data, size is (step * rows)
  imagePublisher:publish(m)
end


local function sendInformation(calibration, h,w,c)
  local m = ros.Message(sensor_spec)
  m.header = generateHeader(m.header)
  m.height = h
  m.width = w
  m.distortion_model = "plumb_bob"
 -- m.R = torch.Tensor(12):zero() --3x4 row-major matrix float64[12]
  --m.K = torch.Tensor(12):zero() --3x4 row-major matrix float64[12]
 -- m.D = torch.Tensor(5):zero() --For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3). float64[]
  --m.P = torch.Tensor(12):zero() --3x4 row-major matrix float64[12]
  m.binning_x = h*c
  m.binning_y = w*c
  m.roi.x_offset = 0
  m.roi.x_offset = 0
  m.roi.height = m.height
  m.roi.width = m.width
  m.roi.do_rectify = false
  infoPublisher:publish(m)
end


local function init()
  camera = ximea.SingleCam()
  local n = camera:getNConnectedDevices()
  if n == 0 then
    print("No camera connected!")
  else
    camera:openCamera(n-1, "RGB24")
  end
  camera:setExposure(exposure)
  if not mycim:isCalibrated() then
    mycim:setCameraName("ximea")
  end
end

local function run()
  init()
  print("Finished initialisation")

  while true do
    if not ros.ok() then
      print("ROS NOT OK")
      return
    end
    print("get image")
    --local image = camera:getImage()
    sendInformation(nil,image:size(1),image:size(2),image:size(3))
    sendImage(torch.Tensor(64,48,3):zero())
    ros.Duration(0.08):sleep()
    ros.spinOnce()
  end
end

run()
