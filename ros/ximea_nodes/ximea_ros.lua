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

local ffi = require 'ffi'
local ros = require 'ros'
local ximea = require 'ximea'


local ximea_ros = {}


local image_spec = ros.MsgSpec('sensor_msgs/Image')
local cameraInfo_spec = ros.MsgSpec('sensor_msgs/CameraInfo')


ximea_ros.image_spec = image_spec
ximea_ros.cameraInfo_spec = cameraInfo_spec


function ximea_ros.createImageMessage(img, serial, color_mode)
  local msg = ros.Message(image_spec)
  msg.header.stamp = ros.Time.now()
  msg.header.frame = serial

  msg.height = img:size(1)    -- image height
  msg.width = img:size(2)     -- image width

  if color_mode == ximea.XI_IMG_FORMAT.RGB24 then
    msg.encoding = "bgr8"
  elseif color_mode == ximea.XI_IMG_FORMAT.MONO8 or color_mode == ximea.XI_IMG_FORMAT.RAW8 then
    msg.encoding = "mono8"
  elseif color_mode == ximea.XI_IMG_FORMAT.MONO16 or color_mode == ximea.XI_IMG_FORMAT.RAW16 then
    msg.encoding = "mono16"
  else
    error('Unsupported color format.')
  end
  msg.step = img:stride(1)
  msg.data = img:reshape(img:nElement())   -- actual matrix data, size is (step * rows)
  msg.is_bigendian = ffi.abi('be')
  return msg
end


return ximea_ros
