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
local image = require 'image'

ros.init('ximea_capture_demo')

local nh = ros.NodeHandle()
local capture = nh:serviceClient('ximea_mono/capture', ros.SrvSpec('ximea_msgs/Capture'))
local sendCommandClient = nh:serviceClient('ximea_mono/send_command', ros.SrvSpec('ximea_msgs/SendCommand'))

local function sendCommand(command_name, value)
  local req = sendCommandClient:createRequest()
  req.command_name = command_name
  req.value = value or 0
  sendCommandClient:call(req)
end

local function setExposure(exposure_micro_sec)
  sendCommand("setExposure", exposure_micro_sec)
end

setExposure(366666)

for i=1,10 do
  print('Capture frame ' .. i)
  local response = capture:call()
  local img = response.images[1].data
  if img then
    print('Cam serial: ' .. response.serials[1])
    print(img:size())
  end
end

ros.shutdown()
