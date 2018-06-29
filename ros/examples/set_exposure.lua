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

ros.init('ximea_set_exposure')

local nh = ros.NodeHandle()

local sendCommandClient = nh:serviceClient('ximea_stereo/send_command', ros.SrvSpec('ximea_msgs/SendCommand'))

local function sendCommand(command_name, value)
  local req = sendCommandClient:createRequest()
  req.command_name = command_name
  req.value = value or 0
  sendCommandClient:call(req)
end

local function setExposure(exposure_micro_sec)
  sendCommand("setExposure", exposure_micro_sec)
end

local exposure = 16666
for i=1,10 do
  print('Capture frame ' .. i)
  setExposure(exposure * i)
--  exposure = exposure + 1000
end

ros.shutdown()
