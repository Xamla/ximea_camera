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
local ximea = require 'ximea.env'


local XI_IMG_FORMAT = ximea.XI_IMG_FORMAT


function ximea.getNCameras()
  local intPtr = ffi.typeof("int[1]")
  local n = intPtr()
  ximea.lib.getNumberConnectedDevices(n)
  return n[0]
end


function ximea.getSerialNumbers()
  local n = ximea.getNCameras()
  local serials = {}

  local c_str = ffi.new("char[32]")
  for i = 1,n do
    ximea.lib.getSerialNumber(i-1, c_str);
    table.insert(serials, ffi.string(c_str))
  end

  return serials
end


function ximea.getXiModeByName(mode_name)
  if type(mode_name) == 'number' then
    return mode_name
  end

  -- normalize mode name
  mode_name = string.upper(mode_name)
  if string.sub(mode_name, 1, 3) == 'XI_' then
    mode_name = string.sub(mode_name, 4)
  end

  -- lookup mode by name
  local mode = XI_IMG_FORMAT[mode_name]
  if mode == nil then
    error(string.format('Unknown Ximea camera mode "%s".', mode_name))
  end
  return mode
end

ximea.OPENCV_IMG_FORMAT = {}
ximea.OPENCV_IMG_FORMAT.CV_32FC1 = '32FC1'
