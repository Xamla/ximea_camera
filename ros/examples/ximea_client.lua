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
require 'ximea.ros.XimeaClient'

ros.init('ximea_client_demo')

local nh = ros.NodeHandle()

local xc = XimeaClient(nh)
xc:setExposure(80000)

--[[for i=1,10 do
  print('Capturing frame ' .. i)
  local img1, img2, serials = xc:getImages()
  print('Imgae 1 size:')
  print(img1:size())
  print('Imgae 2 size:')
  print(img2:size())
  print('Serial numbers:')
  print(serials)
  if i == 5 then
    print('Closing camera...')
    xc:close()
    print('Opening camera...')
    xc:open()
    xc:setExposure(80000)
  end
end
--]]
xc:close()

xc:shutdown()

ros.shutdown()
