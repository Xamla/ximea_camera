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


local serials = ximea.getSerialNumbers()
for i = 1, #serials do
  print("Cam " .. i-1 .. " has serial " .. serials[i])
end

for i = 1, #serials do
  local cam1 = ximea.SingleCam()
  cam1:openCameraWithSerial(serials[i])
  local img = cam1:getImage()
  cv.imshow{"IMG with serial ".. serials[i], img}
  cv.waitKey{-1}
  cam1:close()
end


local b = ximea.SingleCam()
local n = b:getNConnectedDevices()
if n == 0 then
  error("No camera connected!")
else
  b:openCamera(n-1, "RGB24")
end


local exposure = 16666

b:setExposure(exposure)
timer = torch.Timer()

for j = 1,3 do
  for i = 1,50 do
    b:setExposure(exposure)
    exposure = exposure + 1000
    local image = b:getImage()
    cv.imshow{"Image", image}
    cv.waitKey{5}
  end

  print("Close and open camera again")
  b:close()
  b:openCamera(n-1)
end

print('Time elapsed: ' .. timer:time().real .. ' seconds')
