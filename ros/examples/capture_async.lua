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
local cv = require 'cv'
require 'cv.imgproc'
require 'cv.imgcodecs'
require 'cv.highgui'

local function main()

  -- initialize ROS
  ros.init('capture_async')
  local nh = ros.NodeHandle()
  local sp = ros.AsyncSpinner()
  sp:start()

  local cmd = torch.CmdLine()
  cmd:text()
  cmd:text('Capture async exapmle')
  cmd:text()
  cmd:option('-serial', '28670151', 'serial number')
  cmd:option('-exposure', 20000, 'exposure time in micro seconds')
  cmd:option('-action_name', '/ximea_mono/trigger2', 'ximea action name')
  cmd:option('-count', 1, 'number of test trigger runs')
  local opt = cmd:parse(arg or {})

  local ximea_client = XimeaClient(nh, 'ximea_mono', false, false, nil, opt.action_name)
  local serials = {opt.serial}
  local exposures = {opt.exposure}
  ximea_client:setExposure(opt.exposure, serials)

  local N = opt.count
  for i=1,N do
    local capture_task = ximea_client:captureTriggeredAsync(serials, 2, exposures, 1000, 3, 1)
    ximea_client:softwareTrigger()
    sys.sleep(0.1)
    ximea_client:softwareTrigger()
    local ok = capture_task:waitForCompletion(500)
    if not ok then
      capture_task:cancel()
    end
    if capture_task:hasCompletedSuccessfully() then
      ros.INFO("Capture sucessful")

      local image_set = capture_task:getResult()

      local outputDirectoryId = os.date("%Y-%m-%d_%H%M%S")
      local outputDirectory = path.join("/tmp/asyncCapture", outputDirectoryId)
      os.execute("mkdir -p " .. outputDirectory)
      for n = 1, #image_set do
        local images = image_set[n]
        for i = 1, #images do
          if images[i]:nDimension() > 2 then
            images[i] = cv.cvtColor{images[i], nil, cv.COLOR_RGB2BGR}
          end
          cv.imwrite{outputDirectory .. "/" .. n .. "_" .. i .. ".jpg", images[i]}
        end
      end
      print(string.format("Wrote images to folder %s.", outputDirectory))

    else
      local result = capture_task:getResult()
      local err = result.message or 'Unkown error'
      local code = result.code
      ros.ERROR('Action "%s" failed. The following error was reported: \'%s\' (code: %s)', opt.action_name, err, code)
    end
  end

  ximea_client:shutdown()
  sp:stop()
  ros.shutdown()
end

main()
