local ros = require 'ros'
require 'ximea.ros.XimeaClient'
local cv = require 'cv'
require 'cv.imgproc'
require 'cv.imgcodecs'
require 'cv.highgui'


local function main()

  -- initialize ROS
  ros.init('trigger_camera')
  local nh = ros.NodeHandle()
  local sp = ros.AsyncSpinner()
  sp:start()

  local cmd = torch.CmdLine()
  cmd:text()
  cmd:text('Set exposure utility')
  cmd:text()
  cmd:option('-cam', 'scan', 'camera name one of [board, scan, onboard], empty for all')
  cmd:option('-serial', '28674051', 'serial number')
  cmd:option('-exposure', 20000, 'exposure time in micro seconds')
  cmd:option('-frames', 12, 'number of frames')
  cmd:option('-time', 20, 'time in milliseconds')
  cmd:option('-action_name', '/ximea_mono/trigger', 'ximea action name')
  local opt = cmd:parse(arg or {})

  local ximea_client = XimeaClient(nh, 'ximea_mono', false, false, nil, opt.action_name)
  local serials = {'CAMAU1710001', 'CAMAU1639042'}
  ximea_client:setExposure(opt.exposure, serials)

  -- triggering test
  local image_set
  local output

  local exposures = {20000, 20000}
  --for i = 1, 100 do
    local t = torch.Timer()
    local ok, err = pcall(function() output = ximea_client:trigger(serials, 24, exposures, 0) end)
    if ok then
      print('Triggering worked fine')
      image_set = output
      print("Got " .. #image_set .. " image sets in " .. t:time().real .. " seconds.")
      -- write images to disk
      local outputDirectoryId = os.date("%Y-%m-%d_%H%M%S")
      local outputDirectory = path.join("/home/xamla/code/tmp/", outputDirectoryId)
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
      print('Triggering failed with error:')
      print(err)
    end
    print('end of loop #', i)
  --end
  ximea_client:shutdown()
  sp:stop()
  ros.shutdown()
end

main()
