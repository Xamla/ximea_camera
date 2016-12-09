local ros = require 'ros'
require 'ximea.ros.XimeaClient'
cv = require 'cv'
require 'cv.highgui'
require 'cv.imgproc'
require 'cv.imgcodecs'


local CAMERA_SERIALS = {
  board ='40600628',   -- pcb board pose measuring and visual servoing cam
  scan = '28670151',   -- structured light scanning camera
  onboard = '35371951', -- mounted on robot
  spare = '28674051',
}


local function main()
  ros.init('trigger_camera')

  local cmd = torch.CmdLine()
  cmd:text()
  cmd:text('Set exposure utility')
  cmd:text()
  cmd:option('-cam', 'scan', 'camera name one of [board, scan, onboard], empty for all')
  cmd:option('-serial', '', 'serial number')
  cmd:option('-time', 20, 'time in ms')
  cmd:option('frames', 24, 'number of frames')
  local opt = cmd:parse(arg or {})

  local xc = XimeaClient(nh, 'ximea_mono', false, false)

  local serial
  if opt.cam ~= '' then
    serial = CAMERA_SERIALS[opt.cam]
  elseif opt.serial ~= '' then
    serial = opt.serial
  end

  if serial and opt.frames then
    local t = torch.Timer()
    local images = xc:trigger(serial, opt.frames, opt.time * 1000)
    print("Got " .. #images .. " images in " .. t:time().real)
    for i = 1, #images do
      cv.imwrite{"/tmp/" .. i .. ".jpg", images[i]}
    end
  end

  xc:shutdown()

  ros.shutdown()
end


main()
