local ximea = require 'ximea'
local cv = require 'cv'
require 'cv.highgui'


local exposure = 16666


local b = ximea.StereoCam()
local n = b:getNConnectedDevices()
if n == 0 then
  error("No camera connected!")
else
  b:openCamera("RGB24")
end


timer = torch.Timer()

for j = 1,3 do
  for i = 1,50 do
    b:setExposure(exposure)
    b:setExposureWithSerial(b.serial_cam2, exposure + 80000)
    exposure = exposure + 1000
    local image, image2 = b:getImage()
    cv.imshow{"Image  - ".. b.serial_cam1, image}
    cv.imshow{"Image2 - ".. b.serial_cam2, image2}
    cv.waitKey{5}
  end
  print("Close and open camera again")
  b:close()
  b:openCamera("RGB24")
end

print('Time elapsed: ' .. timer:time().real .. ' seconds')
