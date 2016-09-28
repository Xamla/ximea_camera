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
