package.path = package.path .. ';../ximeaClient/?.lua'

local ros = require 'ros'
local image = require 'image'
dofile '../ximea_client/ximeaClient.lua'
ros.init('ximea_capture_demo')


local nh = ros.NodeHandle()

local xc = ximeaClient(nh, "ximea_stereo")
xc:setExposure(366666)
for i=1,10 do
  print('Capture frame ' .. i)
  img = xc:getImage(2)
  if img then
    print(img:size())
    w = image.display{image=img, offscreen=false, win=w}
  end
end

ros.shutdown()
