package.path = package.path .. ';../ximeaClient/?.lua'

local ros = require 'ros'
local image = require 'image'
dofile '../ximea_client/ximeaClient.lua'
ros.init('ximea_capture_demo')


local nh = ros.NodeHandle()

local xc = ximeaClient(nh, "ximea_stereo")

for i=1,1 do
  print('Capture frame ' .. i)
  img = xc:getImage(1)
  if img then
    print(img:size())
    image.display(img)
  end
end

ros.shutdown()
