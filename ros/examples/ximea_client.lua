local ros = require 'ros'
require 'ximea.ros.XimeaClient'

ros.init('ximea_client_demo')

local nh = ros.NodeHandle()

local xc = XimeaClient(nh)

for i=1,10 do
  print('Capturing frame ' .. i)
  local img1, img2, serials = xc:getImages()
  print('Imgae 1 size:')
  print(img1:size())
  print('Imgae 2 size:')
  print(img2:size())
  print('Serial numbers:')
  print(serials)
end

xc:shutdown()

ros.shutdown()
