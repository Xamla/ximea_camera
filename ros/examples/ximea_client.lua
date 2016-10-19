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
