local ximea = require 'ximea'
local cv = require 'cv'
require 'cv.highgui'
local ros = require('ros')
ros.init('Ximea')

local cim_o =require 'CameraInfoManager'

local imagePublisher
local infoPublisher
local camera
local seqenceNumber = 1

--[[command line arguments]]--
cmd = torch.CmdLine()
cmd:text()
cmd:text('Expose Ximea camera to ros environment.')
cmd:text('')
cmd:text('Example:')
cmd:text('$> th ros_node_image_publisher.lua ')
cmd:option('--exposure', 16666, 'exposure time')
cmd:option('--cameraName', "ximea", 'exposure time')
cmd:text()
opt = cmd:parse(arg or {})
local timer = torch.Timer()
print(opt)
local exposure = opt.exposure
local is_reconnected = false -- Is used in init() function and in run()

local nodehandle
nodehandle = ros.NodeHandle()
local cim = cim_o.CameraInfoManager(nodehandle,opt.cameraName,"world")

-- JointPosition --
local image_spec = ros.MsgSpec('sensor_msgs/Image')
imagePublisher = nodehandle:advertise(string.format("%s/image_raw",opt.cameraName), image_spec)


local sensor_spec = ros.MsgSpec('sensor_msgs/CameraInfo')
infoPublisher = nodehandle:advertise(string.format("%s/camera_info",opt.cameraName), sensor_spec)

local spinner
spinner = ros.AsyncSpinner()
spinner:start()


local function generateHeader(header)
  header.seq = seqenceNumber
  header.stamp = ros.Time.now()
  header.frame_id = opt.cameraName
return header
end

local function sendImage(img)
  if img then
  local m = ros.Message(image_spec)
  m.header = generateHeader(m.header)
  m.height = img:size(1)--uint32 image height, that is, number of rows
  m.width = img:size(2) --uint32 image width, that is, number of columns
  m.encoding = "rgb8"--string Encoding of pixels -- channel meaning, ordering, size
             --taken from the list of strings in include/sensor_msgs/image_encodings.h
  m.is_bigendian = 0--uint8 is this data bigendian?

  m.step = img:size(2) * img:size(3)--uint32 Full row length in bytes
  m.data = img:reshape(img:size(1) * img:size(2) * img:size(3))--actual matrix data, size is (step * rows)
  imagePublisher:publish(m)
  end
end


local function sendInformation()
  infoPublisher:publish(cim:getCameraInfo())
end


local function init()
local serials = ximea.getSerialNumbers()
for  i = 1, #serials do
  print("Cam " .. i-1 .. " has serial " .. serials[i])
end
  camera = ximea.SingleCam()
  local n = camera:getNConnectedDevices()
  if n == 0 then
    print("No camera connected!")
  else
  camera:openCameraWithSerial(serials[1],"RGB24")
    
  end
  camera:setExposure(exposure)
end

local function run()
  init()
  print("Finished initialisation")

  while true do
    if not ros.ok() then
      print("ROS NOT OK")
      return
    end
    print("get image")
    local image = camera:getImage()
    sendInformation()
    sendImage(image)
    ros.Duration(0.06):sleep()
    ros.spinOnce()
    seqenceNumber = seqenceNumber +1
  end
end

run()
