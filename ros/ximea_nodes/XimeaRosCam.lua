local ros = require 'ros'
local ximea = require 'ximea'
local ximea_ros = require 'ximea_ros'
local ioboard = require 'IoBoard'

local cv = require 'cv'
require 'cv.imgcodecs'
require 'cv.imgproc'
require 'cv.calib3d'

local XI_RET,XI_RET_TEXT = ximea.XI_RET, ximea.XI_RET_TEXT


local XimeaRosCam = torch.class('ximea_ros.XimeaRosCam', ximea_ros)


local DEFAULT_FLAGS = {
  enableFPNCorrection = true,
  autoWhiteBalance = true
}


local function XI_CHECK(status, msg)
  if status ~= XI_RET.XI_OK then
    ros.ERROR(msg)
    error(msg)
  end
end


function XimeaRosCam:__init(nh, serial, mode, flags)
  self.nh = nh
  self.camera = ximea.SingleCam()
  if type(serial) == 'number' then
    self.camera:open(serial, mode, false)    -- interpret serial arg as device_index
  elseif type(serial) == 'string' then
    self.camera:openCameraWithSerial(serial, mode, false)
  else
    error('serial argument must be of type sring or number.')
  end

  self.serial = self.camera:getSerial()
  self.camera_topic = nh:advertise(self.camera:getSerial(), ximea_ros.image_spec, 1, false)
  self.flags = flags or DEFAULT_FLAGS

  if self.flags.enableFPNCorrection then
    print('Enabling PFN correction')
    XI_CHECK(self.camera:setParamInt(ximea.PARAM.XI_PRM_COLUMN_FPN_CORRECTION, 1))
  end

  if self.flags.autoWhiteBalance and (mode == 'RGB24' or mode == 'RGB32') then
    print('Enabling auto white-balance')
    XI_CHECK(self.camera:setParamInt(ximea.PARAM.XI_PRM_AUTO_WB, 1))
  end

  XI_CHECK(self.camera:startAcquisition())
end


function XimeaRosCam:hasSubscribers()
  return self.camera_topic ~= nil and self.camera_topic:getNumSubscribers() > 0
end


function XimeaRosCam:capture(hardwareTriggered)
  hardwareTriggered = hardwareTriggered or false
  local img = self.camera:getImage(hardwareTriggered)
  if img == nil then
    ros.WARN('Capturing image faild (cam serial: %s)', self.serial)
    return nil
  end

  local rosMessage
  if hardwareTriggered then
    img = cv.cvtColor{img, code = cv.COLOR_BGR2GRAY}
    rosMessage = ximea_ros.createImageMessage(img, self.serial, 0)
  else
    rosMessage = ximea_ros.createImageMessage(img, self.serial, self.camera:getColorMode())
  end
  return rosMessage
end


function XimeaRosCam:publishFrame()
  if not self:hasSubscribers() then
    return
  end

  local msg = self:capture()
  if msg ~= nil then
    self.camera_topic:publish(msg)
  end
end


function XimeaRosCam:setExposure(value)
  self.camera:setExposure(value)
end


function XimeaRosCam:close()
  if self.camera_topic ~= nil then
    self.camera_topic:shutdown()
    self.camera_topic = nil
  end

  if self.camera ~= nil then
    self.camera:close()
    self.camera = nil
  end
end


function XimeaRosCam:startTrigger(numberOfFrames, exposureTimeInMicroSeconds)
  -- Set camera into hardware triggered mode
  print(string.format("[XimeaRosCam:hwTrigger] start hw triggering for cam %s", self.serial))
  print("[XimeaRosCam:hwTrigger] re-configure camera")
  local XI_TRG_EDGE_RISING = 1
  local camera = self.camera
  camera:stopAcquisition()
  camera:setParamInt("trigger_source", XI_TRG_EDGE_RISING)
  camera:setParamInt("acq_buffer_size", 6.5 * numberOfFrames + 50)
  camera:setParamInt("buffers_queue_size", numberOfFrames + 1)
  camera:setParamInt("recent_frame", 0)
  camera:startAcquisition()

  -- Configure IOboard
  local squareSignalFrequency = 1000000 / (exposureTimeInMicroSeconds)
  print("[XimeaRosCam:hwTrigger] configure ioboard. squareSignalFrequency: ", squareSignalFrequency)
  ioboard.open()
  ioboard.setSquareFrequency(squareSignalFrequency)

  -- Start triggering and wait for completion
  print("[XimeaRosCam:hwTrigger] start triggering")
  ioboard.activateSquareSignal()
end


function XimeaRosCam:stopTrigger()
  -- Stop triggering and restore buffer settings
  local XI_TRG_SOFTWARE = 3
  print("[XimeaRosCam:hwTrigger] hold triggering")
  local camera = self.camera
  ioboard.setTrigger(false)
  camera:stopAcquisition()
  camera:setParamInt("trigger_source", XI_TRG_SOFTWARE)
  camera:setParamInt("acq_buffer_size", 50)
  camera:setParamInt("buffers_queue_size", 4)
  camera:setParamInt("recent_frame", 1)
  camera:startAcquisition()
end


function XimeaRosCam:hardwareTriggeredCapture(numberOfFrames, exposureTimeInMicroSeconds)
  local t = torch.Timer()
  print("[XimeaRosCam:hwTrigger] preparing for semi-auto hardware triggering")
  ioboard.open()
  ioboard.setTrigger(ioboard.LOW)

  local XI_TRG_EDGE_RISING = 1
  local XI_TRG_SOFTWARE = 3
  local camera = self.camera
  --camera:stopAcquisition()
  --camera:setParamInt("trigger_source", XI_TRG_EDGE_RISING)
  --camera:startAcquisition()

  local frames = {}
  local processingDelayInMicroSeconds = 1000
  local waitForSeconds = (133333 - exposureTimeInMicroSeconds + processingDelayInMicroSeconds) / 1000000

  for i = 1, numberOfFrames do
    local last = t:time().real

    ioboard.setTrigger(ioboard.HIGH)
    --print("[XimeaRosCam:hwTrigger] wait for s:", waitForSeconds)
    --sys.sleep(waitForSeconds)
    local imageMessage = self:capture(false)
    if i > 20 then -- fringe patterns are shown 133ms, because it takes the projector 400ms to load the next image
      sys.sleep(waitForSeconds)
    end
    ioboard.setTrigger(ioboard.LOW)

    if imageMessage then
      table.insert(frames, imageMessage)
    else
      print("[XimeaRosCam:hwTrigger] ERR: missed frame!")
    end

    --camera:stopAcquisition()
    --camera:setParamInt("trigger_source", XI_TRG_SOFTWARE)
    --camera:startAcquisition()

    print ("Retrieved " .. i .. " in " .. t:time().real - last)
    last = t:time().real
  end

  ioboard.setTrigger(ioboard.HIGH)
  ioboard.close()

  print("[XimeaRosCam:hwTrigger] done in " .. t:time().real)
  return frames
end


function XimeaRosCam:hardwareTriggeredCaptureFullAuto(numberOfFrames, exposureTimeInMicroSeconds)
  local t = torch.Timer()
  self:startTrigger(numberOfFrames, exposureTimeInMicroSeconds)

  -- Retrieve frames from camera buffer
  local processingDelayInMicroSeconds = 1000
  local waitForSeconds = (exposureTimeInMicroSeconds + processingDelayInMicroSeconds) / 1000000
  print("[XimeaRosCam:hwTrigger] retrieving frames")
  local frames = {}
  for i = 1, numberOfFrames do
    sys.sleep(waitForSeconds);
    local last = t:time().real
    local imageMessage = self:capture(true)
    if imageMessage then
      table.insert(frames, imageMessage)
    end
    print ("Retrieved " .. i .. " in " .. t:time().real - last)
    last = t:time().real
  end

  self:stopTrigger()
  print("[XimeaRosCam:hwTrigger] done in " .. t:time().real)
  return frames
end
