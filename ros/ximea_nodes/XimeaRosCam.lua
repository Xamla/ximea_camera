local ros = require 'ros'
local ximea = require 'ximea'
local ximea_ros = require 'ximea_ros'

local cv = require 'cv'
require 'cv.imgcodecs'
require 'cv.imgproc'
require 'cv.calib3d'

local XI_RET,XI_RET_TEXT = ximea.XI_RET, ximea.XI_RET_TEXT

local XimeaRosCam = torch.class('ximea_ros.XimeaRosCam', ximea_ros)
local CaptureMode = {
  continuous = 0,
  triggered = 1
}

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


local function escapeRosName(s)
  -- add undersore if s starts with number
  if s:match('^[0-9]') then
    s = '_' .. s
  end
  -- replace all non alpha-numeric chars with safe char
  return string.gsub(s, "([^A-Za-z0-9_])", '_')
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
  self.camera_topic = nh:advertise('SN_' .. escapeRosName(self.serial), ximea_ros.image_spec, 1, false)
  self.flags = flags or DEFAULT_FLAGS

  if self.flags.enableFPNCorrection then
    print('Enabling PFN correction')
    XI_CHECK(self.camera:setParamInt(ximea.PARAM.XI_PRM_COLUMN_FPN_CORRECTION, 1))
  end

  if self.flags.autoWhiteBalance and (mode == 'RGB24' or mode == 'RGB32') then
    print('Enabling auto white-balance')
    XI_CHECK(self.camera:setParamInt(ximea.PARAM.XI_PRM_AUTO_WB, 1))
  end

  print('Start camera acquisition')
  XI_CHECK(self.camera:startAcquisition())

  self.state = {
    mode = CaptureMode.continuous,
    targetFrameCount = 0,
    frames = {}
  }
end


function XimeaRosCam:hasSubscribers()
  return self.camera_topic ~= nil and self.camera_topic:getNumSubscribers() > 0
end


function XimeaRosCam:capture(hardwareTriggered, timeout)
  hardwareTriggered = hardwareTriggered or false
  timeout = timeout or 1000
  local img = self.camera:getImage(hardwareTriggered, timeout)
  if img == nil then
    local b = -1
    if hardwareTriggered == true then
      b = 1
    elseif hardwareTriggered == false then
      b = 0
    end
    ros.WARN('Capturing image failed (cam serial: %s, hw trigger %d, timeout %d)', self.serial, b, timeout or -1)
    return nil
  end

  local rosMessage
  if hardwareTriggered then
    --img = cv.cvtColor{img, code = cv.COLOR_BGR2GRAY}
    rosMessage = ximea_ros.createImageMessage(img, self.serial, self.camera:getColorMode())
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


function XimeaRosCam:getState()
  return self.state
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
  local XI_GPI_TRIGGER = 1
  local camera = self.camera
  camera:stopAcquisition()
  camera:setParamInt("gpi_selector", 1)
  camera:setParamInt("gpi_mode", XI_GPI_TRIGGER)
  camera:setParamInt("trigger_source", XI_TRG_EDGE_RISING)
  --camera:setParamInt("acq_buffer_size", 6.5 * numberOfFrames + 50)
  camera:setParamInt("buffers_queue_size", numberOfFrames + 1)
  camera:setParamInt("recent_frame", 0)
  print("[XimeaRosCam:hwTrigger] set exposure to " .. exposureTimeInMicroSeconds)
  self:setExposure(exposureTimeInMicroSeconds)
  camera:startAcquisition()
  self.state = {
    mode = CaptureMode.triggered,
    targetFrameCount = numberOfFrames,
    frames = {}
  }
end


function XimeaRosCam:stopTrigger()
  -- Stop triggering and restore buffer settings
  local XI_TRG_SOFTWARE = 3
  print("[XimeaRosCam:hwTrigger] hold triggering")
  local camera = self.camera
  camera:stopAcquisition()
  camera:setParamInt("trigger_source", XI_TRG_SOFTWARE)
  --camera:setParamInt("acq_buffer_size", 50)
  camera:setParamInt("buffers_queue_size", 4)
  camera:setParamInt("recent_frame", 1)
  camera:startAcquisition()
  self.state = {
    mode = CaptureMode.continuous,
    targetFrameCount = 0,
    frames = {}
  }
end


function XimeaRosCam:hardwareTriggeredCapture(numberOfFrames, exposureTimeInMicroSeconds)
  local t = torch.Timer()

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

    --print("[XimeaRosCam:hwTrigger] wait for s:", waitForSeconds)
    --sys.sleep(waitForSeconds)
    local imageMessage = self:capture(false)
    if i > 20 then -- fringe patterns are shown 133ms, because it takes the projector 400ms to load the next image
      sys.sleep(waitForSeconds)
    end

    if imageMessage then
      table.insert(frames, imageMessage)
    else
      print("[XimeaRosCam:hwTrigger] ERR: missed frame!")
    end

    print ("Retrieved " .. i .. " in " .. t:time().real - last)
    last = t:time().real
  end

  --camera:stopAcquisition()
  --camera:setParamInt("trigger_source", XI_TRG_SOFTWARE)
  --camera:startAcquisition()

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


function XimeaRosCam:checkForNewFrame()
  ros.INFO('Check for frame')
  local imageMessage = self:capture(true, 10)
  if imageMessage and self.state.frames ~= nil then
    table.insert(self.state.frames, imageMessage)
    ros.INFO('Received frame #%d', #self.state.frames)
  end
end