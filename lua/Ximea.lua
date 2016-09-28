local ffi = require 'ffi'
local ximea = require 'ximea.env'

local XI_IMG_FORMAT = {
  MONO8               = ffi.C.XI_MONO8,                 -- 8 bits per pixel
  MONO16              = ffi.C.XI_MONO16,                -- 16 bits per pixel
  RGB24               = ffi.C.XI_RGB24,                 -- RGB data format
  RGB32               = ffi.C.XI_RGB32,                 -- RGBA data format
  RGB_PLANAR          = ffi.C.XI_RGB_PLANAR,            -- RGB planar data format
  RAW8                = ffi.C.XI_RAW8,                  -- 8 bits per pixel raw data from sensor
  RAW16               = ffi.C.XI_RAW16,                 -- 16 bits per pixel raw data from sensor
  FRM_TRANSPORT_DATA  = ffi.C.XI_FRM_TRANSPORT_DATA     -- Data from transport layer (e.g. packed). Format see XI_PRM_TRANSPORT_PIXEL_FORMAT
}
ximea.XI_IMG_FORMAT = XI_IMG_FORMAT

function ximea.getNCameras()
  local intPtr = ffi.typeof("int[1]")
  local n = intPtr()
  ximea.lib.getNumberConnectedDevices(n)
  return n[0]
end

function ximea.getSerialNumbers()
  local n = ximea.getNCameras()
  local serials = {}

  local c_str = ffi.new("char[32]")
  for i = 1,n do
    ximea.lib.getSerialNumber(i-1, c_str);
    table.insert(serials, ffi.string(c_str))
  end

  return serials
end

function ximea.getXiModeByName(mode_name)
  if type(mode_name) == 'number' then
    return mode_name
  end

  -- normalize mode name
  mode_name = string.upper(mode_name)
  if string.sub(mode_name, 1, 3) == 'XI_' then
    mode_name = string.sub(mode_name, 4)
  end

  -- lookup mode by name
  local mode = XI_IMG_FORMAT[mode_name]
  if mode == nil then
    error(string.format('Unknown Ximea camera mode "%s".', mode_name))
  end
  return mode
end
