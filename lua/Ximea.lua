local ffi = require 'ffi'
local ximea = require 'ximea.env'

function ximea.getNCameras()
 local intPtr = ffi.typeof"int[1]"
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

