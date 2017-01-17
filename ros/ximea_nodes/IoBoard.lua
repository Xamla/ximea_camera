local posix = require "posix"

local IoBoard = {}
local fileDescriptor = nil
local boardConnected = nil

local SET_TRIGGER_HIGH = "io0.trigger = 1"
local SET_TRIGGER_LOW = "io0.trigger = 0"
local ACTIVATE_SQUARE_FREQUENCY = "io0.trigger = sim0.square"
local SET_SQUARE_FREQUENCY = "sim0.freq = "

IoBoard.LOW = false
IoBoard.HIGH = true

-- This function searches for devices in order to get possible paths for read/write
-- Input:
--  id_vendor - The vendor id of the desired device
--  id_prdocut - The product id of the desired device
-- Output:
--  found - Table with all connected devices (e.g. {"/dev/ttyACM0", "/dev/ttyACM1"}
local function findDevice(id_vendor, id_product)
  local function read_first_line(fn)
    local file = io.open(fn, 'rb')
    local ln
    if file ~= nil then
      ln = file:read('*l')
      file:close()
    end
    return ln
  end

  local function startsWith(string, start)
    return string.sub(string, 1, #start) == start
  end

  local found = {}
  local USB_DEVICES = '/sys/bus/usb/devices'
  for dnbase in path.dir(USB_DEVICES) do
    local dn = path.join(USB_DEVICES, dnbase)
    local vid_fn = path.join(dn, 'idVendor')
    local pid_fn = path.join(dn, 'idProduct')

    if not path.exists(vid_fn) or not path.exists(pid_fn) or
      id_vendor ~= read_first_line(vid_fn) or id_product ~= read_first_line(pid_fn) then
      goto _continue
    end

    for subdir in path.dir(dn) do
      if startsWith(subdir, dnbase .. ':') then
        for subsubdir in path.dir(path.join(dn, subdir)) do
          if startsWith(subsubdir, 'tty') then
            for subsubsubdir in path.dir(path.join(dn, subdir, subsubdir)) do
              if startsWith(subsubsubdir, 'ttyACM') then
                table.insert(found, path.join('/dev', subsubsubdir))
              end
            end
          end
        end
      end
    end

    ::_continue::
  end
  return found
end


local function open(filename, error_on_read_write)
  local fd, err = posix.open(filename, posix.O_RDWR + posix.O_NONBLOCK)
  if err ~= nil then
    error("While opening device! Error: " .. err)
  end

  posix.tcsetattr(fd, 0, {
    cflag = posix.B115200 + posix.CS8 + posix.HUPCL + posix.CREAD,
    iflag = posix.IGNPAR,
    cc = {
      [posix.VTIME] = 0,
      [posix.VMIN] = 1
    }
  })

  return fd
end


function IoBoard.activateSquareSignal()
  return IoBoard.writeLine(ACTIVATE_SQUARE_FREQUENCY)
end


function IoBoard.close()
  if boardConnected then
    boardConnected = nil
    posix.close(fileDescriptor)
  end
end


function IoBoard.findIoBoard()
  local vendor_id = "0483"
  local product_id = "5740"
  return findDevice(vendor_id, product_id)
end


function IoBoard.open()
  local found_usb_ports = IoBoard.findIoBoard()
  if #found_usb_ports == 0 then
    error('IO board not found.')
  else
    print("[IoBoard.open] found ioboard @ " .. found_usb_ports[1])
    fileDescriptor = open(found_usb_ports[1])
    boardConnected = true
  end

  return boardConnected
end


function IoBoard.setSquareFrequency(frequency)
  local ok
  if type(frequency) == "number" then
    ok = IoBoard.writeLine(SET_SQUARE_FREQUENCY .. frequency)
  end

  return ok
end


function IoBoard.setTrigger(high)
  local ok
  if high == true then
    ok = IoBoard.writeLine(SET_TRIGGER_HIGH)
  else
    ok = IoBoard.writeLine(SET_TRIGGER_LOW)
  end

  return ok
end


function IoBoard.trigger()
  IoBoard.setTrigger(true)
  IoBoard.setTrigger()
end


function IoBoard.writeLine(line)
  local ok
  if boardConnected then
    local err
    ok, err = posix.write(fileDescriptor, line .. "\n")
    if not ok then
      print("[IoBoard.writeLine] ERR: could not write to IoBoard: " .. err)
      boardConnected = nil
    end
  else
    print("[IoBoard.writeLine] ERR: IoBoard not connected. Call open() first.")
  end

  return ok
end


return IoBoard
