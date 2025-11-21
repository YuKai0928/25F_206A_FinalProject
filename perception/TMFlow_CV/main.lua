-- PSEUDOCODE on the robot side (TMflow Script)

local ip = "TODO !Actual IP!"    -- Your PC IP
local port = 5000             -- Port you’ll listen on in Python

-- Capture image from camera (TM-specific API)
local img = Vision_Snap()     -- or similar API in TMflow

-- Convert image to JPEG buffer (API may differ)
local jpg_data = Image_ToJpeg(img, 80)  -- quality 80, for example

-- Open TCP socket and send data length + data
local sock = Socket_Connect(ip, port)
local len = string.len(jpg_data)

-- Send 4-byte length header (big-endian)
Socket_Send(sock, IntToBytes(len))
Socket_Send(sock, jpg_data)
Socket_Close(sock)