-- Tuya EF00 (MCU) cluster handlers for AOYAN AY204T and similar devices
-- Licensed under the Apache License, Version 2.0

local messages = require "st.zigbee.messages"
local zcl_messages = require "st.zigbee.zcl"
local generic_body = require "st.zigbee.generic_body"
local data_types = require "st.zigbee.data_types"
local constants = require "st.zigbee.constants"
local log = require "log"

local TUYA_CLUSTER = 0xEF00
local CMD_MCU_SYNC_TIME = 0x24
local DP_TYPE_VALUE = 0x02 -- 4 byte signed integer

-- AOYAN AY204T sends every value twice: via EF00 and via standard ZCL clusters
-- (IASZone, 0x0402, 0x0405, 0x0400). Standard clusters are authoritative, so EF00
-- DPs are only logged. Known DPs: 1=presence, 101=humidity %, 106=lux, 111=temp x10

-- decode big-endian unsigned integer, then apply sign for VALUE type
local function decode_value(dp_type, raw)
  local val = 0
  for i = 1, #raw do
    val = (val << 8) | raw:byte(i)
  end
  if dp_type == DP_TYPE_VALUE and #raw == 4 and val >= 0x80000000 then
    val = val - 0x100000000
  end
  return val
end

-- payload: seq(2) followed by one or more [dp(1) type(1) len(2) data(len)]
local function data_report_handler(driver, device, zb_rx)
  local body = zb_rx.body.zcl_body.body_bytes
  local pos = 3
  while pos + 3 <= #body do
    local dp = body:byte(pos)
    local dp_type = body:byte(pos + 1)
    local len = string.unpack(">I2", body, pos + 2)
    if pos + 3 + len > #body then
      log.warn(string.format("<<< EF00 truncated DP=%d len=%d >>>", dp, len))
      return
    end
    local val = decode_value(dp_type, body:sub(pos + 4, pos + 3 + len))
    if dp == 2 or dp == 102 then -- AOYAN sensitivity / keep time: echo after a write
      log.info(string.format("<<< EF00 setting DP=%d val=%d >>>", dp, val))
    else
      log.debug(string.format("<<< EF00 DP=%d type=%d len=%d val=%d (ignored) >>>", dp, dp_type, len, val))
    end
    pos = pos + 4 + len
  end
end

local function send_tuya_command(device, cmd, payload)
  local zclh = zcl_messages.ZclHeader({ cmd = data_types.ZCLCommandId(cmd) })
  zclh.frame_ctrl:set_cluster_specific()
  zclh.frame_ctrl:set_disable_default_response()
  local addrh = messages.AddressHeader(
    constants.HUB.ADDR,
    constants.HUB.ENDPOINT,
    device:get_short_address(),
    device:get_endpoint(TUYA_CLUSTER),
    constants.HA_PROFILE_ID,
    TUYA_CLUSTER
  )
  local message_body = zcl_messages.ZclMessageBody({
    zcl_header = zclh,
    zcl_body = generic_body.GenericBody(payload)
  })
  device:send(messages.ZigbeeMessageTx({ address_header = addrh, body = message_body }))
end

local CMD_DATA_REQUEST = 0x00
local seq = 0

-- write a VALUE (4 byte) datapoint: seq(2) dp(1) type(1) len(2) value(4)
local function set_dp_value(device, dp, value)
  seq = (seq + 1) % 0x10000
  log.info(string.format("<<< EF00 set DP=%d val=%d >>>", dp, value))
  send_tuya_command(device, CMD_DATA_REQUEST, string.pack(">I2BBI2i4", seq, dp, DP_TYPE_VALUE, 4, value))
end

-- device asks for time: reply with payload size(2) + UTC(4) + local time(4)
local function time_sync_handler(driver, device, zb_rx)
  local utc = os.time()
  local tz_offset = os.time(os.date("*t", utc)) - os.time(os.date("!*t", utc))
  local payload = string.pack(">I2I4I4", 8, utc, utc + tz_offset)
  log.info("<<< EF00 time sync response >>>")
  send_tuya_command(device, CMD_MCU_SYNC_TIME, payload)
end

return {
  data_report_handler = data_report_handler,
  time_sync_handler = time_sync_handler,
  set_dp_value = set_dp_value,
  DP_AOYAN_SENSITIVITY = 2,   -- 0-19
  DP_AOYAN_FADING_TIME = 102, -- seconds, 0-28800
}
