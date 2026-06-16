'use strict';

// ---------------------------------------------------------------------------
// Minimal MAVLink v1 / v2 parser
// Handles ATTITUDE_QUATERNION (msg 31) and ATTITUDE (msg 30)
// Builds SET_MESSAGE_INTERVAL (cmd 511) and REQUEST_DATA_STREAM packets
// ---------------------------------------------------------------------------

const MAV_STX_V1 = 0xFE;
const MAV_STX_V2 = 0xFD;

const MSG_ATTITUDE              = 30;
const MSG_ATTITUDE_QUATERNION   = 31;
const MSG_GPS_RAW_INT           = 24;
const MSG_GPS2_RAW              = 124;
const MSG_COMMAND_LONG          = 76;
const MSG_REQUEST_DATA_STREAM   = 66;
const MSG_STATUSTEXT            = 253;
const MSG_PARAM_SET             = 23;
const MSG_PARAM_REQUEST_READ    = 20;
const MSG_PARAM_VALUE           = 22;

// CRC extra bytes for MAVLink 1 CRC_EXTRA
const CRC_EXTRA = {
  [MSG_ATTITUDE]:            39,
  [MSG_ATTITUDE_QUATERNION]: 246,
  [MSG_GPS_RAW_INT]:         24,
  [MSG_GPS2_RAW]:            87,
  [MSG_COMMAND_LONG]:        152,
  [MSG_REQUEST_DATA_STREAM]: 148,
  [MSG_STATUSTEXT]:          89,
  [MSG_PARAM_SET]:           168,
  [MSG_PARAM_REQUEST_READ]:  214,
  [MSG_PARAM_VALUE]:         220,
};

// ---------------------------------------------------------------------------
// X.25 CRC
// ---------------------------------------------------------------------------
function crcInit() { return 0xFFFF; }

function crcAccumulate(crc, byte) {
  const tmp = (byte ^ (crc & 0xFF)) & 0xFF;
  const tmp2 = (tmp ^ ((tmp << 4) & 0xFF)) & 0xFF;
  return ((crc >> 8) ^ (tmp2 << 8) ^ (tmp2 << 3) ^ (tmp2 >> 4)) & 0xFFFF;
}

function crcBuffer(buf, start, len) {
  let crc = crcInit();
  for (let i = start; i < start + len; i++) {
    crc = crcAccumulate(crc, buf[i]);
  }
  return crc;
}

// ---------------------------------------------------------------------------
// MAVLink v1 frame builder
// header: [len, seq, sysid, compid, msgid]
// payload: Buffer
// ---------------------------------------------------------------------------
function buildV1Frame(seq, sysId, compId, msgId, payload) {
  const len = payload.length;
  const frame = Buffer.alloc(6 + len + 2);
  frame[0] = MAV_STX_V1;
  frame[1] = len;
  frame[2] = seq & 0xFF;
  frame[3] = sysId & 0xFF;
  frame[4] = compId & 0xFF;
  frame[5] = msgId & 0xFF;
  payload.copy(frame, 6);

  // CRC over bytes 1..5+len (excluding STX)
  let crc = crcInit();
  for (let i = 1; i < 6 + len; i++) {
    crc = crcAccumulate(crc, frame[i]);
  }
  // CRC_EXTRA
  const extra = CRC_EXTRA[msgId];
  if (extra !== undefined) {
    crc = crcAccumulate(crc, extra);
  }
  frame[6 + len]     = crc & 0xFF;
  frame[6 + len + 1] = (crc >> 8) & 0xFF;
  return frame;
}

// ---------------------------------------------------------------------------
// SET_MESSAGE_INTERVAL (COMMAND_LONG, cmd 511)
// Request msg_id at interval_us microseconds
// ---------------------------------------------------------------------------
function buildSetMessageInterval(seq, sysId, compId, targetSys, targetComp, msgId, intervalUs) {
  // COMMAND_LONG payload: float[7] params + uint16 cmd + uint8 target_system + uint8 target_component + uint8 confirmation
  // Layout (37 bytes total):
  //   param1..7 : float32 each (28 bytes)
  //   command   : uint16 (2 bytes)
  //   target_sys: uint8
  //   target_cmp: uint8
  //   confirm   : uint8
  // MAVLink COMMAND_LONG (msg 76) field order per spec:
  //   param1(float) param2 param3 param4 param5 param6 param7 command(u16) target_system(u8) target_component(u8) confirmation(u8)
  const payload = Buffer.alloc(33);
  let off = 0;
  payload.writeFloatLE(msgId, off);       off += 4; // param1 = message ID
  payload.writeFloatLE(intervalUs, off);  off += 4; // param2 = interval µs (-1 = disable, 0 = default)
  payload.writeFloatLE(0, off);           off += 4; // param3
  payload.writeFloatLE(0, off);           off += 4; // param4
  payload.writeFloatLE(0, off);           off += 4; // param5
  payload.writeFloatLE(0, off);           off += 4; // param6
  payload.writeFloatLE(0, off);           off += 4; // param7
  payload.writeUInt16LE(511, off);        off += 2; // command = MAV_CMD_SET_MESSAGE_INTERVAL
  payload.writeUInt8(targetSys, off);     off += 1;
  payload.writeUInt8(targetComp, off);    off += 1;
  payload.writeUInt8(0, off);             off += 1; // confirmation
  return buildV1Frame(seq, sysId, compId, MSG_COMMAND_LONG, payload);
}

// ---------------------------------------------------------------------------
// REQUEST_DATA_STREAM (legacy fallback)
// stream_id 10 = EXTRA1 (attitude)
// ---------------------------------------------------------------------------
function buildRequestDataStream(seq, sysId, compId, targetSys, targetComp, streamId, rateHz, startStop) {
  // Fields: target_system(u8) target_component(u8) req_stream_id(u8) req_message_rate(u16) start_stop(u8)
  const payload = Buffer.alloc(6);
  payload.writeUInt16LE(rateHz, 0);       // req_message_rate
  payload.writeUInt8(targetSys, 2);
  payload.writeUInt8(targetComp, 3);
  payload.writeUInt8(streamId, 4);
  payload.writeUInt8(startStop, 5);
  return buildV1Frame(seq, sysId, compId, MSG_REQUEST_DATA_STREAM, payload);
}

// ---------------------------------------------------------------------------
// Parser state machine
// ---------------------------------------------------------------------------
class MAVLinkParser {
  constructor(onMessage) {
    this._cb = onMessage;
    this._buf = Buffer.alloc(0);
  }

  push(chunk) {
    this._buf = Buffer.concat([this._buf, chunk]);
    this._parse();
  }

  _parse() {
    while (this._buf.length > 0) {
      // Find STX
      let stxIdx = -1;
      for (let i = 0; i < this._buf.length; i++) {
        if (this._buf[i] === MAV_STX_V1 || this._buf[i] === MAV_STX_V2) {
          stxIdx = i;
          break;
        }
      }
      if (stxIdx === -1) {
        this._buf = Buffer.alloc(0);
        return;
      }
      if (stxIdx > 0) {
        this._buf = this._buf.slice(stxIdx);
      }

      const stx = this._buf[0];

      if (stx === MAV_STX_V1) {
        // Need at least 6 bytes for header
        if (this._buf.length < 6) return;
        const payloadLen = this._buf[1];
        const totalLen = 6 + payloadLen + 2; // STX+len+seq+sysid+compid+msgid + payload + crc(2)
        if (this._buf.length < totalLen) return;

        const frame = this._buf.slice(0, totalLen);
        this._buf = this._buf.slice(totalLen);

        const msgId = frame[5];
        const payload = frame.slice(6, 6 + payloadLen);
        const crcFrame = (frame[6 + payloadLen] | (frame[6 + payloadLen + 1] << 8));

        // Verify CRC
        let crc = crcInit();
        for (let i = 1; i < 6 + payloadLen; i++) {
          crc = crcAccumulate(crc, frame[i]);
        }
        const extra = CRC_EXTRA[msgId];
        if (extra !== undefined) {
          crc = crcAccumulate(crc, extra);
        }
        if ((crc & 0xFFFF) !== crcFrame) {
          // CRC mismatch — skip this byte and retry
          this._buf = Buffer.concat([frame.slice(1), this._buf]);
          continue;
        }

        const sysId  = frame[3];
        const compId = frame[4];
        const seq    = frame[2];
        this._dispatchMessage(sysId, compId, seq, msgId, payload, 1);

      } else if (stx === MAV_STX_V2) {
        // MAVLink v2: STX(1) LEN(1) INCOMPAT(1) COMPAT(1) SEQ(1) SYSID(1) COMPID(1) MSGID(3) payload CRC(2)
        if (this._buf.length < 10) return;
        const payloadLen  = this._buf[1];
        const incompat    = this._buf[2];
        // Signature flag
        const sigLen      = (incompat & 0x01) ? 13 : 0;
        const totalLen    = 10 + payloadLen + 2 + sigLen;
        if (this._buf.length < totalLen) return;

        const frame = this._buf.slice(0, totalLen);
        this._buf = this._buf.slice(totalLen);

        const msgId = frame[7] | (frame[8] << 8) | (frame[9] << 16);
        const payload = frame.slice(10, 10 + payloadLen);
        const crcFrame = (frame[10 + payloadLen] | (frame[10 + payloadLen + 1] << 8));

        // Verify CRC
        let crc = crcInit();
        for (let i = 1; i < 10 + payloadLen; i++) {
          crc = crcAccumulate(crc, frame[i]);
        }
        const extra = CRC_EXTRA[msgId];
        if (extra !== undefined) {
          crc = crcAccumulate(crc, extra);
        }
        if ((crc & 0xFFFF) !== crcFrame) {
          this._buf = Buffer.concat([frame.slice(1), this._buf]);
          continue;
        }

        const sysId  = frame[5];
        const compId = frame[6];
        const seq    = frame[4];
        this._dispatchMessage(sysId, compId, seq, msgId, payload, 2);
      } else {
        // Should not happen — skip byte
        this._buf = this._buf.slice(1);
      }
    }
  }

  _dispatchMessage(sysId, compId, seq, msgId, payload, mavVersion) {
    try {
      if (msgId === MSG_ATTITUDE_QUATERNION && payload.length >= 32) {
        // Fields (MAVLink spec):
        // time_boot_ms(u32) q1(f32) q2(f32) q3(f32) q4(f32) rollspeed(f32) pitchspeed(f32) yawspeed(f32)
        // q1=w, q2=x, q3=y, q4=z
        const timeBootMs = payload.readUInt32LE(0);
        const q1 = payload.readFloatLE(4);  // w
        const q2 = payload.readFloatLE(8);  // x
        const q3 = payload.readFloatLE(12); // y
        const q4 = payload.readFloatLE(16); // z
        this._cb({
          msgId,
          sysId,
          compId,
          seq,
          timeBootMs,
          w: q1, x: q2, y: q3, z: q4,
          source: 'ATTITUDE_QUATERNION',
        });

      } else if ((msgId === MSG_GPS_RAW_INT || msgId === MSG_GPS2_RAW) && payload.length >= 30) {
        // Wire layout (v1 sorted by type size):
        // time_usec(uint64,0) lat(int32,8) lon(int32,12) alt(int32,16)
        // eph(uint16,20) epv(uint16,22) vel(uint16,24) cog(uint16,26)
        // fix_type(uint8,28) satellites_visible(uint8,29)
        const fixType = payload[28];
        const sats    = payload[29] === 255 ? null : payload[29]; // 255 = unknown
        const source  = msgId === MSG_GPS_RAW_INT ? 'GPS_RAW_INT' : 'GPS2_RAW';
        this._cb({ msgId, sysId, compId, seq, fixType, sats, source });

      } else if (msgId === MSG_PARAM_VALUE && payload.length >= 25) {
        // Wire layout: param_value(float,0) param_count(u16,4) param_index(u16,6) param_id(char[16],8) param_type(u8,24)
        const value = payload.readFloatLE(0);
        let paramId = '';
        for (let i = 8; i < 24; i++) {
          if (payload[i] === 0) break;
          paramId += String.fromCharCode(payload[i]);
        }
        this._cb({ msgId, sysId, compId, seq, paramId, value, source: 'PARAM_VALUE' });

      } else if (msgId === MSG_STATUSTEXT && payload.length >= 1) {
        // severity(uint8, 0) + text(char[50], 1..50)
        const severity = payload[0];
        let text = '';
        const end = Math.min(payload.length, 51);
        for (let i = 1; i < end; i++) {
          if (payload[i] === 0) break;
          text += String.fromCharCode(payload[i]);
        }
        this._cb({ msgId, sysId, compId, seq, severity, text, source: 'STATUSTEXT' });

      } else if (msgId === MSG_ATTITUDE && payload.length >= 28) {
        // Fields: time_boot_ms(u32) roll(f32) pitch(f32) yaw(f32) rollspeed(f32) pitchspeed(f32) yawspeed(f32)
        // Convert Euler to quaternion
        const timeBootMs = payload.readUInt32LE(0);
        const roll  = payload.readFloatLE(4);
        const pitch = payload.readFloatLE(8);
        const yaw   = payload.readFloatLE(12);
        const { w, x, y, z } = eulerToQuat(roll, pitch, yaw);
        this._cb({
          msgId,
          sysId,
          compId,
          seq,
          timeBootMs,
          w, x, y, z,
          source: 'ATTITUDE',
        });
      }
    } catch (e) {
      // Malformed payload — ignore
    }
  }
}

// ---------------------------------------------------------------------------
// PARAM_REQUEST_READ (msg 20) — request a single parameter by name
// Wire layout (20 bytes):
//   param_index(int16, 0)  — set to -1 to use name instead of index
//   target_system(u8, 2)   target_component(u8, 3)   param_id(char[16], 4)
// ---------------------------------------------------------------------------
function buildParamRequestRead(seq, sysId, compId, targetSys, targetComp, paramId) {
  const payload = Buffer.alloc(20, 0);
  payload.writeInt16LE(-1, 0);         // -1 = look up by name
  payload.writeUInt8(targetSys, 2);
  payload.writeUInt8(targetComp, 3);
  const id = String(paramId).slice(0, 16);
  for (let i = 0; i < id.length; i++) payload.writeUInt8(id.charCodeAt(i), 4 + i);
  return buildV1Frame(seq, sysId, compId, MSG_PARAM_REQUEST_READ, payload);
}

// ---------------------------------------------------------------------------
// PARAM_SET (msg 23) — set a named float parameter on a target system
// Wire layout (23 bytes, MAVLink 1 sort order — float first, then uint8s, then char[]):
//   param_value(float32, 0)  target_system(u8, 4)  target_component(u8, 5)
//   param_id(char[16], 6)    param_type(u8, 22)
// ---------------------------------------------------------------------------
function buildParamSet(seq, sysId, compId, targetSys, targetComp, paramId, value) {
  const payload = Buffer.alloc(23, 0);
  payload.writeFloatLE(value, 0);
  payload.writeUInt8(targetSys, 4);
  payload.writeUInt8(targetComp, 5);
  const id = String(paramId).slice(0, 16);
  for (let i = 0; i < id.length; i++) payload.writeUInt8(id.charCodeAt(i), 6 + i);
  payload.writeUInt8(9, 22); // MAV_PARAM_TYPE_REAL32 — ArduPilot stores all params as float
  return buildV1Frame(seq, sysId, compId, MSG_PARAM_SET, payload);
}

// ---------------------------------------------------------------------------
// Euler ZYX → quaternion  (intrinsic roll-pitch-yaw)
// ---------------------------------------------------------------------------
function eulerToQuat(roll, pitch, yaw) {
  const cr = Math.cos(roll  * 0.5);
  const sr = Math.sin(roll  * 0.5);
  const cp = Math.cos(pitch * 0.5);
  const sp = Math.sin(pitch * 0.5);
  const cy = Math.cos(yaw   * 0.5);
  const sy = Math.sin(yaw   * 0.5);
  return {
    w:  cr * cp * cy + sr * sp * sy,
    x:  sr * cp * cy - cr * sp * sy,
    y:  cr * sp * cy + sr * cp * sy,
    z:  cr * cp * sy - sr * sp * cy,
  };
}

module.exports = {
  MAVLinkParser,
  buildSetMessageInterval,
  buildRequestDataStream,
  buildParamSet,
  buildParamRequestRead,
  MSG_ATTITUDE,
  MSG_ATTITUDE_QUATERNION,
  MSG_GPS_RAW_INT,
  MSG_GPS2_RAW,
  MSG_STATUSTEXT,
  MSG_PARAM_VALUE,
};
