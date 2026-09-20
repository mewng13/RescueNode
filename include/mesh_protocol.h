// Wire format for the rescue-node mesh.
//
// Nodes broadcast one node_report_t per reading over ESP-NOW. Every node
// rebroadcasts reports it has not seen before (a flood, deduplicated on
// origin_id + seq) until the hop limit runs out, so a report finds its way to
// the gateway without anyone maintaining a routing table -- which matters when
// nodes are thrown into rubble and the topology is unknown and changing.
//
// The gateway is the one node on a USB cable. It does not rebroadcast; it turns
// each report into a line of JSON for the computer. Two links, two encodings,
// for two different reasons:
//
//   node -> node    40 packed bytes.  Radio airtime is the scarce thing here:
//                   ESP-NOW carries 250 bytes per packet, and every byte on air
//                   costs battery and raises the odds of a collision. The same
//                   reading as JSON is ~230 bytes -- one node would fill a
//                   packet by itself.
//
//   gateway -> PC   JSON. Native USB CDC runs at ~1 MB/s, so the bytes are free,
//                   and a stream you can read with your own eyes is worth a lot
//                   more than the bandwidth it costs.
//
// Both ends of the radio link are ESP32-S3 built by the same compiler, so the
// usual objections to shipping a raw struct (padding, endianness) do not apply.
// `version` still comes first so a node running old firmware can recognise a
// packet it does not understand instead of misreading it.
#pragma once

#include <stdint.h>
#include <stdio.h>
#include <math.h>

#define MESH_PROTO_VERSION  1
#define MESH_CHANNEL        1     // every node must agree; ESP-NOW does not scan
#define MESH_MAX_HOPS       4     // flood radius, keeps a lost packet from living forever
#define MESH_MAX_NODES     32     // sizes the dedup table; ids run 1..MESH_MAX_NODES

// flags
#define MESH_F_GAS        (1 << 0)   // MQ digital pin tripped
#define MESH_F_STILL      (1 << 1)   // IMU sees no movement
#define MESH_F_GYRO_CLIP  (1 << 2)   // gyro hit full scale, orientation under-integrated
#define MESH_F_IMU_READY  (1 << 3)   // past boot calibration
#define MESH_F_DHT_FAIL   (1 << 4)   // temp/humidity fields are meaningless
#define MESH_F_GATEWAY    (1 << 5)   // sender is the USB-attached node

// Fixed point. Each scale is chosen so the widest reading the sensor can
// produce still fits in an int16, and the resolution stays below the noise
// floor -- there is no point spending bits on digits the part cannot measure.
#define MESH_Q_SCALE      32767.0f   // quaternion components live in [-1, 1]
#define MESH_ACCEL_SCALE   4096.0f   // +/-8 g, 0.00024 g per count
#define MESH_GYRO_SCALE      16.0f   // +/-2048 deg/s, 0.06 deg/s per count
#define MESH_TEMP_SCALE      10.0f   // 0.1 C, finer than a DHT11 can resolve

#define MESH_TEMP_INVALID   INT16_MIN
#define MESH_HUM_INVALID    0xFF

// 38 bytes. Field order keeps every multi-byte member naturally aligned while
// still leading with `version`.
typedef struct __attribute__((packed)) {
  uint8_t  version;      //  0
  uint8_t  origin_id;    //  1  who measured this, NOT who last relayed it
  uint16_t seq;          //  2  per-origin counter: dedup, and gap = packet loss
  uint8_t  hops;         //  4  incremented by each relay
  uint8_t  flags;        //  5
  uint16_t volume;       //  6  microphone peak-to-peak
  uint32_t uptime_ms;    //  8  millis() at the origin; nodes share no clock
  int16_t  q[4];         // 12  fused orientation, w x y z
  int16_t  accel[3];     // 20
  int16_t  gyro[3];      // 26
  uint16_t gas_raw;      // 32  MQ ADC counts, 0..4095
  int16_t  temp_c10;     // 34
  uint16_t gas_mv;       // 36  calibrated millivolts; the ESP32 ADC is not linear,
                         //     so raw counts alone cannot be converted back
  uint8_t  humidity;     // 38  whole percent
  uint8_t  reserved;     // 39  spend it on battery level
} node_report_t;

#ifdef __cplusplus
static_assert(sizeof(node_report_t) == 40, "node_report_t must stay 40 bytes on the wire");
#endif

// Saturate rather than wrap. A reading past full scale is wrong either way, but
// wrapping turns "very hot" into "very cold", which is the more dangerous lie.
static inline int16_t mesh_q15(float v, float scale) {
  if (isnan(v)) return 0;
  float s = v * scale;
  if (s >  32767.0f) return  32767;
  if (s < -32768.0f) return -32768;
  return (int16_t)lrintf(s);
}

// ---------------------------------------------------------------- dedup
// One slot per node id: the last sequence number seen from it. The signed
// difference handles seq wrapping past 65535 without a special case.
typedef struct {
  uint16_t last_seq[MESH_MAX_NODES + 1];
  uint8_t  seen[MESH_MAX_NODES + 1];
} mesh_seen_t;

static inline bool mesh_accept(mesh_seen_t *t, const node_report_t *r) {
  if (r->version != MESH_PROTO_VERSION) return false;
  if (r->origin_id == 0 || r->origin_id > MESH_MAX_NODES) return false;
  if (t->seen[r->origin_id] && (int16_t)(r->seq - t->last_seq[r->origin_id]) <= 0)
    return false;
  t->seen[r->origin_id] = 1;
  t->last_seq[r->origin_id] = r->seq;
  return true;
}

// ---------------------------------------------------------------- JSON
// Used only by the gateway, on the USB side. Keys match what dashboard.py and
// the browser already expect, so a mesh reading and a local one look the same.
static inline int mesh_report_to_json(const node_report_t *r, char *buf, size_t n) {
  char temp[16], hum[16];
  if ((r->flags & MESH_F_DHT_FAIL) || r->temp_c10 == MESH_TEMP_INVALID)
    snprintf(temp, sizeof(temp), "null");
  else
    snprintf(temp, sizeof(temp), "%.1f", r->temp_c10 / MESH_TEMP_SCALE);
  if ((r->flags & MESH_F_DHT_FAIL) || r->humidity == MESH_HUM_INVALID)
    snprintf(hum, sizeof(hum), "null");
  else
    snprintf(hum, sizeof(hum), "%u", r->humidity);

  return snprintf(buf, n,
    "{\"node\":%u,\"seq\":%u,\"hops\":%u,\"up\":%lu,"
    "\"qw\":%.4f,\"qx\":%.4f,\"qy\":%.4f,\"qz\":%.4f,"
    "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
    "\"gx\":%.1f,\"gy\":%.1f,\"gz\":%.1f,"
    "\"volume\":%u,\"gas_raw\":%u,\"gas_volts\":%.2f,\"gas\":%s,"
    "\"temp\":%s,\"humidity\":%s,"
    "\"still\":%s,\"clipped\":%s,\"gateway\":%s}",
    r->origin_id, r->seq, r->hops, (unsigned long)r->uptime_ms,
    r->q[0] / MESH_Q_SCALE, r->q[1] / MESH_Q_SCALE,
    r->q[2] / MESH_Q_SCALE, r->q[3] / MESH_Q_SCALE,
    r->accel[0] / MESH_ACCEL_SCALE, r->accel[1] / MESH_ACCEL_SCALE,
    r->accel[2] / MESH_ACCEL_SCALE,
    r->gyro[0] / MESH_GYRO_SCALE, r->gyro[1] / MESH_GYRO_SCALE,
    r->gyro[2] / MESH_GYRO_SCALE,
    r->volume, r->gas_raw, r->gas_mv / 1000.0f,
    (r->flags & MESH_F_GAS) ? "true" : "false",
    temp, hum,
    (r->flags & MESH_F_STILL) ? "true" : "false",
    (r->flags & MESH_F_GYRO_CLIP) ? "true" : "false",
    (r->flags & MESH_F_GATEWAY) ? "true" : "false");
}
