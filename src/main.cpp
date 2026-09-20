#include <Arduino.h>
#include <DHT.h>
#include <Wire.h>
#include <driver/i2s.h>
#include <math.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "mesh_protocol.h"

// Build role, set per-environment in platformio.ini.
//   gateway: sits on the USB cable, prints JSON, never rebroadcasts
//   node:    broadcasts its readings and relays everyone else's
#ifndef MESH_ROLE_GATEWAY
#define MESH_ROLE_GATEWAY 0
#endif

#define MESH_REPORT_HZ      10   // radio rate at rest
#define MESH_REPORT_HZ_MOVE 25   // while the node is being disturbed
#define GATEWAY_LOCAL_HZ    50   // the gateway's own readings go over USB, which is cheap

#define DHT_PIN           4
#define DHT_TYPE          DHT11
#define led               5
#define MQ_AOUT           6
#define MQ_DOUT           7
#define MPU_ADDR          0x68   // I2C address of the MPU6500
#define SDA_PIN           8
#define SCL_PIN           9

// MPU6500 registers
#define REG_SMPLRT_DIV    0x19
#define REG_CONFIG        0x1A   // DLPF for the gyro
#define REG_GYRO_CONFIG   0x1B
#define REG_ACCEL_CONFIG  0x1C
#define REG_ACCEL_CONFIG2 0x1D   // DLPF for the accel
#define REG_ACCEL_XOUT    0x3B   // first of 14 bytes: accel[6] temp[2] gyro[6]
#define REG_PWR_MGMT_1    0x6B   // clears SLEEP bit to wake up the chip
#define REG_WHO_AM_I      0x75   // confirm communication

#define ACCEL_LSB_PER_G  16384.0f   // +/-2 g range
#define GYRO_LSB_PER_DPS    16.4f   // +/-2000 deg/s range. A flick of the wrist
                                    // passes 500 deg/s easily; once the gyro
                                    // pins at full scale the integrated angle
                                    // comes up short and, with no magnetic
                                    // reference, yaw keeps that error forever.
#define GYRO_CLIP_COUNTS   32000    // raw magnitude that counts as pinned

#define IMU_HZ            200    // fusion rate; the display only needs ~50, the
                                 // collapse detector wants every sample it can get
#define IMU_CAL_N         400    // ~2 s of stillness at boot to learn the gyro bias

#define I2S_WS            13
#define I2S_SCK           12
#define I2S_SD            11

// I2S Configuration
#define I2S_SAMPLE_RATE   16000
#define I2S_FRAMES        256    // 32-bit frames per i2s_read (1 KB of stack, ~16 ms of audio)
#define MIC_GAIN_SHIFT    13

DHT dht(DHT_PIN, DHT_TYPE);

static uint8_t  myNodeId = 0;
static uint16_t mySeq = 0;
static mesh_seen_t meshSeen;
static const uint8_t BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Latest reading from each slow sensor. loop() refreshes these on their own
// schedules; the report builder just takes whatever is current.
static int      lastGasRaw = 0;
static int      lastGasMv = 0;
static bool     lastGas = false;
static float    lastTemp = NAN, lastHum = NAN;
static int32_t  lastVolume = 0;

// Reports arriving off the radio are handed to loop() rather than printed from
// the Wi-Fi callback: Serial can block, and blocking there stalls the stack.
static QueueHandle_t meshRx = NULL;

void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t mpuRead8(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1);
  return Wire.read();
}

// accel[3], temp, gyro[3] in one burst: the six accel bytes and the six gyro
// bytes then describe the same instant, which matters once they are fused.
bool mpuBurst(int16_t out[7]) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, 14) != 14) return false;
  uint8_t b[14];
  for (int i = 0; i < 14; i++) b[i] = Wire.read();
  // Assemble each 16-bit value in two steps: the operands of `|` are
  // unsequenced in C++, so (Wire.read() << 8) | Wire.read() lets the compiler
  // fetch the low byte first and silently byte-swaps the result.
  for (int i = 0; i < 7; i++) out[i] = (int16_t)((b[2 * i] << 8) | b[2 * i + 1]);
  return true;
}

// -------------------------------------------------------------------- FUSION

// Shared with loop(). Written only by imuTask, read only by loop(), both under
// the spinlock so a print can never catch half of a quaternion.
struct Pose {
  float q[4];               // w x y z, rotates chip axes into world axes
  float ax, ay, az;         // g
  float gx, gy, gz;         // deg/s, bias removed
  float rest_g;             // |a| measured while still, ~1.00 on a good part
  uint32_t samples, errors, clipped;
  bool still, ready;
};
static Pose pose = {{1, 0, 0, 0}, NAN, NAN, NAN, 0, 0, 0, 1.0f, 0, 0, 0, true, false};
static portMUX_TYPE poseMux = portMUX_INITIALIZER_UNLOCKED;

// Mahony complementary filter. The gyro carries orientation through fast
// movement; the accelerometer slowly pulls "down" back to where gravity says it
// is, which is the only thing keeping the gyro from drifting away.
static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
static float ix = 0, iy = 0, iz = 0;     // integral feedback, removes gyro bias

static void fuse(float gx, float gy, float gz, float ax, float ay, float az,
                 float dt, float rest_g) {
  const float KP = 2.5f, KI = 0.05f;

  float n = sqrtf(ax * ax + ay * ay + az * az);
  if (n > 1e-3f) {
    // An accelerometer measures gravity PLUS whatever linear acceleration the
    // node is under, and it cannot tell them apart. While the node is being
    // shaken or dropped, |a| leaves 1 g and the vector is no longer "down", so
    // trusting it would tip the model in the direction of the shove. Fade the
    // correction out as |a| departs from rest and let the gyro coast instead.
    float trust = 1.0f - fminf(1.0f, fabsf(n - rest_g) / 0.30f);
    if (trust > 0.0f) {
      ax /= n; ay /= n; az /= n;
      // where the filter currently believes "up" points, in chip axes
      float vx = 2 * (q1 * q3 - q0 * q2);
      float vy = 2 * (q0 * q1 + q2 * q3);
      float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
      // error = measured up x estimated up
      float ex = ay * vz - az * vy;
      float ey = az * vx - ax * vz;
      float ez = ax * vy - ay * vx;
      ix += KI * trust * ex * dt;
      iy += KI * trust * ey * dt;
      iz += KI * trust * ez * dt;
      gx += KP * trust * ex + ix;
      gy += KP * trust * ey + iy;
      gz += KP * trust * ez + iz;
    }
  }

  // Rotate by the exact angle the rate implies over dt. The usual first-order
  // step (q += 0.5*dt*q*w) quietly loses angle once the per-step rotation gets
  // big, which is precisely the fast movement that ended up misplaced.
  float w = sqrtf(gx * gx + gy * gy + gz * gz);
  float dw, dx, dy, dz;
  if (w * dt > 1e-6f) {
    float half = 0.5f * w * dt, k = sinf(half) / w;
    dw = cosf(half); dx = gx * k; dy = gy * k; dz = gz * k;
  } else {
    dw = 1.0f; dx = 0.5f * gx * dt; dy = 0.5f * gy * dt; dz = 0.5f * gz * dt;
  }
  float a = q0, b = q1, c = q2, d = q3;
  q0 = a * dw - b * dx - c * dy - d * dz;
  q1 = a * dx + b * dw + c * dz - d * dy;
  q2 = a * dy - b * dz + c * dw + d * dx;
  q3 = a * dz + b * dy - c * dx + d * dw;
  n = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  if (n > 1e-9f) { q0 /= n; q1 /= n; q2 /= n; q3 /= n; }
}

// Point the filter at the measured gravity right away instead of letting it
// walk there over the first few seconds. Yaw starts at zero.
static void seedFromGravity(float ax, float ay, float az) {
  float n = sqrtf(ax * ax + ay * ay + az * az);
  if (n < 1e-3f) return;
  ax /= n; ay /= n; az /= n;
  float roll  = atan2f(ay, az);
  float pitch = atan2f(-ax, hypotf(ay, az));
  float cr = cosf(roll / 2),  sr = sinf(roll / 2);
  float cp = cosf(pitch / 2), sp = sinf(pitch / 2);
  q0 = cp * cr; q1 = cp * sr; q2 = sp * cr; q3 = -sp * sr;
}

static void imuTask(void *) {
  const TickType_t period = pdMS_TO_TICKS(1000 / IMU_HZ);
  TickType_t next = xTaskGetTickCount();

  float bias[3] = {0, 0, 0};        // gyro zero offset, in raw counts
  double calG[3] = {0, 0, 0}, calA = 0;
  int calN = 0;
  float restG = 1.0f;
  uint32_t samples = 0, errors = 0, clipped = 0;
  uint32_t lastMicros = micros();
  int stillFor = 0;

  for (;;) {
    vTaskDelayUntil(&next, period);

    int16_t r[7];
    if (!mpuBurst(r)) { errors++; continue; }

    uint32_t now = micros();
    float dt = (now - lastMicros) / 1e6f;     // real elapsed, not the nominal period
    lastMicros = now;
    if (dt <= 0 || dt > 0.25f) dt = 1.0f / IMU_HZ;

    float ax = r[0] / ACCEL_LSB_PER_G;
    float ay = r[1] / ACCEL_LSB_PER_G;
    float az = r[2] / ACCEL_LSB_PER_G;
    float amag = sqrtf(ax * ax + ay * ay + az * az);

    if (calN < IMU_CAL_N) {
      // Boot calibration: the node is assumed to be sitting still. Learn the
      // gyro's zero offset and what this particular part calls 1 g.
      calG[0] += r[4]; calG[1] += r[5]; calG[2] += r[6];
      calA += amag;
      if (++calN == IMU_CAL_N) {
        for (int i = 0; i < 3; i++) bias[i] = calG[i] / IMU_CAL_N;
        restG = calA / IMU_CAL_N;
        seedFromGravity(ax, ay, az);
        Serial.printf("IMU calibrated: gyro bias %.1f %.1f %.1f counts, rest |a| %.3f g\n",
                      bias[0], bias[1], bias[2], restG);
      }
      taskENTER_CRITICAL(&poseMux);
      pose.ax = ax; pose.ay = ay; pose.az = az;
      taskEXIT_CRITICAL(&poseMux);
      continue;
    }

    // If this ever climbs, the range above is too small for how hard the node
    // is being thrown, and orientation is being under-integrated again.
    if (abs(r[4]) > GYRO_CLIP_COUNTS || abs(r[5]) > GYRO_CLIP_COUNTS ||
        abs(r[6]) > GYRO_CLIP_COUNTS) clipped++;

    float gx = (r[4] - bias[0]) / GYRO_LSB_PER_DPS;
    float gy = (r[5] - bias[1]) / GYRO_LSB_PER_DPS;
    float gz = (r[6] - bias[2]) / GYRO_LSB_PER_DPS;

    // A gyro's zero point wanders with temperature. Whenever the node is
    // genuinely still, creep the stored bias toward what it is reading now --
    // without this, yaw (which gravity cannot correct) drifts steadily.
    bool still = fabsf(gx) < 1.5f && fabsf(gy) < 1.5f && fabsf(gz) < 1.5f &&
                 fabsf(amag - restG) < 0.05f;
    if (still) {
      if (stillFor < IMU_HZ) stillFor++;
      if (stillFor >= IMU_HZ / 2)
        for (int i = 0; i < 3; i++) bias[i] += (r[4 + i] - bias[i]) * 0.002f;
    } else {
      stillFor = 0;
    }

    const float TO_RAD = (float)M_PI / 180.0f;
    fuse(gx * TO_RAD, gy * TO_RAD, gz * TO_RAD, ax, ay, az, dt, restG);
    samples++;

    taskENTER_CRITICAL(&poseMux);
    pose.q[0] = q0; pose.q[1] = q1; pose.q[2] = q2; pose.q[3] = q3;
    pose.ax = ax; pose.ay = ay; pose.az = az;
    pose.gx = gx; pose.gy = gy; pose.gz = gz;
    pose.rest_g = restG;
    pose.samples = samples; pose.errors = errors; pose.clipped = clipped;
    pose.still = still; pose.ready = true;
    taskEXIT_CRITICAL(&poseMux);
  }
}

// ---------------------------------------------------------------------- MESH

static void buildReport(node_report_t *r, const Pose &p) {
  memset(r, 0, sizeof(*r));
  r->version   = MESH_PROTO_VERSION;
  r->origin_id = myNodeId;
  r->seq       = ++mySeq;
  r->hops      = 0;
  r->uptime_ms = millis();
  r->volume    = (uint16_t)(lastVolume < 0 ? 0 : (lastVolume > 65535 ? 65535 : lastVolume));
  r->gas_raw   = (uint16_t)lastGasRaw;
  r->gas_mv    = (uint16_t)(lastGasMv < 0 ? 0 : (lastGasMv > 65535 ? 65535 : lastGasMv));

  for (int i = 0; i < 4; i++) r->q[i] = mesh_q15(p.q[i], MESH_Q_SCALE);
  r->accel[0] = mesh_q15(p.ax, MESH_ACCEL_SCALE);
  r->accel[1] = mesh_q15(p.ay, MESH_ACCEL_SCALE);
  r->accel[2] = mesh_q15(p.az, MESH_ACCEL_SCALE);
  r->gyro[0]  = mesh_q15(p.gx, MESH_GYRO_SCALE);
  r->gyro[1]  = mesh_q15(p.gy, MESH_GYRO_SCALE);
  r->gyro[2]  = mesh_q15(p.gz, MESH_GYRO_SCALE);

  uint8_t f = 0;
  if (lastGas)   f |= MESH_F_GAS;
  if (p.still)   f |= MESH_F_STILL;
  if (p.clipped) f |= MESH_F_GYRO_CLIP;
  if (p.ready)   f |= MESH_F_IMU_READY;
#if MESH_ROLE_GATEWAY
  f |= MESH_F_GATEWAY;
#endif
  // A DHT11 that fails to answer prints nan. Say so explicitly rather than
  // shipping a number that looks like a reading.
  if (isnan(lastTemp) || isnan(lastHum)) {
    f |= MESH_F_DHT_FAIL;
    r->temp_c10 = MESH_TEMP_INVALID;
    r->humidity = MESH_HUM_INVALID;
  } else {
    r->temp_c10 = mesh_q15(lastTemp, MESH_TEMP_SCALE);
    int h = (int)lroundf(lastHum);
    r->humidity = (uint8_t)(h < 0 ? 0 : (h > 100 ? 100 : h));
  }
  r->flags = f;
}

static void onMeshRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len != (int)sizeof(node_report_t)) return;      // not ours, or truncated
  node_report_t r;
  memcpy(&r, data, sizeof(r));
  if (!mesh_accept(&meshSeen, &r)) return;            // duplicate, stale or wrong version

#if MESH_ROLE_GATEWAY
  if (meshRx) xQueueSend(meshRx, &r, 0);              // full queue drops, never blocks
#else
  // Relay. No routing table: rebroadcast anything new until the hop limit runs
  // out, and let the dedup above stop it circling back.
  if (r.hops + 1 < MESH_MAX_HOPS) {
    r.hops++;
    esp_now_send(BROADCAST, (const uint8_t *)&r, sizeof(r));
  }
#endif
}

static void meshBegin() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  // ESP-NOW does not scan for peers, so every node has to be told the same
  // channel up front or they simply never hear each other.
  esp_wifi_set_channel(MESH_CHANNEL, WIFI_SECOND_CHAN_NONE);

  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
#ifdef NODE_ID
  myNodeId = NODE_ID;                    // pin it at build time if you prefer
#else
  myNodeId = 1 + ((mac[5] ^ mac[4]) % MESH_MAX_NODES);
#endif
  Serial.printf("mesh: role %s  node id %u  mac %02X:%02X:%02X:%02X:%02X:%02X  "
                "report %u bytes\n",
                MESH_ROLE_GATEWAY ? "gateway" : "node", myNodeId,
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                (unsigned)sizeof(node_report_t));

  if (esp_now_init() != ESP_OK) { Serial.println("mesh: esp_now_init failed"); return; }
  esp_now_register_recv_cb(onMeshRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST, 6);
  peer.channel = MESH_CHANNEL;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) Serial.println("mesh: add_peer failed");

#if MESH_ROLE_GATEWAY
  meshRx = xQueueCreate(16, sizeof(node_report_t));
#endif
}

void setup() {
  Serial.begin(115200);

  dht.begin();
  pinMode(led, OUTPUT);
  pinMode(MQ_DOUT, INPUT);

  // --- MPU6500 startup ---
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  uint8_t who = mpuRead8(REG_WHO_AM_I);
  Serial.printf("MPU WHO_AM_I = 0x%02X %s\n", who,
                (who == 0x00 || who == 0xFF) ? "<- NO RESPONSE, check wiring" : "<- found");

  mpuWrite(REG_PWR_MGMT_1, 0x00);       // wake
  delay(100);
  mpuWrite(REG_PWR_MGMT_1, 0x01);       // clock off the gyro PLL, steadier than the internal RC
  mpuWrite(REG_CONFIG, 0x01);           // gyro DLPF 184 Hz: 2.9 ms group delay
                                        // instead of 41 Hz's 5.9 ms, and it stops
                                        // the filter eating real fast rotation
  mpuWrite(REG_SMPLRT_DIV, 0x00);       // 1 kHz, so IMU_HZ never reads a stale sample twice
  mpuWrite(REG_GYRO_CONFIG, 0x18);      // +/-2000 deg/s
  mpuWrite(REG_ACCEL_CONFIG, 0x00);     // +/-2 g
  mpuWrite(REG_ACCEL_CONFIG2, 0x02);    // accel DLPF 92 Hz
  delay(50);

  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = I2S_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = I2S_FRAMES,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1, // NOT USED
    .data_in_num = I2S_SD
  };

  i2s_set_pin(I2S_NUM_0, &pin_config);

  // The IMU gets its own task on core 0 so its sample spacing is set by the
  // scheduler, not by however long loop() spent blocked on the microphone or
  // the DHT. Even sampling is what makes integrating the gyro meaningful.
  xTaskCreatePinnedToCore(imuTask, "imu", 4096, NULL, 5, NULL, 0);

  meshBegin();
}

void loop() {
  uint32_t now = millis();
  static uint32_t nextPose = 0, nextImuPrint = 0, nextSlow = 0, nextGas = 0, nextVol = 0, nextStatus = 0;

  // --- microphone ---
  // i2s_read paces this loop at ~16 ms per 256-frame block.
  int32_t audio_samples[I2S_FRAMES];
  size_t bytes_read;
  esp_err_t result = i2s_read(I2S_NUM_0, audio_samples, sizeof(audio_samples),
                              &bytes_read, portMAX_DELAY);

  if (result == ESP_OK && bytes_read > 0) {
    int samples_count = bytes_read / sizeof(int32_t);

    int32_t max_val = INT32_MIN;
    int32_t min_val = INT32_MAX;

    // Analyze the buffer to find extreme peaks
    for (int i = 0; i < samples_count; i++) {
      // >> MIC_GAIN_SHIFT drops the 32-bit frame into int16 range
      int32_t sample = audio_samples[i] >> MIC_GAIN_SHIFT;
      if (sample > max_val) max_val = sample;
      if (sample < min_val) min_val = sample;
    }

    // Calculate Peak-to-Peak Amplitude
    int32_t peak_to_peak = max_val - min_val;

    if (peak_to_peak > 32000) {
      digitalWrite(led, HIGH);  // Turn on LED if sound is loud
    } else {
      digitalWrite(led, LOW);   // Turn off LED if sound is quiet
    }

    lastVolume = peak_to_peak;

    // Print volume level as a visual serial plotter bar
    if (now >= nextVol) {
      nextVol = now + 100;
      Serial.print("Volume_Level:");
      Serial.println(peak_to_peak);
    }
  }

  Pose p;
  taskENTER_CRITICAL(&poseMux);
  p = pose;
  taskEXIT_CRITICAL(&poseMux);

  // --- this node's own reading ---
  // One JSON object per tick rather than a line per sensor: it is a single
  // consistent instant, and a new field costs the parser nothing.
  char line[320];
  node_report_t self;

#if MESH_ROLE_GATEWAY
  // On the cable. Send often -- the 3D model wants every update it can get, and
  // USB bandwidth is free.
  if (now >= nextPose) {
    nextPose = now + (1000 / GATEWAY_LOCAL_HZ);
    buildReport(&self, p);
    if (mesh_report_to_json(&self, line, sizeof(line)) > 0) Serial.println(line);
  }

  // --- everyone else's readings ---
  // Arrived over the radio.
  node_report_t in;
  while (meshRx && xQueueReceive(meshRx, &in, 0) == pdTRUE) {
    if (mesh_report_to_json(&in, line, sizeof(line)) > 0) Serial.println(line);
  }
#else
  // On battery in the rubble. Airtime and power are the budget, so report at a
  // low rate, and only speed up when something is actually happening.
  uint32_t period = 1000 / (p.still ? MESH_REPORT_HZ : MESH_REPORT_HZ_MOVE);
  if (now >= nextPose) {
    nextPose = now + period;
    buildReport(&self, p);
    esp_now_send(BROADCAST, (const uint8_t *)&self, sizeof(self));
  }
  // Same object on the cable once a second, so a node is debuggable over USB.
  if (now >= nextImuPrint) {
    nextImuPrint = now + 1000;
    buildReport(&self, p);
    if (mesh_report_to_json(&self, line, sizeof(line)) > 0) Serial.println(line);
  }
#endif

  if (now >= nextStatus) {
    nextStatus = now + 1000;
    Serial.printf("IMU status: samples %lu  i2c_err %lu  gyro_clipped %lu\n",
                  (unsigned long)p.samples, (unsigned long)p.errors,
                  (unsigned long)p.clipped);
  }

  // --- MQ gas ---
  // The ADC is cheap but nothing here changes in 16 ms.
  if (now >= nextGas) {
    nextGas = now + 500;
    lastGasRaw = analogRead(MQ_AOUT);
    lastGasMv  = analogReadMilliVolts(MQ_AOUT);
    lastGas = !digitalRead(MQ_DOUT);
  }

  // --- DHT11 ---
  // The part itself only updates about once a second.
  if (now >= nextSlow) {
    nextSlow = now + 2000;
    lastHum  = dht.readHumidity();
    lastTemp = dht.readTemperature();
  }
}
