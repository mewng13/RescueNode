#include <Arduino.h>
#include <DHT.h>
#include <Wire.h> 
#include <driver/i2s.h>
#define I2S_WS 13
#define I2S_SD 11
#define I2S_SCK 12

//I2S Configuration
#define I2S_SAMPLE_RATE 16000
#define I2S_FRAMES 256   
#define MIC_GAIN_SHIFT 13

#define MPU_ADDR 0x68 //I2C address of the MPU6500
#define SDA_PIN 8
#define SCL_PIN 9
#define REG_WHO_AM_I 0x75 //WHO_AM_I address (confirm communication)
#define REG_ACCEL_XOUT 0x3B //ACCEL_XOUT_H address (first of 6 bytes of accel data)
#define REG_PWR_MGMT_1 0x6B //PW_MGMT_1 address (clears SLEEP bit to wake up the chip)

//setUP()할떄 MPU6500를 초기화함 (+SLEEP에서 깨움)
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

void setup() {
    Serial.begin(115200);

    // --- MPU6500 startup ---
    Wire.begin(SDA_PIN, SCL_PIN);
    // setClock MUST come after begin(): before it, Wire's mutex doesn't exist yet,
    // so setClock bails out with "could not acquire lock" and the bus silently
    // stays at the default. Measured 50k/100k/400k: error rate is ~37% at all
    // three, so bus speed is not what's dropping reads.
    Wire.setClock(100000);
    uint8_t who = mpuRead8(REG_WHO_AM_I);
    Serial.printf("MPU WHO_AM_I = 0x%02X %s\n", who,
                    (who == 0x00 || who == 0xFF) ? "<- NO RESPONSE, check wiring" : "<- found");
    mpuWrite(REG_PWR_MGMT_1, 0x00);  
    delay(100);

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
}

void loop() {
  delay(100);
    // --- MPU6500: read accel X/Y/Z (6 bytes, big-endian signed 16-bit) ---
  // Reads stay NaN when the chip doesn't ACK, so a missing sensor prints "nan"
  // instead of the -0.00 that Wire.read()'s -1 return value used to produce.
  float ax = NAN, ay = NAN, az = NAN;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT);
  Wire.endTransmission(false);
  if (Wire.requestFrom(MPU_ADDR, 6) == 6) {
    // Assemble each 16-bit value in two steps: the operands of `|` are
    // unsequenced in C++, so (Wire.read() << 8) | Wire.read() lets the compiler
    // fetch the low byte first and silently byte-swaps the result.
    uint8_t b[6];
    for (int i = 0; i < 6; i++) b[i] = Wire.read();
    int16_t rx = (int16_t)((b[0] << 8) | b[1]);
    int16_t ry = (int16_t)((b[2] << 8) | b[3]);
    int16_t rz = (int16_t)((b[4] << 8) | b[5]);
    ax = rx / 16384.0f;   // 16384 counts per g at the default +/-2 g range
    ay = ry / 16384.0f;
    az = rz / 16384.0f;
  }
  int32_t audio_samples[I2S_FRAMES];
  size_t bytes_read;

  esp_err_t result = i2s_read(I2S_NUM_0, audio_samples, sizeof(audio_samples), &bytes_read, portMAX_DELAY);

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

    // Print volume level as a visual serial plotter bar
    Serial.print("Volume_Level:");
    Serial.println(peak_to_peak);
  }

  //Serial.printf("Accel X: %6.2f   Y: %6.2f   Z: %6.2f  (g)\n", ax, ay, az);
}