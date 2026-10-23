#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Adafruit_BMP280.h>
#include <TinyGPSPlus.h>
#include <math.h>

// ---------- HARDWARE ----------
#define SDA_PIN 21
#define SCL_PIN 22

// NEO-6M UART2: GPS TX -> ESP32 RX, GPS RX -> ESP32 TX.
#define GPS_ENABLED 1
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define GPS_BAUD_RATE 9600
#define GPS_MAX_AGE_MS 3000

// ---------- ESP-NOW ----------
#define ESPNOW_CHANNEL 1
#define SENSOR_UPDATE_INTERVAL_MS 100
#define SEND_INTERVAL_MS 1000
#define DISCOVERY_INTERVAL_MS 1000
#define TELEMETRY_VERSION 2

#define MOSFET_SIGNAL 26

const uint16_t IMU_CAL_SAMPLES = 250;
const uint16_t IMU_CAL_DELAY_MS = 10;
const uint16_t BMP_CAL_SAMPLES = 40;
const uint16_t BMP_CAL_DELAY_MS = 120;

const float GRAVITY = 9.80665f;
const float ALTITUDE_KALMAN_Q = 0.6f;
const float ALTITUDE_KALMAN_R = 1.5f;
const float BOOST_VELOCITY_MPS = 3.0f;
const uint8_t BOOST_CONFIRM_SAMPLES = 5;
const float BOOST_MIN_ALTITUDE_M = 1.0f;
const uint8_t APOGEE_DESCENT_SAMPLES = 25;
const float ALTITUDE_DEADBAND_M = 0.02f;
const uint32_t APOGEE_SIGNAL_DURATION_MS = 3000;
const float LANDING_STABLE_BAND_M = 0.30f;
const uint16_t LANDING_STABLE_SAMPLES = 30;
const float LANDING_MIN_DROP_M = 2.0f;

#define MAGIC 0x4D425954UL
#define VERSION 1

#define MSG_DISCOVERY 1
#define MSG_RESPONSE  2
#define MSG_TELEMETRY 3

#define FLAG_MPU_VALID 0x01
#define FLAG_BMP_VALID 0x02
#define FLAG_GPS_VALID 0x04
#define FLAG_ALTITUDE_VALID 0x08

// ---------- PACKETS ----------
struct __attribute__((packed)) ControlMessage {
    uint32_t magic;
    uint8_t type;
    uint8_t version;
};

struct __attribute__((packed)) TelemetryPacket {
    uint32_t magic;
    uint16_t version;
    uint16_t sequence;
    uint32_t uptime_ms;

    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t gx;
    int16_t gy;
    int16_t gz;

    int16_t temperature_centi_c;
    uint32_t pressure_pa;

    int32_t latitude_e7;
    int32_t longitude_e7;
    int32_t gps_altitude_cm;
    uint8_t gps_satellites;

    uint8_t flags;
    uint8_t flight_state;
    int32_t relative_altitude_cm;
    int16_t mpu_temperature_centi_c;
};

static_assert(sizeof(ControlMessage) == 6,
              "ControlMessage size mismatch");

static_assert(sizeof(TelemetryPacket) == 51,
              "TelemetryPacket size mismatch");

enum FlightState : uint8_t {
    STATE_STARTUP = 0,
    STATE_ARMING = 1,
    STATE_BOOST = 2,
    STATE_APOGEE = 3,
    STATE_LANDING = 4
};

// ---------- I2C SENSORS ----------
Adafruit_BMP280 bmp;
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

const uint8_t MPU_CANDIDATES[] = {0x68, 0x69};

uint8_t mpuAddress = 0;
bool mpuInitialized = false;
bool bmpInitialized = false;
bool gpsInitialized = false;
uint8_t mpuChipId = 0;

// ---------- ESP-NOW STATE ----------
uint8_t broadcastMAC[] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

uint8_t receiverMAC[6] = {0};
bool receiverConnected = false;

volatile bool responsePending = false;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

uint16_t sequenceNumber = 0;

unsigned long lastDiscovery = 0;
unsigned long lastSensorUpdate = 0;
unsigned long lastSend = 0;

struct KalmanFilter {
    float estimate = 0.0f;
    float covariance = 1.0f;
    bool initialized = false;
};

KalmanFilter accelFilters[3];
KalmanFilter gyroFilters[3];
KalmanFilter temperatureFilter;
KalmanFilter pressureFilter;
KalmanFilter mpuTemperatureFilter;

int16_t filteredAccel[3] = {0};
int16_t filteredGyro[3] = {0};
int16_t filteredTemperature = 0;
uint32_t filteredPressure = 0;
bool filteredMPUValid = false;
bool filteredBMPValid = false;
float filteredMpuTemperature = NAN;

float gyroBiasRaw[3] = {0.0f};
float accelBiasRaw[3] = {0.0f};
float pressureZeroPa = 0.0f;
bool gyroCalibrated = false;
bool altitudeCalibrated = false;

FlightState flightState = STATE_STARTUP;
bool stateReferenceValid = false;
float previousStateAltitude = 0.0f;
uint32_t previousStateTime = 0;
float maximumAltitude = 0.0f;
uint8_t boostCount = 0;
uint8_t descentCount = 0;
uint16_t stableCount = 0;
bool apogeeSignalActive = false;
uint32_t apogeeSignalStartTime = 0;

float filteredAltitude = 0.0f;
float altitudeCovariance = 1.0f;
bool altitudeFilterInitialized = false;
uint32_t lastAltitudeFilterTime = 0;

void updateFlightState(uint32_t now);

// ---------- I2C HELPERS ----------
bool devicePresent(uint8_t address) {
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
}

bool writeMPURegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(mpuAddress);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool readMPURegister(uint8_t reg, uint8_t &value) {
    Wire.beginTransmission(mpuAddress);
    Wire.write(reg);

    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    if (Wire.requestFrom(mpuAddress, (uint8_t)1) != 1) {
        return false;
    }

    value = Wire.read();
    return true;
}

// ---------- SENSOR INITIALIZATION ----------
void initializeMPU() {
    Serial.println("\nSearching for MPU6050-compatible sensor...");

    for (uint8_t address : MPU_CANDIDATES) {
        if (devicePresent(address)) {
            mpuAddress = address;
            break;
        }
    }

    if (mpuAddress == 0) {
        Serial.println("MPU not found at 0x68 or 0x69.");
        mpuInitialized = false;
        return;
    }

    if (readMPURegister(0x75, mpuChipId)) {
        Serial.printf("MPU address: 0x%02X\n", mpuAddress);
        Serial.printf("WHO_AM_I: 0x%02X\n", mpuChipId);
    }

    // Wake the sensor. Do not reject compatible clones solely
    // because their WHO_AM_I value differs from 0x68.
    if (!writeMPURegister(0x6B, 0x00)) {
        Serial.println("Failed to wake MPU.");
        mpuInitialized = false;
        return;
    }

    if (!writeMPURegister(0x1C, 0x18) ||
        !writeMPURegister(0x1B, 0x18) ||
        !writeMPURegister(0x1A, 0x04)) {
        Serial.println("Failed to configure MPU measurement ranges.");
        mpuInitialized = false;
        return;
    }

    delay(100);

    mpuInitialized = true;
    Serial.println("MPU initialized.");
}

void initializeBMP() {
    Serial.println("\nSearching for BMP280...");

if (bmp.begin(0x76)) {
    bmpInitialized = true;
    Serial.println("BMP280 found at 0x76.");
} else if (bmp.begin(0x77)) {
    bmpInitialized = true;
    Serial.println("BMP280 found at 0x77.");
} else {
    bmpInitialized = false;
    Serial.println("BMP280 not found.");
    return;
}

    bmp.setSampling(
        Adafruit_BMP280::MODE_NORMAL,
        Adafruit_BMP280::SAMPLING_X2,
        Adafruit_BMP280::SAMPLING_X16,
        Adafruit_BMP280::FILTER_X16,
        Adafruit_BMP280::STANDBY_MS_1
    );
}

void initializeGPS() {
#if GPS_ENABLED
    gpsSerial.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    gpsInitialized = true;
    Serial.printf("NEO-6M UART2 started at %lu baud (RX=%d, TX=%d).\n",
                  (unsigned long)GPS_BAUD_RATE, GPS_RX_PIN, GPS_TX_PIN);
#else
    gpsInitialized = false;
    Serial.println("GPS disabled (GPS_ENABLED=0).");
#endif
}

void updateGPS() {
#if GPS_ENABLED
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }
#endif
}

// ---------- ESP-NOW HELPERS ----------
bool addPeer(const uint8_t *mac) {
    if (esp_now_is_peer_exist(mac)) {
        return true;
    }

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, mac, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.encrypt = false;
    peer.ifidx = WIFI_IF_STA;

    esp_err_t result = esp_now_add_peer(&peer);

    if (result != ESP_OK) {
        Serial.printf("Failed to add peer: %s\n",
                      esp_err_to_name(result));
        return false;
    }

    return true;
}

// Older Arduino-ESP32 core 2.x callback signature.
void onReceive(const uint8_t *mac,
               const uint8_t *data,
               int len) {
    if (len != sizeof(ControlMessage)) {
        return;
    }

    ControlMessage msg;
    memcpy(&msg, data, sizeof(msg));

    if (msg.magic != MAGIC ||
        msg.version != VERSION ||
        msg.type != MSG_RESPONSE) {
        return;
    }

    portENTER_CRITICAL(&mux);
    memcpy(receiverMAC, mac, 6);
    responsePending = true;
    portEXIT_CRITICAL(&mux);
}

void processDiscoveryResponse() {
    uint8_t discoveredMAC[6];
    bool pending = false;

    portENTER_CRITICAL(&mux);

    if (responsePending) {
        memcpy(discoveredMAC, receiverMAC, 6);
        responsePending = false;
        pending = true;
    }

    portEXIT_CRITICAL(&mux);

    if (!pending) {
        return;
    }

    if (!addPeer(discoveredMAC)) {
        Serial.println("Could not add discovered receiver.");
        return;
    }

    memcpy(receiverMAC, discoveredMAC, 6);
    receiverConnected = true;

    Serial.printf(
        "\nReceiver discovered: "
        "%02X:%02X:%02X:%02X:%02X:%02X\n",
        receiverMAC[0], receiverMAC[1], receiverMAC[2],
        receiverMAC[3], receiverMAC[4], receiverMAC[5]
    );
}

// ---------- MPU READING ----------
bool readMPU(int16_t &ax, int16_t &ay, int16_t &az,
             int16_t &gx, int16_t &gy, int16_t &gz,
             int16_t &temperature) {
    if (!mpuInitialized) {
        return false;
    }

    Wire.beginTransmission(mpuAddress);
    Wire.write(0x3B);

    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    if (Wire.requestFrom(mpuAddress, (uint8_t)14) != 14) {
        return false;
    }

    ax = (int16_t)((Wire.read() << 8) | Wire.read());
    ay = (int16_t)((Wire.read() << 8) | Wire.read());
    az = (int16_t)((Wire.read() << 8) | Wire.read());

    temperature = (int16_t)((Wire.read() << 8) | Wire.read());

    gx = (int16_t)((Wire.read() << 8) | Wire.read());
    gy = (int16_t)((Wire.read() << 8) | Wire.read());
    gz = (int16_t)((Wire.read() << 8) | Wire.read());

    return true;
}

float updateKalman(KalmanFilter &filter, float measurement,
                   float processNoise, float measurementNoise) {
    if (!filter.initialized) {
        filter.estimate = measurement;
        filter.initialized = true;
        return filter.estimate;
    }

    filter.covariance += processNoise;
    float gain = filter.covariance /
                 (filter.covariance + measurementNoise);
    filter.estimate += gain * (measurement - filter.estimate);
    filter.covariance *= 1.0f - gain;
    return filter.estimate;
}

bool calibrateMPU() {
    if (!mpuInitialized) return false;

    Serial.println("\nKeep the MPU stationary for calibration...");
    double accelSum[3] = {0.0, 0.0, 0.0};
    double gyroSum[3] = {0.0, 0.0, 0.0};
    float gyroMinimum[3] = {INFINITY, INFINITY, INFINITY};
    float gyroMaximum[3] = {-INFINITY, -INFINITY, -INFINITY};

    for (uint16_t sample = 0; sample < IMU_CAL_SAMPLES; sample++) {
        int16_t ax, ay, az, gx, gy, gz, temperature;
        if (!readMPU(ax, ay, az, gx, gy, gz, temperature)) {
            Serial.println("MPU calibration read failed.");
            return false;
        }

        const int16_t accel[3] = {ax, ay, az};
        const int16_t gyro[3] = {gx, gy, gz};
        for (uint8_t axis = 0; axis < 3; axis++) {
            accelSum[axis] += accel[axis];
            gyroSum[axis] += gyro[axis];
            float rateDps = gyro[axis] / 16.4f;
            gyroMinimum[axis] = fminf(gyroMinimum[axis], rateDps);
            gyroMaximum[axis] = fmaxf(gyroMaximum[axis], rateDps);
        }
        delay(IMU_CAL_DELAY_MS);
    }

    bool stableGyro = true;
    for (uint8_t axis = 0; axis < 3; axis++) {
        stableGyro &= gyroMaximum[axis] - gyroMinimum[axis] < 8.0f;
        gyroBiasRaw[axis] = gyroSum[axis] / IMU_CAL_SAMPLES;
    }

    if (stableGyro) {
        gyroCalibrated = true;
        Serial.println("Gyroscope bias calibrated.");
    } else {
        memset(gyroBiasRaw, 0, sizeof(gyroBiasRaw));
        Serial.println("Gyroscope moved during calibration; bias correction skipped.");
    }

    float averageAccel[3];
    for (uint8_t axis = 0; axis < 3; axis++) {
        averageAccel[axis] = (accelSum[axis] / IMU_CAL_SAMPLES) *
                             (GRAVITY / 2048.0f);
    }

    float magnitude = sqrtf(
        averageAccel[0] * averageAccel[0] +
        averageAccel[1] * averageAccel[1] +
        averageAccel[2] * averageAccel[2]);
    uint8_t gravityAxis = 0;
    if (fabsf(averageAccel[1]) > fabsf(averageAccel[gravityAxis])) gravityAxis = 1;
    if (fabsf(averageAccel[2]) > fabsf(averageAccel[gravityAxis])) gravityAxis = 2;

    bool alignedWithGravity =
        fabsf(averageAccel[gravityAxis]) > GRAVITY * 0.80f &&
        fabsf(magnitude - GRAVITY) < 2.0f;

    if (stableGyro && alignedWithGravity) {
        const float expectedGravityCounts =
            averageAccel[gravityAxis] >= 0.0f ? 2048.0f : -2048.0f;
        for (uint8_t axis = 0; axis < 3; axis++) {
            float expectedCounts = axis == gravityAxis
                ? expectedGravityCounts : 0.0f;
            accelBiasRaw[axis] =
                accelSum[axis] / IMU_CAL_SAMPLES - expectedCounts;
        }
        Serial.println("Accelerometer single-position offset calibrated.");
    } else {
        Serial.println("Accelerometer offset skipped; keep one axis aligned with gravity.");
    }

    return gyroCalibrated;
}

bool calibrateAltitudeZero() {
    if (!bmpInitialized) return false;

    Serial.println("\nKeep the BMP280 at the reference height...");
    double pressureSum = 0.0;
    for (uint16_t sample = 0; sample < BMP_CAL_SAMPLES; sample++) {
        float pressure = bmp.readPressure();
        if (!isfinite(pressure) || pressure < 30000.0f || pressure > 120000.0f) {
            Serial.println("Invalid pressure during altitude calibration.");
            return false;
        }
        pressureSum += pressure;
        delay(BMP_CAL_DELAY_MS);
    }

    pressureZeroPa = pressureSum / BMP_CAL_SAMPLES;
    altitudeCalibrated = true;
    altitudeFilterInitialized = true;
    filteredAltitude = 0.0f;
    altitudeCovariance = 1.0f;
    lastAltitudeFilterTime = millis();
    Serial.printf("Altitude reference pressure: %.1f Pa\n", pressureZeroPa);
    return true;
}

void updateFlightState(uint32_t now) {
    if (flightState == STATE_STARTUP) return;

    if (!stateReferenceValid) {
        previousStateAltitude = filteredAltitude;
        previousStateTime = now;
        maximumAltitude = filteredAltitude;
        stateReferenceValid = true;
        return;
    }

    float dt = (uint32_t)(now - previousStateTime) / 1000.0f;
    if (dt <= 0.0f) return;

    float delta = filteredAltitude - previousStateAltitude;
    float velocity = delta / dt;
    previousStateAltitude = filteredAltitude;
    previousStateTime = now;
    if (filteredAltitude > maximumAltitude) maximumAltitude = filteredAltitude;

    switch (flightState) {
        case STATE_ARMING:
            if (velocity >= BOOST_VELOCITY_MPS &&
                filteredAltitude >= BOOST_MIN_ALTITUDE_M) {
                if (boostCount < UINT8_MAX) boostCount++;
            } else {
                boostCount = 0;
            }
            if (boostCount >= BOOST_CONFIRM_SAMPLES) {
                flightState = STATE_BOOST;
                descentCount = 0;
                maximumAltitude = filteredAltitude;
                Serial.println("FLIGHT STATE: BOOST");
            }
            break;

        case STATE_BOOST:
            if (delta < -ALTITUDE_DEADBAND_M) {
                if (descentCount < UINT8_MAX) descentCount++;
            } else if (delta > ALTITUDE_DEADBAND_M) {
                descentCount = 0;
            }
            if (descentCount >= APOGEE_DESCENT_SAMPLES) {
                flightState = STATE_APOGEE;
                stableCount = 0;
                digitalWrite(MOSFET_SIGNAL, HIGH);
                apogeeSignalActive = true;
                apogeeSignalStartTime = now;
                Serial.println("FLIGHT STATE: APOGEE; GPIO26 HIGH");
            }
            break;

        case STATE_APOGEE:
            if (fabsf(delta) < LANDING_STABLE_BAND_M &&
                maximumAltitude - filteredAltitude >= LANDING_MIN_DROP_M) {
                if (stableCount < UINT16_MAX) stableCount++;
            } else {
                stableCount = 0;
            }
            if (stableCount >= LANDING_STABLE_SAMPLES) {
                flightState = STATE_LANDING;
                Serial.println("FLIGHT STATE: LANDING");
            }
            break;

        default:
            break;
    }
}

void updateSensorData() {
    int16_t ax, ay, az, gx, gy, gz, mpuTemperatureRaw;
    filteredMPUValid = readMPU(ax, ay, az, gx, gy, gz,
                               mpuTemperatureRaw);

    if (filteredMPUValid) {
        const float accelReadings[3] = {
            ax - accelBiasRaw[0], ay - accelBiasRaw[1],
            az - accelBiasRaw[2]
        };
        const float gyroReadings[3] = {
            gx - gyroBiasRaw[0], gy - gyroBiasRaw[1],
            gz - gyroBiasRaw[2]
        };

        for (uint8_t axis = 0; axis < 3; axis++) {
            filteredAccel[axis] = (int16_t)lroundf(
                updateKalman(accelFilters[axis], accelReadings[axis],
                             0.1f, 8.0f));
            filteredGyro[axis] = (int16_t)lroundf(
                updateKalman(gyroFilters[axis], gyroReadings[axis],
                             0.1f, 8.0f));
        }

        float mpuTemperature = mpuChipId == 0x70
            ? mpuTemperatureRaw / 333.87f + 21.0f
            : mpuTemperatureRaw / 340.0f + 36.53f;
        filteredMpuTemperature = updateKalman(
            mpuTemperatureFilter, mpuTemperature, 0.01f, 0.5f);
    }

    filteredBMPValid = false;
    if (bmpInitialized) {
        float temperature = bmp.readTemperature();
        float pressure = bmp.readPressure();

        if (isfinite(temperature) && isfinite(pressure) && pressure > 0.0f) {
            float smoothTemperature = updateKalman(
                temperatureFilter, temperature, 0.01f, 0.5f);
            float smoothPressure = updateKalman(
                pressureFilter, pressure, 0.25f, 25.0f);

            if (smoothTemperature * 100.0f >= -32768.0f &&
                smoothTemperature * 100.0f <= 32767.0f &&
                smoothPressure > 0.0f &&
                smoothPressure <= 4294967295.0f) {
                filteredTemperature = (int16_t)lroundf(
                    smoothTemperature * 100.0f);
                filteredPressure = (uint32_t)lroundf(smoothPressure);
                filteredBMPValid = true;
            }
        }
    }

    if (filteredBMPValid && altitudeCalibrated) {
        float pressureRatio = filteredPressure / pressureZeroPa;
        float rawAltitude = 44330.0f *
            (1.0f - powf(pressureRatio, 0.19029495f));
        uint32_t now = millis();

        if (!altitudeFilterInitialized) {
            filteredAltitude = rawAltitude;
            altitudeCovariance = 1.0f;
            altitudeFilterInitialized = true;
        } else {
            float dt = (uint32_t)(now - lastAltitudeFilterTime) / 1000.0f;
            if (dt <= 0.0f) dt = 0.001f;
            altitudeCovariance += ALTITUDE_KALMAN_Q * dt;
            float gain = altitudeCovariance /
                         (altitudeCovariance + ALTITUDE_KALMAN_R);
            filteredAltitude += gain * (rawAltitude - filteredAltitude);
            altitudeCovariance *= 1.0f - gain;
        }

        lastAltitudeFilterTime = now;
        updateFlightState(now);
    }
}

// ---------- TELEMETRY ----------
bool sendTelemetry() {
    TelemetryPacket packet = {};

    packet.magic = MAGIC;
    packet.version = TELEMETRY_VERSION;
    packet.sequence = sequenceNumber++;
    packet.uptime_ms = millis();
    packet.flight_state = (uint8_t)flightState;

    if (filteredMPUValid) {
        packet.ax = filteredAccel[0];
        packet.ay = filteredAccel[1];
        packet.az = filteredAccel[2];
        packet.gx = filteredGyro[0];
        packet.gy = filteredGyro[1];
        packet.gz = filteredGyro[2];
        packet.flags |= FLAG_MPU_VALID;
        if (isfinite(filteredMpuTemperature) &&
            filteredMpuTemperature * 100.0f >= -32768.0f &&
            filteredMpuTemperature * 100.0f <= 32767.0f) {
            packet.mpu_temperature_centi_c = (int16_t)lroundf(
                filteredMpuTemperature * 100.0f);
        }
    }

    if (filteredBMPValid) {
        packet.temperature_centi_c = filteredTemperature;
        packet.pressure_pa = filteredPressure;
        packet.flags |= FLAG_BMP_VALID;
    }

    if (altitudeCalibrated && filteredBMPValid &&
        isfinite(filteredAltitude) &&
        filteredAltitude * 100.0f >= -2147483648.0f &&
        filteredAltitude * 100.0f <= 2147483647.0f) {
        packet.relative_altitude_cm = (int32_t)lroundf(
            filteredAltitude * 100.0f);
        packet.flags |= FLAG_ALTITUDE_VALID;
    }

    if (gpsInitialized && gps.location.isValid() &&
        gps.location.age() <= GPS_MAX_AGE_MS) {
        double latitude = gps.location.lat();
        double longitude = gps.location.lng();

        if (isfinite(latitude) && isfinite(longitude)) {
            packet.latitude_e7 = (int32_t)lround(latitude * 10000000.0);
            packet.longitude_e7 = (int32_t)lround(longitude * 10000000.0);
            packet.gps_altitude_cm =
                gps.altitude.isValid() && isfinite(gps.altitude.meters())
                    ? (int32_t)lround(gps.altitude.meters() * 100.0)
                    : 0;
            packet.gps_satellites = gps.satellites.isValid()
                ? (uint8_t)gps.satellites.value()
                : 0;
            packet.flags |= FLAG_GPS_VALID;
        }
    }

    esp_err_t result = esp_now_send(
        receiverMAC,
        (const uint8_t *)&packet,
        sizeof(packet)
    );

    if (result != ESP_OK) {
        Serial.printf("Telemetry send failed: %s\n",
                      esp_err_to_name(result));
        return false;
    }

    if (flightState == STATE_STARTUP) {
        flightState = STATE_ARMING;
        Serial.println("FLIGHT STATE: ARMING");
    }
    return true;
}

// ---------- SETUP ----------
void setup() {
    Serial.begin(115200);
    delay(500);

    digitalWrite(MOSFET_SIGNAL, LOW);
    pinMode(MOSFET_SIGNAL, OUTPUT);
    flightState = STATE_STARTUP;

    Serial.println("\n=== ESP-NOW ROCKET AVIONICS ===");
    Serial.println("Flight state: STARTUP");

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000);

    initializeMPU();
    initializeBMP();
    initializeGPS();

    if (!calibrateMPU()) {
        Serial.println("MPU calibration incomplete; using uncorrected gyro readings.");
    }
    if (!calibrateAltitudeZero()) {
        Serial.println("Altitude calibration failed; flight-state detection unavailable.");
    }

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    Serial.print("Transmitter MAC: ");
    Serial.println(WiFi.macAddress());

    esp_err_t channelResult =
        esp_wifi_set_channel(
            ESPNOW_CHANNEL,
            WIFI_SECOND_CHAN_NONE
        );

    if (channelResult != ESP_OK) {
        Serial.printf("Channel setup failed: %s\n",
                      esp_err_to_name(channelResult));
    }

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW initialization failed!");
        while (true) {
            delay(1000);
        }
    }

    if (!addPeer(broadcastMAC)) {
        Serial.println("Could not add broadcast peer.");
        while (true) {
            delay(1000);
        }
    }

    esp_now_register_recv_cb(onReceive);

    Serial.println("Waiting for receiver discovery...");
}

// ---------- LOOP ----------
void loop() {
    processDiscoveryResponse();
    updateGPS();

    unsigned long now = millis();

    if (now - lastSensorUpdate >= SENSOR_UPDATE_INTERVAL_MS) {
        lastSensorUpdate = now;
        updateSensorData();
    }

    if (apogeeSignalActive &&
        (uint32_t)(now - apogeeSignalStartTime) >=
            APOGEE_SIGNAL_DURATION_MS) {
        digitalWrite(MOSFET_SIGNAL, LOW);
        apogeeSignalActive = false;
        Serial.println("GPIO26 apogee signal LOW");
    }

    if (!receiverConnected) {
        if (now - lastDiscovery >= DISCOVERY_INTERVAL_MS) {
            lastDiscovery = now;

            ControlMessage discovery = {
                MAGIC,
                MSG_DISCOVERY,
                VERSION
            };

            esp_now_send(
                broadcastMAC,
                (const uint8_t *)&discovery,
                sizeof(discovery)
            );

            Serial.println("Searching for receiver...");
        }

        delay(5);
        return;
    }

    if (now - lastSend >= SEND_INTERVAL_MS) {
        lastSend = now;
        sendTelemetry();
    }
}