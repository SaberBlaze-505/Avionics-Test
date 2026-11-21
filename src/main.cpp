/*
 * ESP32 ROCKET AVIONICS  -  BMP280 + MPU6050 + NEO-6M + ESP-NOW telemetry
 * =========================================================================
 * Features:
 *  1. EEPROM In-Flight Recovery: Survives mid-air reboot (G-shock / brownout).
 *     Restores ground zero pressure, flight phase, peak altitude, and resumes.
 *  2. 100-Sample Ring Buffer: Records the last 100 flight points to EEPROM.
 *     Send 'D' over Serial at 115200 baud to dump the CSV. Send 'C' to clear.
 *  3. MOSFET Safety: Cannot activate if relative altitude <= 30.0 m.
 *     Auto shut-off timer turns MOSFET LOW after 5 seconds (5000 ms).
 *  4. Flight State Machine: Fixed 20 Hz tick.
 *  5. Long Range (LR) Mode: Espressif PHY-layer range boost for ESP-NOW.
 *  6. Forced Maximum TX Power: Configured up to +20.5 dBm.
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <TinyGPSPlus.h>
#include <EEPROM.h>
#include <math.h>
#include <stddef.h>

// ---------- HARDWARE & SENSOR CONFIG ----------
#define SDA_PIN 21
#define SCL_PIN 22
#define MOSFET_SIGNAL 26

// Set SKIP_I2C to 1 to soft-skip all I2C communication & sensor polling (for ejection testing).
// Set to 0 to re-enable BMP280 + MPU6050 once the hardware short is resolved.
#define SKIP_I2C 1

#define I2C_CLOCK_HZ 100000

// NEO-6M UART2: GPS TX -> ESP32 RX, GPS RX -> ESP32 TX.
#define GPS_ENABLED 1
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define GPS_BAUD_RATE 9600
#define GPS_MAX_AGE_MS 3000

// ---------- ESP-NOW & RADIO SETTINGS ----------
#define ESPNOW_CHANNEL 1
#define SEND_INTERVAL_MS 100         // ESP-NOW telemetry rate: 10 Hz
#define DISCOVERY_INTERVAL_MS 1000
#define TELEMETRY_VERSION 3

// Radio boost options:
#define ESPNOW_LR_MODE 1             // 1: Enable Espressif Long Range (LR) Mode, 0: standard 802.11
#define ESPNOW_FORCE_MAX_TX_POWER 1  // 1: Force max hardware TX power (+20.5 dBm)

#define TELEMETRY_MODE_GCS_BINARY     1
#define TELEMETRY_MODE_TEXT_DISCOVERY 2
#define ESPNOW_TELEMETRY_MODE TELEMETRY_MODE_TEXT_DISCOVERY

uint8_t gcsMacAddress[6] = {0x68, 0x09, 0x47, 0x5C, 0x00, 0xF0};

#define SEND_ATTITUDE 1
#define SERIAL_TELEMETRY 1
#define SERIAL_TELEMETRY_INTERVAL_MS 20   // 50 lines/s

// ---------- MPU6050 ----------
const float MPU_ACCEL_LSB_PER_G   = 2048.0f;   // +-16 g
const float MPU_GYRO_LSB_PER_DPS  = 16.4f;     // +-2000 dps
const float MPU_DT_MIN            = 0.005f;    // max 200 Hz IMU update
const float MPU_DT_MAX            = 0.1f;
const float DEADBAND_DPS          = 0.3f;
const float ACCEL_TRUST_BAND_G    = 0.30f;
const float ATTITUDE_KP           = 1.0f;
const uint16_t MPU_FAIL_LIMIT     = 20;

// ---------- BAROMETER ----------
const uint32_t BARO_POLL_MS = 10;
const uint16_t BARO_FAIL_LIMIT = 50;

const uint8_t BARO_CTRL_MEAS_VALUE = (0x01 << 5) | (0x04 << 2) | 0x03;
const uint8_t BARO_CONFIG_VALUE = (0x00 << 5) | (0x02 << 2);

// ---------- CALIBRATION ----------
const uint16_t IMU_CAL_SAMPLES = 250;
const uint16_t IMU_CAL_DELAY_MS = 10;
const uint16_t BMP_CAL_SAMPLES = 60;
const uint32_t BMP_CAL_TIMEOUT_MS = 6000;
const float BMP_CAL_MAX_SPREAD_PA = 40.0f;     // reject calibration if board was moving (~3.5 m spread)

// ---------- SENSOR RETRY (pad only) ----------
const uint32_t SENSOR_RETRY_MS = 2000;

// ---------- BMP280 PRESSURE FILTER ----------
#define BARO_MEDIAN_SIZE 3
const float BARO_KF_PROCESS_NOISE = 1.0f;
const float BARO_KF_MEASUREMENT_NOISE = 9.0f;

// ---------- ALTITUDE FILTER ----------
const float ALT_KF_ACCEL_SIGMA = 3.5f;
const float ALT_KF_MEAS_SIGMA  = 0.30f;

// ---------- FLIGHT LOGIC ----------
const float GRAVITY = 9.80665f;

const uint32_t STATE_STEP_MS = 50;          // flight logic tick (20 Hz)
const uint32_t STATE_WINDOW_MS = 100;
const float ALTITUDE_DEADBAND_M = 0.02f;
const uint32_t BARO_STALE_MS = 250;

// ARMING -> BOOST: 30 consecutive ascending steps
const uint8_t BOOST_ASCENT_SAMPLES = 30;
const float BOOST_MIN_ALTITUDE_M = 1.0f;

// BOOST -> APOGEE: 25 consecutive descending steps
const uint8_t APOGEE_DESCENT_SAMPLES = 25;
const float APOGEE_MIN_DROP_M = 0.5f;

// ---------- MOSFET & SAFETY ----------
const uint32_t APOGEE_SIGNAL_DURATION_MS = 5000; // MOSFET stays HIGH for 5 s
const float MOSFET_MIN_ALTITUDE_M = 30.0f;        // Hard floor: cannot activate <= 30 m

// APOGEE -> LANDED
const float LANDING_STABLE_BAND_M = 1.50f;
const uint16_t LANDING_STABLE_WINDOWS = 30;
const float LANDING_MIN_DROP_M = 2.0f;

// ---------- EEPROM STORAGE CONFIG ----------
#define EEPROM_SIZE 4096
#define EEPROM_MAGIC 0x524F434BUL    // "ROCK"
#define MAX_LOG_RECORDS 100
#define EEPROM_LOG_INTERVAL_MS 250   // 4 Hz logging rate (covers last 25 seconds)

struct __attribute__((packed)) EepromHeader {
    uint32_t magic;              // EEPROM_MAGIC
    uint8_t  flightState;        // 0: STARTUP, 1: ARMING, 2: BOOST, 3: APOGEE, 4: LANDED
    uint8_t  inFlight;           // 1 if in the air (BOOST/APOGEE), 0 if on pad or landed
    uint8_t  mosfetTriggered;    // 1 if fired
    uint8_t  headIndex;          // Index (0..99) of next write
    uint16_t totalRecords;       // Number of valid records (0..100)
    float    pressureZeroPa;     // Calibrated ground zero pressure
    float    peakAltitude;       // Max altitude reached so far
    uint32_t bootCount;          // Total reboots
    uint8_t  checksum;           // Header integrity checksum
};

struct __attribute__((packed)) EepromLogRecord {
    uint32_t timestamp;   // millis()
    float    altitude;    // relative altitude in meters
    float    pressure;    // pressure in hPa
    float    velocity;    // m/s
    int16_t  accelZ;      // body frame vertical accel (cm/s^2)
    int16_t  pitch;       // deg * 10
    int16_t  roll;        // deg * 10
    uint8_t  flightState; // flight state enum
    uint8_t  mosfetState; // 0, 1, 2
};

const int RECORDS_START_ADDR = sizeof(EepromHeader);
EepromHeader eepromHeader;
uint32_t lastEepromLog = 0;
bool inFlightRecovery = false;

// ---------- CONTROL PROTOCOL (Two-Way Transceiver) ----------
#define MAGIC 0x4D425954UL
#define VERSION 1

#define MSG_DISCOVERY       1
#define MSG_RESPONSE        2
#define MSG_MOSFET_TRIGGER  3   // Signal from receiver to turn MOSFET pin HIGH
#define MSG_MOSFET_CUTOFF   4   // Signal from receiver to turn MOSFET pin LOW
#define MSG_COMMAND_ACK     5   // Confirmation sent back to receiver

#define FLAG_MPU_VALID 0x01
#define FLAG_BMP_VALID 0x02
#define FLAG_GPS_VALID 0x04
#define FLAG_ALTITUDE_VALID 0x08

struct __attribute__((packed)) ControlMessage {
    uint32_t magic;
    uint8_t type;
    uint8_t version;
};

static_assert(sizeof(ControlMessage) == 6, "ControlMessage size mismatch");

// ---------- GCS BINARY PROTOCOL ----------
#define GCS_HEADER   0xAB
#define GCS_FOOTER_1 0x0D
#define GCS_FOOTER_2 0x0A

struct __attribute__((packed)) GcsTelemetryPacket {
    uint8_t  header;
    uint32_t packet_id;
    uint32_t timestamp;
    uint8_t  flight_phase;
    float    altitude;
    float    pressure;
    float    temperature;
    float    accel_x;
    float    accel_y;
    float    accel_z;
    float    roll;
    float    pitch;
    float    yaw;
    float    velocity;
    float    max_altitude;
    int32_t  gps_lat;
    int32_t  gps_lon;
    float    gps_altitude;
    uint8_t  gps_satellites;
    uint8_t  gps_fix;
    uint8_t  checksum;
    uint8_t  footer1;
    uint8_t  footer2;
};

static_assert(sizeof(GcsTelemetryPacket) == 71, "GcsTelemetryPacket must be 71 bytes");

enum FlightState : uint8_t {
    STATE_STARTUP = 0,
    STATE_ARMING = 1,
    STATE_BOOST = 2,
    STATE_APOGEE = 3,
    STATE_LANDED = 4
};

// ---------- SENSORS ----------
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

const uint8_t MPU_CANDIDATES[] = {0x68, 0x69};
const uint8_t BARO_CANDIDATES[] = {0x76, 0x77};

uint8_t mpuAddress = 0;
bool mpuInitialized = false;
uint8_t mpuChipId = 0;

uint8_t baroAddress = 0;
bool baroInitialized = false;
uint8_t baroChipId = 0;

bool gpsInitialized = false;

struct BaroCal {
    uint16_t T1; int16_t T2, T3;
    uint16_t P1; int16_t P2, P3, P4, P5, P6, P7, P8, P9;
};
BaroCal baroCal;
int32_t baroTFine = 0;
int32_t lastAdcP = -1, lastAdcT = -1;
uint16_t baroFailCount = 0;

// ---------- ESP-NOW STATE ----------
uint8_t broadcastMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t receiverMAC[6] = {0};
bool receiverConnected = false;

volatile bool responsePending = false;
volatile bool remoteTriggerPending = false;
volatile bool remoteCutoffPending = false;
uint8_t pendingSenderMAC[6] = {0};
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

uint16_t sequenceNumber = 0;
uint32_t gcsPacketId = 0;
unsigned long lastDiscovery = 0;
unsigned long lastSend = 0;
unsigned long lastBaroPoll = 0;
uint32_t lastSendErrorPrint = 0;

// ---------- FILTERS / SENSOR OUTPUTS ----------
struct KalmanFilter {
    float estimate = 0.0f;
    float covariance = 1.0f;
    bool initialized = false;
};

KalmanFilter mpuTemperatureFilter;

int16_t filteredAccel[3] = {0};
int16_t filteredGyro[3] = {0};
bool filteredMPUValid = false;
float filteredMpuTemperature = NAN;
float linAccelMs2[3] = {0.0f};

float gyroBiasRaw[3] = {0.0f};
float accelBiasRaw[3] = {0.0f};
bool gyroCalibrated = false;

float baroPressurePa = 0.0f;
float baroPressureSmoothPa = 0.0f;
float baroTempC = 0.0f;
bool filteredBMPValid = false;
uint32_t lastBaroSampleMs = 0;
float pressureZeroPa = 0.0f;
bool altitudeCalibrated = false;

// ---------- ATTITUDE (quaternion) ----------
float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
float rollDeg = 0.0f, pitchDeg = 0.0f, yawDeg = 0.0f;
uint32_t lastImuMicros = 0;
uint16_t mpuFailCount = 0;
bool attitudeNeedsInit = true;

// ---------- ALTITUDE KALMAN ----------
struct AltitudeKF {
    float h = 0.0f, v = 0.0f;
    float p00 = 1.0f, p01 = 0.0f, p10 = 0.0f, p11 = 1.0f;
    uint32_t lastUs = 0;
    bool init = false;
};
AltitudeKF altKF;
float filteredAltitude = 0.0f;
float filteredVelocity = 0.0f;
float peakAltitude = 0.0f;

// ---------- FLIGHT STATE ----------
FlightState flightState = STATE_STARTUP;
bool stateReferenceValid = false;
float previousSampleAltitude = 0.0f;
float windowAltitude = 0.0f;
uint32_t windowTime = 0;
float maximumAltitude = 0.0f;
uint8_t boostCount = 0;
uint8_t descentCount = 0;
uint16_t stableCount = 0;

// ---------- MOSFET STATE ----------
bool apogeeSignalActive = false;     // Triggered automatically at apogee in-flight
bool manualSignalActive = false;     // Triggered remotely via receiver signal / serial test
bool mosfetTriggered = false;        // Latched status
uint32_t mosfetSignalStartTime = 0;  // Activation timestamp for auto shut-off timer
uint32_t lastStateStep = 0;

uint32_t lastSensorRetry = 0;
uint16_t sensorRetryCount = 0;

// =========================================================================
//                             EEPROM FUNCTIONS
// =========================================================================
uint8_t calculateHeaderChecksum(const EepromHeader &h) {
    const uint8_t *p = (const uint8_t *)&h;
    uint8_t sum = 0xAA;
    for (size_t i = 0; i < offsetof(EepromHeader, checksum); i++) {
        sum ^= p[i];
    }
    return sum;
}

void saveEepromHeader() {
    eepromHeader.checksum = calculateHeaderChecksum(eepromHeader);
    EEPROM.put(0, eepromHeader);
    EEPROM.commit();
}

void initEeprom() {
    EEPROM.begin(EEPROM_SIZE);
    EEPROM.get(0, eepromHeader);

    if (eepromHeader.magic != EEPROM_MAGIC ||
        eepromHeader.checksum != calculateHeaderChecksum(eepromHeader)) {
        Serial.println("[EEPROM] Initializing fresh storage layout...");
        memset(&eepromHeader, 0, sizeof(eepromHeader));
        eepromHeader.magic = EEPROM_MAGIC;
        eepromHeader.bootCount = 1;
        saveEepromHeader();
    } else {
        eepromHeader.bootCount++;
        Serial.printf("[EEPROM] Valid storage found. Boot count: %lu\n",
                      (unsigned long)eepromHeader.bootCount);

        if (eepromHeader.inFlight == 1 &&
            eepromHeader.flightState >= STATE_BOOST &&
            eepromHeader.flightState < STATE_LANDED &&
            eepromHeader.pressureZeroPa > 30000.0f &&
            eepromHeader.pressureZeroPa < 125000.0f) {

            inFlightRecovery = true;
            flightState = (FlightState)eepromHeader.flightState;
            pressureZeroPa = eepromHeader.pressureZeroPa;
            altitudeCalibrated = true;
            maximumAltitude = eepromHeader.peakAltitude;
            peakAltitude = eepromHeader.peakAltitude;
            mosfetTriggered = (eepromHeader.mosfetTriggered == 1);

            Serial.println("\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            Serial.println("!!! WARNING: IN-FLIGHT REBOOT RECOVERY ACTIVATED !!!");
            Serial.printf( "!!! Resumed Flight State: %u                     !!!\n", (unsigned)flightState);
            Serial.printf( "!!! Restored Ground Ref : %.1f Pa              !!!\n", pressureZeroPa);
            Serial.printf( "!!! Restored Peak Alt   : %.1f m               !!!\n", maximumAltitude);
            Serial.printf( "!!! MOSFET Fired Before : %s                   !!!\n", mosfetTriggered ? "YES" : "NO");
            Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        }
        saveEepromHeader();
    }
}

void logEepromSample(uint32_t now) {
    if (flightState < STATE_ARMING) return;

    EepromLogRecord rec;
    rec.timestamp = now;
    rec.altitude = filteredAltitude;
    rec.pressure = baroPressureSmoothPa / 100.0f;
    rec.velocity = filteredVelocity;
    rec.accelZ = (int16_t)lroundf(linAccelMs2[2] * 100.0f);
    rec.pitch = (int16_t)lroundf(pitchDeg * 10.0f);
    rec.roll = (int16_t)lroundf(rollDeg * 10.0f);
    rec.flightState = (uint8_t)flightState;
    rec.mosfetState = (apogeeSignalActive || manualSignalActive) ? 1 : (mosfetTriggered ? 2 : 0);

    int addr = RECORDS_START_ADDR + (eepromHeader.headIndex * sizeof(EepromLogRecord));
    EEPROM.put(addr, rec);

    eepromHeader.headIndex = (eepromHeader.headIndex + 1) % MAX_LOG_RECORDS;
    if (eepromHeader.totalRecords < MAX_LOG_RECORDS) {
        eepromHeader.totalRecords++;
    }

    EEPROM.put(offsetof(EepromHeader, headIndex), eepromHeader.headIndex);
    EEPROM.put(offsetof(EepromHeader, totalRecords), eepromHeader.totalRecords);
    EEPROM.commit();
}

void dumpEepromLog() {
    Serial.println("\n--- EEPROM FLIGHT LOG DUMP (Chronological) ---");
    Serial.printf("Total records: %u, Head Index: %u\n",
                  eepromHeader.totalRecords, eepromHeader.headIndex);
    Serial.println("index,timestamp_ms,altitude_m,pressure_hpa,velocity_mps,accel_z_mps2,pitch_deg,roll_deg,state,mosfet");

    uint8_t startIdx = (eepromHeader.totalRecords < MAX_LOG_RECORDS) ? 0 : eepromHeader.headIndex;
    for (uint16_t i = 0; i < eepromHeader.totalRecords; i++) {
        uint8_t idx = (startIdx + i) % MAX_LOG_RECORDS;
        int addr = RECORDS_START_ADDR + (idx * sizeof(EepromLogRecord));
        EepromLogRecord r;
        EEPROM.get(addr, r);
        Serial.printf("%u,%lu,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%u,%u\n",
                      i,
                      (unsigned long)r.timestamp,
                      r.altitude,
                      r.pressure,
                      r.velocity,
                      r.accelZ / 100.0f,
                      r.pitch / 10.0f,
                      r.roll / 10.0f,
                      r.flightState,
                      r.mosfetState);
    }
    Serial.println("--- END OF EEPROM DUMP ---\n");
}

void clearEepromLog() {
    eepromHeader.headIndex = 0;
    eepromHeader.totalRecords = 0;
    saveEepromHeader();
    Serial.println("[EEPROM] Log records cleared.");
}

// =========================================================================
//                               I2C HELPERS
// =========================================================================
bool devicePresent(uint8_t address) {
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
}

void i2cBusRecovery() {
    pinMode(SDA_PIN, INPUT_PULLUP);
    pinMode(SCL_PIN, OUTPUT);
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(10);

    for (uint8_t i = 0; i < 9; i++) {
        digitalWrite(SCL_PIN, LOW);
        delayMicroseconds(10);
        digitalWrite(SCL_PIN, HIGH);
        delayMicroseconds(10);
        if (digitalRead(SDA_PIN) == HIGH) break;
    }

    pinMode(SDA_PIN, OUTPUT);
    digitalWrite(SDA_PIN, LOW);
    delayMicroseconds(10);
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(SDA_PIN, HIGH);
    delayMicroseconds(10);

    pinMode(SDA_PIN, INPUT_PULLUP);
    pinMode(SCL_PIN, INPUT_PULLUP);
}

void startI2C() {
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(I2C_CLOCK_HZ);
    Wire.setTimeOut(50);
}

void i2cScan() {
    pinMode(SDA_PIN, INPUT_PULLUP);
    pinMode(SCL_PIN, INPUT_PULLUP);
    delay(2);
    Serial.printf("[I2C] idle levels: SDA=%d SCL=%d (both must be 1)\n",
                  digitalRead(SDA_PIN), digitalRead(SCL_PIN));
    startI2C();

    Serial.printf("[I2C] scanning (SDA=%d, SCL=%d, %lu Hz)...\n",
                  SDA_PIN, SCL_PIN, (unsigned long)I2C_CLOCK_HZ);
    uint8_t found = 0;
    for (uint8_t address = 1; address < 127; address++) {
        if (devicePresent(address)) {
            Serial.printf("[I2C] device at 0x%02X", address);
            if (address == 0x68 || address == 0x69) Serial.print("  <- MPU6050");
            if (address == 0x76 || address == 0x77) Serial.print("  <- BMP280/BME280/BMP180");
            Serial.println();
            found++;
        }
    }
    if (found == 0) {
        Serial.println("[I2C] NO DEVICE ANSWERED. Check wiring and power.");
    }
}

// =========================================================================
//                                  MPU6050
// =========================================================================
bool writeMPURegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(mpuAddress);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool readMPURegister(uint8_t reg, uint8_t &value) {
    Wire.beginTransmission(mpuAddress);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)mpuAddress, (int)1) != 1) return false;
    value = Wire.read();
    return true;
}

static int16_t readWord() {
    uint8_t high = Wire.read();
    uint8_t low = Wire.read();
    return (int16_t)((high << 8) | low);
}

bool configureMPU() {
    if (!writeMPURegister(0x6B, 0x01)) return false;
    delay(20);

    if (!writeMPURegister(0x19, 0x01) ||
        !writeMPURegister(0x1A, 0x03) ||
        !writeMPURegister(0x1B, 0x18) ||
        !writeMPURegister(0x1C, 0x18)) {
        return false;
    }

    uint8_t gyroCfg = 0, accelCfg = 0;
    if (!readMPURegister(0x1B, gyroCfg) || !readMPURegister(0x1C, accelCfg) ||
        gyroCfg != 0x18 || accelCfg != 0x18) {
        return false;
    }
    return true;
}

bool initializeMPU(bool verbose) {
    mpuInitialized = false;
    mpuAddress = 0;

    for (uint8_t address : MPU_CANDIDATES) {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0) {
            mpuAddress = address;
            break;
        }
    }

    if (mpuAddress == 0) {
        if (verbose) Serial.println("MPU6050 not found at 0x68 or 0x69.");
        return false;
    }

    uint8_t id = 0;
    if (readMPURegister(0x75, id)) {
        mpuChipId = id;
        Serial.printf("MPU address: 0x%02X, WHO_AM_I: 0x%02X\n", mpuAddress, id);
        if (id == 0x00 || id == 0xFF) return false;
    } else {
        return false;
    }

    writeMPURegister(0x6B, 0x80);
    delay(100);
    if (!devicePresent(mpuAddress)) return false;
    if (!configureMPU()) return false;

    delay(50);
    mpuInitialized = true;
    mpuFailCount = 0;
    Serial.println("MPU initialized.");
    return true;
}

void recoverMPU() {
    Serial.println("MPU read failures; restarting I2C and MPU.");
    Wire.end();
    i2cBusRecovery();
    startI2C();
    configureMPU();
}

bool readMPU(int16_t &ax, int16_t &ay, int16_t &az,
             int16_t &gx, int16_t &gy, int16_t &gz,
             int16_t &temperature) {
    if (!mpuInitialized) return false;

    Wire.beginTransmission(mpuAddress);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)mpuAddress, (int)14) != 14) return false;

    ax = readWord();
    ay = readWord();
    az = readWord();
    temperature = readWord();
    gx = readWord();
    gy = readWord();
    gz = readWord();

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
    float gain = filter.covariance / (filter.covariance + measurementNoise);
    filter.estimate += gain * (measurement - filter.estimate);
    filter.covariance *= 1.0f - gain;
    return filter.estimate;
}

void initAttitudeFromAccel(float ax, float ay, float az) {
    float rollRad = atan2f(ay, az);
    float pitchRad = atan2f(-ax, sqrtf(ay * ay + az * az));
    float cr = cosf(rollRad * 0.5f), sr = sinf(rollRad * 0.5f);
    float cp = cosf(pitchRad * 0.5f), sp = sinf(pitchRad * 0.5f);
    qw = cr * cp;
    qx = sr * cp;
    qy = cr * sp;
    qz = -sr * sp;
}

void updateQuaternion(float gx, float gy, float gz,
                      float ax, float ay, float az, float dt) {
    gx *= PI / 180.0f;
    gy *= PI / 180.0f;
    gz *= PI / 180.0f;

    float aNorm = sqrtf(ax * ax + ay * ay + az * az);
    if (aNorm > 1.0e-6f && fabsf(aNorm - 1.0f) < ACCEL_TRUST_BAND_G) {
        ax /= aNorm; ay /= aNorm; az /= aNorm;

        float vx = 2.0f * (qx * qz - qw * qy);
        float vy = 2.0f * (qw * qx + qy * qz);
        float vz = qw * qw - qx * qx - qy * qy + qz * qz;

        float ex = ay * vz - az * vy;
        float ey = az * vx - ax * vz;
        float ez = ax * vy - ay * vx;

        gx += ATTITUDE_KP * ex;
        gy += ATTITUDE_KP * ey;
        gz += ATTITUDE_KP * ez;
    }

    const float oldW = qw, oldX = qx, oldY = qy, oldZ = qz;

    qw += 0.5f * (-oldX * gx - oldY * gy - oldZ * gz) * dt;
    qx += 0.5f * ( oldW * gx + oldY * gz - oldZ * gy) * dt;
    qy += 0.5f * ( oldW * gy - oldX * gz + oldZ * gx) * dt;
    qz += 0.5f * ( oldW * gz + oldX * gy - oldY * gx) * dt;

    float qNorm = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);
    if (qNorm > 1.0e-6f) {
        qw /= qNorm; qx /= qNorm; qy /= qNorm; qz /= qNorm;
    }
}

void quaternionToEuler() {
    const float radToDeg = 180.0f / PI;

    float sinr = 2.0f * (qw * qx + qy * qz);
    float cosr = 1.0f - 2.0f * (qx * qx + qy * qy);
    rollDeg = atan2f(sinr, cosr) * radToDeg;

    float sinp = 2.0f * (qw * qy - qz * qx);
    if (sinp > 1.0f) sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    pitchDeg = asinf(sinp) * radToDeg;

    float siny = 2.0f * (qw * qz + qx * qy);
    float cosy = 1.0f - 2.0f * (qy * qy + qz * qz);
    yawDeg = atan2f(siny, cosy) * radToDeg;
    if (yawDeg >= 360.0f) yawDeg -= 360.0f;
    if (yawDeg < 0.0f) yawDeg += 360.0f;
}

void computeLinearAccel(const float accelG[3]) {
    float vx = 2.0f * (qx * qz - qw * qy);
    float vy = 2.0f * (qw * qx + qy * qz);
    float vz = qw * qw - qx * qx - qy * qy + qz * qz;
    linAccelMs2[0] = (accelG[0] - vx) * GRAVITY;
    linAccelMs2[1] = (accelG[1] - vy) * GRAVITY;
    linAccelMs2[2] = (accelG[2] - vz) * GRAVITY;
}

bool calibrateMPU() {
    if (!mpuInitialized) return false;

    Serial.println("\nKeep the MPU stationary for calibration...");
    double accelSum[3] = {0.0, 0.0, 0.0};
    double gyroSum[3] = {0.0, 0.0, 0.0};
    float gyroMinimum[3] = {INFINITY, INFINITY, INFINITY};
    float gyroMaximum[3] = {-INFINITY, -INFINITY, -INFINITY};

    gyroCalibrated = false;
    memset(gyroBiasRaw, 0, sizeof(gyroBiasRaw));
    memset(accelBiasRaw, 0, sizeof(accelBiasRaw));

    uint16_t good = 0, bad = 0;
    while (good < IMU_CAL_SAMPLES) {
        int16_t ax, ay, az, gx, gy, gz, temperature;
        if (!readMPU(ax, ay, az, gx, gy, gz, temperature)) {
            if (++bad > IMU_CAL_SAMPLES / 4) return false;
            delay(IMU_CAL_DELAY_MS);
            continue;
        }

        const int16_t accel[3] = {ax, ay, az};
        const int16_t gyro[3] = {gx, gy, gz};
        for (uint8_t axis = 0; axis < 3; axis++) {
            accelSum[axis] += accel[axis];
            gyroSum[axis] += gyro[axis];
            float rateDps = gyro[axis] / MPU_GYRO_LSB_PER_DPS;
            gyroMinimum[axis] = fminf(gyroMinimum[axis], rateDps);
            gyroMaximum[axis] = fmaxf(gyroMaximum[axis], rateDps);
        }
        good++;
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
    }

    float averageAccel[3];
    for (uint8_t axis = 0; axis < 3; axis++) {
        averageAccel[axis] = (accelSum[axis] / IMU_CAL_SAMPLES) * (GRAVITY / MPU_ACCEL_LSB_PER_G);
    }

    float magnitude = sqrtf(averageAccel[0] * averageAccel[0] +
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
            averageAccel[gravityAxis] >= 0.0f ? MPU_ACCEL_LSB_PER_G : -MPU_ACCEL_LSB_PER_G;
        for (uint8_t axis = 0; axis < 3; axis++) {
            float expectedCounts = axis == gravityAxis ? expectedGravityCounts : 0.0f;
            accelBiasRaw[axis] = accelSum[axis] / IMU_CAL_SAMPLES - expectedCounts;
        }
        Serial.println("Accelerometer single-position offset calibrated.");
    }

    attitudeNeedsInit = true;
    lastImuMicros = micros();
    mpuFailCount = 0;
    return gyroCalibrated;
}

void updateMPU() {
    if (!mpuInitialized) {
        filteredMPUValid = false;
        return;
    }

    uint32_t nowUs = micros();
    float dt = (uint32_t)(nowUs - lastImuMicros) / 1000000.0f;
    if (dt < MPU_DT_MIN) return;
    lastImuMicros = nowUs;

    int16_t ax, ay, az, gx, gy, gz, mpuTemperatureRaw;
    if (!readMPU(ax, ay, az, gx, gy, gz, mpuTemperatureRaw)) {
        if (mpuFailCount < UINT16_MAX) mpuFailCount++;
        if (mpuFailCount >= MPU_FAIL_LIMIT) {
            filteredMPUValid = false;
            if (mpuFailCount % MPU_FAIL_LIMIT == 0) recoverMPU();
        }
        return;
    }
    mpuFailCount = 0;
    filteredMPUValid = true;

    const float accelCounts[3] = {
        ax - accelBiasRaw[0], ay - accelBiasRaw[1], az - accelBiasRaw[2]
    };
    const float gyroCounts[3] = {
        gx - gyroBiasRaw[0], gy - gyroBiasRaw[1], gz - gyroBiasRaw[2]
    };

    float gyroDps[3];
    float accelG[3];
    for (uint8_t axis = 0; axis < 3; axis++) {
        gyroDps[axis] = gyroCounts[axis] / MPU_GYRO_LSB_PER_DPS;
        if (fabsf(gyroDps[axis]) < DEADBAND_DPS) gyroDps[axis] = 0.0f;
        accelG[axis] = accelCounts[axis] / MPU_ACCEL_LSB_PER_G;

        filteredAccel[axis] = (int16_t)lroundf(accelCounts[axis]);
        filteredGyro[axis] = (int16_t)lroundf(gyroDps[axis] * MPU_GYRO_LSB_PER_DPS);
    }

    float mpuTemperature = mpuChipId == 0x70
        ? mpuTemperatureRaw / 333.87f + 21.0f
        : mpuTemperatureRaw / 340.0f + 36.53f;
    filteredMpuTemperature = updateKalman(mpuTemperatureFilter, mpuTemperature, 0.01f, 0.5f);

    if (attitudeNeedsInit) {
        initAttitudeFromAccel(accelG[0], accelG[1], accelG[2]);
        attitudeNeedsInit = false;
        quaternionToEuler();
    } else if (dt <= MPU_DT_MAX) {
        updateQuaternion(gyroDps[0], gyroDps[1], gyroDps[2],
                         accelG[0], accelG[1], accelG[2], dt);
        quaternionToEuler();
    }
    computeLinearAccel(accelG);
}

// =========================================================================
//                       BAROMETER (BMP280 / BME280 driver)
// =========================================================================
bool writeBaroRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(baroAddress);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool readBaroRegisters(uint8_t reg, uint8_t *buffer, uint8_t length) {
    Wire.beginTransmission(baroAddress);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)baroAddress, (int)length) != length) return false;
    for (uint8_t i = 0; i < length; i++) buffer[i] = Wire.read();
    return true;
}

static uint16_t leU16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static int16_t leS16(const uint8_t *p) { return (int16_t)leU16(p); }

bool initializeBaro(bool verbose) {
    baroInitialized = false;
    baroAddress = 0;

    for (uint8_t address : BARO_CANDIDATES) {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0) {
            baroAddress = address;
            break;
        }
    }

    if (baroAddress == 0) {
        if (verbose) Serial.println("Barometer not found at 0x76 or 0x77.");
        return false;
    }

    uint8_t id = 0;
    if (!readBaroRegisters(0xD0, &id, 1)) return false;
    baroChipId = id;
    Serial.printf("Barometer at 0x%02X, chip ID 0x%02X\n", baroAddress, id);

    if (id == 0x55) return false;
    if (id != 0x56 && id != 0x57 && id != 0x58 && id != 0x60) return false;

    writeBaroRegister(0xE0, 0xB6);
    delay(10);

    uint8_t raw[24];
    if (!readBaroRegisters(0x88, raw, 24)) return false;
    baroCal.T1 = leU16(&raw[0]);  baroCal.T2 = leS16(&raw[2]);  baroCal.T3 = leS16(&raw[4]);
    baroCal.P1 = leU16(&raw[6]);  baroCal.P2 = leS16(&raw[8]);  baroCal.P3 = leS16(&raw[10]);
    baroCal.P4 = leS16(&raw[12]); baroCal.P5 = leS16(&raw[14]); baroCal.P6 = leS16(&raw[16]);
    baroCal.P7 = leS16(&raw[18]); baroCal.P8 = leS16(&raw[20]); baroCal.P9 = leS16(&raw[22]);

    if (baroCal.T1 == 0 || baroCal.T1 == 0xFFFF || baroCal.P1 == 0 || baroCal.P1 == 0xFFFF) {
        return false;
    }

    if (!writeBaroRegister(0xF5, BARO_CONFIG_VALUE) ||
        !writeBaroRegister(0xF4, BARO_CTRL_MEAS_VALUE)) {
        return false;
    }

    lastAdcP = -1;
    lastAdcT = -1;
    baroFailCount = 0;
    baroInitialized = true;
    Serial.println("Barometer initialized.");
    return true;
}

static int32_t compensateTemperature(int32_t adcT) {
    int32_t var1 = ((((adcT >> 3) - ((int32_t)baroCal.T1 << 1))) * ((int32_t)baroCal.T2)) >> 11;
    int32_t var2 = (((((adcT >> 4) - ((int32_t)baroCal.T1)) *
                      ((adcT >> 4) - ((int32_t)baroCal.T1))) >> 12) * ((int32_t)baroCal.T3)) >> 14;
    baroTFine = var1 + var2;
    return (baroTFine * 5 + 128) >> 8;
}

static uint32_t compensatePressure(int32_t adcP) {
    int64_t var1 = ((int64_t)baroTFine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)baroCal.P6;
    var2 = var2 + ((var1 * (int64_t)baroCal.P5) << 17);
    var2 = var2 + (((int64_t)baroCal.P4) << 35);
    var1 = ((var1 * var1 * (int64_t)baroCal.P3) >> 8) + ((var1 * (int64_t)baroCal.P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)baroCal.P1) >> 33;
    if (var1 == 0) return 0;

    int64_t p = 1048576 - adcP;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)baroCal.P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)baroCal.P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)baroCal.P7) << 4);
    return (uint32_t)p;
}

bool readBaro(float &pressurePa, float &temperatureC, bool &isNew) {
    isNew = false;
    uint8_t d[6];
    if (!readBaroRegisters(0xF7, d, 6)) return false;

    int32_t adcP = ((int32_t)d[0] << 12) | ((int32_t)d[1] << 4) | (d[2] >> 4);
    int32_t adcT = ((int32_t)d[3] << 12) | ((int32_t)d[4] << 4) | (d[5] >> 4);
    if (adcP == 0x80000 || adcT == 0x80000) return false;

    if (adcP == lastAdcP && adcT == lastAdcT) return true;
    lastAdcP = adcP;
    lastAdcT = adcT;

    int32_t tCenti = compensateTemperature(adcT);
    uint32_t pQ24 = compensatePressure(adcP);
    float pressure = pQ24 / 256.0f;

    if (!isfinite(pressure) || pressure < 30000.0f || pressure > 125000.0f) return false;

    pressurePa = pressure;
    temperatureC = tCenti / 100.0f;
    isNew = true;
    return true;
}

static float pressureToAltitude(float pressurePa) {
    return 44330.0f * (1.0f - powf(pressurePa / pressureZeroPa, 0.19029495f));
}

// ---------- BMP280 PRESSURE FILTER ----------
struct PressurePrefilter {
    float buffer[BARO_MEDIAN_SIZE] = {0.0f};
    uint8_t count = 0;
    uint8_t head = 0;
    float estimate = 0.0f;
    float covariance = 9.0f;
    bool init = false;
};
PressurePrefilter pressurePre;

void pressurePrefilterReset() {
    pressurePre = PressurePrefilter();
}

float pressurePrefilterUpdate(float pressurePa, uint32_t nowUs) {
    (void)nowUs;
    pressurePre.buffer[pressurePre.head] = pressurePa;
    pressurePre.head = (pressurePre.head + 1) % BARO_MEDIAN_SIZE;
    if (pressurePre.count < BARO_MEDIAN_SIZE) pressurePre.count++;

    float sorted[BARO_MEDIAN_SIZE];
    for (uint8_t i = 0; i < pressurePre.count; i++) {
        sorted[i] = pressurePre.buffer[i];
    }
    for (uint8_t i = 1; i < pressurePre.count; i++) {
        float key = sorted[i];
        int8_t j = (int8_t)i - 1;
        while (j >= 0 && sorted[j] > key) {
            sorted[j + 1] = sorted[j];
            j--;
        }
        sorted[j + 1] = key;
    }
    float measurement = sorted[pressurePre.count / 2];

    if (!pressurePre.init) {
        pressurePre.estimate = measurement;
        pressurePre.covariance = 9.0f;
        pressurePre.init = true;
        return pressurePre.estimate;
    }

    pressurePre.covariance += BARO_KF_PROCESS_NOISE;
    const float innovationVariance = pressurePre.covariance + BARO_KF_MEASUREMENT_NOISE;
    const float gain = pressurePre.covariance / innovationVariance;
    pressurePre.estimate += gain * (measurement - pressurePre.estimate);
    pressurePre.covariance *= (1.0f - gain);

    return pressurePre.estimate;
}

// ---------- ALTITUDE KALMAN ----------
void altitudeKFReset() {
    altKF = AltitudeKF();
    pressurePrefilterReset();
    filteredAltitude = 0.0f;
    filteredVelocity = 0.0f;
}

void altitudeKFUpdate(float measurement, uint32_t nowUs) {
    if (!altKF.init) {
        altKF.h = measurement;
        altKF.v = 0.0f;
        altKF.p00 = 1.0f; altKF.p01 = 0.0f; altKF.p10 = 0.0f; altKF.p11 = 1.0f;
        altKF.lastUs = nowUs;
        altKF.init = true;
        return;
    }

    float dt = (uint32_t)(nowUs - altKF.lastUs) * 1.0e-6f;
    altKF.lastUs = nowUs;
    if (dt < 0.001f) dt = 0.001f;
    if (dt > 0.5f) dt = 0.5f;

    const float dt2 = dt * dt;
    const float q = ALT_KF_ACCEL_SIGMA * ALT_KF_ACCEL_SIGMA;
    const float r = ALT_KF_MEAS_SIGMA * ALT_KF_MEAS_SIGMA;

    altKF.h += altKF.v * dt;
    float p00 = altKF.p00 + dt * (altKF.p10 + altKF.p01) + dt2 * altKF.p11 + q * dt2 * dt2 * 0.25f;
    float p01 = altKF.p01 + dt * altKF.p11 + q * dt2 * dt * 0.5f;
    float p10 = altKF.p10 + dt * altKF.p11 + q * dt2 * dt * 0.5f;
    float p11 = altKF.p11 + q * dt2;

    float s = p00 + r;
    float k0 = p00 / s;
    float k1 = p10 / s;
    float innovation = measurement - altKF.h;
    altKF.h += k0 * innovation;
    altKF.v += k1 * innovation;
    altKF.p00 = p00 - k0 * p00;
    altKF.p01 = p01 - k0 * p01;
    altKF.p10 = p10 - k1 * p00;
    altKF.p11 = p11 - k1 * p01;
}

bool calibrateAltitudeZero() {
    if (!baroInitialized) return false;

    Serial.println("\nKeep the barometer still at the reference height...");
    double pressureSum = 0.0;
    float pMin = 1.0e9f, pMax = 0.0f;
    uint16_t samples = 0;
    uint32_t start = millis();

    while (samples < BMP_CAL_SAMPLES && (uint32_t)(millis() - start) < BMP_CAL_TIMEOUT_MS) {
        float pressure, temperature;
        bool fresh;
        if (readBaro(pressure, temperature, fresh) && fresh) {
            pressureSum += pressure;
            if (pressure < pMin) pMin = pressure;
            if (pressure > pMax) pMax = pressure;
            samples++;
        }
        delay(5);
    }

    if (samples < BMP_CAL_SAMPLES) {
        Serial.printf("Altitude calibration failed: only %u/%u samples collected (timeout).\n",
                      samples, BMP_CAL_SAMPLES);
        return false;
    }
    if (pMax - pMin > BMP_CAL_MAX_SPREAD_PA) {
        Serial.printf("Altitude calibration rejected: pressure moved %.2f Pa (limit %.2f Pa). Please keep board still.\n",
                      pMax - pMin, BMP_CAL_MAX_SPREAD_PA);
        return false;
    }

    pressureZeroPa = (float)(pressureSum / samples);
    altitudeCalibrated = true;
    altitudeKFReset();
    peakAltitude = 0.0f;
    stateReferenceValid = false;

    eepromHeader.pressureZeroPa = pressureZeroPa;
    eepromHeader.inFlight = 0;
    eepromHeader.flightState = STATE_ARMING;
    eepromHeader.peakAltitude = 0.0f;
    eepromHeader.mosfetTriggered = 0;
    saveEepromHeader();

    Serial.printf("Altitude reference pressure: %.1f Pa (spread %.2f Pa) [Saved to EEPROM]\n",
                  pressureZeroPa, pMax - pMin);
    return true;
}

void updateBaro() {
    if (!baroInitialized) {
        filteredBMPValid = false;
        return;
    }

    float pressure, temperature;
    bool fresh;
    if (!readBaro(pressure, temperature, fresh)) {
        if (baroFailCount < UINT16_MAX) baroFailCount++;
        if (baroFailCount >= BARO_FAIL_LIMIT) {
            filteredBMPValid = false;
            baroInitialized = false;
            Serial.println("Barometer read failures; will re-initialize.");
        }
        return;
    }
    baroFailCount = 0;
    if (!fresh) return;

    uint32_t nowUs = micros();
    baroPressurePa = pressure;
    baroTempC = temperature;
    baroPressureSmoothPa = pressurePrefilterUpdate(pressure, nowUs);
    filteredBMPValid = true;
    lastBaroSampleMs = millis();

    if (!altitudeCalibrated) return;

    altitudeKFUpdate(pressureToAltitude(baroPressureSmoothPa), nowUs);
    filteredAltitude = altKF.h;
    filteredVelocity = altKF.v;
    if (filteredAltitude > peakAltitude) peakAltitude = filteredAltitude;
}

// =========================================================================
//                         ACTUATOR / MOSFET TIMING
// =========================================================================
void updateActuator(uint32_t now) {
    if (apogeeSignalActive) {
        if (filteredAltitude <= MOSFET_MIN_ALTITUDE_M) {
            digitalWrite(MOSFET_SIGNAL, LOW);
            apogeeSignalActive = false;
            Serial.println("MOSFET: Emergency cutoff - altitude at or below 30 m safety floor.");
            return;
        }

        if ((uint32_t)(now - mosfetSignalStartTime) >= APOGEE_SIGNAL_DURATION_MS) {
            digitalWrite(MOSFET_SIGNAL, LOW);
            apogeeSignalActive = false;
            Serial.println("MOSFET deactivated (LOW) after pulse duration.");
        }
    }

    if (manualSignalActive) {
        if ((uint32_t)(now - mosfetSignalStartTime) >= APOGEE_SIGNAL_DURATION_MS) {
            digitalWrite(MOSFET_SIGNAL, LOW);
            manualSignalActive = false;
            Serial.println("MOSFET (Remote/Manual): deactivated (LOW) after pulse duration.");
        }
    }
}

// =========================================================================
//                                FLIGHT STATE
// =========================================================================
void updateFlightState(uint32_t now) {
    if (flightState == STATE_STARTUP) return;

    if (!stateReferenceValid) {
        previousSampleAltitude = filteredAltitude;
        windowAltitude = filteredAltitude;
        windowTime = now;
        maximumAltitude = filteredAltitude;
        stateReferenceValid = true;
        return;
    }

    float delta = filteredAltitude - previousSampleAltitude;
    previousSampleAltitude = filteredAltitude;
    
    if (filteredAltitude > maximumAltitude) {
        maximumAltitude = filteredAltitude;
        if (flightState >= STATE_BOOST && (maximumAltitude - eepromHeader.peakAltitude >= 50.0f)) {
            eepromHeader.peakAltitude = maximumAltitude;
            saveEepromHeader();
        }
    }

    bool windowReady = false;
    float windowDelta = 0.0f;
    if ((uint32_t)(now - windowTime) >= STATE_WINDOW_MS) {
        windowDelta = filteredAltitude - windowAltitude;
        windowAltitude = filteredAltitude;
        windowTime = now;
        windowReady = true;
    }

    switch (flightState) {
        case STATE_ARMING:
            if (delta > ALTITUDE_DEADBAND_M) {
                if (boostCount < UINT8_MAX) boostCount++;
            } else {
                boostCount = 0;
            }
            if (boostCount >= BOOST_ASCENT_SAMPLES &&
                filteredAltitude >= BOOST_MIN_ALTITUDE_M) {
                flightState = STATE_BOOST;
                descentCount = 0;
                maximumAltitude = filteredAltitude;

                eepromHeader.inFlight = 1;
                eepromHeader.flightState = STATE_BOOST;
                eepromHeader.peakAltitude = maximumAltitude;
                saveEepromHeader();

                Serial.println("FLIGHT STATE: BOOST [Saved to EEPROM]");
            }
            break;

        case STATE_BOOST:
            if (delta < -ALTITUDE_DEADBAND_M) {
                if (descentCount < UINT8_MAX) descentCount++;
            } else {
                descentCount = 0;
            }
            if (descentCount >= APOGEE_DESCENT_SAMPLES &&
                maximumAltitude - filteredAltitude >= APOGEE_MIN_DROP_M) {
                flightState = STATE_APOGEE;
                stableCount = 0;
                eepromHeader.flightState = STATE_APOGEE;
                eepromHeader.peakAltitude = maximumAltitude;

                if (filteredAltitude > MOSFET_MIN_ALTITUDE_M) {
                    digitalWrite(MOSFET_SIGNAL, HIGH);
                    apogeeSignalActive = true;
                    mosfetTriggered = true;
                    mosfetSignalStartTime = now;
                    eepromHeader.mosfetTriggered = 1;
                    Serial.println("FLIGHT STATE: APOGEE - MOSFET TRIGGERED (HIGH)");
                } else {
                    digitalWrite(MOSFET_SIGNAL, LOW);
                    apogeeSignalActive = false;
                    Serial.printf("FLIGHT STATE: APOGEE - MOSFET INHIBITED: altitude (%.1f m) <= %.1f m safety floor!\n",
                                  filteredAltitude, MOSFET_MIN_ALTITUDE_M);
                }
                saveEepromHeader();
            }
            break;

        case STATE_APOGEE:
            if (windowReady) {
                if (fabsf(windowDelta) < LANDING_STABLE_BAND_M &&
                    maximumAltitude - filteredAltitude >= LANDING_MIN_DROP_M) {
                    if (stableCount < UINT16_MAX) stableCount++;
                } else {
                    stableCount = 0;
                }
                if (stableCount >= LANDING_STABLE_WINDOWS) {
                    flightState = STATE_LANDED;
                    eepromHeader.inFlight = 0;
                    eepromHeader.flightState = STATE_LANDED;
                    saveEepromHeader();
                    Serial.println("FLIGHT STATE: LANDED [Saved to EEPROM]");
                }
            }
            break;

        default:
            break;
    }
}

void enterArmingIfReady() {
    if (flightState == STATE_STARTUP && baroInitialized && altitudeCalibrated) {
        flightState = STATE_ARMING;
        boostCount = 0;
        stateReferenceValid = false;
        Serial.println("FLIGHT STATE: ARMING");
    }
}

void maintainSensors(uint32_t now) {
    if (flightState > STATE_ARMING || inFlightRecovery) return;

    if (baroInitialized && altitudeCalibrated && flightState != STATE_STARTUP) {
        if (!mpuInitialized && (uint32_t)(now - lastSensorRetry) >= SENSOR_RETRY_MS) {
            lastSensorRetry = now;
            if (initializeMPU(false)) {
                calibrateMPU();
            }
        }
        return;
    }

    if ((uint32_t)(now - lastSensorRetry) < SENSOR_RETRY_MS) return;
    lastSensorRetry = now;
    sensorRetryCount++;

    if (!baroInitialized && (sensorRetryCount % 3 == 0)) {
        Wire.end();
        i2cBusRecovery();
        startI2C();
    }

    if (!mpuInitialized && initializeMPU(false)) {
        calibrateMPU();
    }
    if (!baroInitialized) {
        initializeBaro(false);
    }
    if (baroInitialized && !altitudeCalibrated) {
        calibrateAltitudeZero();
    }
    enterArmingIfReady();
    lastImuMicros = micros();
}

// =========================================================================
//                                 GPS
// =========================================================================
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

struct GpsSnapshot {
    bool valid = false;
    int32_t latE7 = 0, lonE7 = 0, altCm = 0;
    uint8_t satellites = 0;
};

static GpsSnapshot readGpsSnapshot() {
    GpsSnapshot s;
    if (gpsInitialized && gps.location.isValid() &&
        gps.location.age() <= GPS_MAX_AGE_MS) {
        double latitude = gps.location.lat();
        double longitude = gps.location.lng();

        if (isfinite(latitude) && isfinite(longitude)) {
            s.latE7 = (int32_t)lround(latitude * 10000000.0);
            s.lonE7 = (int32_t)lround(longitude * 10000000.0);
            s.altCm = gps.altitude.isValid() && isfinite(gps.altitude.meters())
                ? (int32_t)lround(gps.altitude.meters() * 100.0)
                : 0;
            s.satellites = gps.satellites.isValid()
                ? (uint8_t)gps.satellites.value()
                : 0;
            s.valid = true;
        }
    }
    return s;
}

// =========================================================================
//                              ESP-NOW HELPERS
// =========================================================================
bool addPeer(const uint8_t *mac) {
    if (esp_now_is_peer_exist(mac)) return true;

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, mac, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.encrypt = false;
    peer.ifidx = WIFI_IF_STA;

    esp_err_t result = esp_now_add_peer(&peer);
    if (result != ESP_OK) {
        Serial.printf("Failed to add peer: %s\n", esp_err_to_name(result));
        return false;
    }
    return true;
}

static void handleControl(const uint8_t *mac, const uint8_t *data, int len) {
    if (!data || len <= 0) return;

    // 1. Binary ControlMessage
    if (len == sizeof(ControlMessage)) {
        ControlMessage msg;
        memcpy(&msg, data, sizeof(msg));

        if (msg.magic == MAGIC && msg.version == VERSION) {
            portENTER_CRITICAL(&mux);
            memcpy(pendingSenderMAC, mac, 6);
            if (msg.type == MSG_RESPONSE) {
                responsePending = true;
            } else if (msg.type == MSG_MOSFET_TRIGGER) {
                remoteTriggerPending = true;
            } else if (msg.type == MSG_MOSFET_CUTOFF) {
                remoteCutoffPending = true;
            }
            portEXIT_CRITICAL(&mux);
            return;
        }
    }

    // 2. ASCII String Commands (e.g. "FIRE", "TRIGGER", "MOSFET_HIGH", "OFF", "LOW")
    if (len >= 2 && len <= 32) {
        char cmdBuf[33];
        memcpy(cmdBuf, data, len);
        cmdBuf[len] = '\0';

        // Trim trailing newline, carriage return, or spaces
        for (int i = len - 1; i >= 0; i--) {
            if (cmdBuf[i] == '\r' || cmdBuf[i] == '\n' || cmdBuf[i] == ' ') {
                cmdBuf[i] = '\0';
            } else {
                break;
            }
        }

        if (strcasecmp(cmdBuf, "FIRE") == 0 ||
            strcasecmp(cmdBuf, "TRIGGER") == 0 ||
            strcasecmp(cmdBuf, "MOSFET_ON") == 0 ||
            strcasecmp(cmdBuf, "MOSFET_HIGH") == 0 ||
            strcasecmp(cmdBuf, "HIGH") == 0 ||
            strcasecmp(cmdBuf, "ON") == 0) {
            portENTER_CRITICAL(&mux);
            memcpy(pendingSenderMAC, mac, 6);
            remoteTriggerPending = true;
            portEXIT_CRITICAL(&mux);
            return;
        }

        if (strcasecmp(cmdBuf, "OFF") == 0 ||
            strcasecmp(cmdBuf, "LOW") == 0 ||
            strcasecmp(cmdBuf, "MOSFET_OFF") == 0 ||
            strcasecmp(cmdBuf, "MOSFET_LOW") == 0) {
            portENTER_CRITICAL(&mux);
            memcpy(pendingSenderMAC, mac, 6);
            remoteCutoffPending = true;
            portEXIT_CRITICAL(&mux);
            return;
        }
    }
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    handleControl(info->src_addr, data, len);
}
#else
void onReceive(const uint8_t *mac, const uint8_t *data, int len) {
    handleControl(mac, data, len);
}
#endif

void processDiscoveryResponse() {
    uint8_t discoveredMAC[6];
    bool pending = false;

    portENTER_CRITICAL(&mux);
    if (responsePending) {
        memcpy(discoveredMAC, pendingSenderMAC, 6);
        responsePending = false;
        pending = true;
    }
    portEXIT_CRITICAL(&mux);

    if (!pending) return;

    if (!addPeer(discoveredMAC)) {
        Serial.println("Could not add discovered receiver.");
        return;
    }

    memcpy(receiverMAC, discoveredMAC, 6);
    receiverConnected = true;

    Serial.printf("\nReceiver discovered: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  receiverMAC[0], receiverMAC[1], receiverMAC[2],
                  receiverMAC[3], receiverMAC[4], receiverMAC[5]);
}

// =========================================================================
//                                 TELEMETRY
// =========================================================================
int buildTelemetry(char *buffer, size_t bufferSize, uint16_t seq) {
    int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    int16_t bmpTempCenti = 0, mpuTempCenti = 0;
    uint32_t pressurePa = 0;
    int32_t relAltCm = 0;
    uint8_t flags = 0;
    int16_t rollX10 = 0, pitchX10 = 0, yawX10 = 0;

    if (filteredMPUValid) {
        ax = filteredAccel[0];
        ay = filteredAccel[1];
        az = filteredAccel[2];
        gx = filteredGyro[0];
        gy = filteredGyro[1];
        gz = filteredGyro[2];
        flags |= FLAG_MPU_VALID;
        if (isfinite(filteredMpuTemperature) &&
            filteredMpuTemperature * 100.0f >= -32768.0f &&
            filteredMpuTemperature * 100.0f <= 32767.0f) {
            mpuTempCenti = (int16_t)lroundf(filteredMpuTemperature * 100.0f);
        }
        rollX10 = (int16_t)lroundf(rollDeg * 10.0f);
        pitchX10 = (int16_t)lroundf(pitchDeg * 10.0f);
        yawX10 = (int16_t)lroundf(yawDeg * 10.0f);
    }

    if (filteredBMPValid) {
        bmpTempCenti = (int16_t)lroundf(baroTempC * 100.0f);
        pressurePa = (uint32_t)lroundf(baroPressureSmoothPa);
        flags |= FLAG_BMP_VALID;
    }

    if (altitudeCalibrated && filteredBMPValid &&
        isfinite(filteredAltitude) &&
        filteredAltitude * 100.0f >= -2147483648.0f &&
        filteredAltitude * 100.0f <= 2147483647.0f) {
        relAltCm = (int32_t)lroundf(filteredAltitude * 100.0f);
        flags |= FLAG_ALTITUDE_VALID;
    }

    GpsSnapshot g = readGpsSnapshot();
    if (g.valid) flags |= FLAG_GPS_VALID;

    const uint8_t mosfetState = (apogeeSignalActive || manualSignalActive) ? 1 : (mosfetTriggered ? 2 : 0);

    int length = snprintf(
        buffer, bufferSize,
        "%08lX;%u;%u;%u;%u;%d;%d;%d;%d;%d;%d;%d;%lu;%ld;%ld;%ld;%u;%u;%u;%ld;%d",
        (unsigned long)MAGIC,
        (unsigned)TELEMETRY_VERSION,
        (unsigned)seq,
        (unsigned)flightState,
        (unsigned)mosfetState,
        ax, ay, az,
        gx, gy, gz,
        bmpTempCenti,
        (unsigned long)pressurePa,
        (long)g.latE7, (long)g.lonE7, (long)g.altCm,
        (unsigned)g.satellites,
        (unsigned)flags,
        (unsigned)flightState,
        (long)relAltCm,
        mpuTempCenti);

    if (length <= 0 || length >= (int)bufferSize) return 0;

#if SEND_ATTITUDE
    int extra = snprintf(buffer + length, bufferSize - length,
                         ";%d;%d;%d", rollX10, pitchX10, yawX10);
    if (extra <= 0 || length + extra >= (int)bufferSize) return 0;
    length += extra;
#endif

    return length;
}

static uint8_t gcsPhase(FlightState s) {
    switch (s) {
        case STATE_STARTUP: return 0;
        case STATE_ARMING:  return 1;
        case STATE_BOOST:   return 2;
        case STATE_APOGEE:  return 4;
        case STATE_LANDED:  return 6;
    }
    return 0;
}

static float wrap180(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a <= -180.0f) a += 360.0f;
    return a;
}

static void fillGcsPacket(GcsTelemetryPacket &p, uint32_t id) {
    memset(&p, 0, sizeof(p));
    p.header = GCS_HEADER;
    p.packet_id = id;
    p.timestamp = millis();
    p.flight_phase = gcsPhase(flightState);

    if (filteredBMPValid) {
        p.pressure = baroPressureSmoothPa / 100.0f;
        p.temperature = baroTempC;
        if (altitudeCalibrated) {
            p.altitude = filteredAltitude;
            p.velocity = filteredVelocity;
            p.max_altitude = peakAltitude;
        }
    }

    if (filteredMPUValid) {
        p.accel_x = linAccelMs2[0];
        p.accel_y = linAccelMs2[1];
        p.accel_z = linAccelMs2[2];
        p.roll = rollDeg;
        p.pitch = pitchDeg;
        p.yaw = wrap180(yawDeg);
    }

    GpsSnapshot g = readGpsSnapshot();
    if (g.valid) {
        p.gps_lat = g.latE7;
        p.gps_lon = g.lonE7;
        p.gps_altitude = g.altCm / 100.0f;
        p.gps_satellites = g.satellites;
        p.gps_fix = (g.satellites >= 4) ? 1 : 0;
    }

    uint8_t sum = 0;
    const uint8_t *bytes = (const uint8_t *)&p;
    for (size_t i = 0; i < offsetof(GcsTelemetryPacket, checksum); i++) sum += bytes[i];
    p.checksum = sum;
    p.footer1 = GCS_FOOTER_1;
    p.footer2 = GCS_FOOTER_2;
}

bool sendTelemetry() {
    esp_err_t result;

#if ESPNOW_TELEMETRY_MODE == TELEMETRY_MODE_GCS_BINARY
    GcsTelemetryPacket packet;
    fillGcsPacket(packet, gcsPacketId++);
    result = esp_now_send(receiverMAC, (const uint8_t *)&packet, sizeof(packet));
#else
    char buffer[250];
    int length = buildTelemetry(buffer, sizeof(buffer), sequenceNumber++);
    if (length <= 0) return false;
    result = esp_now_send(receiverMAC, (const uint8_t *)buffer, length);
#endif

    if (result != ESP_OK) {
        if ((uint32_t)(millis() - lastSendErrorPrint) >= 1000) {
            lastSendErrorPrint = millis();
            Serial.printf("Telemetry send failed: %s\n", esp_err_to_name(result));
        }
        return false;
    }
    return true;
}

#if SERIAL_TELEMETRY
void printSerialTelemetry() {
    static uint16_t serialSequence = 0;
    char buffer[250];
    int length = buildTelemetry(buffer, sizeof(buffer), serialSequence++);
    if (length > 0) {
        Serial.println(buffer);
    }
}
#endif

// =========================================================================
//                   REMOTE COMMANDS & TRANSCEIVER LOGIC
// =========================================================================
void sendMosfetAck(uint8_t ackType) {
    if (!receiverConnected) return;

    ControlMessage ackMsg;
    ackMsg.magic = MAGIC;
    ackMsg.type = MSG_COMMAND_ACK;
    ackMsg.version = VERSION;
    esp_now_send(receiverMAC, (const uint8_t *)&ackMsg, sizeof(ackMsg));

    // Instantly transmit updated telemetry so the receiver sees mosfetState update immediately
    sendTelemetry();
}

void triggerMosfetRemote(uint32_t now) {
    digitalWrite(MOSFET_SIGNAL, HIGH);
    manualSignalActive = true;
    mosfetTriggered = true;
    mosfetSignalStartTime = now;
    eepromHeader.mosfetTriggered = 1;
    saveEepromHeader();

    Serial.println("\n=======================================================");
    Serial.println(">>> [RADIO COMMAND] REMOTE MOSFET TRIGGERED (PIN 26 HIGH) <<<");
    Serial.printf( ">>> Pulse Duration: %lu ms | Current Alt: %.2f m       <<<\n",
                  (unsigned long)APOGEE_SIGNAL_DURATION_MS, filteredAltitude);
    Serial.println("=======================================================\n");

    sendMosfetAck(MSG_MOSFET_TRIGGER);
}

void cutoffMosfetRemote() {
    digitalWrite(MOSFET_SIGNAL, LOW);
    manualSignalActive = false;
    apogeeSignalActive = false;

    Serial.println("\n[RADIO COMMAND] REMOTE MOSFET CUTOFF -> PIN 26 SET LOW.");
    sendMosfetAck(MSG_MOSFET_CUTOFF);
}

void processRemoteCommands(uint32_t now) {
    uint8_t sender[6];
    bool trigger = false;
    bool cutoff = false;

    portENTER_CRITICAL(&mux);
    if (remoteTriggerPending) {
        memcpy(sender, pendingSenderMAC, 6);
        remoteTriggerPending = false;
        trigger = true;
    }
    if (remoteCutoffPending) {
        memcpy(sender, pendingSenderMAC, 6);
        remoteCutoffPending = false;
        cutoff = true;
    }
    portEXIT_CRITICAL(&mux);

    if (trigger) {
        if (!receiverConnected || memcmp(receiverMAC, sender, 6) != 0) {
            addPeer(sender);
            memcpy(receiverMAC, sender, 6);
            receiverConnected = true;
            Serial.printf("[ESP-NOW] Paired with command sender MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                          sender[0], sender[1], sender[2], sender[3], sender[4], sender[5]);
        }
        triggerMosfetRemote(now);
    } else if (cutoff) {
        if (!receiverConnected || memcmp(receiverMAC, sender, 6) != 0) {
            addPeer(sender);
            memcpy(receiverMAC, sender, 6);
            receiverConnected = true;
        }
        cutoffMosfetRemote();
    }
}

// =========================================================================
//                                   SETUP
// =========================================================================
void setup() {
    pinMode(MOSFET_SIGNAL, OUTPUT);
    digitalWrite(MOSFET_SIGNAL, LOW);

    Serial.setTxBufferSize(1024);
    Serial.begin(115200);
    delay(500);

    Serial.println("\n=== ESP-NOW ROCKET AVIONICS ===");
    
    // 1. Initialize persistent EEPROM & check for mid-air restart
    initEeprom();

#if !SKIP_I2C
    // 2. Hardware bus recovery and scan
    i2cBusRecovery();
    i2cScan();

    for (uint8_t attempt = 0; attempt < 5; attempt++) {
        if (!mpuInitialized) initializeMPU(attempt == 4);
        if (!baroInitialized) initializeBaro(attempt == 4);
        if (mpuInitialized && baroInitialized) break;
        delay(400);
    }
    initializeGPS();

    if (mpuInitialized) {
        if (!calibrateMPU()) {
            Serial.println("MPU calibration incomplete; using uncorrected gyro readings.");
        }
    } else {
        Serial.println("MPU6050 NOT available - check wiring / replacement.");
    }

    // 3. Barometer initialization & Pad vs Recovery Altitude setup
    if (baroInitialized) {
        if (!inFlightRecovery) {
            if (!calibrateAltitudeZero()) {
                Serial.println("Altitude calibration failed - will retry on pad.");
            }
        } else {
            float p, t;
            bool fresh;
            if (readBaro(p, t, fresh)) {
                baroPressureSmoothPa = p;
                float currentAlt = pressureToAltitude(p);
                altKF.h = currentAlt;
                altKF.v = 0.0f;
                altKF.init = true;
                filteredAltitude = currentAlt;
                Serial.printf("[RECOVERY] Barometer primed at active altitude: %.1f m\n", currentAlt);
            }
        }
    } else {
        Serial.println("Barometer NOT available - will keep retrying.");
    }
#else
    initializeGPS();
    Serial.println("\n[I2C] *** SKIP_I2C ENABLED (Ejection Test Mode) ***");
    Serial.println("[I2C] Bypassing I2C bus & sensor scanning.");
    Serial.println("[I2C] Telemetry sensor readings will default to 0.");
    flightState = STATE_ARMING;
#endif

    // 4. Radio setup (Long Range Mode + Maximum TX Power)
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    WiFi.setSleep(false); // Keep radio active at peak performance

    Serial.print("Transmitter MAC: ");
    Serial.println(WiFi.macAddress());

#if ESPNOW_FORCE_MAX_TX_POWER
    // 84 corresponds to +20.5 dBm (max possible on ESP32 silicon in 0.25 dBm increments)
    // Fall back to 78 (+19.5 dBm) if chip calibration registers reject 84.
    if (esp_wifi_set_max_tx_power(84) != ESP_OK) {
        esp_wifi_set_max_tx_power(78);
    }
    int8_t actualPower = 0;
    esp_wifi_get_max_tx_power(&actualPower);
    Serial.printf("[RADIO] Maximum TX Power configured: %.2f dBm (raw: %d)\n",
                  actualPower * 0.25f, actualPower);
#endif

#if ESPNOW_LR_MODE
    // Switch Station interface physical layer to Espressif Long Range (LR) mode
    esp_err_t protoErr = esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
    if (protoErr == ESP_OK) {
        Serial.println("[RADIO] Espressif Long Range (LR) Mode ENABLED (Huge Range Boost).");
        Serial.println("[RADIO] NOTE: Receiver/GCS ESP32 must also have WIFI_PROTOCOL_LR enabled!");
    } else {
        Serial.printf("[RADIO] WARNING: Failed to enable LR Mode: %s\n", esp_err_to_name(protoErr));
    }
#endif

    esp_err_t channelResult = esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    if (channelResult != ESP_OK) {
        Serial.printf("Channel setup failed: %s\n", esp_err_to_name(channelResult));
    }

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW initialization failed!");
        while (true) delay(1000);
    }

#if ESPNOW_TELEMETRY_MODE == TELEMETRY_MODE_GCS_BINARY
    if (addPeer(gcsMacAddress)) {
        memcpy(receiverMAC, gcsMacAddress, 6);
        receiverConnected = true;
    } else {
        while (true) delay(1000);
    }
#else
    if (!addPeer(broadcastMAC)) {
        while (true) delay(1000);
    }
    Serial.println("Waiting for receiver discovery...");
#endif

    // Register receive callback so this node can receive commands in ALL modes
    esp_now_register_recv_cb(onReceive);

    if (!inFlightRecovery) {
        enterArmingIfReady();
    } else {
        Serial.printf("[RECOVERY] Flight resumed in phase: %u\n", (unsigned)flightState);
    }

    lastImuMicros = micros();
    lastSensorRetry = millis();
    lastEepromLog = millis();

    Serial.println("[CMD] Commands: 'D'=dump EEPROM | 'C'=clear EEPROM | 'F'=fire MOSFET | 'O'=MOSFET off");
}

// =========================================================================
//                                    LOOP
// =========================================================================
void loop() {
    uint32_t now = millis();

    // Check serial for user commands (e.g. 'D' for dump log, 'C' for clear, 'F' for fire test)
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'D' || c == 'd') {
            dumpEepromLog();
        } else if (c == 'C' || c == 'c') {
            clearEepromLog();
        } else if (c == 'F' || c == 'f') {
            Serial.println("[SERIAL CMD] Manual MOSFET fire test initiated...");
            triggerMosfetRemote(now);
        } else if (c == 'O' || c == 'o') {
            Serial.println("[SERIAL CMD] Manual MOSFET cutoff...");
            cutoffMosfetRemote();
        }
    }

    updateGPS();

#if !SKIP_I2C
    updateMPU();

    if (now - lastBaroPoll >= BARO_POLL_MS) {
        lastBaroPoll = now;
        updateBaro();
    }
#endif

    // In-flight recovery safety check
    if (inFlightRecovery && flightState == STATE_APOGEE && !mosfetTriggered) {
        if (filteredAltitude > MOSFET_MIN_ALTITUDE_M) {
            digitalWrite(MOSFET_SIGNAL, HIGH);
            apogeeSignalActive = true;
            mosfetTriggered = true;
            mosfetSignalStartTime = now;
            eepromHeader.mosfetTriggered = 1;
            saveEepromHeader();
            Serial.println("[RECOVERY] Emergency late apogee deployment triggered!");
        }
    }

    // Flight logic (20 Hz)
    if (altitudeCalibrated && filteredBMPValid &&
        (uint32_t)(now - lastStateStep) >= STATE_STEP_MS) {
        lastStateStep = now;
        if ((uint32_t)(now - lastBaroSampleMs) < BARO_STALE_MS) {
            updateFlightState(now);
        }
    }

    // Manage MOSFET shut-off timer (turns pin LOW after 5 s or <= 30 m)
    updateActuator(now);

    // Save periodic circular snapshot (every 250 ms = 4 Hz)
    if (now - lastEepromLog >= EEPROM_LOG_INTERVAL_MS) {
        lastEepromLog = now;
        logEepromSample(now);
    }

#if !SKIP_I2C
    maintainSensors(now);
#endif

#if ESPNOW_TELEMETRY_MODE == TELEMETRY_MODE_TEXT_DISCOVERY
    processDiscoveryResponse();
#endif

    // Process any remote commands from the receiver/ground station
    processRemoteCommands(now);

#if SERIAL_TELEMETRY
    static uint32_t lastSerialTelemetry = 0;
    if (now - lastSerialTelemetry >= SERIAL_TELEMETRY_INTERVAL_MS) {
        lastSerialTelemetry = now;
        printSerialTelemetry();
    }
#endif

#if ESPNOW_TELEMETRY_MODE == TELEMETRY_MODE_TEXT_DISCOVERY
    if (!receiverConnected) {
        if (now - lastDiscovery >= DISCOVERY_INTERVAL_MS) {
            lastDiscovery = now;
            ControlMessage discovery = {MAGIC, MSG_DISCOVERY, VERSION};
            esp_now_send(broadcastMAC, (const uint8_t *)&discovery, sizeof(discovery));
        }
        return;
    }
#endif

    if (receiverConnected && now - lastSend >= SEND_INTERVAL_MS) {
        lastSend = now;
        sendTelemetry();
    }
}