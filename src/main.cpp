#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_BMP280.h>
#include <math.h>

// =====================================================
//  AVIONICS SENSOR & LORA TELEMETRY + FLIGHT STATE
//
//  Packet LoRa (1 baris, pemisah ";"):
//    state;timestamp;ax;ay;az;gx;gy;gz;imuTemp;bmpTemp;pressure;altitude
//    State (0-4) selalu di awal dan ikut terkirim di setiap paket.
//
//  State:
//    0 = STARTUP : kalibrasi semua sensor
//    1 = ARMING  : kalibrasi selesai, sensor terbaca,
//                  telemetry LoRa sudah berjalan
//    2 = BOOST   : ketinggian naik pesat & stabil
//    3 = APOGEE  : 25 sampel turun berturut-turut
//                  -> GPIO26 (MOSFET_SIGNAL) HIGH
//    4 = LANDING : data ketinggian relatif stabil
// =====================================================


// I2C
#define SDA_PIN 21
#define SCL_PIN 22

#define IMU_ADDR 0x68
#define BMP_ADDR 0x76

// LoRa SX1278
#define LORA_SCK  18
#define LORA_MISO 19
#define LORA_MOSI 23

#define LORA_CS   5
#define LORA_RST  14
#define LORA_DIO0 32

#define LORA_FREQUENCY 433500000L

// Test LED (sinyal apogee)
#define MOSFET_SIGNAL 26

// =====================================================
// SYSTEM TIMING
// =====================================================

const uint32_t SENSOR_INTERVAL_MS = 100;     // 10 Hz
const uint32_t TELEMETRY_INTERVAL_MS = 200;  // 5 Hz

// =====================================================
// CALIBRATION SETTINGS
// =====================================================

const uint16_t IMU_CAL_SAMPLES = 250;
const uint16_t IMU_CAL_DELAY_MS = 10;

const uint16_t BMP_CAL_SAMPLES = 40;
const uint16_t BMP_CAL_DELAY_MS = 120;

const float GRAVITY = 9.80665f;

// =====================================================
// KALMAN FILTER SETTINGS
// =====================================================

const float KALMAN_Q = 0.6f;
const float KALMAN_R = 1.5f;

// =====================================================
// FLIGHT STATE SETTINGS  (sesuaikan saat uji)
// =====================================================

enum FlightState : uint8_t
{
    STATE_STARTUP = 0,
    STATE_ARMING  = 1,
    STATE_BOOST   = 2,
    STATE_APOGEE  = 3,
    STATE_LANDING = 4
};

// --- Boost ---
// Naik lebih cepat dari ini (m/s) ...
const float    BOOST_VELOCITY_MPS     = 3.0f;
// ... selama sekian sampel berturut-turut (10 Hz -> 0.5 s)
const uint8_t  BOOST_CONFIRM_SAMPLES  = 5;
// Dan ketinggian minimal di atas titik nol (m)
const float    BOOST_MIN_ALTITUDE_M   = 1.0f;

// --- Apogee ---
// Jumlah penurunan berturut-turut
const uint8_t  APOGEE_DESCENT_SAMPLES = 25;
// Perubahan lebih kecil dari ini dianggap noise (m)
const float    ALTITUDE_DEADBAND_M    = 0.02f;

// --- LED apogee ---
// Lama GPIO26 (MOSFET_SIGNAL) menyala HIGH saat apogee (ms)
const uint32_t APOGEE_MOSFET_DURATION_MS = 3000;

// --- Landing ---
// Perubahan antar sampel dianggap "stabil" jika < band (m)
const float    LANDING_STABLE_BAND_M  = 0.30f;
// Stabil selama sekian sampel berturut-turut (10 Hz -> 3 s)
const uint16_t LANDING_STABLE_SAMPLES = 30;
// Minimal sudah turun sekian meter dari puncak
const float    LANDING_MIN_DROP_M     = 2.0f;

// =====================================================
// TIMERS
// =====================================================

uint32_t lastSensorTime = 0;
uint32_t lastTelemetryTime = 0;
uint32_t lastKalmanTime = 0;

// =====================================================
// SENSOR OBJECTS
// =====================================================

Adafruit_BMP280 bmp;

uint8_t imuChipID = 0;

bool sensorDataValid = false;

// =====================================================
// FLIGHT STATE VARIABLES
// =====================================================

FlightState flightState = STATE_STARTUP;

bool stateRefValid = false;
float prevStateAltitude = 0.0f;
uint32_t prevStateTime = 0;
float maxAltitude = 0.0f;

uint8_t boostCount = 0;
uint8_t descentCount = 0;
uint16_t stableCount = 0;

bool apogeeLedActive = false;
uint32_t apogeeLedStartTime = 0;

// =====================================================
// IMU CALIBRATION VARIABLES
// =====================================================

float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;

float accelBiasX = 0.0f;
float accelBiasY = 0.0f;
float accelBiasZ = 0.0f;

bool gyroCalibrated = false;
bool accelCalibrated = false;

// =====================================================
// BAROMETER CALIBRATION
// =====================================================

float pressureZeroPa = 0.0f;

bool altitudeCalibrated = false;

// =====================================================
// KALMAN FILTER STATE
// =====================================================

float kalmanAltitude = 0.0f;
float kalmanP = 1.0f;

bool kalmanInitialized = false;

// =====================================================
// SENSOR DATA STRUCTURE
// =====================================================

struct SensorData
{
    uint32_t timestamp;

    float ax;
    float ay;
    float az;

    float gx;
    float gy;
    float gz;

    float mpuTemp;
    float bmpTemp;

    float pressure;

    float rawAltitude;
    float filteredAltitude;
};

SensorData sensor = {};

struct RawIMU
{
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t temperature;

    int16_t gx;
    int16_t gy;
    int16_t gz;
};

// =====================================================
// I2C REGISTER WRITE
// =====================================================

bool imuWrite(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(IMU_ADDR);

    Wire.write(reg);
    Wire.write(value);

    return Wire.endTransmission(true) == 0;
}

// =====================================================
// I2C REGISTER READ
// =====================================================

bool imuReadBytes(
    uint8_t reg,
    uint8_t *buffer,
    uint8_t length
)
{
    Wire.beginTransmission(IMU_ADDR);

    Wire.write(reg);

    if (Wire.endTransmission(false) != 0)
    {
        return false;
    }

    uint8_t received = Wire.requestFrom(
        (uint8_t)IMU_ADDR,
        length,
        (uint8_t)true
    );

    if (received != length)
    {
        return false;
    }

    for (uint8_t i = 0; i < length; i++)
    {
        buffer[i] = Wire.read();
    }

    return true;
}

int16_t combineBytes(uint8_t highByte, uint8_t lowByte)
{
    uint16_t value =
        ((uint16_t)highByte << 8) | lowByte;

    return (int16_t)value;
}

bool readIMURaw(RawIMU &raw)
{
    uint8_t d[14];

    if (!imuReadBytes(0x3B, d, 14))
    {
        return false;
    }

    raw.ax = combineBytes(d[0], d[1]);
    raw.ay = combineBytes(d[2], d[3]);
    raw.az = combineBytes(d[4], d[5]);

    raw.temperature = combineBytes(d[6], d[7]);

    raw.gx = combineBytes(d[8], d[9]);
    raw.gy = combineBytes(d[10], d[11]);
    raw.gz = combineBytes(d[12], d[13]);

    return true;
}

// =====================================================
// INITIALIZE IMU
// =====================================================

bool initializeIMU()
{
    if (!imuReadBytes(0x75, &imuChipID, 1))
    {
        Serial.println("# ERROR: IMU not responding");
        return false;
    }

    Serial.print("# IMU WHO_AM_I: 0x");
    Serial.println(imuChipID, HEX);

    if (imuChipID != 0x68 &&
        imuChipID != 0x70 &&
        imuChipID != 0x72)
    {
        Serial.println("# ERROR: Unsupported IMU ID");
        return false;
    }

    if (imuChipID == 0x72)
    {
        Serial.println("# WARNING: Unknown IMU ID 0x72");
        Serial.println("# Using experimental register compatibility");
    }

    // Wake up IMU
    if (!imuWrite(0x6B, 0x00))
    {
        return false;
    }

    delay(100);

    // Accelerometer +/-16g
    if (!imuWrite(0x1C, 0x18))
    {
        return false;
    }

    // Gyroscope +/-2000 deg/s
    if (!imuWrite(0x1B, 0x18))
    {
        return false;
    }

    // Digital low-pass filter
    if (!imuWrite(0x1A, 0x04))
    {
        return false;
    }

    delay(100);

    Serial.println("# IMU initialized");

    return true;
}

// =====================================================
// INITIALIZE BMP280
// =====================================================

bool initializeBMP()
{
    bool found = bmp.begin(BMP_ADDR, 0x58);

    if (!found)
    {
        found = bmp.begin(0x77, 0x58);
    }

    if (!found)
    {
        Serial.println("# ERROR: BMP280 not detected");
        return false;
    }

    bmp.setSampling(
        Adafruit_BMP280::MODE_NORMAL,
        Adafruit_BMP280::SAMPLING_X2,
        Adafruit_BMP280::SAMPLING_X16,
        Adafruit_BMP280::FILTER_X16,
        Adafruit_BMP280::STANDBY_MS_1
    );

    delay(300);

    Serial.println("# BMP280 initialized");

    return true;
}

// =====================================================
// INITIALIZE LORA
// =====================================================

bool initializeLoRa()
{
    SPI.begin(
        LORA_SCK,
        LORA_MISO,
        LORA_MOSI,
        LORA_CS
    );

    LoRa.setPins(
        LORA_CS,
        LORA_RST,
        LORA_DIO0
    );

    if (!LoRa.begin(LORA_FREQUENCY))
    {
        Serial.println("# ERROR: LoRa initialization failed");
        return false;
    }

    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(125000);
    LoRa.setCodingRate4(5);

    LoRa.setPreambleLength(12);
    LoRa.setSyncWord(0x12);

    LoRa.enableCrc();

    LoRa.setTxPower(8);

    Serial.println("# LoRa initialized");
    Serial.println("# Frequency: 433.5 MHz");

    return true;
}

// =====================================================
// IMU STATIONARY CALIBRATION
// =====================================================

bool calibrateIMU()
{
    Serial.println();
    Serial.println("# Starting IMU calibration");
    Serial.println("# Keep the board completely stationary");
    Serial.println("# Place it flat on one face if possible");

    gyroBiasX = 0.0f;
    gyroBiasY = 0.0f;
    gyroBiasZ = 0.0f;

    accelBiasX = 0.0f;
    accelBiasY = 0.0f;
    accelBiasZ = 0.0f;

    gyroCalibrated = false;
    accelCalibrated = false;

    double sumAx = 0.0;
    double sumAy = 0.0;
    double sumAz = 0.0;

    double sumGx = 0.0;
    double sumGy = 0.0;
    double sumGz = 0.0;

    float minGx = INFINITY;
    float minGy = INFINITY;
    float minGz = INFINITY;

    float maxGx = -INFINITY;
    float maxGy = -INFINITY;
    float maxGz = -INFINITY;

    RawIMU raw;

    for (uint16_t i = 0; i < IMU_CAL_SAMPLES; i++)
    {
        if (!readIMURaw(raw))
        {
            Serial.println("# ERROR: IMU calibration read failed");
            return false;
        }

        sumAx += raw.ax;
        sumAy += raw.ay;
        sumAz += raw.az;

        sumGx += raw.gx;
        sumGy += raw.gy;
        sumGz += raw.gz;

        float gx = raw.gx / 16.4f;
        float gy = raw.gy / 16.4f;
        float gz = raw.gz / 16.4f;

        minGx = fminf(minGx, gx);
        minGy = fminf(minGy, gy);
        minGz = fminf(minGz, gz);

        maxGx = fmaxf(maxGx, gx);
        maxGy = fmaxf(maxGy, gy);
        maxGz = fmaxf(maxGz, gz);

        delay(IMU_CAL_DELAY_MS);
    }

    // ---------- Gyroscope bias ----------

    float candidateGx = (sumGx / IMU_CAL_SAMPLES) / 16.4f;
    float candidateGy = (sumGy / IMU_CAL_SAMPLES) / 16.4f;
    float candidateGz = (sumGz / IMU_CAL_SAMPLES) / 16.4f;

    bool stableGyro =
        (maxGx - minGx < 8.0f) &&
        (maxGy - minGy < 8.0f) &&
        (maxGz - minGz < 8.0f);

    if (stableGyro)
    {
        gyroBiasX = candidateGx;
        gyroBiasY = candidateGy;
        gyroBiasZ = candidateGz;

        gyroCalibrated = true;

        Serial.println("# Gyroscope bias calibrated");
    }
    else
    {
        Serial.println("# WARNING: Gyroscope motion detected");
        Serial.println("# Gyroscope calibration skipped");
    }

    // ---------- Accelerometer single-position offset ----------

    float avgAx = (sumAx / IMU_CAL_SAMPLES) * (GRAVITY / 2048.0f);
    float avgAy = (sumAy / IMU_CAL_SAMPLES) * (GRAVITY / 2048.0f);
    float avgAz = (sumAz / IMU_CAL_SAMPLES) * (GRAVITY / 2048.0f);

    float magnitude = sqrtf(
        avgAx * avgAx +
        avgAy * avgAy +
        avgAz * avgAz
    );

    float absX = fabsf(avgAx);
    float absY = fabsf(avgAy);
    float absZ = fabsf(avgAz);

    uint8_t gravityAxis = 0;
    float largest = absX;

    if (absY > largest)
    {
        largest = absY;
        gravityAxis = 1;
    }

    if (absZ > largest)
    {
        largest = absZ;
        gravityAxis = 2;
    }

    bool approximatelyFlat =
        largest > GRAVITY * 0.80f &&
        fabsf(magnitude - GRAVITY) < 2.0f;

    if (approximatelyFlat && stableGyro)
    {
        float expectedX = 0.0f;
        float expectedY = 0.0f;
        float expectedZ = 0.0f;

        if (gravityAxis == 0)
        {
            expectedX = avgAx >= 0 ? GRAVITY : -GRAVITY;
        }
        else if (gravityAxis == 1)
        {
            expectedY = avgAy >= 0 ? GRAVITY : -GRAVITY;
        }
        else
        {
            expectedZ = avgAz >= 0 ? GRAVITY : -GRAVITY;
        }

        accelBiasX = avgAx - expectedX;
        accelBiasY = avgAy - expectedY;
        accelBiasZ = avgAz - expectedZ;

        accelCalibrated = true;

        Serial.println("# Accelerometer single-position offset applied");
    }
    else
    {
        Serial.println("# WARNING: Accelerometer offset skipped");
        Serial.println("# Keep one sensor axis aligned with gravity");
    }

    Serial.printf(
        "# Gyro bias: %.3f, %.3f, %.3f deg/s\n",
        gyroBiasX, gyroBiasY, gyroBiasZ
    );

    Serial.printf(
        "# Accel bias: %.3f, %.3f, %.3f m/s^2\n",
        accelBiasX, accelBiasY, accelBiasZ
    );

    Serial.println("# IMU calibration finished");

    return gyroCalibrated;
}

// =====================================================
// ZERO ALTITUDE CALIBRATION
// =====================================================

bool calibrateAltitudeZero()
{
    Serial.println();
    Serial.println("# Starting altitude zero calibration");
    Serial.println("# Keep the BMP280 at reference height");

    altitudeCalibrated = false;

    double pressureSum = 0.0;

    for (uint16_t i = 0; i < BMP_CAL_SAMPLES; i++)
    {
        float pressure = bmp.readPressure();

        if (!isfinite(pressure) ||
            pressure < 30000.0f ||
            pressure > 120000.0f)
        {
            Serial.println("# ERROR: Invalid calibration pressure");
            return false;
        }

        pressureSum += pressure;

        delay(BMP_CAL_DELAY_MS);
    }

    pressureZeroPa = pressureSum / BMP_CAL_SAMPLES;

    altitudeCalibrated = true;

    Serial.printf("# Zero reference pressure: %.2f Pa\n", pressureZeroPa);
    Serial.println("# Current altitude reference = 0.00 m");

    kalmanAltitude = 0.0f;
    kalmanP = 1.0f;
    kalmanInitialized = true;
    lastKalmanTime = millis();

    Serial.println("# Altitude calibration complete");

    return true;
}

// =====================================================
// RELATIVE ALTITUDE
// =====================================================

float calculateRelativeAltitude(float pressurePa)
{
    if (!altitudeCalibrated ||
        pressureZeroPa <= 0.0f ||
        pressurePa <= 0.0f)
    {
        return NAN;
    }

    float ratio = pressurePa / pressureZeroPa;

    return 44330.0f *
        (1.0f - powf(ratio, 0.19029495f));
}

// =====================================================
// 1D KALMAN FILTER
// =====================================================

float updateAltitudeKalman(float measurement)
{
    uint32_t now = millis();

    if (!kalmanInitialized)
    {
        kalmanAltitude = measurement;
        kalmanP = 1.0f;

        kalmanInitialized = true;
        lastKalmanTime = now;

        return kalmanAltitude;
    }

    float dt = (uint32_t)(now - lastKalmanTime) / 1000.0f;

    lastKalmanTime = now;

    if (dt <= 0.0f)
    {
        dt = 0.001f;
    }

    kalmanP += KALMAN_Q * dt;

    float K = kalmanP / (kalmanP + KALMAN_R);

    kalmanAltitude += K * (measurement - kalmanAltitude);

    kalmanP *= (1.0f - K);

    return kalmanAltitude;
}

// =====================================================
// READ ALL SENSORS
// =====================================================

bool readSensors()
{
    RawIMU raw;

    if (!readIMURaw(raw))
    {
        return false;
    }

    sensor.timestamp = millis();

    // Accelerometer (+/-16g)
    float ax = (raw.ax / 2048.0f) * GRAVITY;
    float ay = (raw.ay / 2048.0f) * GRAVITY;
    float az = (raw.az / 2048.0f) * GRAVITY;

    sensor.ax = ax - accelBiasX;
    sensor.ay = ay - accelBiasY;
    sensor.az = az - accelBiasZ;

    // Gyroscope (+/-2000 deg/s)
    float gx = raw.gx / 16.4f;
    float gy = raw.gy / 16.4f;
    float gz = raw.gz / 16.4f;

    sensor.gx = gx - gyroBiasX;
    sensor.gy = gy - gyroBiasY;
    sensor.gz = gz - gyroBiasZ;

    // IMU temperature
    if (imuChipID == 0x70)
    {
        sensor.mpuTemp = raw.temperature / 333.87f + 21.0f;
    }
    else if (imuChipID == 0x68)
    {
        sensor.mpuTemp = raw.temperature / 340.0f + 36.53f;
    }
    else
    {
        sensor.mpuTemp = NAN;
    }

    // BMP280
    sensor.bmpTemp = bmp.readTemperature();
    sensor.pressure = bmp.readPressure();

    if (!isfinite(sensor.bmpTemp) ||
        !isfinite(sensor.pressure) ||
        sensor.pressure <= 0.0f)
    {
        return false;
    }

    float rawAltitude = calculateRelativeAltitude(sensor.pressure);

    if (!isfinite(rawAltitude))
    {
        return false;
    }

    sensor.rawAltitude = rawAltitude;

    sensor.filteredAltitude = updateAltitudeKalman(rawAltitude);

    return isfinite(sensor.filteredAltitude);
}

// =====================================================
// FLIGHT STATE MACHINE
// =====================================================

// Dipanggil sekali setiap siklus sensor (10 Hz)
// setelah data sensor valid.

void updateFlightState()
{
    // STARTUP -> ARMING ditangani di sendTelemetry()
    // (setelah paket telemetry pertama berhasil dikirim).
    if (flightState == STATE_STARTUP)
    {
        return;
    }

    float altitude = sensor.filteredAltitude;
    uint32_t t = sensor.timestamp;

    if (!stateRefValid)
    {
        prevStateAltitude = altitude;
        prevStateTime = t;
        maxAltitude = altitude;
        stateRefValid = true;
        return;
    }

    float dt = (uint32_t)(t - prevStateTime) / 1000.0f;

    if (dt <= 0.0f)
    {
        return;
    }

    float delta = altitude - prevStateAltitude;
    float velocity = delta / dt;

    prevStateAltitude = altitude;
    prevStateTime = t;

    if (altitude > maxAltitude)
    {
        maxAltitude = altitude;
    }

    switch (flightState)
    {
        // -------------------------------------------------
        case STATE_ARMING:
        // -------------------------------------------------
            // Naik pesat dan stabil
            if (velocity >= BOOST_VELOCITY_MPS &&
                altitude >= BOOST_MIN_ALTITUDE_M)
            {
                if (boostCount < 255) boostCount++;
            }
            else
            {
                boostCount = 0;
            }

            if (boostCount >= BOOST_CONFIRM_SAMPLES)
            {
                flightState = STATE_BOOST;
                descentCount = 0;
                maxAltitude = altitude;

                Serial.println("# STATE -> 2 BOOST");
            }
            break;

        // -------------------------------------------------
        case STATE_BOOST:
        // -------------------------------------------------
            // 25 penurunan berturut-turut
            if (delta < -ALTITUDE_DEADBAND_M)
            {
                if (descentCount < 255) descentCount++;
            }
            else if (delta > ALTITUDE_DEADBAND_M)
            {
                descentCount = 0;
            }
            // di dalam deadband: hitungan dipertahankan

            if (descentCount >= APOGEE_DESCENT_SAMPLES)
            {
                flightState = STATE_APOGEE;
                stableCount = 0;

                // Sinyal apogee: GPIO26 (test LED) HIGH selama 3 detik
                digitalWrite(MOSFET_SIGNAL, HIGH);
                apogeeLedActive = true;
                apogeeLedStartTime = millis();

                Serial.println("# STATE -> 3 APOGEE");
                Serial.println("# GPIO26 MOSFET_SIGNAL = HIGH (3 s)");
            }
            break;

        case STATE_APOGEE:
            // Data ketinggian relatif stabil, dan sudah
            // turun cukup jauh dari puncak.
            if (fabsf(delta) < LANDING_STABLE_BAND_M &&
                (maxAltitude - altitude) >= LANDING_MIN_DROP_M)
            {
                if (stableCount < 65535) stableCount++;
            }
            else
            {
                stableCount = 0;
            }

            if (stableCount >= LANDING_STABLE_SAMPLES)
            {
                flightState = STATE_LANDING;

                Serial.println("# STATE -> 4 LANDING");
            }
            break;

        default:
            break;
    }
}

// =====================================================
// CREATE CSV TELEMETRY (11 kolom)
// =====================================================

bool createCSV(char *buffer, size_t bufferSize)
{
    char imuTempText[24];

    if (isfinite(sensor.mpuTemp))
    {
        snprintf(imuTempText, sizeof(imuTempText), "%.2f", sensor.mpuTemp);
    }
    else
    {
        snprintf(imuTempText, sizeof(imuTempText), "NA");
    }

    int length = snprintf(
        buffer,
        bufferSize,

        "%lu;"
        "%.2f;%.2f;%.2f;"
        "%.2f;%.2f;%.2f;"
        "%s;%.2f;"
        "%.0f;%.2f",

        (unsigned long)sensor.timestamp,

        sensor.ax,
        sensor.ay,
        sensor.az,

        sensor.gx,
        sensor.gy,
        sensor.gz,

        imuTempText,
        sensor.bmpTemp,

        sensor.pressure,
        sensor.filteredAltitude
    );

    return length > 0 &&
           (size_t)length < bufferSize &&
           length <= 250;
}

// =====================================================
// SEND LORA TELEMETRY
// =====================================================

// Format paket:
//   <state>\n
//   <csv 11 kolom>

void sendTelemetry()
{
    if (!sensorDataValid)
    {
        return;
    }

    char csvPacket[180];

    if (!createCSV(csvPacket, sizeof(csvPacket)))
    {
        Serial.println("# ERROR: CSV formatting failed");
        return;
    }

    if (LoRa.beginPacket() == 0)
    {
        Serial.println("# LoRa busy, packet skipped");
        return;
    }

    // Satu baris: state;data sensor (pemisah ";")
    LoRa.print((int)flightState);
    LoRa.print(';');
    LoRa.print(csvPacket);

    int result = LoRa.endPacket(true);

    if (result == 1)
    {
        // Paket masuk antrean TX (bukan ACK dari receiver).
        Serial.print((int)flightState);
        Serial.print(';');
        Serial.println(csvPacket);

        // Telemetry sudah berjalan -> ARMING
        if (flightState == STATE_STARTUP)
        {
            flightState = STATE_ARMING;
            Serial.println("# STATE -> 1 ARMING");
        }
    }
    else
    {
        Serial.println("# ERROR: LoRa transmission start failed");
    }
}

// =====================================================
// APOGEE LED TIMER
// =====================================================

// Mematikan GPIO26 setelah APOGEE_MOSFET_DURATION_MS.

void updateApogeeLed()
{
    if (apogeeLedActive &&
        (uint32_t)(millis() - apogeeLedStartTime) >=
            APOGEE_MOSFET_DURATION_MS)
    {
        digitalWrite(MOSFET_SIGNAL, LOW);
        apogeeLedActive = false;

        Serial.println("# GPIO26 MOSFET_SIGNAL = LOW");
    }
}

void printCSVHeader()
{
    Serial.println(
        "State;"
        "Timestamp_ms;"
        "Accel_X;Accel_Y;Accel_Z;"
        "Gyro_X;Gyro_Y;Gyro_Z;"
        "MPU_Temp_C;"
        "BMP_Temp_C;"
        "Pressure_Pa;"
        "Altitude_m"
    );
}

// =====================================================
// SETUP
// =====================================================

void setup()
{
    Serial.begin(115200);
    delay(1000);

    // GPIO26 dipastikan LOW sebelum jadi output
    digitalWrite(MOSFET_SIGNAL, LOW);
    pinMode(MOSFET_SIGNAL, OUTPUT);

    flightState = STATE_STARTUP;

    Serial.println();
    Serial.println("==================================");
    Serial.println(" AVIONICS LORA SENSOR TRANSMITTER");
    Serial.println("==================================");
    Serial.println("# STATE 0 STARTUP");

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000);
    Wire.setTimeOut(100);

    if (!initializeIMU())
    {
        Serial.println("# FATAL: IMU initialization failed");
        while (true) { delay(1000); }
    }

    if (!initializeBMP())
    {
        Serial.println("# FATAL: BMP280 initialization failed");
        while (true) { delay(1000); }
    }

    if (!initializeLoRa())
    {
        Serial.println("# FATAL: LoRa initialization failed");
        while (true) { delay(1000); }
    }

    Serial.println();
    Serial.println("# CALIBRATION STARTING");
    Serial.println("# Do not move the sensor board");

    if (!calibrateIMU())
    {
        Serial.println("# WARNING: Gyroscope not calibrated");
        Serial.println("# IMU readings may contain bias");
    }

    if (!calibrateAltitudeZero())
    {
        Serial.println("# FATAL: Altitude zero calibration failed");
        while (true) { delay(1000); }
    }

    Serial.println();
    Serial.println("# SYSTEM READY");
    Serial.println("# Sensor polling: 10 Hz");
    Serial.println("# LoRa telemetry target: 5 Hz");
    Serial.println("# Packet: state;data sensor (satu baris, pemisah ;)");
    Serial.println("# States: 0 STARTUP, 1 ARMING, 2 BOOST, 3 APOGEE, 4 LANDING");
    Serial.println("# GPIO26 HIGH for 3 s at APOGEE");
    Serial.println();

    printCSVHeader();

    lastSensorTime = millis();
    lastTelemetryTime = millis();
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop()
{
    uint32_t now = millis();

    updateApogeeLed();

    // Sensor update - 10 Hz
    if ((uint32_t)(now - lastSensorTime) >= SENSOR_INTERVAL_MS)
    {
        lastSensorTime = now;

        sensorDataValid = readSensors();

        if (sensorDataValid)
        {
            updateFlightState();
        }
        else
        {
            Serial.println("# ERROR: Sensor read failed");
        }
    }

    // LoRa telemetry - target 5 Hz
    if ((uint32_t)(now - lastTelemetryTime) >= TELEMETRY_INTERVAL_MS)
    {
        lastTelemetryTime = now;

        sendTelemetry();
    }
}