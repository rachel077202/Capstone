#include "Imu.h"
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// I2C clock rate
constexpr uint32_t BUS_SPEED = 400000; // 400 kHz

// Maximum number of attempts for an IMU initialization
constexpr uint8_t RETRY_ATTEMPTS = 5;
// Delay in milliseconds to wait after a failed IMU initialization
constexpr uint32_t RETRY_DELAY_MS = 50;
// Time in milliseconds in which an unupdated imu is deemed no longer good to use
constexpr uint32_t DEVICE_TIMEOUT_MS = 50;

// Complementary filter blend ratio
constexpr float ALPHA = 0.98f;
// Time in milliseconds in which a fresh attitude is deemed stale
constexpr uint32_t ATTITUDE_TIMEOUT_MS = 50;

/**
 * Initializes the I2C bus that the IMUs transmit data over.
 */
void initalizeI2CBus() {
	Wire.begin();
  Wire.setClock(BUS_SPEED);
}

/**
 * Attempts to initialize the given IMU on the I2C bus with its address bit,
 * and updates its ok status based on whether the the initialization was successful.
 * Returns whether the initialization was successful.
 */
bool initializeImu(ImuDevice& device) {
	ICM_20948_I2C& imu = device.imu;

	imu.begin(Wire, device.AD0);

	device.ok = (imu.status == ICM_20948_Stat_Ok);
	if (!device.ok) return false;

	ICM_20948_fss_t fss;
	fss.a = gpm4;     // accel ±4 g
	fss.g = dps500;   // gyro  ±500 dps

	imu.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), fss);
	imu.setSampleMode(ICM_20948_Internal_Acc, ICM_20948_Sample_Mode_Continuous);
	imu.setSampleMode(ICM_20948_Internal_Gyr,  ICM_20948_Sample_Mode_Continuous);

	device.lastUpdateMs = millis();
	return true;
}

/**
 * Attempts to initialize the given IMU until successful
 * or the maximum number of retry attempts is reached, delaying between each failed attempt.
 * Returns whether the initialization was successful.
 */
bool initializeImuWithRetry(ImuDevice& device) {
  for (uint8_t i = 0; i < RETRY_ATTEMPTS; ++i) {
    if (initializeImu(device)) {
      return true;
    }
    delay(RETRY_DELAY_MS);
  }
  return false;
}

/**
 * Updates the ok status of the given IMU based on whether its last successful use was recent
 * to the given reference time in milliseconds, according to the set timeout.
 */
void updateImuStatus(ImuDevice& device, uint32_t timeMs) {
	device.ok = (timeMs - device.lastUpdateMs < DEVICE_TIMEOUT_MS);
}

/**
 * Calculates roll, pitch, and yaw based on the gyroscope, accelerometer, and magnetometer data
 * produced by the given IMU, applying a complementary blend filter with the
 * given time change in seconds, and stores the values in the given attitude with a timestamp.
 * Assumes that the given IMU is good to use and its AGM data is fresh.
 */
void calculateImuAttitude(ICM_20948_I2C& imu, ImuAttitude& attitude, float deltaTimeS) {
	if (deltaTimeS <= 0.0f || deltaTimeS > 0.1f) deltaTimeS = 0.005f;

	// Gryoscope (deg/s)
  const float gx = imu.gyrX();
  const float gy = imu.gyrY();
  const float gz = imu.gyrZ();

  // Accelerometer (g)
  const float ax = imu.accX() / 1000.0f;
  const float ay = imu.accY() / 1000.0f;
  const float az = imu.accZ() / 1000.0f;

  // Magnetometer
  const float mx = imu.magX();
  const float my = imu.magY();
  const float mz = imu.magZ();

  // Integrate gyro
  const float rollGyro = attitude.roll + gx * deltaTimeS;
  const float pitchGyro = attitude.pitch + gy * deltaTimeS;
  const float yawGyro = attitude.yaw + gz * deltaTimeS;

  // Integrate acceleration
  const float rollAcc = atan2f(ay, az) * RAD_TO_DEG;
  const float pitchAcc = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;

  attitude.roll = ALPHA * rollGyro + (1.0f - ALPHA) * rollAcc;
  attitude.pitch = ALPHA * pitchGyro + (1.0f - ALPHA) * pitchAcc;

  // Integrate magnetometer
  const float rollRad  = attitude.roll  * DEG_TO_RAD;
  const float pitchRad = attitude.pitch * DEG_TO_RAD;
  const float mxComponent = mx * cosf(pitchRad) + mz * sinf(pitchRad);
  const float myComponent =
      mx * sinf(rollRad) * sinf(pitchRad) +
      my * cosf(rollRad) -
      mz * sinf(rollRad) * cosf(pitchRad);
  const float yawMag = atan2f(-myComponent, mxComponent) * RAD_TO_DEG;

  attitude.yaw = ALPHA * yawGyro + (1.0f - ALPHA) * yawMag;

  attitude.lastUpdateMs = millis();
}

/**
 * Attempts to update the given attitude based on the data produced by the given IMU
 * and the given time change in seconds. If successful, the timestamp of the given IMU is updated.
 * Returns whether the read was successful.
 */
bool readImuAttitude(ImuDevice& device, ImuAttitude& attitude, float deltaTimeS) {
  if (!device.ok) return false;

  ICM_20948_I2C& imu = device.imu;

  if (!imu.dataReady()) return false;
  
  imu.getAGMT();

	if (imu.status != ICM_20948_Stat_Ok) return false;

	calculateImuAttitude(imu, attitude, deltaTimeS);
	device.lastUpdateMs = attitude.lastUpdateMs;
  return true;
}

/**
 * Stores the difference between the first two given attitudes
 * in the third given attitude with a timestamp.
 */
void calculateRelativeImuAttitude(
	const ImuAttitude& attitude1,
	const ImuAttitude& attitude2,
	ImuAttitude& relative) {
	relative.roll = attitude1.roll - attitude2.roll;
	relative.pitch = attitude1.pitch - attitude2.pitch;
	relative.yaw = attitude1.yaw - attitude2.yaw;
	relative.lastUpdateMs = millis();
}

/**
 * Returns whether the last update of the given attitude was recent and not stale 
 * to the given reference time in milliseconds, according to the set timeout.
 */
bool isFreshImuAttitude(const ImuAttitude& attitude, uint32_t timeMs) {
	return timeMs - attitude.lastUpdateMs < ATTITUDE_TIMEOUT_MS;
}
