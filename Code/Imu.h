#ifndef IMU_H
#define IMU_H

#include "ICM_20948.h"

/**
 * An IMU reference with its address on the I2C bus, whether its is good to use,
 * and the last time it was successfully used in milliseconds.
 */
struct ImuDevice {
	const uint8_t AD0; 		 // 0 or 1 (library uses 0x68 | ad0 for I2C address)
	ICM_20948_I2C imu;
	bool ok = false;
	uint32_t lastUpdateMs = 0;

	explicit ImuDevice(uint8_t ad0) : AD0(ad0) {}
};

/**
 * A spatial orientation with the last time it was updated in milliseconds.
 */
struct ImuAttitude {
	float roll = 0.0f;
	float pitch = 0.0f;
	float yaw = 0.0f;
	uint32_t lastUpdateMs = 0;
};

/**
 * Initializes the I2C bus that the IMUs transmit data over.
 */
void initalizeI2CBus();

/**
 * Attempts to initialize the given IMU on the I2C bus with its address bit,
 * and updates its ok status based on whether the the initialization was successful.
 * Returns whether the initialization was successful.
 */
bool initializeImu(ImuDevice& device);

/**
 * Attempts to initialize the given IMU until successful
 * or the maximum number of retry attempts is reached, delaying between each failed attempt.
 * Returns whether the initialization was successful.
 */
bool initializeImuWithRetry(ImuDevice& device);

/**
 * Updates the ok status of the given IMU based on whether its last successful use was recent
 * to the given reference time in milliseconds, according to the set timeout.
 */
void updateImuStatus(ImuDevice& device, uint32_t timeMs);

/**
 * Attempts to update the given attitude based on the data produced by the given IMU
 * and the given time change in seconds. If successful, the timestamp of the given IMU is updated.
 * Returns whether the read was successful.
 */
bool readImuAttitude(ImuDevice& device, ImuAttitude& attitude, float deltaTimeS);

/**
 * Stores the difference between the first two given attitudes
 * in the third given attitude with a timestamp.
 */
void calculateRelativeImuAttitude(
	const ImuAttitude& attitude1,
	const ImuAttitude& attitude2,
	ImuAttitude& relative);

/**
 * Returns whether the last update of the given attitude was recent and not stale 
 * to the given reference time in milliseconds, according to the set timeout.
 */
bool isFreshImuAttitude(const ImuAttitude& attitude, uint32_t timeMs);

#endif
