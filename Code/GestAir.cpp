#include "Imu.h"
#include "Controls.h"
//#include "Drone.h"
#include "Display.h"
#include "Debug.h"

ControlMode mode = ControlMode.None;
ControlInput gloveInputs[4] { {A0},{A1},{A2},{A3} };

ControlState gloveMin { 3, 3, 3, 3 }; // Minimum absolute attitude recognized
ControlState gloveMax { 45, 90, 45, 35 }; // Maximum absolute attitude recognized
ControlState gloveScale { 1, 1, 1, 1 }; // Percentage of attitude to drone input [0,1]
ControlState droneInput; // Values sent to drone [-1,1]

// Timestamp of last relative attitude used for drone input in milliseconds
uint32_t lastDroneInputMs = 0;

ImuDevice imuHand(0), imuWrist(1);
ImuAttitude attitudeHand, attitudeWrist, attitudeRelative;

// Cooldown in milliseconds for reinitializations over I2C
constexpr uint32_t I2C_RETRY_INTERVAL_MS = 2000;
// Last initialization tiem in milliseconds over I2C
uint32_t lastI2CRetryMs = 0;

constexpr uint8_t LED_STATUS = 13;
bool heartBeat = false;

LiquidCrystal lcd(7, 6, 5, 4, 3, 2);

// Current time
uint32_t timeMs = 0; // milliseconds
uint32_t timeUs = 0; // microseconds

void setUp() {
	initalizeDebug();

	pinMode(LED_STATUS, OUTPUT);
  digitalWrite(LED_STATUS, LOW);
	initalizeDisplay();

  initalizeI2CBus();
	initializeImuWithRetry(imuHand);
	initializeImuWithRetry(imuWrist);

	timeMs = millis();
	timeUs = micros();
	lastI2CRetryMs = timeMs;

	debugImuStatuses(imuHand, imuWrist);
}

void handleImus() {
	updateImuStatus(imuHand, timeMs);
	updateImuStatus(imuWrist, timeMs);

	if ((!imuHand.ok || !imuWrist.ok) && (timeMs - lastI2CRetry >= I2C_RETRY_INTERVAL_MS)) {
		debugImuStatuses(imuHand, imuWrist);
	  if (!imuHand.ok) initializeImu(imuHand);
	  if (!imuWrist.ok) initializeImu(imuWrist);
	  lastI2CRetryMs = timeMs;
	  debugImuStatuses(imuHand, imuWrist);
	}

	const uint32_t previousTimeUs = timeUs;
	timeUs = micros();
	const float deltaTimeS = (timeUs - previousTimeUs) / 1e6f;

	const bool handUpdated = readImuAttitude(imuHand, attitudeHand, deltaTimeS);
	const bool wristUpdated = readImuAttitude(imuWrist, attitudeWrist, deltaTimeS);

	if ((handUpdated && isFreshImuAttitude(attitudeWrist, timeMs))
		|| (wristUpdated && isFreshImuAttitude(attitudeHand, timeMs))) {
		calculateRelativeImuAttitude(attitudeHand, attitudeWrist, attitudeRelative);
		debugImuAttitudes(attitudeHand, attitudeWrist, attitudeRelative);
	}
}

float mapDroneInput(float val, float min, float max, float scale)
{
  const float absVal = fabs(val);

  if (absVal <= min) return 0.0f;

  float scaledVal = (absVal - min) / (max - min);
  scaledVal = constrain(scaledVal, 0.0f, 1.0f);
  scaledVal *= scale;

  return (val > 0 ? scaledVal : -scaledVal);
}

void mapDroneInputs() {
	if (lastDroneInputMs == attitudeRelative.lastUpdateMs) return;

	droneInput.roll = 
		mapDroneInput(attitudeRelative.roll, gloveMin.roll, gloveMax.roll, gloveScale.roll);
	droneInput.pitch =
		mapDroneInput(attitudeRelative.pitch, gloveMin.pitch, gloveMax.pitch, gloveScale.pitch);
	droneInput.yaw =
		mapDroneInput(attitudeRelative.pitch, gloveMin.pitch, gloveMax.pitch, gloveScale.pitch);
	droneInput.throttle = 
		mapDroneInput(
			attitudeRelative.pitch, 
			gloveMin.throttle, 
			gloveMax.throttle, 
			gloveScale.throttle);

	lastDroneInputMs = attitudeRelative.lastUpdateMs;
}

void loop() {
	heartBeat = !heartBeat;
	digitalWrite(LED_STATUS, heartBeat);

	timeMs = millis();

	handleImus();
	mapDroneInputs();

	readControlInputs(gloveInputs);

	switch (mode) {
		case ControlMode.None:
			if (gloveInputs[0].isActive && gloveInputs[3].isActive) {
				mode = ControlMode.Drone;
				// droneArm();
			}
			break;
		case ControlMode.Drone:
			if (gloveInputs[0].isActive) {
				// droneRotate(droneInput.roll, droneInput.pitch, droneInput.yaw);
			}
			else if (gloveInputs[1].isActive) {
				// droneLift(droneInput.throttle);
			}
			else if (gloveInputs[2].isActive) {
				// droneCircle();
			}
			else if (isControlInputRisingEdge(gloveInputs[3])) {
				mode = ControlMode.Menu;
			}
			break;
		case ControlMode.Menu:
			if (isControlInputRisingEdge(gloveInputs[0])) {
				// menuAccept();
			}
			else if (isControlInputRisingEdge(gloveInputs[1])) {
				// menuUp();
			}
			else if (isControlInputRisingEdge(gloveInputs[2])) {
				// menuDown();
			}
			else if (isControlInputRisingEdge(gloveInputs[3])) {
				mode = ControlMode.Drone;
			}
			break;
	}
}
