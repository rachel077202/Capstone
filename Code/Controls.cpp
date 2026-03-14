#include "Controls.h"
#include <Arduino.h>

// Minimum voltage needed for an input to be deemed high instead of low
constexpr uint32_t INPUT_THRESHOLD = 350;

/**
 * Reads the state of the given input as low or high voltage, storing the previous state.
 */
void readControlInput(ControlInput& input) {
	input.wasActive = input.isActive;
	input.isActive = (analogRead(input.PIN) >= INPUT_THRESHOLD);
}

/**
 * Reads the states of the given inputs as low or high voltage, storing their previous states.
 */
void readControlInputs(ControlInput[]& inputs) {
	for (uint8_t idx = 0; idx < inputs.length; ++idx) {
		readControlInput(inputs[idx]);
	}
}

/**
 * Returns whether the given input is currently active but was not active immediately before.
 */
bool isControlInputRisingEdge(const ControlInput& input) {
	return input.isActive && !input.wasActive;
}

/**
 * Sets the given state with the given values.
 */
void setControlState(ControlState& input, float throttle, float roll, float pitch, float yaw) {
	input.throttle = throttle;
	input.roll = roll;
	input.pitch = pitch;
	input.yaw = yaw;
}
