#ifndef CONTROLS_H
#define CONTROLS_H

// The item to control with inputs
enum ControlMode { None, Drone, Menu };

/**
 * An binary input with a pin number and its current and previous states,
 * where false and true represent low and high voltage respectively.
 */
struct ControlInput {
	const uint8_t PIN;
	bool isActive = false;
	bool wasActive = false;

	explicit ControlInput(uint8_t pin) : PIN(pin) {}
};

/**
 * Control outputs for the drone.
 */
struct ControlState {
	float throttle = 0.0f;
	float roll = 0.0f;
	float pitch = 0.0f;
	float yaw = 0.0f;
}

/**
 * Reads the state of the given input as low or high voltage, storing the previous state.
 */
void readControlInput(ControlInput& input);

/**
 * Reads the states of the given inputs as low or high voltage, storing their previous states.
 */
void readControlInputs(ControlInput[]& inputs);

/**
 * Returns whether the given input is currently active but was not active immediately before.
 */
bool isControlInputRisingEdge(const ControlInput& input);

/**
 * Sets the given state with the given values.
 */
void setControlState(ControlState& input, float throttle, float roll, float pitch, float yaw);

#endif
