#include "Debug.h"
#include <Arduino.h>
#include "Imu.h"

constexpr uint32_t BAUD_RATE = 115200;

void initalizeDebug() {
	Serial.begin(BAUD_RATE);
	Serial.println(F("GestAir"));
	Serial.println(F("Booting..."));
}

void printOkStatus(bool ok) {
	if (ok)
		Serial.println(F("OK"));
	else
		Serial.println(F("FAIL"));
}

void debugImuStatuses(const ImuDevice& hand, const ImuDevice& device wrist) {
	Serial.println(F("=====Imu Statuses====="));
	Serial.print(F("Hand: "));
	printOkStatus(hand.ok);
	Serial.print(F("Wrist: "));
	printOkStatus(wrist.ok);
}

void printAttitude(const ImuAttitude& attitude) {
	Serial.print(F("Roll: "));
	Serial.print(attitude.roll);
	Serial.print(F(", Pitch: "));
	Serial.print(attitude.pitch);
	Serial.print(F(", Yaw: "));
	Serial.print(attitude.yaw);
	Serial.print(F(", Update: "));
	Serial.println(attitude.lastUpdateMs);
}

void debugImuAttitudes(const ImuAttitude& hand, const ImuAttitude& wrist, const ImuAttitude& relative) {
	Serial.println(F("=====Imu Attitudes====="));
	Serial.print(F("Hand: "));
	printAttitude(hand);
	Serial.print(F("Wrist: "));
	printAttitude(wrist);
	Serial.print(F("Relative: "));
	printAttitude(relative);
}
